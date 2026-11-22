#include "lcd_st7789.h"

#include "spi.h"

#include <string.h>

/* 单段像素数：67200 分两段，且小于 HAL_SPI_Transmit_DMA 的 uint16_t 上限 */
#define LCD_SEND_CHUNK      33600U

/* ========== 模块私有状态 ========== */
static uint16_t s_XOffset = 20;     /* 控制器坐标偏移（横屏时列方向偏移 20） */
static uint16_t s_YOffset = 0;      /* 控制器坐标偏移（行方向） */
static uint16_t s_Width   = 280;    /* 当前方向逻辑宽 */
static uint16_t s_Height  = 240;    /* 当前方向逻辑高 */

/* ========== 显示缓冲与 DMA 发送状态 ========== */
/* 裁切后的 280x240 RGB565，放 .dma_buffer（AXI）且 32 字节对齐，供 SPI DMA 取数 */
__attribute__((section(".dma_buffer"), aligned(32)))
static uint16_t s_ShowBuf[LCD_SHOW_PIXELS];

static volatile bool s_FlushBusy = false;   /* 发送进行中标志 */
static uint32_t s_SendOffset = 0U;          /* 已发送像素数 */

/* ========== 寄存器级 SPI 填充（移植自反客，仅用于清屏/整块填充） ==========
 * 摄像头整帧走下方 DMA 路径；此处以纯轮询 TXP 写入 TXDR，供 LCD_Clear 使用。
 * 填充前把 DataSize 切到 16BIT（HAL_SPI_Init 重配），发完切回 8BIT，
 * 命令/参数路径仍走 8BIT 的 HAL_SPI_Transmit。 */

/* 关闭传输并清标志（对应 HAL 内部 SPI_CloseTransfer 的发送分支） */
static void Lcd_SpiCloseTransfer(SPI_HandleTypeDef *hspi)
{
    uint32_t itflag = hspi->Instance->SR;

    __HAL_SPI_CLEAR_EOTFLAG(hspi);
    __HAL_SPI_CLEAR_TXTFFLAG(hspi);

    __HAL_SPI_DISABLE(hspi);
    __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_EOT | SPI_IT_TXP | SPI_IT_RXP | SPI_IT_DXP |
                                SPI_IT_UDR | SPI_IT_OVR | SPI_IT_FRE | SPI_IT_MODF));
    CLEAR_BIT(hspi->Instance->CFG1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);

    if (hspi->State != HAL_SPI_STATE_BUSY_RX)
    {
        if ((itflag & SPI_FLAG_UDR) != 0UL)
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_UDR);
            __HAL_SPI_CLEAR_UDRFLAG(hspi);
        }
    }
    if (hspi->State != HAL_SPI_STATE_BUSY_TX)
    {
        if ((itflag & SPI_FLAG_OVR) != 0UL)
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_OVR);
            __HAL_SPI_CLEAR_OVRFLAG(hspi);
        }
    }
    if ((itflag & SPI_FLAG_MODF) != 0UL)
    {
        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_MODF);
        __HAL_SPI_CLEAR_MODFFLAG(hspi);
    }
    if ((itflag & SPI_FLAG_FRE) != 0UL)
    {
        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FRE);
        __HAL_SPI_CLEAR_FREFLAG(hspi);
    }

    hspi->TxXferCount = (uint16_t)0UL;
    hspi->RxXferCount = (uint16_t)0UL;
}

/* 等待标志（带超时） */
static HAL_StatusTypeDef Lcd_SpiWaitFlag(SPI_HandleTypeDef *hspi, uint32_t flag,
                                         FlagStatus status, uint32_t tickstart, uint32_t timeout)
{
    while ((__HAL_SPI_GET_FLAG(hspi, flag) ? SET : RESET) == status)
    {
        if (((HAL_GetTick() - tickstart) >= timeout) || (timeout == 0U))
        {
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

/* 发送收尾：等 TXC → 请求挂起 → 等待挂起完成 → 关闭 → 清挂起标志 */
static void Lcd_SpiFinishTransfer(SPI_HandleTypeDef *hspi, uint32_t tickstart, uint32_t timeout)
{
    Lcd_SpiWaitFlag(hspi, SPI_FLAG_TXC, RESET, tickstart, timeout);

    SET_BIT(hspi->Instance->CR1, SPI_CR1_CSUSP);
    Lcd_SpiWaitFlag(hspi, SPI_FLAG_SUSP, RESET, tickstart, timeout);
    Lcd_SpiCloseTransfer(hspi);
    SET_BIT(hspi->Instance->IFCR, SPI_IFCR_SUSPC);
}

/* 启动发送：TSIZE=0 不限制长度，半双工时方向切发送 */
static void Lcd_SpiStartTransfer(SPI_HandleTypeDef *hspi)
{
    if (hspi->Init.Direction == SPI_DIRECTION_1LINE)
    {
        SPI_1LINE_TX(hspi);
    }
    MODIFY_REG(hspi->Instance->CR2, SPI_CR2_TSIZE, 0);
    __HAL_SPI_ENABLE(hspi);
    if (hspi->Init.Mode == SPI_MODE_MASTER)
    {
        SET_BIT(hspi->Instance->CR1, SPI_CR1_CSTART);
    }
}

/* 单色填充发送：count 个 16 位像素全部为 color（清屏/填充/画点用） */
static HAL_StatusTypeDef Lcd_SpiTransmitFill(SPI_HandleTypeDef *hspi, uint16_t color, uint32_t count)
{
    uint32_t tickstart;
    const uint32_t timeout = 1000U;
    const uint32_t color32 = ((uint32_t)color << 16) | color;   /* 32 位快路径：两像素一批 */
    uint32_t remain = count;
    HAL_StatusTypeDef errorcode = HAL_OK;

    __HAL_LOCK(hspi);

    if (hspi->State != HAL_SPI_STATE_READY)
    {
        __HAL_UNLOCK(hspi);
        return HAL_BUSY;
    }
    if (count == 0U)
    {
        __HAL_UNLOCK(hspi);
        return HAL_ERROR;
    }

    /* 切换 16 位数据宽度 */
    hspi->Init.DataSize = SPI_DATASIZE_16BIT;
    if (HAL_SPI_Init(hspi) != HAL_OK)
    {
        hspi->Init.DataSize = SPI_DATASIZE_8BIT;
        HAL_SPI_Init(hspi);
        __HAL_UNLOCK(hspi);
        return HAL_ERROR;
    }

    tickstart = HAL_GetTick();
    hspi->State = HAL_SPI_STATE_BUSY_TX;
    hspi->ErrorCode = HAL_SPI_ERROR_NONE;
    hspi->pRxBuffPtr = NULL;
    hspi->TxISR = NULL;
    hspi->RxISR = NULL;

    Lcd_SpiStartTransfer(hspi);

    while (remain > 0U)
    {
        if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXP))
        {
            if ((remain > 1U) && (hspi->Init.FifoThreshold > SPI_FIFO_THRESHOLD_01DATA))
            {
                *(__IO uint32_t *)&hspi->Instance->TXDR = color32;
                remain -= 2U;
            }
            else
            {
                *(__IO uint16_t *)&hspi->Instance->TXDR = color;
                remain--;
            }
        }
        else if (((HAL_GetTick() - tickstart) >= timeout) || (timeout == 0U))
        {
            Lcd_SpiCloseTransfer(hspi);
            hspi->Init.DataSize = SPI_DATASIZE_8BIT;
            HAL_SPI_Init(hspi);
            __HAL_UNLOCK(hspi);
            SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_TIMEOUT);
            hspi->State = HAL_SPI_STATE_READY;
            return HAL_ERROR;
        }
    }

    Lcd_SpiFinishTransfer(hspi, tickstart, timeout);
    if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
    {
        errorcode = HAL_ERROR;
    }

    /* 恢复 8 位数据宽度（命令/参数按 8 位传输） */
    hspi->Init.DataSize = SPI_DATASIZE_8BIT;
    HAL_SPI_Init(hspi);

    __HAL_UNLOCK(hspi);
    hspi->State = HAL_SPI_STATE_READY;
    return errorcode;
}

/* ========== 命令/数据底层写入（软件 CS：整条命令期间 CS 保持低） ========== */

static void Lcd_WriteCmd(uint8_t cmd)
{
    HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi4, &cmd, 1U, 100U);
    HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);
}

static void Lcd_WriteData8(uint8_t val)
{
    HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET);
    HAL_SPI_Transmit(&hspi4, &val, 1U, 100U);
    HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);
}

/* 16 位参数：高字节在前（拆成两个 8 位发送，与 ST7789 时序一致） */
static void Lcd_WriteData16(uint16_t val)
{
    Lcd_WriteData8((uint8_t)(val >> 8));
    Lcd_WriteData8((uint8_t)(val & 0xFFU));
}

/* 设置显存写入窗口并发 RAMWR(0x2C)，坐标叠加当前方向偏移 */
static void Lcd_SetAddress(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    Lcd_WriteCmd(0x2A);
    Lcd_WriteData16(x1 + s_XOffset);
    Lcd_WriteData16(x2 + s_XOffset);

    Lcd_WriteCmd(0x2B);
    Lcd_WriteData16(y1 + s_YOffset);
    Lcd_WriteData16(y2 + s_YOffset);

    Lcd_WriteCmd(0x2C);
}

/* ========== 公开接口 ========== */

void LCD_BacklightOn(void)
{
    HAL_GPIO_WritePin(LCD_BL_PORT, LCD_BL_PIN, GPIO_PIN_SET);
}

void LCD_BacklightOff(void)
{
    HAL_GPIO_WritePin(LCD_BL_PORT, LCD_BL_PIN, GPIO_PIN_RESET);
}

void LCD_SetDirection(LcdDirection_t dir)
{
    switch (dir)
    {
    case LCD_DIR_H:      /* 横屏 280x240 */
        Lcd_WriteCmd(0x36); Lcd_WriteData8(0x70);
        s_XOffset = 20; s_YOffset = 0;
        s_Width = LCD_PANEL_HEIGHT; s_Height = LCD_PANEL_WIDTH;
        break;
    case LCD_DIR_V:      /* 竖屏 240x280（面板原生） */
        Lcd_WriteCmd(0x36); Lcd_WriteData8(0x00);
        s_XOffset = 0; s_YOffset = 20;
        s_Width = LCD_PANEL_WIDTH; s_Height = LCD_PANEL_HEIGHT;
        break;
    case LCD_DIR_H_FLIP: /* 横屏 180 度 */
        Lcd_WriteCmd(0x36); Lcd_WriteData8(0xA0);
        s_XOffset = 20; s_YOffset = 0;
        s_Width = LCD_PANEL_HEIGHT; s_Height = LCD_PANEL_WIDTH;
        break;
    case LCD_DIR_V_FLIP: /* 竖屏 180 度 */
        Lcd_WriteCmd(0x36); Lcd_WriteData8(0xC0);
        s_XOffset = 0; s_YOffset = 20;
        s_Width = LCD_PANEL_WIDTH; s_Height = LCD_PANEL_HEIGHT;
        break;
    default:
        break;
    }
}

/* ST7789 上电初始化（序列与反客例程一致，默认横屏） */
void LCD_Init(void)
{
    HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET);
    LCD_BacklightOff();
    HAL_Delay(10);            /* 上电/复位后需等待至少 5ms 才能发指令 */

    Lcd_WriteCmd(0x01);       /* SWRESET 软复位：MCU 中途复位可能让面板停在半截命令状态，先复位接口状态机 */
    HAL_Delay(10);            /* SWRESET 后需 ≥5ms 才能发后续指令 */

    Lcd_WriteCmd(0x3A); Lcd_WriteData8(0x05);    /* 接口像素格式：16 位 RGB565 */

    /* 电源/电压类寄存器：厂家固定值 */
    Lcd_WriteCmd(0xB2);                                       /* 后门电压 */
    Lcd_WriteData8(0x0C); Lcd_WriteData8(0x0C); Lcd_WriteData8(0x00);
    Lcd_WriteData8(0x33); Lcd_WriteData8(0x33);
    Lcd_WriteCmd(0xB7); Lcd_WriteData8(0x35);                 /* 栅极电压 VGH/VGL */
    Lcd_WriteCmd(0xBB); Lcd_WriteData8(0x19);                 /* 公共电压 VCOM */
    Lcd_WriteCmd(0xC0); Lcd_WriteData8(0x2C);                 /* LCM 控制 */
    Lcd_WriteCmd(0xC2); Lcd_WriteData8(0x01);                 /* VDV/VRH 用户可调 */
    Lcd_WriteCmd(0xC3); Lcd_WriteData8(0x12);                 /* VRH */
    Lcd_WriteCmd(0xC4); Lcd_WriteData8(0x20);                 /* VDV */
    Lcd_WriteCmd(0xC6); Lcd_WriteData8(0x0F);                 /* 刷新帧率 60Hz */
    Lcd_WriteCmd(0xD0);                                       /* 电源控制 */
    Lcd_WriteData8(0xA4); Lcd_WriteData8(0xA1);

    /* Gamma 校正：正极 / 负极 */
    Lcd_WriteCmd(0xE0);
    Lcd_WriteData8(0xD0); Lcd_WriteData8(0x04); Lcd_WriteData8(0x0D); Lcd_WriteData8(0x11);
    Lcd_WriteData8(0x13); Lcd_WriteData8(0x2B); Lcd_WriteData8(0x3F); Lcd_WriteData8(0x54);
    Lcd_WriteData8(0x4C); Lcd_WriteData8(0x18); Lcd_WriteData8(0x0D); Lcd_WriteData8(0x0B);
    Lcd_WriteData8(0x1F); Lcd_WriteData8(0x23);
    Lcd_WriteCmd(0xE1);
    Lcd_WriteData8(0xD0); Lcd_WriteData8(0x04); Lcd_WriteData8(0x0C); Lcd_WriteData8(0x11);
    Lcd_WriteData8(0x13); Lcd_WriteData8(0x2C); Lcd_WriteData8(0x3F); Lcd_WriteData8(0x44);
    Lcd_WriteData8(0x51); Lcd_WriteData8(0x2F); Lcd_WriteData8(0x1F); Lcd_WriteData8(0x1F);
    Lcd_WriteData8(0x20); Lcd_WriteData8(0x23);

    Lcd_WriteCmd(0x21);                                       /* 反显开（常黑面板） */

    LCD_SetDirection(LCD_DIR_H);                              /* 默认横屏 280x240 */

    Lcd_WriteCmd(0x11); HAL_Delay(120);                       /* 退出休眠，等待电源/时钟稳定 */
    Lcd_WriteCmd(0x29);                                       /* 开显示 */

    LCD_Clear(LCD_COLOR_BLACK);                               /* 清屏 */
    LCD_BacklightOn();                                        /* 最后开背光，避免闪白 */
}

void LCD_Clear(uint16_t color)
{
    Lcd_SetAddress(0, 0, (uint16_t)(s_Width - 1U), (uint16_t)(s_Height - 1U));

    HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET);
    Lcd_SpiTransmitFill(&hspi4, color, (uint32_t)s_Width * s_Height);
    HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);
}

/* ========== 显示缓冲与 SPI DMA 路径 ========== */

/* 发送一段（内部）：从 s_SendOffset 起最多 LCD_SEND_CHUNK 像素 */
static void Lcd_SendChunk(void)
{
    uint32_t remain = LCD_SHOW_PIXELS - s_SendOffset;
    uint16_t len = (remain > LCD_SEND_CHUNK) ? (uint16_t)LCD_SEND_CHUNK : (uint16_t)remain;

    if (HAL_SPI_Transmit_DMA(&hspi4, (const uint8_t *)(s_ShowBuf + s_SendOffset), len) != HAL_OK)
    {
        /* 启动失败：释放片选并恢复 8 位，避免 SPI 停在发送态 */
        HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);
        hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
        (void)HAL_SPI_Init(&hspi4);
        s_FlushBusy = false;
        return;
    }
    s_SendOffset += len;
}

/* 居中裁切：源宽 320 保留中间 280 列，逐行拷贝到显示缓冲 */
void LCD_CropToBuffer(const uint16_t *src, uint16_t src_width)
{
    uint16_t x_offset;
    uint32_t row;

    if ((src == NULL) || (src_width < LCD_SHOW_WIDTH))
    {
        return;
    }

    x_offset = (uint16_t)((src_width - LCD_SHOW_WIDTH) / 2U);

    for (row = 0U; row < LCD_SHOW_HEIGHT; row++)
    {
        (void)memcpy(&s_ShowBuf[row * LCD_SHOW_WIDTH],
                     &src[(uint32_t)row * src_width + x_offset],
                     LCD_SHOW_WIDTH * 2U);
    }
}

/* 在显示缓冲内填充矩形（纯内存，不碰 SPI），边界自动裁剪 */
void LCD_FillRectBuffer(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
    uint32_t row;
    uint32_t col;

    if ((x >= LCD_SHOW_WIDTH) || (y >= LCD_SHOW_HEIGHT))
    {
        return;
    }
    if (((uint32_t)x + width) > LCD_SHOW_WIDTH)
    {
        width = (uint16_t)(LCD_SHOW_WIDTH - x);
    }
    if (((uint32_t)y + height) > LCD_SHOW_HEIGHT)
    {
        height = (uint16_t)(LCD_SHOW_HEIGHT - y);
    }

    for (row = 0U; row < height; row++)
    {
        uint16_t *dst = &s_ShowBuf[(uint32_t)(y + row) * LCD_SHOW_WIDTH + x];

        for (col = 0U; col < width; col++)
        {
            dst[col] = color;
        }
    }
}

/* ========== 8x8 点阵字模（仅含帧率显示所需字符） ========== */
static const char s_LcdFontChars[] = "0123456789: CD";

static const uint8_t s_LcdFontData[][8] = {
    {0x3C, 0x66, 0x6E, 0x76, 0x66, 0x66, 0x3C, 0x00},  /* 0 */
    {0x18, 0x38, 0x18, 0x18, 0x18, 0x18, 0x7E, 0x00},  /* 1 */
    {0x3C, 0x66, 0x06, 0x0C, 0x30, 0x60, 0x7E, 0x00},  /* 2 */
    {0x3C, 0x66, 0x06, 0x1C, 0x06, 0x66, 0x3C, 0x00},  /* 3 */
    {0x0C, 0x1C, 0x3C, 0x6C, 0x7E, 0x0C, 0x0C, 0x00},  /* 4 */
    {0x7E, 0x60, 0x7C, 0x06, 0x06, 0x66, 0x3C, 0x00},  /* 5 */
    {0x3C, 0x60, 0x60, 0x7C, 0x66, 0x66, 0x3C, 0x00},  /* 6 */
    {0x7E, 0x06, 0x0C, 0x18, 0x30, 0x30, 0x30, 0x00},  /* 7 */
    {0x3C, 0x66, 0x66, 0x3C, 0x66, 0x66, 0x3C, 0x00},  /* 8 */
    {0x3C, 0x66, 0x66, 0x3E, 0x06, 0x06, 0x3C, 0x00},  /* 9 */
    {0x00, 0x18, 0x18, 0x00, 0x18, 0x18, 0x00, 0x00},  /* : */
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  /* 空格 */
    {0x3C, 0x66, 0x60, 0x60, 0x60, 0x66, 0x3C, 0x00},  /* C */
    {0x78, 0x6C, 0x66, 0x66, 0x66, 0x6C, 0x78, 0x00},  /* D */
};

/* 在显示缓冲上绘制单个 8x8 字符（内部） */
static void Lcd_DrawGlyph(uint16_t x, uint16_t y, char ch, uint16_t fg, uint16_t bg)
{
    const uint8_t *glyph = NULL;
    uint32_t idx;
    uint32_t row;
    uint32_t col;

    for (idx = 0U; s_LcdFontChars[idx] != '\0'; idx++)
    {
        if (s_LcdFontChars[idx] == ch)
        {
            glyph = s_LcdFontData[idx];
            break;
        }
    }
    if (glyph == NULL)
    {
        return;
    }

    for (row = 0U; row < 8U; row++)
    {
        for (col = 0U; col < 8U; col++)
        {
            uint16_t px = (uint16_t)(x + col);
            uint16_t py = (uint16_t)(y + row);

            if ((px < LCD_SHOW_WIDTH) && (py < LCD_SHOW_HEIGHT))
            {
                s_ShowBuf[(uint32_t)py * LCD_SHOW_WIDTH + px] =
                    (((glyph[row] >> (7U - col)) & 0x01U) != 0U) ? fg : bg;
            }
        }
    }
}

/* 在显示缓冲上绘制字符串（纯内存，不碰 SPI） */
void LCD_DrawStringBuffer(uint16_t x, uint16_t y, const char *str, uint16_t fg, uint16_t bg)
{
    uint16_t cx = x;

    if (str == NULL)
    {
        return;
    }

    while (*str != '\0')
    {
        Lcd_DrawGlyph(cx, y, *str, fg, bg);
        cx = (uint16_t)(cx + 8U);
        str++;
    }
}

/* 启动整帧发送：切 16 位 → 设窗口 → 拉低片选 → 清理 cache → 首段 DMA */
void LCD_FlushBuffer(void)
{
    if (s_FlushBusy)
    {
        return;     /* 上一帧未发完，主动丢弃本帧 */
    }

    s_FlushBusy = true;
    s_SendOffset = 0U;

    /* 1) 8 位模式下设置窗口并发 RAMWR（命令/参数必须 8 位，须在切 16 位前完成） */
    Lcd_SetAddress(0U, 0U, (uint16_t)(LCD_SHOW_WIDTH - 1U), (uint16_t)(LCD_SHOW_HEIGHT - 1U));

    /* 2) 切 16 位数据宽度（HAL 按 HALFWORD 对齐搬运），准备整帧 DMA */
    hspi4.Init.DataSize = SPI_DATASIZE_16BIT;
    if (HAL_SPI_Init(&hspi4) != HAL_OK)
    {
        hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
        (void)HAL_SPI_Init(&hspi4);
        s_FlushBusy = false;
        return;
    }

    /* 3) 拉低片选进入数据阶段，两段之间保持低 */
    HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET);

    /* DMA 从内存取数，发送前必须把 D-Cache 刷回 */
    SCB_CleanDCache_by_Addr((uint32_t *)s_ShowBuf, (int32_t)LCD_SHOW_BYTES);

    Lcd_SendChunk();
}

bool LCD_IsFlushBusy(void)
{
    return s_FlushBusy;
}

/* DMA 发送完成（EOT 中断）：还有剩余就续发下一段，否则收尾 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI4)
    {
        return;
    }

    if (s_SendOffset < LCD_SHOW_PIXELS)
    {
        Lcd_SendChunk();
    }
    else
    {
        HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);   /* 整帧结束，释放片选 */
        hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
        (void)HAL_SPI_Init(&hspi4);
        s_FlushBusy = false;
    }
}
