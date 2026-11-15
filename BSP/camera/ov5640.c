#include "ov5640.h"

#include "ov5640_cfg.h"

#include "i2c.h"

/* ========== SCCB 单寄存器读写（16 位寄存器地址，HAL 自动处理高字节在前） ========== */

Ov5640Err_t OV5640_WriteReg(uint16_t reg, uint8_t val)
{
    if (HAL_I2C_Mem_Write(OV5640_I2C_HANDLE, OV5640_SCCB_ADDR, reg,
                          I2C_MEMADD_SIZE_16BIT, &val, 1U, OV5640_I2C_TIMEOUT) != HAL_OK)
    {
        return OV5640_ERR_I2C;
    }
    return OV5640_OK;
}

Ov5640Err_t OV5640_ReadReg(uint16_t reg, uint8_t *val)
{
    if (val == NULL)
    {
        return OV5640_ERR_PARAM;
    }
    if (HAL_I2C_Mem_Read(OV5640_I2C_HANDLE, OV5640_SCCB_ADDR, reg,
                         I2C_MEMADD_SIZE_16BIT, val, 1U, OV5640_I2C_TIMEOUT) != HAL_OK)
    {
        return OV5640_ERR_I2C;
    }
    return OV5640_OK;
}

/* ========== 上电与复位 ========== */

/* PWDN 上电时序：拉高 → 保持 → 拉低（模块硬件 RC 复位 6~10ms）→ 等待稳定后才准 SCCB 通信 */
Ov5640Err_t OV5640_HwPowerOn(void)
{
    HAL_GPIO_WritePin(OV5640_PWDN_PORT, OV5640_PWDN_PIN, GPIO_PIN_SET);
    HAL_Delay(OV5640_PWDN_HOLD_MS);

    HAL_GPIO_WritePin(OV5640_PWDN_PORT, OV5640_PWDN_PIN, GPIO_PIN_RESET);
    HAL_Delay(OV5640_RC_RESET_MS);
    HAL_Delay(OV5640_SETTLE_MS);

    return OV5640_OK;
}

/* 软复位：先把主时钟切回输入时钟，再触发软件复位 */
Ov5640Err_t OV5640_SoftReset(void)
{
    Ov5640Err_t err;

    err = OV5640_WriteReg(OV5640_REG_CLK_SEL, 0x11);    /* 手册建议：复位前直接用 XVCLK 作主时钟 */
    if (err != OV5640_OK)
    {
        return err;
    }
    err = OV5640_WriteReg(OV5640_REG_SYS_CTRL, 0x82);   /* 执行软复位 */
    if (err != OV5640_OK)
    {
        return err;
    }
    HAL_Delay(OV5640_SOFT_RESET_MS);

    return OV5640_OK;
}

/* 读 ID 校验芯片在位（0x300A=0x56，0x300B=0x40） */
Ov5640Err_t OV5640_CheckId(void)
{
    uint8_t id_h = 0;
    uint8_t id_l = 0;
    Ov5640Err_t err;

    err = OV5640_ReadReg(OV5640_REG_CHIPID_H, &id_h);
    if (err != OV5640_OK)
    {
        return err;
    }
    err = OV5640_ReadReg(OV5640_REG_CHIPID_L, &id_l);
    if (err != OV5640_OK)
    {
        return err;
    }

    if ((((uint16_t)id_h << 8) | id_l) != OV5640_CHIPID_VALUE)
    {
        return OV5640_ERR_ID;
    }
    return OV5640_OK;
}

/* ========== 初始化与参数设置 ========== */

/* 下发初始化表（表内已含 RGB565 与 4:3 窗口配置），再按 fmt 修正格式寄存器 */
Ov5640Err_t OV5640_Init(Ov5640Pixfmt_t fmt)
{
    Ov5640Err_t err;

    for (uint32_t i = 0; i < OV5640_INIT_CFG_NUM; i++)
    {
        err = OV5640_WriteReg(OV5640_INIT_CFG[i][0], (uint8_t)OV5640_INIT_CFG[i][1]);
        if (err != OV5640_OK)
        {
            return err;
        }
    }

    return OV5640_SetPixfmt(fmt);
}

/* 设置输出尺寸：ISP 窗口固定 4:3，输出宽高必须保持 4:3，否则画面变形 */
Ov5640Err_t OV5640_SetFramesize(uint16_t width, uint16_t height)
{
    if ((width == 0U) || (height == 0U) || ((uint32_t)width * 3U != (uint32_t)height * 4U))
    {
        return OV5640_ERR_PARAM;
    }

    /* timing 类寄存器需 group 提交：0x03 开始 → 写入 → 0x13 结束 → 0xA3 生效 */
    OV5640_WriteReg(OV5640_REG_GRP_ACCESS, 0x03);
    OV5640_WriteReg(OV5640_REG_DVPHO_H, (uint8_t)(width >> 8));
    OV5640_WriteReg(OV5640_REG_DVPHO_L, (uint8_t)(width & 0xFFU));
    OV5640_WriteReg(OV5640_REG_DVPVO_H, (uint8_t)(height >> 8));
    OV5640_WriteReg(OV5640_REG_DVPVO_L, (uint8_t)(height & 0xFFU));
    OV5640_WriteReg(OV5640_REG_GRP_ACCESS, 0x13);
    if (OV5640_WriteReg(OV5640_REG_GRP_ACCESS, 0xA3) != OV5640_OK)
    {
        return OV5640_ERR_I2C;
    }

    return OV5640_OK;
}

/* ========== 自动对焦（AF 固件由 ov5640_af.c 提供，每次上电下载一次） ========== */

/* 下载 AF 固件至片内 MCU：复位 MCU → 连续写入 0x8000 → 释放复位 → 轮询就绪 */
Ov5640Err_t OV5640_AF_DownloadFirmware(void)
{
    uint8_t status;

    if (OV5640_WriteReg(OV5640_REG_SYS_RST, 0x20) != OV5640_OK)     /* Bit[5]=1：MCU 复位，开放固件写入 */
    {
        return OV5640_ERR_I2C;
    }
    if (HAL_I2C_Mem_Write(OV5640_I2C_HANDLE, OV5640_SCCB_ADDR, OV5640_AF_MCU_BASE,
                          I2C_MEMADD_SIZE_16BIT, (uint8_t *)OV5640_AF_Firmware,
                          (uint16_t)OV5640_AF_Firmware_Size, 1000U) != HAL_OK)
    {
        return OV5640_ERR_I2C;
    }
    if (OV5640_WriteReg(OV5640_REG_SYS_RST, 0x00) != OV5640_OK)     /* 释放 MCU 复位，固件开始运行 */
    {
        return OV5640_ERR_I2C;
    }

    /* 固件启动有初始化过程，轮询状态直到 0x70（就绪，镜头回无穷远） */
    for (uint32_t i = 0; i < 100U; i++)
    {
        if (OV5640_ReadReg(OV5640_REG_AF_STATUS, &status) != OV5640_OK)
        {
            return OV5640_ERR_I2C;
        }
        if (status == 0x70U)
        {
            return OV5640_OK;
        }
        HAL_Delay(10);
    }
    return OV5640_ERR_AF;                                           /* 超时未就绪：固件未写入成功 */
}

/* 触发持续自动对焦：传感器检测到画面离焦时自行对焦，无需干预 */
Ov5640Err_t OV5640_AF_TriggerContinuous(void)
{
    return OV5640_WriteReg(OV5640_REG_AF_CMD, 0x04);
}

/* 触发单次自动对焦（过程约 500ms） */
Ov5640Err_t OV5640_AF_TriggerSingle(void)
{
    return OV5640_WriteReg(OV5640_REG_AF_CMD, 0x03);
}

/* 释放对焦马达：镜头回到初始（无穷远）位置 */
Ov5640Err_t OV5640_AF_Release(void)
{
    return OV5640_WriteReg(OV5640_REG_AF_CMD, 0x08);
}

/* 查询对焦状态：done 输出 1=空闲/完成，0=正在对焦 */
Ov5640Err_t OV5640_AF_QueryStatus(uint8_t *done)
{
    uint8_t status;

    if (done == NULL)
    {
        return OV5640_ERR_PARAM;
    }
    if (OV5640_ReadReg(OV5640_REG_AF_STATUS, &status) != OV5640_OK)
    {
        return OV5640_ERR_I2C;
    }
    *done = ((status == 0x10U) || (status == 0x20U) || (status == 0x70U)) ? 1U : 0U;
    return OV5640_OK;
}

/* 切换输出像素格式（一期实现 RGB565；YUV422 待后续补充） */
Ov5640Err_t OV5640_SetPixfmt(Ov5640Pixfmt_t fmt)
{
    Ov5640Err_t err;
    uint8_t reg;

    switch (fmt)
    {
    case OV5640_PIXFMT_RGB565:
        /* RGB565：低字节先出，小端 uint16 即标准 0xRRGGBB，无需 REV16 */
        err = OV5640_WriteReg(OV5640_REG_FMT_CTRL, 0x6F);
        if (err != OV5640_OK)
        {
            return err;
        }
        err = OV5640_WriteReg(OV5640_REG_FMT_MUX, 0x01);
        if (err != OV5640_OK)
        {
            return err;
        }
        /* 关 JPEG：3821 Bit[5]=0，保留 Bit[0] 水平合并 */
        err = OV5640_ReadReg(OV5640_REG_MIRROR, &reg);
        if (err != OV5640_OK)
        {
            return err;
        }
        err = OV5640_WriteReg(OV5640_REG_MIRROR, reg & (uint8_t)~0x20U);
        if (err != OV5640_OK)
        {
            return err;
        }
        /* 3002 Bit[4:2] 使能 VFIFO/JFIFO/JPG 复位释放，3006 Bit[5]/Bit[3] 关 JPG 时钟 */
        err = OV5640_ReadReg(0x3002U, &reg);
        if (err != OV5640_OK)
        {
            return err;
        }
        err = OV5640_WriteReg(0x3002U, (uint8_t)((reg & (uint8_t)~0x1CU) | 0x1CU));
        if (err != OV5640_OK)
        {
            return err;
        }
        err = OV5640_ReadReg(0x3006U, &reg);
        if (err != OV5640_OK)
        {
            return err;
        }
        return OV5640_WriteReg(0x3006U, (uint8_t)(reg & (uint8_t)~0x28U));

    case OV5640_PIXFMT_JPEG:
        /* 预留：JPEG 需配合 DCMI 快照模式与 JPEG Codec，后续实现 */
        return OV5640_ERR_PARAM;

    case OV5640_PIXFMT_YUV422:
    default:
        /* 预留：后续实现 */
        return OV5640_ERR_PARAM;
    }
}

/* 镜像/翻转：1 使能、0 关闭（画面方向与摄像头物理安装不一致时使用） */
Ov5640Err_t OV5640_SetOrientation(uint8_t mirror, uint8_t flip)
{
    Ov5640Err_t err;
    uint8_t reg;

    err = OV5640_ReadReg(OV5640_REG_FLIP, &reg);
    if (err != OV5640_OK)
    {
        return err;
    }
    reg = (uint8_t)((reg & (uint8_t)~0x06U) | (flip ? 0x06U : 0x00U));
    err = OV5640_WriteReg(OV5640_REG_FLIP, reg);
    if (err != OV5640_OK)
    {
        return err;
    }

    err = OV5640_ReadReg(OV5640_REG_MIRROR, &reg);
    if (err != OV5640_OK)
    {
        return err;
    }
    reg = (uint8_t)((reg & (uint8_t)~0x06U) | (mirror ? 0x06U : 0x00U));
    return OV5640_WriteReg(OV5640_REG_MIRROR, reg);
}
