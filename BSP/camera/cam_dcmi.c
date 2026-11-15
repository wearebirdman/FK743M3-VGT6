#include "cam_dcmi.h"

#include "dcmi.h"

/* ========== 帧缓冲 ========== */
/* 放入 .dma_buffer 段（AXI SRAM），32 字节对齐满足 cache line 要求，
 * 避免 SCB_InvalidateDCache 误伤相邻数据 */
__attribute__((section(".dma_buffer"), aligned(32)))
static uint32_t s_FrameBuf[CAM_FRAME_WORDS];

/* ========== 模块私有状态 ========== */
static CamFrameFn s_FrameCb = NULL;                 /* 帧完成回调（ISR 上下文） */
static volatile uint32_t s_FrameCount = 0;          /* 完整帧计数 */
static volatile CamErr_t s_LastError = CAM_OK;      /* 最近错误码 */

/* ========== 裁剪配置 ========== */

/* RGB565 下 DCMI 按 PCLK 计数：1 像素 = 2 个 PCLK。
 * 寄存器均为 0 基（HAL_DCMI_ConfigCrop 直接写寄存器值）：
 *   X0    = 水平起始偏移（PCLK 数），必须为偶数，否则字节错位、色彩错乱
 *   Y0    = 垂直起始偏移（行数）
 *   XSize = 有效 PCLK 数 = 输出宽 * 2 - 1
 *   YSize = 有效行数     = 输出高 - 1
 * 居中裁剪：水平偏移 = (320-280)/2 = 20 像素 = 40 PCLK；垂直不裁，Y0 = 0 */
static CamErr_t Cam_ConfigCrop(void)
{
    uint32_t x_offset;
    uint32_t y_offset;

    if ((CAM_OUT_WIDTH > CAM_SENSOR_WIDTH) || (CAM_OUT_HEIGHT > CAM_SENSOR_HEIGHT))
    {
        return CAM_ERR_PARAM;   /* 裁剪不允许输出大于传感器画面（等于则该方向不裁） */
    }

    x_offset = (uint32_t)(CAM_SENSOR_WIDTH - CAM_OUT_WIDTH);        /* PCLK 数，恒为偶数 */
    y_offset = (uint32_t)(CAM_SENSOR_HEIGHT - CAM_OUT_HEIGHT) / 2U; /* 行数，居中 */

    HAL_DCMI_ConfigCrop(&hdcmi,
                        x_offset,
                        y_offset,
                        CAM_OUT_WIDTH * 2U - 1U,
                        CAM_OUT_HEIGHT - 1U);
    HAL_DCMI_EnableCrop(&hdcmi);

    return CAM_OK;
}

/* ========== 捕获控制 ========== */

/* 启动连续采集：配置裁剪 → CONTINUOUS 模式 + 循环 DMA，帧缓冲被逐帧覆盖（单缓冲，一期够用） */
CamErr_t Cam_CaptureStart(CamFrameFn cb)
{
    if (Cam_ConfigCrop() != CAM_OK)
    {
        return CAM_ERR_PARAM;
    }

    s_FrameCb = cb;
    s_FrameCount = 0;
    s_LastError = CAM_OK;

    if (HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS,
                           (uint32_t)s_FrameBuf, CAM_FRAME_WORDS) != HAL_OK)
    {
        s_FrameCb = NULL;
        return CAM_ERR_STATE;
    }
    return CAM_OK;
}

/* 停止采集：HAL 内部会一并中止绑定的 DMA 流 */
void Cam_CaptureStop(void)
{
    HAL_DCMI_Stop(&hdcmi);
    s_FrameCb = NULL;
}

uint32_t Cam_GetFrameCount(void)
{
    return s_FrameCount;
}

CamErr_t Cam_GetLastError(void)
{
    return s_LastError;
}

/* 取帧：D-Cache 已开启，读前必须失效，否则 CPU 可能读到缓存中的旧数据 */
const uint8_t *Cam_GetFrame(void)
{
    SCB_InvalidateDCache_by_Addr(s_FrameBuf, (int32_t)CAM_FRAME_BYTES);
    return (const uint8_t *)s_FrameBuf;
}

/* ========== HAL 弱符号重写（DCMI 中断回调） ========== */

/* 帧完成（VSYNC 帧尾）：每帧触发一次，仅做计数与唤醒，不搬数据 */
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *phdcmi)
{
    if (phdcmi->Instance == DCMI)
    {
        s_FrameCount++;
        if (s_FrameCb != NULL)
        {
            s_FrameCb();
        }
    }
}

/* 同步错误/溢出：HAL 已停止捕获，唤醒任务让其感知错误状态 */
void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *phdcmi)
{
    if (phdcmi->Instance == DCMI)
    {
        s_LastError = CAM_ERR_STATE;
        if (s_FrameCb != NULL)
        {
            s_FrameCb();
        }
    }
}
