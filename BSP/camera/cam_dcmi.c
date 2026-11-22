#include "cam_dcmi.h"

#include "dcmi.h"

/* ========== 帧缓冲 ========== */
/* 双缓冲放入 .dma_buffer 段（AXI SRAM），32 字节对齐满足 cache line 要求；
 * 两块必须连续等长，供 HAL 多缓冲分支使用（总长 76800 words > 65535） */
__attribute__((section(".dma_buffer"), aligned(32)))
static uint32_t s_FrameBuf[CAM_FRAME_BLOCK_COUNT][CAM_FRAME_WORDS];

/* ========== 模块私有状态 ========== */
static CamFrameFn s_FrameCb = NULL;                 /* 帧完成回调（ISR 上下文） */
static volatile uint8_t  s_ReadyIndex = 0U;         /* 最近完成块索引（0=A，1=B） */
static volatile uint32_t s_FrameCount = 0U;         /* 完整帧计数 */
static volatile CamErr_t s_LastError = CAM_OK;      /* 最近错误码 */

/* ========== 捕获控制 ========== */

/* 启动双缓冲采集：不裁剪，整幅 320x240 由硬件 DBM 在两块间交替写入 */
CamErr_t Cam_CaptureStart(CamFrameFn cb)
{
    s_FrameCb = cb;
    s_FrameCount = 0U;
    s_ReadyIndex = 0U;
    s_LastError = CAM_OK;

    HAL_DCMI_DisableCrop(&hdcmi);   /* 确保不裁剪，采集完整 320x240 */

    /* 长度传总长 76800 words > 65535，HAL 自动切成两块等长的多缓冲 */
    if (HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS,
                           (uint32_t)s_FrameBuf, CAM_FRAME_TOTAL_WORDS) != HAL_OK)
    {
        s_FrameCb = NULL;
        return CAM_ERR_STATE;
    }
    return CAM_OK;
}

uint32_t Cam_GetFrameCount(void)
{
    return s_FrameCount;
}

CamErr_t Cam_GetLastError(void)
{
    return s_LastError;
}

/* 取帧：返回最近完成块的基址；D-Cache 已开启，读前只失效该块 */
const uint8_t *Cam_GetFrame(void)
{
    const uint32_t *buf = s_FrameBuf[s_ReadyIndex];

    SCB_InvalidateDCache_by_Addr((uint32_t *)buf, (int32_t)CAM_FRAME_BYTES);
    return (const uint8_t *)buf;
}

/* ========== HAL 弱符号重写（DCMI 中断回调） ========== */

/* 帧完成（VSYNC 帧尾）：每帧触发一次，记录刚完成块并唤醒，不搬数据 */
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *phdcmi)
{
    if (phdcmi->Instance == DCMI)
    {
        /* HAL 已在 DMA 完成中断里翻转 CT 并重装地址，CT 现指向下一块，刚完成的为互补块 */
        const DMA_Stream_TypeDef *dma = (const DMA_Stream_TypeDef *)phdcmi->DMA_Handle->Instance;
        uint32_t ct = ((dma->CR & DMA_SxCR_CT) != 0U) ? 1U : 0U;

        s_ReadyIndex = (uint8_t)(ct ^ 1U);
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
