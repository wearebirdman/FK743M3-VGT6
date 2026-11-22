#include "vision_task.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "lcd_st7789.h"
#include "usart.h"
#include "vision_core.h"

/* ========== 任务参数 ========== */
#define VISION_TASK_STACK       (384U * 4U)             /* 1.5 KB，含 vsnprintf 的栈需求余量 */
#define VISION_PRINT_PERIOD     10U                     /* 每 10 帧上报一次 */
#define VISION_BOX_THICK        2U                      /* 外接框线宽（源分辨率） */
#define VISION_CROSS_HALF       5U                      /* 质心十字半长（源分辨率） */

static osThreadId_t s_VisionTaskHandle;     /* 任务句柄（采集任务通知目标） */
static VisResult_t s_Result;                /* 最近一次识别结果（识别任务写、采集任务读） */

/* ========== 内部实现 ========== */

/* 串口上报：只报最大目标，坐标与面积换算回源分辨率，便于上位机直接使用
 * frames 为视觉任务已处理帧数，用于确认识别链路在持续运行 */
static void Vis_Report(const VisResult_t *result, uint32_t frames)
{
    if (result->count == 0U)
    {
        Uart_Printf("[Vis] f=%u none\r\n", (unsigned int)frames);
        return;
    }

    const VisBlob_t *blob = &result->blob[0];

    Uart_Printf("[Vis] f=%u n=%u cx=%u cy=%u area=%u\r\n",
                (unsigned int)frames,
                (unsigned int)result->count,
                (unsigned int)(blob->cx * VIS_DECIM),
                (unsigned int)(blob->cy * VIS_DECIM),
                (unsigned int)(blob->area * VIS_DECIM * VIS_DECIM));
}

/* 源坐标（320 域）转显示缓冲坐标（280 域）后填充，纯内存不碰 SPI */
static void Vis_FillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
    int32_t bx = (int32_t)x - (int32_t)LCD_CROP_OFFSET;
    int32_t ex = bx + (int32_t)width;

    if ((ex <= 0) || (bx >= (int32_t)LCD_SHOW_WIDTH) || (y >= LCD_SHOW_HEIGHT))
        return;

    if (bx < 0)
        bx = 0;
    if (ex > (int32_t)LCD_SHOW_WIDTH)
        ex = (int32_t)LCD_SHOW_WIDTH;

    LCD_FillRectBuffer((uint16_t)bx, y, (uint16_t)(ex - bx), height, color);
}

#if VISION_DEBUG_MASK
/* 调试：把二值掩码放大 2 倍绘到屏上（白色=前景），用于目视调 HSV 阈值 */
static void Vis_DrawMask(void)
{
    const uint8_t *mask = Vision_GetMask();

    for (uint32_t y = 0U; y < VIS_HEIGHT; y++)
    {
        const uint8_t *row = &mask[y * VIS_WIDTH];
        uint32_t x = 0U;

        while (x < VIS_WIDTH)
        {
            uint32_t x0 = x;

            if (row[x] == 0U)
            {
                x++;
                continue;
            }

            while ((x < VIS_WIDTH) && (row[x] != 0U))
                x++;

            Vis_FillRect((uint16_t)(x0 * VIS_DECIM), (uint16_t)(y * VIS_DECIM),
                         (uint16_t)((x - x0) * VIS_DECIM), (uint16_t)VIS_DECIM, LCD_COLOR_WHITE);
        }
    }
}
#endif

/* ========== 任务创建 ========== */

void Vision_Init(void)
{
    const osThreadAttr_t attr = {
        .name       = "Vision_Proc",
        .stack_size = VISION_TASK_STACK,
        .priority   = osPriorityAboveNormal,   /* 必须高于采集任务：推屏耗时接近帧周期，低优先级会被饿死 */
    };

    s_VisionTaskHandle = osThreadNew(Vision_Proc, NULL, &attr);
}

/* ========== 帧接入 ========== */

/* 采集任务在帧中断唤醒后立即调用：先做降采样阈值（约 1~2ms）固化本帧快照，
 * 随后通知视觉任务；视觉任务优先级更高，会在此处立刻抢占完成连通域计算 */
void Vision_PushFrame(const uint16_t *frame)
{
    if ((s_VisionTaskHandle == NULL) || (frame == NULL))
        return;

    Vision_BuildMask(frame);
    (void)xTaskNotifyGive((TaskHandle_t)s_VisionTaskHandle);
}

/* ========== 任务主体 ========== */

void Vision_Proc(void *argument)
{
    VisResult_t result;
    uint32_t frames = 0U;

    (void)argument;

    for (;;)
    {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   /* 只处理最新帧，不积压 */

        Vision_FindBlobs(&result);

        taskENTER_CRITICAL();
        s_Result = result;
        taskEXIT_CRITICAL();

        frames++;
        if ((frames % VISION_PRINT_PERIOD) == 0U)
            Vis_Report(&result, frames);
    }
}

/* ========== 结果叠加 ========== */

/* 采集任务裁切后调用：在显示缓冲上画外接框 + 质心十字，结果落后一帧，不影响观感 */
void Vision_DrawOverlay(void)
{
#if VISION_DEBUG_MASK
    Vis_DrawMask();
#else
    VisResult_t result;
    const VisBlob_t *blob;
    uint16_t x0;
    uint16_t y0;
    uint16_t x1;
    uint16_t y1;
    uint16_t width;
    uint16_t height;
    uint16_t cx;
    uint16_t cy;

    taskENTER_CRITICAL();
    result = s_Result;
    taskEXIT_CRITICAL();

    if (result.count == 0U)
        return;

    blob = &result.blob[0];
    x0 = (uint16_t)(blob->x0 * VIS_DECIM);
    y0 = (uint16_t)(blob->y0 * VIS_DECIM);
    x1 = (uint16_t)(blob->x1 * VIS_DECIM + (VIS_DECIM - 1U));
    y1 = (uint16_t)(blob->y1 * VIS_DECIM + (VIS_DECIM - 1U));
    width = (uint16_t)(x1 - x0 + 1U);
    height = (uint16_t)(y1 - y0 + 1U);

    /* 外接框四边描线，不填充，避免遮挡目标本身 */
    Vis_FillRect(x0, y0, width, VISION_BOX_THICK, LCD_COLOR_GREEN);
    Vis_FillRect(x0, (uint16_t)(y1 - VISION_BOX_THICK + 1U), width, VISION_BOX_THICK, LCD_COLOR_GREEN);
    Vis_FillRect(x0, y0, VISION_BOX_THICK, height, LCD_COLOR_GREEN);
    Vis_FillRect((uint16_t)(x1 - VISION_BOX_THICK + 1U), y0, VISION_BOX_THICK, height, LCD_COLOR_GREEN);

    /* 质心十字 */
    cx = (uint16_t)(blob->cx * VIS_DECIM);
    cy = (uint16_t)(blob->cy * VIS_DECIM);
    Vis_FillRect((cx > VISION_CROSS_HALF) ? (uint16_t)(cx - VISION_CROSS_HALF) : 0U, cy,
                 (uint16_t)(VISION_CROSS_HALF * 2U + 1U), 1U, LCD_COLOR_WHITE);
    Vis_FillRect(cx, (cy > VISION_CROSS_HALF) ? (uint16_t)(cy - VISION_CROSS_HALF) : 0U,
                 1U, (uint16_t)(VISION_CROSS_HALF * 2U + 1U), LCD_COLOR_WHITE);
#endif
}
