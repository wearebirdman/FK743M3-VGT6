#include "camera_task.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "cam_dcmi.h"
#include "lcd_st7789.h"
#include "ov5640.h"
#include "usart.h"

/* ========== 任务参数 ========== */
#define CAMERA_TASK_STACK       (256U * 4U)     /* 1 KB，任务仅做初始化与统计打印 */
#define CAMERA_RETRY_MAX        3U              /* 探测芯片重试次数 */
#define CAMERA_PRINT_PERIOD     15U             /* 每 15 帧打印一次统计（约 1s） */

static osThreadId_t s_CameraTaskHandle;         /* 任务句柄（ISR 通知目标） */

/* ========== 帧中断回调 ========== */

/* DCMI 帧完成中断里调用：仅唤醒采集任务，不做任何数据处理 */
static void Cam_FrameNotify(void)
{
    BaseType_t woken = pdFALSE;

    if (s_CameraTaskHandle != NULL)
    {
        vTaskNotifyGiveFromISR((TaskHandle_t)s_CameraTaskHandle, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

/* ========== 任务创建 ========== */

/* 创建采集任务，需在 osKernelStart 之前调用（main.c USER CODE 2） */
void Camera_Init(void)
{
    const osThreadAttr_t attr = {
        .name       = "Camera_Proc",
        .stack_size = CAMERA_TASK_STACK,
        .priority   = osPriorityNormal,
    };

    s_CameraTaskHandle = osThreadNew(Camera_Proc, NULL, &attr);
}

/* ========== 任务主体 ========== */

void Camera_Proc(void *argument)
{
    (void)argument;

    /* ---- 上电时序并探测芯片（失败重试） ---- */
    Ov5640Err_t err = OV5640_ERR_I2C;
    for (uint32_t retry = 0; retry < CAMERA_RETRY_MAX && err != OV5640_OK; retry++)
    {
        OV5640_HwPowerOn();
        OV5640_SoftReset();
        err = OV5640_CheckId();
    }
    if (err != OV5640_OK)
    {
        Uart_Printf("[Cam] OV5640 not detected (err=%d)\r\n", (int)err);
        for (;;)
        {
            osDelay(1000);
        }
    }

    /* ---- 下发初始化表（RGB565）并设置 320x240 输出 ---- */
    err = OV5640_Init(OV5640_PIXFMT_RGB565);
    if (err == OV5640_OK)
    {
        err = OV5640_SetFramesize(CAM_SENSOR_WIDTH, CAM_SENSOR_HEIGHT);
    }
    if (err != OV5640_OK)
    {
        Uart_Printf("[Cam] OV5640 init fail (err=%d)\r\n", (int)err);
        for (;;)
        {
            osDelay(1000);
        }
    }

    /* ---- 初始化 LCD（ST7789 横屏 280x240，与帧缓冲同序） ---- */
    LCD_Init();

    /* ---- 下载 AF 固件并启动持续自动对焦（失败不影响出图，仅无法对焦） ---- */
    if (OV5640_AF_DownloadFirmware() == OV5640_OK)
    {
        OV5640_AF_TriggerContinuous();
        Uart_Printf("[Cam] AF ready (continuous)\r\n");
    }
    else
    {
        Uart_Printf("[Cam] AF firmware download fail\r\n");
    }

    /* ---- 启动 DCMI 连续采集（硬件裁剪 320x240 → 280x240） ---- */
    if (Cam_CaptureStart(Cam_FrameNotify) != CAM_OK)
    {
        Uart_Printf("[Cam] DCMI start fail\r\n");
        for (;;)
        {
            osDelay(1000);
        }
    }
    Uart_Printf("[Cam] 280x240 RGB565 capture running\r\n");

    /* ---- 帧循环：等帧 → 整帧推屏 → 统计打印 ---- */
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        LCD_CopyBuffer(0, 0, CAM_OUT_WIDTH, CAM_OUT_HEIGHT,
                       (const uint16_t *)Cam_GetFrame());

        uint32_t count = Cam_GetFrameCount();
        if ((count % CAMERA_PRINT_PERIOD) == 0U)
        {
            const uint32_t *words = (const uint32_t *)Cam_GetFrame();
            Uart_Printf("[Cam] frame=%u w0=%08X w1=%08X err=%d\r\n",
                        count, words[0], words[1], (int)Cam_GetLastError());
        }
    }
}
