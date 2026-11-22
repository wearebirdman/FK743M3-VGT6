#include "camera_task.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "cam_dcmi.h"
#include "lcd_st7789.h"
#include "ov5640.h"
#include "usart.h"
#include "vision_task.h"

/* ========== 任务参数 ========== */
#define CAMERA_TASK_STACK       (256U * 4U)     /* 1 KB，任务做采集调度/裁切推屏/统计打印 */
#define CAMERA_RETRY_MAX        3U              /* 探测芯片重试次数 */
#define CAMERA_PRINT_PERIOD     60U             /* 每 60 帧打印一次统计（约 1s） */

/* 显示抽帧系数：1=每帧都显示（20fps 采集 → 20fps 显示），2=每两帧显示（30fps 采集 → 15fps 显示） */
#define CAMERA_DISPLAY_DIV      1U

static osThreadId_t s_CameraTaskHandle;         /* 任务句柄（ISR 通知目标） */

/* ========== 帧率统计（显示于 LCD 左上角） ========== */
static uint32_t s_FpsTick = 0U;                 /* 上次统计时刻 */
static uint32_t s_CapFrames = 0U;               /* 本秒采集帧数 */
static uint32_t s_DispFrames = 0U;              /* 本秒推屏次数 */
static uint32_t s_CapFps = 0U;                  /* 上一秒采集帧率 */
static uint32_t s_DispFps = 0U;                 /* 上一秒显示帧率 */

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

/* ========== 帧率统计 ========== */

/* 每秒刷新一次采集/显示帧率（pushed 表示本帧是否真正推屏） */
static void Cam_UpdateFps(bool pushed)
{
    uint32_t now = HAL_GetTick();

    s_CapFrames++;
    if (pushed)
        s_DispFrames++;

    if ((now - s_FpsTick) >= 1000U)
    {
        s_CapFps = s_CapFrames;
        s_DispFps = s_DispFrames;
        s_CapFrames = 0U;
        s_DispFrames = 0U;
        s_FpsTick = now;
    }
}

/* 格式化为 "C:xx D:xx"（两位整数，便于 8x8 字体显示） */
static void Cam_FormatFps(char *buf)
{
    buf[0] = 'C';
    buf[1] = ':';
    buf[2] = (char)('0' + ((s_CapFps / 10U) % 10U));
    buf[3] = (char)('0' + (s_CapFps % 10U));
    buf[4] = ' ';
    buf[5] = 'D';
    buf[6] = ':';
    buf[7] = (char)('0' + ((s_DispFps / 10U) % 10U));
    buf[8] = (char)('0' + (s_DispFps % 10U));
    buf[9] = '\0';
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

    /* ---- 启动 DCMI 双缓冲采集（整幅 320x240，硬件 DBM 交替写两块） ---- */
    if (Cam_CaptureStart(Cam_FrameNotify) != CAM_OK)
    {
        Uart_Printf("[Cam] DCMI start fail\r\n");
        for (;;)
        {
            osDelay(1000);
        }
    }
    Uart_Printf("[Cam] 320x240 RGB565 capture 60fps, display 30fps\r\n");

    /* ---- 帧循环：每帧视觉识别（60fps），偶数帧裁切推屏（30fps） ---- */
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        const uint16_t *frame = (const uint16_t *)Cam_GetFrame();
        uint32_t frame_no = Cam_GetFrameCount();
        bool pushed = false;

        /* 每帧都做视觉：先固化本帧快照并唤醒识别任务（其优先级更高，会立刻抢占算完本帧） */
        Vision_PushFrame(frame);

        /* 按抽帧系数决定本帧是否推屏；上一帧还没发完就直接丢弃，不阻塞采集 */
        if (((frame_no % CAMERA_DISPLAY_DIV) == 0U) && (!LCD_IsFlushBusy()))
        {
            char line[12];

            LCD_CropToBuffer(frame, CAM_OUT_WIDTH);
            Vision_DrawOverlay();
            Cam_FormatFps(line);
            LCD_DrawStringBuffer(4U, 4U, line, LCD_COLOR_WHITE, LCD_COLOR_BLACK);
            LCD_FlushBuffer();
            pushed = true;
        }

        Cam_UpdateFps(pushed);

        if ((frame_no % CAMERA_PRINT_PERIOD) == 0U)
        {
            Uart_Printf("[Cam] frame=%u err=%d\r\n", frame_no, (int)Cam_GetLastError());
        }
    }
}
