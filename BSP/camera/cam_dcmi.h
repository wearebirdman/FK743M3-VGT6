#ifndef __CAM_DCMI_H
#define __CAM_DCMI_H

#include "main.h"

/* ========== 尺寸参数 ==========
 * 传感器输出 320x240（4:3，不动初始化表），DCMI 硬件裁剪居中取 280x240，
 * 与屏幕分辨率一致，帧缓冲可直接整帧送屏显示 */
#define CAM_SENSOR_WIDTH        320U
#define CAM_SENSOR_HEIGHT       240U
#define CAM_OUT_WIDTH           280U
#define CAM_OUT_HEIGHT          240U

/* RGB565：2 字节/像素 */
#define CAM_FRAME_BYTES         (CAM_OUT_WIDTH * CAM_OUT_HEIGHT * 2U)   /* 134400 */
#define CAM_FRAME_WORDS         (CAM_FRAME_BYTES / 4U)                  /* 33600，< NDTR 上限 65535 */

/* 错误码定义 */
typedef enum {
    CAM_OK = 0,           /* 成功 */
    CAM_ERR_PARAM,        /* 裁剪参数非法（输出大于传感器、水平偏移为奇数等） */
    CAM_ERR_STATE,        /* 外设状态错误（启动失败/同步错误/溢出） */
} CamErr_t;

/* 帧完成回调类型（在 DCMI 中断上下文执行，实现必须非阻塞） */
typedef void (*CamFrameFn)(void);

/* 函数接口 */
CamErr_t        Cam_CaptureStart(CamFrameFn cb);   /* 配置裁剪并启动连续采集（循环 DMA，单帧缓冲） */
void            Cam_CaptureStop(void);             /* 停止采集并释放 DMA */
uint32_t        Cam_GetFrameCount(void);           /* 已捕获完整帧计数 */
CamErr_t        Cam_GetLastError(void);            /* 最近一次错误码 */
const uint8_t * Cam_GetFrame(void);                /* 失效 cache 后返回帧缓冲首地址（280x240 RGB565） */

#endif /* __CAM_DCMI_H */
