#ifndef __CAM_DCMI_H
#define __CAM_DCMI_H

#include "main.h"

/* ========== 尺寸参数 ==========
 * 传感器固定输出 320x240（4:3，不随屏幕变化），DCMI 不裁剪，
 * 帧缓冲为 320x240 RGB565；显示端按屏幕宽度自行裁切 */
#define CAM_SENSOR_WIDTH        320U
#define CAM_SENSOR_HEIGHT       240U
#define CAM_OUT_WIDTH           320U
#define CAM_OUT_HEIGHT          240U

/* RGB565：2 字节/像素，单帧 153600 字节 */
#define CAM_FRAME_BYTES         (CAM_OUT_WIDTH * CAM_OUT_HEIGHT * 2U)   /* 153600 */
#define CAM_FRAME_WORDS         (CAM_FRAME_BYTES / 4U)                  /* 38400，单块 < NDTR 上限 65535 */

/* 双缓冲：两块连续等长供 HAL DBM 使用，总长 76800 words > 65535 触发多缓冲分支 */
#define CAM_FRAME_BLOCK_COUNT   2U
#define CAM_FRAME_TOTAL_WORDS   (CAM_FRAME_WORDS * CAM_FRAME_BLOCK_COUNT)

/* 错误码定义 */
typedef enum {
    CAM_OK = 0,           /* 成功 */
    CAM_ERR_STATE,        /* 外设状态错误（启动失败/同步错误/溢出） */
} CamErr_t;

/* 帧完成回调类型（在 DCMI 中断上下文执行，实现必须非阻塞） */
typedef void (*CamFrameFn)(void);

/* 函数接口 */
CamErr_t        Cam_CaptureStart(CamFrameFn cb);   /* 启动双缓冲 DBM 连续采集（回调在中断上下文） */
uint32_t        Cam_GetFrameCount(void);           /* 已捕获完整帧计数 */
CamErr_t        Cam_GetLastError(void);            /* 最近一次错误码 */
const uint8_t * Cam_GetFrame(void);                /* 失效 cache 后返回最近完成块首地址（320x240 RGB565） */

#endif /* __CAM_DCMI_H */
