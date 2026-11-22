#ifndef __VISION_TASK_H
#define __VISION_TASK_H

#include "main.h"

/* 调试开关：1 = 屏幕叠加二值掩码（目视调 HSV 阈值），0 = 叠加识别外接框 */
#define VISION_DEBUG_MASK   0U

/* 函数接口 */
void Vision_Init(void);                             /* 创建识别任务（需在 osKernelStart 之前调用） */
void Vision_Proc(void *argument);                   /* 识别任务主体 */
void Vision_PushFrame(const uint16_t *frame);       /* 采集任务每帧调用：生成掩码并唤醒识别任务 */
void Vision_DrawOverlay(void);                      /* 采集任务推屏后调用：叠加最近一次识别结果 */

#endif /* __VISION_TASK_H */
