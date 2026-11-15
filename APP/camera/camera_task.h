#ifndef __CAMERA_TASK_H
#define __CAMERA_TASK_H

#include "main.h"

/* 函数接口 */
void Camera_Init(void);                 /* 创建采集任务（需在 osKernelStart 之前调用） */
void Camera_Proc(void *argument);       /* 采集任务主体 */

#endif /* __CAMERA_TASK_H */
