#ifndef __UART_TASK_H
#define __UART_TASK_H

#include "main.h"

/* 函数接口 */
void Uart_Proc(void *argument);                     /* RX 任务：等规范化消息 → 快照 → 交 Uart_OnLine */
void Uart_OnLine(const char *line, uint16_t len);   /* 消息处理回调（弱定义，业务层可重写） */

#endif /* __UART_TASK_H */
