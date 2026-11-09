#include "uart_task.h"
#include "usart.h"

#include <string.h>

/* 消息处理回调：默认镜像回显，业务层可重写为自己的命令解析
 * line: 标准化消息（无结束符、'\0' 结尾）; len: 有效长度（不含 '\0'） */
__weak void Uart_OnLine(const char *line, uint16_t len)
{
    (void)len;
    Uart_Printf("[Rx] %s\r\n", line);
}

/* RX 任务：阻塞等规范化消息 → 快照到栈缓冲 → 交 Uart_OnLine 处理
 * 数据规范化（剥结束符/补 '\0'/计长度）已在接收 ISR 完成 */
void Uart_Proc(void *argument)
{
    (void)argument;

    Uart_Init();

    for (;;)
    {
        uint16_t len;
        uint8_t *buf = Uart_RxWaitFrame(&len);

        /* 快照到栈缓冲：回调期间到达的新帧不会破坏本条消息 */
        char line[UART_RX_BUF_MAX];
        memcpy(line, buf, (size_t)len + 1U);

        Uart_OnLine(line, len);
    }
}
