#ifndef __LED_H
#define __LED_H

#include "main.h"

/* LED定义宏 */
#define LED_PORT             GPIOC
#define LED_PIN              GPIO_PIN_13
#define LED_OFF_LEVEL        GPIO_PIN_SET
#define LED_ON_LEVEL         GPIO_PIN_RESET


/* LED函数声明 */
void Led_On();         /* 点亮指定LED */
void Led_Off();        /* 熄灭指定LED */
void Led_Toggle();     /* 反转指定LED */

#endif /* __LED_H */
