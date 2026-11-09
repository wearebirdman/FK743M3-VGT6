#include "led.h"

/* LED开启函数 */
void Led_On(void)
{
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, LED_ON_LEVEL);
}

/* LED关闭函数 */
void Led_Off(void)
{
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, LED_OFF_LEVEL);
}

/* LED反转函数 */
void Led_Toggle(void)
{
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
}
