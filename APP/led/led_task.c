#include "led_task.h"
#include "led.h"

void Led_Disp(void *argument)
{
    for(;;)
    {
        Led_On();
        osDelay(500);
        Led_Off();
        osDelay(500);
    }
}