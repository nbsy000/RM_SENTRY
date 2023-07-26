#include "main.h"
#include "task_StateLED.h"

/**
 *  @brief  иав╢л╛╣ф
 *  @param  нч
 *  @retval нч
 */
void task_StateLED(void)
{
    while(1)
    {  
        BLUE_light_toggle();
        vTaskDelay(500);         
        RED_light_toggle();
        vTaskDelay(500);
    }
}
