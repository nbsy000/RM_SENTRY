#include "main.h"
#include "HW_FrictionWheel.h"

/**
  * @brief  设置摩擦轮转速（油门值）
  * @param  两轮速度值
  * @retval None
  */
int16_t accelerator0;
int16_t accelerator1;
//extern uint16_t throttle;

//摩擦轮上电初始化
void FrictionWheel_Init(void)
{
  delay_ms(600); //随便延时一下
//  FrictionWheel_SetSpeed(400, 400);
  delay_ms(1000);
//  FrictionWheel_SetSpeed(600, 600);

  //写到ARR里面的是1000，占空比就是 pulse/ARR * 100%
}



