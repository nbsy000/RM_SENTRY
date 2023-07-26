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

//改变占空比，设置摩擦轮转速
int16_t Left_realSpeed;
int16_t Right_realSpeed;
void FrictionWheel_SetSpeed(int16_t tmpAccelerator0, int16_t tmpAccelerator1)
{
  //限幅
//  accelerator0 = LIMIT_MAX_MIN(tmpAccelerator0, 900, 400);
//  accelerator1 = LIMIT_MAX_MIN(tmpAccelerator1, 900, 400);
    accelerator0 = LIMIT_MAX_MIN(tmpAccelerator0, 15050, 0);
    accelerator1 = LIMIT_MAX_MIN(tmpAccelerator1, 15050, 0);

  //赋值
  //2006的话，就算PID把电流值用CAN发过去
  extern _2006_motor_t FrictionMotor[2];
  FrictionMotor[0].pid_speed.SetPoint = accelerator0;
  FrictionMotor[1].pid_speed.SetPoint = -accelerator1;  ////注意！！！！这个2006的转向是反着的！！！！

  float fsend;
  Left_realSpeed = FrictionMotor[0].RealSpeed;
	Right_realSpeed =  FrictionMotor[1].RealSpeed;
  fsend = PID_Calc(&FrictionMotor[0].pid_speed, FrictionMotor[0].RealSpeed, 0); //用速度环算电流值输出
  FrictionMotor[0].I_Set = (int16_t)LIMIT_MAX_MIN(fsend, FrictionCurrentLimit, -FrictionCurrentLimit);
  fsend = PID_Calc(&FrictionMotor[1].pid_speed, FrictionMotor[1].RealSpeed, 0); //用速度环算电流值输出
  FrictionMotor[1].I_Set = (int16_t)LIMIT_MAX_MIN(fsend, FrictionCurrentLimit, -FrictionCurrentLimit);

//  Friction_Can1Send(FrictionMotor[0].I_Set, FrictionMotor[1].I_Set); //发拨弹电流
}

