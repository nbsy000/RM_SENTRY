#include "main.h"
#include "task_ZeroCheck.h"

extern gyro_Typedef Gyro_Right,Gyro_Left,Gyro_ChassisYaw;
extern gimbal_motor_t MotoPitch, MotoYaw,MotoPitch_L, MotoYaw_L;
extern _2006_motor_t BodanMotor;
extern int32_t dt;   
static float ZeroCheck(ZeroCheck_Typedef *Zero,float value,int16_t Zerocheck_mode);

/**
  * @brief  过零检测任务
  * @param  None
  * @retval None
  */
void task_ZeroCheck(void)
{
    while (1)
    {
        MotoPitch.Angle_Inc = ZeroCheck(&MotoPitch.zerocheck, MotoPitch.Angle_ABS, Position);        //云台PITCH电机
        MotoYaw.Angle_Inc = ZeroCheck(&MotoYaw.zerocheck, MotoYaw.Angle_ABS, Position);              //云台YAW电机
        MotoPitch_L.Angle_Inc = ZeroCheck(&MotoPitch_L.zerocheck, MotoPitch_L.Angle_ABS, Position);        //云台PITCH电机
        MotoYaw_L.Angle_Inc = ZeroCheck(&MotoYaw_L.zerocheck, MotoYaw_L.Angle_ABS, Position);              //云台YAW电机
        BodanMotor.Angle_Inc = ZeroCheck(&BodanMotor.zerocheck, BodanMotor.Angle_ABS, Position);     //拨弹电机
				Gyro_Left.GZ = ZeroCheck(&Gyro_Left.zerocheck_yaw,Gyro_Left.YAW_ABS,Speed)/dt; 
				Gyro_Left.GY = ZeroCheck(&Gyro_Left.zerocheck_yaw,Gyro_Left.PITCH,Speed)/dt; 
	
        vTaskDelay(1);
    }
}

inline float PitchAngleOutPut(uint8_t Gimbal_Type)
{
	if(Gimbal_Type == GIMBAL_RIGHT)
    return MotoPitch.Angle_Inc;
	else 
		return MotoPitch_L.Angle_Inc;
}

inline float YawAngleOutPut(uint8_t Gimbal_Type)
{
	if(Gimbal_Type == GIMBAL_RIGHT)
    return MotoYaw.Angle_Inc;
	else
		return MotoYaw_L.Angle_Inc;
}

inline float GyroPitchOutPut(uint8_t Gimbal_Type)
{
	if(Gimbal_Type == GIMBAL_RIGHT)
    return Gyro_Right.PITCH_INC;
	else
		return Gyro_Left.PITCH_INC;
}

inline float GyroYawOutPut(uint8_t Gimbal_Type)
{
	if(Gimbal_Type == GIMBAL_RIGHT)
    return Gyro_Right.YAW_INC;
	else
		return Gyro_Left.YAW_INC;
}


/**
  * @brief  位置式和速度式过零检测
             Zero->ActualValue 表示检测量当前值
             Zero->LastValue 表示检测量上一次值
             Zero->CountCycle 表示检测量过零时越变值，即计数周期
             Zero->PreError 表示检测量差值
             使用此函数前要声明明对应检测量结构体的 Zero->CountCycle与Zero->LastValue
  * @param  ZeroCheck_Typedef *Zero  过零检测结构体
  *         float value  待检测量
            short Zerocheck_mode：取值Position或Speed
  * @retval 取决于Zerocheck_mode，分别输出过零检测后位置值或速度值
  */
static float ZeroCheck(ZeroCheck_Typedef *Zero, float value, int16_t Zerocheck_mode)
{
    Zero->ActualValue = value;

    Zero->PreError = Zero->ActualValue - Zero->LastValue;
    Zero->LastValue = Zero->ActualValue;

    if (Zero->PreError > 0.6f * Zero->CountCycle)
    {
        Zero->PreError = Zero->PreError - Zero->CountCycle;
        Zero->Circle--;
    }
    if (Zero->PreError < -0.6f * Zero->CountCycle)
    {
        Zero->PreError = Zero->PreError + Zero->CountCycle;
        Zero->Circle++;
    }

    if (Zerocheck_mode == Position)
        return Zero->ActualValue + Zero->Circle * Zero->CountCycle;
    else if (Zerocheck_mode == Speed)
        return Zero->PreError;
    else
        return 0;
}
