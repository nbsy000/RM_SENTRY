#include "main.h"

uint8_t ChassisYaw_Last_State = ChassisYaw_SLEEP;
uint8_t ChassisYaw_Updata_Flag = 0;

uint8_t aim_flag = 0;

short C_I;
void ChassisYaw_task(void *pvParameters)
{
	while (1) 
	{
			//状态更新
			ChassisYaw_Updata_Flag = (ChassisYaw_Last_State != Sentry_State.ChassisYaw_Mode);		
		
			//根据大Yaw轴的模式选择
			if (Sentry_State.ChassisYaw_Mode == ChassisYaw_PC) ChassisYaw_PC_Act();
			else if (Sentry_State.ChassisYaw_Mode == ChassisYaw_RC) ChassisYaw_RC_Act();
			else if (Sentry_State.ChassisYaw_Mode == ChassisYaw_STOP) ChassisYaw_STOP_Act();
			else if  (Sentry_State.ChassisYaw_Mode == ChassisYaw_DEBUG) ChassisYaw_DEBUG_Act();
			else ChassisYaw_SLEEP_Act(); 
			
			//发送电流值进行控制
			Yaw_I_Control(C_I);
	 
			//读取当前状态
			ChassisYaw_Last_State = Sentry_State.ChassisYaw_Mode;
				
			vTaskDelay(2);		 
	}
}

/*********************************************************************************************************
*函 数 名: ChassisYaw_PC_Act
*功能说明: 自动控制大Yaw轴的运动
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ChassisYaw_PC_Act()
{
	  if (ChassisYaw_Last_State != ChassisYaw_PC) //应该是用来防止模式切换的时候运动抽风的
    {
			
        Motor_9025.PCYaw = Gyro_ChassisYaw.YAW_ABS;//使PID参数都是正的
    }
		
//		if((chassis.PC_State==OUTPOST)||(chassis.PC_State==PATROL))
//		{
			if(Motor_9025.aim_flag == SYNE_NONE)//没有识别到目标
				Motor_9025.PCYaw = Motor_9025.PCYaw + Motor_9025.SYNEYaw;
			else//识别到目标
				Motor_9025.PCYaw = Gyro_ChassisYaw.YAW_ABS + Motor_9025.SYNEYaw;
//		}
//		
//		else if((chassis.PC_State==TOPATH1)||(chassis.PC_State==BACKPATH1))
//		{}
//		else
//				Motor_9025.PCYaw = Gyro_ChassisYaw.YAW_ABS;
		
		Motor_9025.PidPos.SetPoint = Motor_9025.PCYaw;
    //大Yaw限幅+赋值 限幅在PID中有
    Motor_9025.PidSpeed.SetPoint = PID_Calc(&Motor_9025.PidPos, Gyro_ChassisYaw.YAW_ABS);//目前用陀螺仪

    C_I = PID_Calc(&Motor_9025.PidSpeed, Gyro_ChassisYaw.GZ);//使用陀螺仪Yaw轴数据,这个还需确定，看用不用反馈的Speed
			
}

/*********************************************************************************************************
*函 数 名: ChassisYaw_RC_Act
*功能说明: 遥控器控制大Yaw轴的运动
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ChassisYaw_RC_Act(void)
{
    if (ChassisYaw_Last_State != ChassisYaw_RC) //应该是用来防止模式切换的时候运动抽风的
    {
        Motor_9025.PidPos.SetPoint = Gyro_ChassisYaw.YAW_ABS;//取符号主要是陀螺仪与电机角是反向的，这么取可以使PID参数都是正的
    }
		
		Motor_9025.PidPos.SetPoint += 0.0003 * (RC_Ctl.rc.ch2 - 1024);//设置每秒最快66度，调试设置慢一点
		
		float fsend;
			  
    //大Yaw限幅+赋值 限幅在PID中有
    Motor_9025.PidSpeed.SetPoint = PID_Calc(&Motor_9025.PidPos, Gyro_ChassisYaw.YAW_ABS);//取负数是因为陀螺仪与电机角相反
    fsend = PID_Calc(&Motor_9025.PidSpeed, Gyro_ChassisYaw.GZ);//使用陀螺仪Yaw轴数据
		C_I = LIMIT_MAX_MIN(fsend,2000,-2000);

}

/*********************************************************************************************************
*函 数 名: ChassisYaw_DEBUG_Act
*功能说明: 遥控器控制大Yaw轴的运动
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
float test_chassisyaw_angle = 0;
uint8_t CY_DIAABLE = 0;
uint8_t now_place_flag = 0;
uint8_t set_speed_flag = 0;
float Speed_Yaw = 0;
void ChassisYaw_DEBUG_Act(void)
{
		float fsend;
    if (ChassisYaw_Last_State != ChassisYaw_DEBUG) //应该是用来防止模式切换的时候运动抽风的
    {
        Motor_9025.PidPos.SetPoint = Gyro_ChassisYaw.YAW_ABS;//取符号主要是陀螺仪与电机角是正的，这么取可以使PID参数都是正的
    }
		
		if(now_place_flag == 1)
			Motor_9025.PidPos.SetPoint = Gyro_ChassisYaw.YAW_ABS;
		else if(now_place_flag == 2)
			Motor_9025.PidPos.SetPoint += 0.0005 * (RC_Ctl.rc.ch2 - 1024);//设置每秒最快660度，调试设置慢一点
		
    //大Yaw限幅+赋值 限幅在PID中有
    Motor_9025.PidSpeed.SetPoint = PID_Calc(&Motor_9025.PidPos, Gyro_ChassisYaw.YAW_ABS);//目前先用电机角
		if(set_speed_flag)
			Motor_9025.PidSpeed.SetPoint = Speed_Yaw;
    fsend = PID_Calc(&Motor_9025.PidSpeed, Gyro_ChassisYaw.GZ)*CY_DIAABLE;//使用陀螺仪Yaw轴数据,这个还需确定，看用不用反馈的Speed
		C_I = LowPass(fsend,C_I,0.1);//低通滤波
		
}

/*********************************************************************************************************
*函 数 名: ChassisYaw_SLEEP_Act
*功能说明: 大Yaw轴停止不动
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ChassisYaw_STOP_Act(void)
{
	  if (ChassisYaw_Last_State != ChassisYaw_STOP) //应该是用来防止模式切换的时候运动抽风的
    {
        Motor_9025.PidPos.SetPoint = Gyro_ChassisYaw.YAW_ABS;//取符号主要是陀螺仪与电机角是反向的，这么取可以使PID参数都是正的
    }
//		C_I = 0;//发送电流值进行控制
}

/*********************************************************************************************************
*函 数 名: ChassisYaw_SLEEP_Act
*功能说明: 大Yaw轴掉电
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ChassisYaw_SLEEP_Act(void)
{
		 C_I = 0;//发送电流值进行控制
}


/**
  * @brief  大YawPID以及相关参数初始化
  * @param  None
  * @retval None
  */
void ChassisYaw_Init()
{
	
	//初始化大Yaw轴得PID
	Motor_9025.PidPos.P = 5.0f;
	Motor_9025.PidPos.I = 0.0f;
	Motor_9025.PidPos.D = 0.0f;
	Motor_9025.PidPos.IMax = 0.0f;
	Motor_9025.PidPos.SetPoint = 0.0f;
	Motor_9025.PidPos.OutMax = 110.0f;//目前这个速度不会有异响

	Motor_9025.PidSpeed.P = 13.0f;
	Motor_9025.PidSpeed.I = 0.31f;
	Motor_9025.PidSpeed.D = 0.0f;                
	Motor_9025.PidSpeed.IMax = 250.0f;  
	Motor_9025.PidSpeed.SetPoint =  0.0f;
	Motor_9025.PidSpeed.OutMax = 2000.0f;//电流输出值的限定
	Motor_9025.PidSpeed.I_Sep = I_SEP;
	Motor_9025.PidSpeed.I_U = 28.0f;
	Motor_9025.PidSpeed.I_L = 5.0f;
	Motor_9025.PidSpeed.RC_DF = 0.1f;
	
	
	//相关参数初始化
	Motor_9025.Yaw_init = 9540;
}
