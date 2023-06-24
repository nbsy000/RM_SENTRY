#include "main.h"
#include "task_GimbalR.h"

gimbal_motor_t MotoPitch, MotoYaw,MotoPitch_L,MotoYaw_L;
float RCPitch_Scal = 0.00015f, RCYaw_Scal = 0.00016f; //遥控器对两轴控制的放大比率
float patrol_step_pitch = 0.030f/*0.035f */, patrol_step_yaw =0.03f;// 0.005f;                    //巡逻设定值更新
Gimbal_Typedef Gimbal_R,Gimbal_L;

extern int16_t Gimbal_init_flag;
extern gyro_Typedef Gyro_Right,Gyro_Left,Gyro_ChassisYaw;
extern State_t Sentry_State;
extern uint8_t aim_flag;
extern uint8_t distance;
extern uint8_t is_game_start;
extern uint8_t armor_flag;

uint8_t debug_yaw = 0;
uint8_t debug_pitch = 0;

float ChassisYaw_Inc = 0.0f;//大Yaw轴增量
int ChassisYaw_Cnt = 0;
/**
 * @brief 云台任务主体
 * @param 无
 * @retval 无
 */

void task_GimbalR(void *parameter)
{
    //进入任务时执行一次复位
    PID_Gimbal_Init(); //第一次执行云台任务是 给云台的PID参数进行初始化
    Gimbal_Limit_Init();    //限幅校正
		Gimbal_Init();//初始化
		
    while (1)
    {
			//云台姿态结算
			 Gimbal_GYRO_Cal();
			
			Aim_Disconnect_Act();
			
			//根据辅瞄结果来判断云台位值
			if((Gimbal_R.armor_state==ARMOR_AIMED)&&(Gimbal_L.armor_state==ARMOR_AIMED))			//俩云台都识别到
				armor_flag = SYNE_BOTH;
			else if((Gimbal_R.armor_state==ARMOR_AIMED)&&(Gimbal_L.armor_state==ARMOR_NO_AIM))			//右云台识别到
				armor_flag = SYNE_RIGHT;
			else if((Gimbal_R.armor_state==ARMOR_NO_AIM)&&(Gimbal_L.armor_state==ARMOR_AIMED))			//左云台识别到
				armor_flag = SYNE_LEFT;				
			else//俩云台正常巡航
				armor_flag = SYNE_NONE;
			
			extern float aim_pitch,aim_yaw;
			extern float aim_pitch_l,aim_yaw_l;
			Gimbal_R.aim_Pitch = aim_pitch;
			Gimbal_R.aim_Yaw = aim_yaw;
			
	 		Gimbal_L.aim_Pitch = aim_pitch_l;
			Gimbal_L.aim_Yaw = aim_yaw_l;			
			//主云台
			{
        Gimbal_R.ModeUpdate_Flag = (Gimbal_R.LastMode != Sentry_State.Gimbal_R_Mode); //判断模式是否更换
        Gimbal_R.LastMode = Sentry_State.Gimbal_R_Mode;                               //更新上次状态

				//获取姿态
        MotoYaw.actualAngle = CombYawOutput(GIMBAL_RIGHT);
        MotoPitch.actualAngle = CombPitchOutput(GIMBAL_RIGHT);
			
        if (Sentry_State.Gimbal_R_Mode == Gimbal_R_RC)      
						Gimbal_RC_Act(&Gimbal_R);				
        else if (Sentry_State.Gimbal_R_Mode == Gimbal_R_DEBUG) 
						Gimbal_DEBUG_Act(&Gimbal_R);
				else if (Sentry_State.Gimbal_R_Mode == Gimbal_R_AIM)
						Gimbal_AIM_Act(&Gimbal_R);
				else 
						Gimbal_SLEEP_Act(&Gimbal_R);
			}
			//左云台
			{
					Gimbal_L.ModeUpdate_Flag = (Gimbal_L.LastMode != Sentry_State.Gimbal_L_Mode); //判断模式是否更换
					Gimbal_L.LastMode = Sentry_State.Gimbal_L_Mode;                               //更新上次状态
				
					//获取姿态
					MotoYaw_L.actualAngle = CombYawOutput(GIMBAL_LEFT);
					MotoPitch_L.actualAngle = CombPitchOutput(GIMBAL_LEFT);
				
					if (Sentry_State.Gimbal_L_Mode == Gimbal_L_RC)      
							Gimbal_RC_Act(&Gimbal_L);
					else if (Sentry_State.Gimbal_L_Mode == Gimbal_L_DEBUG) 
							Gimbal_DEBUG_Act(&Gimbal_L);
					else if (Sentry_State.Gimbal_L_Mode == Gimbal_L_AIM)
							Gimbal_AIM_Act(&Gimbal_L);	
					else 
							Gimbal_SLEEP_Act(&Gimbal_L);					
			}		
			
			//目前PC模式是一起的
			if((Sentry_State.Gimbal_L_Mode == Gimbal_L_PC)&&(Sentry_State.Gimbal_R_Mode == Gimbal_R_PC))
			{	
					Gimbal_PC_Act(NO_SYNE);//目前是没有协同的
			}		
			
			PitchYaw_Can2Send(MotoPitch.I_Set,MotoYaw.I_Set,MotoPitch_L.I_Set,MotoYaw_L.I_Set);//发送电流值
			
			Gimbal_Syne_Send(ChassisYaw_Inc);
			
       vTaskDelay(1);
    }
}


float Test_Pitch_Pos = 0, Test_Yaw_Pos = 0;
void Gimbal_DEBUG_Act(Gimbal_Typedef *Gimbal)
{
		gimbal_motor_t *MotoPitch  = Gimbal->Pitch;
		gimbal_motor_t *MotoYaw = Gimbal->Yaw;	

    MotoYaw->PidPos.SetPoint = Test_Yaw_Pos;                            
    MotoPitch->PidPos.SetPoint = Test_Pitch_Pos;        
	
		Gimbal_PID_Cal(Gimbal,GYRO,DEBUG_PID);
}

/**
  * @brief  遥控器模式下，更新上云台工作状态
  * @param  遥控器值
  * @retval None
  */
void Gimbal_RC_Act(Gimbal_Typedef *Gimbal)
{
    extern RC_Ctl_t RC_Ctl;
		gimbal_motor_t *MotoPitch  = Gimbal->Pitch;
		gimbal_motor_t *MotoYaw = Gimbal->Yaw;	

    if (Gimbal->ModeUpdate_Flag) //应该是用来防止模式切换的时候运动抽风的
    {
        MotoYaw->PidPos.SetPoint = MotoYaw->actualAngle;
        MotoPitch->PidPos.SetPoint = MotoPitch->actualAngle;
    }
    else
    { //如果模式没有发生变化，则正常执行遥控器赋值的部分
        MotoYaw->PidPos.SetPoint += RCYaw_Scal * (RC_Ctl.rc.ch0 - 1024);
        MotoPitch->PidPos.SetPoint -= RCPitch_Scal * (RC_Ctl.rc.ch1 - 1024);//-号与云台转向有关，现在的云台是左大右小，与遥控器相反
    }
 
		Gimbal_PID_Cal(Gimbal,GYRO,NO_DEBUG_PID);
}


/**
  * @brief  PC模式下，更新上云台工作状态
  * @param  None
  * @retval None
  */
uint8_t armor_flag = 0; //总的扫描状态
uint8_t last_armor_flag = 0;//上一颗
float Last_Pitch,Last_Yaw;
float Last_Pitch_L,Last_Yaw_L;
float K_AIM_pitch = 0.15;
float K_AIM_yaw = 0.05;
void Gimbal_PC_Act(uint8_t Syne_Flag)
{
    if (Gimbal_L.ModeUpdate_Flag)//只要一个云台更新就够
    {
        MotoYaw.PCsetAngle = MotoYaw.actualAngle;
        MotoPitch.PCsetAngle = MotoPitch.actualAngle;
			
        MotoYaw_L.PCsetAngle = MotoYaw_L.actualAngle;
        MotoPitch_L.PCsetAngle = MotoPitch_L.actualAngle;
			
				armor_flag = SYNE_NONE;
				last_armor_flag = SYNE_NONE;
    }


			if(armor_flag == SYNE_NONE)//识别到目标
				Gimbal_Attack_None();
			else//识别到目标
			{
					ChassisYaw_Cnt = 500;//识别到后原地停止0.5s
					if(Syne_Flag == SYNE)//开启协同
					{
							switch(armor_flag)
							{
								case SYNE_LEFT:
									Gimbal_Attack_Left();	
									break;
								case SYNE_RIGHT:
									Gimbal_Attack_Right();
									break;	
								case SYNE_BOTH:
									Gimbal_Attack_Both();
									break;														
								default:
									Gimbal_Attack_Nosyne();
									break;
							}				
					}
					else//没开启协同
						Gimbal_Attack_Nosyne();			
			}
			
		MotoPitch.PCsetAngle = K_AIM_pitch*MotoPitch.PCsetAngle + (1-K_AIM_pitch)*Last_Pitch;
		MotoYaw.PCsetAngle = K_AIM_yaw*MotoYaw.PCsetAngle + (1-K_AIM_yaw)*Last_Yaw;
			
		MotoPitch.PidPos.SetPoint = MotoPitch.PCsetAngle;
		MotoYaw.PidPos.SetPoint = MotoYaw.PCsetAngle;	
			
		Last_Pitch = MotoPitch.PCsetAngle;
		Last_Yaw = MotoYaw.PCsetAngle;	
			
		MotoPitch_L.PCsetAngle = K_AIM_pitch*MotoPitch_L.PCsetAngle + (1-K_AIM_pitch)*Last_Pitch_L;
		MotoYaw_L.PCsetAngle = K_AIM_yaw*MotoYaw_L.PCsetAngle + (1-K_AIM_yaw)*Last_Yaw_L;
			
		MotoPitch_L.PidPos.SetPoint = MotoPitch_L.PCsetAngle;
		MotoYaw_L.PidPos.SetPoint = MotoYaw_L.PCsetAngle;
		
		Last_Pitch_L = MotoPitch_L.PCsetAngle;
		Last_Yaw_L = MotoYaw_L.PCsetAngle;
				
		Gimbal_PID_Cal(&Gimbal_R,GYRO,NO_DEBUG_PID);
		Gimbal_PID_Cal(&Gimbal_L,GYRO,NO_DEBUG_PID);
		
		
		//云台卸力
//		if(Gimbal_R.armor_state==ARMOR_AIMED)
//			Gimbal_SLEEP_Act(&Gimbal_L);
//		else if(Gimbal_L.armor_state==ARMOR_AIMED)
//			Gimbal_SLEEP_Act(&Gimbal_R);
}

/**
  * @brief  云台辅瞄 为了减小云台转动时对大Yaw轴的影响，两者巡航的时候需要朝向相同
  * @param  None
  * @retval None
  */
void Gimbal_PC_Act2()
{
    if (Gimbal_L.ModeUpdate_Flag)//只要一个云台更新就够 ，从0值开始好一些
    {
				Gimbal_R.patrol_dir_yaw = 1;
				Gimbal_R.patrol_dir_pitch = 0;
				MotoPitch.PCsetAngle = 0;
				MotoYaw.PCsetAngle = 0;
			
				Gimbal_L.patrol_dir_yaw = 0;
				Gimbal_L.patrol_dir_pitch = 0;
				MotoPitch_L.PCsetAngle = 0;
				MotoYaw_L.PCsetAngle = 0;
    }	

		if(Gimbal_R.armor_state==ARMOR_AIMED)
		{
				extern float aim_yaw, aim_pitch;
				MotoPitch.PCsetAngle = aim_pitch;
				MotoYaw.PCsetAngle = aim_yaw;				
		}
		else 
				Gimbal_Cruise(&Gimbal_R,PITCH_YAW);			
		
		//左
		if(Gimbal_L.armor_state==ARMOR_AIMED)
		{
			extern float aim_yaw_l, aim_pitch_l;
			MotoPitch_L.PCsetAngle = aim_pitch_l;
			MotoYaw_L.PCsetAngle = aim_yaw_l;	
		}
		else
				Gimbal_Cruise(&Gimbal_L,PITCH_YAW);

		MotoPitch_L.PidPos.SetPoint = MotoPitch_L.PCsetAngle;
		MotoYaw_L.PidPos.SetPoint = MotoYaw_L.PCsetAngle;
		
		Gimbal_PID_Cal(&Gimbal_R,GYRO,NO_DEBUG_PID);
		Gimbal_PID_Cal(&Gimbal_L,GYRO,NO_DEBUG_PID);
}

/**
  * @brief  云台掉电模式
  * @param  None
  * @retval None
  */
void Gimbal_SLEEP_Act(Gimbal_Typedef *Gimbal)
{
		Gimbal->Pitch->I_Set = 0;
		Gimbal->Yaw->I_Set = 0;
}

/**
  * @brief  云台辅瞄测试模式
  * @param  None
  * @retval None
  */
void Gimbal_AIM_Act(Gimbal_Typedef *Gimbal)
{
		gimbal_motor_t *MotoPitch = Gimbal->Pitch;
		gimbal_motor_t *MotoYaw = Gimbal->Yaw;		
	
    if (Gimbal->ModeUpdate_Flag)
    {
        MotoPitch->PCsetAngle = MotoPitch->actualAngle;
        MotoYaw->PCsetAngle = MotoYaw->actualAngle;	
    }	
		
		if(Gimbal->armor_state==ARMOR_AIMED)//识别到目标
		{
				MotoPitch->PCsetAngle = Gimbal->aim_Pitch;
				MotoYaw->PCsetAngle = Gimbal->aim_Yaw;	
		}			
		else
		{
        MotoPitch->PCsetAngle = MotoPitch->actualAngle;
        MotoYaw->PCsetAngle = MotoYaw->actualAngle;	
		}	
		
		MotoPitch->PidPos.SetPoint = MotoPitch->PCsetAngle;
    MotoYaw->PidPos.SetPoint = MotoYaw->PCsetAngle;
		Gimbal_PID_Cal(Gimbal,GYRO,NO_DEBUG_PID);	
}


/**
  * @brief  云台的协同进攻,俩云台都识别到目标
  * @param  None
  * @retval None
  */
void Gimbal_Attack_Both()
{
	 //左云台跟着辅瞄
	MotoPitch_L.PCsetAngle = Gimbal_L.aim_Pitch;
	MotoYaw_L.PCsetAngle = Gimbal_L.aim_Yaw;	
	
	 //右云台跟着辅瞄
	MotoPitch.PCsetAngle = Gimbal_R.aim_Pitch;
	MotoYaw.PCsetAngle = Gimbal_R.aim_Yaw;	
	
	//大Yaw轴跟随
//	if(MotoYaw.actualAngle > -39.0f)
//			ChassisYaw_Inc = 38 + MotoYaw.actualAngle;//计算大Yaw的实际转角
//	else if(MotoYaw_L.actualAngle < 38.0f)	
//			ChassisYaw_Inc = -40 + MotoYaw_L.actualAngle;//计算大Yaw的实际转角
	
	ChassisYaw_Inc = (MotoYaw_L.PCsetAngle + MotoYaw.PCsetAngle)/2.0; 
}


/**
  * @brief  云台的协同进攻,左云台识别到目标
  * @param  None
  * @retval None
  */
void Gimbal_Attack_Left()
{
	 //左云台跟着辅瞄
		MotoPitch_L.PCsetAngle = Gimbal_L.aim_Pitch;
		MotoYaw_L.PCsetAngle = Gimbal_L.aim_Yaw;	
	
		//大Yaw向左云台靠近
		ChassisYaw_Inc = -41 + MotoYaw_L.actualAngle;//计算大Yaw的实际转角
	
		//右云台靠近左云台 Pitch轴巡航，Yaw轴过去
		Gimbal_Cruise(&Gimbal_R,ONLY_PITCH);	
		MotoYaw.PCsetAngle = -41.0f;	
}

/**
  * @brief  云台的协同进攻,右云台识别到目标
  * @param  None
  * @retval None
  */
void Gimbal_Attack_Right()
{
	 //右云台跟着辅瞄
		MotoPitch.PCsetAngle = Gimbal_R.aim_Pitch;
		MotoYaw.PCsetAngle = Gimbal_R.aim_Yaw;	
	
		//大Yaw向右云台靠近
		ChassisYaw_Inc = 41 + MotoYaw.actualAngle;//计算大Yaw的实际转角
	
		//左云台靠近右云台 Pitch轴巡航，Yaw轴过去
		Gimbal_Cruise(&Gimbal_L,ONLY_PITCH);	
		MotoYaw_L.PCsetAngle = 41.0f;			
}

/**
  * @brief  云台的巡航，俩云台均未识别到目标
  * @param  None
  * @retval None
  */
float Chassis_Yaw_Speed = 70.0f;
float K_Chassis_Yaw = 1.0f;
void Gimbal_Attack_None()
{
		//右
		Gimbal_Cruise(&Gimbal_R,ONLY_PITCH);	
		
		//左
		Gimbal_Cruise(&Gimbal_L,ONLY_PITCH);	
	
		//大Yaw轴(目前假设右云台活动范围为正，左云台活动范围为负)
		if(ChassisYaw_Cnt > 0)
		{
//			ChassisYaw_Inc = -Chassis_Yaw_Speed/500.0f*arm_cos_f32((Chassis_Yaw_Speed/(35.0f*K_Chassis_Yaw))*(float)ChassisYaw_Cnt/1000.0f);//最大速度为70，最大范围为2*35
			ChassisYaw_Cnt -- ;
		}
		else
			ChassisYaw_Inc = -0.12f;//80.0?
	
}

/**
  * @brief  云台的巡航，没有云台协同时各自打各自的
  * @param  None
  * @retval None
  */
int time_cnt = 0;
int time_cnt_l = 0;
void Gimbal_Attack_Nosyne()
{
		float mid_offset;//中心偏移
	
		if(Gimbal_R.armor_state==ARMOR_AIMED)//左云台
		{
				extern float aim_yaw, aim_pitch;
				MotoPitch.PCsetAngle = aim_pitch;
				MotoYaw.PCsetAngle = aim_yaw;	
				time_cnt = 500;
		}
		else 
		{
				if(time_cnt == 0)
					Gimbal_Cruise(&Gimbal_R,ONLY_PITCH);
				else
					time_cnt--;
		}			
		
		//左
		if(Gimbal_L.armor_state==ARMOR_AIMED)
		{
				extern float aim_yaw_l, aim_pitch_l;
				MotoPitch_L.PCsetAngle = aim_pitch_l;
				MotoYaw_L.PCsetAngle = aim_yaw_l;
				time_cnt_l = 500;
		}
		else
		{
				if(time_cnt_l == 0)
					Gimbal_Cruise(&Gimbal_L,ONLY_PITCH);
				else
					time_cnt_l--;
			}
		
//		//大Yaw轴
//		if((armor_flag == SYNE_BOTH)&(Gimbal_L.target_id == Gimbal_R.target_id))//
//		{
//			mid_offset = (MotoYaw_L.PCsetAngle + MotoYaw.PCsetAngle)/2.0; 
//			if(mid_offset > 3.5f)
//				ChassisYaw_Inc = mid_offset - 3.5f;
//			else if(mid_offset < -3.5f)
//				ChassisYaw_Inc = mid_offset + 3.5f;
//			else
//				ChassisYaw_Inc = 0.0f;//	
//		}
//		else
				ChassisYaw_Inc = 0.0f;//
}

/**
  * @brief  云台巡航
  * @param  云台类型，即左右云台
  * @param  巡航类型 pitch轴巡航或者俩轴巡航
  * @retval None
  */
void Gimbal_Cruise(Gimbal_Typedef *Gimbal,uint8_t Cruise_Mode)
{
		gimbal_motor_t *MotoPitch  = Gimbal->Pitch;
		gimbal_motor_t *MotoYaw = Gimbal->Yaw;
	
		float step_speed_pitch,step_speed_yaw;
	
		//计算pitch轴步长
		MotoPitch->PCsetAngle = LIMIT_MAX_MIN(MotoPitch->PCsetAngle,MotoPitch->PATROL_MAX_ANGLE-patrol_step_pitch,MotoPitch->PATROL_MIN_ANGLE+patrol_step_pitch);
		step_speed_pitch = patrol_step_pitch*sqrt(1 - pow(MotoPitch->PCsetAngle*2.0/abs(MotoPitch->PATROL_MAX_ANGLE-MotoPitch->PATROL_MIN_ANGLE),2));

		if ((MotoPitch->PCsetAngle + patrol_step_pitch >= MotoPitch->PATROL_MAX_ANGLE) && Gimbal->patrol_dir_pitch)
				Gimbal->patrol_dir_pitch = 0; //电机到上限反向
		if ((MotoPitch->PCsetAngle - patrol_step_pitch <= MotoPitch->PATROL_MIN_ANGLE) && !Gimbal->patrol_dir_pitch)
				Gimbal->patrol_dir_pitch = 1; //电机到下限反向
		
		MotoPitch->PCsetAngle += (Gimbal->patrol_dir_pitch == 1) ? (+step_speed_pitch) : (-step_speed_pitch);
		
		//俩轴都巡航
		if(Cruise_Mode == PITCH_YAW)
		{
			MotoYaw->PCsetAngle = LIMIT_MAX_MIN(MotoYaw->PCsetAngle,MotoYaw->PATROL_MAX_ANGLE-patrol_step_yaw,MotoYaw->PATROL_MIN_ANGLE+patrol_step_yaw);
			step_speed_yaw = patrol_step_yaw*sqrt(1 - pow(MotoYaw->PCsetAngle*2.0/abs(MotoYaw->PATROL_MAX_ANGLE-MotoYaw->PATROL_MIN_ANGLE),2));

			if ((MotoYaw->PCsetAngle + patrol_step_yaw >= MotoYaw->PATROL_MAX_ANGLE) && Gimbal->patrol_dir_yaw)
					Gimbal->patrol_dir_yaw = 0; //电机到上限反向
			if ((MotoYaw->PCsetAngle - patrol_step_yaw <= MotoYaw->PATROL_MIN_ANGLE) && !Gimbal->patrol_dir_yaw)
					Gimbal->patrol_dir_yaw = 1; //电机到下限反向
			
			MotoYaw->PCsetAngle += (Gimbal->patrol_dir_yaw == 1) ? (+step_speed_yaw) : (-step_speed_yaw);
		}
		else
			MotoYaw->PCsetAngle = MotoYaw->actualAngle;
			
}

/**
  * @brief  云台PID输出计算
  * @param  Gimbal 云台类型结构体指针
  * @param  Feedback_Type pid反馈类型，主要是速度是否为陀螺仪
  * @param  Is_Debug 是否是Debug模式
  * @retval None
  */
uint8_t speed_debug = 0;
float test_pitch_speed = 0.0f;
float test_yaw_speed = 0.0f;
float K_Speed_Pitch = 1.0f;
float A_Speed_Pitch = 5.0f;//
float K_Speed_Yaw = 1.0f;
float A_Speed_Yaw = 5.0f;//
int32_t count = 0;
uint8_t Fuzzy_Flag = 0;
void Gimbal_PID_Cal(Gimbal_Typedef *Gimbal,uint8_t Feedback_Type,uint8_t Is_Debug)
{
		gimbal_motor_t *MotoPitch  = Gimbal->Pitch;
		gimbal_motor_t *MotoYaw = Gimbal->Yaw;	

//		float fsendpitch,fsendyaw;//电流PID输出，这里需要设置一个中间变量，直接将结果赋值给I_set容易出现越界，
//		float yaw_speed,pitch_speed;
//		float Yaw_Speed_SetPoint,Pitch_Speed_SetPoint;
//		//位置环  
//    MotoPitch->PidPos.SetPoint = LIMIT_MAX_MIN(MotoPitch->PidPos.SetPoint, MotoPitch->MAX_ANGLE, MotoPitch->MIN_ANGLE);
//    Pitch_Speed_SetPoint = PID_Calc(&MotoPitch->PidPos, MotoPitch->actualAngle, 0) ;
//    MotoYaw->PidPos.SetPoint = LIMIT_MAX_MIN(MotoYaw->PidPos.SetPoint, MotoYaw->MAX_ANGLE, MotoYaw->MIN_ANGLE);
//    Yaw_Speed_SetPoint = PID_Calc(&MotoYaw->PidPos, MotoYaw->actualAngle, 0);
//	
//		if(speed_debug)//进行速度环调试 			//速度是cos，位置是sin，测试时放置0度
//		{
//				count++;
//				Pitch_Speed_SetPoint = A_Speed_Pitch*arm_cos_f32((A_Speed_Pitch/(MotoPitch->PATROL_MAX_ANGLE*K_Speed_Pitch))*(float)count/1000.0f);
//				Yaw_Speed_SetPoint = A_Speed_Yaw*arm_cos_f32((A_Speed_Yaw/(MotoYaw->PATROL_MAX_ANGLE*K_Speed_Yaw))*(float)count/1000.0f);			
//		}	
//		
//		//反馈选择
//		if(Feedback_Type == GYRO) //陀螺仪反馈
//		{
//				yaw_speed = MotoYaw->Gyro_Speed;
//				pitch_speed = MotoPitch->Gyro_Speed;			
//		}
//		else if(Feedback_Type == GYRO_PITCH)//Yaw轴用电机角速度，主要在丢失大Yaw之后用于切换
//		{
//				yaw_speed = MotoYaw->real_speed;
//				pitch_speed = MotoPitch->Gyro_Speed;					
//		}
//		else
//		{
//				yaw_speed = MotoYaw->real_speed;
//				pitch_speed = MotoPitch->real_speed;
//		}
//		
//		//速度环
//		if(Fuzzy_Flag)//模糊
//		{
//			MotoPitch->PidSpeedF.SetPoint = Pitch_Speed_SetPoint;
//			fsendpitch = FuzzyPID_Calc(&MotoPitch->PidSpeedF, pitch_speed) + ((MotoPitch->PidPos.SetPoint+MotoPitch->actualAngle)*MotoPitch->K1+MotoPitch->K2);
//		}

//		else//普通
//		{
//			MotoPitch->PidSpeed.SetPoint = Pitch_Speed_SetPoint;
//			fsendpitch = PID_Calc(&MotoPitch->PidSpeed, pitch_speed, 0) + ((MotoPitch->PidPos.SetPoint+MotoPitch->actualAngle)*MotoPitch->K1+MotoPitch->K2);
//		}

//		MotoYaw->PidSpeed.SetPoint = Yaw_Speed_SetPoint;
//		fsendyaw = PID_Calc(&MotoYaw->PidSpeed, yaw_speed, 0);	
//		fsendpitch = (int16_t)LIMIT_MAX_MIN(fsendpitch,LimitPitch,-LimitPitch);
//		fsendyaw = (int16_t)LIMIT_MAX_MIN(fsendyaw,LimitYaw,-LimitYaw);
//		
//		if(Is_Debug)//debug模式 
//		{
//			fsendpitch *= debug_pitch;
//			fsendyaw *= debug_yaw;		
//		}
//		
//		MotoPitch->I_Set = (1-K_pitch)*MotoPitch->I_Set + K_pitch*fsendpitch;
//		MotoYaw->I_Set = (1-K_yaw)*MotoYaw->I_Set + K_yaw*fsendyaw;

		float fsend; 
		int Last_Pitch_I,Last_Yaw_I;
		
		Last_Pitch_I = MotoPitch->I_Set;
    MotoPitch->PidPos.SetPoint = LIMIT_MAX_MIN(MotoPitch->PidPos.SetPoint, MotoPitch->MAX_ANGLE, MotoPitch->MIN_ANGLE);
    MotoPitch->PidSpeed.SetPoint = PID_Calc(&MotoPitch->PidPos, MotoPitch->actualAngle, 0) ;
		fsend = PID_Calc(&MotoPitch->PidSpeed, MotoPitch->Gyro_Speed, 0);
		MotoPitch->I_Set = LIMIT_MAX_MIN(fsend, LimitPitch, -LimitPitch);
		MotoPitch->I_Set = (1-MotoPitch->K_LP)*MotoPitch->I_Set + MotoPitch->K_LP*Last_Pitch_I;
		
		Last_Yaw_I = MotoYaw->I_Set;
    MotoYaw->PidPos.SetPoint = LIMIT_MAX_MIN(MotoYaw->PidPos.SetPoint, MotoYaw->MAX_ANGLE, MotoYaw->MIN_ANGLE);
    MotoYaw->PidSpeed.SetPoint = PID_Calc(&MotoYaw->PidPos, MotoYaw->actualAngle, 0);
		fsend = PID_Calc(&MotoYaw->PidSpeed, MotoYaw->Gyro_Speed, 0);
		MotoYaw->I_Set = LIMIT_MAX_MIN(fsend, LimitYaw, -LimitYaw);
		MotoYaw->I_Set = (1-MotoYaw->K_LP)*MotoYaw->I_Set + MotoYaw->K_LP*Last_Yaw_I;
}


/**
  * @brief  云台结构体初始化
  * @param  None
  * @retval None
  */
void Gimbal_Init()
{
		//左云台
		Gimbal_L.GimbalType = GIMBAL_LEFT;
		Gimbal_L.armor_state = ARMOR_NO_AIM;
		Gimbal_L.ModeUpdate_Flag = 0;
		Gimbal_L.LastMode = Gimbal_L_SLEEP;
		Gimbal_L.Pitch = &MotoPitch_L;
		Gimbal_L.Yaw = &MotoYaw_L;	

		//右云台
		Gimbal_R.GimbalType = GIMBAL_RIGHT;
		Gimbal_R.armor_state = ARMOR_NO_AIM;
		Gimbal_R.ModeUpdate_Flag = 0;
		Gimbal_R.LastMode = Gimbal_R_SLEEP;
		Gimbal_R.Pitch = &MotoPitch;
		Gimbal_R.Yaw = &MotoYaw;

}


/**
  * @brief  云台限幅，主要用于陀螺仪角作为反馈时用电机角限幅，目前暂时不适用
						使用的角度值为电机角
  * @param  None
  * @retval None
  */
float Gimbal_Limit(gimbal_motor_t Gimbal_Motor,uint8_t TYPE)
{
	//增量值
	float INCAngle_Low,INCAngle_High;
	
	if(TYPE == PATROL)//自动模式
	{
		INCAngle_Low = Gimbal_Motor.PATROL_MIN_ANGLE - Gimbal_Motor.motoAngle;
		INCAngle_High = Gimbal_Motor.PATROL_MAX_ANGLE - Gimbal_Motor.motoAngle;
	}
	else
	{
		INCAngle_Low = Gimbal_Motor.MIN_ANGLE - Gimbal_Motor.motoAngle;
		INCAngle_High = Gimbal_Motor.MAX_ANGLE - Gimbal_Motor.motoAngle;		
	}
	
	Gimbal_Motor.PidPos.SetPoint = LIMIT_MAX_MIN(Gimbal_Motor.PidPos.SetPoint,Gimbal_Motor.gyroAngle+INCAngle_High,Gimbal_Motor.gyroAngle+INCAngle_Low);
	
	return Gimbal_Motor.PidPos.SetPoint;
}

/**
  * @brief  上云台姿态输出计算（陀螺仪值和电机值的互补滤波）
  * @param  None
  * @retval None
  */
float moto_pitch, moto_yaw, moto_pitch_init, moto_yaw_init;
float gyro_pitch, gyro_yaw, gyro_pitch_init, gyro_yaw_init;
float comb_pitch, comb_yaw;
float moto_pitch_l, moto_yaw_l, moto_pitch_l_init, moto_yaw_l_init;
float gyro_pitch_l, gyro_yaw_l, gyro_pitch_l_init, gyro_yaw_l_init;
float comb_pitch_l, comb_yaw_l;
float k_pitch = 0;
float k_yaw = 0;
int8_t init_comb_flag = 1;
void Gimbal_GYRO_Cal(void)
{
    if (init_comb_flag)
    {
				//主云台
        moto_pitch_init = 1950.0f;
        gyro_pitch_init = GyroPitchOutPut(GIMBAL_RIGHT);
        //左 3102- 右8495
        moto_yaw_init = 1250.0f;
        gyro_yaw_init = GyroYawOutPut(GIMBAL_RIGHT);
			
				//左云台
				moto_pitch_l_init = 4762.0f;
        gyro_pitch_l_init = GyroPitchOutPut(GIMBAL_LEFT);
        //左 3102- 右8495
        moto_yaw_l_init = 7610.0f;
        gyro_yaw_l_init = GyroYawOutPut(GIMBAL_LEFT);
        init_comb_flag = 0;		
    }

		//实际上目前只用电机角
    MotoPitch.motoAngle = (PitchAngleOutPut(GIMBAL_RIGHT) - moto_pitch_init) / 8192.0f * 360.0f;
    MotoPitch.gyroAngle = Gyro_Right.PITCH;
    comb_pitch = k_pitch * MotoPitch.gyroAngle + (1 - k_pitch) * MotoPitch.motoAngle; //一阶互补滤波（没用到）
		MotoPitch.Gyro_Speed = Gyro_Right.GY*180.0f/PI;

    MotoYaw.motoAngle = (YawAngleOutPut(GIMBAL_RIGHT) - moto_yaw_init) / 8192.0f * 360.0f;
    MotoYaw.gyroAngle = Gyro_Right.YAW_ABS;
    comb_yaw = k_yaw * MotoYaw.gyroAngle + (1 - k_yaw) * MotoYaw.motoAngle; //一阶互补滤波
		MotoYaw.Gyro_Speed = (Gyro_Right.GZ-Gyro_ChassisYaw.GZ)*180.0f/PI;
		
		//左云台
		MotoPitch_L.motoAngle = (PitchAngleOutPut(GIMBAL_LEFT) - moto_pitch_l_init) / 8192.0f * 360.0f;
    MotoPitch_L.gyroAngle = Gyro_Left.PITCH;
    comb_pitch_l = k_pitch * MotoPitch_L.gyroAngle + (1 - k_pitch) * MotoPitch_L.motoAngle; //一阶互补滤波
		MotoPitch_L.Gyro_Speed = (Gyro_Left.GY)*180.0f/PI;

    MotoYaw_L.motoAngle = (YawAngleOutPut(GIMBAL_LEFT) - moto_yaw_l_init) / 8192.0f * 360.0f;
    MotoYaw_L.gyroAngle = Gyro_Left.YAW_ABS;
    comb_yaw_l = k_yaw * MotoYaw_L.gyroAngle + (1 - k_yaw) * MotoYaw_L.motoAngle; //一阶互补滤波
		MotoYaw_L.Gyro_Speed = (Gyro_Left.GZ-Gyro_ChassisYaw.GZ)*180.0f/PI;
}

/**
  * @brief  上云台旋转角度限位
  * @param  None
  * @retval None
  */
void Gimbal_Limit_Init(void)
{

		//主云台  pitch 2222下-1620上   中：2030 
		MotoPitch.MAX_ANGLE = +(2200.0f-1950)/8192.0f*360.0f; 
		MotoPitch.ZERO_POS = 0.0f;
		MotoPitch.MIN_ANGLE =-(1950.0f-1680)/8192.0f*360.0f;
		//面朝云台  左 633- 右2000   1100
    MotoYaw.MAX_ANGLE = (2000.0f-1250)/8192.0f*360.0f;
    MotoYaw.ZERO_POS = 0.0f;
    MotoYaw.MIN_ANGLE = -(1250.0f-640)/8192.0f*360.0f;
		//巡逻的范围一般比限位的范围要窄，因为相机的广角足够宽
		//2400     1980
    MotoPitch.PATROL_MAX_ANGLE =+(2100.0f-1950)/8192.0f*360.0f;;
    MotoPitch.PATROL_ZERO_POS = +0;
    MotoPitch.PATROL_MIN_ANGLE =-(1950.0f-1800)/8192.0f*360.0f;;

		//860   1640   1100
    MotoYaw.PATROL_MAX_ANGLE = (1640.0f-1250)/8192.0f*360.0f;
    MotoYaw.PATROL_ZERO_POS =  0;
    MotoYaw.PATROL_MIN_ANGLE =-(1250.0f-860)/8192.0f*360.0f;
		
		//左云台 5040  4500 中4762
		MotoPitch_L.MAX_ANGLE = +(5040.0f-4762)/8192.0f*360.0f; 
		MotoPitch_L.ZERO_POS = 0.0f;
		MotoPitch_L.MIN_ANGLE =-(4762.0f-4500)/8192.0f*360.0f;
		//左 8240  6900   7780
		MotoYaw_L.MAX_ANGLE = (8240.0f-7610)/8192.0f*360.0f;
    MotoYaw_L.ZERO_POS = 0.0f;
    MotoYaw_L.MIN_ANGLE = -(7610.0f-6900)/8192.0f*360.0f;
		//巡逻的范围一般比限位的范围要窄，因为相机的广角足够宽
		//1900
    MotoPitch_L.PATROL_MAX_ANGLE =+(4912.0f-4762)/8192.0f*360.0f;
    MotoPitch_L.PATROL_ZERO_POS = +0;
    MotoPitch_L.PATROL_MIN_ANGLE =-(4762.0f-4612)/8192.0f*360.0f;

		//4169  7068
    MotoYaw_L.PATROL_MAX_ANGLE = (8000.0f-7610)/8192.0f*360.0f;
    MotoYaw_L.PATROL_ZERO_POS =  0;
    MotoYaw_L.PATROL_MIN_ANGLE =-(7610.0f-7220)/8192.0f*360.0f;
}

/**
  * @brief  云台电机pid初始化
  * @param  None
  * @retval None
  */
void PID_Gimbal_Init(void)
{
/*陀螺仪反馈*/
		//主云台
    MotoPitch.PidPos.P = 16.0f;//100;//15.0f;//7.0;//7.3;//25.0f;
    MotoPitch.PidPos.I = 0.2;//0.3;//0.0f;//0.22;//0.0f;
    MotoPitch.PidPos.D = 20.0f;//1400;//0;//3;//3.0f;
    MotoPitch.PidPos.IMax = 50.0f;
		MotoPitch.PidPos.I_U = 2.0;
		MotoPitch.PidPos.I_L = 0.4;
    MotoPitch.PidPos.RC_DF = 0.5;//没有很明显的滤波效果
		MotoPitch.PidPos.OutMax = 125.0f;

    MotoPitch.PidSpeed.P = 145;//28;//180;//260;//300;//130.0f;//160.0f;
    MotoPitch.PidSpeed.I = 2.4;//0.5;//2;//1.4;//1.5;//1.6f;
    MotoPitch.PidSpeed.D = 2400;//0;//4;//0.0f;                
    MotoPitch.PidSpeed.IMax = 2000;  
		MotoPitch.PidSpeed.I_U = 20;//28;
		MotoPitch.PidSpeed.I_L = 6;//8;
		MotoPitch.PidSpeed.RC_DF = 0.4;
		MotoPitch.PidSpeed.OutMax = 28000;
	
		MotoPitch.K1 = 0.0f;//130.0f;
		MotoPitch.K2 = 0.0f;//-1550;
	
		MotoPitch.K_LP = 0.2;
	//速度环的模糊PID参数
    MotoPitch.PidSpeedF.Kp0 = 150.0f;//260f;
    MotoPitch.PidSpeedF.Ki0 = 1.6f;//1.4
    MotoPitch.PidSpeedF.Kd0 = 0.0f;                
    MotoPitch.PidSpeedF.IMax = 2000;//3800  
		MotoPitch.PidSpeedF.I_U = 28;//20;
		MotoPitch.PidSpeedF.I_L = 6;//8;
		MotoPitch.PidSpeedF.OutMax = 28000;
		
		MotoPitch.PidSpeedF.stair = 1.0f;//
		MotoPitch.PidSpeedF.Kp_stair = 10.0f;//30
		MotoPitch.PidSpeedF.Ki_stair = 0.1f;//0
		MotoPitch.PidSpeedF.Kd_stair = 0.0f;
			
    //////////////////////////////////////////////////////////////
    MotoYaw.PidPos.P = 16.0f;//18.0f;
    MotoYaw.PidPos.I = 0.0f;
    MotoYaw.PidPos.D = 0.0f;
    MotoYaw.PidPos.IMax = 0.0f;
    MotoYaw.PidPos.RC_DF = 0.5;//没有很明显的滤波效果
		MotoYaw.PidPos.OutMax = 200.0f;

    MotoYaw.PidSpeed.P = 200.0f;//230.0f;
    MotoYaw.PidSpeed.I = 1.7f;//2.1f;
    MotoYaw.PidSpeed.D = 0.0f;    
    MotoYaw.PidSpeed.IMax = 900.0f;
		MotoYaw.PidSpeed.I_U = 28;
		MotoYaw.PidSpeed.I_L = 7;
		MotoYaw.PidSpeed.OutMax = 28000;
		
	  MotoYaw.K1 = 0.0f;
		MotoYaw.K2 = 0.0f;
				
		//左云台
    MotoPitch_L.PidPos.P = 13.0f;//7.1f;//20.0f;
    MotoPitch_L.PidPos.I = 0.22;//0.32f;
    MotoPitch_L.PidPos.D = 5;//6.0f;
    MotoPitch_L.PidPos.IMax = 60;//90.0f;
    MotoPitch_L.PidPos.RC_DF = 0.5;//没有很明显的滤波效果
		MotoPitch_L.PidPos.OutMax = 125.0f;

    MotoPitch_L.PidSpeed.P = 135;//220.0f;//150.0f;
    MotoPitch_L.PidSpeed.I = 3.3;//1.45f;
    MotoPitch_L.PidSpeed.D = 1900;//4.0f;                
    MotoPitch_L.PidSpeed.IMax = 2000.0f;
		MotoPitch_L.PidSpeed.I_U = 20;//28;
		MotoPitch_L.PidSpeed.I_L = 6;//8;
		MotoPitch_L.PidSpeed.OutMax = 28000;
		MotoPitch_L.PidSpeed.RC_DF = 0.5;

		MotoPitch_L.K1 = 0.0f;
		MotoPitch_L.K2 = 0.0f;
		
		MotoPitch_L.K_LP = 0.3;
		
			//速度环的模糊PID参数
    MotoPitch_L.PidSpeedF.Kp0 = 220.0f;//160.0f;
    MotoPitch_L.PidSpeedF.Ki0 = 1.45f;
    MotoPitch_L.PidSpeedF.Kd0 = 4.0f;                
    MotoPitch_L.PidSpeedF.IMax = 2000;  
		MotoPitch_L.PidSpeedF.I_U = 28;//28;
		MotoPitch_L.PidSpeedF.I_L = 6;//8;
		MotoPitch_L.PidSpeedF.OutMax = 28000;
		
		MotoPitch_L.PidSpeedF.stair = 1.0f;
		MotoPitch_L.PidSpeedF.Kp_stair = 10.0f;
		MotoPitch_L.PidSpeedF.Ki_stair = 0.1f;
		MotoPitch_L.PidSpeedF.Kd_stair = 0.0f;
    //////////////////////////////////////////////////////////////
    MotoYaw_L.PidPos.P = 15.0f;//18.0f;
    MotoYaw_L.PidPos.I = 0.0f;
    MotoYaw_L.PidPos.D = 0.0f;
    MotoYaw_L.PidPos.IMax = 0.0f;
    MotoYaw_L.PidPos.RC_DF = 0.5;//没有很明显的滤波效果
		MotoYaw_L.PidPos.OutMax = 200.0f;

    MotoYaw_L.PidSpeed.P = 160.0f;//230.0f;
    MotoYaw_L.PidSpeed.I = 1.5f;//2.1f;
    MotoYaw_L.PidSpeed.D = 0.0f;    
    MotoYaw_L.PidSpeed.IMax = 900.0f;
		MotoYaw_L.PidSpeed.I_U = 28;
		MotoYaw_L.PidSpeed.I_L = 7;
		MotoYaw_L.PidSpeed.OutMax = 28000;
		
	  MotoYaw_L.K1 = 0.0f;
		MotoYaw_L.K2 = 0.0f;
		
}

/**
  * @brief  辅瞄通信掉线处理
  * @param  None
  * @retval None
  */
void Aim_Disconnect_Act()
{
		//热量控制的调电检测
		if(Robo_Disconnect.AimShootDisconnect > 500)
		{
				Gimbal_R.armor_state = ARMOR_NO_AIM;
		}
		else
			Robo_Disconnect.AimShootDisconnect ++;
}


/**
  * @brief  获取滤波后的云台两轴姿态（pitch和yaw）
  * @param  None
  * @retval pitch和yaw的角度
  */
float CombPitchOutput(uint8_t Gimbal_Type)
{
	if(Gimbal_Type == GIMBAL_RIGHT)
    return comb_pitch;
	else
		return comb_pitch_l;
}
float CombYawOutput(uint8_t Gimbal_Type)
{
	if(Gimbal_Type == GIMBAL_RIGHT)
    return comb_yaw;
	else
		return comb_yaw_l;
}
