/**********************************************************************************************************
 * @文件     GimbalTask.c
 * @说明     云台控制
 * @版本  	 V1.0
 * @作者     黄志雄
 * @日期     2020.1
 **********************************************************************************************************/
/**********************************************************************************************************
 * @文件     GimbalTask.c
 * @说明     云台控制
 * @版本  	 V2.0
 * @作者     戴军
 * @日期     2022.4
 **********************************************************************************************************/
#include "main.h"
/*----------------------------------内部变量---------------------------*/

int inttoshort[4];
short GimbalAct_Init_Flag = 0;

short YawCurrent, PitchCurrent;
float GimbalYawPos, GimbalPitchPos, GimbalLastPitchPos, DeltaGimbalPitchPos;

float Recent_Angle_Buff;
float Recent_Yaw_Angle_Armor;
float Recent_Pitch_Angle_Armor;
float pitchpos;

float patrol_dir_pitch = 1;
float patrol_dir_yaw = 1;
float patrol_step_pitch = 0.08f;
float patrol_step_yaw = 0.12f;
/*-----------------------------------结构体-----------------------------*/
Pid_Typedef PidPitchSpeed, PidPitchPos, PidYawSpeed, PidYawPos;
Pid_Typedef PidPitchAidPos, PidPitchAidSpeed, PidYawAidPos, PidYawAidSpeed, PidPitchBuffSpeed, PidYawBuffSpeed;
FeedForward_t PitchPosFF, PitchSpeedFF, YawPosFF, YawSpeedFF;
TD_t PitchTD,YawTD;
/*----------------------------------外部变量---------------------------*/
extern RC_Ctl_t RC_Ctl;
extern Status_t Status;
extern char PitchMotor_ReceiveFlag;
extern Gimbal_Typedef Gimbal;
extern Gyro_Typedef GyroReceive; // 陀螺仪数据
extern RobotInit_Struct Infantry;
extern short FrictionWheel_speed;
extern F405_typedef F405;
extern PC_Receive_t PC_Receive;
extern char Budan;
extern float Buff_Yaw_Motor;
extern char q_flag;
extern FeedForward_Typedef FF_w;
extern char pitch_lose_flag;
extern TaskHandle_t User_Tasks[TASK_NUM];
extern PCRecvData pc_recv_data;

uint32_t cnt_last;
/**********************************************************************************************************
 *函 数 名: Gimbal_Powerdown_Cal
 *功能说明: 云台掉电模式
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void Gimbal_Powerdown_Cal()
{
	if (GimbalAct_Init_Flag != Gimbal_Powerdown_Mode)
	{
		Laser_Off();
		GimbalAct_Init_Flag = Gimbal_Powerdown_Mode;
	}

	/***********************************************************************************/
	PidPitchPos.SetPoint = Infantry.pitch_min_motor;

	/**************************************计算电流值**************************************/
	// PITCH
	PidPitchPos.ActualValue = Gimbal.Pitch.Gyro;
	PidPitchSpeed.SetPoint = -PID_Calc(&PidPitchPos);
	PidPitchSpeed.ActualValue = GyroReceive.GY;
}
/**********************************************************************************************************
 *函 数 名: FuzzyGimbal_Act_Cal
 *功能说明: 模糊云台正常模式(电机角)
 *形    参: rc  mouse  Pc_RecvData
 *返 回 值: 无
 **********************************************************************************************************/
float k_yaw;
char ch2_return_flag;
int diudiao = 0;
double yuzhi = 0.002;
float yaw_freq = 1;
float yaw_Amplite = 20;
void MotorGimbal_Act_Cal(Remote rc, Mouse mouse)
{
	if (GimbalAct_Init_Flag != Gimbal_Act_Mode)
	{
		Laser_On();
		GimbalPitchPos = Gimbal.Pitch.Gyro; // 从大符模式切回，保持pitch电机角，yaw陀螺仪角，不乱动
		GimbalYawPos = Gimbal.Yaw.Gyro;
		GimbalAct_Init_Flag = Gimbal_Act_Mode;
		ch2_return_flag = 0;
	}

	if (Status.ControlMode == Control_RC_Mode) // Rc_Control
	{
		if (rc.ch2 != 1024)
		{
			GimbalYawPos = GimbalYawPos + (1024 - rc.ch2) * 0.0008f;
			ch2_return_flag = 1;
		}
		else if (ch2_return_flag)
		{
			GimbalYawPos = Gimbal.Yaw.Gyro;
			ch2_return_flag = 0;
		}

		GimbalPitchPos = GimbalPitchPos - (1024 - rc.ch3) * 0.0004f;
		FF_w.Now_DeltIn = (1024 - rc.ch2) * 0.0016f;
	}
	if (Status.ControlMode == Control_MouseKey_Mode) // Mouse_Key
	{
		GimbalPitchPos -= mouse.y * 0.006f; // 原本：0.005f
		GimbalYawPos -= mouse.x * 0.004f;	// 0.0016
		GimbalPitchPos += mouse.z * 0.004f;

		FF_w.Now_DeltIn = -mouse.x * 0.0032f;
	}

	// PITCH限位
	DeltaGimbalPitchPos = Gimbal.Pitch.Gyro - Gimbal.Pitch.MotorTransAngle - Infantry.init_delta_pitch;
	GimbalPitchPos = LIMIT_MAX_MIN(GimbalPitchPos, Infantry.pitch_max_gyro + DeltaGimbalPitchPos, Infantry.pitch_min_gyro + DeltaGimbalPitchPos);

	/***********************************************************************************/

	//	PidPitchPos.SetPoint = GimbalPitchPos;
	//	PidYawPos.SetPoint = GimbalYawPos;
	//
}
/**********************************************************************************************************
 *函 数 名: FuzzyGimbal_Act_Cal
 *功能说明: 模糊云台正常模式(IMU角)
 *形    参: rc  mouse  Pc_RecvData
 *返 回 值: 无
 **********************************************************************************************************/

void GyroGimbal_Act_Cal(Remote rc, Mouse mouse)
{
	if (GimbalAct_Init_Flag != Gimbal_Act_Mode)
	{
		Laser_On();
		GimbalPitchPos = Gimbal.Pitch.Gyro;
		GimbalLastPitchPos = Gimbal.Pitch.Gyro;
		GimbalYawPos = Gimbal.Yaw.Gyro;
		GimbalAct_Init_Flag = Gimbal_Act_Mode;
	}

	if (Status.ControlMode == Control_RC_Mode) // Rc_Control
	{
		GimbalYawPos = (rc.ch2 == 1024) ? Gimbal.Yaw.Gyro : (GimbalYawPos + (1024 - rc.ch2) * 0.0010f);
		GimbalPitchPos = (rc.ch3 == 1024) ? Gimbal.Pitch.Gyro : (GimbalPitchPos - (1024 - rc.ch3) * 0.0005f); // 旧陀螺仪
		FF_w.Now_DeltIn = (1024 - rc.ch2) * 0.0005f;
	}
	if (Status.ControlMode == Control_MouseKey_Mode) // Mouse_Key
	{
		GimbalPitchPos -= mouse.y * 0.005f;
		GimbalYawPos -= mouse.x * 0.005f;
		GimbalPitchPos -= mouse.z * 0.001f;
		FF_w.Now_DeltIn = -mouse.x * 0.005f;
	}

	// PITCH限位
	DeltaGimbalPitchPos = Gimbal.Pitch.Gyro - Gimbal.Pitch.MotorTransAngle - Infantry.init_delta_pitch;
	LIMIT_MAX_MIN(GimbalPitchPos,
				  Infantry.pitch_max_gyro + DeltaGimbalPitchPos,
				  Infantry.pitch_min_gyro + DeltaGimbalPitchPos);

}

/**********************************************************************************************************
 *函 数 名: Gimbal_Armor_Cal
 *功能说明: 云台辅瞄模式
 *形    参: rc  mouse  Pc_RecvData
 *返 回 值: 无
 **********************************************************************************************************/
float speed_limit = 30.0f;
char Aim_Follow;
float Inte_z;
extern float pc_yaw, pc_pitch;
float last_aim_yaw;

void Gimbal_Armor_Cal(Remote rc, Mouse mouse)
{
	if (GimbalAct_Init_Flag != Gimbal_Armor_Mode)
	{
		GimbalAct_Init_Flag = Gimbal_Armor_Mode;
		last_aim_yaw = GimbalYawPos = Gimbal.Yaw.Gyro;
		GimbalPitchPos = Gimbal.Pitch.Gyro;
		pc_yaw = Gimbal.Yaw.Gyro;
		pc_pitch = Gimbal.Pitch.Gyro;

		// Inte_z = 0;
	}

	if (pc_recv_data.enemy_id != 0) // 判断辅瞄是否工作
	{
		Recent_Pitch_Angle_Armor = pc_pitch;
		Recent_Yaw_Angle_Armor = pc_yaw;
		Inte_z += mouse.z * 0.002f;

		if (ABS(Recent_Yaw_Angle_Armor - Gimbal.Yaw.Gyro) < 70 && ABS(Recent_Pitch_Angle_Armor - (Gimbal.Pitch.Gyro)) < 60) // 程序安全
		{
			GimbalPitchPos = Recent_Pitch_Angle_Armor + Inte_z;
			GimbalYawPos = Recent_Yaw_Angle_Armor;
			FF_w.Now_DeltIn = ((GimbalYawPos - last_aim_yaw) > 0.2f) ? ((GimbalYawPos - last_aim_yaw)) : (0);
		}
		else
		{
			GimbalPitchPos = Gimbal.Pitch.Gyro; // 更新值
			GimbalYawPos = Gimbal.Yaw.Gyro;
			FF_w.Now_DeltIn = 0;
		}
	}

	last_aim_yaw = GimbalYawPos;

	// PITCH限位
	DeltaGimbalPitchPos = Gimbal.Pitch.Gyro - Gimbal.Pitch.MotorTransAngle - Infantry.init_delta_pitch;
	GimbalPitchPos = LIMIT_MAX_MIN(GimbalPitchPos,
								   Infantry.pitch_max_gyro + DeltaGimbalPitchPos,
								   Infantry.pitch_min_gyro + DeltaGimbalPitchPos);

}

/**********************************************************************************************************
 *函 数 名: Gimbal_Buff_Cal
 *功能说明: 云台打符模式
 *形    参: rc  mouse  Pc_RecvData
 *返 回 值: 无
 **********************************************************************************************************/
void Gimbal_Buff_Cal(Remote rc, Mouse mouse)
{
	if (GimbalAct_Init_Flag != Gimbal_BigBuf_Mode)
	{
		GimbalAct_Init_Flag = Gimbal_BigBuf_Mode;
		GimbalYawPos = Gimbal.Yaw.Gyro;
		GimbalPitchPos = Gimbal.Pitch.Gyro;
		pc_yaw = Gimbal.Yaw.Gyro;
		pc_pitch = Gimbal.Pitch.Gyro;
	}

	if (pc_recv_data.enemy_id != 0) // 判断辅瞄是否工作
	{
		Recent_Pitch_Angle_Armor = pc_pitch;
		Recent_Yaw_Angle_Armor = pc_yaw;
		Inte_z += mouse.z * 0.002f;

		if (ABS(Recent_Yaw_Angle_Armor - Gimbal.Yaw.Gyro) < 70 && ABS(Recent_Pitch_Angle_Armor - Gimbal.Pitch.Gyro) < 60) // 程序安全
		{
			GimbalPitchPos = Recent_Pitch_Angle_Armor + Inte_z; // 更新值
			GimbalYawPos = Recent_Yaw_Angle_Armor;
		}
		else
		{
			GimbalPitchPos = Gimbal.Pitch.Gyro; // 更新值
			GimbalYawPos = Gimbal.Yaw.Gyro;
		}
	}

	// PITCH限位
	DeltaGimbalPitchPos = Gimbal.Pitch.Gyro - Gimbal.Pitch.MotorTransAngle - Infantry.init_delta_pitch;
	GimbalPitchPos = LIMIT_MAX_MIN(GimbalPitchPos,
								   Infantry.pitch_max_gyro + DeltaGimbalPitchPos,
								   Infantry.pitch_min_gyro + DeltaGimbalPitchPos);

}

///**********************************************************************************************************
//*函 数 名: Gimbal_DropShot_Cal
//*功能说明: 云台吊射模式
//*形    参: rc  mouse  Pc_RecvData
//*返 回 值: 无
//**********************************************************************************************************/

// void Gimbal_DropShot_Cal(Remote rc,Mouse mouse)
//{
//	if( GimbalAct_Init_Flag!=Gimbal_DropShot_Mode)
//	{
//		Laser_On();
//		GimbalAct_Init_Flag=Gimbal_DropShot_Mode;
//		GimbalYawPos = Gimbal.Yaw.MotorTransAngle;//定轴模式
//		GimbalPitchPos = Gimbal.Pitch.MotorTransAngle;
//	}
//
//	if(Status.ControlMode==Control_RC_Mode)//Rc_Control
//	{
//	  GimbalYawPos   += (1024-rc.ch2)*0.0005f;
//		GimbalPitchPos -= (1024-rc.ch3)*0.0005f;//陀螺仪
//	}
//	if(Status.ControlMode==Control_MouseKey_Mode)//Mouse_Key
//	{
//     GimbalPitchPos -= mouse.y*0.005f;
//     GimbalYawPos   -= mouse.x*0.005f;
//	}
//
//	GimbalPitchPos=LIMIT_MAX_MIN(GimbalPitchPos,Infantry.pitch_max_motor,Infantry.pitch_min_motor);//限位(用电机角度)
//   /***********************************************************************************/
//	PidPitchPos.SetPoint = GimbalPitchPos;
//	PidYawPos.SetPoint = GimbalYawPos;
//   /**************************************计算电流值**************************************/
//	PidPitchSpeed.SetPoint=PID_Calc(&PidPitchPos,Gimbal.Pitch.MotorTransAngle);
//	inttoshort[0]=-(PID_Calc(&PidPitchSpeed,GyroReceive.GY));//旧陀螺仪
//	PitchCurrent=(short)inttoshort[0];
//
//	PidYawSpeed.SetPoint=PID_Calc(&PidYawPos,Gimbal.Yaw.MotorTransAngle);
//	PidYawSpeed.SetPoint=LIMIT_MAX_MIN(PidYawSpeed.SetPoint,5.5f,-5.5f);
//	inttoshort[1]=PID_Calc(&PidYawSpeed,GyroReceive.GZ);
//	YawCurrent=inttoshort[1];
// }

/**********************************************************************************************************
 *函 数 名: Gimbal_Test_Cal
 *功能说明: 云台测试模式
 *形    参: rc  mouse  Pc_RecvData
 *返 回 值: 无
 **********************************************************************************************************/
void Gimbal_Test_Cal(Remote rc, Mouse mouse)
{
	if (GimbalAct_Init_Flag != Gimbal_Test_Mode)
	{
		GimbalAct_Init_Flag = Gimbal_Test_Mode;
		GimbalYawPos = Gimbal.Yaw.MotorTransAngle;
		GimbalPitchPos = Gimbal.Pitch.MotorTransAngle;
	}
	if (Status.ControlMode == Control_RC_Mode) // Rc_Control
	{
		GimbalYawPos += (1024 - rc.ch2) * 0.00005f;
		GimbalPitchPos -= (1024 - rc.ch3) * 0.00005f; // 陀螺仪
	}

	// PITCH限位
	DeltaGimbalPitchPos = Gimbal.Pitch.Gyro - Gimbal.Pitch.MotorTransAngle - Infantry.init_delta_pitch;
	GimbalPitchPos = LIMIT_MAX_MIN(GimbalPitchPos,
								   Infantry.pitch_max_gyro + DeltaGimbalPitchPos,
								   Infantry.pitch_min_gyro + DeltaGimbalPitchPos);

	/***************************************************************************************/
	//	PidPitchAidPos.SetPoint = GimbalPitchPos;
	//  PidPitchAidPos.SetPoint = TestPos[countertest%4][0]; //矩形
	//  PidPitchAidPos.SetPoint = 15*arm_sin_f32((float)counterbase/w+3.1415926/2); //正弦
	//  PidPitchAidPos.SetPoint = TD_Calculate(&PitchTD,TestPitch);

	//	PidYawAidPos.SetPoint = GimbalYawPos;
	//  PidYawAidPos.SetPoint = TestPos[countertest%4][1]; //矩形
	//  PidYawAidPos.SetPoint = 30*arm_sin_f32((float)counterbase/w); //正弦
	//  PidYawAidPos.SetPoint = TD_Calculate(&YawTD,TestYaw);
	//  PidYawAidPos.SetPoint = TD_Calculate(&YawTD,GimbalYawPos);

	//  PidYawAidSpeed.SetPoint = A*arm_sin_f32((float)counterbase/w); //正弦
}

/**********************************************************************************************************
 *函 数 名: Gimbal_SI_Cal
 *功能说明: 云台系统辨识模式
 *形    参: rc  mouse  Pc_RecvData
 *返 回 值: 无
 **********************************************************************************************************/
short F_Change_flag = 0; // 切换频率标志
float Gimbal_direct;

int T;					   // 周期
int T_cnt = 0;			   // 计数
int T_Time_cnt = 0;		   // 周期次数计数
int F_cnt = 0, F_cnt_last; // 指向F的指针
float F = 1;

void Gimbal_SI_Cal(float Gimbal_pitch, float Gimbal_yaw)
{
	if (GimbalAct_Init_Flag != Gimbal_SI_Mode)
	{
		GimbalPitchPos = 0;
		GimbalYawPos = Gimbal.Yaw.Gyro;
		GimbalPitchPos = Gimbal.Pitch.MotorTransAngle; // 用于阶跃响应测试
		GimbalYawPos = Gimbal.Yaw.Gyro + 40.0f;
		F = 1;
		GimbalAct_Init_Flag = Gimbal_SI_Mode;
	}

	T_change();
	//		GimbalPitchPos = LIMIT_MAX_MIN((Gimbal_direct*Gimbal_pitch),Infantry.pitch_max_motor,Infantry.pitch_min_motor);

	/**************************************计算电流值**************************************/
	/***************************************************************************************/
	// PITCH限位
	DeltaGimbalPitchPos = Gimbal.Pitch.Gyro - Gimbal.Pitch.MotorTransAngle - Infantry.init_delta_pitch;
	LIMIT_MAX_MIN(GimbalPitchPos,
				  Infantry.pitch_max_gyro + DeltaGimbalPitchPos,
				  Infantry.pitch_min_gyro + DeltaGimbalPitchPos);

	/***********************************************************************************/
	//	FuzzyAidPidPitchPos.SetPoint = GimbalPitchPos;
	//	FuzzyAidPidPitchPos.ActualValue = Gimbal.Pitch.MotorTransAngle;

	//	PidPitchAidSpeed.SetPoint = -FuzzyPID_Calc(&FuzzyAidPidPitchPos);
	//	PidPitchAidSpeed.ActualValue = GyroReceive.GY;
	//	inttoshort[0] = -(PID_Calc(&PidPitchAidSpeed)); //旧陀螺仪
	//	PitchCurrent = -(short)inttoshort[0] * Infantry.motor_pn;

	//	FuzzyAidPidYawPos.SetPoint = GimbalYawPos;
	//	FuzzyAidPidYawPos.ActualValue = Gimbal.Yaw.Gyro;

	//	PidYawAidSpeed.SetPoint = FuzzyPID_Calc(&FuzzyAidPidYawPos);
	//	PidYawAidSpeed.ActualValue = GyroReceive.GZ;
	//	inttoshort[1] = -PID_Calc(&PidYawAidSpeed);
	//	YawCurrent = inttoshort[1];

	///************************************** 用于开环系统辨识 ******************************************/
	//  	PitchCurrent= Gimbal_direct*Gimbal_pitch;
	//	  YawCurrent = Gimbal_direct*Gimbal_yaw;
}

void get_F(void)
{
	static int j = 0;
	if (F < 22)
	{
		j++;
		F += 0.5f;
	}
	else if (F == 22)
	{
		F = 24;
	}
	else if (F >= 24 && F < 40)
	{
		F = F + 2;
	}
	else if (F == 40)
	{
		F = 50;
	}
	else if (F >= 50 && F < 120)
	{
		F = F + 10;
	}
	else if (F == 120)
	{
		F = 200;
	}
	else if (F == 250)
	{
		F = 333;
	}
	else if (F == 333)
	{
		F = 500;
	}
}
/**********************************************************************************************************
 *函 数 名: T_Change
 *功能说明: 根据不同的频率获得对应不同周期下sin信号,F低于50重复10周期，否则20周期
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void T_change(void)
{
	T = round(1000 / F);
	if (GimbalAct_Init_Flag == Gimbal_SI_Mode)
	{
		if (F_Change_flag == 0)
		{
			Gimbal_direct = sin(2 * 3.14 * T_cnt / T);
			if (T_cnt >= T)
			{
				T_cnt = 0;
				T_Time_cnt++;
			}
			T_cnt++;
			F_cnt_last = F_cnt;

			if (T_Time_cnt >= 10 && F < 22)
			{
				T_Time_cnt = 0;
				F_cnt++;
			}
			else if (T_Time_cnt >= 10 && F >= 22 && F < 50)
			{
				T_Time_cnt = 0;
				F_cnt++;
			}
			else if (T_Time_cnt >= 20 && F >= 50)
			{
				T_Time_cnt = 0;
				F_cnt++;
			}
			if (F_cnt != F_cnt_last) // 频率改变
			{
				F_Change_flag = 1;
			}
		}
		else if (F_Change_flag == 1)
		{
			Gimbal_direct = 0;
			if ((ABS(PidPitchPos.SetPoint - PidPitchPos.ActualValue) < 0.2f) || (ABS(PidYawPos.SetPoint - PidYawPos.ActualValue) < 0.2f)) // 回到初始位置
			{
				get_F();
				T_cnt = 0;
				F_Change_flag = 0;
			}
		}
	}
	else if (GimbalAct_Init_Flag != Gimbal_SI_Mode)
	{
		F = 1;
	}
}

/**********************************************************************************************************
 *函 数 名: Gimbal_PC_Cal
 *功能说明: 云台巡航辅瞄
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
float Change_Yaw; // 辅瞄丢时的Yaw值,主要方便辅瞄调的时候不会一直一个方向
int Armor_cnt = 0;
extern uint8_t armor_state; //没目标
void Gimbal_PC_Cal()
{
	if (GimbalAct_Init_Flag != Gimbal_PC_Mode)
	{
		GimbalAct_Init_Flag = Gimbal_PC_Mode;
		Recent_Yaw_Angle_Armor = Change_Yaw = last_aim_yaw = Gimbal.Yaw.Gyro;
		Recent_Pitch_Angle_Armor = Gimbal.Pitch.Gyro;
	}

	DeltaGimbalPitchPos = Gimbal.Pitch.Gyro - Gimbal.Pitch.MotorTransAngle - Infantry.init_delta_pitch;

	if (NAV_car.Gimbal_PC_State == NAV_STATE) // 导航--云台跟随
	{
		if((armor_state == ARMOR_AIMED)&&
			ABS(pc_yaw - Gimbal.Yaw.Gyro) < 70 && ABS(pc_pitch - (Gimbal.Pitch.Gyro)) < 60)//识别到
		{
			GimbalPitchPos = pc_pitch;
			GimbalYawPos = pc_yaw;
		}			
		else //执行导航
		{
		GimbalPitchPos = Gimbal.Pitch.Gyro;
		GimbalYawPos += NAV_car.NAV_w * 0.001f;
		}
//		FF_w.Now_DeltIn = NAV_car.NAV_w * 0.001f;//前馈
	}

	else if(NAV_car.Gimbal_PC_State == ARMOR_STATE) //辅瞄模式
	{
		if (armor_state == ARMOR_AIMED) // 判断辅瞄是否工作
		{
			Recent_Pitch_Angle_Armor = pc_pitch;
			Recent_Yaw_Angle_Armor = pc_yaw;

			if (ABS(Recent_Yaw_Angle_Armor - Gimbal.Yaw.Gyro) < 70 && ABS(Recent_Pitch_Angle_Armor - (Gimbal.Pitch.Gyro)) < 60) // 程序安全
			{
				GimbalPitchPos = Recent_Pitch_Angle_Armor;
				GimbalYawPos = Recent_Yaw_Angle_Armor;
				FF_w.Now_DeltIn = ((GimbalYawPos - last_aim_yaw) > 0.2f) ? ((GimbalYawPos - last_aim_yaw)) : (0);
			}
			else
			{
				GimbalPitchPos = Gimbal.Pitch.Gyro; // 更新值
				GimbalYawPos = Gimbal.Yaw.Gyro;
				FF_w.Now_DeltIn = 0;
			}
			Change_Yaw = GimbalYawPos;
			Armor_cnt = 1000; // 停500ms
		}

		else
		{
			if (Armor_cnt < 0)
			{
				if ((Recent_Pitch_Angle_Armor + patrol_step_pitch >= Infantry.pitch_max_gyro - 15 + DeltaGimbalPitchPos))
					patrol_dir_pitch = 0; // 电机到上限反向
				if ((Recent_Pitch_Angle_Armor - patrol_step_pitch <= Infantry.pitch_min_gyro + 8 + DeltaGimbalPitchPos))
					patrol_dir_pitch = 1; // 电机到下限反向

				if ((Recent_Yaw_Angle_Armor + patrol_step_yaw >= Change_Yaw + 200.0f))
					patrol_dir_yaw = 0; // 电机到上限反向
				if ((Recent_Yaw_Angle_Armor - patrol_step_yaw <= Change_Yaw - 200.0f))
					patrol_dir_yaw = 1; // 电机到下限反向

				Recent_Pitch_Angle_Armor += (patrol_dir_pitch == 1) ? (+patrol_step_pitch) : (-patrol_step_pitch);
				Recent_Yaw_Angle_Armor +=patrol_step_yaw;
			}
			else //主要用于辅瞄和巡航的切换过渡
			{
				Armor_cnt--;
				Recent_Pitch_Angle_Armor = Gimbal.Pitch.Gyro;
				Recent_Yaw_Angle_Armor = Gimbal.Yaw.Gyro;
			}

			GimbalPitchPos = Recent_Pitch_Angle_Armor;
			GimbalYawPos = Recent_Yaw_Angle_Armor;
		}
		last_aim_yaw = GimbalYawPos;
	}
	
	else//自动模式过渡
	{
			GimbalPitchPos = Recent_Pitch_Angle_Armor = Gimbal.Pitch.Gyro;
			GimbalYawPos = Recent_Yaw_Angle_Armor = Gimbal.Yaw.Gyro;
	}

	// PITCH限位

	GimbalPitchPos = LIMIT_MAX_MIN(GimbalPitchPos,
								   Infantry.pitch_max_gyro + DeltaGimbalPitchPos,
								   Infantry.pitch_min_gyro + DeltaGimbalPitchPos);
}

/**********************************************************************************************************
 *函 数 名: Gimbal_CurrentPid_Cal
 *功能说明: 发送电流值
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void GimbalPos_Set(void)
{
	switch (Status.GimbalMode)
	{
	case Gimbal_Powerdown_Mode:
		Gimbal_Powerdown_Cal();
		break;

	case Gimbal_Act_Mode:
		MotorGimbal_Act_Cal(RC_Ctl.rc, RC_Ctl.mouse);
		break;

	case Gimbal_Armor_Mode:
		Gimbal_Armor_Cal(RC_Ctl.rc, RC_Ctl.mouse);
		break;

	case Gimbal_BigBuf_Mode:
	case Gimbal_SmlBuf_Mode:
		Gimbal_Buff_Cal(RC_Ctl.rc, RC_Ctl.mouse);
		break;

		//		case Gimbal_DropShot_Mode:
		//			Gimbal_DropShot_Cal(RC_Ctl.rc,RC_Ctl.mouse);
		//			break;

	case Gimbal_SI_Mode:
		Gimbal_SI_Cal(8000.0, 0); // pitch
		//			Gimbal_SI_Cal(0.0, 30.0);		//yaw
		break;

	case Gimbal_Jump_Mode:
		GyroGimbal_Act_Cal(RC_Ctl.rc, RC_Ctl.mouse);
		break;

	case Gimbal_Test_Mode:
		Gimbal_Test_Cal(RC_Ctl.rc, RC_Ctl.mouse);
		break;

	case Gimbal_PC_Mode:
		Gimbal_PC_Cal();
		break;

	default:
		Gimbal_Powerdown_Cal();
		break;
	}

	//	if (q_flag)
	//		F405.Gimbal_Flag = 7; //云台自己打弹
	//	else
	//		F405.Gimbal_Flag = Status.GimbalMode;
	F405.Gimbal_Flag = Status.GimbalMode;
}
/**********************************************************************************************************
 *函 数 名: Gimbal_CurrentPid_Cal
 *功能说明: 发送电流值
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
float GimbalYawPos_last, GimbalPitchPos_last;
void Gimbal_CurrentPid_Cal(void)
{
	switch (Status.GimbalMode)
	{
	case Gimbal_Powerdown_Mode:
		PitchCurrent = 0;
		YawCurrent = 0;
		TD_Clear(&PitchTD,Gimbal.Pitch.Gyro);
	  TD_Clear(&YawTD,Gimbal.Yaw.Gyro);
		break;

	case Gimbal_Act_Mode:
	case Gimbal_PC_Mode:
		PidPitchAidPos.SetPoint = TD_Calculate(&PitchTD,GimbalPitchPos);
		PidPitchAidPos.ActualValue = Gimbal.Pitch.Gyro;
		PidPitchAidSpeed.SetPoint = -PID_Calc(&PidPitchAidPos); //+FeedForward_Cal(&PitchPosFF,PidPitchAidPos.SetPoint);
		PidPitchAidSpeed.ActualValue = Gimbal.Pitch.AngularSpeed;
		inttoshort[0] = -(PID_Calc(&PidPitchAidSpeed)); // 旧陀螺仪
		PitchCurrent = -(short)inttoshort[0] * Infantry.motor_pn;

		PidYawAidPos.SetPoint = TD_Calculate(&YawTD,GimbalYawPos);
		PidYawAidPos.ActualValue = Gimbal.Yaw.Gyro;
		PidYawAidSpeed.SetPoint = PID_Calc(&PidYawAidPos) + FeedForward_Cal(&YawPosFF, PidYawAidPos.SetPoint);
		PidYawAidSpeed.ActualValue = Gimbal.Yaw.AngularSpeed;
		inttoshort[1] = -PID_Calc(&PidYawAidSpeed);
		YawCurrent = -(short)inttoshort[1];
		break;

	case Gimbal_Armor_Mode:
		PidPitchAidPos.SetPoint = TD_Calculate(&PitchTD,GimbalPitchPos);
		PidPitchAidPos.ActualValue = Gimbal.Pitch.Gyro;
		PidPitchAidSpeed.SetPoint = -PID_Calc(&PidPitchAidPos); //+FeedForward_Cal(&PitchPosFF,PidPitchAidPos.SetPoint);
		PidPitchAidSpeed.ActualValue = Gimbal.Pitch.AngularSpeed;
		inttoshort[0] = -(PID_Calc(&PidPitchAidSpeed)); // 旧陀螺仪
		PitchCurrent = -(short)inttoshort[0] * Infantry.motor_pn;

		PidYawAidPos.SetPoint = TD_Calculate(&YawTD,GimbalYawPos);
		PidYawAidPos.ActualValue = Gimbal.Yaw.Gyro;
		PidYawAidSpeed.SetPoint = PID_Calc(&PidYawAidPos); //+FeedForward_Cal(&YawPosFF,PidYawAidPos.SetPoint);
		PidYawAidSpeed.ActualValue = Gimbal.Yaw.AngularSpeed;
		inttoshort[1] = -PID_Calc(&PidYawAidSpeed);
		YawCurrent = -(short)inttoshort[1];
		break;

	case Gimbal_BigBuf_Mode:
	case Gimbal_SmlBuf_Mode:
		PidPitchAidPos.SetPoint = TD_Calculate(&PitchTD,GimbalPitchPos);
		PidPitchAidPos.ActualValue = Gimbal.Pitch.Gyro;
		PidPitchAidSpeed.SetPoint = -PID_Calc(&PidPitchAidPos); //+FeedForward_Cal(&PitchPosFF,PidPitchAidPos.SetPoint);
		PidPitchAidSpeed.ActualValue = Gimbal.Pitch.AngularSpeed;
		inttoshort[0] = -(PID_Calc(&PidPitchAidSpeed)); // 旧陀螺仪
		PitchCurrent = -(short)inttoshort[0] * Infantry.motor_pn;

		PidYawAidPos.SetPoint = TD_Calculate(&YawTD,GimbalYawPos);
		PidYawAidPos.ActualValue = Gimbal.Yaw.Gyro;
		PidYawAidSpeed.SetPoint = PID_Calc(&PidYawAidPos) + FeedForward_Cal(&YawPosFF, PidYawAidPos.SetPoint);
		PidYawAidSpeed.ActualValue = Gimbal.Yaw.AngularSpeed;
		inttoshort[1] = -PID_Calc(&PidYawAidSpeed);
		YawCurrent = -(short)inttoshort[1];
		break;

	case Gimbal_SI_Mode:
		// Gimbal_SI_Cal(8000.0, 0); // pitch
		//			Gimbal_SI_Cal(0.0, 30.0);		//yaw
		break;

	case Gimbal_Jump_Mode:
		PidPitchAidPos.SetPoint = TD_Calculate(&PitchTD,GimbalPitchPos);
		PidPitchPos.ActualValue = Gimbal.Pitch.Gyro;
		PidPitchSpeed.SetPoint = -PID_Calc(&PidPitchPos);
		PidPitchSpeed.ActualValue = GyroReceive.GY;
		inttoshort[0] = -(PID_Calc(&PidPitchSpeed)); // 旧陀螺仪
		PitchCurrent = (short)(-inttoshort[0]) * Infantry.motor_pn;

		PidYawAidPos.SetPoint = TD_Calculate(&YawTD,GimbalYawPos);
		PidYawPos.ActualValue = Gimbal.Yaw.Gyro;
		PidYawSpeed.SetPoint = PID_Calc(&PidYawPos); // 需要YAW轴陀螺仪角度做ActualValue
		PidYawSpeed.ActualValue = GyroReceive.GZ;
		inttoshort[1] = -PID_Calc(&PidYawSpeed);
		YawCurrent = (short)inttoshort[1];

		break;

	case Gimbal_Test_Mode:
		PidPitchAidPos.SetPoint = TD_Calculate(&PitchTD,GimbalPitchPos);
		PidPitchAidPos.ActualValue = Gimbal.Pitch.Gyro;
		PidPitchAidSpeed.SetPoint = -PID_Calc(&PidPitchAidPos); //+FeedForward_Cal(&PitchPosFF,PidPitchAidPos.SetPoint);
		PidPitchAidSpeed.ActualValue = Gimbal.Pitch.AngularSpeed;
		inttoshort[0] = -(PID_Calc(&PidPitchAidSpeed)); // 旧陀螺仪
		PitchCurrent = -(short)inttoshort[0] * Infantry.motor_pn;
	
		PidYawAidPos.SetPoint = TD_Calculate(&YawTD,GimbalYawPos);
		PidYawAidPos.ActualValue = Gimbal.Yaw.Gyro;
		PidYawAidSpeed.SetPoint = PID_Calc(&PidYawAidPos); //+FeedForward_Cal(&YawPosFF,PidYawAidPos.SetPoint);
		PidYawAidSpeed.ActualValue = Gimbal.Yaw.AngularSpeed;
		inttoshort[1] = -PID_Calc(&PidYawAidSpeed);
		YawCurrent = inttoshort[1];
		break;

	default:
		PitchCurrent = 0;
		YawCurrent = 0;
		break;
	}
}

/**********************************************************************************************************
 *函 数 名: Gimbal_FF_Init
 *功能说明: 云台前馈初始化
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void Gimbal_FF_Init()
{

	PitchPosFF.param[0] = 0;
	PitchPosFF.param[1] = 0;
	PitchPosFF.param[2] = 0;
	PitchPosFF.val = 0;
	PitchPosFF.val_dot = 0;
	PitchPosFF.val_ddot = 0;
	PitchPosFF.OutMax = 5;

	PitchSpeedFF.param[0] = 0;
	PitchSpeedFF.param[1] = 0;
	PitchSpeedFF.param[2] = 0;
	PitchSpeedFF.val = 0;
	PitchSpeedFF.val_dot = 0;
	PitchSpeedFF.val_ddot = 0;
	PitchSpeedFF.OutMax = 5;

	YawPosFF.param[0] = 0;
	YawPosFF.param[1] = 1000;
	YawPosFF.param[2] = 0;
	YawPosFF.val = 0;
	YawPosFF.val_dot = 0;
	YawPosFF.val_dot_out_RC = 0.8;
	YawPosFF.val_ddot = 0;
	YawPosFF.OutMax = 0;

	YawSpeedFF.param[0] = 0;
	YawSpeedFF.param[1] = 0;
	YawSpeedFF.param[2] = 0;
	YawSpeedFF.val = 0;
	YawSpeedFF.val_dot = 0;
	YawSpeedFF.val_ddot = 0;
	YawSpeedFF.OutMax = 0;
}

/**********************************************************************************************************
 *函 数 名: Pid_Yaw_MotorPos_GyroSpeed
 *功能说明: Yaw轴辅瞄双环PID
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void PidGimbalMotor_Init(void)
{
#if Robot_ID == 3
    
#elif Robot_ID == 44
	/********************************************* 44号车 ********************************************************/

	// 辅瞄yaw
	PidYawAidPos.P = 22.0f; // yaw PID位置环（辅瞄）
	PidYawAidPos.I = 0.1f;
	PidYawAidPos.D = 0.0f;
	PidYawAidPos.IMax = 40.0f;
	PidYawAidPos.SetPoint = 0.0f;
	PidYawAidPos.OutMax = 500.0f;
	PidYawAidPos.I_L = 0.2f;
	PidYawAidPos.I_U = 0.4f;
	PidYawAidPos.RC_DF = 0.07f;
	
	PidYawAidSpeed.P = -330.0f; // 32000  yaw速度环PID(辅瞄)
	PidYawAidSpeed.I = -2.0f;
	PidYawAidSpeed.D = 0.0f;
	PidYawAidSpeed.IMax = 1000.0f;
	PidYawAidSpeed.SetPoint = 0.0f;
	PidYawAidSpeed.OutMax = 30000.0f;
	PidYawAidSpeed.I_L = 0.3f;
	PidYawAidSpeed.I_U = 0.7f;
	PidYawAidSpeed.RC_DF = 0.5f;
	// 辅瞄pitch
	PidPitchAidPos.P = -20.0f; // 普通PID位置环(辅瞄)
	PidPitchAidPos.I = -0.06f;
	PidPitchAidPos.D = 0.0f;
	PidPitchAidPos.IMax = 45.0f;
	PidPitchAidPos.SetPoint = 0.0f;
	PidPitchAidPos.OutMax = 500.0f;
	PidPitchAidPos.I_L = 0.2f;
	PidPitchAidPos.I_U = 0.4f;
	PidPitchAidPos.RC_DF = 0.5f;

	PidPitchAidSpeed.P = -310.0f; // 速度环PID（辅瞄）
	PidPitchAidSpeed.I = -2.0f;
	PidPitchAidSpeed.D = 0.0f;
	PidPitchAidSpeed.IMax = 2500.0f;
	PidPitchAidSpeed.SetPoint = 0.0f;
	PidPitchAidSpeed.OutMax = 30000.0f;
	PidPitchAidSpeed.I_L = 18.0f;
	PidPitchAidSpeed.I_U = 42.0f;
	PidPitchAidSpeed.RC_DF = 0.5f;

#endif
}

/**********************************************************************************************************
 *函 数 名: Gimbal_task
 *功能说明: 云台任务函数
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
uint32_t Gimbal_high_water;
UBaseType_t GGGG;
extern TaskHandle_t RCReceiveTask_Handler; // 任务句柄
extern TaskHandle_t PCReceiveTask_Handler; // 任务句柄
//extern uint8_t Remote_Receive_Flag;
// extern uint8_t PC_ReceiveFlag;

void Gimbal_task(void *pvParameters)
{
	portTickType xLastWakeTime;
	const portTickType xFrequency = 1;

	Gimbal_FF_Init();
	vTaskDelay(50);
	while (1)
	{
		xLastWakeTime = xTaskGetTickCount();

		GimbalPos_Set();

		IWDG_Feed();

		vTaskDelayUntil(&xLastWakeTime, xFrequency);

#if INCLUDE_uxTaskGetStackHighWaterMark
		Gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}
