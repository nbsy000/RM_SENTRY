/**********************************************************************************************************
 * @文件     ActionTask.c
 * @说明     动作任务
 * @版本  	 V1.0
 * @作者     黄志雄
 * @日期     2020.1
 **********************************************************************************************************/
/**********************************************************************************************************
 * @文件     ActionTask.c
 * @说明     模式切换任务
 * @版本  	 V2.0
 * @作者     戴军
 * @日期     2022.4
 **********************************************************************************************************/
#include "main.h"
/*--------------内部变量-------------------*/
short Mouse_Key_Flag;
short waitFlag[10] = {10, 0, 0, 0, 0, 0, 0, 0, 0, 0};
short SpeedMode = 1; // 摩擦轮射速挡位
short ctrl_rising_flag, pre_key_ctrl, v_rising_flag, pre_key_v, c_rising_flag, pre_key_c, pre_key_e, e_rising_flag, x_rising_flag, pre_key_x, Press_Key_x_Flag, v_rising_flag, pre_key_b;
short q_rising_flag, b_rising_flag, f_rising_flag, g_rising_flag, pre_key_f, mouse_Press_l_rising_flag, pre_mouse_l, r_rising_flag, pre_key_r, z_rising_flag, pre_key_z, Press_Key_z_Flag;
short pre_key_q, pre_key_g, pre_v_rising_flag; // 上一v按键按下状态
short v_high_flag;
char q_flag, f_flag, HighFreq_flag;
char k_slow;
char smallBuff_flag;
short Turning_flag;		// 小陀螺标志
char Graphic_Init_flag; // 图形初始化标志
char Budan, Buff_flag;
short PC_Sendflag;
float Buff_Yaw_Motor;
char Buff_Init;
// 弹仓盖舵机 持续时间
short SteeringEngineDelay = 0;
/*---------------结构体--------------------*/
Status_t Status;
NAV_t NAV_car;
/*--------------外部变量-------------------*/
extern RC_Ctl_t RC_Ctl;
extern unsigned char magazineState;
extern Pid_Typedef PidBodanMotorPos, PidBodanMotorSpeed;
extern RobotInit_Struct Infantry;
extern F105_Typedef F105;
extern F405_typedef F405;
extern PC_Receive_t PC_Receive;
extern float Bodan_Pos;
extern Gimbal_Typedef Gimbal;
extern float MirocPosition; // 控制拨盘旋转
extern short FrictionWheel_speed;
extern RobotInit_Struct Infantry;
extern float Onegrid;
extern PCSendData pc_send_data;
extern ShootTask_typedef Shoot;
/**********************************************************************************************************
*函 数 名: Status_Act
*功能说明: 步兵车动作函数，分底盘，云台，射击三部分
					 底盘： 1)  Chassis_Act_Cal(Remote rc,Key key)                                 底盘正常模式
									2)  Chassis_SelfProtect_Cal(Remote rc,Key key)                         自我保护模式（小陀螺）
									3)	Chassis_Solo_Cal(Remote rc,Key key)																 单挑模式
									4)  Chassis_Powerdown_Cal()                                            底盘锁死模式

					 云台： 1)  Gimbal_Act_Cal(Remote rc,Mouse mouse,PC_Recv_t *Pc_RecvData)       云台正常模式
									2)  Gimbal_Armor_Cal(PC_Recv_t *Pc_Recv, RC_Ctl_t *RC_clt)             辅助射击模式
									3)  Gimbal_ShenFu_Cal(PC_Recv_t *Pc_Recv, RC_Ctl_t *RC_clt)            击打神符模式
									4)  Gimbal_Powerdown_Cal()                                             云台掉电模式

					 射击   1)  Shoot_Check_Cal();                                                 检录模式使用
							····2)  Shoot_Fire_Cal();                                                  正常射击模式
									3)  Shoot_Armor_Cal();                                                 辅助射击模式
									4)  Shoot_ShenFu_Cal();                                                能量机关模式
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Status_Act(void)
{
	SetInputMode(RC_Ctl.rc);
	switch (Status.ControlMode)
	{
	case Control_RC_Mode:
		Remote_Process(RC_Ctl.rc);
		break;
	case Control_MouseKey_Mode:
		Mouse_Key_Process(RC_Ctl);
		break;
	case Control_Powerdown_Mode:
		Powerdown_Process();
		break;
	case Control_PC_Mode:
		PC_Process(RC_Ctl.rc);
		break;
	default:
		break;
	}

	if (Mouse_Key_Flag == 3)
	{
		MouseKey_Act_Cal(RC_Ctl);
	}
	F405Can1Send(&F405); // 发送405信息
}

/**********************************************************************************************************
 *函 数 名: SetInputMode
 *功能说明: 控制模式选择
 *形    参: rc
 *返 回 值: 无
 **********************************************************************************************************/
void SetInputMode(Remote rc)
{
	switch (rc.s1)
	{
	case 1:
		Status.ControlMode = Control_RC_Mode;
		break;
	case 3:
		//		Status.ControlMode = Control_MouseKey_Mode;
		Status.ControlMode = Control_PC_Mode;
		break;
	case 2:
		Status.ControlMode = Control_Powerdown_Mode;
		SteeringEngine_Set(Infantry.MagOpen);

		break;
	}
	//	Tx2_Off_Test(rc);	//遥控器控制tx2关机判断
}

/**********************************************************************************************************
 *函 数 名: Powerdown_Process
 *功能说明: 断电模式
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void Powerdown_Process()
{
	if (Mouse_Key_Flag != 1)
	{
		Mouse_Key_Flag = 1;
		Budan = 1;
		q_flag = 0;
		f_flag = 0;
		F405.SuperPowerLimit = 0;
		HighFreq_flag = 0;
		Buff_Init = 0;
	}
	Status.ChassisMode = Chassis_Powerdown_Mode;
	Status.GimbalMode = Gimbal_Powerdown_Mode;
	Status.ShootMode = Shoot_Powerdown_Mode;
}


/**********************************************************************************************************
 *函 数 名: Remote_Process
 *功能说明: 遥控器模式选择
 *形    参: rc
 *返 回 值: 无
 **********************************************************************************************************/

void Remote_Process(Remote rc)
{
	if (Mouse_Key_Flag != 2)
	{
		Mouse_Key_Flag = 2;
		Budan = 0;
	}

	if (rc.s2 == 3) // 正常模式
	{
		Buff_Init = 0;
		Status.GimbalMode = Gimbal_Act_Mode;
		Status.ChassisMode = Chassis_Powerdown_Mode;
		if((RC_Ctl.rc.ch1 - 1024) > 300)
			Status.ShootMode = Shoot_Check_Mode;
		else
			Status.ShootMode = Shoot_Powerdown_Mode;
		SteeringEngine_Set(Infantry.MagClose);
	}

	if (rc.s2 == 1) // 云台测试模式
	{
		Status.GimbalMode = Gimbal_Armor_Mode;
		Status.ChassisMode = Chassis_Powerdown_Mode;
		if((RC_Ctl.rc.ch1 - 1024) > 300)
			Status.ShootMode = Shoot_Tx2_Mode;
		else
			Status.ShootMode = Shoot_Powerdown_Mode;
		SteeringEngine_Set(Infantry.MagOpen);
	}

	if (rc.s2 == 2) // 检录模式 单独发射机构
	{
		Status.GimbalMode = Gimbal_Act_Mode;
		Status.ChassisMode = Chassis_Act_Mode;
			Status.ShootMode = Shoot_Powerdown_Mode;
		SteeringEngine_Set(Infantry.MagOpen);
	}

}

/**********************************************************************************************************
*函 数 名: MouseKey_Act_Cal
*功能说明: 键鼠模式
		   w,s,a,d          前进左右后退
		   q								辅瞄，自己打弹（按住）
					 e             		切换优先级
		   r                抬头   Pitch抬升  方便换电池
		   f                飞坡模式
					 g								拨盘反转一格（按一下）
					 shift+z					大符(先按紧shift,再按一下z进入/退出,然后就可松开shift，避免误触)
		   shift+x          小符(先按紧shift,再按一下x进入/退出,然后就可松开shift，避免误触)
		   c                高低射频切换
		   v                缓慢移动（按住）
					 b								图形初始化
		   shift            使用超级电容（按住）
		   ctrl             小陀螺(按住)
		   mouse.press_r    （按住）辅瞄，操作手自己打弹
		   mouse.press_l    单击点射(单发)/长按连发
					 鼠标滚轮         辅瞄微调

*形    参: RC_Ctl
*返 回 值: 无
**********************************************************************************************************/
extern float Inte_z;
extern float GimbalPitchPos;
extern ShootTask_typedef Shoot;
extern int Shoot_Init_flag;
float k_onegrid = 0.01f;
float k_single = 0.9f;
short aaaa = 500;
char ARMOR_FLAG = 0;
char Last_shift;
char shift_press_flag, shift_flag;
// extern char switchPriority;
extern FuzzyPID FuzzyAidPidPitchPos;
extern Pid_Typedef PidPitchAidSpeed;
extern Gyro_Typedef GyroReceive; // 陀螺仪数据
int intshort[4];
short pitchcurrent;
void MouseKey_Act_Cal(RC_Ctl_t RC_Ctl)
{
	static int waitb = 0;

	/******************************拨盘反转 g键*****************************************/
	g_rising_flag = RC_Ctl.key.g - pre_key_g;
	pre_key_g = RC_Ctl.key.g;
	if (g_rising_flag == 1)
	{
		Shoot.ReverseRotation = 1;
		Shoot.ShootContinue = 0;
		g_rising_flag = 0;
	}

	/******************************图形界面初始化 b键*****************************************/
	b_rising_flag = RC_Ctl.key.b - pre_key_b;
	pre_key_b = RC_Ctl.key.b;
	if (RC_Ctl.key.b == 1)
	{

		F405.Graphic_Init_Flag = 1;
		Graphic_Init_flag = 1;
	}

	else
	{
		//			waitb++;
		//			if (waitb > 5)
		//			{
		F405.Graphic_Init_Flag = 0;
		Graphic_Init_flag = 0;
		//				waitb = 0;
		//			}
	}

	/******************************自我保护模式(小陀螺) ctrl键*****************************************/
	if (Status.GimbalMode != Gimbal_BigBuf_Mode && Status.GimbalMode != Gimbal_SmlBuf_Mode)
	{
		ctrl_rising_flag = RC_Ctl.key.ctrl - pre_key_ctrl;
		pre_key_ctrl = RC_Ctl.key.ctrl;
		if (ctrl_rising_flag == 1)
			Turning_flag++;
		if (RC_Ctl.key.ctrl == 1) // 还没有被设置模式或者一直按着的情况
		{
			if (Status.ChassisMode == Chassis_Act_Mode || Status.ChassisMode == Chassis_SelfProtect_Mode)
				Status.ChassisMode = Chassis_SelfProtect_Mode;
		}
		else if (Status.ChassisMode == Chassis_SelfProtect_Mode)
		{
			Status.ChassisMode = Chassis_Act_Mode;
		}
	}
	/******************************切换优先级 e键*****************************************/
	e_rising_flag = RC_Ctl.key.e - pre_key_e;
	pre_key_e = RC_Ctl.key.e;
//	if (RC_Ctl.key.e == 1)
//	{
//		pc_send_data.change_priority_flag = 1;
//	}
//	else
//	{
//		pc_send_data.change_priority_flag = 0;
//	}

	/********************************超级电容控制**********************************************/
	if (RC_Ctl.key.shift == 1)
	{
		F405.SuperPowerLimit = 2; // 使用超级电容
	}
	else
	{
		F405.SuperPowerLimit = 0; // 不使用超级电容
	}

	/****************************** 辅瞄自行打弹模式 q键*****************************************/

	q_rising_flag = RC_Ctl.key.q - pre_key_q;
	pre_key_q = RC_Ctl.key.q;
	if (Status.ShootMode != Shoot_Powerdown_Mode)
	{
		if (q_rising_flag == 1 /*&& (Status.GimbalMode == Gimbal_Armor_Mode
								|| Status.GimbalMode == Gimbal_BigBuf_Mode
								|| Status.GimbalMode == Gimbal_SmlBuf_Mode)*/
			&& Status.ShootMode == Shoot_Fire_Mode)
		{
			//			Status.GimbalMode = Gimbal_Armor_Mode;
			Status.ShootMode = Shoot_Tx2_Mode;
			q_flag = 1;
		}
		else if (q_rising_flag == 1 && Status.ShootMode == Shoot_Tx2_Mode && Status.GimbalMode != Gimbal_Powerdown_Mode)
		{
			//			Status.GimbalMode = Gimbal_Act_Mode;
			Status.ShootMode = Shoot_Fire_Mode;
			q_flag = 0;
		}
	}

	/******************************高低射频切换 c键*****************************************/
	c_rising_flag = RC_Ctl.key.c - pre_key_c;
	pre_key_c = RC_Ctl.key.c;
	if (c_rising_flag == 1)
	{
		HighFreq_flag = !HighFreq_flag;
	}

	/******************************飞坡模式 f键*****************************************/
	//	f_rising_flag = RC_Ctl.key.f - pre_key_f;
	//	pre_key_f = RC_Ctl.key.f;
	//	if (f_rising_flag == 1)
	//	{
	//		if (!f_flag)
	//		{
	//			Status.GimbalMode = Gimbal_Jump_Mode;
	//			Status.ChassisMode = Chassis_Jump_Mode;
	//			f_flag = 1;
	//		}
	//		else
	//		{
	//			Status.GimbalMode = Gimbal_Act_Mode;
	//			Status.ChassisMode = Chassis_Act_Mode;
	//			f_flag = 0;
	//		}
	//	}

	/******************************缓慢移动模式 v键*****************************************/

	if (RC_Ctl.key.v == 1)
	{
		Inte_z = 0;
	}

	///******************************大符模式 z键*****************************************/
	z_rising_flag = RC_Ctl.key.z - pre_key_z;
	pre_key_z = RC_Ctl.key.z;
	if (z_rising_flag == 1 && RC_Ctl.key.shift == 1)
	{
		if (Status.GimbalMode == Gimbal_BigBuf_Mode) //
		{
			Status.GimbalMode = Gimbal_Act_Mode;
			//            Status.ShootMode = Shoot_Fire_Mode;
			Buff_Init = 0;
		}
		else
		{
			if (Buff_Init == 0)
			{
				Buff_Yaw_Motor = Gimbal.Yaw.Gyro;
				Buff_Init = 1;
			}
			Status.GimbalMode = Gimbal_BigBuf_Mode;
			// Status.ShootMode = Shoot_Tx2_Mode;
		}
	}

	//		/******************************Small  Buff 打小符 x键*****************************************/
	x_rising_flag = RC_Ctl.key.x - pre_key_x;
	pre_key_x = RC_Ctl.key.x;
	if (x_rising_flag == 1 && RC_Ctl.key.shift == 1)
	{
		if (Status.GimbalMode != Gimbal_SmlBuf_Mode)
		{
			if (Buff_Init == 0)
			{
				Buff_Yaw_Motor = Gimbal.Yaw.Gyro;
				Buff_Init = 1;
			}
			Status.GimbalMode = Gimbal_SmlBuf_Mode;
			// Status.ShootMode = Shoot_Tx2_Mode;
		}
		else
		{
			Buff_Init = 0;
			Status.GimbalMode = Gimbal_Act_Mode;
			//            Status.ShootMode = Shoot_Fire_Mode;
		}
	}

	/******************************鼠标按键射击*******************************************/
	mouse_Press_l_rising_flag = RC_Ctl.mouse.press_l - pre_mouse_l;
	pre_mouse_l = RC_Ctl.mouse.press_l;

	if (Status.ShootMode != Shoot_Fire_Mode && Status.ShootMode != Shoot_Powerdown_Mode && RC_Ctl.mouse.press_l == 1)
	{
		Status.ShootMode = Shoot_Fire_Mode;
	} // 手动射击抢断自瞄

	if (Status.ShootMode == Shoot_Fire_Mode)
	{
		if (RC_Ctl.mouse.press_l == 1 && Shoot.ReverseRotation == 0)
		{
			waitFlag[5]++;
			if ((waitFlag[5] < 50) && (waitFlag[5] > 0)) // 短按     //延时可以调一调
			{
				if (mouse_Press_l_rising_flag == 1)
				{
					waitFlag[5] = 0;
					if (Shoot.HeatControl.IsShootAble == 1) // Heat_Limit
					{
						if (ABS(Bodan_Pos - PidBodanMotorPos.SetPoint) < PluckThreholdPos && F405.Fric_Flag == 1)
						{
							MirocPosition -= Onegrid;
							Shoot.ShootContinue = 0;
						}
					}
				}
			}
			if (waitFlag[5] > 100) // 长按
			{

				if (Shoot.HeatControl.IsShootAble == 1)
				{
					if (ABS(Bodan_Pos - PidBodanMotorPos.SetPoint) < PluckThreholdPos && F405.Fric_Flag == 1)
					{
						Shoot.ShootContinue = 1;
					}
				}
			}
		}
		else if (RC_Ctl.mouse.press_l == 0 && Shoot.ReverseRotation == 0) // 不按
		{
			waitFlag[5] = 0;
			if (Shoot.ShootContinue == 1 || ABS(Bodan_Pos - PidBodanMotorPos.SetPoint) < 1000)
			{
				PidBodanMotorPos.SetPoint = Bodan_Pos;
			}
			Shoot.ShootContinue = 0;
		}
	}
	/******************************辅助射击控制（辅瞄） 右键*****************************************/
	if (RC_Ctl.mouse.press_r == 1 && (Status.GimbalMode != Gimbal_BigBuf_Mode && Status.GimbalMode != Gimbal_SmlBuf_Mode))
	{
		Status.GimbalMode = Gimbal_Armor_Mode;
		//        if(RC_Ctl.mouse.press_l != 1)
		//        Status.ShootMode = Shoot_Tx2_Mode;
	}
	else if (/*q_flag == 0 && */ Status.GimbalMode == Gimbal_Armor_Mode)
	{
		Status.GimbalMode = Gimbal_Act_Mode;
		//        Status.ShootMode = Shoot_Fire_Mode;
	}
}

/**********************************************************************************************************
 *函 数 名: Mouse_Key_Process
 *功能说明: 键鼠模式初始化
 *形    参: RC_Ctl
 *返 回 值: 无
 **********************************************************************************************************/
void Mouse_Key_Process(RC_Ctl_t RC_Ctl)
{
	/******************************初次参数初始*****************************************/
	if (Mouse_Key_Flag != 3)
	{
		SpeedMode = 1;
		Mouse_Key_Flag = 3;
		magazineState = 0x00; // Close
		SteeringEngine_Set(Infantry.MagClose);
		Status.GimbalMode = Gimbal_Act_Mode;
		Status.ChassisMode = Chassis_Act_Mode;
		Status.ShootMode = Shoot_Fire_Mode;
		Budan = 0;
	}

	// 控制补弹开关弹舱盖
	if (Budan)
	{
		SteeringEngine_Set(Infantry.MagOpen);
	}
	else
	{
		SteeringEngine_Set(Infantry.MagClose);
	}

	//  控制摩擦轮开关
	if (RC_Ctl.rc.s2 == 2)
	{
		Status.ShootMode = Shoot_Powerdown_Mode;
	}
	else
	{
		if (Status.ShootMode != Shoot_Tx2_Mode)
			Status.ShootMode = Shoot_Fire_Mode;
	}
}

/**********************************************************************************************************
 *函 数 名: PC_Process
 *功能说明: 自动模式选择
 *形    参: rc
 *返 回 值: 无
 **********************************************************************************************************/
uint8_t Gimbal_State_Update;
uint8_t Chassis_State_Update;
void PC_Process(Remote rc)
{
	if (Mouse_Key_Flag != 5)
	{
		Mouse_Key_Flag = 5;
		Budan = 0;
		NAV_car.NAV_State = BEFOREGAME; // 比赛开始前
		NAV_car.mode_update_time = NAV_car.pc_mode_time = GetTime_s();
	}

	Navigation_State(); // 导航策略

	if (rc.s2 == 3) // 底盘模式
	{
		Buff_Init = 0;
		Status.GimbalMode = Gimbal_PC_Mode;
		Status.ChassisMode = Chassis_PC_Mode; // Chassis_Powerdown_Mode;
		Status.ShootMode = Shoot_Powerdown_Mode;
		SteeringEngine_Set(Infantry.MagClose);
	}

	if (rc.s2 == 1) // 全模式
	{
		Status.GimbalMode = Gimbal_PC_Mode;
		Status.ChassisMode = Chassis_PC_Mode;
		Status.ShootMode = Shoot_PC_Mode;
		SteeringEngine_Set(Infantry.MagOpen);
	}

	if (rc.s2 == 2) // 导航测试
	{
		Chassis_Gimbal_Shoot_State(NAV_STATE, NAV_STATE, NAV_STATE);
		Status.GimbalMode = Gimbal_PC_Mode;
		Status.ChassisMode = Chassis_PC_Mode;
		Status.ShootMode = Shoot_Powerdown_Mode;
		SteeringEngine_Set(Infantry.MagOpen);
	}
}

/**********************************************************************************************************
 *函 数 名: Navigation_State
 *功能说明: 导航决策
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void Navigation_State()
{

	/**********************全局状态***************************/
	/*
		不管当前状态，直接转跳到目标状态
		遥控器：用于调试
		部分裁判系统数据：前哨战无。。。
		部分云台手指令：回巡逻区。。。。
	*/
	
	if ((RC_Ctl.rc.ch0 - 1024) > 300||// 右右
		(F105.JudgeReceive_info.commd_keyboard == 'S') )
	//	||	F105.JudgeReceive_info.defend_flag)	 
		NAV_car.NAV_State = TO_PATROL;		// 去巡逻区
	if ((RC_Ctl.rc.ch0 - 1024) < -300) // 右左
		NAV_car.NAV_State = TO_OUTPOST; // 去前哨站
	
	if ((RC_Ctl.rc.ch1 - 1024) > 300) // 右上s
		NAV_car.NAV_State = TO_HIGHLAND; // 去高地
	if ((RC_Ctl.rc.ch1 - 1024) < -300) //右下
		NAV_car.NAV_State = TO_SOURCE;	// 去资源岛
	
	if ((RC_Ctl.rc.ch2 - 1024) > 300 ||// 左右
		(F105.JudgeReceive_info.commd_keyboard == 'D') )	 
		NAV_car.NAV_State = PATROL;	 //巡逻区
	if ((RC_Ctl.rc.ch2 - 1024) < -300) // 左左
		NAV_car.NAV_State = OUTPOST;		 //  前哨站

	if ((RC_Ctl.rc.ch3 - 1024) > 300)	 // 左上
		NAV_car.NAV_State = HIGHLAND;	 // 高地
	if ((RC_Ctl.rc.ch3 - 1024) < -300) // 左下
		NAV_car.NAV_State = SOURCE;		 // 资源岛
	
	/***********************决策状态转换**************************/
	NAV_State_Invert();		
	
	/***********************决策状态执行**************************/
	NAV_State_Act();	
	
	
	/********************状态更新时的过渡状态***************/
	if (NAV_car.Last_NAV_State != NAV_car.NAV_State)
	{
		NAV_car.mode_update_time = GetTime_s();
	}
	
	if (GetTime_s() - NAV_car.mode_update_time < NAV_car.MODE_UPDATE_INTERVAL) // 过渡时间1.0s
	{
		Chassis_Gimbal_Shoot_State(STOP_STATE, STOP_STATE, STOP_STATE);
		NAV_car.NAV_x = 0;
		NAV_car.NAV_y = 0;
		NAV_car.NAV_w = 0;
	}

	NAV_car.Last_NAV_State = NAV_car.NAV_State;
}


/**********************************************************************************************************
*函 数 名: NAV_State_Invert
*功能说明: 决策转态转换
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void NAV_State_Invert()
{
	switch (NAV_car.NAV_State)
	{
	case BEFOREGAME: // 比赛开始前  	
		if (F105.JudgeReceive_info.is_game_start)  
		{
			if( GetTime_s() - NAV_car.game_start_time > NAV_car.GAME_START_INTERVAL)//延时10s，等工程
				NAV_car.NAV_State =  TO_OUTPOST;// 去高地或前哨战
		}
		else
		{
			NAV_car.game_start_time = GetTime_s();
			if( GetTime_s() - NAV_car.pc_mode_time > NAV_car.PC_MODE_INTERVAL)//2min内裁判系统数据没有开始比赛
					NAV_car.NAV_State = PATROL;
		}
		break;
		
	case OUTPOST: // 前哨站
		if( F105.JudgeReceive_info.commd_keyboard == 'W' )//云台手按键W
				NAV_car.NAV_State = TO_SOURCE;
		break;
		
	case PATROL: // 巡逻区
		break;
	
	case SOURCE: // 资源岛
		break;
	
	case HIGHLAND: // 高地
		if( F105.JudgeReceive_info.enemy_defend_flag)//敌方前哨战无
				NAV_car.NAV_State = TO_OUTPOST;
		break;
	
	case TO_OUTPOST: // 去前哨站
		if (NAV_car.NAV_Path_State == FINISHED)
			NAV_car.NAV_State = OUTPOST; // 前哨站
		break;
		
	case TO_SOURCE: // 去资源导
		if (NAV_car.NAV_Path_State == FINISHED)
			NAV_car.NAV_State = SOURCE; // 哨站
		break;
		
	case TO_PATROL: // 去巡逻区
		if (NAV_car.NAV_Path_State == FINISHED)
			NAV_car.NAV_State = PATROL; // 巡逻区
		break;
		
	case TO_HIGHLAND: // 去高地
		if (NAV_car.NAV_Path_State == FINISHED)
			NAV_car.NAV_State = HIGHLAND; // 高地
		break;
		
	case PATROL_SAFE: // 前哨战还在前的巡逻区状态
		break;
	
	case TEST1: // 测试路线1
		break;
	case TEST2: // 路线测试2
		break;
	
	default:
		NAV_car.NAV_State = BEFOREGAME;//比赛开始前
		break;
	}
}

/**********************************************************************************************************
*函 数 名: NAV_State_Act
*功能说明: 决策状态执行，主要根据当前状态给底盘云台发射机构赋状态
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void NAV_State_Act()
{
	switch (NAV_car.NAV_State)
	{
	case BEFOREGAME: // 比赛开始前
		Chassis_Gimbal_Shoot_State(STOP_STATE, STOP_STATE, STOP_STATE);
		break;
	case OUTPOST: // 前哨站
		Chassis_Gimbal_Shoot_State(STOP_STATE, ARMOR_STATE, ARMOR_STATE);
		break;
	case PATROL: // 巡逻区
		Chassis_Gimbal_Shoot_State(PROTECT_STATE, ARMOR_STATE, ARMOR_STATE);
		break;
	case SOURCE: // 资源岛
		Chassis_Gimbal_Shoot_State(STOP_STATE, ARMOR_STATE, ARMOR_STATE);
		break;
	case HIGHLAND: // 高地
		Chassis_Gimbal_Shoot_State(STOP_STATE, ARMOR_STATE, ARMOR_STATE);
		break;
	case TO_OUTPOST: // 去前哨站
		Chassis_Gimbal_Shoot_State(NAV_STATE, NAV_STATE, NAV_STATE);
		break;
	case TO_SOURCE: // 去资源导
		Chassis_Gimbal_Shoot_State(NAV_STATE, NAV_STATE, NAV_STATE);
		break;
	case TO_PATROL: // 去巡逻区
		Chassis_Gimbal_Shoot_State(NAV_STATE, NAV_STATE, NAV_STATE);
		break;
	case TO_HIGHLAND: // 去高地
		Chassis_Gimbal_Shoot_State(NAV_STATE, NAV_STATE, NAV_STATE);
		break;
	case PATROL_SAFE: // 前哨战还在前的巡逻区状态
		Chassis_Gimbal_Shoot_State(PROTECT_STATE, ARMOR_STATE, ARMOR_STATE);
		break;
	case TEST1: // 测试路线1
		break;
	case TEST2: // 路线测试2
		break;
	default:
		Chassis_Gimbal_Shoot_State(STOP_STATE, STOP_STATE, STOP_STATE);
		break;
	}
}


/**********************************************************************************************************
 *函 数 名: Chassis_Gimbal_Shoot_State
 *功能说明: 模式选择任务
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void Chassis_Gimbal_Shoot_State(int Chassis_Mode, int Gimbal_Mode, int Shoot_Mode)
{
	NAV_car.Chassis_PC_State = (enum CHASSIS_GIMBAL_SHOOT_STATE)Chassis_Mode;
	NAV_car.Gimbal_PC_State = (enum CHASSIS_GIMBAL_SHOOT_STATE)Gimbal_Mode;
	NAV_car.Shoot_PC_State = (enum CHASSIS_GIMBAL_SHOOT_STATE)Shoot_Mode;
}


/**********************************************************************************************************
*函 数 名: NAV_Init
*功能说明: 导航相关参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void NAV_Init()
{
	NAV_car.GAME_START_INTERVAL = 10.0f;
	NAV_car.MODE_UPDATE_INTERVAL = 1.0f;
	NAV_car.PC_MODE_INTERVAL = 120.0f;
}

/**********************************************************************************************************
 *函 数 名: ModeChoose_task
 *功能说明: 模式选择任务
 *形    参: pvParameters
 *返 回 值: 无
 **********************************************************************************************************/
uint32_t ModeChoose_high_water;
extern char Chassis_ID;
void ModeChoose_task(void *pvParameters)
{
	NAV_Init();
	while (1)
	{
		Status_Act();
		IWDG_Feed();
		vTaskDelay(5);

#if INCLUDE_uxTaskGetStackHighWaterMark
		ModeChoose_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}
