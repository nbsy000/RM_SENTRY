#include "main.h"

/*----------------------------------内部变量---------------------------*/
int Chassis_R = 327;//底盘半径为327mm
int Wheel_R = 83;//轮子半径83mm
float reRatio = 3591/187;//3508电机减速比
float aimRSpeed[4] = {0};//四个全向轮的目标转速 mm/s

uint8_t Chassis_Last_State = Chassis_SLEEP;//底盘的上一刻状态
uint8_t Chassis_Updata_Flag = 0;


short WheelCurrentSend[4];
short Set_Jump[4] = {0};
float Current_Change[4] = {0};//电流环增量
float Current_f[4] = {0};//输出电流f
float Flow[4] = {0};//实际电流f
float speed[4] = {0};//实际速度f

//功率限制系数 PowerLimit
short Actual_P_max;						//实际最大功率

//启动有关，后面要加
float ABSready_flag=0; 
float Goready_flag=0;
float T_ABS=1000.0f;//刹车时间
float T_SETUP=800.0f;//启动时间
/*----------------------------------结构体-----------------------------*/
Chassis_Motor_t ChassisMotor[4];//四个电机结构体
ChassisState_t chassis;

Pid_Typedef Pid_Current[4];
Pid_Typedef pidChassisWheelSpeed[4];

/*----------------------------------外部变量---------------------------*/
extern uint16_t PathDotIndex;
extern uint8_t Anomalies_tag;

extern short MyBuffering_Energy;
extern float output_fil;
extern float Input[4];
extern float Output[4];
extern char slow_flag;

extern char output_filter;
extern uint8_t defend_flag;


/**********************************************************************************************************
*函 数 名: Chassis_task
*功能说明: 底盘任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
uint32_t Chassis_high_water;

extern uint8_t JudgeReveice_Flag;
extern TaskHandle_t SDCardTask_Handler;
extern uint8_t is_game_start;

void Chassis_task(void *pvParameters)
{
	portTickType xLastWakeTime;
	const portTickType xFrequency = 2;
	
	chassis.NavigatePathNum = 1;//路径选择
	chassis.PC_State = BEFOREGAME;//还未开始比赛
	while (1) 
	{
		xLastWakeTime = xTaskGetTickCount();
				
		//状态更新
		Chassis_Updata_Flag = (Chassis_Last_State != Sentry_State.Chassis_Mode);
		
		//根据底盘状态值确定进哪个执行函数
    if (Sentry_State.Chassis_Mode == Chassis_Patrol) Chassis_Patrol_Act();
    else if (Sentry_State.Chassis_Mode == Chassis_RC) Chassis_RC_Act();
		else if (Sentry_State.Chassis_Mode == Chassis_Protect) Chassis_Protect_Act();
		else if (Sentry_State.Chassis_Mode == Chassis_DEBUG) Chassis_Patrol_Act(); 
    else Chassis_SLEEP_Act();	
		
		ChassisMotorSend(WheelCurrentSend[0],WheelCurrentSend[1],WheelCurrentSend[2],WheelCurrentSend[3]); 
			
		//功率限制
		PowerLimit();
			
		//读取当前状态
		Chassis_Last_State = Sentry_State.Chassis_Mode;
		
		IWDG_Feed();//喂狗	
		
		VOFA_Send();
		
		vTaskDelayUntil(&xLastWakeTime,xFrequency);//2ms		
		 
#if INCLUDE_uxTaskGetStackHighWaterMark
        Chassis_high_water = uxTaskGetStackHighWaterMark(NULL);//用于观测该任务剩余的栈空间
#endif
    }
}

/**********************************************************************************************************
*函 数 名: PowerLimit
*功能说明: 功率限制
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
#define MaxOutPower  250     //最大输出功率限制
#define   K_P        3.2*1E5   //功率系数
float	test_KP = 0;//K_P的系数测试

float test_W_Chassis_t1 = 0,test_W_Chassis_t2 = 0;	//测试估算功率修正
float W_Grad[10] = {0.98f,0.98f,0.98f,0.95f,0.95f,0.9f,0.9f,0.9f,0.85f,0.85f};//削减系数
short DescendFlag;
float ExcPower;					//外加功率，主要为了尽可能使用能量
float EnergyMargin = 20.0f;		//留有的缓存能量余量
float My_P_max;				//计算的当前最大功率
void PowerLimit(void)
{
	float W_Chassis_t = 0;//底盘功率
	static float PowerMargin  = 150.0f;   //瓦，功率超出量，便于快速起步
	static float k_ExcessPower;
		
	//设置最大功率
	//剩余能量少，为防止超功率，改低功率
	if(JudgeReceive.remainEnergy <= EnergyMargin)
	{
		My_P_max = JudgeReceive.MaxPower*0.8f;
	}
	else 
	{
		// 裁判系统每0.02s一次
		ExcPower = PowerMargin*(JudgeReceive.remainEnergy-EnergyMargin)/(60.0f-EnergyMargin);
		My_P_max = LIMIT_MAX_MIN(ExcPower+JudgeReceive.MaxPower, MaxOutPower, JudgeReceive.MaxPower);
	}
		
	//接收电流滤波  输出的电流值存在了Flow()中 ---目前其实不需要
	Current_Filter_Excu();
		
  //计算当前功率  电流x系数x速度 = 功率  这个主要是由于电流于力矩成线性关系，系数KP测试结果
	for(int i=0;i<4;i++)
	{
		W_Chassis_t += ABS(WheelCurrentSend[i]*ChassisMotor[i].RealSpeed);//功率计算
	}
	
	test_KP = W_Chassis_t/JudgeReceive.realChassispower;
	W_Chassis_t /= K_P;//比例系数
	test_W_Chassis_t1 = W_Chassis_t;	
	
	
	//当前功率超过最大功率 分次削减速度,最多削减10次
	DescendFlag = 0;
	while(W_Chassis_t > My_P_max && DescendFlag < 10)
	{
		W_Chassis_t = 0;
		
		for(int i=0;i<4;i++)//通过削减速度减小电流值
		{
			//乘以衰减系数
			pidChassisWheelSpeed[i].SetPoint *= W_Grad[DescendFlag];			
			
			//速度环+电流环
			Pid_Current[i].SetPoint = PID_Calc(&pidChassisWheelSpeed[i],ChassisMotor[i].RealSpeed);
//			Current_Filter_Excu();
//			Current_Set_Jump();
//			Current_Change[i] = PID_Calc(&Pid_Current[i],Flow[i]);
//			if(Set_Jump[i] == 0)
//			{
//				Current_f[i] += Current_Change[i];
//			}
//			else if(Set_Jump[i] == 1)
//			{
//				Current_f[i] = Pid_Current[i].SetPoint;
//			}
			WheelCurrentSend[i] = Pid_Current[i].SetPoint;
			
		W_Chassis_t += ABS(WheelCurrentSend[i]*ChassisMotor[i].RealSpeed);//功率计算		
		}	
		
		W_Chassis_t /= K_P;
		DescendFlag++;
	}
	
	test_W_Chassis_t2 = W_Chassis_t;	
}
/*********************************************************************************************************
*函 数 名: Chassis_RC_Act
*功能说明: 使用遥控器控制前后左右移动，没有小陀螺，小陀螺单独写了 s1 中  s2 上  
						没有底盘跟随,此时的前后左右是以底盘为坐标系
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Chassis_RC_Act(void)
{
	static float chassis_yaw_init;
	//首次进入该模式，进行变量重初始化，主要为了防止出现奇怪问题
	if(Chassis_Last_State != Chassis_RC)
	{
		Chassis_Last_State = Chassis_RC;
		chassis_yaw_init = chassis.Alpha;
	}		
	
	float Theta = (chassis.Alpha - chassis_yaw_init)/360.0f*2*PI;//θ角为与y方向的夹角，具体符号取值与运动分解的θ的取向有关
//	Theta = 0.0f;
	float CosTheTa=arm_cos_f32(Theta);
	float SinTheTa=arm_sin_f32(Theta);
	
	//底盘方位自适应，即以云台的朝向为正前方，使用的是电机角   下面这些系数都是凭感觉给的，想要多少速度转速自己改
	chassis.carSpeedx = (RC_Ctl.rc.ch1-1024)*8*SinTheTa + (RC_Ctl.rc.ch0-1024)*8*CosTheTa; //364-1684  即660*8最大为5280mm/s
	chassis.carSpeedy = (RC_Ctl.rc.ch1-1024)*8*CosTheTa - (RC_Ctl.rc.ch0-1024)*8*SinTheTa;	
	
	if((RC_Ctl.rc.ch3-1024)>50)
		chassis.carSpeedw = (RC_Ctl.rc.ch3-1074)*0.2;
	else if((RC_Ctl.rc.ch3-1024)<-50)
		chassis.carSpeedw = (RC_Ctl.rc.ch3-974)*0.2;
	else 
		chassis.carSpeedw = 0 ;
	
	//计算每个电机的目标转速
	aimRSpeed[0] = -chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) + chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[1] = +chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) + chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[2] = +chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) + chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[3] = -chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) + chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	
	//PID计算每个电机的控制电流
	Pid_SpeedCurrent(aimRSpeed);
	
}

/*********************************************************************************************************
*函 数 名: Chassis_Protect_Act
*功能说明: 使用遥控器控制小陀螺移动，ch3控制 s1 下  s2 上
						让底盘一边小陀螺一边移动，移动的坐标系是以大Yaw轴,
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
float Theta = 0.0f;
int init_encoder[4];
int average;
int dif_encoder[4];
int test_w = 25;
float K_W = 0.5f;
uint8_t test_enable = 0;
void Chassis_Protect_Act(void)
{
	//首次进入该模式，进行变量重初始化，主要为了防止出现奇怪问题
	if(Chassis_Last_State != Chassis_Protect)
	{
		init_encoder[0]=ChassisMotor[0].Encoder_INC;
		init_encoder[1]=ChassisMotor[1].Encoder_INC;
		init_encoder[2]=ChassisMotor[2].Encoder_INC;
		init_encoder[3]=ChassisMotor[3].Encoder_INC;
		Chassis_Last_State = Chassis_Protect;
	}	
	
//	Theta = -(Motor_9025.Yaw_init - Motor_9025.multiAngle)/36000.0f*2*PI;//θ角为与y方向的夹角，具体符号取值与运动分解的θ的取向有关
//	float CosTheTa=arm_cos_f32(Theta);
//	float SinTheTa=arm_sin_f32(Theta);
//	
//	//底盘方位自适应，即以云台的朝向为正前方，使用的是电机角   下面这些系数都是凭感觉给的，想要多少速度转速自己改
//	chassis.carSpeedx = (RC_Ctl.rc.ch1-1024)*5*SinTheTa + (RC_Ctl.rc.ch0-1024)*5*CosTheTa; //364-1684  即660*5最大为3300mm/s
//	chassis.carSpeedy = (RC_Ctl.rc.ch1-1024)*5*CosTheTa - (RC_Ctl.rc.ch0-1024)*5*SinTheTa;	
	
	average = 0;
	for(int i=0;i<4;i++)
	{
		ChassisMotor[i].total_x=ChassisMotor[i].Encoder_INC-init_encoder[i];
		average += (ChassisMotor[i].total_x/4);
	}
	
	for(int i=0;i<4;i++)
		dif_encoder[i] = average - ChassisMotor[i].total_x;
	
	if((RC_Ctl.rc.ch3-1024)>50)
		chassis.carSpeedw = (RC_Ctl.rc.ch3-1074)*0.2;
	else if((RC_Ctl.rc.ch3-1024)<-50)
		chassis.carSpeedw = (RC_Ctl.rc.ch3-974)*0.2;
	else 
		chassis.carSpeedw = 0 ;
		
	
	//计算每个电机的目标转速
	aimRSpeed[0] = -chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R-test_enable*K_W*dif_encoder[0];
	aimRSpeed[1] = +chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R-test_enable*K_W*dif_encoder[1];
	aimRSpeed[2] = +chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R-test_enable*K_W*dif_encoder[2];
	aimRSpeed[3] = -chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R-test_enable*K_W*dif_encoder[3];
	
	//PID计算每个电机的控制电流
	Pid_SpeedCurrent(aimRSpeed);
	
}



/**
  * @brief  Chassis_Patrol_Act 自动模式
  * @param  None
  * @retval None
  */
float offset_x,offset_y;
float Patrol_X = 1500;
int8_t Patrol_Cnt = 0;
int count_cnt = 2500;
int Patrol_Last_Cnt;
int Patrol_flag;
void Chassis_Patrol_Act()
{
		//首次进入该模式，进行变量重初始化，主要为了防止出现奇怪问题
	if(Chassis_Updata_Flag)
	{
		chassis.totalX = PathInforms[1].PathDotsInfoArray[0].x;
		chassis.totalY = PathInforms[1].PathDotsInfoArray[0].y;	

		offset_x  = PathInforms[1].PathDotsInfoArray[0].x-PCReceive.now_x;
		offset_y  = PathInforms[1].PathDotsInfoArray[0].y-PCReceive.now_y;
		
		chassis.Yangle = Gyro_Chassis.YAW_ABS;//以当前陀螺仪角度为前,即以当前时刻的底盘坐标系作为里程计坐标系
		
		chassis.Init_X = chassis.Radar_totalX;
		chassis.Init_Y = chassis.Radar_totalY;
		
		chassis.N_Yaw_angle_init = chassis.Alpha;
		
		Chassis_Last_State = Chassis_Patrol;
		chassis.PC_State = PATROL_SAFE;//还未开始比赛		
	}		
	
//	if(((RC_Ctl.rc.ch0-1024)>300))//右右
//		chassis.PC_State = PATROL_SAFE;//前哨站
//	if(((RC_Ctl.rc.ch0-1024)<-300))//右左
//		chassis.PC_State = PATROL;//巡逻区
//	if(((RC_Ctl.rc.ch1-1024)>300)||(JudgeReceive.commd_keyboard=='W'))//右上
//		chassis.PC_State = TOPATH1;//去前哨站
//	if((RC_Ctl.rc.ch1-1024)<-300||(JudgeReceive.commd_keyboard=='S')||defend_flag)//右上
//		chassis.PC_State = PATROL;	//回巡逻区
//	if((RC_Ctl.rc.ch3-1024)>300)//左上
//		chassis.PC_State = TEST1;
//	if((RC_Ctl.rc.ch3-1024)<-300)//左下
//		chassis.PC_State = TEST2;	

//	if(((RC_Ctl.rc.ch2-1024)>300))//左左
//		
//	if(((RC_Ctl.rc.ch2-1024)>300))//左右
//	
	
//	if(chassis.Barrier_flag)//遇到障碍物
//	{
////		Chassis_SLEEP_Act();
//	}

//	else
	
	if(Patrol_Cnt != Patrol_Last_Cnt)
				Patrol_flag = 1;
	if(1)
	{
		switch(chassis.PC_State)
		{
			case BEFOREGAME://比赛开始前	
			
				if(is_game_start)//比赛开始
				{
						if(count_cnt == 0)
						{
							chassis.PC_State = TOPATH1;//前往前哨站
							chassis.NAV_State = CONTINUED;//路径开始
							PathDotIndex = 1;
						}
						count_cnt -- ;
				}
				break;
				
			case TOPATH1://路径1去
				
					chassis.NavigatePathNum = 1;//去前哨战
					NAV_PATH_Act(RADAR);
			
					if((chassis.NAV_State == FINISHED)||(PathDotIndex>PathInforms[chassis.NavigatePathNum].PathDotsNum-20))//路径完成
							chassis.PC_State = OUTPOST;//前哨站
					break;
				
			case OUTPOST://前哨站
				Outpost_Act();
			
//				if(JudgeReceive.commd_keyboard=='T')
//				{
//						chassis.PC_State = SOURCETO;//回巡逻区
//						chassis.NAV_State = CONTINUED;//路径开始
//						PathDotIndex = 1;
//				}
				
				if(defend_flag)//前哨站被炸
				{
						chassis.PC_State = BACKPATH1;//回巡逻区
						chassis.NAV_State = CONTINUED;//路径开始
						PathDotIndex = 1;
				}
				break;
			
			case BACKPATH1://路径回
				
				chassis.NavigatePathNum = 2;//回
				NAV_PATH_Act(RADAR);
			
				if((chassis.NAV_State == FINISHED)||(PathDotIndex>PathInforms[chassis.NavigatePathNum].PathDotsNum-20))//路径完成
					chassis.PC_State = PATROL;//前哨站
				break;
				
			case PATROL_SAFE://前哨战还在时巡逻区（独立于比赛开始状态）
				Patrol_Safe_Act();
				break;
				
			case PATROL://巡逻区
				Patrol_Act();
				break;	
			
			case TEST1://测试路线1
				chassis.NavigatePathNum = 3;//测试路线1
				NAV_PATH_Act(RADAR);
				break;
			
			case TEST2://测试路线2
				chassis.NavigatePathNum = 4;//测试路线1
				NAV_PATH_Act(RADAR);
				break;
		}
	}
	
	chassis.Last_PC_State = chassis.PC_State;
	Patrol_Last_Cnt = Patrol_Cnt;

}


/*********************************************************************************************************
*函 数 名:
*功能说明: 
*形    参: 
*返 回 值: 
**********************************************************************************************************/
void Chassis_Patrol_Act1(uint8_t Mode)
{
	static float offset_x,offset_y;
	//首次进入该模式，进行变量重初始化，主要为了防止出现奇怪问题
	if(Chassis_Updata_Flag)
	{
		chassis.totalX = PathInforms[chassis.NavigatePathNum].PathDotsInfoArray[0].x;
		chassis.totalY = PathInforms[chassis.NavigatePathNum].PathDotsInfoArray[0].y;	

		offset_x  = PathInforms[chassis.NavigatePathNum].PathDotsInfoArray[0].x-PCReceive.now_x;
		offset_y  = PathInforms[chassis.NavigatePathNum].PathDotsInfoArray[0].y-PCReceive.now_y;
		
		PathDotIndex = 1;//保证每次重新进入自动模式后从第一个点开始
		chassis.Yangle = Gyro_Chassis.YAW_ABS;//以当前陀螺仪角度为前,即以当前时刻的底盘坐标系作为里程计坐标系
		
		Chassis_Last_State = Chassis_Patrol;
	}		
	
	//读取当前角度
	chassis.nowYangle = Gyro_Chassis.YAW_ABS;//当前角度
	float Theta = (chassis.nowYangle-chassis.Yangle)*2*PI/360.0f;
//	float Theta = chassis.Alpha - chassis.Yangle;
	float CosTheTa=arm_cos_f32(Theta);
	float SinTheTa=arm_sin_f32(Theta);
	
	//计算里程计
	//瞬时速度
	chassis.insW = -Wheel_R/(reRatio*Chassis_R)*(ChassisMotor[0].Inencoder+ChassisMotor[1].Inencoder+ChassisMotor[2].Inencoder+ChassisMotor[3].Inencoder);
	chassis.insX = Wheel_R*(ChassisMotor[1].Inencoder-ChassisMotor[3].Inencoder)*PI/(8192*reRatio);
	chassis.insY = Wheel_R*(ChassisMotor[2].Inencoder-ChassisMotor[0].Inencoder)*PI/(8192*reRatio);
	
	//XY方向累加，该值为当前实际值，单位mm
	chassis.totalX += chassis.insX*CosTheTa - chassis.insY*SinTheTa;
	chassis.totalY += chassis.insX*SinTheTa + chassis.insY*CosTheTa;
	
	//数据传入
	if(Mode == ENCODER)
	{
		chassis.CurrentState.X = chassis.totalX;
		chassis.CurrentState.Y = chassis.totalY;
	}
	else
	{
		chassis.CurrentState.X = PCReceive.now_x+offset_x;
		chassis.CurrentState.Y = PCReceive.now_y+offset_y;	
	}
	
//	chassis.CurrentState.Alpha = Theta;//????????------------这个偏航角到底指那个方向
	
	//进行导航控制
	chassisNavigate(&chassis, &PathInforms[chassis.NavigatePathNum]);
	
	//数据传出
	chassis.aimVx = chassis.AimState.Vx;
	chassis.aimVy = chassis.AimState.Vy;
	
	//速度转换，转换成以车为坐标系的速度（不考虑大Yaw轴方向）
	chassis.carSpeedx = chassis.aimVy*SinTheTa + chassis.aimVx*CosTheTa;
	chassis.carSpeedy = chassis.aimVy*CosTheTa - chassis.aimVx*SinTheTa;
	chassis.carSpeedw = 0 ;
	
	//分解到每个电机的转速
	aimRSpeed[0] = -chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[1] = +chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[2] = +chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[3] = -chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	
	//PID计算每个电机的控制电流
	Pid_SpeedCurrent(aimRSpeed);
	
}



/*********************************************************************************************************
*函 数 名: Chassis_Patrol_Act2
*功能说明: 自动小陀螺
*形    参: 
*返 回 值: 
**********************************************************************************************************/
float offset_x = 0.0f;//小陀螺偏移距离
float offset_y = 0.0f;//小陀螺偏移距离
void Chassis_Patrol_Act2(void)
{
	extern ActReceive_t PCReceive;
	
	static float A;
	static float W;
	int angle;
	static int count = 0;
	
	//首次进入该模式，进行变量重初始化，主要为了防止出现奇怪问题
	if(Chassis_Last_State != Chassis_Patrol)
	{
		chassis.Init_X = chassis.Radar_totalX;
		chassis.Init_Y = chassis.Radar_totalY;
		Chassis_Last_State = Chassis_Patrol;
		count = 0;
	}
	
	float Theta = (Motor_9025.Yaw_init - Motor_9025.multiAngle)/36000.0f*2*PI;//θ角为与y方向的夹角，具体符号取值与运动分解的θ的取向有关
	float CosTheTa=arm_cos_f32(Theta);
	float SinTheTa=arm_sin_f32(Theta);
	
	offset_x = chassis.Init_X - PCReceive.now_x;
	offset_y = chassis.Init_Y - PCReceive.now_y;
	
	if(count==0)
	{
		A = (rand()%11-5)*4;//-20到20
		W = (rand()%2+1)/6.0f*2*PI;
		count = 6000;
	}
	count--;
	chassis.carSpeedw = 65+A*(float)arm_sin_f32(W*((float)count/2000.0f)) ;
		
	//底盘方位自适应，即以云台的朝向为正前方，使用的是电机角   下面这些系数都是凭感觉给的，想要多少速度转速自己改
	chassis.carSpeedx = offset_x*SinTheTa + offset_y*CosTheTa; //364-1684  即660*5最大为3300mm/s
	chassis.carSpeedy = offset_y*CosTheTa - offset_x*SinTheTa;
	
	chassis.carSpeedx = 0.0f;
	chassis.carSpeedy = 0.0f;
	
	//分解到每个电机的转速
	aimRSpeed[0] = -chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[1] = +chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[2] = +chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[3] = -chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	
	//PID计算每个电机的控制电流
	Pid_SpeedCurrent(aimRSpeed);
	
}

/*********************************************************************************************************
*函 数 名: Chassis_Patrol_Act_mmWave
*功能说明: 毫米波雷达模式
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
//extern float distancex,distancey,distancez,angle;
void Chassis_Patrol_Act_mmWave(void)
{
	//首次进入该模式，进行变量重初始化，主要为了防止出现奇怪问题
	if(Chassis_Last_State != Chassis_Patrol)
	{
		Chassis_Last_State = Chassis_Patrol;
	}
	

	//Theta = -angle/360.0f*2*PI;//θ角为与y方向的夹角，具体符号取值与运动分解的θ的取向有关
	float CosTheTa=arm_cos_f32(Theta);
	float SinTheTa=arm_sin_f32(Theta);
	
	float Wave_x;
	float Wave_y;
	
	//毫米波雷达跟踪
//	if(distancey > MIN_DST_Y && distancey < MAX_DST_Y)
//	{
//		if(distancey > DST_Y + 0.1)
//			Wave_y = 1000;
//		else if(distancey < DST_Y - 0.1)
//			Wave_y = -1000;
//		else
//			Wave_y = 0;
//	}
//	else
//		Wave_y = 0;
	
	Wave_x = 0;
//	
	
	//底盘方位自适应，即以云台的朝向为正前方，使用的是电机角   下面这些系数都是凭感觉给的，想要多少速度转速自己改
	chassis.carSpeedx = Wave_y*SinTheTa + Wave_x*CosTheTa; //364-1684  即660*5最大为3300mm/s
	chassis.carSpeedy = Wave_x*CosTheTa - Wave_y*SinTheTa;	
	
	chassis.carSpeedw = 0;//60rmp
		
	//计算每个电机的目标转速
	aimRSpeed[0] = -chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[1] = +chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[2] = +chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[3] = -chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	
	//PID计算每个电机的控制电流
	Pid_SpeedCurrent(aimRSpeed);


}

/*********************************************************************************************************
*函 数 名:
*功能说明: 
*形    参: 
*返 回 值: 
**********************************************************************************************************/
void Chassis_SLEEP_Act(void)
{
	WheelCurrentSend[0] = 0;
	WheelCurrentSend[1] = 0;
	WheelCurrentSend[2] = 0;
	WheelCurrentSend[3] = 0;

}

/*********************************************************************************************************
*函 数 名:
*功能说明: 
*形    参: 
*返 回 值: 
**********************************************************************************************************/
void Chassis_DEBUG_Act(void)
{
	WheelCurrentSend[0] = 0;
	WheelCurrentSend[1] = 0;
	WheelCurrentSend[2] = 0;
	WheelCurrentSend[3] = 0;

}






void NAV_PATH_Act(uint8_t Mode)
{
	//首次进入该模式，进行变量重初始化，主要为了防止出现奇怪问题
	if(chassis.PC_State != chassis.Last_PC_State)
	{	
		PathDotIndex = 1;//保证每次重新进入自动模式后从第一个点开始
	}		
	
	//读取当前角度
	chassis.nowYangle = Gyro_Chassis.YAW_ABS;//当前角度
	float Theta = (chassis.nowYangle-chassis.Yangle)*2*PI/360.0f;
//	float Theta = chassis.Alpha - chassis.Yangle;
	float CosTheTa=arm_cos_f32(Theta);
	float SinTheTa=arm_sin_f32(Theta);
	
	//计算里程计
	//瞬时速度
	chassis.insW = -Wheel_R/(reRatio*Chassis_R)*(ChassisMotor[0].Inencoder+ChassisMotor[1].Inencoder+ChassisMotor[2].Inencoder+ChassisMotor[3].Inencoder);
	chassis.insX = Wheel_R*(ChassisMotor[1].Inencoder-ChassisMotor[3].Inencoder)*PI/(8192*reRatio);
	chassis.insY = Wheel_R*(ChassisMotor[2].Inencoder-ChassisMotor[0].Inencoder)*PI/(8192*reRatio);
	
	//XY方向累加，该值为当前实际值，单位mm
	chassis.totalX += chassis.insX*CosTheTa - chassis.insY*SinTheTa;
	chassis.totalY += chassis.insX*SinTheTa + chassis.insY*CosTheTa;
	
	//数据传入
	if(Mode == ENCODER)
	{
		chassis.CurrentState.X = chassis.totalX;
		chassis.CurrentState.Y = chassis.totalY;
	}
	else
	{
		chassis.CurrentState.X = PCReceive.now_x+offset_x;
		chassis.CurrentState.Y = PCReceive.now_y+offset_y;	
	}
	
//	chassis.CurrentState.Alpha = Theta;//????????------------这个偏航角到底指那个方向
	
	//进行导航控制
	chassisNavigate(&chassis, &PathInforms[chassis.NavigatePathNum]);
	
	//数据传出
	chassis.aimVx = chassis.AimState.Vx;
	chassis.aimVy = chassis.AimState.Vy;
	
	//速度转换，转换成以车为坐标系的速度（不考虑大Yaw轴方向）
	chassis.carSpeedx = chassis.aimVy*SinTheTa + chassis.aimVx*CosTheTa;
	chassis.carSpeedy = chassis.aimVy*CosTheTa - chassis.aimVx*SinTheTa;
	chassis.carSpeedw = 0 ;
	
	//分解到每个电机的转速
	aimRSpeed[0] = -chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[1] = +chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[2] = +chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[3] = -chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	
	//PID计算每个电机的控制电流
	Pid_SpeedCurrent(aimRSpeed);
	
}


/**
  * @brief  Outpost_Act 前哨站策略
  * @param  None
  * @retval None
  */
void Outpost_Act()
{
	if(chassis.PC_State != chassis.Last_PC_State)
	{
		chassis.Init_X = chassis.Radar_totalX;//通过改这个值来改变移动的中心位置
		chassis.Init_Y = chassis.Radar_totalY;
	}
	float Theta = (chassis.Alpha - chassis.N_Yaw_angle_init)/360.0f*2*PI;//θ角为与y方向的夹角，具体符号取值与运动分解的θ的取向有关
	float CosTheTa=arm_cos_f32(Theta);
	float SinTheTa=arm_sin_f32(Theta);
	
	//计算得到底盘x，y，w三向速度大小
	Position_Control(chassis.Init_X,chassis.Init_Y,Theta);
	
	chassis.carSpeedx = 0;
	chassis.carSpeedy = 0;
	chassis.carSpeedw = 0;

	//分解到每个电机的转速
	aimRSpeed[0] = -chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[1] = +chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[2] = +chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[3] = -chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	
	//PID计算每个电机的控制电流
	Pid_SpeedCurrent(aimRSpeed);	
	
}	

/**
  * @brief  Patrol_Act 巡逻区策略
  * @param  None
  * @retval None
  */
float set_Speed = 300;
int MAX_DISTANCE = 500;
float step_x = 50.0f;//0.8m/s
float PC_setx,PC_sety;
void Patrol_Act()
{
	static uint8_t Toword = 1;

	if(chassis.PC_State != chassis.Last_PC_State)
	{
//		chassis.Init_X = chassis.Radar_totalX;//通过改这个值来改变移动的中心位置
//		chassis.Init_Y = chassis.Radar_totalY;
		Patrol_Cnt = 2;
		PC_setx = chassis.Init_X-Patrol_Cnt*Patrol_X-MAX_DISTANCE;
		PC_sety = chassis.Init_Y;
	}
	float Theta = (chassis.Alpha - chassis.N_Yaw_angle_init)/360.0f*2*PI;//θ角为与y方向的夹角，具体符号取值与运动分解的θ的取向有关
	float CosTheTa=arm_cos_f32(Theta);
	float SinTheTa=arm_sin_f32(Theta);
	
	
	if(Motor_9025.aim_flag != SYNE_NONE)//识别到目标，保持原地
	{
			chassis.carSpeedx = 0;
			chassis.carSpeedy = 0;
	}
	else//未识别到目标
	{
			
//			if(chassis.Radar_totalX > ((chassis.Init_X-Patrol_Cnt*Patrol_X)+MAX_DISTANCE-step_x)) PC_setx = (chassis.Init_X-Patrol_Cnt*Patrol_X) - MAX_DISTANCE;
//			if(chassis.Radar_totalX < ((chassis.Init_X-Patrol_Cnt*Patrol_X)-MAX_DISTANCE+step_x)) PC_setx = (chassis.Init_X-Patrol_Cnt*Patrol_X) + MAX_DISTANCE;
			PC_setx = chassis.Init_X-Patrol_Cnt*Patrol_X;	
			PC_sety = chassis.Init_Y;//Y向不动
		
//			Position_Control(PC_setx,PC_sety,Theta);
	}
	
	//计算得到底盘x，y，w三向速度大小
//	if(Patrol_flag)
//	{
//		chassis.carSpeedw = 0;
//		if((chassis.Radar_totalX < ((chassis.Init_X-Patrol_Cnt*Patrol_X)-step_x))&&(chassis.Radar_totalX < ((chassis.Init_X-Patrol_Cnt*Patrol_X)+step_x)))
//				Patrol_flag = 0;
//	}
//	else
		chassis.carSpeedw = 60;
	
		
	
	
	
	//分解到每个电机的转速
	aimRSpeed[0] = -chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[1] = +chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[2] = +chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[3] = -chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	
	//PID计算每个电机的控制电流
	Pid_SpeedCurrent(aimRSpeed);
	
}

/**
  * @brief  Patrol_Safe_Act 前哨战还在前巡逻区策略
  * @param  None
  * @retval None
  */
void Patrol_Safe_Act()
{
	if(chassis.PC_State != chassis.Last_PC_State)
	{
//		chassis.Init_X = chassis.Radar_totalX;//通过改这个值来改变移动的中心位置
//		chassis.Init_Y = chassis.Radar_totalY;
			Patrol_Cnt = 0;
	}
	float Theta = (chassis.Alpha - chassis.N_Yaw_angle_init)/360.0f*2*PI;//θ角为与y方向的夹角，具体符号取值与运动分解的θ的取向有关
	float CosTheTa=arm_cos_f32(Theta);
	float SinTheTa=arm_sin_f32(Theta);
	
	PC_setx = chassis.Init_X-Patrol_Cnt*Patrol_X;
	PC_sety = chassis.Init_Y;
	
	//计算得到底盘x，y，w三向速度大小
//	Position_Control(PC_setx,PC_sety,Theta);
	
	
//	if(Patrol_flag)
//	{
//		chassis.carSpeedw = 0;
//		if((chassis.Radar_totalX < ((chassis.Init_X-Patrol_Cnt*Patrol_X)-step_x))&&(chassis.Radar_totalX < ((chassis.Init_X-Patrol_Cnt*Patrol_X)+step_x)))
//				Patrol_flag = 0;
//	}
//	else
		chassis.carSpeedw = 60;

	//分解到每个电机的转速
	aimRSpeed[0] = -chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[1] = +chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[2] = +chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[3] = -chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	
	//PID计算每个电机的控制电流
	Pid_SpeedCurrent(aimRSpeed);	


}


/**
  * @brief  定位移动控制
  * @param  aim_x 目标x
  * @param  aim_y 目标y
  * @param  Theta 世界坐标系与底盘坐标系的夹角
  * @retval None
  */
void Position_Control(float aim_x,float aim_y,float Theta)
{
	float Error_X = aim_x - chassis.Radar_totalX;
	float Error_Y = aim_y - chassis.Radar_totalY;
	
	chassis.carSpeedx = (Error_Y*arm_sin_f32(Theta)+Error_X*arm_cos_f32(Theta))*4.0f;
	chassis.carSpeedy = (Error_Y*arm_cos_f32(Theta)-Error_X*arm_sin_f32(Theta))*4.0f;

	chassis.carSpeedx = LIMIT_MAX_MIN(chassis.carSpeedx,500,-500);
	chassis.carSpeedy = LIMIT_MAX_MIN(chassis.carSpeedy,500,-500);
}


/**********************************************************************************************************
*函 数 名: Current_Filter_Excu
*功能说明: 将四个轮子的电流反馈值分别滤波
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Current_Filter_Excu(void)
{
	for(int i = 0;i < 4;i++)
	{
		Input[i] = (float)ChassisMotor[i].Current; 
	}
	Fir(Input,Output);
}

/**********************************************************************************************************
*函 数 名: Pid_ChassisWheelInit
*功能说明: 底盘XY向运动PID参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Pid_ChassisWheelInit(void)
{
	
	for(int i=0;i<4;i++)
	{
			pidChassisWheelSpeed[i].P = 5.8f;
			pidChassisWheelSpeed[i].I = 0.3f;
			pidChassisWheelSpeed[i].D = 0.0f;
			pidChassisWheelSpeed[i].IMax = 600.0f;
			pidChassisWheelSpeed[i].SetPoint = 0.0f;	
			pidChassisWheelSpeed[i].OutMax = 16000.0f;	
	}
	
}

/*********************************************************************************************************
*函 数 名: Pid_SpeedCurrent
*功能说明: 计算电机的速度环与电流环
*形    参: aimRSpeed 目标转速
*返 回 值: 无
**********************************************************************************************************/

void Pid_SpeedCurrent(float aimRSpeed[])
{
	//滤波
	/*斜着走*/	
	LowPass_SetChassis(&pidChassisWheelSpeed[0].SetPoint,aimRSpeed[0]);
	LowPass_SetChassis(&pidChassisWheelSpeed[1].SetPoint,aimRSpeed[1]);
	LowPass_SetChassis(&pidChassisWheelSpeed[2].SetPoint,aimRSpeed[2]);
	LowPass_SetChassis(&pidChassisWheelSpeed[3].SetPoint,aimRSpeed[3]);
		
	//速度环+电流环
	int i = 0;
	for(i = 0;i<4;i++)
	{
		Pid_Current[i].SetPoint = PID_Calc(&pidChassisWheelSpeed[i],ChassisMotor[i].RealSpeed);
//		Current_Filter_Excu();//对反馈电流进行滤波
//		Current_Set_Jump();//判断是否需要进行电流环
//		Current_Change[i] = PID_Calc(&Pid_Current[i],Flow[i]);
//		if(Set_Jump[i] == 0)
//		{
//			Current_f[i] += Current_Change[i];
//		}
//		else if(Set_Jump[i] == 1)
//		{
//			Current_f[i] = Pid_Current[i].SetPoint;
//		}
		WheelCurrentSend[i] = Pid_Current[i].SetPoint;
	}
}

