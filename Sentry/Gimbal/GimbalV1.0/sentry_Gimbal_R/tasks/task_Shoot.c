#include "main.h"
#include "task_Shoot.h"

_2006_motor_t BodanMotor;
_2006_motor_t FrictionMotor[2];

extern int16_t Shoot_init_flag;
extern State_t Sentry_State;
extern block_disconnect_t block_disconnect;
extern float distance;
uint8_t Heat_ShootAbleFlag = 0; //底盘给过来的热量控制能否打弹标志位 
uint8_t is_game_start = 0;        // 底盘给过来的判断是否正式开始比赛的标志位

//用于摩擦轮和拨弹之间延时的
uint8_t Shoot_ModeUpdate_Flag = 0;                //标志上次模式和当前模式是否一致
uint8_t BodanDelay_OVER = 0;                        //标志拨弹延时是否已经结束
int32_t BodanDelay_Tick = 0;                        //拨弹开启的延时计数
volatile uint8_t Shoot_LastMode = 0x00; //存放上次进入shoot_task时的状态
#define BodanDelay_Threshold 200                    //存放拨弹延时的上门限

//堵转检测相关
uint8_t block_bullet_flag;     //卡弹标志 ，置位时表示发生了卡弹，需要做拨盘反转处理
int16_t block_bullet_tick = 0; //卡弹计数器，如果卡弹计数超过一定的秒数，则认为卡弹，置位卡弹标志并执行反转
uint8_t block_bullet_cnt = 0;  //-----目前没用

//这几个变量是每个shoot_Act共用的，以后也许可以改成一个结构体，然后各自的Act用各自的成员值
uint32_t shootDelayTick;         //记录单连发模式下，每两发子弹之间的实时时间间隔
float testInc = 29510.0f;        //26910.0f;//拨一颗弹丸需给到pos的增量值
uint32_t delayTick_oneShot = 50;//60 //走掉一颗弹丸间隔的时间(ms)------弹频
float bodanLastPos;              //存放上次单发结束时的拨弹电机位置值
float RC_Bodan;

//******************内部函数声明***********************************************//
static void PID_Shoot_Init(void); //初始化bodan电机的PID参数
static void Shoot_RC_Act(void);
static void Shoot_PC_Act(void);
static void Shoot_SLEEP_Act(void);
static void Shoot_DEBUG_Act(void);
static void aiming(void);
static void Shoot_RC_PID_Cal(void);
static void Shoot_PC_PID_Cal(void);
inline static void Shoot_SLEEP_PID_Cal(void);
static void Block_Check(void);
static void Firing_Freq_Cal(void);

int test1 =0;
void task_Shoot(void *parameter)
{
    //第一次进任务的复位
    PID_Shoot_Init();

    while (1)
    {
				//状态位更新
        Shoot_ModeUpdate_Flag = (Shoot_LastMode != Sentry_State.Shoot_R_Mode); 
        Shoot_LastMode = Sentry_State.Shoot_R_Mode;                              //更新上次状态
        BodanDelay_Tick = (Shoot_ModeUpdate_Flag) ? (0) : (BodanDelay_Tick + 1);  //根据状态是否切换，判断延时tick是清零还是递增
        BodanDelay_OVER = (BodanDelay_Tick >= BodanDelay_Threshold);                //根据延时时间和门限值，判定延时是否结束
			
				//距离判断射频
				Firing_Freq_Cal();

				//掉线检测处理
				Shoot_Disconnect_Act();			
			
				//模式判断
        if (Sentry_State.Shoot_R_Mode == Shoot_R_PC)					 Shoot_PC_Act();
        else if (Sentry_State.Shoot_R_Mode == Shoot_R_RC)      Shoot_RC_Act();
        else if (Sentry_State.Shoot_R_Mode == Shoot_R_SLEEP)   Shoot_SLEEP_Act();
        else if (Sentry_State.Shoot_R_Mode == Shoot_R_DEBUG)   Shoot_DEBUG_Act();
				
				//堵转检测
				Block_Check();
				
				//发拨弹电流
				Bodan_Can1Send(BodanMotor.I_Set,FrictionMotor[0].I_Set, FrictionMotor[1].I_Set); 
							
        vTaskDelay(2);

    }
}


/**
  * @brief  视觉瞄准函数：在激活状态下，如果实际云台姿态和CV给来的云台姿态（即敌人位置）
  * @param  None
  * @retval None
  */
float pitch_thresh = 2.0f;
float yaw_thresh = 3.0f;
static void aiming(void)
{
    extern uint8_t CV_Shoot_ABLE; //判定视觉方面是否能够打子弹

    if (Gimbal_R.armor_state==ARMOR_AIMED &&
        ABS(Gimbal_R.aim_Pitch - MotoPitch.actualAngle) <= pitch_thresh &&
        ABS(Gimbal_R.aim_Yaw - MotoYaw.actualAngle) <= yaw_thresh)
		{
        CV_Shoot_ABLE = 1;
		}
    else CV_Shoot_ABLE = 0;
}


/**
  * @brief  DEBUG模式下的打弹控制
  * @param  None
  * @retval None
  */
int16_t test_fric_speed0 = 5000, test_fric_speed1 = 5000;
int8_t Bodan_Enable_DEBUG = 1;
int8_t Shoot_Enable_DEBUG = 0;
uint8_t Bodan_Speed_Debug = 0;
int16_t delayTick_debug = 500;
float debug_nowangle,debug_setpointangle;
extern RC_Ctl_t RC_Ctl;
static void Shoot_DEBUG_Act(void)
{
    FrictionWheel_SetSpeed(test_fric_speed0, test_fric_speed1); //*设置摩擦轮
    float fsend;

    shootDelayTick++;
    if (shootDelayTick >= delayTick_debug)
    {
        bodanLastPos = BodanMotor.Angle_Inc; //有减速比啊啊啊啊！___*( ￣皿￣)/#____
        shootDelayTick = 0;                  //清空计数变量值
    }

		if(((RC_Ctl.rc.ch3-1024)>100)||Bodan_Enable_DEBUG)
			BodanMotor.pid_pos.SetPoint = bodanLastPos + testInc;     		//位置值设定为当前值增加一格弹丸的角度
		if(Bodan_Speed_Debug == 0)
			BodanMotor.pid_speed.SetPoint = PID_Calc(&BodanMotor.pid_pos, BodanMotor.Angle_Inc,0,0); //用位置环算速度环设定值
    fsend = PID_Calc(&BodanMotor.pid_speed, BodanMotor.RealSpeed,0,0);                       //用速度环算电流值输出

    BodanMotor.I_Set = (int16_t)LIMIT_MAX_MIN(fsend, BodanCurrentLimit, -BodanCurrentLimit)* Shoot_Enable_DEBUG;
		FrictionMotor[0].I_Set*= Shoot_Enable_DEBUG;
		FrictionMotor[1].I_Set*= Shoot_Enable_DEBUG;

		debug_nowangle = BodanMotor.Angle_Inc%(int)2*testInc;
		debug_setpointangle = (int)BodanMotor.pid_speed.SetPoint%(int)2*testInc;
}


/**
  * @brief  PC控制下的打弹函数
  * @param  None
  * @retval None
  */
extern Gimbal_Typedef Gimbal_R,Gimbal_L;
static void Shoot_PC_Act(void)
{
    FrictionWheel_SetSpeed(FrictionWheel_L_Speed_High, FrictionWheel_R_Speed_High); //*设置摩擦轮
    extern uint8_t CV_Shoot_ABLE;     //判定视觉方面是否能够打子弹
	
    aiming();//瞄准判断
	
		Heat_ShootAbleFlag = 1 ;
    if (CV_Shoot_ABLE  && Heat_ShootAbleFlag)
    {
        shootDelayTick++;
        if (shootDelayTick >= delayTick_oneShot)
        {
            bodanLastPos = BodanMotor.Angle_Inc; //有减速比啊啊啊啊！___*( ￣皿￣)/#____
            shootDelayTick = 0;                  //清空计数变量值
        }

				BodanMotor.pid_pos.SetPoint = bodanLastPos + testInc; //位置值设定为当前值增加一格弹丸的角度
				Shoot_PC_PID_Cal();
    }
    else
        Shoot_SLEEP_PID_Cal();
}

/**
  * @brief  遥控器控制下的打弹，目前没有堵转处理
  * @param  None
  * @retval None
  */
uint8_t Positive_Flag=1;
static void Shoot_RC_Act(void)
{
    FrictionWheel_SetSpeed(FrictionWheel_L_Speed_High, FrictionWheel_L_Speed_High); //*设置摩擦轮
		//底盘给过来的热量控制能否打弹标志位
    if (Heat_ShootAbleFlag)
    {
        shootDelayTick++;
        if (shootDelayTick >= delayTick_oneShot)
        {		
            bodanLastPos = BodanMotor.Angle_Inc; //有减速比啊啊啊啊！___*( ￣皿￣)/#____
            shootDelayTick = 0;                  //清空计数变量值
        }

				BodanMotor.pid_pos.SetPoint = bodanLastPos + testInc; //位置值设定为当前值增加一格弹丸的角度
				Shoot_PC_PID_Cal();
    }
    else
        Shoot_SLEEP_PID_Cal();
}


/**
  * @brief  掉电模式打弹控制
  * @param  None
  * @retval None
  */
static void Shoot_SLEEP_Act(void)
{
	if(Shoot_ModeUpdate_Flag)//模式切换
	{
			BodanMotor.pid_pos.SetPoint = BodanMotor.Angle_Inc;
	}

	FrictionWheel_SetSpeed(FrictionWheel_L_Speed_Off, FrictionWheel_R_Speed_Off); //*关摩擦轮

	Shoot_SLEEP_PID_Cal();
}


/**
  * @brief  自动控制下的打弹PID
  * @param  None
  * @retval None
  */
static void Shoot_PC_PID_Cal(void)
{
    float fsend;
    BodanMotor.pid_speed.SetPoint = PID_Calc(&BodanMotor.pid_pos, BodanMotor.Angle_Inc, 0,0); //
//		BodanMotor.pid_speed.SetPoint = 60*18;
    fsend = PID_Calc(&BodanMotor.pid_speed, BodanMotor.RealSpeed, 0,0); //用速度环算电流值输出
    BodanMotor.I_Set = (int16_t)LIMIT_MAX_MIN(fsend, BodanCurrentLimit, -BodanCurrentLimit);
	  BodanMotor.I_Set = BodanDelay_OVER ?(BodanMotor.I_Set) : (0);
}


/**
  * @brief  掉电模式下的打弹PID
  * @param  None
  * @retval None
  */
inline static void Shoot_SLEEP_PID_Cal(void)
{
		BodanMotor.I_Set = 0;
}


/**
  * @brief  设置摩擦轮转速（油门值）
  * @param  两轮速度值
  * @retval None
  */
void FrictionWheel_SetSpeed(int16_t tmpAccelerator0, int16_t tmpAccelerator1)
{

  //赋值
  extern _2006_motor_t FrictionMotor[2];
  FrictionMotor[0].pid_speed.SetPoint = -LIMIT_MAX_MIN(tmpAccelerator0, 15050, 0);
  FrictionMotor[1].pid_speed.SetPoint = LIMIT_MAX_MIN(tmpAccelerator1, 15050, 0);  ////注意！！！！这个2006的转向是反着的！！！！

  float fsend;
  fsend = PID_Calc(&FrictionMotor[0].pid_speed, FrictionMotor[0].RealSpeed, 0,0); //用速度环算电流值输出
  FrictionMotor[0].I_Set = (int16_t)LIMIT_MAX_MIN(fsend, FrictionCurrentLimit, -FrictionCurrentLimit);
  fsend = PID_Calc(&FrictionMotor[1].pid_speed, FrictionMotor[1].RealSpeed, 0,0 ); //用速度环算电流值输出
  FrictionMotor[1].I_Set = (int16_t)LIMIT_MAX_MIN(fsend, FrictionCurrentLimit, -FrictionCurrentLimit);

}


/**
  * @brief  计算射击频率
  * @param  None
  * @retval None
  */
void Firing_Freq_Cal()
{
	if(distance < 2.0f)//小于3m
		delayTick_oneShot = 45;
	else if(distance < 3.5f)
		delayTick_oneShot = 60;
	else
		delayTick_oneShot = 80;

}

/**
  * @brief  堵转检查的处理
  * @param  None
  * @retval None
  */
void Block_Check()
{
		//先来堵转检测
	block_bullet_tick = ((BodanMotor.pid_pos.SetPoint-BodanMotor.Angle_Inc)>0.5*testInc)? block_bullet_tick++ : 0;
	
	if(block_bullet_tick > 500)//堵转1s
		BodanMotor.I_Set = 0;
}

/**
  * @brief  枪管热量掉线处理
  * @param  None
  * @retval None
  */
void Shoot_Disconnect_Act()
{
		//热量控制的调电检测
		if(Robo_Disconnect.HeatDiscount > 1000)
		{
			delayTick_oneShot = 50;
			Heat_ShootAbleFlag = 1;
		}
		else
			Robo_Disconnect.HeatDiscount ++;
}

/**
  * @brief  拨弹电机pid初始化
  * @param  None
  * @retval None
  */
static void PID_Shoot_Init(void)
{

    FrictionMotor[0].pid_speed.P = 25.0f;//27.0f;
    FrictionMotor[0].pid_speed.I = 0.0f;//1.2f;//0.0f;
    FrictionMotor[0].pid_speed.D = 0.0f;
    FrictionMotor[0].pid_speed.IMax = 800.0f;//100;
    FrictionMotor[0].pid_speed.SetPoint = 0;
		FrictionMotor[0].pid_speed.OutMax = 16000;

    FrictionMotor[1].pid_speed.P = 25.0f;//27.0f;
    FrictionMotor[1].pid_speed.I = 0.0f;//0.0f;
    FrictionMotor[1].pid_speed.D = 0.0f;
    FrictionMotor[1].pid_speed.IMax = 800.0f;//100;//0.0f;
    FrictionMotor[1].pid_speed.SetPoint = 0;	
		FrictionMotor[1].pid_speed.OutMax = 16000;
	
    BodanMotor.pid_speed.P = 8.5f;//1.5f;
    BodanMotor.pid_speed.I = 0.5f;
    BodanMotor.pid_speed.D = 0.0f;//0.5f;
    BodanMotor.pid_speed.IMax = 600.0f;
		BodanMotor.pid_speed.OutMax = 10000;

    BodanMotor.pid_pos.P = 0.35f;//0.28f;
    BodanMotor.pid_pos.I = 0.00f;//0.0f;
    BodanMotor.pid_pos.D = 0.2f;//10.0f;
    BodanMotor.pid_pos.IMax = 0.0f;//0.0f;
    BodanMotor.pid_pos.SetPoint = BodanMotor.Angle_ABS;
		BodanMotor.pid_pos.OutMax = 900000;

}
