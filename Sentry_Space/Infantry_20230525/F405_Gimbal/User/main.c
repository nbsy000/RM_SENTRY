/**********************************************************************************************************
 * @文件     main.c
 * @说明     主文件
 * @版本  	 V1.0
 * @作者     黄志雄
 * @日期     2019.10
 **********************************************************************************************************/
/**********************************************************************************************************
 * @文件     main.c
 * @说明     主文件
 * @版本  	 V2.0
 * @作者     戴军
 * @日期     2022.6
 **********************************************************************************************************/
#include "main.h"
#include "debug.h"

unsigned volatile long run_time_check = 0; //用于做各种简易计数器计数
short fric_flag = 0;					   //摩擦轮电机初始化标志
RobotInit_Struct Infantry;
char Judge_Lost;
extern short FrictionWheel_speed;
extern short KalMan_doneflag;
extern TD_t PitchTD,YawTD;
/**********************************************************************************************************
 *函 数 名: main
 *功能说明: 主函数
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/

int main()
{

	BSP_Init();
	Robot_Init();
  
  #ifdef DEBUG_MODE
    SEGGER_RTT_Init();
    LOG_CLEAR();
  
  #ifdef JSCOPE_RTT_MODE
    JscopeRTTInit();
  #endif
  
  #endif
  
	startTast();
	vTaskStartScheduler();

	while (1)
	{
	}
}

/**********************************************************************************************************
 *函 数 名: System_Config
 *功能说明: 系统初始化
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void BSP_Init(void)
{
	USART1_Configuration();
	delay_ms(100);
	USART2_Configuration();
	delay_ms(100);
	MicroSwConfigration();
	delay_ms(100);
	SteeringEngine_Configuration();
	delay_ms(100);
	OS_TICK_Configuration();
	delay_ms(100);
	CAN1_Configuration();
	delay_ms(100);
	CAN2_Configuration();
	delay_ms(100);
//	IWDG_Config(IWDG_Prescaler_128 ,625);
//	delay_ms(100);
    VOFA_USART_Configuration();
	delay_ms(3000);
//  My_GPIO_Init();
	SteeringEngine_Set(Infantry.MagOpen);
	delay_ms(100);
	COUNTER_Configuration();
}
/**********************************************************************************************************
 *函 数 名: System_Init
 *功能说明: 系统参数初始化
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void Robot_Init(void)
{
  global_debugger.imu_debugger.recv_msgs_num[0] = 0;
  global_debugger.imu_debugger.recv_msgs_num[1] = 0;
	ZeroCheck_Init();
	Infantry_Init();
	Pid_ChassisPosition_Init();
	PidGimbalMotor_Init();
  Gimbal_FF_Init();
  TD_Init(&YawTD,20000,0.02);
  TD_Init(&PitchTD,10000,0.015);
	Pid_BodanMotor_Init();
	Pid_Friction_Init();
  while (!checkIMUOn()); //检查IMU是否开启
  INS_Init();
  ICM20602_init(&IMUReceive, &IMUData);
	RC_Rst();
}

/**********************************************************************************************************
 *函 数 名: Infantry_Init
 *功能说明: 步兵参数初始化
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void Infantry_Init(void)
{

#if Robot_ID == 3
	/***************************************** 3 号车 **************************************************************/
	Infantry.Yaw_init = 2750; //  3号车
	Infantry.Pitch_init = 7440;
	Infantry.MagOpen = 830;
	Infantry.MagClose = 1900;
	Infantry.Solo_Yaw_init = 7715;
	Infantry.PitchMotorID = 0x206;
	Infantry.YawMotorID = 0x205;
	Infantry.FricMotorID[0] = 0x202;
	Infantry.FricMotorID[1] = 0x203;
	Infantry.BodanMotorID = 0x201;
	Infantry.pitch_max_motor = 35;
	Infantry.pitch_min_motor = -13;
	Infantry.pitch_max_gyro = 31;
	Infantry.pitch_min_gyro = -13;
	Infantry.gyro_pn = 1;
	Infantry.motor_pn = 1;

#elif Robot_ID == 4
	/***************************************** 4 号车 **************************************************************/
	Infantry.Yaw_init = 3475; // 4号车
	Infantry.Pitch_init = 4737;
	Infantry.MagOpen = 1000;
	Infantry.MagClose = 2170;
	Infantry.Solo_Yaw_init = 20;
	Infantry.PitchMotorID = 0x205;
	Infantry.YawMotorID = 0x206;
	Infantry.FricMotorID[0] = 0x202;
	Infantry.FricMotorID[1] = 0x201;
	Infantry.BodanMotorID = 0x203;
	Infantry.pitch_max_motor = 39;
	Infantry.pitch_min_motor = -13;
	Infantry.pitch_max_gyro = 39;
	Infantry.pitch_min_gyro = -14;
	Infantry.gyro_pn = -1; //给送入的陀螺仪pitch施加系数使其方向正确
	Infantry.motor_pn = 1;

#elif Robot_ID == 14
	/***************************************** 14 号车 **************************************************************/
	Infantry.Yaw_init = 4747; // 4728        // 14号车
	Infantry.Pitch_init = 2035;
	Infantry.MagOpen = 1900;
	Infantry.MagClose = 750;
	Infantry.Solo_Yaw_init = 20;
	Infantry.PitchMotorID = 0x205;
	Infantry.YawMotorID = 0x206;
	Infantry.FricMotorID[0] = 0x203;
	Infantry.FricMotorID[1] = 0x201;
	Infantry.BodanMotorID = 0x202;
	Infantry.pitch_max_motor = 37;
	Infantry.pitch_min_motor = -15;
	Infantry.pitch_max_gyro = 36;
	Infantry.pitch_min_gyro = -15;
	Infantry.gyro_pn = 1;
	Infantry.motor_pn = 1;

#elif Robot_ID == 5
	/***************************************** 5 号车 **************************************************************/
	Infantry.Yaw_init = 3757; // 1022斜着走   //2050直着走   // 5号车  //3757
	Infantry.Pitch_init = 6154;
	Infantry.MagOpen = 2150;
	Infantry.MagClose = 1000;
	Infantry.Solo_Yaw_init = 20;
	Infantry.PitchMotorID = 0x206;
	Infantry.YawMotorID = 0x205;
	Infantry.FricMotorID[0] = 0x202;
	Infantry.FricMotorID[1] = 0x201;
	Infantry.BodanMotorID = 0x203;
	Infantry.pitch_max_motor = 32;
	Infantry.pitch_min_motor = -14;
	Infantry.pitch_max_gyro = 32;
	Infantry.pitch_min_gyro = -14;
	Infantry.gyro_pn = 1;  //  陀螺仪安装正负
	Infantry.motor_pn = 1; //  电机安装正负

#elif Robot_ID == 44
	/***************************************** 44 号车 **************************************************************/
	Infantry.Yaw_init = 6111; // 44号车
	Infantry.Pitch_init = 698;
	Infantry.MagOpen = 1900;
	Infantry.MagClose = 750;
	Infantry.Solo_Yaw_init = 20;
	Infantry.PitchMotorID = 0x207;
	Infantry.YawMotorID = 0x205;
	Infantry.FricMotorID[0] = 0x202;
	Infantry.FricMotorID[1] = 0x203;
	Infantry.BodanMotorID = 0x204;
	Infantry.pitch_max_motor = 36;
	Infantry.pitch_min_motor = -11;
	Infantry.pitch_max_gyro = 36;
	Infantry.pitch_min_gyro = -11;
	Infantry.gyro_pn = 1;
	Infantry.motor_pn = 1;

#endif
}

///**********************************************************************************************************
// *函 数 名: delay_ms
// *功能说明: ms延时
// *形    参: 无
// *返 回 值: 无
// **********************************************************************************************************/
//void delay_ms(unsigned long t)
//{
//	int i;
//	for (i = 0; i < t; i++)
//	{
//		int a = 10300;
//		while (a--)
//			;
//	}
//}
//void delay_us(unsigned long t)
//{
//	int i;
//	for (i = 0; i < t; i++)
//	{
//		int a = 37;
//		while (a--)
//			;
//	}
//}
/**********************************************************************************************************
 *函 数 名: Offline_Check_task
 *功能说明: 掉线检测任务
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
uint32_t Offline_Check_high_water;
extern Disconnect Robot_Disconnect;
void Offline_Check_task(void *pvParameters)
{
	while (1)
	{
    TaskCheck(OfflineCheckTask);
		/*电机\IMU掉线检测*/
		if(Robot_Disconnect.YawMotor_DisConnect>10||Robot_Disconnect.PitchMotor_DisConnect>10||Robot_Disconnect.Gyro_DisConnect>20)
		{
		    Robot_Stop();
		}
		else
		{
				Robot_Recover();
		}
		Robot_Disconnect.Gyro_DisConnect++;
		Robot_Disconnect.PitchMotor_DisConnect++;
		Robot_Disconnect.YawMotor_DisConnect++;

		/*发射机构掉线 */
		if (Robot_Disconnect.Friction_DisConnect[0] > 10 || Robot_Disconnect.Friction_DisConnect[1] > 10 || Robot_Disconnect.Pluck_DisConnect > 10)
		{
//			Shoot_Stop();
		}
		else
		{
//			Shoot_Recover();
		}
		Robot_Disconnect.Friction_DisConnect[0]++;
		Robot_Disconnect.Friction_DisConnect[1]++;
		Robot_Disconnect.Pluck_DisConnect++;

		/*遥控器掉线检测*/
//		if (Robot_Disconnect.RC_DisConnect > 10)
//		{
//			RC_Rst();
//		}
//		Robot_Disconnect.RC_DisConnect++;

		/*底盘板或者裁判系统掉线检测*/
		if (Robot_Disconnect.F105_DisConect > 15 || Judge_Lost == 1)
		{
			F105_Rst();
		}
		Robot_Disconnect.F105_DisConect++;

		/* PC暂断 */
		if (Robot_Disconnect.PC_DisConnect > 10)
		{
		}
		Robot_Disconnect.PC_DisConnect++;
		IWDG_Feed();
		vTaskDelay(5); // 5
#if INCLUDE_uxTaskGetStackHighWaterMark
		Offline_Check_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}
