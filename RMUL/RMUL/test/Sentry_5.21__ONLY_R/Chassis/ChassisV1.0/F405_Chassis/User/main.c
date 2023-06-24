#include "main.h"


roboDisconnect Robot_Disconnect;
char Judge_Lost;
extern ChassisState_t chassis;

extern enum CHARGESTATE_Typedef ChargeState;
extern TaskHandle_t ChassisTask_Handler; //任务句柄
extern TaskHandle_t PowerControlTask_Handler; //任务句柄
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
	
	while(SysTick_Config(168000));	
	startTast();
	vTaskStartScheduler();
	while(1)
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
//	DAC1_Init();
	CAN1_Configuration();
	CAN2_Configuration();
	VOFA_USART_Configuration();
	UART4_Configuration();
	TIM2_Configuration();
	COUNTER_Configuration();
	TIM4_Configuration();
//	IWDG_Config(IWDG_Prescaler_64 ,625);
	i2c_init();
	delay_ms(200);//等主控板初始化完成，防止主控板初始化过程中向底盘发送错误数据

}

/**********************************************************************************************************
*函 数 名: System_Init
*功能说明: 系统参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Robot_Init(void)
{
	INS_Init();
	ICM20602_init(&IMUReceive, &IMUData);
	Pid_ChassisWheelInit();
	ChassisYaw_Init();
	PathInit();//路径初始化
	delay_ms(300);
}

/**********************************************************************************************************
*函 数 名: Offline_Check_task
*功能说明: 裁判系统离线
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Judge_Rst()
{
	JudgeReceive.MaxPower=150;
	JudgeReceive.remainEnergy=0;
}
/**********************************************************************************************************
*函 数 名: Offline_Check_task
*功能说明: 掉线检测任务(任务优先级太低，在这里放串口通信会导致数据发不出去)
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
uint32_t Offline_Check_high_water;
extern uint8_t seq;
void Offline_Check_task(void *pvParameters)
{
   while (1) {
    
		 /*云台发送的遥控器检测*/
		if(Robot_Disconnect.RemoteDisconnect>100)
		{
			RC_Rst();
		}
		Robot_Disconnect.RemoteDisconnect++;
		
		/*裁判系统掉线检测*/
		if(Robot_Disconnect.JudgeDisconnect>100)
		{
			Judge_Rst();
			Judge_Lost=1;//需要发送给云台的裁判系统离线标志位
		}else
		{
		  Judge_Lost=0;
		}
		Robot_Disconnect.JudgeDisconnect++;
		
		/*底盘电机掉线检测*/
		for(int i=0;i<4;i++)
		{
				if(Robot_Disconnect.ChassisDisconnect[i]>100)
				{

				}
			Robot_Disconnect.ChassisDisconnect[i]++;
		}			
		
		/*大Yaw轴电机掉线检测*/
		if(Robot_Disconnect.ChassisYawconnect>100)
		{
		}
		Robot_Disconnect.ChassisYawconnect++;

		/*底盘陀螺仪掉线检测*/
		if(Robot_Disconnect.ChassisGyroDisconnect>100)
		{
		}
		Robot_Disconnect.ChassisGyroDisconnect++;
		
		/*大Yaw轴陀螺仪掉线检测*/
		if(Robot_Disconnect.ChassisYawGyroDisconnect>100)
		{
		}
		Robot_Disconnect.ChassisYawGyroDisconnect++;
		
		IWDG_Feed();//喂狗

		vTaskDelay(1); 
		 
#if INCLUDE_uxTaskGetStackHighWaterMark
        Offline_Check_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

