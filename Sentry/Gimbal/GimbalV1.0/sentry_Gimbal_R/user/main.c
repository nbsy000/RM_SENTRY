#include "main.h"

static void hardware_init(void);
roboDisconnect Robo_Disconnect;
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//即四位全部用于抢占优先级
    //（注意：一旦初始化好 NVIC 的优先级分组后，切不可以在应用中再次更改。）
    //在 NVIC 分组为 4 的情况下，抢占优先级可配置范围是 0-15，
    //数值越小，抢占优先级的级别越高
    
    hardware_init();  //老规矩 上电初始化
		Rob_Init();
    delay_ms(100); 
    start_the_very_first_task();   //这个函数来开启引导任务
    vTaskStartScheduler();   //开启任务调度器（后面vans了就）
    while (1) {;}
}

static void hardware_init(void)
{
    TIM2_Configuration();
    delay_ms(100);  //等陀螺仪初始化完成
    LED_Configuration();//点上云台状态灯
    TIM4_Configuration();  //TIM4监测cpu运行时间 
		COUNTER_Configuration();
    delay_ms(100);     
    USART2_Configuration();  //和PC通信用
    USART3_Configuration();  //和遥控器通信用
    UART4_Configuration();  //发送cpu运行状态
		VOFA_USART_Configuration();//VOFA
		delay_ms(100);//等陀螺仪初始化完成
    CAN_Configuration();
    //IWDG_Config(IWDG_Prescaler_128,3125);
    delay_ms(100); 
		 /* IMU初始化 */
    INS_Init();
    ICM20602_init(&IMUReceive, &IMUData);
	
}


void Rob_Init(void)
{
		Robo_Disconnect.HeatDiscount = 0;
		Robo_Disconnect.RemoteDisconnect = 0;

}
