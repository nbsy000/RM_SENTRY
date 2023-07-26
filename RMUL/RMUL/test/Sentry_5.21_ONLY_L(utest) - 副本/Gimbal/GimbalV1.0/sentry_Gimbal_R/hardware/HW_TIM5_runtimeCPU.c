#include "main.h"
#include "HW_TIM5_runtimeCPU.h"

void TIM5_Configuration(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); //使能TIM2时钟
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;

    //时基配置：
    //需要定时器的频率是系统时钟节拍的10~20倍,而这里的系统节拍是1kHz
    TIM_TimeBaseInitStruct.TIM_Prescaler = 7 - 1;               //时基结构体的预分频数2.4Mhz(168M/70=2.4M=2400k)
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; //计数模式（递增计数）
    TIM_TimeBaseInitStruct.TIM_Period = 1200 - 1;                 //f=2.4MHz/120=20kHz ~ T=0.05ms（写到ARR里去）
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;     //时钟分频数（不分频）
    //TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;            //重复计数器：写入0到RCR里去（即不重复的意思）
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStruct);

    NVIC_InitTypeDef nvicInit;
    nvicInit.NVIC_IRQChannel = TIM5_IRQn;
    nvicInit.NVIC_IRQChannelPreemptionPriority = 2;
    nvicInit.NVIC_IRQChannelSubPriority = 0;
    nvicInit.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicInit);
    
    //TIM_ARRPreloadConfig(TIM5,ENABLE);                                       //使能TIM8的ARR预装载寄存器
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);  //使能更新中断
    TIM_ClearFlag(TIM5, TIM_FLAG_Update); //清空中断标志位
    TIM_Cmd(TIM5,ENABLE);
}

/* 中断服务函数：用于统计运行时间 */
volatile uint32_t CPU_RunTime = 0;
extern  uint32_t Pc_RX_Num;
extern  uint32_t Last_Pc_Rx_Num;
extern  uint32_t Last_Shoot_Rx_Num;
extern  uint8_t armor_state;
uint8_t shoot_able_flag;
void TIM5_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
    {
        CPU_RunTime++;
			  if((CPU_RunTime%6000) == 0) 
				{
					if(Pc_RX_Num==Last_Pc_Rx_Num)
						 armor_state = ARMOR_NO_AIM;
				  Last_Pc_Rx_Num = Pc_RX_Num;
				}
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
    }
}
