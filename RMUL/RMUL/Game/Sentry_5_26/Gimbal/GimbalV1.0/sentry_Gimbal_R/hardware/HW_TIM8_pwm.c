#include "main.h"
#include "HW_TIM8_pwm.h"
//#define SRC_DATA_LEN 2  //(最后面放几个0做延迟)
uint32_t pulse;//占空比

/**
  * @brief  TIM8工作模式配置
  * @param  None
  * @retval None
  */
void TIM8_Configuration(void)             
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);      //使能TIM8时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);     //使能GPIOC时钟

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;      //PB6 TIM8--Ch1     PB7 TIM8--Ch2
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;              //复用输出模式  
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;            //下拉(到GND)输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;        //输出速率
	GPIO_Init(GPIOC,&GPIO_InitStructure);                   //按照该结构体配置对应端口

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8);   //用引脚PC6；把TIM8引到AF2
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8);   //用引脚PC7；把TIM8引到AF2    
					
    //时基配置
	TIM_TimeBaseInitStruct.TIM_Prescaler=7*48-1;                               //时基结构体的预分频数24Mhz(168M/7=24M)
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;              //计数模式（递增计数）
	TIM_TimeBaseInitStruct.TIM_Period=1000-1;                              //f=24MHz/48000=500Hz ~ T=2.0ms（写到ARR里去）
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;                  //时钟分频数（不分频）
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter=0;                         //重复计数器：写入0到RCR里去（即不重复的意思）
	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseInitStruct);
	
   TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;                          //输出比较模式设定为PWM1
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;              //输出比较使能（写到CCER寄存器里面去）
	TIM_OCInitStruct.TIM_Pulse = pulse;                                     //写到CCR里去，比较寄存器的装载值（一个Period里的脉冲数）
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;                  //输出通道为高电平有效
    TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;           //互补输出不使能
    TIM_OC1Init(TIM8,&TIM_OCInitStruct), TIM_OC2Init(TIM8,&TIM_OCInitStruct);
    TIM_OC1PreloadConfig(TIM8,TIM_OCPreload_Enable), TIM_OC2PreloadConfig(TIM8,TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM8,ENABLE);                                       //使能TIM8的ARR预装载寄存器
    TIM_Cmd(TIM8,ENABLE);                                                    //使能TIM8外设
	TIM_CtrlPWMOutputs(TIM8,ENABLE);                                         //主输出使能（写到BDTR里去？）
}
