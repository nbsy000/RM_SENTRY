#include "main.h"


/**********************************************************************************************************
*函 数 名: CAN2_Configuration
*功能说明: can2配置函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void CAN2_Configuration(void)
{
	GPIO_InitTypeDef 		GPIO_InitStructure;
	CAN_InitTypeDef        	CAN_InitStructure;
	CAN_FilterInitTypeDef  	CAN_FilterInitStructure;
	NVIC_InitTypeDef 		NVIC_InitStructure;

	//打开时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_12 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* CAN1 Enabling interrupt */									  
	NVIC_InitStructure.NVIC_IRQChannel=CAN2_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);									
	
	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = CAN2_TX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* CAN  BaudRate = RCC_APB1PeriphClock/(CAN_SJW+CAN_BS1+CAN_BS2)/CAN_Prescaler */
	CAN_DeInit(CAN2);
	CAN_StructInit(&CAN_InitStructure);   

	CAN_InitStructure.CAN_TTCM=DISABLE;
	CAN_InitStructure.CAN_ABOM=ENABLE;
	CAN_InitStructure.CAN_AWUM=ENABLE;
	CAN_InitStructure.CAN_NART=DISABLE;
	CAN_InitStructure.CAN_RFLM=DISABLE;
	CAN_InitStructure.CAN_TXFP=ENABLE;
	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;   
//	CAN_InitStructure.CAN_Mode=CAN_Mode_LoopBack;
	CAN_InitStructure.CAN_SJW  = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_11tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
	CAN_InitStructure.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+11+2)/3=1kbps

	CAN_Init(CAN2,&CAN_InitStructure);	// CAN2	
	
	//FIFO0，列表模式
	CAN_FilterInitStructure.CAN_FilterNumber=15;	 
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList;	 // 标识符屏蔽位模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit;   // 16位过滤器
	CAN_FilterInitStructure.CAN_FilterIdHigh = Remote_Control_ID<<5;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x101<<5;//大Yaw轴的陀螺仪
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh =  GIMBAL_SYNE_ID<<5;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x107<<5;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;	 // FIFO0指向过滤器
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	//掩码模式，所有都接（正常情况下是接收不到FIFO0中的，接收有优先级），用于调试
	CAN_FilterInitStructure.CAN_FilterNumber = 16; //选择过滤器1
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; //掩码模式
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0;
	CAN_FilterInitStructure.CAN_FilterIdLow =  0x0;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0;  
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO1;//fifo
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	//CAN中断使能
	CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
	CAN_ITConfig(CAN2,CAN_IT_FMP1,ENABLE);
	CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);
}

/**********************************************************************************************************
*函 数 名: CAN2_TX_IRQHandler
*功能说明: can2发送中断
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void CAN2_TX_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET) 
	{
		CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
	}
}

/**********************************************************************************************************
*函 数 名: CAN2_RX0_IRQHandler
*功能说明: can2接收中断0
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
	CanRxMsg Can2_rx_message_0;
void CAN2_RX0_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET)
	{
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
		CAN_Receive(CAN2, CAN_FIFO0, &Can2_rx_message_0);
		Can2Receive0(Can2_rx_message_0);
	}
}
/**********************************************************************************************************
*函 数 名: CAN2_RX1_IRQHandler
*功能说明: can2接收中断1
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
	CanRxMsg Can2_rx_message_1;
void CAN2_RX1_IRQHandler(void)
{

	if (CAN_GetITStatus(CAN2,CAN_IT_FMP1)!= RESET)
	{
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP1);
		CAN_Receive(CAN2, CAN_FIFO1, &Can2_rx_message_1);
		Can2Receive1(Can2_rx_message_1); // ID陀螺仪代码冲突
	}
}
