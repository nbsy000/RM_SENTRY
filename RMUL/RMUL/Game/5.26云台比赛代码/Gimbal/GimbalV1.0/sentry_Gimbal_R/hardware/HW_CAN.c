#include "main.h"

CanRxMsg Can1_rx_message_0,Can1_rx_message_1,Can2_rx_message_0,Can2_rx_message_1;

/*********************************CAN1****************************************/
/**
  * @brief  配置CAN1
  * @param  None
  * @retval None
  */
static void CAN1_Configuration(void)
{
		CAN_InitTypeDef        canInit;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpioInit;
    NVIC_InitTypeDef       nvicInit;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

		GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_CAN1);

    gpioInit.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    gpioInit.GPIO_Mode = GPIO_Mode_AF;
		gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    gpioInit.GPIO_OType = GPIO_OType_PP;
		gpioInit.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOB, &gpioInit);
    
    nvicInit.NVIC_IRQChannel = CAN1_RX0_IRQn;
    nvicInit.NVIC_IRQChannelPreemptionPriority = 1;
    nvicInit.NVIC_IRQChannelSubPriority = 0;
    nvicInit.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicInit);
		
	nvicInit.NVIC_IRQChannel = CAN1_RX1_IRQn;
    nvicInit.NVIC_IRQChannelPreemptionPriority = 1;
    nvicInit.NVIC_IRQChannelSubPriority = 0;
    nvicInit.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicInit);
		
    nvicInit.NVIC_IRQChannel = CAN1_TX_IRQn;
    nvicInit.NVIC_IRQChannelPreemptionPriority = 1;
    nvicInit.NVIC_IRQChannelSubPriority = 0;
    nvicInit.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicInit);
    
    CAN_DeInit(CAN1);
    CAN_StructInit(&canInit);
    
    canInit.CAN_TTCM = DISABLE;
    canInit.CAN_ABOM = ENABLE;
    canInit.CAN_AWUM = ENABLE;
    canInit.CAN_NART = DISABLE;
    canInit.CAN_RFLM = DISABLE;
    canInit.CAN_TXFP = ENABLE;
    canInit.CAN_Mode = CAN_Mode_Normal;
    canInit.CAN_SJW  = CAN_SJW_1tq;
    canInit.CAN_BS1 = CAN_BS1_9tq;
    canInit.CAN_BS2 = CAN_BS2_4tq;
		canInit.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN1, &canInit);

	can_filter.CAN_FilterNumber = 0; 
	can_filter.CAN_FilterMode = CAN_FilterMode_IdList; 
	can_filter.CAN_FilterScale = CAN_FilterScale_16bit;
	can_filter.CAN_FilterIdHigh =  FrictionMotor_Up_0_ID<<5; 
	can_filter.CAN_FilterIdLow = FrictionMotor_Up_1_ID<<5;
	can_filter.CAN_FilterMaskIdHigh = BodanMotor_Up_ID<<5; 
	can_filter.CAN_FilterMaskIdLow =  0x000<<5;  
	can_filter.CAN_FilterFIFOAssignment = 0;
	can_filter.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&can_filter);

	can_filter.CAN_FilterNumber = 1;
//	can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
	can_filter.CAN_FilterMode = CAN_FilterMode_IdList; 
	can_filter.CAN_FilterScale = CAN_FilterScale_16bit;
	can_filter.CAN_FilterIdHigh = 0x100<<5 ;
	can_filter.CAN_FilterIdLow = 0x101<<5;
	can_filter.CAN_FilterMaskIdHigh =  0 | CAN_ID_STD;
	can_filter.CAN_FilterMaskIdLow =  0 | CAN_ID_STD;
	can_filter.CAN_FilterFIFOAssignment = 1;
	can_filter.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&can_filter);	
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
    CAN_ITConfig(CAN1,CAN_IT_FMP1,ENABLE);	
    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 
}


/**
  * @brief  CAN1发送中断
  * @param  None
  * @retval None
  */
void CAN1_TX_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
		CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
}
/**
  * @brief  CAN1接收中断0
  * @param  None
  * @retval None
  */
void CAN1_RX0_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
        CAN_Receive(CAN1, CAN_FIFO0, &Can1_rx_message_0);
        CAN1_DataReceive_0();
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    }
}


/**
  * @brief  CAN1接收中断1
  * @param  None
  * @retval None
  */
void CAN1_RX1_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1,CAN_IT_FMP1)!= RESET)
    {
        CAN_Receive(CAN1, CAN_FIFO1, &Can1_rx_message_1);
		CAN1_DataReceive_1();
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP1);
    }
}





/*********************************CAN2****************************************/
/**
  * @brief  配置CAN2
  * @param  None
  * @retval None
  */
static void CAN2_Configuration(void)
{
	CAN_InitTypeDef canInit;
	CAN_FilterInitTypeDef can_filter;
	GPIO_InitTypeDef gpioInit;
	NVIC_InitTypeDef nvicInit;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
	
	gpioInit.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_12 ;
	gpioInit.GPIO_Mode = GPIO_Mode_AF;
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
	gpioInit.GPIO_OType = GPIO_OType_PP;
	gpioInit.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &gpioInit);
	
	nvicInit.NVIC_IRQChannel = CAN2_RX0_IRQn;
	nvicInit.NVIC_IRQChannelPreemptionPriority = 1;
	nvicInit.NVIC_IRQChannelSubPriority = 0;
	nvicInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicInit);

	nvicInit.NVIC_IRQChannel = CAN2_RX1_IRQn;
	nvicInit.NVIC_IRQChannelPreemptionPriority = 1;
	nvicInit.NVIC_IRQChannelSubPriority = 0;
	nvicInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicInit);

	nvicInit.NVIC_IRQChannel = CAN2_TX_IRQn;
	nvicInit.NVIC_IRQChannelPreemptionPriority = 1;
	nvicInit.NVIC_IRQChannelSubPriority = 0;
	nvicInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicInit); 
		
	CAN_DeInit(CAN2);
	CAN_StructInit(&canInit);
		
	canInit.CAN_TTCM = DISABLE;
	canInit.CAN_ABOM = ENABLE; 
	canInit.CAN_AWUM = ENABLE; 
	canInit.CAN_NART = DISABLE; 
	canInit.CAN_RFLM = DISABLE; 
	canInit.CAN_TXFP = ENABLE; 
	canInit.CAN_Mode = CAN_Mode_Normal;
	canInit.CAN_SJW = CAN_SJW_1tq;
	canInit.CAN_BS1 = CAN_BS1_9tq;
	canInit.CAN_BS2 = CAN_BS2_4tq;
	canInit.CAN_Prescaler = 3; //CAN BaudRate 42/(1+9+4)/3=1Mbps
	CAN_Init(CAN2, &canInit);
		

	can_filter.CAN_FilterNumber = 15;
	can_filter.CAN_FilterMode = CAN_FilterMode_IdList;
	can_filter.CAN_FilterScale = CAN_FilterScale_16bit;
	can_filter.CAN_FilterIdHigh = MotoPitch_ID_R << 5;
	can_filter.CAN_FilterIdLow = MotoYaw_ID_R << 5; 
	can_filter.CAN_FilterMaskIdHigh = MotoPitch_ID_L << 5;
	can_filter.CAN_FilterMaskIdLow = MotoYaw_ID_L << 5;
	can_filter.CAN_FilterFIFOAssignment = 0;
	can_filter.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&can_filter);

	//FIFO1，只接收gyro_id
	can_filter.CAN_FilterNumber = 16;
	can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;//掩码模式，该过滤器需要接收5种ID
	//can_filter.CAN_FilterMode = CAN_FilterMode_IdList;
	can_filter.CAN_FilterScale = CAN_FilterScale_16bit;
	can_filter.CAN_FilterIdHigh = 0x0 << 5;
	can_filter.CAN_FilterIdLow = 0x0 << 5;//后3位不屏蔽
	can_filter.CAN_FilterMaskIdHigh = 0x0<<5;;
	can_filter.CAN_FilterMaskIdLow = 0x0 << 5;
	can_filter.CAN_FilterFIFOAssignment = 1;
	can_filter.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&can_filter);

	CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
	CAN_ITConfig(CAN2,CAN_IT_FMP1,ENABLE);		
	CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);
}

/**
  * @brief  CAN2发送中断
  * @param  None
  * @retval None
  */
void CAN2_TX_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET) 
		CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
}
/**
  * @brief  CAN2接收中断
  * @param  None
  * @retval None
  */
void CAN2_RX0_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET)
	{
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &Can2_rx_message_0);
        CAN2_DataReceive_0();
    }
}

/**
  * @brief  CAN2接收中断
  * @param  None
  * @retval None
  */
void CAN2_RX1_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN2,CAN_IT_FMP1)!= RESET)
    {
				CAN_ClearITPendingBit(CAN2, CAN_IT_FMP1);
        CAN_Receive(CAN2, CAN_FIFO1, &Can2_rx_message_1);
        CAN2_DataReceive_1();
    }
}


/**
  * @brief  调用上面的两个函数配置CAN1和CAN2
  * @param  None
  * @retval None
  */
void CAN_Configuration(void)
{
    CAN1_Configuration();
    CAN2_Configuration();
}
