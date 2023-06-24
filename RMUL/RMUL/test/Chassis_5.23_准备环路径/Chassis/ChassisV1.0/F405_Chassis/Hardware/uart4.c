//uart4 f4初始化(裁判系统通信)

#include "main.h"


#if (testChassis == 0)
unsigned char JudgeReceiveBuffer[JudgeBufBiggestSize];
unsigned char JudgeSend[SEND_MAX_SIZE];
/**********************************************************************************************************
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void UART4_Configuration(void)
{
	USART_InitTypeDef usart;
	GPIO_InitTypeDef  gpio;
	NVIC_InitTypeDef  nvic;
	DMA_InitTypeDef   dma;
		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); 

	gpio.GPIO_Pin = GPIO_Pin_10;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC,&gpio);
	
	gpio.GPIO_Pin = GPIO_Pin_11;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC,&gpio);

	usart.USART_BaudRate = 115200;
	usart.USART_WordLength = USART_WordLength_8b;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(UART4,&usart);
	
	USART_ClearFlag(UART4, USART_FLAG_TXE | USART_FLAG_TC);

//	NVIC_InitTypeDef nvicInit;
//	nvicInit.NVIC_IRQChannel = UART4_IRQn;
//	nvicInit.NVIC_IRQChannelPreemptionPriority = 2;
//	nvicInit.NVIC_IRQChannelSubPriority = 0;
//	nvicInit.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&nvicInit);
//	USART_ITConfig(UART4,USART_IT_TC,ENABLE);
//	USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);
	
	{
		//配置DMA1的接收中断
		nvic.NVIC_IRQChannel = DMA1_Stream2_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority = 2;
		nvic.NVIC_IRQChannelSubPriority = 1;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
		
		DMA_InitTypeDef   DMA_InitStructure;
		
		/* 配置DMA1 Stream2，UART4接收 */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
		DMA_DeInit(DMA1_Stream2);
		DMA_InitStructure.DMA_Channel= DMA_Channel_4;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(UART4->DR);
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)JudgeReceiveBuffer;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		DMA_InitStructure.DMA_BufferSize = JudgeBufBiggestSize;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
		DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
		DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(DMA1_Stream2,&DMA_InitStructure);
		
		DMA_ITConfig(DMA1_Stream2,DMA_IT_TC,ENABLE);
		DMA_Cmd(DMA1_Stream2,ENABLE);
	}
		
	{
		//配置DMA1的发送中断
		nvic.NVIC_IRQChannel = DMA1_Stream4_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority = 3;
		nvic.NVIC_IRQChannelSubPriority = 1;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
			
		DMA_InitTypeDef   DMA_InitStructure;
		DMA_DeInit(DMA1_Stream4);
		//while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE);//等待DMA可配置 
		
		/* 配置DMA1 Stream4，UART4发送 */
		DMA_InitStructure.DMA_Channel            = DMA_Channel_4;               //通道选择
		DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(UART4->DR);      		//DMA外设地址
		DMA_InitStructure.DMA_Memory0BaseAddr    = (u32)JudgeSend;      		//DMA 存储器0地址
		DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;  //存储器到外设模式
		DMA_InitStructure.DMA_BufferSize         = SEND_MAX_SIZE;       		//数据传输量 
		DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;   //外设非增量模式
		DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;        //存储器增量模式
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据长度:8位
		DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;     //存储器数据长度:8位
		DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;             //使用普通模式 
		DMA_InitStructure.DMA_Priority           = DMA_Priority_High;         	//中等优先级
		DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;         
		DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
		DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;      //存储器突发单次传输
		DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;  //外设突发单次传输
		DMA_Init(DMA1_Stream4, &DMA_InitStructure);                             //初始化DMA Stream4

		DMA_ITConfig(DMA1_Stream4,DMA_IT_TC,ENABLE);							//DMA2传输完成中断
		DMA_Cmd(DMA1_Stream4, DISABLE);											//不使能
	}
		
	USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);	                      //使能USART4DMA接收
	USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);                       //使能USART4DMA发送
	USART_Cmd(UART4,ENABLE);
}

uint8_t JudgeReveice_Flag;
//void UART4_IRQHandler(void)
//{
//	if (USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)
//	{
//			(void)UART4->SR;
//			(void)UART4->DR;
//			DMA_Cmd(DMA1_Stream2, DISABLE); //先关DMA
//			JudgeReveice_Flag = 1;
//			DMA_SetCurrDataCounter(DMA1_Stream2, JudgeBufBiggestSize);  //重新装填待传输的字节数
//			DMA_Cmd(DMA1_Stream2, ENABLE);  //重开DMA
//	}
//}

/**********************************************************************************************************
*函 数 名: DMA1_Stream_IRQHandler
*功能说明: usart2 DMA接收中断
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void DMA1_Stream2_IRQHandler(void)
{	
	if(DMA_GetITStatus(DMA1_Stream2, DMA_IT_TCIF2))
	{ 
		extern TaskHandle_t JudgeReceiveTask_Handler; //任务句柄
		JudgeReveice_Flag = 1;
		DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
	}
}

/**********************************************************************************************************
*函 数 名: DMA1_Stream4_IRQHandler
*功能说明: usart2 DMA发送中断
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void DMA1_Stream4_IRQHandler(void)
{	
	if(DMA_GetITStatus(DMA1_Stream4, DMA_IT_TCIF4))
	{
		DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
		DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);
		DMA_Cmd(DMA1_Stream4, DISABLE);			
	}
}

//用于测试底盘控制的遥控器控制
#else
volatile uint8_t sbus_rx_buffer[2][RX_USART3_BUFFER];  //尝试双缓冲模式

/**
  * @brief  配置USART3
  * @param  None
  * @retval None
  */
void USART3_Configuration(void)
{
    GPIO_InitTypeDef gpioInit;
    USART_InitTypeDef usartInit;
    NVIC_InitTypeDef nvicInit;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

    gpioInit.GPIO_Pin = GPIO_Pin_11;
    gpioInit.GPIO_Mode = GPIO_Mode_AF;
    gpioInit.GPIO_OType = GPIO_OType_PP;
    gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpioInit.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &gpioInit);

    usartInit.USART_BaudRate = 100000;
    usartInit.USART_WordLength = USART_WordLength_8b;
    usartInit.USART_StopBits = USART_StopBits_1;
    usartInit.USART_Parity = USART_Parity_Even;
    usartInit.USART_Mode = USART_Mode_Rx;
    usartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3, &usartInit);

    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
    USART_Cmd(USART3, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);

    nvicInit.NVIC_IRQChannel = USART3_IRQn;
    nvicInit.NVIC_IRQChannelPreemptionPriority = 0;
    nvicInit.NVIC_IRQChannelSubPriority = 0;
    nvicInit.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicInit);

    {
        DMA_InitTypeDef dmaInit;
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
        DMA_DeInit(DMA1_Stream1);
        DMA_Cmd(DMA1_Stream1,DISABLE);
        dmaInit.DMA_Channel = DMA_Channel_4;
        dmaInit.DMA_PeripheralBaseAddr = (uint32_t) & (USART3->DR);
        dmaInit.DMA_Memory0BaseAddr = (uint32_t)&sbus_rx_buffer[0][0];
        dmaInit.DMA_DIR = DMA_DIR_PeripheralToMemory;
        dmaInit.DMA_BufferSize = RX_USART3_BUFFER;
        dmaInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        dmaInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
        dmaInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        dmaInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        dmaInit.DMA_Mode = DMA_Mode_Circular;
        dmaInit.DMA_Priority = DMA_Priority_VeryHigh;
        dmaInit.DMA_FIFOMode = DMA_FIFOMode_Disable;
        dmaInit.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        dmaInit.DMA_MemoryBurst =  DMA_MemoryBurst_Single;//DMA_Mode_Normal;
        dmaInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        //双缓冲模式
        DMA_DoubleBufferModeConfig(DMA1_Stream1,(uint32_t)&sbus_rx_buffer[1][0], DMA_Memory_0);   //first used memory configuration         
        DMA_DoubleBufferModeCmd(DMA1_Stream1, ENABLE);         
        DMA_Init(DMA1_Stream1,&dmaInit); 
        USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
        DMA_Cmd(DMA1_Stream1,ENABLE);
    }
}
/**
  * @brief  USART3空闲中断
  * @param  None
  * @retval None
  */
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
        USART_ClearITPendingBit(USART3,USART_IT_IDLE);//清空中断标志位
        (void)USART3->DR; //清空数据寄存器，等待下一次接收
        
        if(DMA_GetCurrDataCounter(DMA1_Stream1) == RX_USART3_BUFFER)
            RemoteReceive(DMA_GetCurrentMemoryTarget(DMA1_Stream1)?sbus_rx_buffer[0]:sbus_rx_buffer[1]);
        else 
        {//这个地方是可能收到了残缺帧，故从下一帧开始重新收完整帧
            DMA_Cmd(DMA1_Stream1,DISABLE);
            DMA_SetCurrDataCounter(DMA1_Stream1,RX_USART3_BUFFER);  
            DMA_Cmd(DMA1_Stream1,ENABLE);
        }        
    }
}
#endif


