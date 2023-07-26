#include "main.h"
#include "HW_UART4_debugPC.h"

unsigned char GimbalReceiveBuffer[GimbalBufBiggestSize];
unsigned char GimbalSend[SEND_MAX_SIZE];

/**
  * @brief  UART4配置
  * @param  None
  * @retval None
  */
void UART4_Configuration(void)
{
    USART_InitTypeDef uartInit;
    GPIO_InitTypeDef gpioInit;
		NVIC_InitTypeDef nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);

    gpioInit.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10;
    gpioInit.GPIO_Mode = GPIO_Mode_AF;
    gpioInit.GPIO_OType = GPIO_OType_PP;
    gpioInit.GPIO_Speed = GPIO_Speed_100MHz;
    gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &gpioInit);

    uartInit.USART_BaudRate = 115200;
    uartInit.USART_WordLength = USART_WordLength_8b;
    uartInit.USART_StopBits = USART_StopBits_1;
    uartInit.USART_Parity = USART_Parity_No;
    uartInit.USART_Mode = USART_Mode_Tx/*|USART_Mode_Rx*/;
    uartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(UART4, &uartInit);
    
    USART_ClearFlag(UART4, USART_FLAG_TXE);

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
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)GimbalReceiveBuffer;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		DMA_InitStructure.DMA_BufferSize = GimbalBufBiggestSize;
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
		DMA_InitStructure.DMA_Memory0BaseAddr    = (u32)GimbalSend;      		//DMA 存储器0地址
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

//重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{   
    USART_SendData(UART4, (uint8_t)ch);  // 发送一个字节数据到串口
    while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET) ;// 等待发送完毕
    return (ch);
}

//重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
    while (USART_GetFlagStatus(UART4, USART_FLAG_RXNE) == RESET) ;// 等待串口输入数据
    return (int)USART_ReceiveData(UART4);
}

/**
  * @brief  UART4中断服务函数
  * @param  None
  * @retval None
  */
void UART4_IRQHandler(void)
{
    if (USART_GetITStatus(UART4, USART_IT_TC) != RESET)
    {
        USART_ClearITPendingBit(UART4,USART_IT_TC);
    }
}

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
