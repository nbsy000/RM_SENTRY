#include "main.h"
#include "HW_UART4_debugPC.h"

unsigned char GimbalBuffer[GIMBAL_RECVBUF_SIZE];
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

    uartInit.USART_BaudRate = 256000;
    uartInit.USART_WordLength = USART_WordLength_8b;
    uartInit.USART_StopBits = USART_StopBits_1;
    uartInit.USART_Parity = USART_Parity_No;
    uartInit.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
    uartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(UART4, &uartInit);
    
		USART_ClearFlag(UART4, USART_FLAG_TXE | USART_FLAG_TC);
		
    USART_ClearFlag(UART4, USART_FLAG_TXE);
//		USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);//空
		
		USART_Cmd(UART4, ENABLE);

    nvic.NVIC_IRQChannel = UART4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
		
    USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
    USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);

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
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)GimbalBuffer;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		DMA_InitStructure.DMA_BufferSize = GIMBAL_RECVBUF_SIZE;
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
		nvic.NVIC_IRQChannelPreemptionPriority = 0;
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

		DMA_ITConfig(DMA1_Stream4,DMA_IT_TC|DMA_IT_TE,ENABLE);							//DMA2传输完成中断
		DMA_Cmd(DMA1_Stream4, DISABLE);											//不使能
	}
		
	USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);	                      //使能USART4DMA接收
	USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);                       //使能USART4DMA发送
	USART_Cmd(UART4,ENABLE);
}


/**
  * @brief  UART4中断服务函数
  * @param  None
  * @retval None
  */
void UART4_IRQHandler(void)
{
    if (USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)
    {
//				uint8_t length;
//        USART_ClearITPendingBit(UART4,USART_IT_IDLE);//清空中断标志位
        (void)UART4->DR; //清空数据寄存器，等待下一次接收
//        DMA_Cmd(DMA1_Stream2, DISABLE);//关闭DMA，准备重新配置
//			  DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);	// Clear Transfer Complete flag
//				DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TEIF2);	// Clear Transfer error flag	
////        rc_len = USART1_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA2_Stream2)
//				if (Verify_CRC8_Check_Sum(GimbalBuffer, GIMBAL_RECVBUF_SIZE))
//						Gimbal_Receive(GimbalBuffer);
//				DMA_SetCurrDataCounter(DMA1_Stream2, GIMBAL_RECVBUF_SIZE);
//				DMA_Cmd(DMA1_Stream2,ENABLE);
    }
}

/**********************************************************************************************************
*函 数 名: DMA1_Stream_IRQHandler
*功能说明: usart2 DMA接收中断
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
uint8_t temp4[GIMBAL_RECVBUF_SIZE]; //这里要改为全局变量，不然crc不通过
int16_t Crcpass4, crcNopass4;
int16_t buffindex4;
void DMA1_Stream2_IRQHandler(void)
{	
	static uint8_t temptemp[2 * GIMBAL_RECVBUF_SIZE];
  int16_t PackPoint, n;
	if(DMA_GetITStatus(DMA1_Stream2, DMA_IT_TCIF2))
	{ 
		DMA_Cmd(DMA1_Stream2,DISABLE);
    memcpy(temptemp + GIMBAL_RECVBUF_SIZE, GimbalBuffer, GIMBAL_RECVBUF_SIZE);
		for(PackPoint = 0; PackPoint < GIMBAL_RECVBUF_SIZE; PackPoint++) //防止错位，不一定数组元素的第一个就为
			{
					if (temptemp[PackPoint] == '!')
					{
							for (n = 0; n < GIMBAL_RECVBUF_SIZE; n++)
							{
									temp4[n] = temptemp[(n + PackPoint)];
							}
							crcNopass4++;
							if (Verify_CRC8_Check_Sum(temp4, GIMBAL_RECVBUF_SIZE))
									Gimbal_Receive(temp4);
							else
							{
									buffindex4++;
									buffindex4 = buffindex4 % 4;
							}
							break;
					}
			}
		DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2);
    DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
    memcpy(temptemp, temptemp + GIMBAL_RECVBUF_SIZE, GIMBAL_RECVBUF_SIZE);
		DMA_Cmd(DMA1_Stream2,ENABLE);
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
		DMA_Cmd(DMA1_Stream4, DISABLE);	
		DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
		DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);
		DMA_SetCurrDataCounter(DMA1_Stream4, SEND_MAX_SIZE);		
		while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
		USART_ClearFlag(UART4, USART_FLAG_TXE);
		Classify_Send_Msg(GimbalSend);
	}
//	else if(DMA_GetITStatus(DMA1_Stream4, DMA_IT_TEIF4))
//	{
//		DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TEIF4);	
//	}
}
