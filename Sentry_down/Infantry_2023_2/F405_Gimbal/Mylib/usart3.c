/**********************************************************************************************************
 * @文件     usart3.c
 * @说明     usart3遥控器
 * @版本  	 V1.0
 * @作者     郭嘉豪
 * @日期     2021.11
**********************************************************************************************************/
#include "main.h"
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

