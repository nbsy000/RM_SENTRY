#include "main.h"
#include "HW_USART2_PC.h"


uint8_t PCbuffer[PC_RECVBUF_SIZE] = {0, 0, 0};
extern uint8_t SendToPC_Buff[PC_SENDBUF_SIZE];

/**
  * @brief  串口2配置，PC通信
  * @param  None
  * @retval None
  */
void USART2_Configuration(void)
{
    USART_InitTypeDef usartInit;
    GPIO_InitTypeDef  gpioInit;
    NVIC_InitTypeDef  nvicInit;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);

    gpioInit.GPIO_Pin = GPIO_Pin_2;
    gpioInit.GPIO_Mode = GPIO_Mode_AF;
    gpioInit.GPIO_OType = GPIO_OType_PP;
    gpioInit.GPIO_Speed = GPIO_Speed_100MHz;
    gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpioInit);

    gpioInit.GPIO_Pin = GPIO_Pin_3;
    gpioInit.GPIO_Mode = GPIO_Mode_AF;
    gpioInit.GPIO_OType = GPIO_OType_PP;
    gpioInit.GPIO_Speed = GPIO_Speed_100MHz;
    gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpioInit);

    gpioInit.GPIO_Pin = GPIO_Pin_1;
    gpioInit.GPIO_Mode = GPIO_Mode_OUT;
    gpioInit.GPIO_OType = GPIO_OType_PP;
    gpioInit.GPIO_Speed = GPIO_Speed_100MHz;
    gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpioInit);

    usartInit.USART_BaudRate = 115200;
    usartInit.USART_WordLength = USART_WordLength_8b;
    usartInit.USART_StopBits = USART_StopBits_1;
    usartInit.USART_Parity = USART_Parity_No;
    usartInit.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    usartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &usartInit);

    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
    
    USART_Cmd(USART2, ENABLE);
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);

    nvicInit.NVIC_IRQChannel = USART2_IRQn;
    nvicInit.NVIC_IRQChannelPreemptionPriority = 1;
    nvicInit.NVIC_IRQChannelSubPriority = 0;
    nvicInit.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicInit);

    {
        DMA_InitTypeDef dmaInit;
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
        DMA_DeInit(DMA1_Stream5);
        dmaInit.DMA_Channel = DMA_Channel_4;
        dmaInit.DMA_PeripheralBaseAddr = (uint32_t) & (USART2->DR);
        dmaInit.DMA_Memory0BaseAddr = (uint32_t)PCbuffer;
        dmaInit.DMA_DIR = DMA_DIR_PeripheralToMemory;
        dmaInit.DMA_BufferSize = PC_RECVBUF_SIZE;
        dmaInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        dmaInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
        dmaInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        dmaInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        dmaInit.DMA_Mode = DMA_Mode_Circular;
        dmaInit.DMA_Priority = DMA_Priority_VeryHigh;
        dmaInit.DMA_FIFOMode = DMA_FIFOMode_Disable;
        dmaInit.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        dmaInit.DMA_MemoryBurst = DMA_Mode_Normal;
        dmaInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

        nvicInit.NVIC_IRQChannel = DMA1_Stream5_IRQn;
        nvicInit.NVIC_IRQChannelPreemptionPriority = 1;
        nvicInit.NVIC_IRQChannelSubPriority = 3;
        nvicInit.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&nvicInit);

        DMA_Init(DMA1_Stream5, &dmaInit);
#ifndef NEW_SHOOTAIM
        DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);
#endif				
        DMA_Cmd(DMA1_Stream5, ENABLE);
    }

    {
        DMA_InitTypeDef dmaInit;
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
        DMA_DeInit(DMA1_Stream6);
        dmaInit.DMA_Channel = DMA_Channel_4;
        dmaInit.DMA_PeripheralBaseAddr = (uint32_t) & (USART2->DR);
        dmaInit.DMA_Memory0BaseAddr = (uint32_t)SendToPC_Buff;
        dmaInit.DMA_DIR = DMA_DIR_MemoryToPeripheral;
        dmaInit.DMA_BufferSize = PC_SENDBUF_SIZE;
        dmaInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        dmaInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
        dmaInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        dmaInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        dmaInit.DMA_Mode = DMA_Mode_Normal;
        dmaInit.DMA_Priority = DMA_Priority_VeryHigh;
        dmaInit.DMA_FIFOMode = DMA_FIFOMode_Disable;
        dmaInit.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        dmaInit.DMA_MemoryBurst = DMA_Mode_Normal;
        dmaInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

        nvicInit.NVIC_IRQChannel = DMA1_Stream6_IRQn;
        nvicInit.NVIC_IRQChannelPreemptionPriority = 1;
        nvicInit.NVIC_IRQChannelSubPriority = 1;
        nvicInit.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&nvicInit);

        DMA_Init(DMA1_Stream6, &dmaInit);
        DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);
        DMA_Cmd(DMA1_Stream6, DISABLE);
    }
}

/**
  * @brief  串口2空闲中断配置
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
        (void)USART2->SR; //clear the IDLE int
        (void)USART2->DR;
			
#ifdef NEW_SHOOTAIM			
			  DMA_Cmd(DMA1_Stream5, DISABLE);
        DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
        DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);

        PCReceive(PCbuffer);

        DMA_SetCurrDataCounter(DMA1_Stream5, PC_RECVBUF_SIZE);
        DMA_Cmd(DMA1_Stream5, ENABLE);
#endif
    }
}


#ifndef NEW_SHOOTAIM
uint8_t tempPC[PC_RECVBUF_SIZE]; //这里要改为全局变量，不然crc不通过
int16_t Crcpass, crcNopass;
uint8_t ErrorBuff[PC_RECVBUF_SIZE * 4];
int16_t buffindex;
/**
  * @brief  串口2 DMA接收中断
  * @param  None
  * @retval None
  */
void DMA1_Stream5_IRQHandler(void)
{
    static uint8_t temptemp[2 * PC_RECVBUF_SIZE];
    int16_t PackPoint, n;
    if (DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5))
    {
//        DMA_Cmd(DMA1_Stream5,DISABLE);
        memcpy(temptemp + PC_RECVBUF_SIZE, PCbuffer, PC_RECVBUF_SIZE);
        for (PackPoint = 0; PackPoint < PC_RECVBUF_SIZE; PackPoint++) //防止错位，不一定数组元素的第一个就为
        {
            if (temptemp[PackPoint] == '!')
            {
                for (n = 0; n < PC_RECVBUF_SIZE; n++)
                {
                    tempPC[n] = temptemp[(n + PackPoint)];
                }
                crcNopass++;
                if (Verify_CRC8_Check_Sum(tempPC, PC_RECVBUF_SIZE))
                    PCReceive(tempPC);
                else
                {
                    buffindex++;
                    buffindex = buffindex % 4;
                }
                break;
            }
        }
				memcpy(temptemp, temptemp + PC_RECVBUF_SIZE, PC_RECVBUF_SIZE);

        DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
        DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
        //DMA_Cmd(DMA1_Stream5,ENABLE);
    }
}

#endif
/**
  * @brief  串口2 DMA发送中断
  * @param  None
  * @retval None
  */
void DMA1_Stream6_IRQHandler(void)
{
    if (DMA_GetFlagStatus(DMA1_Stream6, DMA_IT_TCIF6) == SET)
    {		
				while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
					;
			  DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6);
        DMA_ClearITPendingBit(DMA1_Stream6, DMA_FLAG_TCIF6);
        USART_ClearFlag(USART2, USART_FLAG_TC);
    }
}
