#include "main.h"
#include "HW_USART6.h"


//uint8_t NAV_Recv_Buf[NAV_RECVBUF_SIZE] = {0, 0, 0};
//uint8_t NAV_Send[NAV_SENDBUF_SIZE];
//NAV_Recv_t NAV_Recv;
///**
//  * @brief  串口2配置，PC通信
//  * @param  None
//  * @retval None
//  */
//void USART6_Configuration(void)
//{
//    USART_InitTypeDef usartInit;
//    GPIO_InitTypeDef  gpioInit;
//    NVIC_InitTypeDef  nvicInit;

//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

//    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
//    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

//    gpioInit.GPIO_Pin = GPIO_Pin_6;
//    gpioInit.GPIO_Mode = GPIO_Mode_AF;
//    gpioInit.GPIO_OType = GPIO_OType_PP;
//    gpioInit.GPIO_Speed = GPIO_Speed_100MHz;
//    gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
//    GPIO_Init(GPIOC, &gpioInit);

//    gpioInit.GPIO_Pin = GPIO_Pin_7;
//    gpioInit.GPIO_Mode = GPIO_Mode_AF;
//    gpioInit.GPIO_OType = GPIO_OType_PP;
//    gpioInit.GPIO_Speed = GPIO_Speed_100MHz;
//    gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
//    GPIO_Init(GPIOC, &gpioInit);

//    usartInit.USART_BaudRate = 115200;
//    usartInit.USART_WordLength = USART_WordLength_8b;
//    usartInit.USART_StopBits = USART_StopBits_1;
//    usartInit.USART_Parity = USART_Parity_No;
//    usartInit.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
//    usartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//    USART_Init(USART6, &usartInit);

//    USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);
//    
//    USART_Cmd(USART6, ENABLE);
//    USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
//    USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);

//    nvicInit.NVIC_IRQChannel = USART6_IRQn;
//    nvicInit.NVIC_IRQChannelPreemptionPriority = 1;
//    nvicInit.NVIC_IRQChannelSubPriority = 0;
//    nvicInit.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&nvicInit);

//    {
//				//接收
//        DMA_InitTypeDef dmaInit;
//        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
//        DMA_DeInit(DMA2_Stream1);
//        dmaInit.DMA_Channel = DMA_Channel_5;
//        dmaInit.DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR);
//        dmaInit.DMA_Memory0BaseAddr = (uint32_t)NAV_Recv_Buf;
//        dmaInit.DMA_DIR = DMA_DIR_PeripheralToMemory;
//        dmaInit.DMA_BufferSize = NAV_RECVBUF_SIZE;
//        dmaInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//        dmaInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
//        dmaInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//        dmaInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//        dmaInit.DMA_Mode = DMA_Mode_Circular;
//        dmaInit.DMA_Priority = DMA_Priority_VeryHigh;
//        dmaInit.DMA_FIFOMode = DMA_FIFOMode_Disable;
//        dmaInit.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//        dmaInit.DMA_MemoryBurst = DMA_Mode_Normal;
//        dmaInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

//        nvicInit.NVIC_IRQChannel = DMA2_Stream5_IRQn;
//        nvicInit.NVIC_IRQChannelPreemptionPriority = 1;
//        nvicInit.NVIC_IRQChannelSubPriority = 3;
//        nvicInit.NVIC_IRQChannelCmd = ENABLE;
//        NVIC_Init(&nvicInit);

//        DMA_Init(DMA2_Stream1, &dmaInit);
//        DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);
//        DMA_Cmd(DMA2_Stream1, ENABLE);
//    }

//    {
//				//发送
//        DMA_InitTypeDef dmaInit;
//        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
//        DMA_DeInit(DMA2_Stream6);
//        dmaInit.DMA_Channel = DMA_Channel_5;
//        dmaInit.DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR);
//        dmaInit.DMA_Memory0BaseAddr = (uint32_t)NAV_Send;
//        dmaInit.DMA_DIR = DMA_DIR_MemoryToPeripheral;
//        dmaInit.DMA_BufferSize = NAV_SENDBUF_SIZE;
//        dmaInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//        dmaInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
//        dmaInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//        dmaInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//        dmaInit.DMA_Mode = DMA_Mode_Normal;
//        dmaInit.DMA_Priority = DMA_Priority_VeryHigh;
//        dmaInit.DMA_FIFOMode = DMA_FIFOMode_Disable;
//        dmaInit.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//        dmaInit.DMA_MemoryBurst = DMA_Mode_Normal;
//        dmaInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

//        nvicInit.NVIC_IRQChannel = DMA2_Stream6_IRQn;
//        nvicInit.NVIC_IRQChannelPreemptionPriority = 1;
//        nvicInit.NVIC_IRQChannelSubPriority = 1;
//        nvicInit.NVIC_IRQChannelCmd = ENABLE;
//        NVIC_Init(&nvicInit);

//        DMA_Init(DMA2_Stream6, &dmaInit);
//        DMA_ITConfig(DMA2_Stream6, DMA_IT_TC, ENABLE);
//        DMA_Cmd(DMA2_Stream6, DISABLE);
//    }
//}

///**
//  * @brief  串口2空闲中断配置
//  * @param  None
//  * @retval None
//  */
//void USART6_IRQHandler(void)
//{
//    if (USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
//    {
//        (void)USART6->SR; //clear the IDLE int
//        (void)USART6->DR;
//    }
//}


//uint8_t tempNAV[NAV_RECVBUF_SIZE]; //这里要改为全局变量，不然crc不通过
//int16_t Crcpass6, crcNopass6;
//int16_t buffindex6;
///**
//  * @brief  串口2 DMA接收中断
//  * @param  None
//  * @retval None
//  */
//void DMA2_Stream5_IRQHandler(void)
//{
//    static uint8_t temptemp[2 * NAV_RECVBUF_SIZE];
//    int16_t PackPoint, n;
//    if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF2))
//    {
//        DMA_Cmd(DMA2_Stream1,DISABLE);
//        memcpy(temptemp + NAV_RECVBUF_SIZE, NAV_Recv_Buf, NAV_RECVBUF_SIZE);
//        for (PackPoint = 0; PackPoint < NAV_RECVBUF_SIZE; PackPoint++) //防止错位，不一定数组元素的第一个就为
//        {
//            if (temptemp[PackPoint] == '!')
//            {
//                for (n = 0; n < NAV_RECVBUF_SIZE; n++)
//                {
//                    tempNAV[n] = temptemp[(n + PackPoint)];
//                }
//                crcNopass6++;
//                if (Verify_CRC8_Check_Sum(tempNAV, NAV_RECVBUF_SIZE))
//                    NAVReceive(tempNAV);
//                else
//                {
//                    buffindex6++;
//                    buffindex6 = buffindex6 % 4;
//                }
//                break;
//            }
//        }
//        DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF2);
//        DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF2);
//        memcpy(temptemp, temptemp + NAV_RECVBUF_SIZE, NAV_RECVBUF_SIZE);
//        DMA_Cmd(DMA2_Stream1,ENABLE);
//    }
//}

///**
//  * @brief  串口2 DMA发送中断
//  * @param  None
//  * @retval None
//  */
//void DMA2_Stream6_IRQHandler(void)
//{
//    if (DMA_GetFlagStatus(DMA2_Stream6, DMA_IT_TCIF6) == SET)
//    {
//        DMA_Cmd(DMA2_Stream6, DISABLE);
//        DMA_SetCurrDataCounter(DMA2_Stream6, NAV_RECVBUF_SIZE);
//        DMA_ClearFlag(DMA2_Stream6, DMA_FLAG_TCIF6);
//        DMA_ClearITPendingBit(DMA2_Stream6, DMA_IT_TCIF6);
//    }
//}
