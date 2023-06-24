#include "main.h"
#include "uart5.h"

uint8_t NAV_Recv_Buf[NAV_RECVBUF_SIZE] = {0, 0, 0};
uint8_t NAV_Send[NAV_SENDBUF_SIZE];
NAV_Recv_t NAV_Recv;
/**********************************************************************************************************
*函 数 名: UART5_Configuration
*功能说明: usart5配置函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void UART5_Configuration(void)
{
    USART_InitTypeDef usart5;
		GPIO_InitTypeDef  gpio;
		NVIC_InitTypeDef  nvic;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);
	
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5); 
	
    gpio.GPIO_Pin = GPIO_Pin_2;
		gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOD,&gpio);

	  gpio.GPIO_Pin = GPIO_Pin_12;
		gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOC,&gpio);
		
		usart5.USART_BaudRate = 115200;
		usart5.USART_WordLength = USART_WordLength_8b;
		usart5.USART_StopBits = USART_StopBits_1;
		usart5.USART_Parity = USART_Parity_No;
		usart5.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
    usart5.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(UART5,&usart5);
		
		USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);  
		USART_Cmd(UART5,ENABLE);
		USART_DMACmd(UART5,USART_DMAReq_Rx,ENABLE);	
		
//		nvic.NVIC_IRQChannel = UART5_IRQn;
//    nvic.NVIC_IRQChannelPreemptionPriority = 1;
//    nvic.NVIC_IRQChannelSubPriority = 1;
//    nvic.NVIC_IRQChannelCmd = ENABLE;

		nvic.NVIC_IRQChannel = DMA1_Stream0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
		
		{
			DMA_InitTypeDef  dma;
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
			DMA_DeInit(DMA1_Stream0);
			dma.DMA_Channel= DMA_Channel_4;
			dma.DMA_PeripheralBaseAddr = (uint32_t)&(UART5->DR);
			dma.DMA_Memory0BaseAddr = (uint32_t)NAV_Recv_Buf;
			dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
			dma.DMA_BufferSize = NAV_RECVBUF_SIZE;
			dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
			dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
			dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
			dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
			dma.DMA_Mode = DMA_Mode_Circular;
			dma.DMA_Priority = DMA_Priority_VeryHigh;
			dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
			dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
			dma.DMA_MemoryBurst = DMA_Mode_Normal;
			dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
			DMA_Init(DMA1_Stream0,&dma);
			DMA_ITConfig(DMA1_Stream0,DMA_IT_TC,ENABLE);
			DMA_Cmd(DMA1_Stream0,ENABLE);
		}	
}

/**********************************************************************************************************
*函 数 名: DMA1_Stream0_IRQHandler
*功能说明: usart5 DMA接收中断
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
uint8_t tempNAV[NAV_RECVBUF_SIZE]; //这里要改为全局变量，不然crc不通过
int16_t Crcpass5, crcNopass5;
int16_t buffindex5;
/**
  * @brief  串口2 DMA接收中断
  * @param  None
  * @retval None
  */
void DMA1_Stream0_IRQHandler(void)
{
    static uint8_t temptemp[2 * NAV_RECVBUF_SIZE];
    int16_t PackPoint, n;
    if (DMA_GetITStatus(DMA1_Stream0, DMA_IT_TCIF0))
    {
//        DMA_Cmd(DMA1_Stream0,DISABLE);
        memcpy(temptemp + NAV_RECVBUF_SIZE, NAV_Recv_Buf, NAV_RECVBUF_SIZE);
        for (PackPoint = 0; PackPoint < NAV_RECVBUF_SIZE; PackPoint++) //防止错位，不一定数组元素的第一个就为
        {
            if (temptemp[PackPoint] == '!')
            {
                for (n = 0; n < NAV_RECVBUF_SIZE; n++)
                {
                    tempNAV[n] = temptemp[(n + PackPoint)];
                }
                crcNopass5++;
                if (Verify_CRC8_Check_Sum(tempNAV, NAV_RECVBUF_SIZE))
                    NAVReceive(tempNAV);
                else
                {
                    buffindex5++;
                    buffindex5 = buffindex5 % 4;
                }
                break;
            }
        }
        DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0);
        DMA_ClearITPendingBit(DMA1_Stream0, DMA_IT_TCIF0);
        memcpy(temptemp, temptemp + NAV_RECVBUF_SIZE, NAV_RECVBUF_SIZE);
        DMA_Cmd(DMA1_Stream0,ENABLE);
    }
}