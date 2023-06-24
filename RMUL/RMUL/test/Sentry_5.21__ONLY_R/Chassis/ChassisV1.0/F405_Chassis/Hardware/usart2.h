#ifndef _USART2_H_
#define _USART2_H_

//#if VOFA

/******************************** VOFA 串口+DMA定义 ***********************************/
//串口波特率
#define      VOFA_USART_BAUD_RATE                       115200
//所用串口
#define      VOFA_USARTx                                USART2
//串口时钟
#define      VOFA_USART_APBxClock_FUN                   RCC_APB1PeriphClockCmd
#define      VOFA_USART_CLK                             RCC_APB1Periph_USART2
//串口GPIO口时钟
#define      VOFA_USART_GPIO_APBxClock_FUN              RCC_AHB1PeriphClockCmd
#define      VOFA_USART_GPIO_CLK                        RCC_AHB1Periph_GPIOA

//TX口
#define      VOFA_USART_TX_PORT                         GPIOA   
#define      VOFA_USART_TX_PIN                          GPIO_Pin_2
#define      VOFA_USART_TX_AF                           GPIO_AF_USART2
#define      VOFA_USART_TX_SOURCE                       GPIO_PinSource2 

//RX口
#define      VOFA_USART_RX_PORT                         GPIOA
#define      VOFA_USART_RX_PIN                          GPIO_Pin_3
#define      VOFA_USART_RX_AF                           GPIO_AF_USART2
#define      VOFA_USART_RX_SOURCE                       GPIO_PinSource3

//TX_DMA时钟
#define      VOFA_DMA_TX_AHBxClock_FUN                  RCC_AHB1PeriphClockCmd
#define      VOFA_DMA_TX_CLK                            RCC_AHB1Periph_DMA1

//TX_DMA通道
#define			 VOFA_DMA_TX_STREAM													DMA1_Stream6
#define			 VOFA_DMA_TX_CHANNEL												DMA_Channel_4
#define      VOFA_DMA_TX_IRQ                            DMA1_Stream6_IRQn
#define      VOFA_DMA_TX_INT_FUN                        DMA1_Stream6_IRQHandler
#define			 VOFA_DMA_FLAG_TCIF													DMA_FLAG_TCIF6
#define      VOFA_DMA_IT_STATUS                         DMA_IT_TCIF6
void VOFA_USART_Configuration(void);
void VOFA_Send(void);

//#elseif PC_ROS

//#define PC_SENDBUF_SIZE 24

//#define PC_RECVBUF_SIZE 9
//#define PC_SEND_BUF_SIZE 9
//#define SendToPC_Buff 10;
//void USART2_Configuration(void);
//#else 

//	#define mmWaveBufferSize 40

////	#define UART1_BUF_LEN                     256
////	#define maxDetectedObjectsNum             10

////	#define MAX_DST_Y             1.2
////	#define MIN_DST_Y             0.4
////	#define DST_Y             		0.7

////	#define MAX_ANGLE             60
////	#define MIN_ANGLE             -60

////	#define MAX_DST_Z             0.05
////	#define MIN_DST_Z             -0.05

////	#define MAX_DST_X							0.35
////	#define MIN_DST_X							-0.35

////	void USART2_Configuration(void);


//#endif

#endif
