#ifndef __BSP_USART2_PC_H
#define __BSP_USART2_PC_H


//#ifdef NEW_SHOOTAIM
#define PC_SENDBUF_SIZE sizeof(PCSendData)
#define PC_RECVBUF_SIZE sizeof(PCRecvData)

//#else
//#define PC_SENDBUF_SIZE 16
//#define PC_RECVBUF_SIZE 10

//#endif


void USART2_Configuration(void);

#endif 
