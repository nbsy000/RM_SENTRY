#ifndef __UART5_H__
#define __UART5_H__ 

typedef __packed struct
{
	unsigned char head;		//[0]
	float x_now;			//[1][2]
	float y_now;			//[3][4]
	float w_now;			//[5][6]
	unsigned char nav_path_state; //[]
	unsigned char crc;		//[14]
} NAV_Recv_t;

typedef __packed struct
{
	unsigned char head; //[0]
	unsigned char nav_state;//[1]
	unsigned char crc;	//[2]
} NAV_Send_t;

void UART5_Configuration(void);

extern uint8_t NAV_Recv_Buf[];
extern uint8_t NAV_Send_Buf[];
extern NAV_Recv_t NAV_Recv;
extern NAV_Send_t NAV_Send;

#define NAV_SENDBUF_SIZE sizeof(NAV_Send)
#define NAV_RECVBUF_SIZE sizeof(NAV_Recv)

#endif
