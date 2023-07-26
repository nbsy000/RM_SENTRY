#ifndef __UART5_H__
#define __UART5_H__

#define NAV_SENDBUF_SIZE 3
#define NAV_RECVBUF_SIZE 15

typedef struct
{
	unsigned char head;		//[0]
	short x_now;			//[1][2]
	short y_now;			//[3][4]
	short w_now;			//[5][6]
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

#endif
