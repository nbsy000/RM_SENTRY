#ifndef __HW_USART6_H
#define __HW_USART6_H

#define NAV_SENDBUF_SIZE 16
#define NAV_RECVBUF_SIZE 10

typedef __packed struct
{
	unsigned char head; //[0]
	int x_now;			//[1][2][3][4]
	int y_now;			//[5][6][7][8]
	unsigned char crc; //[9]
}NAV_Recv_t;

typedef __packed struct
{
	unsigned char head; //[0]
	float Pitch_now; 		//[1][2][3][4]
	float Yaw_now;				//[5][6][7][8]
	short x_now;			//[9][10]
	short y_now;			//[11][12]
	short w_now;			//[13][14]
	unsigned char tail; //[15]
	unsigned char crc; //[16]
}NAV_Send_t;

void USART6_Configuration(void);

#endif 

