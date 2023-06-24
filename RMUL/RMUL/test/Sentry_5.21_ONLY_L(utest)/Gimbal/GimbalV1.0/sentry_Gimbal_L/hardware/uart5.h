#ifndef __UART5_H__
#define __UART5_H__ 

#define NAV_SENDBUF_SIZE 16
#define NAV_RECVBUF_SIZE 15

typedef  struct
{
	unsigned char head; //[0]
	short x_now;			//[1][2]
	short y_now;			//[3][4]
	short w_now;			//[5][6]
	uint8_t Barrier_Flag;//[]
	unsigned char crc; //[14]
}NAV_Recv_t;

typedef  struct
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

void UART5_Configuration(void);

#endif
