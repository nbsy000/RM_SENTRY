#ifndef __BSP_USART2_PC_H
#define __BSP_USART2_PC_H
#define PC_OLD

#ifdef PC_OLD


//#ifdef NEW_SHOOTAIM
//#define PC_SENDBUF_SIZE sizeof(PCSendData)
//#define PC_RECVBUF_SIZE sizeof(PCRecvData)

//#else
//#define PC_SENDBUF_SIZE 16
//#define PC_RECVBUF_SIZE 10

//#endif

#define PC_SENDBUF_SIZE sizeof(PCSendData)
#define PC_RECVBUF_SIZE sizeof(PCRecvData)

typedef struct
{
	float RCPitch;
	float RCYaw;
	int16_t ReceiveFromTx2BulletCnt;
	int16_t FrictionWheel_speed;
	int16_t DisConnect;
}PC_Recv_t;
#endif

#ifdef PC_ROS
#define PC_SENDBUF_SIZE 17
#define PC_RECVBUF_SIZE 23

typedef __packed struct
{
	unsigned char head; //[0]
	float RCPitch; 		//[1][2][3][4]
	float RCYaw;				//[5][6][7][8]
	short ReceiveFromTx2BulletCnt; //[9][10]
	short FrictionWheel_speed; //[11][12]
	short DisConnect;	//[13][14]
	short x_set;			//[15][16]
	short y_set;			//[17][18]
	short w_set;			//[19][20]
	unsigned char tail; //[21]
	unsigned char crc; //[22]
}PC_Recv_t;

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
}PC_Send_t;

#endif
void USART2_Configuration(void);

#endif //__BSP_USART2_PC_H
