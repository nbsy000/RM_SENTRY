#ifndef __TASK_DATASEND_H
#define __TASK_DATASEND_H

//上云台发送数据的报头
#ifndef Remote_Control_ID
#define Remote_Control_ID 0x105  //线上遥控器数据的报头ID
#endif 

//云台协同
#define Gimbal_SYNE_ID 0x306



typedef struct
{
	short RemoteF;
	float RemoteR;
	uint8_t RemoteFlag;
	
	short SyneF;
	float SyneR;
	uint8_t SyneFlag;
	
	short OthersF;
	float OthersR;
	uint8_t OthersFlag;
}Send_Rate_t;


void PitchYaw_Can2Send(int16_t pitchR, int16_t yawR,int16_t pitchL,int16_t yawL);
void Bodan_Can1Send(int16_t bodanVal,int16_t fric0, int16_t fric1);
void Remote_Send(void);
void Down_CAN2Send(void);
void Gimbal_Syne_Send(float ChassisYaw_Angle);
void Frame_Send_Rate(float Rate);
void Classify_Send_Msg(uint8_t Buf[]);

extern Send_Rate_t SendRate;

#endif //__TASK_DATASEND_H
