#ifndef __CHASSISTASK_H__
#define __CHASSISTASK_H__
#include "main.h"
typedef struct
{
	short carSpeedx;
	short carSpeedy;
	short carSpeedw;
} ChassisSpeed_t;

typedef __packed struct
{
	short NAV_x; 
	short NAV_y; 
	short NAV_w; 
	unsigned char Barrier_Flag;
	uint8_t NAV_State;
	uint8_t Last_NAV_State;
} NAV_t;

void Chassis_Powerdown_Cal(void);
void Chassis_Act_Cal(Remote rc, Key key);
void Chassis_SelfProtect_Cal(Remote rc, Key key);
void Chassis_Solo_Cal(Remote rc, Key key);
void Chassis_Jump_Cal(Remote rc, Key key);

float ChassisPostionAngle_TranSform(short InitPos);

void Pid_ChassisPosition_Init(void);

void Chassis_CurrentPid_Cal(void);

void Current_Filter_Excu(void);
void Current_Set_Jump(void);

void Chassis_PC_Cal(void);

void Chassis_Flag_Set(void);
void Chassis_task(void *pvParameters);

extern NAV_t NAV_car;
#endif
