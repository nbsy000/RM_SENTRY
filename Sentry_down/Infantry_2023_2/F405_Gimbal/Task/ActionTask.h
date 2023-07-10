#ifndef __ACTIONTASK_H__
#define __ACTIONTASK_H__

#include "main.h"

/* 导航的总状态 */
enum NAV_STATE
{
	BEFOREGAME,	 // 比赛开始前
	OUTPOST,	 // 前哨站
	PATROL,		 // 巡逻区
	SOURCE,		 // 资源岛
	HIGHLAND,	 // 高地
	TO_OUTPOST,	 // 去前哨站
	TO_SOURCE,	 // 去资源导
	TO_PATROL,	 // 去巡逻区
	TO_HIGHLAND, // 去高地
	PATROL_SAFE, // 前哨战还在前的巡逻区状态
	TEST1,		 // 测试路线1
	TEST2,		 // 路线测试2
};

/*底盘、云台、发射机构自动模式状态*/
enum CHASSIS_STATE
{
	NAV_STATE,	   // 导航状态
	PROTECT_STATE, // 小陀螺
	ARMOR_STATE, // 辅瞄状态
	STOP_STATE,	   // 模式跟新
};


/*步兵模式选择结构体*/
typedef struct
{
	short ControlMode;
	short ChassisMode;
	short ShootMode;
	short GimbalMode;
	short RstMode;
	short BulletSpeedMode;
	float BulletSpeed;
	short UseSw_NoSw;
} Status_t;

#define Control_RC_Mode 300
#define Control_MouseKey_Mode 301
#define Control_Powerdown_Mode 302
#define Control_PC_Mode 303

#define Chassis_Powerdown_Mode 0
#define Chassis_Act_Mode 1
#define Chassis_SelfProtect_Mode 2
#define Chassis_Solo_Mode 3
#define Chassis_Jump_Mode 4
#define Chassis_Test_Mode 5
#define Chassis_PC_Mode 6

#define Gimbal_Powerdown_Mode 7
#define Gimbal_Act_Mode 3
#define Gimbal_Armor_Mode 0
#define Gimbal_BigBuf_Mode 2
#define Gimbal_DropShot_Mode 4
#define Gimbal_SI_Mode 5
#define Gimbal_Jump_Mode 6
#define Gimbal_SmlBuf_Mode 1
#define Gimbal_Test_Mode 9
#define Gimbal_PC_Mode 10

#define Shoot_Fire_Mode 344
#define Shoot_Powerdown_Mode 345
#define Shoot_Check_Mode 346
#define Shoot_Tx2_Mode 347
#define Shoot_PC_Mode 348

void SetInputMode(Remote);
void Status_Act(void);

void MouseKey_Act_Cal(RC_Ctl_t RC_Ctl);

void Remote_Process(Remote rc);
void Mouse_Key_Process(RC_Ctl_t RC_Ctl);
void Powerdown_Process(void);
void Tx2_Off_Test(Remote rc);
void PC_Process(Remote rc);
void Navigation_State(void);
void Chassis_Gimbal_Shoot_State(int Chassis_Mode, int Gimbal_Mode, int Shoot_Mode);

void ModeChoose_task(void *pvParameters);

extern uint8_t Gimbal_State_Update;
extern uint8_t Chassis_State_Update;
#endif
