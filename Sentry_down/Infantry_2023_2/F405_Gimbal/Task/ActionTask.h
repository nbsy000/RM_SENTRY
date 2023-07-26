#ifndef __ACTIONTASK_H__
#define __ACTIONTASK_H__

#include "main.h"

/* 导航的总状态 */
enum NAV_STATE
{
	BEFOREGAME,	 // 比赛开始前
	TO_HIGHLAND, // 去高地
	TO_SOURCE,	 // 去资源导
	TO_PATROL,	 // 去巡逻区
	TO_OUTPOST,	 // 去前哨站
	OUTPOST,	 // 前哨站
	PATROL,		 // 巡逻区
	SOURCE,		 // 资源岛
	HIGHLAND,	 // 高地
	PATROL_SAFE, // 前哨战还在前的巡逻区状态
	TEST1,		 // 测试路线1
	TEST2,		 // 路线测试2
};

/*底盘、云台、发射机构自动模式状态*/
enum CHASSIS_GIMBAL_SHOOT_STATE
{
	NAV_STATE,	   // 导航状态
	PROTECT_STATE, // 小陀螺
	ARMOR_STATE, // 辅瞄状态
	STOP_STATE,	   // 模式更新
};

//导航路径路径状态
enum NAV_PATH_STATE
{
	CONTINUED,
	FINISHED,
};


typedef struct
{
	//接收数据包
	short NAV_x;
	short NAV_y;
	short NAV_w;

	enum NAV_STATE NAV_State;
	enum NAV_STATE Last_NAV_State;
	enum CHASSIS_GIMBAL_SHOOT_STATE Gimbal_PC_State;
	enum CHASSIS_GIMBAL_SHOOT_STATE Chassis_PC_State;
	enum CHASSIS_GIMBAL_SHOOT_STATE Shoot_PC_State;
	enum NAV_PATH_STATE NAV_Path_State;
	
	//状态延时变量
	double game_start_time;//比赛开始时间
	double pc_mode_time;//进入自动模开始时间
	double mode_update_time;//模式更新开始时间
	
  double GAME_START_INTERVAL;//比赛开始时间间隔
  double PC_MODE_INTERVAL;//进入自动模式时间间隔
  double MODE_UPDATE_INTERVAL;//模式更新时间间隔
} NAV_t;


/*模式选择结构体*/
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
/**PC****/
void NAV_Init(void);
void PC_Process(Remote rc);
void Navigation_State(void);
void NAV_State_Invert(void);
void NAV_State_Act(void);
void Chassis_Gimbal_Shoot_State(int Chassis_Mode, int Gimbal_Mode, int Shoot_Mode);

void ModeChoose_task(void *pvParameters);

extern NAV_t NAV_car;
#endif
