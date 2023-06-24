#ifndef __CHASSISTASK_H__
#define __CHASSISTASK_H__
#include "main.h"

#define YES 1
#define NO  0

//开启协同时俩云台状态
enum Syne_State
{
	SYNE_BOTH,
	SYNE_RIGHT,
	SYNE_LEFT,
	SYNE_NONE,
};

/* PC模式下总的状态 */
enum PC_STATE
{
    BEFOREGAME,//比赛开始前
    TOPATH1,//路径1去
    BACKPATH1,//路径1回
    OUTPOST,//前哨站
    PATROL,//巡逻区
		SOURCETO,
		SOURCEBACK,
		PATROL_SAFE,//前哨战还在前的巡逻区状态
		TEST1,//测试路线1
		TEST2,//路线测试2
};

//固定路径的里程计反馈
enum NAV_FEEDBACK
{
		ENCODER,//编码器
		RADAR,//雷达
};

//固定路径状态
enum NAV_STATE
{
		CONTINUED,//进行中
		FINISHED,//完成
};


typedef struct Power{
	short Actual_P_max;					//最大功率
	short Self_Protect_Limit;			//小陀螺转速限制
	float k_BAT;						//不同功率下的直行速度上线
	short Excess_P_max_J;				//起步时无输出滤波最大超限功率
	short Excess_P_max_P;     			//稳定时输出滤波后最大超限功率
	short CurrentMax;					//最大输出电流
	short Follow_W;         			//跟随速度上限
}Power_Typedef;


//底盘电机相关
typedef struct{
	
	//电调发送的数据
	short Encoder;   //这个是编码值，0-8191，对应于0-360度
	short RealSpeed;  //转子转速，单位RPM  电机空转转速位482rpm，对应于转子大约为 482*19
	short Current;// 实际电流值，-16384-16384，对应于-20A到20A
	
	//发送给电调的电流
	short sendCurrent;// 发送电流，-16384-16384，对应于-20A到20A

	//过零检测
	ZeroCheck_Typedef ZeroCheck_Motor;
	
	//过零检测后的编码器的值
	float	Encoder_INC;
	float Inencoder;//编码器增量
		
	int total_x;
}Chassis_Motor_t;

//底盘状态，以下数据都是用于导航，所以坐标系都是世界坐标系
typedef struct
{
    float X;
    float Y;
    float Alpha;

    float Vx;
    float Vy;
    float w;
} ChassisNavTypedef;

typedef struct 
{
	//车的目标控制速度，以云台为坐标系或者以车为坐标系
	short carSpeedx;
	short carSpeedy;
	short carSpeedw;
	
	//结算出的实际速度，这些速度都是以车为坐标系
	float insX;//瞬时速度
	float insY;
	float insW;
	
	//绝对坐标系
	float totalX;//
	float totalY;//
	float Yangle;//绝对坐标系x的角度
	float nowYangle;//当前时刻的车的Y向角度	
	float Alpha;//偏航角
	
	//绝对坐标系
	float aimVx;
	float aimVy;	
	
	//导航路径选择
	uint8_t NavigatePathNum;
	
	//世界坐标系下的数据
	ChassisNavTypedef CurrentState;  //底盘现状态，位置、角度、速度...
  ChassisNavTypedef AimState;     //底盘期望状态

	uint8_t armor_hurt_flag[4];//收到攻击
	
	//自动模式下所属状态
	uint8_t PC_State;
	uint8_t Last_PC_State;
	uint8_t NAV_State;//固定路径状态
	uint8_t Barrier_flag;//障碍物标志
	float Radar_totalX;
	float Radar_totalY;
	float N_Yaw_angle_init;
	
	float Init_X;
	float Init_Y;
	
} ChassisState_t;
//底盘类

void Chassis_Speed_Cal(void);
void PowerLimit(void);
void Current_Filter_Excu(void);

void Pid_ChassisWheelInit(void);
void Chassis_task(void *pvParameters);

void Chassis_Power_Control_Init(void);
void Pid_SpeedCurrent(float *);

void Chassis_Patrol_Act(void);
void Chassis_Patrol_Act1(uint8_t Mode);
void Chassis_Patrol_Act2(void);
void Chassis_RC_Act(void);
void Chassis_SLEEP_Act(void);
void Chassis_DEBUG_Act(void);
void Chassis_Protect_Act(void);

void Patrol_Act(void);
void NAV_PATH_Act(uint8_t Mode);
void Outpost_Act(void);
void Patrol_Safe_Act(void);

void Position_Control(float aim_x,float aim_y,float Theta);

extern ChassisState_t chassis;
#endif
