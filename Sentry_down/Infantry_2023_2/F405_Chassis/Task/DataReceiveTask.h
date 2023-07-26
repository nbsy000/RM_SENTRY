#ifndef __DATARECEIVETASK_H
#define __DATARECEIVETASK_H
#include "main.h"

#define Chassis_Powerdown_Mode 					0
#define Chassis_Act_Mode  							1
#define Chassis_SelfProtect_Mode 				2
#define Chassis_Solo_Mode  							3
#define Chassis_Jump_Mode               4
#define Chassis_Test_Mode               5
#define Chassis_PC_Mode									6

#define Gimbal_Powerdown_Mode 					7
#define Gimbal_Act_Mode 								3
#define Gimbal_Armor_Mode  							0
#define Gimbal_BigBuf_Mode 						  2
#define Gimbal_DropShot_Mode 						4
#define Gimbal_SI_Mode 									5
#define Gimbal_Jump_Mode                6
#define Gimbal_AntiSP_Mode              7
#define Gimbal_SmlBuf_Mode              1

typedef struct{
	char SuperPowerLimit;	  //0为超级电容关闭，不为0则开启使用超级电容
	char Chassis_Flag;			//模式见上
	char AutoFire_Flag;					//0表示手动开火，1为自动开火
	char Laser_Flag;				//0表示激光关闭，1为打开
	short Pitch_100;				//pitch角度,乘了100之后发
   short Yaw_100;			    	//yaw角度,乘了100之后发
	char Gimbal_Flag;				//模式见上
	char Graphic_Init_Flag;	//0为进入初始化模式，1为初始化结束
	char Freq_state;			  //射频状态，0表示正常射频，1表示高射频
    char Enemy_ID;
	/*打包数据*/
	char Send_Pack1;	
	char Fric_Flag;
	
	char PC_Mode;//PC状态下的模式
}F405_typedef;

enum ARMOR_ID
{
    ARMOR_AIM_LOST = 0,
    ARMOR_ID_1,
    ARMOR_ID_2,
    ARMOR_ID_3,
    ARMOR_ID_4,
    ARMOR_ID_5,
    ARMOR_ID_Sentry,
};

typedef struct{
	short Angle;
	short RealSpeed;  
  short Current;	
}RM820RReceive_Typedef;

typedef struct 
{
  short carSpeedx;
	short carSpeedy;
	short carSpeedw;
	
	short Last_carSpeedx;
	short Last_carSpeedy;
	short Last_carSpeedw;
	
	short ABSLastcarSpeedx;
	short ABSLastcarSpeedy;
	short ABSLastcarSpeedw;
} ChassisSpeed_t;

typedef struct
{
  char HeatUpdate_NoUpdate;
	char SpeedUpdate_NoUpdate;
	
	//0x101
	uint32_t event;
	uint8_t self_outpost_hp;
	uint16_t self_base_hp;
	
	//0x0201
	uint8_t robot_id;
	uint8_t RobotLevel;
	uint16_t remainHP;
	uint16_t maxHP;
	uint16_t HeatCool17;		//17mm枪口每秒冷却值
	uint16_t HeatMax17;			//17mm枪口热量上限
	uint16_t BulletSpeedMax17;	//17mm枪口上限速度
	uint16_t MaxPower;			//底盘功率限制上限

	//0x0202
	uint16_t realChassisOutV;
	uint16_t realChassisOutA;
	float realChassispower;
	uint16_t remainEnergy;       //剩余能量
	short shooterHeat17;

	//0x0203
	 float x;
	 float y;
	 float z;
	 float angle;	
	
	//0x0207
	uint8_t bulletFreq;		//射击频率
	uint8_t ShootCpltFlag; //已射出一发子弹标志位
	
	//0x0208
  uint16_t num_17mm;
  uint16_t num_coin;
	
	//flag
	short HeatUpdateFlag;	
	
	//not in use
	uint8_t cardType;
	uint8_t CardIdx;

	float bulletSpeed;		//当前射速
	float LastbulletSpeed;
    
	uint8_t game_progress;
	
	uint16_t enemy_3_hp;
	uint16_t enemy_4_hp;
	uint16_t enemy_5_hp;
	
	uint16_t outpost_hp;//前哨站血量
	
	uint16_t remain_time;
	
	//0x303 雷达的云台手
	float target_position_x;//目标x m
	float target_position_y;
	float target_position_z;
	uint8_t commd_keyboard;
	uint16_t target_robot_ID;
	
}
JudgeReceive_t;


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


void Can1Receive0(CanRxMsg rx_message0);
//void Can2Receive0(CanRxMsg *rx_message);
void Can2Receive1(CanRxMsg *rx_message);

void JudgeBuffReceive(unsigned char ReceiveBuffer[],uint16_t DataLen);

void F405_Rst(void);
void JudgeReceive_task(void);


#endif
