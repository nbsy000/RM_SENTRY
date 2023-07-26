#ifndef __DATARECEIVETASK_H
#define __DATARECEIVETASK_H
#include "main.h"

#define Remote_Control_ID  				0x105   //线上遥控器数据的报头ID

//陀螺仪相关
#define Gyro_Pitch_ID 0x100
#define Gyro_Yaw_ID 0x101

#define NAV_DATA_ID 0x107
#define GIMBAL_SYNE_ID 0x306


/*遥控器结构体*/
typedef __packed struct
{
		unsigned short ch0;
		unsigned short ch1;
		unsigned short ch2;
		unsigned short ch3;
		unsigned short s1;
		unsigned short s2;
}Remote;
/*鼠标结构体*/
typedef __packed 	struct
{
		short x;
		short y;
		short z;
		unsigned char press_l;
		unsigned char press_r;
}Mouse;
/*键盘结构体*/
typedef __packed struct
{
		unsigned short w,s,a,d,q,e,r,f,g,z,x,c,v,b,shift,ctrl;
}Key;

/*遥键鼠结构体综合*/
typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
  char RCrecvd,RCDisconnectCnt;//RCrecvd为数据接收标志位
}RC_Ctl_t;


typedef struct GYRO
{
	float GX;
	float GY;
	float GZ;
	float PITCH;
	float ROLL;
	float YAW_ABS;

	float YAW_INC;
	float PITCH_INC;

} Gyro_Typedef;

//接收到的速度
typedef struct
{	
	short now_x;
	short now_y;
	short now_w;
	
	uint8_t Stop_flag;
}ActReceive_t;


typedef struct
{
  char HeatUpdate_NoUpdate;
	char SpeedUpdate_NoUpdate;
	
	//0x001包头相关
	uint8_t game_type;
	uint8_t game_progress;
	uint16_t remain_time;  
	

	//0x003包头相关
	uint16_t red_outpost_hp;
	uint16_t blue_outpost_hp;
	
	//0x101相关
	uint32_t event;
	
	//0x0201
	uint8_t robot_id;
	uint8_t RobotLevel;
	uint16_t remainHP;
	uint16_t maxHP;
	uint16_t HeatCool17_0;		//17mm枪口每秒冷却值  0id
	uint16_t HeatMax17_0;			//17mm枪口热量上限
	uint16_t BulletSpeedMax17_0;	//17mm枪口上限速度
	uint16_t HeatCool17_2;
	uint16_t HeatMax17_2;
  uint16_t BulletSpeedMax17_2;
	uint16_t MaxPower;			//底盘功率限制上限

	//0x0202
	uint16_t realChassisOutV;
	uint16_t realChassisOutA;
	float realChassispower;
	uint16_t remainEnergy;       //剩余能量
	short shooterHeat17_0;
	short shooterHeat17_2;
	
	//0x204包头相关
	uint8_t RobotBuffState;
	uint8_t HPBuff;
	uint8_t CoolBuff;
	uint8_t DefenceBuff;
	uint8_t AttackBuff;
	
	//0x0207
	uint8_t bulletFreq;		//射击频率
	uint8_t ShootCpltFlag_0; //已射出一发子弹标志位
	uint8_t ShootCpltFlag_2;
	   
	//0x206包头相关 (伤害状态)
	uint8_t hurt_armor_id;//受击的装甲模块ID号
	uint8_t hurt_type;//受击类型：
	
	//0x303 雷达的云台手
	float target_position_x;//目标x m
	float target_position_y;
	float target_position_z;
	uint8_t commd_keyboard;
	uint16_t target_robot_ID;
	
	//flag
	short HeatUpdateFlag;	
	
	//not in use
	uint8_t cardType;
	uint8_t CardIdx;

	float bulletSpeed;		//当前射速
	float LastbulletSpeed;
}
JudgeReceive_t;

//接收消息的数量(目前只针对can)
typedef struct
{
	short ChassisMotorF[4];//检测周期内接收帧数
	float ChassisMotorR[4];//接收率
	
	short ChassisGyroF;
	float ChassisGyroR;
	
	short YawMotorF;
	float YawMotorR;
	
#ifdef NEW_INS	
	short ChassisYaw_100F;
	float ChassisYaw_100R;
	
	short ChassisYaw_101F;
	float ChassisYaw_101R;
#else
	short ChassisYawGyroF;
	float ChassisYawGyroR;
#endif
	
	short heatF;
	float heatR;
	
	short RemoteF;
	float RemoteR;

}Frame_Rate_t;


void Can1Receive0(CanRxMsg rx_message0);
void Can1Receive1(CanRxMsg rx_message1);
void Can2Receive0(CanRxMsg rx_message0);
void Can2Receive1(CanRxMsg rx_message1);

void JudgeBuffReceive(unsigned char ReceiveBuffer[],uint16_t DataLen);

void RC_Rst(void);

void Chassis_Rst(void);
void PC_Receive(unsigned char PCReceivebuffer[]);
void Frame_Acceptance_Rate(float Rate);

extern RC_Ctl_t RC_Ctl;
extern Frame_Rate_t FrameRate;//记录接收帧数
extern Gyro_Typedef Gyro_Chassis;//底盘陀螺仪数据
extern Gyro_Typedef Gyro_ChassisYaw;//大Yaw轴数据
extern JudgeReceive_t JudgeReceive;
extern ActReceive_t PCReceive;//PC发送的速度

#endif
