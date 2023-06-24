#ifndef __RDM_L_CONTROL_H
#define __RDM_L_CONTROL_H

#include "main.h"

//定义广播模式
#define BOARDCAST

#define RMD_L_ID  				0x141   //canID，自己的设定
#define RMD_BCAST					0x280

#define RMD_CAN					CAN1	//使用can1发送接收电机消息

#define MOTOR_CLOSE    					0x80  	//关闭电机，同时清除电机运行状态和之前接收到的控制指令
#define MOTOR_STOP							0x81  	//停止电机，但不清理电机之前的控制指令
#define MOTOR_START							0x88  	//运行电机，恢复停止电机指令
#define READ_ENCODER    				0x90 	//读取编码器指令，返回帧包括编码器的相对值、绝对值和零值，（我认为没必要用）
#define READ_SINGLE_ANGLE 			0x94 	//读取单圈角度指令
#define READ_MULTI_ANGLE 				0x92 	//读取多圈角度指令
#define READ_POWER							0x71	//读取功率
#define MOTOR_STATE_TISE 				0x9C 	//读取电机状态指令，包括温度，电流，速度，编码
#define I_CONTROL								0xA1 	//转矩控制指令，输出电流控制力矩
#define SPEED_CONTROL 					0xA2 	//速度控制指令
#define MULTI_ANGLE_CONTROL 		0xA3	//多圈位置控制指令，不重新设置限速
#define SINGLE_ANGLE_CONTROL 		0xA5	//单圈位置控制指令，不重新设置限速
#define INCREMENT_ANGLE_CONTROL 0xA7	//增量位置控制，不重新设置限速


//9025电机的数据  成员的类型是根据协议中变量的类型定的
typedef struct{
	//控制值
	int C_MAngle;//多圈角度控制值  数值类型int 单位0.01度
	uint16_t C_SAngle;//单圈控制值  发送数值类型 uint16_t 范围0-35999，对应0-359.99
	int C_IAngle;//增量角度控制
	int C_Speed;//速度控制值 单位0.01dps
	short C_I;//电流控制值 数值范围 -2000-2000 对应-32A-32A
	
	//接收值
	short I; //电流值 -2048-2048 对应电流为-33A-33A
	short speed;   //速度    单位为dps，即度每秒
	uint16_t encoder; //编码器  0-16383   
	uint16_t signalAngle; //根据编码器换算成角度值 0-35999  单位0.01度
	int64_t	multiAngle;//多圈角度值，有正负  单位0.01度
	int16_t power;//实际功率
	
	ZeroCheck_Typedef zerocheck;
	
	//普通PID
  Pid_Typedef PidSpeed;
  Pid_Typedef PidPos;
	
	uint16_t Yaw_init;//Yaw轴初始值
	//自动模式
	uint8_t aim_flag;
	float PCYaw;
	float SYNEYaw;
	
	
}Motor_9025_t;

void Chassis_Yaw_Close(void);
void Chassis_Yaw_Stop(void);
void Chassis_Yaw_Start(void);
void Get_ChassisYaw_State(void);
void Get_ChassisYaw_MAngle(void);
void Get_ChassisYaw_Power(void);
void Yaw_I_Control(short I);
void Yaw_Speed_Control(int speed);
void Yaw_MAngle_Control(int Angle);
void Yaw_SAngle_Control(uint16_t Angle);
void Yaw_IAngle_Control(int Angle);

void Chassis_Yaw_Receive(uint8_t* Data);

extern Motor_9025_t Motor_9025;
extern uint16_t encoder;
extern uint16_t signalAngle;
extern int64_t	multiAngle;

#endif
