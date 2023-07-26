#ifndef __TASK_GIMBALUP_H
#define __TASK_GIMBALUP_H

#define gyroLimitYaw   30000
#define gyroLimitPitch 30000 

//-------------------结构体定义------------------//
//云台电机类
typedef struct
{
  uint16_t Angle_ABS; //角度
  int16_t real_flow;  //实际转矩电流测量
	int16_t real_speed;  //实际转速 单位rpm
  

	float actualAngle;//当前实际的角度值
	float PCsetAngle;//PC自动的设置值
	
	//普通PID
  PID_Typedef PidSpeed;
  PID_Typedef PidPos;
//  PID_Typedef PidSpeedV;      //视觉用的PID参数
//  PID_Typedef PidPosV;
	//模糊PID
//	Fuzzy_Typedef PidSpeed_Fuzzy;
//	Fuzzy_Typedef PidPos_Fuzzy;
//	Fuzzy_Typedef PidSpeedV_Fuzzy;
//	Fuzzy_Typedef PidPosV_Fuzzy;

//	FeedForward_Typedef FF;
	
  ZeroCheck_Typedef zerocheck;
  int32_t Angle_Inc;  //过零检测后得到的角度
  int16_t I_Set;      //转矩电流设定 
	int16_t I_Set_Fuzzy;
	
	//软件限位
	int32_t MAX_ANGLE;
	int32_t MIN_ANGLE;
	int32_t ZERO_POS;
	
	//自动移动时的限位
	int32_t PATROL_MAX_ANGLE;
	int32_t PATROL_MIN_ANGLE;
	int32_t PATROL_ZERO_POS;
}gimbal_motor_t;

//云台类
typedef struct
{
	uint8_t armor_state;//瞄准状态
	
	uint8_t moto_gyro_pid_flag;//pid反馈标志位
	
	uint8_t GimbalType;//区分左右
	
	gimbal_motor_t Pitch;
	gimbal_motor_t Yaw;
	
	float aim_Pitch;
	float aim_Yaw;
	
}Gimbal_Typedef;


void task_Gimbal_L(void* parameter);

extern gimbal_motor_t MotoPitch, MotoYaw,MotoPitch_L,MotoYaw_L;
extern Gimbal_Typedef Gimbal_L;

#endif //__TASK_GIMBALUP_H
