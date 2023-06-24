#ifndef __TASK_GimbalR_H
#define __TASK_GimbalR_H

#define LimitYaw   30000  //6020限制电压
#define LimitPitch 30000

//云台是否进行协同
enum Syne_Type
{
 SYNE,
 NO_SYNE,
};

//开启协同时俩云台状态
enum Syne_State
{
	SYNE_BOTH,
	SYNE_RIGHT,
	SYNE_LEFT,
	SYNE_NONE,
};

//巡航选择
enum Cruise_Type
{
	ONLY_PITCH,
	PITCH_YAW,
};

#define PATROL 1
#define NO_PATROL 0

#define GIMBAL_LEFT 0
#define GIMBAL_RIGHT 1

#define GYRO 0
#define GYRO_PITCH 2//只有Pitch用陀螺仪角速度
#define MOTOR 1

#define DEBUG_PID 1
#define NO_DEBUG_PID 0

//-------------------结构体定义------------------//
//云台电机类
typedef struct
{
  uint16_t Angle_ABS; //角度
  int16_t real_flow;  //实际转矩电流测量
	int16_t real_speed;  //实际转速 单位rpm
  

	float actualAngle;//当前实际的角度值，目前是使用陀螺仪的
	float motoAngle;//当前实际电机角
	float gyroAngle;//当前实际陀螺仪角度
	float Gyro_Speed;
	float PCsetAngle;//PC自动的设置值
	
	//普通PID
  PID_Typedef PidSpeed;
  PID_Typedef PidPos;
  FuzzyPID PidSpeedF;      //模糊用的PID参数
  FuzzyPID PidPosF;

	FeedForward_Typedef FF;
	
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
	
	//重力补偿系数
	float K1;
	float K2;
	
	//低通系数
	float K_LP;
}gimbal_motor_t;


//云台类
typedef struct
{
	uint8_t armor_state;//瞄准状态
	uint8_t moto_gyro_pid_flag;//pid反馈标志位
	uint8_t GimbalType;//区分左右
	uint8_t ModeUpdate_Flag;//模式更新
	uint8_t LastMode;//上一次模式
	uint8_t patrol_dir_pitch;
	uint8_t patrol_dir_yaw;
	
	gimbal_motor_t *Pitch;
	gimbal_motor_t *Yaw;
	
	float aim_Pitch;
	float aim_Yaw;
	uint8_t target_id;
	
}Gimbal_Typedef;

void task_GimbalR(void* parameter);

float CombPitchOutput(uint8_t Gimbal_Type); //获取滤波后的pitch角度的函数
float CombYawOutput(uint8_t Gimbal_Type);   //获取滤波后的yaw角度的函数
void Gimbal_Limit_Init(void);//云台角度限幅函数
void PID_Gimbal_Init(void);
void Gimbal_GYRO_Cal(void);
void Gimbal_Init(void);//初始化
float Gimbal_Limit(gimbal_motor_t Gimbal_Motor,uint8_t TYPE);

void Gimbal_PC_Act(uint8_t Syne_Flag);
void Gimbal_PC_Act2(void);
void Gimbal_RC_Act(Gimbal_Typedef *Gimbal);
void Gimbal_SLEEP_Act(Gimbal_Typedef *Gimbal);
void Gimbal_DEBUG_Act(Gimbal_Typedef *Gimbal);
void Gimbal_AIM_Act(Gimbal_Typedef *Gimbal);

void Gimbal_Attack_Both(void);
void Gimbal_Attack_Left(void);
void Gimbal_Attack_Right(void);
void Gimbal_Attack_None(void);
void Gimbal_Attack_Nosyne(void);

void Gimbal_PID_Cal(Gimbal_Typedef *Gimbal,uint8_t Feedback_Type,uint8_t Is_Debug);

void Aim_Disconnect_Act(void);
void Gimbal_Cruise(Gimbal_Typedef *Gimbal,uint8_t Cruise_Mode);

extern Gimbal_Typedef Gimbal_R,Gimbal_L;
extern gimbal_motor_t MotoPitch, MotoYaw,MotoPitch_L,MotoYaw_L;

#endif //__TASK_GimbalR_H
