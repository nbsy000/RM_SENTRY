#ifndef __TASK_SHOOT_H
#define __TASK_SHOOT_H

#define FrictionWheel_L_Speed_Low 5600
#define FrictionWheel_L_Speed_High 7000//µ¯ËÙ25
#define FrictionWheel_L_Speed_Off 0

#define FrictionWheel_R_Speed_Low 5600
#define FrictionWheel_R_Speed_High 7000
#define FrictionWheel_R_Speed_Off 0

#define BodanCurrentLimit 10000
#define BodanCurrentLimit 10000
#define FrictionCurrentLimit 10000

typedef struct
{
	uint16_t Angle_ABS; //0~8191(0x1fff)
	int32_t Angle_Inc;
	uint16_t LastAngle;
	int16_t RealSpeed; //RPM
	PID_Typedef pid_speed;
	PID_Typedef pid_pos;
	ZeroCheck_Typedef zerocheck;
	int16_t I_Set;
	uint8_t is_finish;
} _2006_motor_t;

//typedef struct
//{
//	uint16_t angle; //0~8191(0x1fff)
//	int16_t real_speed;	  //
//	PID_Typedef pid;
//	int16_t I_Set;
//} _3510_motor_t;

void task_Shoot(void *parameter);

void FrictionWheel_SetSpeed(int16_t tmpAccelerator0, int16_t tmpAccelerator1);
void Shoot_Disconnect_Act(void);

#endif //__TASK_SHOOT_H
