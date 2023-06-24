#ifndef __TASK_ACTIONUPDATE_H
#define __TASK_ACTIONUPDATE_H

typedef struct
{
	uint8_t Chassis_Mode;
	uint8_t ChassisYaw_Mode;
	uint8_t Shoot_R_Mode;
  uint8_t Shoot_L_Mode;
	uint8_t Gimbal_R_Mode;
  uint8_t Gimbal_L_Mode;
} State_t;

#define Chassis_RC 0x01
#define Chassis_Patrol 0x02
#define Chassis_SLEEP 0x00
#define Chassis_DEBUG 0x03
#define Chassis_Protect 0x04

#define ChassisYaw_RC 0x01
#define ChassisYaw_PC 0x02
#define ChassisYaw_STOP 0x03
#define ChassisYaw_DEBUG 0x04
#define ChassisYaw_SLEEP 0x00

#define Shoot_R_RC 0x01
#define Shoot_R_PC 0x02
#define Shoot_R_SLEEP 0x00
#define Shoot_R_DEBUG 0x09

#define Gimbal_R_RC 0x01
#define Gimbal_R_PC 0x02
#define Gimbal_R_SLEEP 0x00
#define Gimbal_R_DEBUG 0x03
#define Gimbal_R_STOP 0x04
#define Gimbal_R_AIM 0x05//∏®√È≤‚ ‘

#define Gimbal_L_RC 0x01
#define Gimbal_L_PC 0x02
#define Gimbal_L_SLEEP 0x00
#define Gimbal_L_DEBUG 0x03
#define Gimbal_L_STOP 0x04
#define Gimbal_L_AIM 0x05

#define Shoot_L_RC 0x04
#define Shoot_L_PC 0x08
#define Shoot_L_SLEEP 0x00
#define Shoot_L_DEBUG 0x09

typedef struct
{
		uint8_t Shoot_Debug;
		uint8_t Gimbal_R_Debug;
		uint8_t Gimbal_L_Debug;
}Debug_Mode_Type;
 

void task_ActionUpdate(void);
void Rst_Debug(void);
void Remote_Disconnect_Act(void);

extern Debug_Mode_Type Debug_Mode;

#endif //__TASK_ACTIONUPDATE_H
