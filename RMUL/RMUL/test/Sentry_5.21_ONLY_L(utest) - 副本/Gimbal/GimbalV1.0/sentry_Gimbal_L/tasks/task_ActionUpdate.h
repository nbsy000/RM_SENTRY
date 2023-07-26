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
#define Chassis_SLEEP 0x03
#define Chassis_DEBUG 0x04
#define Chassis_Protect 0x05

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

#define Gimbal_L_RC 0x04
#define Gimbal_L_PC 0x08
#define Gimbal_L_SLEEP 0x00
#define Gimbal_L_DEBUG 0x03

#define Shoot_L_RC 0x04
#define Shoot_L_PC 0x08
#define Shoot_L_SLEEP 0x00
#define Shoot_L_DEBUG 0x09

/***********RC***********/
typedef __packed struct
{
	uint16_t ch0;
	uint16_t ch1;
	uint16_t ch2;
	uint16_t ch3;
	uint8_t s1;
	uint8_t s2;
}Remote;
typedef __packed struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t press_l;
	uint8_t press_r;
}Mouse;
typedef __packed struct
{
		uint16_t w_,s_,a_,d_,q_,e_,r_,f_,g_,z_,x_,c_,v_,b_,shift_,ctrl_;
}Key;

typedef struct
{
	Remote rc;
	Mouse mouse;
	Key key;
    int8_t RCrecvd,RCDisconnectCnt;
	uint8_t *rx_buffer;
}RC_Ctl_t;

typedef struct
{
		uint8_t Shoot_Debug;
}Debug_Mode_Type;
 
void task_ActionUpdate(void);
void reset_remote(void);

void Rst_Debug(void);

extern Debug_Mode_Type Debug_Mode;

#endif //__TASK_ACTIONUPDATE_H
