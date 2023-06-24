#ifndef __TASK_BLOCKDISCONNECT_H
#define __TASK_BLOCKDISCONNECT_H

typedef struct
{
	uint32_t BullectCnt;

    //掉电计时
	uint32_t Pitch_Last_Cnt;
	uint32_t Yaw_Last_Cnt;
	uint32_t Chassis_Last_Cnt;
	uint32_t Bodan_Last_Cnt;
	uint32_t Gyro_Last_Cnt;
	uint32_t PC_Last_Cnt;
	uint32_t Judge_Last_Cnt;
	uint32_t FrictionLeft_Last_Cnt;
	uint32_t FrictionRight_Last_Cnt;
	uint32_t Bullect_Last_Cnt;
	uint32_t Drone_Last_Cnt;
    //掉电计时
	uint32_t Pitch_Disconnect_Cnt;
	uint32_t Yaw_Disconnect_Cnt;
	uint32_t Chassis_Disconnect_Cnt;
	uint32_t Bodan_Disconnect_Cnt;
	uint32_t Gyro_Disconnect_Cnt;
	uint32_t PC_Disconnect_Cnt;
	uint32_t Judge_Disconnect_Cnt;
	uint32_t FrictionLeft_Disconnect_Cnt;
	uint32_t FrictionRight_Disconnect_Cnt;
	uint32_t NoBullect_Cnt;
	uint32_t Drone_Disconnect_Cnt;


//堵转计时？
	uint32_t Pitch_block;
	uint32_t Yaw_block;
	uint32_t Bodan_block;
	uint32_t Friction_block;


//掉电标志位
	uint8_t is_pitch_down;
	uint8_t is_yaw_down;
	uint8_t is_chassis_down;
	uint8_t is_bodan_down;
	uint8_t is_gyro_down;
	uint8_t is_pc_down;
	uint8_t is_judge_down;
	uint8_t is_friction_down;
	uint8_t is_drone_down;
	uint8_t is_pitch_block;
	uint8_t is_yaw_block;
	uint8_t is_bodan_block;
	uint8_t is_friction_block;

	uint8_t is_left_limit, is_right_limit;

	uint8_t is_no_bullet;
} block_disconnect_t;


void task_BlockDisconnect(void);

#endif //__TASK_BLOCKDISCONNECT_H

