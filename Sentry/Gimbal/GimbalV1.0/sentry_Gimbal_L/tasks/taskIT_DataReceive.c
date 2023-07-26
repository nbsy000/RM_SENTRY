#include "main.h"
#include "taskIT_DataReceive.h"

extern CanRxMsg Can1_rx_message_0, Can1_rx_message_1, Can2_rx_message_0, Can2_rx_message_1;
extern RC_Ctl_t RC_Ctl;
extern Gimbal_Typedef Gimbal_L;
gyro_Typedef Gyro_Right, Gyro_Left, Gyro_ChassisYaw; // 自己的陀螺仪，左边云台陀螺仪，底盘大Yaw轴陀螺仪
Frame_Rate_t FrameRate;

short aim_Up_Pitch_100;
int aim_Up_Yaw_100;
float aim_Up_Pitch; // 下云台发来的Pitch协同值
float aim_Up_Yaw;	// 下云台发来的Yaw协同值
uint8_t aim_Valid_Flag;
int Bullet_Speed, Bullet_Speed_R;
uint8_t Attack_color;
extern uint8_t Heat_ShootAbleFlag_0;

extern uint8_t Heat_ShootAbleFlag;
extern uint8_t is_game_start;
extern int16_t chassis_speed;
/**
 * @brief  待扩展FIFO
 * @param  None
 * @retval None
 */
void CAN1_DataReceive_0(void)
{

	extern _2006_motor_t FrictionMotor[2];
	extern _2006_motor_t BodanMotor;
	extern block_disconnect_t block_disconnect;

	if (Can1_rx_message_0.StdId == FrictionMotor_Up_0_ID)
	{
		FrictionMotor[0].Angle_ABS = Can1_rx_message_0.Data[0] << 8 | Can1_rx_message_0.Data[1];
		FrictionMotor[0].RealSpeed = Can1_rx_message_0.Data[2] << 8 | Can1_rx_message_0.Data[3];
		FrameRate.firc0F++;
	}
	else if (Can1_rx_message_0.StdId == FrictionMotor_Up_1_ID)
	{
		FrictionMotor[1].Angle_ABS = Can1_rx_message_0.Data[0] << 8 | Can1_rx_message_0.Data[1];
		FrictionMotor[1].RealSpeed = Can1_rx_message_0.Data[2] << 8 | Can1_rx_message_0.Data[3];
		FrameRate.firc1F++;
	}
	else if (Can1_rx_message_0.StdId == BodanMotor_Up_ID)
	{
		BodanMotor.Angle_ABS = Can1_rx_message_0.Data[0] << 8 | Can1_rx_message_0.Data[1];
		BodanMotor.RealSpeed = Can1_rx_message_0.Data[2] << 8 | Can1_rx_message_0.Data[3];
		FrameRate.BodanF++;
	}
}

/**
 * @brief  大Yaw轴数据接收和底盘数据
 * @param  None
 * @retval None
 */
uint32_t chassis_offline_tick = 0; // 底盘掉电检测计数器，计数到20000后认为底盘掉电
uint8_t chassis_offline_flag = 0;  // 底盘掉电标志位，若掉电则置1
uint8_t aim_flag;
uint8_t nav_state;

gyro_Typedef Gyro_ChassisYaw;
void CAN1_DataReceive_1(void) // 接收两轴电机数据
{
	if (Can1_rx_message_1.StdId == SHOOTING_HEAT_ID)
	{
		Heat_ShootAbleFlag_0 = Can1_rx_message_1.Data[0]; // 0号
		Heat_ShootAbleFlag = Can1_rx_message_1.Data[1];	  // 2号
		is_game_start = Can1_rx_message_1.Data[2];
		aim_flag = Can1_rx_message_1.Data[3]; // 瞄准下底盘停止信号
		nav_state = Can1_rx_message_1.Data[5];//导航状态
		Attack_color = (uint8_t)Can1_rx_message_1.Data[6];

		Robo_Disconnect.HeatDiscount = 0;
		FrameRate.heatF++;
		Judge_Msg_Send();
	}
	// else if (Can1_rx_message_1.StdId == BULLET_SPEED_ID)
	// {
	// 	Bullet_Speed_R = (short)(Can1_rx_message_1.Data[0] << 8 | Can1_rx_message_1.Data[1]);
	// 	Bullet_Speed = (short)(Can1_rx_message_1.Data[2] << 8 | Can1_rx_message_1.Data[3]);
	// 	chassis_offline_tick = 0; // 掉电计数清零
	// 	FrameRate.bulletSpeedF++;
	// }

	else if (Can1_rx_message_1.StdId == 0x101)
	{
		memcpy(&Gyro_ChassisYaw.YAW_ABS, &Can1_rx_message_1.Data[0], 4);
		memcpy(&Gyro_ChassisYaw.GZ, &Can1_rx_message_1.Data[4], 4);
	}
}

/**
 * @brief  上云台两轴电机信息、拨弹电机信息接收
 * @param  None
 * @retval None
 */
extern uint8_t armor_state;
void CAN2_DataReceive_0(void)
{

	extern gimbal_motor_t MotoPitch_L;
	extern gimbal_motor_t MotoYaw_L;

	if (Can2_rx_message_0.StdId == Remote_Control_ID) // 接收从上云台传来的遥控器数据帧
	{

		RC_Ctl.rc.s1 = ((int8_t)Can2_rx_message_0.Data[5] >> 2) & 0x03;
		RC_Ctl.rc.s2 = ((int8_t)Can2_rx_message_0.Data[5]) & 0x03;
		if (RC_Ctl.rc.s2 == 0 || RC_Ctl.rc.s1 == 0)
			return; // 如果传过来空的包就直接丢掉

		RC_Ctl.rc.ch0 = ((int16_t)Can2_rx_message_0.Data[0] << 3 | (int16_t)Can2_rx_message_0.Data[1] >> 5) & 0x7ff;
		RC_Ctl.rc.ch1 = ((int16_t)Can2_rx_message_0.Data[1] << 6 | (int16_t)Can2_rx_message_0.Data[2] >> 2) & 0x7ff;
		RC_Ctl.rc.ch2 = ((int16_t)Can2_rx_message_0.Data[2] << 9 | (int16_t)Can2_rx_message_0.Data[3] << 1 | (int16_t)Can2_rx_message_0.Data[4] >> 7) & 0x7ff;
		RC_Ctl.rc.ch3 = ((int16_t)Can2_rx_message_0.Data[4] << 4 | (int16_t)Can2_rx_message_0.Data[5] >> 4) & 0x7ff;
		armor_state = Can2_rx_message_0.Data[6];
		Remote_Can1Send(); // 转发遥控器的指令
		FrameRate.RemoteF++;
		Robo_Disconnect.RemoteDisconnect = 0;
	}

	else if (Can2_rx_message_0.StdId == MotoPitch_ID_L)
	{
		MotoPitch_L.Angle_ABS = Can2_rx_message_0.Data[0] << 8 | Can2_rx_message_0.Data[1];
		MotoPitch_L.real_speed = Can2_rx_message_0.Data[2] << 8 | Can2_rx_message_0.Data[3];
		MotoPitch_L.real_flow = Can2_rx_message_0.Data[4] << 8 | Can2_rx_message_0.Data[5];
		FrameRate.PitchLF++;
	}
	else if (Can2_rx_message_0.StdId == MotoYaw_ID_L)
	{
		MotoYaw_L.Angle_ABS = Can2_rx_message_0.Data[0] << 8 | Can2_rx_message_0.Data[1];
		MotoYaw_L.real_speed = Can2_rx_message_0.Data[2] << 8 | Can2_rx_message_0.Data[3];
		MotoYaw_L.real_flow = Can2_rx_message_0.Data[4] << 8 | Can2_rx_message_0.Data[5];
		FrameRate.YawLF++;
	}
}

/**
 * @brief  上云台陀螺仪信息接收
 * @param  None
 * @retval None
 */
void CAN2_DataReceive_1(void) // 接收陀螺仪
{
#ifdef NEW_INS
	if (Can2_rx_message_1.StdId == 0x100)
	{
		memcpy(&IMUReceive.Gyro[0], Can2_rx_message_1.Data, 2);
		memcpy(&IMUReceive.Gyro[1], &Can2_rx_message_1.Data[2], 2);
		memcpy(&IMUReceive.Gyro[2], &Can2_rx_message_1.Data[4], 2);
		FrameRate.GyroPitchLF++;
	}
	else if (Can2_rx_message_1.StdId == 0x101)
	{
		memcpy(&IMUReceive.Acc[0], Can2_rx_message_1.Data, 2);
		memcpy(&IMUReceive.Acc[1], &Can2_rx_message_1.Data[2], 2);
		memcpy(&IMUReceive.Acc[2], &Can2_rx_message_1.Data[4], 2);
		FrameRate.GyroYawLF++;
	}

#else
	if (Can2_rx_message_1.StdId == Gyro_Pitch_ID)
	{
		memcpy(&Gyro_Left.PITCH, &Can2_rx_message_1.Data[0], 4);
		memcpy(&Gyro_Left.GY, &Can2_rx_message_1.Data[4], 4);
		FrameRate.GyroPitchLF++;
	}
	else if (Can2_rx_message_1.StdId == Gyro_Yaw_ID)
	{
		memcpy(&Gyro_Left.YAW_ABS, &Can2_rx_message_1.Data[0], 4);
		memcpy(&Gyro_Left.GZ, &Can2_rx_message_1.Data[4], 4);
		FrameRate.GyroYawLF++;
	}
#endif
}

/**
 * @brief  云台信息接收
 * @param  None
 * @retval None
 */
float ChassisYaw_Inc = 0.0f;
void Gimbal_Receive(uint8_t Buf[])
{
	switch (Buf[1])
	{
	case (uint8_t)(Remote_Control_ID & 0xff):
		RC_Ctl.rc.s1 = ((int8_t)Buf[7] >> 2) & 0x03;
		RC_Ctl.rc.s2 = ((int8_t)Buf[7]) & 0x03;
		if (RC_Ctl.rc.s2 == 0 || RC_Ctl.rc.s1 == 0)
			return; // 如果传过来空的包就直接丢掉

		RC_Ctl.rc.ch0 = ((int16_t)Buf[2] << 3 | (int16_t)Buf[3] >> 5) & 0x7ff;
		RC_Ctl.rc.ch1 = ((int16_t)Buf[3] << 6 | (int16_t)Buf[4] >> 2) & 0x7ff;
		RC_Ctl.rc.ch2 = ((int16_t)Buf[4] << 9 | (int16_t)Buf[5] << 1 | (int16_t)Buf[6] >> 7) & 0x7ff;
		RC_Ctl.rc.ch3 = ((int16_t)Buf[6] << 4 | (int16_t)Buf[7] >> 4) & 0x7ff;
		Remote_Can1Send(); // 转发遥控器的指令
		FrameRate.RemoteF++;
		break;
	case (uint8_t)(Gimbal_SYNE_ID & 0xff):
		armor_state = Buf[2];
		memcpy(&ChassisYaw_Inc, &Buf[3], 4);
		Gimbal_Syne_Send(Buf);
		FrameRate.SyneF++;
		break;
	}
}

/**
 * @brief  雷达信息接收
 * @param  None
 * @retval None
 */
void NAVReceive(uint8_t Buf[])
{
	extern NAV_Recv_t NAV_Recv;
	float x_now, y_now, w_now;
	memcpy(&x_now, &Buf[1], 4);
	memcpy(&y_now, &Buf[5], 4);
	memcpy(&w_now, &Buf[9], 4);
	NAV_Recv.x_now = (short)x_now;
	NAV_Recv.y_now = (short)y_now;
	NAV_Recv.w_now = (short)w_now;
	NAV_Recv.nav_path_state = Buf[13];
	NAV_Msg_Send(); // 接收到导航数据后再进行发送

	FrameRate.NavF++;
}

/**
 * @brief  TX2信息接收
 * @param  None
 * @retval None
 */
volatile long run_time_check;
int pc_yaw;
short pc_pitch;
int pc_yaw_down;
short pc_pitch_down;
uint8_t distance;
short Armor_Aimed;
uint8_t armor_state = 0; // 表示辅瞄是不是有找到目标
uint8_t last_armor_state = ARMOR_NO_AIM;
float aim_yaw, aim_pitch;
float last_aim_yaw, last_aim_pitch;
float aim_yaw_down, aim_pitch_down;
uint32_t Pc_RX_Num = 0;
uint32_t Last_Pc_Rx_Num = 0;
uint32_t Last_Shoot_Rx_Num = 0;
short tx2_last_receive_flag; // 表示有没有数据更新
short tx2_receive_flag;
float aim_yaw_Limit = 10000.0f, aim_pitch_Limit = 4200.0f; // 对应着 aim_yaw在-100到100之间，aim_pitch在-30-30之间
float aim_yaw_Limit_V = 100.0f, aim_pitch_Limit_V = 30.0f;
uint16_t distance_Limit = 100 * 10;
uint8_t CV_Shoot_ABLE = 0; // 判定视觉方面是否能够打子弹
PC_Receive_t PC_Receive_L;

uint8_t target_id;

void PCReceive(uint8_t PCReceivebuffer[])
{

#ifndef NEW_SHOOTAIM
	run_time_check++;
	Pc_RX_Num++;
	pc_pitch = (short)(PCReceivebuffer[2] << 8 | PCReceivebuffer[3]); // 这里不能转为float，不然负数传过来会变为正数
	pc_yaw = (int)(PCReceivebuffer[4] << 24 | PCReceivebuffer[5] << 16 | PCReceivebuffer[6] << 8 | PCReceivebuffer[7] << 0);
	target_id = PCReceivebuffer[8];

	aim_yaw = (float)pc_yaw / 100.0f;
	aim_pitch = -(float)pc_pitch / 100.0f;

	//		if((aim_yaw>1000) || (aim_pitch>100))
	//		{
	//			aim_yaw = 0;
	//			aim_pitch = 0;
	//		}

	//		if(ABS(aim_yaw-last_aim_yaw)>30)
	//			aim_yaw = last_aim_yaw;
	//		if(ABS(aim_pitch-last_aim_pitch)>30)
	//			aim_pitch = last_aim_pitch;

	tx2_receive_flag = (uint8_t)(PCReceivebuffer[1] & 0xff); // 作为更新标志位
	if (tx2_receive_flag == tx2_last_receive_flag)			 // 如果没有数据更新
	{
		Gimbal_L.armor_state = ARMOR_NO_AIM; // 没目标
	}
	else
	{
		Gimbal_L.armor_state = ARMOR_AIMED; // 有目标
	}
	PC_Receive_L.RCPitch = (float)(aim_pitch); // 更新值
	PC_Receive_L.RCYaw = (float)(aim_yaw);

	last_aim_yaw = aim_yaw;
	last_aim_pitch = aim_pitch;

	PC_Receive_L.DisConnect = 0;
	tx2_last_receive_flag = tx2_receive_flag;

#else
	if (PCReceivebuffer[0] == '!' && Verify_CRC16_Check_Sum(PCReceivebuffer, PC_RECVBUF_SIZE))
	{
		// 数据解码
		memcpy(&pc_recv_data, PCReceivebuffer, PC_RECVBUF_SIZE);

		aim_yaw = (float)(pc_recv_data.yaw);
		aim_pitch = -(float)(pc_recv_data.pitch) / 100.0f;

		Gimbal_L.aim_Pitch = aim_pitch;
		Gimbal_L.aim_Yaw = aim_yaw;

		if (pc_recv_data.enemy_id != 0)
			Gimbal_L.armor_state = ARMOR_AIMED; // 有目标
		else
			Gimbal_L.armor_state = ARMOR_NO_AIM; // 没目标

		target_id = pc_recv_data.enemy_id;
	}
#endif
}

/**********************************************************************************************************
 *函 数 名: Frame_Acceptance_Rate
 *功能说明: 计算消息帧的接收率
 *形    参: Rate 执行该函数的时间间隔（ms）
 *返 回 值: 无
 **********************************************************************************************************/
void Frame_Acceptance_Rate(float Rate)
{
	FrameRate.PitchLR = FrameRate.PitchLF / Rate;
	FrameRate.YawLR = FrameRate.YawLF / Rate;
	FrameRate.GyroPitchLR = FrameRate.GyroPitchLF * 5 / Rate;
	FrameRate.GyroYawLR = FrameRate.GyroYawLF * 5 / Rate;
	FrameRate.ChassisYaw_100R = FrameRate.ChassisYaw_100F / Rate;
	FrameRate.ChassisYaw_101R = FrameRate.ChassisYaw_101F / Rate;
	FrameRate.firc0R = FrameRate.firc0F / Rate;
	FrameRate.firc1R = FrameRate.firc1F / Rate;
	FrameRate.BodanR = FrameRate.BodanF / Rate;
	FrameRate.RemoteR = FrameRate.RemoteF * 14 / Rate;
	FrameRate.SyneR = FrameRate.SyneF * 2 / Rate;
	FrameRate.heatR = FrameRate.heatF * 50 / Rate;
	FrameRate.bulletSpeedR = FrameRate.bulletSpeedF / Rate;
	FrameRate.NavR = FrameRate.NavF / Rate;

	FrameRate.PitchLF = 0;
	FrameRate.YawLF = 0;
	FrameRate.GyroPitchLF = 0;
	FrameRate.GyroYawLF = 0;
	FrameRate.ChassisYaw_100F = 0;
	FrameRate.ChassisYaw_101F = 0;
	FrameRate.firc0F = 0;
	FrameRate.firc1F = 0;
	FrameRate.BodanF = 0;
	FrameRate.RemoteF = 0;
	FrameRate.SyneF = 0;
	FrameRate.heatF = 0;
	FrameRate.bulletSpeedF = 0;
	FrameRate.NavF = 0;
}
