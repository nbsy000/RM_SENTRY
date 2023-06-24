#include "main.h"
#include "taskIT_DataReceive.h"

extern CanRxMsg Can1_rx_message_0, Can1_rx_message_1, Can2_rx_message_0, Can2_rx_message_1;
gyro_Typedef Gyro_Right,Gyro_Left,Gyro_ChassisYaw;//自己的陀螺仪，左边云台陀螺仪，底盘大Yaw轴陀螺仪
PC_Receive_t PC_Receive;
Frame_Rate_t FrameRate;


/**
 * @brief  待扩展FIFO
 * @param  None
 * @retval None
 */
void CAN1_DataReceive_0(void)
{
		extern RC_Ctl_t RC_Ctl;
    extern _2006_motor_t FrictionMotor[2];
    extern _2006_motor_t BodanMotor;
	   //extern gimbal_motor_t MotoPitch;
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
				BodanMotor.RealSpeed = Can1_rx_message_0.Data[2] << 8 | Can1_rx_message_0.Data[3];
				FrameRate.BodanF++;
    }
}

 
/**
  * @brief  上云台两轴电机信息接收
  * @param  None
  * @retval None    
  */

void CAN1_DataReceive_1(void) //接收两轴电机数据
{
#ifdef NEW_INS
		if (Can1_rx_message_1.StdId == Gyro_R_Pitch_ID)
    {
				memcpy(&IMUReceive.Gyro[0], Can1_rx_message_1.Data, 2);
				memcpy(&IMUReceive.Gyro[1], &Can1_rx_message_1.Data[2], 2);
				memcpy(&IMUReceive.Gyro[2], &Can1_rx_message_1.Data[4], 2);
				FrameRate.GyroPitchRF++;
				global_debugger.imu_debugger[0].recv_msgs_num[1]++;
    }
    else if (Can1_rx_message_1.StdId == Gyro_R_Yaw_ID)
    {
				memcpy(&IMUReceive.Acc[0], Can1_rx_message_1.Data, 2);
				memcpy(&IMUReceive.Acc[1], &Can1_rx_message_1.Data[2], 2);
				memcpy(&IMUReceive.Acc[2], &Can1_rx_message_1.Data[4], 2);
				FrameRate.GyroYawRF++;
				global_debugger.imu_debugger[0].recv_msgs_num[1]++;
    }
#endif
 
}

/**
  * @brief  云台两轴电机信息
  * @param  None
  * @retval None
  */
uint16_t RX_Number;
short aim_Up_Pitch_100;
int aim_Up_Yaw_100;
float aim_Up_Pitch;//下云台发来的Pitch协同值
float aim_Up_Yaw;//下云台发来的Yaw协同值
uint8_t aim_Valid_Flag;
int Bullet_Speed;
uint8_t Attack_color;

uint8_t aim_flag;
void CAN2_DataReceive_0(void)
{
	
	  extern gimbal_motor_t MotoPitch;
    extern gimbal_motor_t MotoYaw;
	  extern gimbal_motor_t MotoPitch_L;
    extern gimbal_motor_t MotoYaw_L;
	
	  if (Can2_rx_message_0.StdId == MotoPitch_ID_R)
    {
        MotoPitch.Angle_ABS = Can2_rx_message_0.Data[0] << 8 | Can2_rx_message_0.Data[1];
        MotoPitch.real_speed = Can2_rx_message_0.Data[2] << 8 | Can2_rx_message_0.Data[3];
        MotoPitch.real_flow = Can2_rx_message_0.Data[4] << 8 | Can2_rx_message_0.Data[5]; 
				FrameRate.PitchRF++;
    }
    else if(Can2_rx_message_0.StdId == MotoYaw_ID_R)
    {
        MotoYaw.Angle_ABS = Can2_rx_message_0.Data[0] << 8 | Can2_rx_message_0.Data[1];
        MotoYaw.real_speed = Can2_rx_message_0.Data[2] << 8 | Can2_rx_message_0.Data[3];
        MotoYaw.real_flow = Can2_rx_message_0.Data[4] << 8 | Can2_rx_message_0.Data[5];
				FrameRate.YawRF++;
    }
		else if(Can2_rx_message_0.StdId == MotoPitch_ID_L)
    {
        MotoPitch_L.Angle_ABS = Can2_rx_message_0.Data[0] << 8 | Can2_rx_message_0.Data[1];
        MotoPitch_L.real_speed = Can2_rx_message_0.Data[2] << 8 | Can2_rx_message_0.Data[3];
        MotoPitch_L.real_flow = Can2_rx_message_0.Data[4] << 8 | Can2_rx_message_0.Data[5]; 
				FrameRate.PitchLF++;
    }
    else if(Can2_rx_message_0.StdId == MotoYaw_ID_L)
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

uint32_t gyro_cnt;
float gyro_pitch_test;

uint32_t cnt_temp, cnt1; //float cnt2,cnt2temp;
float gz10e6;            //?????
extern uint8_t Heat_ShootAbleFlag;
extern uint8_t is_game_start;
float aim_yaw_l,aim_pitch_l;
extern Gimbal_Typedef Gimbal_R,Gimbal_L;
void CAN2_DataReceive_1(void) //接收陀螺仪
{
//#ifdef NEW_INS
//		if (Can2_rx_message_1.StdId == Gyro_R_Pitch_ID)
//    {
//				memcpy(&IMUReceive.Gyro[0], Can2_rx_message_1.Data, 2);
//				memcpy(&IMUReceive.Gyro[1], &Can2_rx_message_1.Data[2], 2);
//				memcpy(&IMUReceive.Gyro[2], &Can2_rx_message_1.Data[4], 2);
//				FrameRate.GyroPitchRF++;
//				global_debugger.imu_debugger[0].recv_msgs_num[1]++;
//    }
//    else if (Can2_rx_message_1.StdId == Gyro_R_Yaw_ID)
//    {
//				memcpy(&IMUReceive.Acc[0], Can2_rx_message_1.Data, 2);
//				memcpy(&IMUReceive.Acc[1], &Can2_rx_message_1.Data[2], 2);
//				memcpy(&IMUReceive.Acc[2], &Can2_rx_message_1.Data[4], 2);
//				FrameRate.GyroYawRF++;
//				global_debugger.imu_debugger[0].recv_msgs_num[1]++;
//    }

//#else	
		if (Can2_rx_message_1.StdId == Gyro_R_Pitch_ID)
    {
        memcpy(&Gyro_Right.PITCH, &Can2_rx_message_1.Data[0], 4);
        memcpy(&Gyro_Right.GY, &Can2_rx_message_1.Data[4], 4);
				FrameRate.GyroPitchRF++;
    }
    else if (Can2_rx_message_1.StdId == Gyro_R_Yaw_ID)
    {
        memcpy(&Gyro_Right.YAW_ABS, &Can2_rx_message_1.Data[0], 4);
        memcpy(&Gyro_Right.GZ, &Can2_rx_message_1.Data[4], 4);
				FrameRate.GyroYawRF++;
				
    }
//#endif				
		else if (Can2_rx_message_1.StdId == Gyro_L_Pitch_ID)
    {
        memcpy(&Gyro_Left.PITCH, &Can2_rx_message_1.Data[0], 4);
        memcpy(&Gyro_Left.GY, &Can2_rx_message_1.Data[4], 4);
				FrameRate.GyroPitchLF++;
    }
    else if (Can2_rx_message_1.StdId == Gyro_L_Yaw_ID)
    {
        memcpy(&Gyro_Left.YAW_ABS, &Can2_rx_message_1.Data[0], 4);
        memcpy(&Gyro_Left.GZ, &Can2_rx_message_1.Data[4], 4);
				FrameRate.GyroYawLF++;
    }

		if(Can2_rx_message_1.StdId == AIM_LEFT_ID)
		{
				short pitch_l;
				int yaw_l;
				memcpy(&pitch_l, &Can2_rx_message_1.Data[0], 2);
				memcpy(&yaw_l, &Can2_rx_message_1.Data[2], 4);	
				Gimbal_L.armor_state = Can2_rx_message_1.Data[6];
				aim_pitch_l = -pitch_l/100.0f;
				aim_yaw_l = yaw_l/100.0f;
		}
		else if(Can2_rx_message_1.StdId == SHOOTING_HEAT_ID)
		{
				memcpy(&Heat_ShootAbleFlag, &Can2_rx_message_1.Data[0], 1);
				memcpy(&is_game_start, &Can2_rx_message_1.Data[1], 1);			
				memcpy(&Bullet_Speed, &Can2_rx_message_1.Data[2], 2);	
				memcpy(&Attack_color, &Can2_rx_message_1.Data[4], 1);	
				FrameRate.heatF++;
				Robo_Disconnect.HeatDiscount = 0;
		}
}

/**
  * @brief  云台信息接收
  * @param  None
  * @retval None
  */
float inc_pitch = 1;
float inc_yaw = 2;
void Gimbal_Receive(uint8_t Buf[])
{
		switch(Buf[1])
		{
			case (uint8_t)(AIM_LEFT_ID &0xff):
			{
				short pitch_l;
				int yaw_l;
				memcpy(&pitch_l, &Buf[2], 2);
				memcpy(&yaw_l, &Buf[4], 4);	
				Gimbal_L.armor_state = Buf[8];
				Gimbal_L.target_id = Buf[9];
				aim_pitch_l = -pitch_l/100.0f;
				aim_yaw_l = yaw_l/100.0f;					
			}				
				break;
			case (uint8_t)(SHOOTING_HEAT_ID &0xff):
				memcpy(&Heat_ShootAbleFlag, &Buf[2], 1);
				memcpy(&is_game_start, &Buf[3], 1);			
				memcpy(&Bullet_Speed, &Buf[4], 2);	
				memcpy(&Attack_color, &Buf[5], 1);	
				FrameRate.heatF++;
				Robo_Disconnect.HeatDiscount = 0;
				break;
			case (uint8_t)(GIMBAL_GYRO_ID &0xff):
			{
				short pitch_l,pitch_last;
				float yaw_l,yaw_last;
				memcpy(&pitch_l, &Buf[2], 2);
				memcpy(&yaw_l, &Buf[4], 4);	
				Gimbal_L.armor_state = Buf[8];
				aim_pitch_l = -(float)(pitch_l/100.0f+inc_pitch);
				aim_yaw_l = yaw_l+inc_yaw;
				
//				aim_pitch_l = (K_AIM_pitch)*aim_pitch_l + (1-K_AIM_pitch)*pitch_last;
//				aim_yaw_l = (K_AIM_yaw)*aim_yaw_l + (1-K_AIM_yaw)*yaw_last;
				
				pitch_last = aim_pitch_l;
				yaw_last = aim_yaw_l;
				memcpy(&Gyro_ChassisYaw.GZ,&Buf[10],4);
				Gimbal_L.target_id = Buf[14];
				FrameRate.GimbalLGyroF++;
			}
				break;
		}
}

/**
  * @brief  TX2信息接收
  * @param  None
  * @retval None
  */
volatile long run_time_check;
int pc_yaw;
short pc_pitch;
int pc_yaw_l;
short pc_pitch_l;
uint8_t distance,distance_l;
short Armor_Aimed;
uint8_t armor_state = 0; //表示辅瞄是不是有找到目标
uint8_t last_armor_state=ARMOR_NO_AIM;
float aim_yaw, aim_pitch;
float last_aim_yaw,last_aim_pitch;
uint32_t Pc_RX_Num=0;
uint32_t Last_Pc_Rx_Num=0;
uint32_t Last_Shoot_Rx_Num = 0;
//float Last_aim_yaw,Last_aim_pitch;
short tx2_last_receive_flag; //表示有没有数据更新
short tx2_receive_flag;
float aim_yaw_Limit = 10000.0f, aim_pitch_Limit = 4200.0f;//对应着 aim_yaw在-100到100之间，aim_pitch在-30-30之间
float aim_yaw_Limit_V = 100.0f,aim_pitch_Limit_V= 30.0f;
uint16_t distance_Limit = 100 * 10;
uint8_t CV_Shoot_ABLE = 0; //判定视觉方面是否能够打子弹
uint8_t CV_Shoot_ABLE_L = 0; //判定视觉方面是否能够打子弹
PC_Receive_t PC_Receive_R,PC_Receive_L;


extern gimbal_motor_t MotoPitch_L, MotoYaw_L;
#ifdef PC_OLD
void PCReceive(uint8_t PCReceivebuffer[])
{
	
#ifndef NEW_SHOOTAIM 	
    run_time_check++;
	  Pc_RX_Num++;
    pc_pitch = (short)(PCReceivebuffer[2] << 8 | PCReceivebuffer[3]); //这里不能转为float，不然负数传过来会变为正数
    pc_yaw = (int)(PCReceivebuffer[4] << 24 | PCReceivebuffer[5] << 16 | PCReceivebuffer[6] << 8 | PCReceivebuffer[7] << 0);
		Gimbal_R.target_id = PCReceivebuffer[8];

    aim_yaw = (float)(pc_yaw) / 100.0f;
    aim_pitch = -(float)(pc_pitch) / 100.0f;
	
		tx2_receive_flag = (uint8_t)(PCReceivebuffer[1]&0xff);         //作为更新标志位
		if (tx2_receive_flag == tx2_last_receive_flag) //如果没有数据更新
		{
				Gimbal_R.armor_state = ARMOR_NO_AIM; //没目标
		}
		else
		{
				Gimbal_R.armor_state = ARMOR_AIMED; //有目标
		}

    PC_Receive_R.RCPitch = (float)(aim_pitch); //更新值
    PC_Receive_R.RCYaw = (float)(aim_yaw);
		
		last_aim_yaw = aim_yaw;
		last_aim_pitch = aim_pitch;
    PC_Receive_R.DisConnect = 0;
    tx2_last_receive_flag = tx2_receive_flag;
		
		
#else 		
		if (PCReceivebuffer[0] == '!' && Verify_CRC16_Check_Sum(PCReceivebuffer, PC_RECVBUF_SIZE))
    {
        //数据解码
        memcpy(&pc_recv_data, PCReceivebuffer, PC_RECVBUF_SIZE);
			
				aim_yaw = (float)(pc_recv_data.yaw);
				aim_pitch = -(float)(pc_recv_data.pitch) / 100.0f;
			
				if(pc_recv_data.enemy_id != 0)
					Gimbal_R.armor_state = ARMOR_AIMED; //有目标
				else
					Gimbal_R.armor_state = ARMOR_NO_AIM; //没目标
				
				Gimbal_R.target_id = pc_recv_data.enemy_id;
				
    }
#endif
}
#endif

#ifdef PC_ROS
/**********************************************************************************************************
*函 数 名: PCReceive
*功能说明: PC数据接收
*形    参: 
*返 回 值: 无
**********************************************************************************************************/
extern uint8_t PCbuffer[PC_RECVBUF_SIZE];
extern PC_Recv_t PC_Recv;
void PCReceive(unsigned char PCReceivebuffer[]){
	memcpy(&PC_Recv,&PCReceivebuffer, sizeof(PC_Send_t)); 
}
#endif

/**********************************************************************************************************
*函 数 名: Frame_Acceptance_Rate
*功能说明: 计算消息帧的接收率
*形    参: Rate 执行该函数的时间间隔（ms）
*返 回 值: 无
**********************************************************************************************************/
void Frame_Acceptance_Rate(float Rate)
{
	FrameRate.PitchLR = FrameRate.PitchLF/Rate;
	FrameRate.YawLR = FrameRate.YawLF/Rate;
	FrameRate.PitchRR = FrameRate.PitchRF/Rate;
	FrameRate.YawRR = FrameRate.YawRF/Rate;	
	FrameRate.GyroPitchLR = FrameRate.GyroPitchLF/Rate;
	FrameRate.GyroYawLR = FrameRate.GyroYawLF/Rate;
	FrameRate.GyroPitchRR = FrameRate.GyroPitchRF/Rate;
	FrameRate.GyroYawRR = FrameRate.GyroYawRF/Rate;		
	FrameRate.GimbalLGyroR = FrameRate.GimbalLGyroF/Rate;
	FrameRate.firc0R = FrameRate.firc0F*2/Rate;	
	FrameRate.firc1R = FrameRate.firc1F*2/Rate;		
	FrameRate.BodanR = FrameRate.BodanF/Rate;		
	FrameRate.RemoteR = FrameRate.RemoteF*14.0/Rate;	
	FrameRate.heatR = FrameRate.heatF*50/Rate;	
	FrameRate.bulletSpeedR = FrameRate.bulletSpeedF/Rate;		
	
	FrameRate.PitchLF = 0;
	FrameRate.YawLF = 0;
	FrameRate.PitchRF = 0;
	FrameRate.YawRF = 0;	
	FrameRate.GyroPitchLF = 0;
	FrameRate.GyroYawLF = 0;
	FrameRate.GyroPitchRF = 0;
	FrameRate.GyroYawRF = 0;		
	FrameRate.GimbalLGyroF = 0;
	FrameRate.firc0F = 0;	
	FrameRate.firc1F = 0;		
	FrameRate.BodanF = 0;		
	FrameRate.RemoteF = 0;	
	FrameRate.heatF = 0;
	FrameRate.bulletSpeedF = 0;		

}


