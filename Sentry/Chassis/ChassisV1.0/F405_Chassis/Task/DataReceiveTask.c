#include "main.h"


/*---------------------------------内部变量---------------------------------*/
unsigned char SaveBuffer[90];
uint8_t Anomalies_tag;
uint8_t Abnormal_Data;

/*----------------------------------结构体----------------------------------*/
RC_Ctl_t RC_Ctl;//遥控器数据
Gyro_Typedef Gyro_Chassis;//底盘陀螺仪数据
Gyro_Typedef Gyro_ChassisYaw;//大Yaw轴数据
JudgeReceive_t JudgeReceive;
ActReceive_t PCReceive;//PC.RC发送的速度
Frame_Rate_t FrameRate;//记录接收帧数

/*---------------------------------外部变量---------------------------------*/
extern Chassis_Motor_t ChassisMotor[4];//底盘4个电机
extern ChassisState_t chassis;//底盘速度
extern roboDisconnect Robot_Disconnect;
extern Motor_9025_t Motor_9025;

/*--------------------------临时变量（debug使用）---------------------------*/

/**********************************************************************************************************
*函 数 名: JudgeBuffReceive
*功能说明: 裁判系统接收函数
*形    参: ReceiveBuffer[]  DataLen
*返 回 值: 无
**********************************************************************************************************/
float Last_chassisPower=0;
char TickCount=0;
uint16_t receivePower;
short Bullet_Speed_1_100;//弹速
short Bullet_Speed_2_100;
uint8_t is_game_start;
uint8_t Attack_color;
uint8_t defend_flag;
uint8_t Attack_color;
extern int8_t Patrol_Cnt;
void JudgeBuffReceive(unsigned char ReceiveBuffer[],uint16_t DataLen)
{
	uint16_t cmd_id;
	short PackPoint;
	memcpy(&SaveBuffer[JudgeBufBiggestSize],&ReceiveBuffer[0],JudgeBufBiggestSize);		//把ReceiveBuffer[0]地址拷贝到SaveBuffer[24], 依次拷贝24个, 把这一次接收的存到数组后方
	for(PackPoint=0;PackPoint<JudgeBufBiggestSize;PackPoint++)		//先处理前半段数据(在上一周期已接收完成)
	{
		if(SaveBuffer[PackPoint]==0xA5) 
		{	
			if((Verify_CRC8_Check_Sum(&SaveBuffer[PackPoint],5)==1))		//frame_header 五位数据校验
			{
				cmd_id=(SaveBuffer[PackPoint+6])&0xff;
				cmd_id=(cmd_id<<8)|SaveBuffer[PackPoint+5];  
				DataLen=SaveBuffer[PackPoint+2]&0xff;
				DataLen=(DataLen<<8)|SaveBuffer[PackPoint+1];
				if ((cmd_id == 0x0001) && (Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint], DataLen + 9))) //对应着5+4+4
				{
					JudgeReceive.game_type = SaveBuffer[PackPoint + 7 + 0] & 0x0F;			  //取出低四位
					JudgeReceive.game_progress = (SaveBuffer[PackPoint + 7 + 0] & 0xF0) >> 4; //取出高四位
					JudgeReceive.remain_time = SaveBuffer[PackPoint + 7 + 1];
         is_game_start = (JudgeReceive.game_type == 0x01 && JudgeReceive.game_progress >=0x04)?1:0;
				}
				if ((cmd_id == 0x0003) && (Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint], DataLen + 9)))
				{
				   JudgeReceive.red_outpost_hp = SaveBuffer[PackPoint + 7 + 12] << 8 | SaveBuffer[PackPoint + 7 + 13];
					 JudgeReceive.blue_outpost_hp = SaveBuffer[PackPoint + 7 + 28] << 8 | SaveBuffer[PackPoint + 7 + 29];
				}
				
				if((cmd_id == 0x101) && (Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint], DataLen + 9)))
				{
				    JudgeReceive.event = (SaveBuffer[PackPoint + 7 + 3] << 24) | (SaveBuffer[PackPoint + 7 + 2] << 16) | (SaveBuffer[PackPoint + 7 + 1] << 8) | (SaveBuffer[PackPoint + 7 + 0]);
					  defend_flag = (uint8_t)((JudgeReceive.event >> 10)==0);
				}				
				//机器人状态数据
				if((cmd_id==0x0201)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9))) 
				{
					memcpy(&JudgeReceive.robot_id,&SaveBuffer[PackPoint+7+0],1);
					Attack_color = (JudgeReceive.robot_id == 7)? (1) : (0); //等于7说明自身是红方,打击蓝色
					memcpy(&JudgeReceive.RobotLevel,&SaveBuffer[PackPoint+7+1],1);
					memcpy(&JudgeReceive.remainHP,&SaveBuffer[PackPoint+7+2],2);
					memcpy(&JudgeReceive.maxHP,&SaveBuffer[PackPoint+7+4],2);
					memcpy(&JudgeReceive.HeatCool17_0,&SaveBuffer[PackPoint+7+6],2);
					memcpy(&JudgeReceive.HeatMax17_0,&SaveBuffer[PackPoint+7+8],2);
					memcpy(&JudgeReceive.BulletSpeedMax17_0,&SaveBuffer[PackPoint+7+10],2);
					memcpy(&JudgeReceive.HeatCool17_2,&SaveBuffer[PackPoint+7+12],2);
					memcpy(&JudgeReceive.HeatMax17_2,&SaveBuffer[PackPoint+7+14],2);
					memcpy(&JudgeReceive.BulletSpeedMax17_2,&SaveBuffer[PackPoint+7+16],2);
					memcpy(&JudgeReceive.MaxPower,&SaveBuffer[PackPoint+7+24],2);
					if(JudgeReceive.MaxPower == 0)
						JudgeReceive.MaxPower = 150;
				}
					
				//实时功率、热量数据  50Hz
				if((cmd_id==0x0202)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9)))
				{
					memcpy(&JudgeReceive.realChassisOutV,&SaveBuffer[PackPoint+7+0],2);
					memcpy(&JudgeReceive.realChassisOutA,&SaveBuffer[PackPoint+7+2],2);
					memcpy(&JudgeReceive.realChassispower,&SaveBuffer[PackPoint+7+4],4);
					memcpy(&JudgeReceive.remainEnergy,&SaveBuffer[PackPoint+7+8],2);
					memcpy(&JudgeReceive.shooterHeat17_0,&SaveBuffer[PackPoint+7+10],2);                              // 2个字节
					memcpy(&JudgeReceive.shooterHeat17_2, &SaveBuffer[PackPoint+7+12], 2);
					ShootingHeat_CAN2Send();    //发送给上下云台的热量数据 
					Last_chassisPower=JudgeReceive.realChassispower;
					FrameRate.heatF++;
				}
				
				//位置信息
				if((cmd_id==0x0203)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9)))
				{
					memcpy(&JudgeReceive.x,&SaveBuffer[PackPoint+7+0],4);
					memcpy(&JudgeReceive.y,&SaveBuffer[PackPoint+7+4],4);
					memcpy(&JudgeReceive.z,&SaveBuffer[PackPoint+7+8],4);
					memcpy(&JudgeReceive.angle,&SaveBuffer[PackPoint+7+12],4);
				}
												
				//实时增益数据
				if((cmd_id==0x0204)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9)))
				{
					//Can2Send2(SaveBuffer[PackPoint+7+0]);
				}
				
				//受击类型数据
				if ((cmd_id == 0x0206) && (Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint], DataLen + 9)))
				{
					JudgeReceive.hurt_armor_id = SaveBuffer[PackPoint+7+0]&0x0F;
					JudgeReceive.hurt_type = (SaveBuffer[PackPoint+7+0]&0xF0) >> 4;
					chassis.armor_hurt_flag[0] = ((JudgeReceive.hurt_armor_id==0)&&(JudgeReceive.hurt_type==0));
					chassis.armor_hurt_flag[1] = ((JudgeReceive.hurt_armor_id==1)&&(JudgeReceive.hurt_type==0));
					chassis.armor_hurt_flag[2] = ((JudgeReceive.hurt_armor_id==2)&&(JudgeReceive.hurt_type==0));
					chassis.armor_hurt_flag[3] = ((JudgeReceive.hurt_armor_id==3)&&(JudgeReceive.hurt_type==0));	
					if(JudgeReceive.hurt_type)//如果不是因为装甲伤害扣血，认为此时发生了异常情况
					{
						Anomalies_tag = 1;//异常标志位置1
						Abnormal_Data = JudgeReceive.hurt_type;//伤害类型
					}			
				}
				
				//实时射击信息 射击后发送
				if((cmd_id==0x0207)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9)))
				{
					extern uint8_t ShootCpltFlag_0,ShootCpltFlag_2;
					memcpy(&JudgeReceive.bulletFreq, &SaveBuffer[PackPoint+7+2],1);
					memcpy(&JudgeReceive.bulletSpeed,&SaveBuffer[PackPoint+7+3],4);
					if ((SaveBuffer[PackPoint + 7 + 0] == 1) && (SaveBuffer[PackPoint + 7 + 1] == 1))//这里是把1号发射机构作为射频
					{
						Bullet_Speed_1_100 = JudgeReceive.bulletSpeed;
						ShootCpltFlag_0 = 1;
					}
					else if((SaveBuffer[PackPoint + 7 + 0] == 1) && (SaveBuffer[PackPoint + 7 + 1] == 2))
					{
						Bullet_Speed_2_100 = JudgeReceive.bulletSpeed;
						ShootCpltFlag_2 = 1;
					}
				}	

				//雷达和云台手消息
				if((cmd_id==0x0303)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9)))
				{	
					memcpy(&JudgeReceive.target_position_x,&SaveBuffer[PackPoint+7+0],4);
					memcpy(&JudgeReceive.target_position_y,&SaveBuffer[PackPoint+7+4],4);
					memcpy(&JudgeReceive.target_position_z,&SaveBuffer[PackPoint+7+8],4);
					memcpy(&JudgeReceive.commd_keyboard,&SaveBuffer[PackPoint+7+12],1);
					memcpy(&JudgeReceive.target_robot_ID,&SaveBuffer[PackPoint+7+13],2);
				}
				
				//雷达和云台手消息
				if((cmd_id==0x0301)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9)))
				{			
							int i = 0;
				}
				
			}
		}
	Robot_Disconnect.JudgeDisconnect =0;
	memcpy(&SaveBuffer[0],&SaveBuffer[JudgeBufBiggestSize],JudgeBufBiggestSize);		//把SaveBuffer[24]地址拷贝到SaveBuffer[0], 依次拷贝24个，把之前存到后面的数据提到前面，准备处理
	}
}


/**********************************************************************************************************
*函 数 名: Can1Receive0
*功能说明: can1接收处理函数0，接收电调传回的速度，电流值
*形    参: rx_message0
*返 回 值: 无
**********************************************************************************************************/
void Can1Receive0(CanRxMsg rx_message0)
{
	switch(rx_message0.StdId)
	{ 
		case 0x201: 
				ChassisMotor[0].Encoder=rx_message0.Data[0]<<8 | rx_message0.Data[1];		//编码器的值
				ChassisMotor[0].RealSpeed=rx_message0.Data[2]<<8 | rx_message0.Data[3];		//转子转速
				ChassisMotor[0].Current=rx_message0.Data[4]<<8 | rx_message0.Data[5];			//实际转矩电流
				Robot_Disconnect.ChassisDisconnect[0]=0;
				FrameRate.ChassisMotorF[0]++;
				break;
		case 0x202:
				ChassisMotor[1].Encoder=rx_message0.Data[0]<<8 | rx_message0.Data[1];		//编码器的值
				ChassisMotor[1].RealSpeed=rx_message0.Data[2]<<8 | rx_message0.Data[3];
				ChassisMotor[1].Current=rx_message0.Data[4]<<8 | rx_message0.Data[5];
				Robot_Disconnect.ChassisDisconnect[1]=0;
				FrameRate.ChassisMotorF[1]++;
				break;
		case 0x203:
				ChassisMotor[2].Encoder=rx_message0.Data[0]<<8 | rx_message0.Data[1];		//编码器的值
				ChassisMotor[2].RealSpeed=rx_message0.Data[2]<<8 | rx_message0.Data[3];
				ChassisMotor[2].Current=rx_message0.Data[4]<<8 | rx_message0.Data[5];
				Robot_Disconnect.ChassisDisconnect[2]=0;
				FrameRate.ChassisMotorF[2]++;
				break;
		case 0x204:
				ChassisMotor[3].Encoder=rx_message0.Data[0]<<8 | rx_message0.Data[1];		//编码器的值
				ChassisMotor[3].RealSpeed=rx_message0.Data[2]<<8 | rx_message0.Data[3];
				ChassisMotor[3].Current=rx_message0.Data[4]<<8 | rx_message0.Data[5];
				Robot_Disconnect.ChassisDisconnect[3]=0;
				FrameRate.ChassisMotorF[3]++;
				break; 
	}
}

/**********************************************************************************************************
*函 数 名: Can1Receive1
*功能说明: can1接收处理函数1 接收底盘陀螺仪数据和大Yaw轴数据
*形    参: rx_message
*返 回 值: 无
**********************************************************************************************************/
void Can1Receive1(CanRxMsg rx_message1)
{
	switch(rx_message1.StdId)
	{ 
		 case Gyro_Pitch_ID:
			 memcpy(&Gyro_Chassis.PITCH, &rx_message1.Data[0], 4);
			 memcpy(&Gyro_Chassis.GY, &rx_message1.Data[4], 4);
			 break;
		 
		 case Gyro_Yaw_ID:
			 memcpy(&Gyro_Chassis.YAW_ABS, &rx_message1.Data[0], 4);
			 memcpy(&Gyro_Chassis.GZ, &rx_message1.Data[4], 4);
		 
		 	 Robot_Disconnect.ChassisGyroDisconnect = 0; 
		   FrameRate.ChassisGyroF++;
			 break;
		 
		 case RMD_L_ID:
			 Chassis_Yaw_Receive(&rx_message1.Data[0]);
			 Robot_Disconnect.ChassisYawconnect = 0; 
		   FrameRate.YawMotorF++;
			 break;
	 }		
}
	

/**********************************************************************************************************
*函 数 名: Can2Receive0
*功能说明: can2接收函数，接收遥控器和大Yaw轴陀螺仪
*形    参: rx_message1
*返 回 值: 无
**********************************************************************************************************/
void Can2Receive0(CanRxMsg rx_message)
{
	switch(rx_message.StdId)
	{
#ifdef NEW_INS
		if (rx_message.StdId == 0x100)
    {
				memcpy(&IMUReceive.Gyro[0], rx_message.Data, 2);
				memcpy(&IMUReceive.Gyro[1], &rx_message.Data[2], 2);
				memcpy(&IMUReceive.Gyro[2], &rx_message.Data[4], 2);
				FrameRate.ChassisYaw_100F ++;
    }
    else if (rx_message.StdId == 0x101)
    {
				memcpy(&IMUReceive.Acc[0], rx_message.Data, 2);
				memcpy(&IMUReceive.Acc[1], &rx_message.Data[2], 2);
				memcpy(&IMUReceive.Acc[2], &rx_message.Data[4], 2);
				FrameRate.ChassisYaw_101F ++;
    }	
#else		
		
		case Gyro_Yaw_ID:
			 memcpy(&Gyro_ChassisYaw.YAW_ABS, &rx_message.Data[0], 4);
			 memcpy(&Gyro_ChassisYaw.GZ, &rx_message.Data[4], 4);
			 Gyro_ChassisYaw.GZ	= Gyro_ChassisYaw.GZ*180.0f/PI;
		
			 Robot_Disconnect.ChassisYawGyroDisconnect = 0; 
			 FrameRate.ChassisYawGyroF++;
			 break;
#endif		
		case Remote_Control_ID://接收从上云台传来的遥控器数据帧
        RC_Ctl.rc.s1 = ((int8_t) rx_message.Data[5]>>2 ) & 0x03;
        RC_Ctl.rc.s2 = ((int8_t) rx_message.Data[5]) & 0x03;
        if(RC_Ctl.rc.s2 == 0 || RC_Ctl.rc.s1 == 0) 
            return;//如果传过来空的包就直接丢掉
        RC_Ctl.rc.ch0=((int16_t)rx_message.Data[0]<<3 | (int16_t)rx_message.Data[1]>>5) & 0x7ff;
        RC_Ctl.rc.ch1=((int16_t)rx_message.Data[1]<<6 | (int16_t)rx_message.Data[2]>>2 ) & 0x7ff;
        RC_Ctl.rc.ch2=((int16_t)rx_message.Data[2]<<9 | (int16_t)rx_message.Data[3]<<1 | (int16_t)rx_message.Data[4]>>7 ) & 0x7ff;
        RC_Ctl.rc.ch3=((int16_t)rx_message.Data[4]<<4 | (int16_t)rx_message.Data[5]>>4 ) & 0x7ff; 
				aim_flag = rx_message.Data[6];
				Robot_Disconnect.RemoteDisconnect = 0; //遥控器掉电检测
				FrameRate.RemoteF++;
				break;
		case NAV_DATA_ID:
				memcpy(&PCReceive,&rx_message.Data[0], 7);
				chassis.NAV_vx = LIMIT_MAX_MIN(PCReceive.now_x,3500,-3500);//mm/s
				chassis.NAV_vy = LIMIT_MAX_MIN(PCReceive.now_y,3500,-3500);
				chassis.NAV_vw = LIMIT_MAX_MIN((PCReceive.now_w/1000.0f)*180.0f/PI,60,-60);
				break;
		case GIMBAL_SYNE_ID:
				Motor_9025.aim_flag = rx_message.Data[0];
				memcpy(&Motor_9025.SYNEYaw,&rx_message.Data[1], 4);
				break;
	}
}


/**********************************************************************************************************
*函 数 名: Can2Receive1
*功能说明: can2接收函数
*形    参: rx_message0
*返 回 值: 无
**********************************************************************************************************/
void Can2Receive1(CanRxMsg rx_message)
{

}


/**********************************************************************************************************
*函 数 名: RC_Rst
*功能说明: 遥控器数据复位
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void RC_Rst(void)
{
		RC_Ctl.rc.ch0 = 1024;
		RC_Ctl.rc.ch1 = 1024;
		RC_Ctl.rc.ch2 = 1024;
		RC_Ctl.rc.ch3 = 1024;
		RC_Ctl.mouse.x = 0;
		RC_Ctl.mouse.y = 0;
		RC_Ctl.mouse.z = 0;
		RC_Ctl.mouse.press_l = 0;                                                
		RC_Ctl.mouse.press_r = 0;
	
		RC_Ctl.key.w = 0;
		RC_Ctl.key.s = 0;                            
		RC_Ctl.key.a = 0;
		RC_Ctl.key.d = 0;
		RC_Ctl.key.q = 0;
		RC_Ctl.key.e = 0;
		RC_Ctl.key.r = 0;
		RC_Ctl.key.f = 0;
		RC_Ctl.key.shift = 0;
		RC_Ctl.key.ctrl = 0;
	
		RC_Ctl.rc.s1 = 2;
		RC_Ctl.rc.s2 = 2;
} 

/**********************************************************************************************************
*函 数 名: Chassis_Rst
*功能说明: 底盘主控板掉线执行函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Chassis_Rst(void)
{
	chassis.carSpeedx = 0;
	chassis.carSpeedy = 0;
	chassis.carSpeedw = 0;
}



/**********************************************************************************************************
*函 数 名: PCReceive
*功能说明: PC数据接收
*形    参: 
*返 回 值: 无
**********************************************************************************************************/
void PC_Receive(unsigned char PCReceivebuffer[])
{
	memcpy(&PCReceive,&PCReceivebuffer[1], 8); 
}

/**********************************************************************************************************
*函 数 名: Frame_Acceptance_Rate
*功能说明: 计算消息帧的接收率
*形    参: Rate 执行该函数的时间间隔（ms）
*返 回 值: 无
**********************************************************************************************************/
void Frame_Acceptance_Rate(float Rate)
{
	//计算接收率
	for(int i=0;i<4;i++)
		FrameRate.ChassisMotorR[i] = FrameRate.ChassisMotorF[i]/Rate;
	FrameRate.ChassisGyroR = FrameRate.ChassisGyroF*5/Rate;//4ms发一帧
	FrameRate.YawMotorR = FrameRate.YawMotorF/Rate;
	FrameRate.RemoteR = FrameRate.RemoteF*14/Rate;//14ms发一帧
	FrameRate.heatR = FrameRate.heatF*50/Rate;
	
#ifdef NEW_INS
	FrameRate.ChassisYaw_100R = FrameRate.ChassisYaw_100F/Rate;
	FrameRate.ChassisYaw_101R = FrameRate.ChassisYaw_101F/Rate;	
#else
	FrameRate.ChassisYawGyroR = FrameRate.ChassisYawGyroF/Rate;
#endif

	
	//初始化
	for(int i=0;i<4;i++)
		FrameRate.ChassisMotorF[i] = 0;
	FrameRate.ChassisGyroF = 0;
	FrameRate.YawMotorF = 0;
	FrameRate.RemoteF = 0;
	FrameRate.heatF = 0;
	
#ifdef NEW_INS		
	FrameRate.ChassisYaw_100F = 0;
	FrameRate.ChassisYaw_101F = 0;
#else
	FrameRate.ChassisYawGyroF = 0;
#endif

}
