#include "main.h"


/*---------------------------------内部变量---------------------------------*/


/*----------------------------------结构体----------------------------------*/


/*---------------------------------外部变量---------------------------------*/
extern char Judge_Lost;
extern ChassisState_t chassis;

/*--------------------------临时变量（debug使用）---------------------------*/

/****************************************CAN1**************************************************************/

/**********************************************************************************************************
*函 数 名: ChassisMotorSend
*功能说明: 底盘电机电流值发送
*形    参: 四个电机电流值
*返 回 值: 无
**********************************************************************************************************/
void ChassisMotorSend(short a,short b,short c,short d)
{
    CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = 0x200;

    tx_message.Data[0] = (unsigned char)((a>>8)&0xff);
    tx_message.Data[1] = (unsigned char)( a &0xff);  
    tx_message.Data[2] = (unsigned char)((b>>8)&0xff);
    tx_message.Data[3] = (unsigned char)( b &0xff);
    tx_message.Data[4] = (unsigned char)((c>>8)&0xff);
    tx_message.Data[5] = (unsigned char)( c&0xff);
    tx_message.Data[6] = (unsigned char)((d>>8)&0xff);
    tx_message.Data[7] = (unsigned char)( d &0xff);
    CAN_Transmit(CAN1,&tx_message);
}



/****************************************CAN2**************************************************************/


/**
  * @brief  发送到CAN总线上的上下枪口热量
  * @brief  发送底盘接收到的关于比赛的惺惺相惜，防止在比赛正式开始前，哨兵下枪管抽风
  * @param  从裁判系统传过来的枪口热量
  * @retval None
  */
void ShootingHeat_CAN2Send(void)
{
  CanTxMsg tx_message;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = 0x08;
  tx_message.StdId = SHOOTING_HEAT_ID;

	extern uint8_t is_game_start;
	extern uint8_t defend_flag;
	extern uint8_t ShootAbleFlag_0, ShootAbleFlag_2;
	extern uint8_t Attack_color;

  tx_message.Data[0] = ShootAbleFlag_0;//1号发射机构
  tx_message.Data[1] = ShootAbleFlag_2;//2号发射机构
  tx_message.Data[2] = is_game_start;  //把核个标志发送给上下云台，防止在正式开始前，云台打弹
  tx_message.Data[3] = chassis.armor_hurt_flag[0];
  tx_message.Data[4] = defend_flag;
  tx_message.Data[5] = chassis.PC_State;//底盘自动模式状态
  tx_message.Data[6] = Attack_color;//1红0蓝（自己）
  tx_message.Data[7] = chassis.armor_hurt_flag[1];

  CAN_Transmit(CAN2, &tx_message);
}



/**********************************************************************************************************
*函 数 名: BulletSpeed_CAN2Send
*功能说明: 发送弹速，目前不发
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void BulletSpeed_CAN2Send(void)
{
	extern short Bullet_Speed_1_100;
	extern short Bullet_Speed_2_100;
	
  CanTxMsg tx_message;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = 0x08;
  tx_message.StdId = BULLET_SPEED_ID;
	
	Bullet_Speed_1_100 = (short)(JudgeReceive.bulletSpeed * 100);
  tx_message.Data[0] = (uint8_t)( (Bullet_Speed_1_100 >> 8) & 0x00FF);
  tx_message.Data[1] = (uint8_t)( (Bullet_Speed_1_100 >> 0) & 0x00FF);
	tx_message.Data[2] = (uint8_t)( (Bullet_Speed_2_100 >> 8) & 0x00FF);
  tx_message.Data[3] = (uint8_t)( (Bullet_Speed_2_100 >> 0) & 0x00FF);
  tx_message.Data[4] = 0;
  tx_message.Data[5] = 0;
  tx_message.Data[6] = 0;
  tx_message.Data[7] = 0;

  CAN_Transmit(CAN2, &tx_message);
}

/**********************************************************************************************************
*函 数 名: ChassisYaw_Send
*功能说明: 当大Yaw轴使用新陀螺仪代码时，需要使用改函数发送大Yaw轴计算完的数据
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ChassisYaw_Send(void)
{
	extern Gyro_Typedef Gyro_ChassisYaw;//大Yaw轴数据;
	
  CanTxMsg tx_message;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = 0x08;
  tx_message.StdId = CHASSISYAW_ID;

	memcpy(&tx_message.Data[0],&Gyro_ChassisYaw.YAW_ABS,4);
	memcpy(&tx_message.Data[4],&Gyro_ChassisYaw.GZ,4);	

  CAN_Transmit(CAN2, &tx_message);
}

/**********************************************************************************************************
*函 数 名: Uart4Send
*功能说明: 串口4发送函数（测试）
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
extern unsigned char JudgeSend[SEND_MAX_SIZE];
void Uart4Send()
{
	for(int i=0;i<8;i++)
		JudgeSend[i] = 1+i;
	DMA_Cmd(DMA1_Stream4, ENABLE);	
}
