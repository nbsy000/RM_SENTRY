#include "main.h"
#include "task_DataSend.h"

extern State_t Sentry_State;
extern float Pitch_Actual, Yaw_Actual;
CanTxMsg tx_message;
CanTxMsg tx_message_gimbal;
Send_Rate_t SendRate;
/**
  * @brief  发送拨弹电机的信息，设置拨弹电机的转速
  * @param  给到拨弹电机的电流值
  * @retval None
  */
void Bodan_Can1Send(int16_t bodanVal,int16_t fric0, int16_t fric1)
{
	CanTxMsg tx_message;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	tx_message.StdId = 0x200;

	fric0 = LIMIT_MAX_MIN(fric0, FrictionCurrentLimit, -FrictionCurrentLimit);
	fric1 = LIMIT_MAX_MIN(fric1, FrictionCurrentLimit, -FrictionCurrentLimit);
	bodanVal = LIMIT_MAX_MIN(bodanVal, BodanCurrentLimit, -BodanCurrentLimit);

	tx_message.Data[0] = (uint8_t)((fric0 >> 8) & 0xff);
	tx_message.Data[1] = (uint8_t)(fric0 & 0xff);
	tx_message.Data[2] = (uint8_t)((fric1 >> 8) & 0xff);
	tx_message.Data[3] = (uint8_t)(fric1 & 0xff);
	tx_message.Data[4] = (uint8_t)((bodanVal >> 8) & 0xff);
	tx_message.Data[5] = (uint8_t)(bodanVal & 0xff);
	CAN_Transmit(CAN1, &tx_message);
}



/**
* @brief  云台协同消息发送消息发送	
  * @param  两轴电机电流值
  * @retval None
  */

void Gimbal_Syne_Send(float ChassisYaw_Angle)
{
	extern unsigned char GimbalSend[];
	extern uint8_t armor_flag;
	static uint8_t i = 0;

	GimbalSend[0] = '!';
	GimbalSend[1] = (uint8_t)(Gimbal_SYNE_ID &0xff);
	GimbalSend[2] = armor_flag;
	memcpy(&GimbalSend[3],&ChassisYaw_Angle,4);
				
	Append_CRC8_Check_Sum(GimbalSend,SEND_MAX_SIZE);	
	DMA_Cmd(DMA1_Stream4, ENABLE);   //在程序里面使能DMA的发送
	SendRate.OthersF++;
}



/**
  * @brief  云台电机消息发送	
  * @param  两轴电机电流值
  * @retval None
  */
uint16_t TX_Number=1;
TickType_t Last_Tick,Now_Tick,Interval;

void PitchYaw_Can2Send(int16_t pitchR, int16_t yawR,int16_t pitchL,int16_t yawL)
{
	Last_Tick = Now_Tick;
	Now_Tick = xTaskGetTickCount();
	Interval = Now_Tick - Last_Tick;
	
	tx_message_gimbal.IDE = CAN_ID_STD;
	tx_message_gimbal.RTR = CAN_RTR_DATA;
	tx_message_gimbal.DLC = 0x08;
	tx_message_gimbal.StdId = 0x1FF;

	yawR = LIMIT_MAX_MIN(yawR, LimitYaw, -LimitYaw);
	pitchR = LIMIT_MAX_MIN(pitchR, LimitPitch, -LimitPitch);
	yawL = LIMIT_MAX_MIN(yawL, LimitYaw, -LimitYaw);
	pitchL = LIMIT_MAX_MIN(pitchL, LimitPitch, -LimitPitch);
	
	tx_message_gimbal.Data[0] = (uint8_t)((yawR >> 8) & 0xff);
	tx_message_gimbal.Data[1] = (uint8_t)(yawR & 0xff);
	tx_message_gimbal.Data[2] = (uint8_t)((yawL >> 8) & 0xff);
	tx_message_gimbal.Data[3] = (uint8_t)(yawL & 0xff);
	tx_message_gimbal.Data[4] = (uint8_t)((pitchR >> 8) & 0xff);
	tx_message_gimbal.Data[5] = (uint8_t)(pitchR & 0xff);
	tx_message_gimbal.Data[6] = (uint8_t)((pitchL >> 8) & 0xff);
	tx_message_gimbal.Data[7] = (uint8_t)(pitchL & 0xff);
	
	TX_Number = !TX_Number;
	CAN_Transmit(CAN2, &tx_message_gimbal);
}


/**
  * @brief  给到底盘和下云台主控板的遥控数据(MD直接把遥控器的原始数据给过去吧，压缩一下给过去)
  *         然后在下云台和底盘那里各写各的解码函数
  * @param  None
  * @retval None
  */
CanTxMsg CAN2_tx_message_remote;
CanTxMsg CAN2_tx_down;
extern uint8_t armor_state;   //表示辅瞄是不是有找到目标

void Remote_Send(void)
{
    extern RC_Ctl_t RC_Ctl ;
		if(RC_Ctl.rc.s1 && RC_Ctl.rc.s2)  //如果是零包就丢掉（虽然我还是没整清楚零包是怎么出现的MD） lwm:初始化的时候会出现
    {
			#ifdef CAN_CHOOSE
        CAN2_tx_message_remote.IDE = CAN_ID_STD;
        CAN2_tx_message_remote.RTR = CAN_RTR_DATA;
        CAN2_tx_message_remote.DLC = 0x08;
        CAN2_tx_message_remote.StdId = Remote_Control_ID;

        CAN2_tx_message_remote.Data[0] = (uint8_t)((RC_Ctl.rc.ch0>>3) & 0xff);
        CAN2_tx_message_remote.Data[1] = (uint8_t)((RC_Ctl.rc.ch0<<5)|(RC_Ctl.rc.ch1>>6) & 0xff);
        CAN2_tx_message_remote.Data[2] = (uint8_t)((RC_Ctl.rc.ch1<<2)|(RC_Ctl.rc.ch2>>9) & 0xff);
        CAN2_tx_message_remote.Data[3] = (uint8_t)((RC_Ctl.rc.ch2>>1) & 0xff);
        CAN2_tx_message_remote.Data[4] = (uint8_t)((RC_Ctl.rc.ch2<<7)|(RC_Ctl.rc.ch3>>4) & 0xff);
        CAN2_tx_message_remote.Data[5] = (uint8_t)((RC_Ctl.rc.ch3<<4)|((RC_Ctl.rc.s1<<2) & 0x0C)|((RC_Ctl.rc.s2) & 0x03) & 0xff);
		
        CAN2_tx_message_remote.Data[6] = (uint8_t)armor_state;   
        CAN_Transmit(CAN2, &CAN2_tx_message_remote);
			#else
				extern unsigned char GimbalSend[];
				GimbalSend[0] = '!';
				GimbalSend[1] = (uint8_t)(Remote_Control_ID &0xff);
			
        GimbalSend[2] = (uint8_t)((RC_Ctl.rc.ch0>>3) & 0xff);
        GimbalSend[3] = (uint8_t)(((RC_Ctl.rc.ch0<<5)|(RC_Ctl.rc.ch1>>6)) & 0xff);
        GimbalSend[4] = (uint8_t)(((RC_Ctl.rc.ch1<<2)|(RC_Ctl.rc.ch2>>9)) & 0xff);
        GimbalSend[5] = (uint8_t)((RC_Ctl.rc.ch2>>1) & 0xff);
        GimbalSend[6] = (uint8_t)(((RC_Ctl.rc.ch2<<7)|(RC_Ctl.rc.ch3>>4)) & 0xff);
        GimbalSend[7] = (uint8_t)(((((RC_Ctl.rc.ch3<<4)|(RC_Ctl.rc.s1<<2))& 0x0C)|(RC_Ctl.rc.s2 & 0x03)) & 0xff);
        GimbalSend[8] = (uint8_t)armor_state;
				
				Append_CRC8_Check_Sum(GimbalSend,SEND_MAX_SIZE);	
				DMA_Cmd(DMA1_Stream4, ENABLE); 
			#endif
    }
}

/**
* @brief  Classify_Send_Msg 对发送成功的数据进行分类处理
  * @param  None
  * @retval None
  */
void Classify_Send_Msg(uint8_t Buf[])
{
	switch(Buf[1])
	{
		case(uint8_t)(Remote_Control_ID &0xff):
				SendRate.RemoteF++;
			break;
		case (uint8_t)(Gimbal_SYNE_ID &0xff): 
				SendRate.SyneF++;
			break;
		default:
			SendRate.OthersF ++;
		break;	
	}

}

/**********************************************************************************************************
*函 数 名: Frame_Send_Rate
*功能说明: 计算消息帧的发送率
*形    参: Rate 执行该函数的时间间隔（ms）
*返 回 值: 无
**********************************************************************************************************/
void Frame_Send_Rate(float Rate)
{
	SendRate.RemoteR = SendRate.RemoteF*14/Rate;
	SendRate.SyneR = SendRate.SyneF*2/Rate;	
	SendRate.OthersR = SendRate.OthersF/Rate;
	
	SendRate.RemoteF = 0;
	SendRate.SyneF = 0;
	SendRate.OthersF = 0;
}
