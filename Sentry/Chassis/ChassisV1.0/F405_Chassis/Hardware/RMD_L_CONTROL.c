/**
	*@brief  RMD_L_CONTROL.c RMD_L 9025电机的协议控制
	*@date 2023.3
	*@attention 主要参考RMD的官方CAN协议，没有将全部都控制指令写入，只写了哨兵大YAW轴需要的部分
*/

#include "main.h"


Motor_9025_t Motor_9025;

/*发送指令*/

/*********************************************************************************************************
*函 数 名: Chassis_Yaw_Close
*功能说明: 关闭电机，同时清除电机运行状态和之前接收到的控制指令
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Chassis_Yaw_Close()
{
		CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = RMD_L_ID;
	
		tx_message.Data[0] = MOTOR_CLOSE;
		
		CAN_Transmit(RMD_CAN,&tx_message);
}


/*********************************************************************************************************
*函 数 名: Chassis_Yaw_Stop
*功能说明: 停止电机，但不清理电机之前的控制指令
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Chassis_Yaw_Stop()
{
		CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = RMD_L_ID;
	
		tx_message.Data[0] = MOTOR_STOP;
		
		CAN_Transmit(RMD_CAN,&tx_message);
}


/*********************************************************************************************************
*函 数 名: Chassis_Yaw_Start
*功能说明: 运行电机，恢复停止电机
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Chassis_Yaw_Start()
{
		CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = RMD_L_ID;
	
		tx_message.Data[0] = MOTOR_START;
		
		CAN_Transmit(RMD_CAN,&tx_message);
}

/*********************************************************************************************************
*函 数 名: Chassis_Yaw_State
*功能说明: 电机状态读取,返回值主要有电流、转速、编码器位置
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Get_ChassisYaw_State()
{
		CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = RMD_L_ID;
	
		tx_message.Data[0] = MOTOR_STATE_TISE;
		tx_message.Data[1] = 0;
		tx_message.Data[2] = 0;
		tx_message.Data[3] = 0;
		tx_message.Data[4] = 0;
		tx_message.Data[5] = 0;
		tx_message.Data[6] = 0;
		tx_message.Data[7] = 0;
		
		CAN_Transmit(RMD_CAN,&tx_message);
}

/*********************************************************************************************************
*函 数 名: Get_ChassisYaw_Power
*功能说明: 获取功率值
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Get_ChassisYaw_Power()
{
		CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = RMD_L_ID;
	
		tx_message.Data[0] = READ_POWER;
		
		CAN_Transmit(RMD_CAN,&tx_message);
}

/*********************************************************************************************************
*函 数 名: Get_ChassisYaw_MAngle
*功能说明: 获取多圈角度值，使用该值可以直接省略过零检测的部分,单圈角度值可以直接通过状态读取的编码值获得
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Get_ChassisYaw_MAngle()
{
		CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = RMD_L_ID;
	
		tx_message.Data[0] = READ_MULTI_ANGLE;
		
		CAN_Transmit(RMD_CAN,&tx_message);
}

/*********************************************************************************************************
*函 数 名: Yaw_I_Control
*功能说明: 转矩控制指令，输出电流控制力矩，电流控制值 数值范围 -2000-2000 对应-32A-32A，
			返回帧为电机状态
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Yaw_I_Control(short I)
{
		CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;   

#ifdef BOARDCAST	//广播模式	
    tx_message.StdId = RMD_BCAST;
	
		//进行限幅
	  I = LIMIT_MAX_MIN(I,2000,-2000);
		memcpy(&tx_message.Data[0],&I,2);
#else
    tx_message.StdId = RMD_L_ID;
	
		tx_message.Data[0] = I_CONTROL;
		//进行限幅
	  I = LIMIT_MAX_MIN(I,2000,-2000);
		memcpy(&tx_message.Data[4],&I,2);	
	
#endif	
		CAN_Transmit(RMD_CAN,&tx_message);
}


/*********************************************************************************************************
*函 数 名: Yaw_Speed_Control
*功能说明: 速度控制值 单位0.01dps
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Yaw_Speed_Control(int speed)
{
		CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = RMD_L_ID;
	
		tx_message.Data[0] = SPEED_CONTROL;
		memcpy(&tx_message.Data[4],&speed,4);
	
		CAN_Transmit(RMD_CAN,&tx_message);
}

/*********************************************************************************************************
*函 数 名: Yaw_MAngle_Control
*功能说明: 多圈角度控制值  数值类型int32 单位0.01度
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Yaw_MAngle_Control(int Angle)
{
		CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = RMD_L_ID;
	
		tx_message.Data[0] = MULTI_ANGLE_CONTROL;
		memcpy(&tx_message.Data[4],&Angle,4);
		
		CAN_Transmit(RMD_CAN,&tx_message);
}

/*********************************************************************************************************
*函 数 名: Yaw_SAngle_Control
*功能说明: 发送数值类型 uint16_t 范围0-35999，对应0-359.99
			Data[1]是方向位,0x00是顺时针，0x01是逆时针
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Yaw_SAngle_Control(uint16_t Angle)
{
		uint8_t sign;
		//判断控制角度与当前实际角度相对关系，选择最短角度的方向
		if((Angle-Motor_9025.signalAngle)>17999)
		{
			sign = 0x00;//顺时针
			Angle = 36000-Angle;
		}
		else 
			sign = 0x01;//逆时针
	
		//确保不会出现0-35999之外的数
		Angle = LIMIT_MAX_MIN(Angle,35999,0);
	
		CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = RMD_L_ID;
	
		tx_message.Data[0] = SINGLE_ANGLE_CONTROL;
		tx_message.Data[1] = sign;
		memcpy(&tx_message.Data[4],&Angle,2);
		
		CAN_Transmit(RMD_CAN,&tx_message);	 
}

/*********************************************************************************************************
*函 数 名: Yaw_IAngle_Control
*功能说明: 增量位置控制,int类型，转动方向由符号确定
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Yaw_IAngle_Control(int Angle)
{
		CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    	
    tx_message.StdId = RMD_L_ID;
	
		tx_message.Data[0] = INCREMENT_ANGLE_CONTROL;
		memcpy(&tx_message.Data[4],&Angle,4);	
	
		CAN_Transmit(RMD_CAN,&tx_message);	
}


/* 接收指令 */

/*********************************************************************************************************
*函 数 名: Chassis_Yaw_Receive
*功能说明: 对9025的返回帧进行读取，目前只读取电机运动状态的返回帧
*形    参: 无
*返 回 值: 无
						不使用指令读取多圈角度值，更新频率太慢
**********************************************************************************************************/
float Low_Speed = 0.0f;
uint16_t encoder = 0;
uint16_t signalAngle = 0;
int64_t	multiAngle;
void Chassis_Yaw_Receive(uint8_t* Data)
{
	//读取返回帧类型,以下几种的返回帧内容相同
	if(
//		(Data[0]==MOTOR_STATE_TISE)||
	   Data[0]==I_CONTROL
//	   (Data[0]==SPEED_CONTROL)||
//	   (Data[0]==MULTI_ANGLE_CONTROL)||
//	   (Data[0]==SINGLE_ANGLE_CONTROL)||
//	   (Data[0]==INCREMENT_ANGLE_CONTROL)
	)
	{
		memcpy(&Motor_9025.I,&Data[2],2);
		memcpy(&Motor_9025.speed,&Data[4],2);
		memcpy(&Motor_9025.encoder,&Data[6],2);
		
		memcpy(&encoder,&Data[6],2);
//		Motor_9025.encoder = encoder;
//		Low_Speed = LowPass((float)Motor_9025.speed,Low_Speed,0.02);//低通滤波
		
		//将编码值转化为角度值
		Motor_9025.signalAngle = encoder*0.549316f;	//9025编码器的值感觉没有按照手册上的一样，这里是看贝哥代码里面的
	}	
}

