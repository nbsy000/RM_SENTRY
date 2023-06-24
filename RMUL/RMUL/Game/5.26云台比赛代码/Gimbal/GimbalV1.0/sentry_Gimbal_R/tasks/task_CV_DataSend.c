#include "main.h"
#include "task_CV_DataSend.h"

extern gimbal_motor_t MotoPitch, MotoYaw,MotoPitch_L,MotoYaw_L;
extern gyro_Typedef Gyro_ChassisYaw;

uint8_t SendToPC_Buff[PC_SENDBUF_SIZE];
extern int16_t PC_Sendflag;
int16_t chassis_speed;
float chassis_speed_trans;
short chassis_speed_trans_100;
static void USART2_SendtoPC(void);

typedef union{
	TickType_t xTickCount;
	unsigned char data[4];
}time_use;
time_use sendTOPC_time;


PCRecvData pc_recv_data;
PCSendData pc_send_data;
/**
  * @brief  定期向TX2发送云台姿态的任务
  * @param  None
  * @retval None
  */
uint32_t TX2_high_water;

#ifdef PC_ROS
PC_Send_t PC_Send;
extern uint8_t PCbuffer[PC_RECVBUF_SIZE];
#endif

void task_CV_DataSend(void *pvParameters)
{

    while (1)
    {	

#ifdef NEW_SHOOTAIM			
      SendtoPC();  //定时给PC发送云台的信息
#else 
			USART2_SendtoPC();
#endif

			vTaskDelay(5);    //(1)  //为了和视觉那边进行接发匹配
#if INCLUDE_uxTaskGetStackHighWaterMark
        TX2_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}


/**
  * @brief  编码数据报包发送给TX2
  * @param  None
  * @retval None
  */
extern int SendToTx2BullectCnt;
short pitch_l,pitch_r;
int yaw_l,yaw_r,chassis_yaw;
short down_sampling_rate = 2; //1000为1帧			//4
int PC_TX_num;                //和Pc之间通信的计数器，用于降低通讯帧率
//short send_bullet_speed;
#ifdef PC_OLD
static void USART2_SendtoPC(void)
{

        PC_TX_num = 0;
        SendToPC_Buff[0] = '!';
    
			  extern short Bullet_Speed;
		    extern uint8_t Attack_color;
        pitch_r = -(short)( MotoPitch.actualAngle * 100);
        yaw_r = (int)( MotoYaw.actualAngle * 100);
				pitch_l = (short)( MotoPitch_L.actualAngle * 100);
				yaw_l = (int)( MotoYaw_L.actualAngle * 100);
				chassis_yaw = (int)( Gyro_ChassisYaw.YAW_ABS * 100);  
			  //send_bullet_speed = (short)Bullet_Speed;
			
			  SendToPC_Buff[1] = (unsigned char)(Attack_color);
        SendToPC_Buff[2] = (unsigned char)((pitch_r >> 8) & 0x00FF);
        SendToPC_Buff[3] = (unsigned char)((pitch_r)&0x00FF);

        SendToPC_Buff[4] = (unsigned char)((yaw_r >> 24) & 0x000000FF);
        SendToPC_Buff[5] = (unsigned char)((yaw_r >> 16) & 0x000000FF);
        SendToPC_Buff[6] = (unsigned char)((yaw_r >> 8) & 0x000000FF);
        SendToPC_Buff[7] = (unsigned char)((yaw_r >> 0) & 0x000000FF);
		
			
			  sendTOPC_time.xTickCount = xTaskGetTickCountFromISR();
				SendToPC_Buff[8] = sendTOPC_time.data[3];
				SendToPC_Buff[9] = sendTOPC_time.data[2];
				SendToPC_Buff[10] = sendTOPC_time.data[1];
				SendToPC_Buff[11] = sendTOPC_time.data[0];
//				
				uint16_t sendbulletspeed  = 25;
				SendToPC_Buff[12]= (unsigned char)((sendbulletspeed>> 8) & 0x00FF);
				SendToPC_Buff[13]= (unsigned char)((sendbulletspeed) & 0x00FF);
				
		   // SendToPC_Buff[12] = 0x30;
        SendToPC_Buff[15] = '#';
        Append_CRC8_Check_Sum(SendToPC_Buff, 16);

        DMA_Cmd(DMA1_Stream6, ENABLE);   //在程序里面使能DMA的发送
}


/**
 * @brief 在这里写发送数据的封装
 * @param[in] void
 */

void SendtoPCPack(unsigned char *buff)
{
	  extern uint8_t Attack_color;

    pc_send_data.start_flag = '!';
    pc_send_data.robot_color = Attack_color;
    pc_send_data.shoot_level = 2;//30弹速
		pc_send_data.mode = 0;
    pc_send_data.which_balance = 0;
    pc_send_data.change_priority_flag = 0;
    pc_send_data.frame_id++;
	
    pc_send_data.pitch = -(short)((MotoPitch.actualAngle)* 100.0f);
    pc_send_data.yaw = MotoYaw.actualAngle;

    Append_CRC16_Check_Sum((unsigned char *)&pc_send_data, PC_SENDBUF_SIZE);
    memcpy(buff, (void *)&pc_send_data, PC_SENDBUF_SIZE);
}

/**
 * @brief 发送数据调用
 * @param[in] void
 */
void SendtoPC(void)
{
    SendtoPCPack(SendToPC_Buff);

    DMA_Cmd(DMA1_Stream6, ENABLE);   //在程序里面使能DMA的发送
}



#endif

#ifdef PC_ROS
/**********************************************************************************************************
*函 数 名: USART1_SEND
*功能说明: PC
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
extern uint8_t PC_SendBuf[PC_SENDBUF_SIZE];
int size_q = sizeof(PC_Send_t);
static void USART2_SendtoPC(){
	PC_Send.head = 0xaa;
	PC_Send.tail = 0xbb;
	memcpy(PC_SendBuf,&PC_Send,sizeof(PC_Send_t));
	Append_CRC8_Check_Sum(PC_SendBuf,sizeof(PC_Send_t));//CRC
	DMA_SetCurrDataCounter(DMA1_Stream6,sizeof(PC_Send_t));
	DMA_Cmd(DMA1_Stream6, ENABLE);
}
#endif

