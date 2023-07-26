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
void task_CV_DataSend(void *pvParameters)
{
		static int count = 0;
    while (1)
    {
#ifdef NEW_SHOOTAIM			
      SendtoPC();  //定时给PC发送云台的信息
#else 
			USART2_SendtoPC();
#endif
			
			if(count%10 == 0)//20Hz
				SendtoNAV();//发送导航数据
			
			count++;
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
short down_sampling_rate = 4; //1000为1帧			//4
int PC_TX_num;                //和Pc之间通信的计数器，用于降低通讯帧率
//short send_bullet_speed;
static void USART2_SendtoPC(void)
{
        PC_TX_num = 0;
        SendToPC_Buff[0] = '!';
    
        extern float Pitch_Actual, Yaw_Actual;
			  extern short Bullet_Speed,Bullet_Speed_R;
		    extern uint8_t Attack_color;
        pitch_r = (short)( MotoPitch.actualAngle * 100);
        yaw_r = (int)( MotoYaw.actualAngle * 100);
				pitch_l = -(short)( MotoPitch_L.actualAngle * 100);
				yaw_l = (int)( MotoYaw_L.actualAngle * 100);
				chassis_yaw = (int)( Gyro_ChassisYaw.YAW_ABS * 100);
			
				SendToPC_Buff[1] = (unsigned char)(Attack_color);
        SendToPC_Buff[2] = (unsigned char)((pitch_l >> 8) & 0x00FF);
        SendToPC_Buff[3] = (unsigned char)((pitch_l)&0x00FF);

        SendToPC_Buff[4] = (unsigned char)((yaw_l >> 24) & 0x000000FF);
        SendToPC_Buff[5] = (unsigned char)((yaw_l >> 16) & 0x000000FF);
        SendToPC_Buff[6] = (unsigned char)((yaw_l >> 8) & 0x000000FF);
        SendToPC_Buff[7] = (unsigned char)((yaw_l >> 0) & 0x000000FF);
		
			
			  sendTOPC_time.xTickCount = xTaskGetTickCountFromISR();
				SendToPC_Buff[8] = sendTOPC_time.data[3];
				SendToPC_Buff[9] = sendTOPC_time.data[2];
				SendToPC_Buff[10] = sendTOPC_time.data[1];
				SendToPC_Buff[11] = sendTOPC_time.data[0];
				
				uint16_t sendbulletspeed  = 25;
				SendToPC_Buff[12]= (unsigned char)((sendbulletspeed>> 8) & 0x00FF);
				SendToPC_Buff[13]= (unsigned char)((sendbulletspeed) & 0x00FF);
				
		   // SendToPC_Buff[12] = 0x30;
        SendToPC_Buff[15] = '#';
        Append_CRC8_Check_Sum(SendToPC_Buff, 16);
				
				
        GPIO_SetBits(GPIOA, GPIO_Pin_1);
        DMA_Cmd(DMA1_Stream6, ENABLE);   //在程序里面使能DMA的发送
				
//				Aim_Msg_Send();
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
		pc_send_data.mode = 0;//正常辅
    pc_send_data.which_balance = 0;
    pc_send_data.change_priority_flag = 0;
    pc_send_data.frame_id++;
    pc_send_data.pitch = -(short)(MotoPitch_L.actualAngle * 100.0f);
    pc_send_data.yaw = MotoYaw_L.actualAngle;

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

/**
  * @brief  发送数据给导航
  * @param  None
  * @retval None
  */
void SendtoNAV(void)
{
		extern uint8_t nav_state;
		NAV_Send.head = '!';
		NAV_Send.nav_state = nav_state;
		Append_CRC8_Check_Sum((unsigned char *)&NAV_Send, NAV_SENDBUF_SIZE);
		memcpy(NAV_Send_Buf, (void *)&NAV_Send, NAV_SENDBUF_SIZE);
    DMA_Cmd(DMA1_Stream7, ENABLE);   //在程序里面使能DMA的发送
}