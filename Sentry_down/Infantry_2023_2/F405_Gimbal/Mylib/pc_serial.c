/**
 ******************************************************************************
 * @file    pc_uart.c
 * @brief   serial数据接发
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "pc_serial.h"
#include "arm_atan2_f32.h"
#include "debug.h"
#include "main.h"

PCRecvData pc_recv_data;
PCSendData pc_send_data;
float pc_pitch,pc_yaw;
uint8_t PC_Shoot_flag;
uint8_t last_shoot_flag;
uint8_t armor_state; 
extern Disconnect Robot_Disconnect;

void PCSolve(void)
{
    LossUpdate(&global_debugger.pc_receive_debugger, 0.02);
}

void PCReceive(unsigned char *PCbuffer)
{
    if (PCbuffer[0] == '!' )//&& Verify_CRC16_Check_Sum(PCbuffer, PC_RECVBUF_SIZE))
    {
        //数据解码
        memcpy(&pc_recv_data, PCbuffer, PC_RECVBUF_SIZE);
        /*目标位置数据处理*/
        pc_pitch = pc_recv_data.pitch/100.0f;
        pc_yaw = pc_recv_data.yaw;
        /*射击标志位*/
//        PC_Shoot_flag = pc_recv_data.shoot_flag - last_shoot_flag;
//        last_shoot_flag = pc_recv_data.shoot_flag;
				if(pc_recv_data.enemy_id != 0)
					armor_state = ARMOR_AIMED; //有目标
				else
					armor_state = ARMOR_NO_AIM; //没目标
        PCSolve();
				
				Robot_Disconnect.PC_DisConnect = 0;
    }
}

/**
 * @brief 在这里写发送数据的封装
 * @param[in] void
 */
extern F105_Typedef F105;
extern Gimbal_Typedef Gimbal;
extern Status_t Status;
void SendtoPCPack(unsigned char *buff)
{
    pc_send_data.start_flag = '!';
    pc_send_data.pitch = (short)(Gimbal.Pitch.Gyro * 100.0f);
    pc_send_data.yaw = Gimbal.Yaw.Gyro;

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

    DMA_Cmd(PC_UART_SEND_DMAx_Streamx, ENABLE);
}

/********************************NAV导航雷达数据接发收函数***********************************/

/**
  * @brief  雷达信息接收
  * @param  None
  * @retval None
  */
int LimitSpeed = 1500;
int LimitRotate = 120;
void NAVReceive(uint8_t Buf[])
{
		//数据解码
		memcpy(&NAV_Recv, Buf, NAV_RECVBUF_SIZE);	
	
		//数据引入
		NAV_car.NAV_x = -LIMIT_MAX_MIN(NAV_Recv.y_now,LimitSpeed,-LimitSpeed);
		NAV_car.NAV_y = LIMIT_MAX_MIN(NAV_Recv.x_now,LimitSpeed,-LimitSpeed);
		NAV_car.NAV_w = LIMIT_MAX_MIN((NAV_Recv.w_now/1000.0f)*180.0f/PI,LimitRotate,-LimitRotate);
		NAV_car.NAV_Path_State = NAV_Recv.nav_path_state;
	
		Robot_Disconnect.NAV_DisConnect = 0;
		
}



/**
  * @brief  发送数据给导航
  * @param  None
  * @retval None
  */
void SendtoNAV(void)
{
		NAV_Send.head = '!';
		NAV_Send.nav_state = NAV_car.NAV_State;
		Append_CRC8_Check_Sum((unsigned char *)&NAV_Send, NAV_SENDBUF_SIZE);
		memcpy(NAV_Send_Buf, (void *)&NAV_Send, NAV_SENDBUF_SIZE);
    DMA_Cmd(DMA1_Stream7, ENABLE);   //在程序里面使能DMA的发送
}