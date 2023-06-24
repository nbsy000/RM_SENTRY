#include "main.h"
#include "HW_DBUS.h" 

RC_Ctl_t RC_Ctl;
extern volatile uint8_t sbus_rx_buffer[2][RX_USART3_BUFFER];

/**
  * @brief  遥控器消息解码
  * @param  None
  * @retval None 	
  */ 
void RemoteReceive(volatile uint8_t * const ptr_sbus_rx_buffer) //遥控器接收没问题
{
	extern Frame_Rate_t FrameRate;
    if(ptr_sbus_rx_buffer ==NULL)
        return ;
	RC_Ctl.rc.ch0 = (ptr_sbus_rx_buffer[0]| (ptr_sbus_rx_buffer[1] << 8)) & 0x07ff; //!< Channel 0
	RC_Ctl.rc.ch1 = ((ptr_sbus_rx_buffer[1] >> 3) | (ptr_sbus_rx_buffer[2] << 5)) & 0x07ff; //!< Channel 1
	RC_Ctl.rc.ch2 = ((ptr_sbus_rx_buffer[2] >> 6) | (ptr_sbus_rx_buffer[3] << 2) | (ptr_sbus_rx_buffer[4] << 10)) & 0x07ff;//!< Channel 2
	RC_Ctl.rc.ch3 = ((ptr_sbus_rx_buffer[4] >> 1) | (ptr_sbus_rx_buffer[5] << 7)) & 0x07ff; //!< Channel 3
	RC_Ctl.rc.s1 = ((ptr_sbus_rx_buffer[5] >> 6)& 0x0003); //!< Switch left
	RC_Ctl.rc.s2 = ((ptr_sbus_rx_buffer[5] >> 4)& 0x0003); //!< Switch right

	SendRate.RemoteFlag = 1;          //转发遥控器的指令	
	Remote_Send();
	FrameRate.RemoteF++;
	Robo_Disconnect.RemoteDisconnect = 0;
}

/**
  * @brief  初始化从遥控器传来的参数
  * @param  None
  * @retval None
  */
void reset_remote(void)
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
	
	RC_Ctl.rc.s1 = 2;
	RC_Ctl.rc.s2 = 2;
}

