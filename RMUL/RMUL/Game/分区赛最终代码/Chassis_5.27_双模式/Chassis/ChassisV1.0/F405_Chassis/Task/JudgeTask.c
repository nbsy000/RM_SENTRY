#include "main.h"

extern TaskHandle_t JudgeReceiveTask_Handler; //任务句柄
extern unsigned char JudgeReceiveBuffer[JudgeBufBiggestSize];
extern uint8_t JudgeReveice_Flag;
map_sentry_data_t map_sentry_data;

/**********************************************************************************************************
*函 数 名: JudgeReceive_task
*功能说明: 数据接收任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void JudgeReceive_task()
{
		while(1)
		{
			static int count = 0;
			if(JudgeReveice_Flag)
			{
					JudgeBuffReceive(JudgeReceiveBuffer,0);
					JudgeReveice_Flag = 0;
			}
			
			//枪口热量控制的部分
			HeatUpdate_0();   //计算枪口热量值，这个函数的输出值，会通过CAN发送到上下云台板上去
			HeatUpdate_2();   //17mm枪管的ID号按偶数增长的
			
			if(count%1000 == 0)//1Hz
			{
//					Send_Path_Judge();	
			}
			
			vTaskDelay(1);
		}
}


/**
	* @brief  发送307给无人机显示路径
  * @param  None
  * @retval None
  */
void Send_Path_Judge(void)
{
		uint8_t Path_num = 1;
	
		map_sentry_data.intention = 3;
	
		if(chassis.PC_State == TOPATH1)//去前哨战
			Path_num = 1;
		else if(chassis.PC_State == BACKPATH1)//去前哨战
			Path_num = 2;
		
		map_sentry_data.start_position_x = PathInforms[Path_num].PathDotsInfoArray[0].x/100;
		map_sentry_data.start_position_y = PathInforms[Path_num].PathDotsInfoArray[0].y/100;
		
		for(int i=0;i<49;i++)
		{
				map_sentry_data.delta_x[i] = (PathInforms[Path_num].PathDotsInfoArray[(i+1)*10].x - PathInforms[Path_num].PathDotsInfoArray[i*10].x)/100;
				map_sentry_data.delta_y[i] = (PathInforms[Path_num].PathDotsInfoArray[(i+1)*10].y - PathInforms[Path_num].PathDotsInfoArray[i*10].y)/100;
		}
		
		referee_data_pack_handle(0xA5,0x0307,(uint8_t *)&map_sentry_data,sizeof(map_sentry_data));
}

/**********************************************************************************************************
*函 数 名: referee_data_pack_handle
*功能说明: 裁判系统图形数据打包发送
*形    参: uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len
*返 回 值: 无
**********************************************************************************************************/
uint8_t seq = 0;	
extern uint8_t JudgeSend[];
void referee_data_pack_handle(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
	uint16_t frame_length = frameheader_len + cmd_len + len + crc_len;   //数据帧长度	

	memset(JudgeSend,0,frame_length);  //存储数据的数组清零
	
	/*****帧头打包*****/
	JudgeSend[0] = sof;//数据帧起始字节
	memcpy(&JudgeSend[1],(uint8_t*)&len, sizeof(len));//数据帧中data的长度
	JudgeSend[3] = seq;//包序号
	Append_CRC8_Check_Sum(JudgeSend,frameheader_len);  //帧头校验CRC8

	/*****命令码打包*****/
	memcpy(&JudgeSend[frameheader_len],(uint8_t*)&cmd_id, cmd_len);
	
	/*****数据打包*****/
	memcpy(&JudgeSend[frameheader_len+cmd_len], p_data, len);
	Append_CRC16_Check_Sum(JudgeSend,frame_length);  //一帧数据校验CRC16

	if (seq == 0xff) seq=0;
  else seq++;
	
	/*****数据上传*****/
  while(DMA_GetCmdStatus(DMA1_Stream4)==ENABLE);
	DMA1_Stream4->NDTR = frame_length;
	DMA_Cmd(DMA1_Stream4, ENABLE);
	
}
