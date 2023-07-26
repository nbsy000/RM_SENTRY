#include "main.h"

//TODO! 这个掉电检测考虑不做成独立的task了
//TODO! 在这个block_disconnect 结构体里面把
//TODO! 需要做掉电检测的的tick和flag放进去，
//TODO! 在那些需要计数的地方，自己加自己的就行了

//extern _2006_motor_t BodanMotor;
//extern block_disconnect_t block_disconnect;

static void disconnect_count(void);//掉电时间计数器
static void disconnect_reset(void);//掉电计数值复位
/**
  * @brief  断线检测
  * @param  None
  * @retval None
  */
void task_BlockDisconnect(void)
{
//  /*这里面是之前裸机时写的掉电检测*/  {
//	int32_t timenow;// = /*这个地方要用FreeRTOS给的API了到时候*///GetSysCnt();

//	if (timenow - block_disconnect.PC_Last_Cnt > 10)		//pc
//		block_disconnect.PC_Disconnect_Cnt++;
//	else
//		block_disconnect.PC_Disconnect_Cnt = 0;
//	if (block_disconnect.PC_Disconnect_Cnt >= 100)
//		block_disconnect.is_pc_down = 1;
//	else
//		block_disconnect.is_pc_down = 0;

//	//	
//	//	if(timenow - block_disconnect.Judge_Last_Cnt>10)	//裁判系统
//	//		block_disconnect.Judge_Disconnect_Cnt++;
//	//	else 
//	//		block_disconnect.Judge_Disconnect_Cnt = 0;
//	//	if(block_disconnect.Judge_Disconnect_Cnt>=200)
//	//		block_disconnect.is_judge_down = 1;
//	//	else 
//	//		block_disconnect.is_judge_down = 0;


//	if (timenow - block_disconnect.Yaw_Last_Cnt > 10)		//yaw离线检测
//		block_disconnect.Yaw_Disconnect_Cnt++;
//	else
//		block_disconnect.Yaw_Disconnect_Cnt = 0;
//	if (block_disconnect.Yaw_Disconnect_Cnt >= 1000)
//		block_disconnect.is_yaw_down = 1;
//	else
//		block_disconnect.is_yaw_down = 0;

//	if (timenow - block_disconnect.Pitch_Last_Cnt > 10)		//pitch离线检测
//		block_disconnect.Pitch_Disconnect_Cnt++;
//	else
//		block_disconnect.Pitch_Disconnect_Cnt = 0;
//	if (block_disconnect.Pitch_Disconnect_Cnt >= 1000)
//		block_disconnect.is_pitch_down = 1;
//	else
//		block_disconnect.is_pitch_down = 0;


//	//	if(timenow - block_disconnect.Bodan_Last_Cnt>10)		//拨弹电机离线检测
//	//		block_disconnect.Bodan_Disconnect_Cnt++;
//	//	else 
//	//		block_disconnect.Bodan_Disconnect_Cnt = 0;					
//	//	if(block_disconnect.Bodan_Disconnect_Cnt>=1000)
//	//		block_disconnect.is_bodan_down = 1;
//	//	else 
//	//		block_disconnect.is_bodan_down = 0;


//	if (timenow - block_disconnect.Gyro_Last_Cnt > 10)		//云台
//		block_disconnect.Gyro_Disconnect_Cnt++;
//	else
//		block_disconnect.Gyro_Disconnect_Cnt = 0;
//	if (block_disconnect.Gyro_Disconnect_Cnt >= 1000)
//		block_disconnect.is_gyro_down = 1;
//	else
//		block_disconnect.is_gyro_down = 0;

//	//	if(block_disconnect.BullectCnt - block_disconnect.Bullect_Last_Cnt>10)		//No bullect
//	//		block_disconnect.NoBullect_Cnt++;
//	//	else 
//	//		block_disconnect.NoBullect_Cnt = 0;					
//	//	if(block_disconnect.NoBullect_Cnt>=2000)
//	//		block_disconnect.is_no_bullet = 1;
//	//	else 
//	//		block_disconnect.is_no_bullet = 0;
//	//	

//	if (timenow - block_disconnect.FrictionLeft_Last_Cnt > 10)		//fritction离线检测
//		block_disconnect.FrictionLeft_Disconnect_Cnt++;
//	else
//		block_disconnect.FrictionLeft_Disconnect_Cnt = 0;

//	if (timenow - block_disconnect.FrictionRight_Last_Cnt > 10)
//		block_disconnect.FrictionRight_Disconnect_Cnt++;
//	else
//		block_disconnect.FrictionRight_Disconnect_Cnt = 0;

//	if (block_disconnect.FrictionLeft_Disconnect_Cnt >= 1000 || block_disconnect.FrictionRight_Disconnect_Cnt >= 1000)
//		block_disconnect.is_friction_down = 1;
//	else
//		block_disconnect.is_friction_down = 0;
//	/************************************************/
////		block_disconnect.is_yaw_down = 0;	                   
////		block_disconnect.is_pitch_down = 0;
////		block_disconnect.is_bodan_down = 0;
////		block_disconnect.is_gyro_down = 0;
////		block_disconnect.is_no_bullet = 0;
////		block_disconnect.is_friction_down = 0;
//	/***********************************************/
////	if(timenow - block_disconnect.Drone_Last_Cnt>2000)
////		block_disconnect.Drone_Disconnect_Cnt++;
////	else 
////		block_disconnect.Drone_Disconnect_Cnt = 0;
////	if(block_disconnect.Drone_Disconnect_Cnt>=5000)
////	{
////		block_disconnect.is_drone_down = 1;
////		Sentry.drone_alarm = 1;
////	}
////	else 
////		block_disconnect.is_drone_down = 0;

////	//拨弹电机堵转检测
////	if(ABS(BodanMotor.I_Set)>=BodanCurrentLimit*0.95)
////		block_disconnect.Bodan_block++;
////	else
////		block_disconnect.Bodan_block=0;

//}

    
    disconnect_reset();//初次进入掉电检测任务时执行一次复位
    //下面是借鉴英雄的RTOS代码里面的掉电检测(不过感觉好像有一点问题)
    while(1)
    {
        	
	if(block_disconnect.Bodan_Disconnect_Cnt>100)   
        ;  //执行拨弹掉电的操作
		
	
	/*底盘掉线检测*/
	if(block_disconnect.Chassis_Disconnect_Cnt>150)
		;  //执行底盘掉电检测
	
	/*云台掉线检测*/
	if(block_disconnect.Pitch_Disconnect_Cnt > 100)
		;  //执行pitch电机掉电的操作
    
    disconnect_count();//
    //诸如此类
	
    disconnect_count();//掉电计数增加一次
        //IWDG_Feed();//喂狗，即清空看门狗计数器
		 
    vTaskDelay(10); 	 
    }
}

//掉电时间计数器
static void disconnect_count(void)
{
    ;
}

//掉电计数值复位
static void disconnect_reset(void)
{
    block_disconnect.Bodan_Disconnect_Cnt = 0;  //诸如此类
    ;
}

