/**********************************************************************************************************
 * @文件     StartTask.c
 * @说明     操作系统启动文件
 * @版本  	 V1.0
 * @作者     陈志鹏
 * @日期     2020.1
**********************************************************************************************************/
#include "main.h"
//uint32_t CPU_high_water;

/*任务优先级数值越低，优先级越低*/

#define START_TASK_PRIO 1  //任务优先级
#define START_STK_SIZE 512 //任务堆栈1
TaskHandle_t StartTask_Handler; //任务句柄

#define CPU_TASK_PRIO 2  //任务优先级
#define CPU_STK_SIZE 128 //任务堆栈2
TaskHandle_t CPUTask_Handler; //任务句柄

#define OFFLINE_CHECK_TASK_PRIO 18  //任务优先级
#define OFFLINE_CHECK_STK_SIZE 128 //任务堆栈
TaskHandle_t OfflineCheckTask_Handler; //任务句柄

#define JUDGERECEIVE_TASK_PRIO 20  //任务优先级
#define JUDGERECEIVE_STK_SIZE 512 //任务堆栈3
TaskHandle_t JudgeReceiveTask_Handler; //任务句柄

#define CHASSIS_TASK_PRIO 29  //任务优先级
#define CHASSIS_STK_SIZE 1024 //任务堆栈4
TaskHandle_t ChassisTask_Handler; //任务句柄

#define CHASSISYAW_TASK_PRIO 28  //任务优先级
#define CHASSISYAW_STK_SIZE 512 //任务堆栈4
TaskHandle_t ChassisYawTask_Handler; //任务句柄

#define task_ActionUpdate_PRIO 29
#define task_ActionUpdate_SIZE 512
TaskHandle_t task_ActionUpdate_Handler;

#define SDCard_TASK_PRIO 20  //任务优先级
#define SDCard_STK_SIZE 1024 //任务堆栈2
TaskHandle_t SDCardTask_Handler; //任务句柄

#define INS_TASK_PRIO 31  //任务优先级
#define INS_STK_SIZE 128 //任务堆栈2
TaskHandle_t INS_Task_Handler; //任务句柄

#define task_ZeroCheck_PRIO 30
#define task_ZeroCheck_SIZE 512
TaskHandle_t task_ZeroCheck_Handler;

TaskHandle_t User_Tasks[TASK_NUM];

/**********************************************************************************************************
*函 数 名: start_task
*功能说明: 创建所有任务
*形    参: *pvParameters
*返 回 值: 无
**********************************************************************************************************/
void start_task(void *pvParameters)
{
  taskENTER_CRITICAL();//进入临界区
	
#ifdef DEBUG_MODE_FREERTOS								
	xTaskCreate((TaskFunction_t)CPU_task,          //任务函数
                (const char *)"CPU_task",          //任务名称
                (uint16_t)CPU_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)CPU_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&CPUTask_Handler); //任务句柄
#endif // DEBUG_MODE_FREERTOS
//								
//	xTaskCreate((TaskFunction_t)SDCard_task,          //任务函数
//                (const char *)"SDCard_task",          //任务名称
//                (uint16_t)SDCard_STK_SIZE,            //任务堆栈大小
//                (void *)NULL,                        //传递给任务函数的参数
//                (UBaseType_t)SDCard_TASK_PRIO,        //任务优先级
//                (TaskHandle_t *)&SDCardTask_Handler); //任务句柄
								
	//状态更新 Status_Act
	xTaskCreate((TaskFunction_t)task_ActionUpdate,
							(const char*)"task_ActionUpdate",
							(uint16_t)task_ActionUpdate_SIZE,
							(void*)NULL,
							(UBaseType_t)task_ActionUpdate_PRIO,
							(TaskHandle_t*)&task_ActionUpdate_Handler);
		
		
	xTaskCreate((TaskFunction_t)Chassis_task,          //任务函数
                (const char *)"Chassis_task",          //任务名称
                (uint16_t)CHASSIS_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)CHASSIS_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&ChassisTask_Handler); //任务句柄
								
	xTaskCreate((TaskFunction_t)ChassisYaw_task,          //任务函数
                (const char *)"ChassisYaw_task",          //任务名称
                (uint16_t)CHASSISYAW_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)CHASSISYAW_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&ChassisYawTask_Handler); //任务句柄								
								
	xTaskCreate((TaskFunction_t)Offline_Check_task,          //任务函数
                (const char *)"Offline_Check_task",          //任务名称
                (uint16_t)OFFLINE_CHECK_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)OFFLINE_CHECK_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&OfflineCheckTask_Handler); //任务句柄

								
	xTaskCreate((TaskFunction_t)JudgeReceive_task,          //任务函数
                (const char *)"JudgeReceive_task",          //任务名称
                (uint16_t)JUDGERECEIVE_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)JUDGERECEIVE_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&JudgeReceiveTask_Handler); //任务句柄
								
	//过零检测任务
	xTaskCreate((TaskFunction_t)task_ZeroCheck,
							(const char*)"task_ZeroCheck",
							(uint16_t)task_ZeroCheck_SIZE,
							(void*)NULL,
							(UBaseType_t)task_ZeroCheck_PRIO,
							(TaskHandle_t*)&task_ZeroCheck_Handler); 								
								
#ifdef NEW_INS								 
		//陀螺仪解算任务
		 xTaskCreate((TaskFunction_t)ins_task,
						 (const char*)"ins_task",
						 (uint16_t)INS_STK_SIZE,
						 (void*)NULL,
						 (UBaseType_t)INS_TASK_PRIO,
						 (TaskHandle_t*)&INS_Task_Handler);   
#endif  //NEW_INS
								
	vTaskDelete(StartTask_Handler); //删除开始任务
								
  taskEXIT_CRITICAL();            //退出临界区
}

/**********************************************************************************************************
*函 数 名: startTast
*功能说明: 创建初始化任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void startTast(void)
{
    xTaskCreate((TaskFunction_t)start_task,          //任务函数
                (const char *)"start_task",          //任务名称
                (uint16_t)START_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)START_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&StartTask_Handler); //任务句柄
}


