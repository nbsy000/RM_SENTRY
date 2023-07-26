#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Start_Task.h"

#define task_ActionUpdate_PRIO 30
#define task_ActionUpdate_SIZE 512
TaskHandle_t task_ActionUpdate_Handler;

#define task_GimbalR_PRIO 30
#define task_GimbalR_SIZE 1024
TaskHandle_t task_GimbalR_Handler;

#define task_Shoot_PRIO 30
#define task_Shoot_SIZE 1024
TaskHandle_t task_Shoot_Handler;

#define task_CV_DataSend_PRIO 31  //30
#define task_CV_DataSend_SIZE 512
TaskHandle_t task_CV_DataSend_Handler;

#define task_StateLED_PRIO 9
#define task_StateLED_SIZE 128
TaskHandle_t task_StateLED_Handler = NULL;

#define task_ZeroCheck_PRIO 30
#define task_ZeroCheck_SIZE 512
TaskHandle_t task_ZeroCheck_Handler;

#define CPU_TASK_PRIO 2  //任务优先级
#define CPU_STK_SIZE 128 //任务堆栈2
TaskHandle_t CPUTask_Handler; //任务句柄

#define INS_TASK_PRIO 31  //任务优先级
#define INS_STK_SIZE 128 //任务堆栈2
TaskHandle_t INS_Task_Handler; //任务句柄

#define START_TASK_PRIO 1
#define START_STK_SIZE 512
TaskHandle_t StartTask_Handler = NULL;

TaskHandle_t User_Tasks[TASK_NUM];

//这个就是引导任务了
static void start_task(void *pvParameters)
{
    //BaseType_t xReturn = pdPASS;
    
    taskENTER_CRITICAL();//临界段保护

    //状态更新 Status_Act
    xTaskCreate((TaskFunction_t)task_ActionUpdate,   
                (const char *)"task_ActionUpdate",
                (uint16_t)task_ActionUpdate_SIZE,
                (void *)NULL,
                (UBaseType_t)task_ActionUpdate_PRIO,
                (TaskHandle_t *)&task_ActionUpdate_Handler);

    //云台任务
    xTaskCreate((TaskFunction_t)task_GimbalR,
                (const char *)"task_GimbalR",
                (uint16_t)task_GimbalR_SIZE,
                (void *)NULL,
                (UBaseType_t)task_GimbalR_PRIO,
                (TaskHandle_t *)&task_GimbalR_Handler);
                
    //射击任务
    xTaskCreate((TaskFunction_t)task_Shoot,
                (const char *)"task_Shoot",
                (uint16_t)task_Shoot_SIZE,
                (void *)NULL,
                (UBaseType_t)task_Shoot_PRIO,
                (TaskHandle_t *)&task_Shoot_Handler);
//                
     // 和TX2通信的任务  (定期发云台角度和正脉冲)
     xTaskCreate((TaskFunction_t)task_CV_DataSend,
                 (const char*)"task_CV_DataSend",
                 (uint16_t)task_CV_DataSend_SIZE,
                 (void*)NULL,
                 (UBaseType_t)task_CV_DataSend_PRIO,
                 (TaskHandle_t*)&task_CV_DataSend_Handler);  
								 
    //闪状态灯任务
    xTaskCreate((TaskFunction_t)task_StateLED,
                (const char*)"task_StateLED",
                (uint16_t)task_StateLED_SIZE,
                (void*)NULL,
                (UBaseType_t)task_StateLED_PRIO,
                (TaskHandle_t*)&task_StateLED_Handler);
								
#ifdef NEW_INS								 
		//陀螺仪解算任务
		 xTaskCreate((TaskFunction_t)ins_task,
						 (const char*)"ins_task",
						 (uint16_t)INS_STK_SIZE,
						 (void*)NULL,
						 (UBaseType_t)INS_TASK_PRIO,
						 (TaskHandle_t*)&INS_Task_Handler);   
#endif  //NEW_INS
                
    //过零检测任务
    xTaskCreate((TaskFunction_t)task_ZeroCheck,
                (const char*)"task_ZeroCheck",
                (uint16_t)task_ZeroCheck_SIZE,
                (void*)NULL,
                (UBaseType_t)task_ZeroCheck_PRIO,
                (TaskHandle_t*)&task_ZeroCheck_Handler);                
                
#ifdef DEBUG_MODE_FREERTOS
    xTaskCreate((TaskFunction_t)CPU_task,               //任务函数
                (const char *)"CPU_task",               //任务名称
                (uint16_t)CPU_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                           //传递给任务函数的参数
                (UBaseType_t)CPU_TASK_PRIO,             //任务优先级
                (TaskHandle_t *)&CPUTask_Handler); //任务句柄
#endif // DEBUG_MODE_FREERTOS
                
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界段
}

void start_the_very_first_task(void)
{
    //开启引导任务
    xTaskCreate((TaskFunction_t)start_task,          //任务函数
                (const char *)"start_task",          //任务名称
                (uint16_t)START_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)START_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&StartTask_Handler); //任务句柄
}
