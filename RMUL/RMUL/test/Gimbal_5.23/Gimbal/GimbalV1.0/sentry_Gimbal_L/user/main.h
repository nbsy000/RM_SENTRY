/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       main.c/h
  * @brief      stm32初始化以及开始任务freeRTOS。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef MAIN_H
#define MAIN_H

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

#define Latitude_At_ShenZhen 22.57025f

#define NEW_INS  //使用新陀螺仪

#ifndef PI
#define PI 3.14159265358979f
#endif

#define CAN_CHOOSE 0

#define NEW_SHOOTAIM

#define ABS(x) ((x)>0? (x):(-(x))) 
#define LIMIT_MAX_MIN(x, max, min)	(((x) <= (min)) ? (min):(((x) >= (max)) ? (max) : (x)))  //限幅宏函数

#define GIMBAL_LEFT 0
#define GIMBAL_RIGHT 1

//-------------------------------------头文件包含---------------------------------------//
//Standard Lib
#include <stm32f4xx.h>	 
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

//hardware
#include "HW_CAN.h"
#include "HW_USART2_PC.h"
#include "HW_USART3_RC.h"
#include "HW_UART4.h"
#include "HW_TIM8_pwm.h"
#include "HW_TIM2_delay.h"
#include "HW_GPIO_LED.h"
#include "HW_FrictionWheel.h"
#include "HW_IWDG.h"
#include "uart5.h"
#include "tim4.h"
#include "counter.h"
#include "debug.h"

//Algorithm
#include "algo_PID.h"
#include "algo_CRC.h"
#include "algo_ADT_LoopQueue.h"
#include "algo_FIR.h"
#include "fuzzy_pid.h"
#include "yaw_fuzzy_pid.h"
#include "icm20602.h"
#include "ins.h"
#include "kalman_filter.h"
#include "QuaternionEKF.h"

//tasks
#include "task_ZeroCheck.h"
#include "task_ActionUpdate.h"
#include "task_DataSend.h"
#include "task_Gimbal_L.h"
#include "task_Shoot.h"
#include "task_StateLED.h"
#include "task_UsageCPU.h"
#include "task_BlockDisconnect.h"
#include "task_CV_DataSend.h"//#include "DataSendTask.h"
#include "task_ins.h"
#include "taskIT_DataReceive.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "start_task.h"

typedef  struct{
		int HeatDiscount;
		int RemoteDisconnect;
		int ChassisYawconnect;
		int ChassisGyroDisconnect;
	  int ChassisYawGyroDisconnect;
		int JudgeDisconnect;
		int AimShootDisconnect;
} roboDisconnect;

extern roboDisconnect Robo_Disconnect;


TickType_t GetSysCnt(void);
void Rob_Init(void);

extern roboDisconnect Robo_Disconnect;
    
#endif /* __MAIN_H */
