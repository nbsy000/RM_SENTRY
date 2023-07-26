/**
 ******************************************************************************
 * @file   
 * @brief   
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "main.h"
#include "task_ins.h"

/* IMU */
IMU IMUReceive;
IMU_Data_t IMUData;

/**
 * @brief 底盘位姿估计任务
 * @param[in] void
 */
void ins_task(void *pvParameters)
{
    portTickType xLastWakeTime;
    const portTickType xFrequency = 1; // 1kHZ

    //适当延时
    vTaskDelay(100);

    while (1)
    {
				extern gyro_Typedef Gyro_Right;
        xLastWakeTime = xTaskGetTickCount();

				//获取左云台陀螺仪
        INS_Task(&IMUReceive, &IMUData);
				Gyro_Right.YAW_ABS = INS.YawTotalAngle;
				Gyro_Right.PITCH = INS.Pitch;

        /*  喂狗 */

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
