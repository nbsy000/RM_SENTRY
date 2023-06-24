#ifndef CHASIS_ESTIMATE_TASK_H
#define CHASIS_ESTIMATE_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "icm20602.h"
#include "counter.h"
#include "ins.h"

void ChasisEstimate_task(void *pvParameters);

extern IMU IMUReceive;
extern IMU_Data_t IMUData;

#endif // !CHASIS_ESTIMATE_TASK_H
