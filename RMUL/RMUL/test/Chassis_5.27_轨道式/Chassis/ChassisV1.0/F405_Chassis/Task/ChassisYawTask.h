#ifndef __CHASSISYAWTASK_H
#define __CHASSISYAWTASK_H

extern uint8_t aim_flag;

void ChassisYaw_task(void *pvParameters);

void ChassisYaw_RC_Act(void);
void ChassisYaw_SLEEP_Act(void);
void ChassisYaw_DEBUG_Act(void);
void ChassisYaw_PC_Act(void);
void ChassisYaw_STOP_Act(void);

void ChassisYaw_Init(void);

extern short C_I;
#endif
