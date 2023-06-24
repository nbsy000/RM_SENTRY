#ifndef __TASK_DATASEND_H
#define __TASK_DATASEND_H


void Gimbal_Gyro_Send(void);
void Bodan_Can1Send(int16_t bodanVal,int16_t fric0, int16_t fric1);
void Friction_Can1Send(int16_t fric0, int16_t fric1); //·¢Ä¦²ÁÂÖµçÁ÷
void Remote_Can1Send(void);
void Judge_Msg_Send(void);
void Gimbal_Send(void);
void NAV_Msg_Send(void);
void Aim_Msg_Send(void);
void Gimbal_Syne_Send(uint8_t Buf[]);
#endif //__TASK_DATASEND_H
