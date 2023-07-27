#ifndef _PC_SERIAL_H
#define _PC_SERIAL_H

#include "pc_uart.h"
#include "algorithmOfCRC.h"
#include "ins.h"

#include "FreeRTOS.h"
#include "task.h"

void PCReceive(unsigned char *PCbuffer);
void SendtoPC(void);

void NAVReceive(uint8_t Buf[]);
void SendtoNAV(void);
extern PCRecvData pc_recv_data;
extern uint8_t armor_state; 
extern float pc_pitch,pc_yaw;
extern float distance;
#endif // !_PC_SERIAL_H
