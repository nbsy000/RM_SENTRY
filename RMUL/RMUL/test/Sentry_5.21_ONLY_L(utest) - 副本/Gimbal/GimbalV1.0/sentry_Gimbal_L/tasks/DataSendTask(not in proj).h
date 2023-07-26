#ifndef __DATASENDTASK_H__
#define __DATASENDTASK_H__

typedef struct{
	char SuperPowerLimit;	//0为超级电容关闭，不为0则开启使用超级电容
	char Chassis_Flag;			//0代表正常模式  1代表小陀螺  2代表大陀螺  3代表单挑模式 
	char Mag_Flag;					//0表示弹仓盖关闭，1为打开
	char Laser_Flag;				//0表示激光关闭，1为打开
}F405_typedef;

//void BodanCan1Send(short a);
//void F405Can1Send(F405_typedef *F405_Send);
//void GimbalCan2Send(short X,short Y);
//void ChassisCan1Send(short *carSpeedx,short *carSpeedy,short *carSpeedw);

//void USART6_SendtoPC(void);
//void Tx2_Off_CheckAndSet(u8* Buff);
//void TX2_task(void *pvParameters);

void task_CV_DataSend(void *pvParameters);

#define Tx2_Off 0xff			//关机模式
#define Tx2_Small_Buff 0x10	    //小符模式
#define Tx2_Big_Buff 0x20		//大符模式
#define Tx2_Armor 0x30			//辅瞄模式

#endif
