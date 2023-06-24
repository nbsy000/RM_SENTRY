#ifndef __TASK_ZEROCHECK_H
#define __TASK_ZEROCHECK_H

#define Position 1
#define Speed    2


typedef struct 
{
	float Circle;           //转过圈数
	float CountCycle;       //转过一圈的总计数周期
	float LastValue;        //检测过零量上一次的值	
	float ActualValue;      //检测过零量当前值
	float PreError;         //检测量判断差值
}ZeroCheck_Typedef;


void task_ZeroCheck(void);
void ZeroCheck_Init(void);

float PitchAngleOutPut(uint8_t);
float GyroPitchOutPut(uint8_t);

float YawAngleOutPut(uint8_t);
float GyroYawOutPut(uint8_t);

#endif //__TASK_ZEROCHECK_H

