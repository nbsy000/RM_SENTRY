#ifndef __ALGO_PID_H
#define __ALGO_PID_H

typedef struct PID
{
	float SetPoint; //设定目标值

	float P; //比例常数
	float I; //积分常数
	float D; //微分常数

	float LastError; //前次误差
	float PreError;	 //当前误差
	float SumError;	 //积分误差
	float dError;

	float IMax;	   //积分限制
	float I_Limit; //积分分离PID

	float POut; //比例输出
	float IOut; //积分输出
	float DOut; //微分输出
}PID_Typedef;

float PID_Calc(PID_Typedef *P, float ActualValue, uint8_t way);

#endif //__ALGO_PID_H
