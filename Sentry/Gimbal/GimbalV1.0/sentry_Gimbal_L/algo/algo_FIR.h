#pragma once
typedef struct
{
	double* h;			//滤波系数数组头指针
	short length_h;		//滤波队列长度
}CoeFIR;				//滤波器参数结构体

//typedef struct
//{
//	double* x;
//	short length_x;
//}FIRInput;  //好像也没用到这个

//typedef struct
//{
//	double* y;
//	short length_y;  
//}FIROutput; //好像没用到这个

typedef struct
{
	LoopQueue_double x_delay;   //循环队列
	CoeFIR coe;					//滤波器参数结构体
}FilterFIR;						//滤波器变量结构体（包含一套滤波参数、待滤波变量队列）

double FIR_filter_RT(FilterFIR Filter, double NewInput);
FilterFIR FIRInit(CoeFIR coeInit);
