#include "main.h"
//#include "stdafx.h"
//#include "Filter.h"

/**
  * @brief  初始化FIR滤波器
  * @param  一套滤波器的参数
  * @retval 滤波器变量结构体
  */
FilterFIR FIRInit(CoeFIR coeInit)
{
	FilterFIR OutputFilter;									//待返回的结构体
	OutputFilter.x_delay = InitLoopQueue(coeInit.length_h); //把待滤波变量用一个环形队列初始化
	OutputFilter.coe = coeInit;								//把传入的参数给到待返回的滤波器结构体里去
	return OutputFilter;
}

/**
  * @brief  滤波函数
  * @param  一套完整滤波器变量、新加入的滤波值
  * @retval 滤波结果
  */
double FIR_filter_RT(FilterFIR Filter, double NewInput)
{
	LoopQueue_double *x_delay = &(Filter.x_delay); //接收滤波队列的地址
	CoeFIR coe = Filter.coe;					   //接收滤波参数结构体
	PushQueue(x_delay, NewInput);				   //新元素入队
//	int cnt = 0;  //这个cnt好像没用到
	short i;	  //循环变量
	double y = 0; //接收滤波结果
	for (i = 0; i < coe.length_h; i++)
	{
	 	// 核心递推公式大概
	 	y += coe.h[coe.length_h - 1 - i] * x_delay->Queue[(x_delay->Head + i) % x_delay->length];
		//推测：y += 第i个滤波参数 * 第i个滤波数值（只是推测）
	}

	return y;
}
