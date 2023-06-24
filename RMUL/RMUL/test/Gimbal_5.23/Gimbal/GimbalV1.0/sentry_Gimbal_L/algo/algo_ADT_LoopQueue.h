#include <stdlib.h>

//数据结构：循环块队列
typedef struct
{
	double * Queue;  //队列块指针
	short Head;   	 //标志队列头的序号
	short length;    //队列长度记录
	short IfEmpty;   //标识队列是否为空
}LoopQueue_double;
LoopQueue_double InitLoopQueue(int lengthOfQueue);
void PushQueue(LoopQueue_double* queue, double PushNum);
#pragma once
