#include "main.h"
//#include "stdafx.h"
//#include "Filter.h"


//循环队列，头指针指向下一个push进来的节点
/**
 * @brief  初始化循环队列
 * @param  需要的队列长
 * @retval 循环队列变量
 */
LoopQueue_double InitLoopQueue(int lengthOfQueue)
{
	double* Init = (double*)malloc((lengthOfQueue) * sizeof(double));  		//要块空间
	LoopQueue_double OutputQueue;  		//待返回的队列变量
	OutputQueue.length = lengthOfQueue;  //给到队列长度
	OutputQueue.Queue = Init;			 //指向队列块
	OutputQueue.Head = 0;				 //初始化队列头
	OutputQueue.IfEmpty = 1;			 //初始化空队标志
	int i;		//循环变量
	for (i = 0;i < lengthOfQueue;i++)
	{
		OutputQueue.Queue[i] = 0; //队列全初始化为0  （可以考虑和calloc合并简化一下）
	}
	return OutputQueue;
}

/**
 * @brief  入队函数
 * @param  队列变量的地址，待入队的变量
 * @retval 无
 */
void PushQueue(LoopQueue_double* queue, double PushNum)
{
	queue->IfEmpty = 0;  //更新空值标识
	(queue->Queue)[queue->Head] = PushNum;  //在队列头覆盖待入队的变量
	queue->Head = ((queue->Head) + 1) % (queue->length); //后移一位队列头的位置
}

