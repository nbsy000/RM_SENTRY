/**********************************************************************************************************
 * @文件     ZeroCheckTask.c
 * @说明     过零检测
 * @版本  	 V1.0
 * @作者     黄志雄
 * @日期     2020.1
**********************************************************************************************************/
#include "main.h"
extern Gyro_Typedef Gyro_Chassis;//底盘陀螺仪数据
extern Gyro_Typedef Gyro_ChassisYaw;//大Yaw陀螺仪数据
extern Chassis_Motor_t ChassisMotor[4];//四个电机
extern Motor_9025_t Motor_9025;//9025电机


void task_ZeroCheck(void)
{

    ZeroCheck_Init();
    //过零检测结构体的初始化
    while(1)
    {
				ZeroCheck_cal();
        vTaskDelay(1);
    }
}

/**********************************************************************************************************
*函 数 名: ZeroCheck
*功能说明: 位置式和速度式过零检测
					 Zero->ActualValue 表示检测量当前值
					 Zero->LastValue 表示检测量上一次值
					 Zero->CountCycle 表示检测量过零时越变值，即计数周期
					 Zero->PreError 表示检测量差值
					 使用此函数前要申明对应检测量结构体的 Zero->CountCycle与Zero->LastValue
*形    参: ZeroCheck_Typedef *Zero  过零检测结构体
  *        float value  待检测量
*返 回 值: 取决于Zerocheck_mode，分别输出过零检测后位置值或速度值
**********************************************************************************************************/
float ZeroCheck(ZeroCheck_Typedef *Zero,float value,short Zerocheck_mode)
{
	Zero->ActualValue=value;
	
	Zero->PreError=Zero->ActualValue-Zero->LastValue;
	Zero->LastValue=Zero->ActualValue;
	
	if(Zero->PreError>0.7f*Zero->CountCycle)
	{
		Zero->PreError=Zero->PreError-Zero->CountCycle;
		Zero->Circle++;
	}
	if(Zero->PreError<-0.7f*Zero->CountCycle)
	{
		Zero->PreError=Zero->PreError+Zero->CountCycle;
		Zero->Circle--;
	}
	
	if(Zerocheck_mode==Position)
		return Zero->ActualValue - Zero->Circle*Zero->CountCycle;
	else if(Zerocheck_mode==Speed)
	  return Zero->PreError;
	else 
		return 0;
}



/**********************************************************************************************************
*函 数 名: ZeroCheck_cal
*功能说明: 过零检测执行函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ZeroCheck_cal(void)
{
	extern float dt;
	
	static int count = 0;
	count ++;
	if(count%2) //500Hz
	{
		for(int i=0;i<4;i++)
		{			
			ChassisMotor[i].Inencoder = ZeroCheck(&ChassisMotor[i].ZeroCheck_Motor,ChassisMotor[i].Encoder,Speed);
		}
		
		Motor_9025.multiAngle = ZeroCheck(&Motor_9025.zerocheck,Motor_9025.signalAngle,Position);//不直接读是因为9025的多圈角度值更新频率太低
	}
	
	//1000Hz
	
#ifdef NEW_INS
	Gyro_ChassisYaw.GZ = ZeroCheck(&Gyro_ChassisYaw.zerocheck_yaw,Gyro_ChassisYaw.YAW_ABS,Speed)/dt;
	ChassisYaw_Send();
#endif
}


/**********************************************************************************************************
*函 数 名: ZeroCheck_Init
*功能说明: 过零检测结构体参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ZeroCheck_Init(void)
{
	
	//四个电机
	for(int i=0;i<4;i++)
	{
		ChassisMotor[i].ZeroCheck_Motor.CountCycle=8192;
		ChassisMotor[i].ZeroCheck_Motor.LastValue=ChassisMotor[i].Encoder;
		ChassisMotor[i].ZeroCheck_Motor.Circle = 0;
	}
	
	//大Yaw轴电机
	Motor_9025.zerocheck.CountCycle=36000;
	Motor_9025.zerocheck.LastValue=Motor_9025.signalAngle;
	Motor_9025.zerocheck.Circle = 0;
}


