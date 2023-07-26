#ifndef __PID_H
#define __PID_H

#define I_SEP 1
#define I_NOSEP 0


typedef struct PID{
		float SetPoint;			//设定目标值
	
		float ActualValue;  //实际值
		float LastValue;//上一刻的实际值

    float DeadZone;
		float P;						//比例常数
		float I;						//积分常数
		float D;						//微分常数
		
		float LastError;		//前次误差
		float PreError;			//当前误差
		float SumError;			//积分误差
	
		float IMax;					//积分限制
		
		float POut;					//比例输出
		float IOut;					//积分输出
		float DOut;					//微分输出
	  float DOut_last;    //上一次微分输出
		float OutMax;       //限幅
	  float Out;          //总输出
		float Out_last;     //上一次输出
		
		uint8_t I_Sep;			//采用变速积分标志位
		float I_U;          //变速积分上限  用来提响应
		float I_L;          //变速积分下限  用来降超调
		
		float RC_DF;        //不完全微分滤波系数
}Pid_Typedef;

float PID_Calc(Pid_Typedef * P, float ActualValue);
#endif
