#ifndef __ALGO_PID_H
#define __ALGO_PID_H

#define I_SEP 1
#define I_NOSEP 0

#define ORDER  11

#define DIR_ON 1 //开dir
#define DIR_OFF 0 //关dir

typedef struct{
		float K1;			
	  float K2;
	  float Last_DeltIn;
	  float Now_DeltIn;
	  float Out;
	  float OutMax;
}FeedForward_Typedef;

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
		float dError;	
	
		float IMax;					//积分限制
		float I_Limit; 			//积分分离PID		
	
		float POut;					//比例输出
		float IOut;					//积分输出
		float DOut;					//微分输出
	  float DOut_last;    //上一次微分输出
		float OutMax;       //限幅
	  float Out;          //总输出
		float Out_last;     //上一次输出
		
		float I_U;          //变速积分上限
		float I_L;          //变速积分下限
		
		float RC_DF;        //不完全微分滤波系数
		
		float BufferDout[ORDER + 1];  //DIR滤波缓冲
		
}PID_Typedef;


/*********模糊pid部分*/
typedef struct
{
		float SetPoint;			//设定目标值
	
		float ActualValue;  //实际值
		float LastValue;//上一刻的实际值

    float DeadZone;
		
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
		
		float I_U;          //变速积分上限
		float I_L;          //变速积分下限
		
		float RC_DM;        //微分先行滤波系数
		float RC_DF;        //不完全微分滤波系数
	
	  float Kp0;          //PID初值
	  float Ki0;
  	float Kd0;
	
	  float dKp;          //PID变化量
	  float dKi;
  	float dKd;
	
    float stair ;	      //动态调整梯度   //0.25f
	  float Kp_stair;                      //0.015f
	  float Ki_stair;                      //0.0005f
	  float Kd_stair;                      //0.001f
	  
}FuzzyPID;

float PID_Calc(PID_Typedef *P, float ActualValue,uint8_t I_Sep,uint8_t DIR_STATE);
void PID_Clear(PID_Typedef *P);
float FeedForward_Calc(FeedForward_Typedef *FF);
float Fir_Dout(float Input,float* Buffer);
void Fir_Dout_Clear(float* Buffer);
float FuzzyPID_Calc(FuzzyPID *P,float ActualValue);
#endif //__ALGO_PID_H
