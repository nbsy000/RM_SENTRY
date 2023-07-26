#include "main.h"
/**
  * @brief  PID反馈算法
  * @param  PID_Struct *P  PID参数结构体
  *         ActualValue    PID计算反馈量（当前真实检测值）
  * @retval PID反馈计算输出值
  */
float PID_Calc(Pid_Typedef *P, float ActualValue)
{
		P->LastValue = P->ActualValue;
		P->ActualValue = ActualValue;
		P->LastError = P->PreError;
		P->PreError = P->SetPoint - P->ActualValue;
	  if((ABS(P->PreError)< P->DeadZone ))   //死区控制
		{
			P->PreError = 0.0f;			
		}
		
		      //微分先行
		float DM = P->D*(P->ActualValue - P->LastValue);   //微分先行
		
		if(P->I_Sep == I_SEP)//积分分离
		{
					 //变速积分   (积分分离)
			if(ABS(P->PreError) < P->I_L )			
			{					 
				P->SumError += (P->PreError+P->LastError)/2.0f;    
				P->SumError = LIMIT_MAX_MIN(P->SumError,P->IMax,- P->IMax);
			}
			 else if( ABS(P->PreError) < P->I_U )
			{
					 //梯形积分
				P->SumError += (P->PreError+P->LastError)/2.0f*(P->I_U - ABS(P->PreError))/(P->I_U - P->I_L);    
				P->SumError = LIMIT_MAX_MIN(P->SumError,P->IMax,- P->IMax);		
			}
				else
					P->SumError = 0;
		}
		else 
		{
				P->SumError += (P->PreError+P->LastError)/2.0f;    
				P->SumError = LIMIT_MAX_MIN(P->SumError,P->IMax,- P->IMax);		
		}
				
		P->POut = P->P * P->PreError;
		
		P->IOut = P->I * P->SumError;
		    
		    //不完全微分
		P->DOut_last = P->DOut; 
		P->DOut = DM * P->RC_DF + P->DOut_last * ( 1 - P->RC_DF );    
		
		P->Out_last  = P->Out;
		P->Out = LIMIT_MAX_MIN(P->POut+P->IOut+P->DOut,P->OutMax,-P->OutMax);	
		return P->Out;
}

