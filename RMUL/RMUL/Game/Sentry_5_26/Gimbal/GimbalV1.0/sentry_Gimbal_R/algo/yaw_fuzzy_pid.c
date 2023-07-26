/**********************************************************************************************************
 * @文件     pid.c
 * @说明     pid算法
 * @版本  	 V1.0
 * @作者     黄志雄
 * @日期     2020.1
**********************************************************************************************************/
#include "main.h"

/**********************************************************************************************************
*函 数 名: PID_Calc
*功能说明: PID反馈算法
*形    参: PID_Struct *P  PID参数结构体
  *        ActualValue    PID计算反馈量（当前真实检测值）
*返 回 值: PID反馈计算输出值
**********************************************************************************************************/
//float PID_Calc(Pid_Typedef *P, float ActualValue)
//{
//		P->PreError = P->SetPoint - ActualValue;
//		P->dError = P->PreError - P->LastError;
//	
//	  P->SetPointLast = P->SetPoint;
//	
//		P->SumError += P->PreError;
//		P->LastError = P->PreError;
//		
//		if(P->SumError >= P->IMax)
//			P->SumError = P->IMax;
//		else if(P->SumError <= -P->IMax)
//			P->SumError = -P->IMax;
//		
//		P->POut = P->P * P->PreError;
//		P->IOut = P->I * P->SumError;
//		P->DOut = P->D * P->dError;
//		
//		return LIMIT_MAX_MIN(P->POut+P->IOut+P->DOut,P->OutMax,-P->OutMax); 
//}




/*********模糊pid部分*/

#define YAW_IS_Kp 1
#define YAW_IS_Ki 2
#define YAW_IS_Kd 3
//给pitch轴用的参数
//#define stair 0.4
//#define pstair 0.03 //与普通PID的参数可能值有关
//#define istair 0.00002
//#define dstair 0.005
//给yaw轴用的参数
#define yaw_stair 0.4
#define yaw_pstair 0.02
#define yaw_istair 0.00002
#define yaw_dstair 0.005
// 
#define YAW_NL   -(3*yaw_stair)
#define YAW_NM	 -(2*yaw_stair)
#define YAW_NS	 -(1*yaw_stair)
#define YAW_ZE	 0
#define YAW_PS	 (1*yaw_stair)
#define YAW_PM	 (2*yaw_stair)
#define YAW_PL	 (3*yaw_stair)

#define YAW_NLp   -(3*yaw_pstair)
#define YAW_NMp	 -(2*yaw_pstair)
#define YAW_NSp	 -(1*yaw_pstair)
#define YAW_ZEp	 0
#define YAW_PSp	 (1*yaw_pstair)
#define YAW_PMp	 (2*yaw_pstair)
#define YAW_PLp	 (3*yaw_pstair)
 
#define YAW_NLi   -(3*yaw_istair)
#define YAW_NMi	 -(2*yaw_istair)
#define YAW_NSi	 -(1*yaw_istair)
#define YAW_ZEi	 0
#define YAW_PSi	 (1*yaw_istair)
#define YAW_PMi	 (2*yaw_istair)
#define YAW_PLi	 (3*yaw_istair) 

#define YAW_NLd   -(3*yaw_dstair)
#define YAW_NMd	 -(2*yaw_dstair)
#define YAW_NSd	 -(1*yaw_dstair)
#define YAW_ZEd	 0
#define YAW_PSd	 (1*yaw_dstair)
#define YAW_PMd	 (2*yaw_dstair)
#define YAW_PLd	 (3*yaw_dstair)
 
 
static const float fuzzyRuleKp[7][7]={
	YAW_PLp,	YAW_PLp,	YAW_PMp,	YAW_PMp,	YAW_PSp,	YAW_ZEp,	YAW_ZEp,
	YAW_PLp,	YAW_PLp,	YAW_PMp,	YAW_PSp,	YAW_PSp,	YAW_ZEp,	YAW_NSp,
	YAW_PMp,	YAW_PMp,	YAW_PMp,	YAW_PSp,	YAW_ZEp,	YAW_NSp,	YAW_NSp,
	YAW_PMp,	YAW_PMp,	YAW_PSp,	YAW_ZEp,	YAW_NSp,	YAW_NMp,	YAW_NMp,
	YAW_PSp,	YAW_PSp,	YAW_ZEp,	YAW_NSp,	YAW_NSp,	YAW_NMp,	YAW_NMp,
	YAW_PSp,	YAW_ZEp,	YAW_NSp,	YAW_NMp,	YAW_NMp,	YAW_NMp,	YAW_NLp,
	YAW_ZEp,	YAW_ZEp,	YAW_NMp,	YAW_NMp,	YAW_NMp,	YAW_NLp,	YAW_NLp
};
 
static const float fuzzyRuleKi[7][7]={
	YAW_NLi,	YAW_NLi,	YAW_NMi,	YAW_NMi,	YAW_NSi,	YAW_ZEi,	YAW_ZEi,
	YAW_NLi,	YAW_NLi,	YAW_NMi,	YAW_NSi,	YAW_NSi,	YAW_ZEi,	YAW_ZEi,
	YAW_NLi,	YAW_NLi,	YAW_NSi,	YAW_NSi,	YAW_ZEi,	YAW_PSi,	YAW_PSi,
	YAW_NMi,	YAW_NMi,	YAW_NSi,	YAW_ZEi,	YAW_PSi,	YAW_PMi,	YAW_PMi,
	YAW_NMi,	YAW_NMi,	YAW_ZEi,	YAW_PSi,	YAW_PSi,	YAW_PMi,	YAW_PLi,
	YAW_ZEi,	YAW_ZEi,	YAW_PSi,	YAW_PSi,	YAW_PMi,	YAW_PLi,	YAW_PLi,
	YAW_ZEi,	YAW_ZEi,	YAW_PSi,	YAW_PMi,	YAW_PMi,	YAW_PLi,	YAW_PLi
};
 
static const float fuzzyRuleKd[7][7]={
	YAW_PSd,	YAW_NSd,	YAW_NLd,	YAW_NLd,	YAW_NLd,	YAW_NMd,	YAW_PSd,
	YAW_PSd,	YAW_NSd,	YAW_NLd,	YAW_NMd,	YAW_NMd,	YAW_NSd,	YAW_ZEd,
	YAW_ZEd,	YAW_NSd,	YAW_NMd,	YAW_NMd,	YAW_NSd,	YAW_NSd,	YAW_ZEd,
	YAW_ZEd,	YAW_NSd,	YAW_NSd,	YAW_NSd,	YAW_NSd,	YAW_NSd,	YAW_ZEd,
	YAW_ZEd,	YAW_ZEd,	YAW_ZEd,	YAW_ZEd,	YAW_ZEd,	YAW_ZEd,	YAW_ZEd,
	YAW_PLd,	YAW_NSd,	YAW_PSd,	YAW_PSd,	YAW_PSd,	YAW_PSd,	YAW_PLd,
	YAW_PLd,	YAW_PMd,	YAW_PMd,	YAW_PMd,	YAW_PSd,	YAW_PSd,	YAW_PLd
};
 

 //关键算法
static Fuzzy_Typedef yaw_fuzzy(float e,float ec)
{

     float etemp,ectemp;
     float eLefttemp,ecLefttemp;
     float eRighttemp ,ecRighttemp;
 
     short eLeftIndex,ecLeftIndex;
     short eRightIndex,ecRightIndex;

     Fuzzy_Typedef      fuzzy_PID;
    	//etemp = e > 3.0 ? 0.0 : (e < - 3.0 ? 0.0 : (e >= 0.0 ? (e >= 2.0 ? 2.5: (e >= 1.0 ? 1.5 : 0.5)) : (e >= -1.0 ? -0.5 : (e >= -2.0 ? -1.5 : (e >= -3.0 ? -2.5 : 0.0) ))));

		


     if(e>YAW_PL||e<YAW_NL)
			 etemp=0;//不做计算

		 if(YAW_PL>e&&e>YAW_PM)
			 etemp=YAW_PM;
		 else if(YAW_PM>e&&e>YAW_PS)
			 etemp=YAW_PS;
		 else if(YAW_PS>e&&e>YAW_ZE)
			 etemp=YAW_ZE;
		 else if(YAW_ZE>e&&e>YAW_NS)
			 etemp=YAW_NS;
		 else if(YAW_NS>e&&e>YAW_NM)
			 etemp=YAW_NM;
		 else if(YAW_NM>e&&e>YAW_NL)
			 etemp=YAW_NL;
 
		 //计算E隶属度
		 	//eRighttemp= etemp == 0.0 ? 0.0:((e-etemp)/stair);
		  //eLefttemp= etemp == 0.0 ? 0.0:(1- eRighttemp);
	    if( etemp == 0)
				eRighttemp=0;
			else
				eRighttemp=((e-etemp)/yaw_stair);
		 
		  if( etemp == 0)
				eLefttemp=0;
			else
				eLefttemp=(1- eRighttemp);
		 
		 


	eLeftIndex =(short) ((etemp-YAW_NL)/yaw_stair);       //例如 etemp=2.5，YAW_NL=-3，那么得到的序列号为5  【0 1 2 3 4 5 6】
	eRightIndex=(short) (eLeftIndex+1);
	   

		 
		 
		 if(ec>YAW_PL||ec<YAW_NL)
			 ectemp=0;
		 if(YAW_PL>ec&&ec>YAW_PM)
			 ectemp=YAW_PM;
		 else if(YAW_PM>ec&&ec>YAW_PS)
			 ectemp=YAW_PS;
		 else if(YAW_PS>ec&&ec>YAW_ZE)
			 ectemp=YAW_ZE;
		 else if(YAW_ZE>ec&&ec>YAW_NS)
			 ectemp=YAW_NS;
		 else if(YAW_NS>ec&&ec>YAW_NM)
			 ectemp=YAW_NM;
		 else if(YAW_NM>ec&&ec>YAW_NL)
			 ectemp=YAW_NL;
	  //计算EC隶属度
		 	//ecRighttemp= ectemp == 0.0 ? 0.0:((ec-ectemp)/stair);
	    //ecLefttemp = ectemp == 0.0 ? 0.0:(1- ecRighttemp);
	    if( ectemp == 0)
				ecRighttemp=0;
			else
				ecRighttemp=((ec-ectemp)/yaw_stair);
		 
		  if( ectemp == 0)
				ecLefttemp=0;
			else
				ecLefttemp=(1- ecRighttemp);
			
			
		 	ecLeftIndex =(short) ((ectemp-YAW_NL)/yaw_stair);  
	    ecRightIndex= (short)(eLeftIndex+1);
	 
	
	
	
	
	
//     eLeftIndex = (int)e;
//     eRightIndex = eLeftIndex;
//     eLeftIndex = (int)((etemp-0.5) + 3);        //[-3,3] -> [0,6]
//     eRightIndex = (int)((etemp+0.5) + 3);
// 
//     eLefttemp =etemp == 0.0 ? 0.0:((etemp+0.5)-e);
//     eRighttemp=etemp == 0.0 ? 0.0:( e-(etemp-0.5));
// 
//     ectemp = ec > 3.0 ? 0.0 : (ec < - 3.0 ? 0.0 : (ec >= 0.0 ? (ec >= 2.0 ? 2.5: (ec >= 1.0 ? 1.5 : 0.5)) : (ec >= -1.0 ? -0.5 : (ec >= -2.0 ? -1.5 : (ec >= -3.0 ? -2.5 : 0.0) ))));
// 
//     ecLeftIndex = (int)((ectemp-0.5) + 3);        //[-3,3] -> [0,6]
//     ecRightIndex = (int)((ectemp+0.5) + 3);
// 
//     ecLefttemp =ectemp == 0.0 ? 0.0:((ectemp+0.5)-ec);
//     ecRighttemp=ectemp == 0.0 ? 0.0:( ec-(ectemp-0.5));
 
/*************************************反模糊*************************************/
 
 
 
 
	fuzzy_PID.Kp = (eLefttemp * ecLefttemp *  fuzzyRuleKp[eLeftIndex][ecLeftIndex]                   
   + eLefttemp * ecRighttemp * fuzzyRuleKp[eLeftIndex][ecRightIndex]
   + eRighttemp * ecLefttemp * fuzzyRuleKp[eRightIndex][ecLeftIndex]
   + eRighttemp * ecRighttemp * fuzzyRuleKp[eRightIndex][ecRightIndex]);
 
	fuzzy_PID.Ki =   (eLefttemp * ecLefttemp * fuzzyRuleKi[eLeftIndex][ecLeftIndex]
   + eLefttemp * ecRighttemp * fuzzyRuleKi[eLeftIndex][ecRightIndex]
   + eRighttemp * ecLefttemp * fuzzyRuleKi[eRightIndex][ecLeftIndex]
   + eRighttemp * ecRighttemp * fuzzyRuleKi[eRightIndex][ecRightIndex]);

 
	fuzzy_PID.Kd = (eLefttemp * ecLefttemp *    fuzzyRuleKd[eLeftIndex][ecLeftIndex]
   + eLefttemp * ecRighttemp * fuzzyRuleKd[eLeftIndex][ecRightIndex]
   + eRighttemp * ecLefttemp * fuzzyRuleKd[eRightIndex][ecLeftIndex]
   + eRighttemp * ecRighttemp * fuzzyRuleKd[eRightIndex][ecRightIndex]);
return fuzzy_PID;
 
}
 
//此处的全局变量用来保存模糊的kp，ki，kd增量 
Fuzzy_Typedef YAW_OUT ;

float Yaw_FuzzyPID_Calc(Fuzzy_Typedef *pid,float ActualValue)
{
	
	static float LastError;
 
	pid->ActPoint = ActualValue;
	pid->PreError = pid->SetPoint - pid->ActPoint;             //目标值-实际值
	pid->dError = pid->PreError - LastError;            //误差变化率
	pid->SumError += pid->PreError;
	
			
		if(pid->SumError >= pid->IMax)
			pid->SumError = pid->IMax;
		else if(pid->SumError <= -pid->IMax)
			pid->SumError = -pid->IMax;
	
	pid->LastError = pid->PreError;
	LastError=pid->PreError;
	
		pid->POut = pid->Kp * pid->PreError;
		pid->IOut = pid->Ki* pid->SumError;
		pid->DOut = pid->Kd * pid->dError;		
		
		
	YAW_OUT = yaw_fuzzy(pid->PreError, pid->dError);      //模糊调整  kp,ki,kd   形参1当前误差，形参2前后误差的差值
  //return pid->POut+pid->IOut+pid->DOut;
	//return (pid->Kp+OUT.Kp)*pid->PreError + (pid->Kd+OUT.Kd)*pid->dError + (pid->Ki+OUT.Ki)*pid->SumError;
return (pid->Kp+YAW_OUT.Kp)*pid->PreError + (pid->Kd+YAW_OUT.Kd)*pid->dError + (pid->Ki)*pid->SumError; //p 和 d 先行
}

