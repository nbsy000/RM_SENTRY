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
float PID_Calc(Pid_Typedef *P, float ActualValue)
{
		P->PreError = P->SetPoint - ActualValue;
		P->dError = P->PreError - P->LastError;
	
	  P->SetPointLast = P->SetPoint;
	
		P->SumError += P->PreError;
		P->LastError = P->PreError;
		
		if(P->SumError >= P->IMax)
			P->SumError = P->IMax;
		else if(P->SumError <= -P->IMax)
			P->SumError = -P->IMax;
		
		P->POut = P->P * P->PreError;
		P->IOut = P->I * P->SumError;
		P->DOut = P->D * P->dError;
		
		return LIMIT_MAX_MIN(P->POut+P->IOut+P->DOut,P->OutMax,-P->OutMax); 
}




/*********模糊pid部分*/

#define IS_Kp 1
#define IS_Ki 2
#define IS_Kd 3

#define stair 0.4
#define pstair 0.03
#define istair 0.0005
#define dstair 0.005
 
#define NL   -(3*stair)
#define NM	 -(2*stair)
#define NS	 -(1*stair)
#define ZE	 0
#define PS	 (1*stair)
#define PM	 (2*stair)
#define PL	 (3*stair)

#define NLp   -(3*pstair)
#define NMp	 -(2*pstair)
#define NSp	 -(1*pstair)
#define ZEp	 0
#define PSp	 (1*pstair)
#define PMp	 (2*pstair)
#define PLp	 (3*pstair)
 
#define NLi   -(3*istair)
#define NMi	 -(2*istair)
#define NSi	 -(1*istair)
#define ZEi	 0
#define PSi	 (1*istair)
#define PMi	 (2*istair)
#define PLi	 (3*istair) 

#define NLd   -(3*dstair)
#define NMd	 -(2*dstair)
#define NSd	 -(1*dstair)
#define ZEd	 0
#define PSd	 (1*dstair)
#define PMd	 (2*dstair)
#define PLd	 (3*dstair)
 
 
static const float fuzzyRuleKp[7][7]={
	PLp,	PLp,	PMp,	PMp,	PSp,	ZEp,	ZEp,
	PLp,	PLp,	PMp,	PSp,	PSp,	ZEp,	NSp,
	PMp,	PMp,	PMp,	PSp,	ZEp,	NSp,	NSp,
	PMp,	PMp,	PSp,	ZEp,	NSp,	NMp,	NMp,
	PSp,	PSp,	ZEp,	NSp,	NSp,	NMp,	NMp,
	PSp,	ZEp,	NSp,	NMp,	NMp,	NMp,	NLp,
	ZEp,	ZEp,	NMp,	NMp,	NMp,	NLp,	NLp
};
 
static const float fuzzyRuleKi[7][7]={
	NLi,	NLi,	NMi,	NMi,	NSi,	ZEi,	ZEi,
	NLi,	NLi,	NMi,	NSi,	NSi,	ZEi,	ZEi,
	NLi,	NLi,	NSi,	NSi,	ZEi,	PSi,	PSi,
	NMi,	NMi,	NSi,	ZEi,	PSi,	PMi,	PMi,
	NMi,	NMi,	ZEi,	PSi,	PSi,	PMi,	PLi,
	ZEi,	ZEi,	PSi,	PSi,	PMi,	PLi,	PLi,
	ZEi,	ZEi,	PSi,	PMi,	PMi,	PLi,	PLi
};
 
static const float fuzzyRuleKd[7][7]={
	PSd,	NSd,	NLd,	NLd,	NLd,	NMd,	PSd,
	PSd,	NSd,	NLd,	NMd,	NMd,	NSd,	ZEd,
	ZEd,	NSd,	NMd,	NMd,	NSd,	NSd,	ZEd,
	ZEd,	NSd,	NSd,	NSd,	NSd,	NSd,	ZEd,
	ZEd,	ZEd,	ZEd,	ZEd,	ZEd,	ZEd,	ZEd,
	PLd,	NSd,	PSd,	PSd,	PSd,	PSd,	PLd,
	PLd,	PMd,	PMd,	PMd,	PSd,	PSd,	PLd
};
 

 //关键算法
PID fuzzy(float e,float ec)
{

     float etemp,ectemp;
     float eLefttemp,ecLefttemp;
     float eRighttemp ,ecRighttemp;
 
     short eLeftIndex,ecLeftIndex;
     short eRightIndex,ecRightIndex;

     PID      fuzzy_PID;
    	//etemp = e > 3.0 ? 0.0 : (e < - 3.0 ? 0.0 : (e >= 0.0 ? (e >= 2.0 ? 2.5: (e >= 1.0 ? 1.5 : 0.5)) : (e >= -1.0 ? -0.5 : (e >= -2.0 ? -1.5 : (e >= -3.0 ? -2.5 : 0.0) ))));

		


     if(e>PL||e<NL)
			 etemp=0;//不做计算

		 if(PL>e&&e>PM)
			 etemp=PM;
		 else if(PM>e&&e>PS)
			 etemp=PS;
		 else if(PS>e&&e>ZE)
			 etemp=ZE;
		 else if(ZE>e&&e>NS)
			 etemp=NS;
		 else if(NS>e&&e>NM)
			 etemp=NM;
		 else if(NM>e&&e>NL)
			 etemp=NL;
 
		 //计算E隶属度
		 	//eRighttemp= etemp == 0.0 ? 0.0:((e-etemp)/stair);
		  //eLefttemp= etemp == 0.0 ? 0.0:(1- eRighttemp);
	    if( etemp == 0)
				eRighttemp=0;
			else
				eRighttemp=((e-etemp)/stair);
		 
		  if( etemp == 0)
				eLefttemp=0;
			else
				eLefttemp=(1- eRighttemp);
		 
		 


	eLeftIndex =(short) ((etemp-NL)/stair);       //例如 etemp=2.5，NL=-3，那么得到的序列号为5  【0 1 2 3 4 5 6】
	eRightIndex=(short) (eLeftIndex+1);
	   

		 
		 
		 if(ec>PL||ec<NL)
			 ectemp=0;
		 if(PL>ec&&ec>PM)
			 ectemp=PM;
		 else if(PM>ec&&ec>PS)
			 ectemp=PS;
		 else if(PS>ec&&ec>ZE)
			 ectemp=ZE;
		 else if(ZE>ec&&ec>NS)
			 ectemp=NS;
		 else if(NS>ec&&ec>NM)
			 ectemp=NM;
		 else if(NM>ec&&ec>NL)
			 ectemp=NL;
	  //计算EC隶属度
		 	//ecRighttemp= ectemp == 0.0 ? 0.0:((ec-ectemp)/stair);
	    //ecLefttemp = ectemp == 0.0 ? 0.0:(1- ecRighttemp);
	    if( ectemp == 0)
				ecRighttemp=0;
			else
				ecRighttemp=((ec-ectemp)/stair);
		 
		  if( ectemp == 0)
				ecLefttemp=0;
			else
				ecLefttemp=(1- ecRighttemp);
			
			
		 	ecLeftIndex =(short) ((ectemp-NL)/stair);  
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
PID OUT ;

float FuzzyPID_Calc(PID *pid)
{
	
	static float LastError;
 
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
		
		
	OUT = fuzzy(pid->PreError, pid->dError);      //模糊调整  kp,ki,kd   形参1当前误差，形参2前后误差的差值
  //return pid->POut+pid->IOut+pid->DOut;
	//return (pid->Kp+OUT.Kp)*pid->PreError + (pid->Kd+OUT.Kd)*pid->dError + (pid->Ki+OUT.Ki)*pid->SumError;
return (pid->Kp+OUT.Kp)*pid->PreError + (pid->Kd+OUT.Kd)*pid->dError + (pid->Ki)*pid->SumError; //p 和 d 先行
}