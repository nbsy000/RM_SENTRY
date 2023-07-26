#include "main.h"
#include "algo_HeatControl.h"
 
extern JudgeReceive_t JudgeReceive;		//裁判系统接收结构体 热量上限，热量冷却值，当前热量均从裁判系统接收；接收时判断并置位射击完成标志位

uint8_t IsShootAble_0=0,HeatUpdateFlag_0=0,ShootCpltFlag_0=0;
uint8_t IsShootAble_2=0,HeatUpdateFlag_2=0,ShootCpltFlag_2=0;


int32_t HeatMax17_0, HeatCool17_0 ;
int32_t HeatMax17_2, HeatCool17_2;
short BulletHeat17_0 = 10;
short BulletHeat17_2 = 10;

int32_t CurHeat17_0, LastHeat17_0, AvailableHeat17_0 ;
int32_t CurHeat17_2, LastHeat17_2, AvailableHeat17_2; //当前热量， 上一次热量, 自行计算热量

int32_t Shooted17Cnt_0 ;
int32_t Shooted17Cnt_2;	//一周期内已打出子弹数
int32_t AvailableBullet17_0 ;
int32_t AvailableBullet17_2;	//下一周期允许打弹数

uint8_t ShootAbleFlag_0;
uint8_t ShootAbleFlag_2;

/**********************************************************************************************************
*函 数 名: HeatControl
*功能说明: 热量控制
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
//上云台： 4 4 0.55f
//下云台： 10 10 0.5f

void HeatControl_0(void)
{
	if(HeatUpdateFlag_0 == 1)	//热量更新
	{
		Shooted17Cnt_0 = 0;
		AvailableHeat17_0 = HeatMax17_0 - CurHeat17_0 + HeatCool17_0;
		if(ShootCpltFlag_0 == 1)	//检测到发弹。为热量更新后打出的子弹
		{
			AvailableHeat17_0 -= BulletHeat17_0;	
			ShootCpltFlag_0 = 0;	//已处理完本次发弹
		}
		AvailableBullet17_0 = AvailableHeat17_0 / BulletHeat17_0;
		ShootAbleFlag_0 = (AvailableBullet17_0 < 9)?0:1;		
	}	
	
	else if((ShootCpltFlag_0 == 1) && (HeatUpdateFlag_0 == 0))	//热量没有更新，但检测到发弹
	{
		ShootCpltFlag_0 = 0;		//已处理完本次发弹
		Shooted17Cnt_0++;		//发射了一发子弹
		ShootAbleFlag_0 = (Shooted17Cnt_0 >= AvailableBullet17_0-9)?0:1;		
	}
}

void HeatControl_2(void)
{
	if(HeatUpdateFlag_2 == 1)	//热量更新
	{
		Shooted17Cnt_2 = 0;
		AvailableHeat17_2 = HeatMax17_2 - CurHeat17_2 + HeatCool17_2;
		if(ShootCpltFlag_2 == 1)	//检测到发弹。为热量更新后打出的子弹
		{
			AvailableHeat17_2 -= BulletHeat17_2;	
			ShootCpltFlag_2 = 0;	//已处理完本次发弹
		}
		AvailableBullet17_2 = AvailableHeat17_2 / BulletHeat17_2;
		ShootAbleFlag_2 = (AvailableBullet17_2 < 9)?0:1;		
	}	
	
	else if((ShootCpltFlag_2 == 1) && (HeatUpdateFlag_2 == 0))	//热量没有更新，但检测到发弹
	{
		ShootCpltFlag_2 = 0;		//已处理完本次发弹
		Shooted17Cnt_2++;		//发射了一发子弹
		ShootAbleFlag_2 = (Shooted17Cnt_2 >= AvailableBullet17_2-9)?0:1;		
	}
}

/**********************************************************************************************************
*函 数 名: HeatUpdate
*功能说明: 热量更新
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
float HeatControlThreshold_0 = 0.7f;   	//开启热量控制的阈值
float HeatControlThreshold_2 = 0.7f;   	//开启热量控制的阈值

void HeatUpdate_0(void)
{
    HeatMax17_0 = JudgeReceive.HeatMax17_0;
    HeatCool17_0 = (JudgeReceive.HeatCool17_0) / 10;//接收频率10Hz，表示单次接收之间的冷却
    CurHeat17_0 = JudgeReceive.shooterHeat17_0;
	
	if(CurHeat17_0 != LastHeat17_0)//
	{
		HeatUpdateFlag_0 = 1;
		ShootCpltFlag_0 = 0;			//热量更新则将发射标志位清零(没有待处理的打弹)
	}
	
	if(CurHeat17_0 < HeatControlThreshold_0*HeatMax17_0)
	{
		ShootAbleFlag_0 = 1;
		ShootCpltFlag_0 = 0;
	}
	else
	{
		if((ShootCpltFlag_0 == 1) || (HeatUpdateFlag_0 == 1))
		HeatControl_0();
	}
	
	HeatUpdateFlag_0 = 0;		//已处理完本次热量更新
	LastHeat17_0 = CurHeat17_0;
	IsShootAble_0 = ShootAbleFlag_0;
}


void HeatUpdate_2(void)
{
    HeatMax17_2 = JudgeReceive.HeatMax17_2;
    HeatCool17_2 = (JudgeReceive.HeatCool17_2) / 10;
    CurHeat17_2 = JudgeReceive.shooterHeat17_2;
	
	if(CurHeat17_2 != LastHeat17_2)
	{
		HeatUpdateFlag_2 = 1;
		ShootCpltFlag_2 = 0;			//热量更新则将发射标志位清零(没有待处理的打弹)
	}
	
	if(CurHeat17_2 < HeatControlThreshold_2*HeatMax17_2)
	{
		ShootAbleFlag_2 = 1;
		ShootCpltFlag_2 = 0;
	}
	else
	{
		if((ShootCpltFlag_2 == 1) || (HeatUpdateFlag_2 == 1))
		HeatControl_2();
	}
	
	HeatUpdateFlag_2 = 0;		//已处理完本次热量更新
	LastHeat17_2 = CurHeat17_2;
	IsShootAble_2 = ShootAbleFlag_2;
}

