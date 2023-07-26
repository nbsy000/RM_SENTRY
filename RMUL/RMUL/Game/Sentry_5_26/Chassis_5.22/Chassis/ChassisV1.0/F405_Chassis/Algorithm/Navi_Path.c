#include "main.h"

PathInfoTypedef PathInforms[PATH_NUM];//路径
Pid_Typedef TrajectCorrectPID;//导航PID
uint16_t PathDotIndex = 0;//路线选取

int count = 0;
/**********************************************************************************************************
*函 数 名: PathInit
*功能说明: 路径初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void PathInit()
{		
		PathInforms[0].PathDotsInfoArray = NULL;
		PathInforms[0].PathDotsNum = 0;
		PathInforms[1].PathDotsInfoArray = path0;//去前哨站路径
		PathInforms[1].PathDotsNum = PATH_MAX0;//路径点数
		PathInforms[2].PathDotsInfoArray = path1;//回巡逻区路径
		PathInforms[2].PathDotsNum = PATH_MAX1;//路径点数
		PathInforms[3].PathDotsInfoArray = path2;//测试路径1
		PathInforms[3].PathDotsNum = PATH_MAX2;//路径点数
		PathInforms[4].PathDotsInfoArray = path3;//测试路径2
		PathInforms[4].PathDotsNum = PATH_MAX3;//路径点数
		PathInforms[5].PathDotsInfoArray = path4;//测试路径3
		PathInforms[5].PathDotsNum = PATH_MAX4;//路径点数
		PathInforms[6].PathDotsInfoArray = path5;
		PathInforms[6].PathDotsNum = PATH_MAX5;//路径点数
		PathInforms[7].PathDotsInfoArray = path6;
		PathInforms[7].PathDotsNum = PATH_MAX6;//路径点数
	
		//pid参数初始化
		TrajectCorrectPID.P = 3.0f;
		TrajectCorrectPID.D = 0.0f;
		TrajectCorrectPID.OutMax = 1000.0f;
	
		PathDotIndex = 1;//路线当前点数
		
}


/**********************************************************************************************************
*函 数 名: chassisNavigate
*功能说明: 进行导肮
*形    参: ChassisInfo 底盘类结构体  PathInfo 需要进行导航的数据
*返 回 值: 无
**********************************************************************************************************/
float navSpeed_Limit = 2000.0f; 
void chassisNavigate(ChassisState_t *ChassisInfo, PathInfoTypedef *PathInfo)
{
    float dx, dy;
    uint16_t PathDotIndexMAX;
    const NavigationPoints *PathDotsInfo = PathInfo->PathDotsInfoArray;

    volatile ChassisNavTypedef *pCurrentState = &(ChassisInfo->CurrentState);
    volatile ChassisNavTypedef *pAimState = &(ChassisInfo->AimState);
					
    // Navigate dots change count in one cycle
    uint8_t ChangeNavigateDotCount = 0;
    float NavigateVx, NavigateVy; //x,y方向分速度
    static u8 last_num = 0;     //上一次的路径好
	
    if (last_num != ChassisInfo->NavigatePathNum)
    {
        //第一次last_num=0;PathDotIndex赋值为1;
        //之后last_num =ChassisInfo->NavigatePathNum
        PathDotIndex = 1;
    }

    PathDotIndexMAX = PathInfo->PathDotsNum - 1;

    pAimState->X = PathDotsInfo[PathDotIndex].x;//x方向的目标值
    pAimState->Y = PathDotsInfo[PathDotIndex].y;//y方向的目标值
    pAimState->Alpha = PathDotsInfo[PathDotIndex].alpha; //偏航

    dx = PathDotsInfo[PathDotIndex].x - pCurrentState->X;//计算与目标方向的偏差值
    dy = PathDotsInfo[PathDotIndex].y - pCurrentState->Y;

		//速度限制 mm/s
    if (PathDotsInfo[PathDotIndex - 1].v > navSpeed_Limit)
    {
        NavigateVx = navSpeed_Limit * arm_cos_f32(PathDotsInfo[PathDotIndex - 1].V_Theta);
        NavigateVy = navSpeed_Limit * arm_sin_f32(PathDotsInfo[PathDotIndex - 1].V_Theta);
    }
    else
    {
        NavigateVx = PathDotsInfo[PathDotIndex - 1].v * arm_cos_f32(PathDotsInfo[PathDotIndex - 1].V_Theta);
        NavigateVy = PathDotsInfo[PathDotIndex - 1].v * arm_sin_f32(PathDotsInfo[PathDotIndex - 1].V_Theta);
    }

		//如果差值向量与原路径向量角度为钝角，前移路径点
    while (dx * NavigateVx + dy * NavigateVy < 0.0f && PathDotIndex < PathDotIndexMAX) //速度矢量和指向下一点矢量夹角判断是否过，更新路径点，
    {
        ChangeNavigateDotCount++;
        PathDotIndex++;
        dx = PathDotsInfo[PathDotIndex].x - pCurrentState->X;
        dy = PathDotsInfo[PathDotIndex].y - pCurrentState->Y;
        if (PathDotsInfo[PathDotIndex - 1].v > navSpeed_Limit)
        {
            NavigateVx = navSpeed_Limit * arm_cos_f32(PathDotsInfo[PathDotIndex - 1].V_Theta);
            NavigateVy = navSpeed_Limit * arm_sin_f32(PathDotsInfo[PathDotIndex - 1].V_Theta);
        }
        else
        {
            NavigateVx = PathDotsInfo[PathDotIndex - 1].v * arm_cos_f32(PathDotsInfo[PathDotIndex - 1].V_Theta);
            NavigateVy = PathDotsInfo[PathDotIndex - 1].v * arm_sin_f32(PathDotsInfo[PathDotIndex - 1].V_Theta);
        }
    }
    //Change Navigate Dot
    if (ChangeNavigateDotCount > 0) // if ChangeNavigateDotCount is more than two maybe a hazard. 超过两次可能是有问题了  有点不理解，总是会更新多次的
    {
        pAimState->X = PathDotsInfo[PathDotIndex].x;
        pAimState->Y = PathDotsInfo[PathDotIndex].y;
    }

    //Multi dot navigate
    if (PathDotIndex < PathDotIndexMAX) //navigate regulate
    {
        float foreNavigateX = PathDotsInfo[PathDotIndex - 1].x;
        float foreNavigateY = PathDotsInfo[PathDotIndex - 1].y;
        float postNavigateX = PathDotsInfo[PathDotIndex].x;
        float postNavigateY = PathDotsInfo[PathDotIndex].y;
        float TrajectoryDis;
        float NavigateIntervalX, NavigateIntervalY, NavigateIntervalDis;
        float aimVt, aimVtX, aimVtY;
        //Trajectory deviation correct
        NavigateIntervalX = postNavigateX - foreNavigateX;
        NavigateIntervalY = postNavigateY - foreNavigateY;
        NavigateIntervalDis = sqrt(NavigateIntervalX * NavigateIntervalX + NavigateIntervalY * NavigateIntervalY); //两点距离 论文里的|AB|
        //distance of the two parallel line,using vector product??fh:not this
        //NOte:the result is signed,notes the reference original
        TrajectoryDis = -((pCurrentState->X - foreNavigateX) * NavigateIntervalY - (pCurrentState->Y - foreNavigateY) * NavigateIntervalX)
						/ NavigateIntervalDis; //论文里的|A'O|

        //tangential velocity compute
				TrajectCorrectPID.SetPoint = 0.0f;
        aimVt = PID_Calc(&TrajectCorrectPID, TrajectoryDis);

        //偏航校正的速度校正，三全向轮直接分解到x，y方向
        //the x  component aimVt aimVtX : aimVtX = aimVt * cos(V_Theta + pi/2)
        aimVtX = -aimVt * NavigateIntervalY / NavigateIntervalDis;

        //the y  component aimVt aimVtY : aimVtY = aimVt * sin(V_Theta + pi/2)
        aimVtY = +aimVt * NavigateIntervalX / NavigateIntervalDis;
        pAimState->Vx = aimVtX + NavigateVx;
        pAimState->Vy = aimVtY + NavigateVy;

        last_num = ChassisInfo->NavigatePathNum; //不断的存上一个的路径号，用于上面的判断是否开新的路径号
    }
    else //the last dot 目前在执行到这一点后直接停止
    {
        pAimState->X=PathDotsInfo[PathDotIndexMAX].x;
        pAimState->Y=PathDotsInfo[PathDotIndexMAX].y;
				pAimState->Alpha=PathDotsInfo[PathDotIndexMAX].alpha;
				
        //PathDotIndex = 1;
				pAimState->Vx = 0;
				pAimState->Vy = 0;
				chassis.NAV_State = FINISHED;//状态更新
		//底盘切状态！！！！
		
    }
}






