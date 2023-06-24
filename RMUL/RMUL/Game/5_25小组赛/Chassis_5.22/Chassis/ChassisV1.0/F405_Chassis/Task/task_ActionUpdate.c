#include "main.h"
#include "task_ActionUpdate.h"

State_t Sentry_State;
Debug_Mode_Type Debug_Mode;

static void update_state_with_RCdata(void);
static void sentry_state_reflect(uint8_t gimbal_r_mode,uint8_t gimbal_l_mode,
                                 uint8_t shoot_r_mode,uint8_t shoot_l_mode,
                                 uint8_t chassis_mode,uint8_t chassisyaw_mode); //reflect当作做一个映射的意思

/**
 * @brief 状态机本体，执行每次的状态刷新
 * @param 无
 * @retval 无
 */
void task_ActionUpdate(void)
{
    RC_Rst(); //刚进任务时，执行一次清零
		Rst_Debug();
    Sentry_State.Chassis_Mode = Chassis_SLEEP;
    while (1)
    {
        update_state_with_RCdata(); //从遥控器更新当前状态
				vTaskDelay(10);//遥控器数据是14ms发送一次
    }
}

/**
  * @brief  用遥控器信息更新工作状态
  * @param  None
  * @retval None
  */
static void update_state_with_RCdata(void)
{

    switch (RC_Ctl.rc.s1)
    {
    case 1:
        switch (RC_Ctl.rc.s2)
        {
        case 1: //自动移动
            sentry_state_reflect(Gimbal_R_PC,
                                 Gimbal_L_PC,
                                 Shoot_R_PC,
                                 Shoot_L_PC,
                                 Chassis_Patrol,
																 ChassisYaw_PC);
            break;
        case 2:
            sentry_state_reflect(Gimbal_R_SLEEP,
                                 Gimbal_L_SLEEP,
                                 Shoot_R_SLEEP,
                                 Shoot_L_SLEEP,
                                 Chassis_Patrol,
																 ChassisYaw_SLEEP);
            break;
        case 3: //挂在短滑轨上测辅瞄的模式
            sentry_state_reflect(Gimbal_R_PC,
                                 Gimbal_L_PC,
                                 Shoot_R_SLEEP,
                                 Shoot_L_SLEEP, 
                                 Chassis_SLEEP,
																 ChassisYaw_PC);
            break;
        default:
            break;
        }
        break;
    case 2:
        switch (RC_Ctl.rc.s2)
        {
        case 1: //s1 下 s2 上  底盘掉电
            sentry_state_reflect(Gimbal_R_SLEEP,
                                 Gimbal_L_SLEEP,
                                 Shoot_R_SLEEP,
                                 Shoot_L_SLEEP,
                                 Chassis_RC,
																 ChassisYaw_RC);
            break;
        case 2: //掉电模式
            sentry_state_reflect(Gimbal_R_SLEEP,
                                 Gimbal_L_SLEEP,
                                 Shoot_R_SLEEP,
                                 Shoot_L_SLEEP,
                                 Chassis_SLEEP,
																 ChassisYaw_SLEEP);
            break;
        case 3: //下云台检录打子弹
            sentry_state_reflect(Gimbal_R_SLEEP,
                                 Gimbal_L_RC,
                                 Shoot_R_SLEEP,
                                 Shoot_L_SLEEP,
                                 Chassis_SLEEP,
																 ChassisYaw_SLEEP);
            break;
        default:
            break;
        }
        break;
    case 3:
        switch (RC_Ctl.rc.s2)
        {
        case 1: //s1 中  s2 上  遥控器控制
            sentry_state_reflect(Gimbal_R_SLEEP,
                                 Gimbal_L_SLEEP,
                                 Shoot_R_SLEEP,
                                 Shoot_L_SLEEP,
                                 Chassis_Protect,
																 ChassisYaw_SLEEP);
            break;
        case 2: //下云台遥控器调试
            sentry_state_reflect(Gimbal_R_SLEEP,
                                 Gimbal_L_SLEEP,
                                 Shoot_R_SLEEP,
                                 Shoot_L_SLEEP,
                                 Chassis_SLEEP,
																 ChassisYaw_PC);
            break;
        case 3: //底盘检录模式，底盘运动
            sentry_state_reflect(Gimbal_R_RC,
                                 Gimbal_L_SLEEP,
                                 Shoot_R_SLEEP,
                                 Shoot_L_SLEEP,
                                 Chassis_SLEEP,
																 ChassisYaw_SLEEP);
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

static void sentry_state_reflect(uint8_t gimbal_r_mode,
                                 uint8_t gimbal_l_mode,
                                 uint8_t shoot_r_mode,
                                 uint8_t shoot_l_mode,
                                 uint8_t chassis_mode,
																 uint8_t chassisyaw_mode)
{
    Sentry_State.Gimbal_R_Mode = gimbal_r_mode;
    Sentry_State.Gimbal_L_Mode = gimbal_l_mode;
    Sentry_State.Shoot_R_Mode = shoot_r_mode;
    Sentry_State.Shoot_L_Mode = shoot_l_mode;
    Sentry_State.Chassis_Mode = chassis_mode;
	  Sentry_State.ChassisYaw_Mode = chassisyaw_mode;
	
//	  if((RC_Ctl.rc.s1!=2)||(RC_Ctl.rc.s2!=2))//最高掉电权授予2,2,防止自己在debug的时候出现智障操作让车疯了，注释即可不用遥控器
//		{
			if(Debug_Mode.Chassis_Debug)
				Sentry_State.Chassis_Mode	= Chassis_DEBUG;
			if(Debug_Mode.ChassisYaw_Debug)
				Sentry_State.ChassisYaw_Mode	= ChassisYaw_DEBUG;
//		}		
}

/**
  * @brief  debug_mode 变量初始化
  * @param  None
  * @retval None
  */
void Rst_Debug()
{
	Debug_Mode.ChassisYaw_Debug = 0;
	Debug_Mode.Chassis_Debug = 0;
}