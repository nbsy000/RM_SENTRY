#include "main.h"
#include "task_Gimbal_L.h"

gimbal_motor_t MotoPitch, MotoYaw,MotoPitch_L,MotoYaw_L;
float RCPitch_Scal = 0.00005f, RCYaw_Scal = 0.00004f; //遥控器对两轴控制的放大比率

int32_t PITCH_MAX_ANGLE, PITCH_MIN_ANGLE, PITCH_ZERO_POS; //pitch限幅度参数
int32_t YAW_MAX_ANGLE, YAW_MIN_ANGLE, YAW_ZERO_POS;       //yaw限幅度参数

float Pitch_Actual, Yaw_Actual;
float PC_PitchSet, PC_YawSet;

Gimbal_Typedef Gimbal_L;

extern int16_t Gimbal_init_flag;
extern gyro_Typedef Gyro_Left;
extern State_t Sentry_State;
extern uint8_t aim_flag;
extern uint8_t distance;
extern uint8_t is_game_start;
//用于切换mode时，pitch和yaw不复位的
uint8_t GimbalUp_ModeUpdate_Flag = 0;                 //标志上次模式和当前模式是否一致
volatile uint8_t GimbalL_LastMode = Gimbal_L_SLEEP; //存放上次进入Gimbal_Dn_task时的状态

//******************内部函数声明***********************************************//
static float CombPitchOutput(uint8_t Gimbal_Type); //获取滤波后的pitch角度的函数
static float CombYawOutput(uint8_t Gimbal_Type);   //获取滤波后的yaw角度的函数
static void Gimbal_Limit(void);//云台角度限幅函数
static void Gimbal_GYRO_Cal(void);

/**
 * @brief 云台任务主体
 * @param 无
 * @retval 无
 */
void task_Gimbal_L(void *parameter)
{
    //进入任务时执行一次复位
    Gimbal_Limit();    //限幅校正
    while (1)
    {
				//云台姿态结算
				Gimbal_GYRO_Cal();
			
				Gimbal_Gyro_Send();
			
				//获取姿态
				MotoYaw_L.actualAngle = CombYawOutput(GIMBAL_LEFT);
				MotoPitch_L.actualAngle = CombPitchOutput(GIMBAL_LEFT);
        vTaskDelay(1);
    }
}



float moto_pitch, moto_yaw, moto_pitch_init, moto_yaw_init;
float gyro_pitch, gyro_yaw, gyro_pitch_init, gyro_yaw_init;
float comb_pitch, comb_yaw;
float moto_pitch_l, moto_yaw_l, moto_pitch_l_init, moto_yaw_l_init;
float gyro_pitch_l, gyro_yaw_l, gyro_pitch_l_init, gyro_yaw_l_init;
float comb_pitch_l, comb_yaw_l;
float k_pitch = 0;
float k_yaw = 0;
int8_t init_comb_flag = 1;
/**
  * @brief  上云台姿态输出计算（陀螺仪值和电机值的互补滤波）
  * @param  None
  * @retval None
  */
static void Gimbal_GYRO_Cal(void)
{
    if (init_comb_flag)
    {
				//主云台
        moto_pitch_init = 1950.0f;
        gyro_pitch_init = GyroPitchOutPut(GIMBAL_RIGHT);
        //左 3102- 右8495
        moto_yaw_init = 1250.0f;
        gyro_yaw_init = GyroYawOutPut(GIMBAL_RIGHT);
			
				//左云台
				moto_pitch_l_init = 4762.0f;
        gyro_pitch_l_init = GyroPitchOutPut(GIMBAL_LEFT);
        //左 3102- 右8495
        moto_yaw_l_init = 7610.0f;
        gyro_yaw_l_init = GyroYawOutPut(GIMBAL_LEFT);
        init_comb_flag = 0;		
    }


		//实际上目前只用电机角
    moto_pitch = (PitchAngleOutPut(GIMBAL_RIGHT) - moto_pitch_init) / 8192.0f * 360.0f;
    gyro_pitch = (GyroPitchOutPut(GIMBAL_RIGHT) - gyro_pitch_init);
    comb_pitch = k_pitch * gyro_pitch + (1 - k_pitch) * moto_pitch; //一阶互补滤波

    moto_yaw = (YawAngleOutPut(GIMBAL_RIGHT) - moto_yaw_init) / 8192 * 360.0f;
    gyro_yaw = (GyroYawOutPut(GIMBAL_RIGHT) - gyro_yaw_init);
    comb_yaw = k_yaw * gyro_yaw + (1 - k_yaw) * moto_yaw; //一阶互补滤波
		
		//左云台
		moto_pitch_l = (PitchAngleOutPut(GIMBAL_LEFT) - moto_pitch_l_init) / 8192.0f * 360.0f;
    gyro_pitch_l = (GyroPitchOutPut(GIMBAL_LEFT) - gyro_pitch_l_init);
    comb_pitch_l = k_pitch * gyro_pitch_l + (1 - k_pitch) * moto_pitch_l; //一阶互补滤波

    moto_yaw_l = (YawAngleOutPut(GIMBAL_LEFT) - moto_yaw_l_init) / 8192 * 360.0f;
    gyro_yaw_l = (GyroYawOutPut(GIMBAL_LEFT) - gyro_yaw_l_init);
    comb_yaw_l = k_yaw * gyro_yaw_l + (1 - k_yaw) * moto_yaw_l; //一阶互补滤波
}

/**
  * @brief  上云台旋转角度限位
  * @param  None
  * @retval None
  */
void Gimbal_Limit(void)
{


		//主云台  pitch 2222下-1620上   中：2030 
		MotoPitch.MAX_ANGLE = +(2200.0f-1950)/8192.0f*360.0f; 
		MotoPitch.ZERO_POS = 0.0f;
		MotoPitch.MIN_ANGLE =-(1950.0f-1680)/8192.0f*360.0f;
		//面朝云台  左 633- 右2000   1100
    MotoYaw.MAX_ANGLE = (2000.0f-1250)/8192.0f*360.0f;
    MotoYaw.ZERO_POS = 0.0f;
    MotoYaw.MIN_ANGLE = -(1250.0f-640)/8192.0f*360.0f;
		//巡逻的范围一般比限位的范围要窄，因为相机的广角足够宽
		//2400     1980
    MotoPitch.PATROL_MAX_ANGLE =+(2100.0f-1950)/8192.0f*360.0f;;
    MotoPitch.PATROL_ZERO_POS = +0;
    MotoPitch.PATROL_MIN_ANGLE =-(1950.0f-1800)/8192.0f*360.0f;;

		//860   1640   1100
    MotoYaw.PATROL_MAX_ANGLE = (1640.0f-1250)/8192.0f*360.0f;
    MotoYaw.PATROL_ZERO_POS =  0;
    MotoYaw.PATROL_MIN_ANGLE =-(1250.0f-860)/8192.0f*360.0f;
		
		//左云台 5040  4500 中4762
		MotoPitch_L.MAX_ANGLE = +(5040.0f-4762)/8192.0f*360.0f; 
		MotoPitch_L.ZERO_POS = 0.0f;
		MotoPitch_L.MIN_ANGLE =-(4762.0f-4500)/8192.0f*360.0f;
		//左 8240  6900   7780
		MotoYaw_L.MAX_ANGLE = (8240.0f-7610)/8192.0f*360.0f;
    MotoYaw_L.ZERO_POS = 0.0f;
    MotoYaw_L.MIN_ANGLE = -(7610.0f-6900)/8192.0f*360.0f;
		//巡逻的范围一般比限位的范围要窄，因为相机的广角足够宽
		//1900
    MotoPitch_L.PATROL_MAX_ANGLE =+(4912.0f-4762)/8192.0f*360.0f;
    MotoPitch_L.PATROL_ZERO_POS = +0;
    MotoPitch_L.PATROL_MIN_ANGLE =-(4762.0f-4612)/8192.0f*360.0f;

		//4169  7068
    MotoYaw_L.PATROL_MAX_ANGLE = (8000.0f-7610)/8192.0f*360.0f;
    MotoYaw_L.PATROL_ZERO_POS =  0;
    MotoYaw_L.PATROL_MIN_ANGLE =-(7610.0f-7220)/8192.0f*360.0f;
}
/**
  * @brief  获取滤波后的云台两轴姿态（pitch和yaw）
  * @param  None
  * @retval pitch和yaw的角度
  */
static float CombPitchOutput(uint8_t Gimbal_Type)
{
	if(Gimbal_Type == GIMBAL_RIGHT)
    return comb_pitch;
	else
		return comb_pitch_l;
}
static float CombYawOutput(uint8_t Gimbal_Type)
{
	if(Gimbal_Type == GIMBAL_RIGHT)
    return comb_yaw;
	else
		return comb_yaw_l;
}

