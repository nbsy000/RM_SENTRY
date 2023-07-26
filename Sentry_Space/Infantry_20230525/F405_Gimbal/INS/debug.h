#ifndef _DEBUG_H
#define _DEBUG_H

#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TRUE 1
#define FALSE 0

//#include "tools.h"

/* whether use debug mode or not */
 #define DEBUG_MODE
#define JSCOPE_RTT_MODE

/*  不同的任务设置一个开关，方便快速开关切换   */

#ifdef DEBUG_MODE

#include "SEGGER_RTT.h"
#include <stdio.h>

// RTT
#define PRINTF(str, ...)       \
    SEGGER_RTT_SetTerminal(0); \
    SEGGER_RTT_printf(0, str, ##__VA_ARGS__) //字符串发送
#define PRINTF_F(str_temp, str, format) \
    sprintf(str_temp, str, format);     \
    SEGGER_RTT_SetTerminal(0);          \
    SEGGER_RTT_printf(0, str_temp); //带浮点数的发送

#define LOG_PROTO(window, type, color, format, ...)     \
    SEGGER_RTT_printf(window, "  %s%s" format "\r\n%s", \
                      color,                            \
                      type,                             \
                      ##__VA_ARGS__,                    \
                      RTT_CTRL_RESET)

/* 清屏*/
#define LOG_CLEAR() SEGGER_RTT_WriteString(0, "  " RTT_CTRL_CLEAR)

/* 无颜色日志输出 */
#define LOG(format, ...) LOG_PROTO(0, "", "", format, ##__VA_ARGS__)

/* 有颜色格式日志输出 */
#define LOG_INFO(format, ...) LOG_PROTO(0, "[INFO]: ", RTT_CTRL_TEXT_BRIGHT_GREEN, format, ##__VA_ARGS__)
#define LOG_WARNING(format, ...) LOG_PROTO(0, "[WARNING]: ", RTT_CTRL_TEXT_BRIGHT_YELLOW, format, ##__VA_ARGS__)
#define LOG_ERROR(format, ...) LOG_PROTO(0, "[ERROR]: ", RTT_CTRL_TEXT_BRIGHT_RED, format, ##__VA_ARGS__)
#define LOG_FATAL(format, ...) LOG_PROTO(0, "[FATAL]: ", RTT_CTRL_TEXT_BRIGHT_BLUE, format, ##__VA_ARGS__)

// JSCOPE RTT MODE
#ifdef JSCOPE_RTT_MODE

#pragma pack(push, 1)

typedef struct
{
    float set_point; //目标

    float raw_point; //原数据

} acValBuffer;

#pragma pack(pop)

void JscopeRTTInit(void);
void JscopeWrite(float setPoint, float samplePoint);

#endif // JSCOPE_RTT_MODE

#endif

// USER DEFINED
typedef struct IMU_Debugger
{
    uint16_t recv_msgs_num[2]; // GYRO + ACCE

    /*  接收帧率计算定义 */
    uint32_t last_can_cnt[2];
    float can_dt[2];

    uint16_t loss_num[2];
} IMU_Debugger;
typedef struct Friction_Debugger
{
    uint16_t recv_msgs_num[2];

    /*  接收帧率计算定义 */
    uint32_t last_can_cnt[2];
    float can_dt[2];

    uint16_t loss_num[2];
} Friction_Debugger;
typedef struct Gimbal_Debugger
{
    uint16_t recv_msgs_num;

    /*  接收帧率计算定义 */
    uint32_t last_can_cnt;
    float can_dt;

    uint16_t loss_num;
} Gimbal_Debugger;
typedef struct RobotDebugger
{
    uint32_t last_cnt;
    float dt;
} RobotDebugger;

typedef struct RemoteDebugger
{
    uint32_t last_cnt;
    float dt;
    uint16_t recv_msgs_num;
} RemoteDebugger;
typedef struct GlobalDebugger
{
    IMU_Debugger imu_debugger;
    RobotDebugger robot_debugger;
    RemoteDebugger remote_debugger; //遥控器
    Gimbal_Debugger gimbal_debugger;
    Friction_Debugger friction_debugger;
} GlobalDebugger;

extern GlobalDebugger global_debugger;
extern float Ozone[8];

int8_t checkIMUOn(void);
#endif
