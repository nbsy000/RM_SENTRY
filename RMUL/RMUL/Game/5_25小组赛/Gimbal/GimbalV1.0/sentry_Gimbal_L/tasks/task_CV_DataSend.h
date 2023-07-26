#ifndef __TASK_CV_DATA_SEND_H__
#define __TASK_CV_DATA_SEND_H__

/*   发送数据定义 */

#pragma pack(push, 1)     //不进行字节对齐
typedef struct PCSendData //数据顺序不能变,注意32字节对齐
{
    int8_t start_flag;
    uint8_t robot_color : 1;
    uint8_t shoot_level : 2;
    uint8_t mode : 2;
    uint8_t which_balance : 2;
    uint8_t change_priority_flag : 1;
    uint8_t frame_id;
    short pitch;
    float yaw;
    int16_t crc16;
} PCSendData;

typedef struct PCRecvData
{
    int8_t start_flag;
    uint8_t enemy_id : 3;   //敌方ID，如果是0的话不击打,云台也不动
    uint8_t shoot_flag : 1; //打击标志位
    uint8_t mode : 2;       //上位机当前所处模式，将与下位机的模式进行检验
    uint8_t _ : 2;
    uint8_t frame_id;
    short pitch;
    float yaw;
    int16_t crc16;
} PCRecvData;

#pragma pack(pop) //不进行字节对齐



void task_CV_DataSend(void *pvParameters);
void SendtoPC(void);

extern PCRecvData pc_recv_data;
extern PCSendData pc_send_data;

#endif //__TASK_CV_DATA_SEND_H__
