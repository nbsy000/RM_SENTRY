#ifndef __JUDGETASK_H
#define __JUDGETASK_H


#define frameheader_len  5       //帧头长度
#define cmd_len          2       //命令码长度
#define crc_len          2       //CRC16校验

typedef __packed struct
{
 uint8_t intention;
 uint16_t start_position_x;
 uint16_t start_position_y;
 int8_t delta_x[49];
 int8_t delta_y[49];
}map_sentry_data_t;

void JudgeReceive_task(void );
void referee_data_pack_handle(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len);
void Send_Path_Judge(void);

#endif
