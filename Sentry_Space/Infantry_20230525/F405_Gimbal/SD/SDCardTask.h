#ifndef __SDCARDTASK_H
#define __SDCARDTASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "ff.h"
#include <stdio.h>
#include <string.h>
#include "bsp_spi_sdcard.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} TestStatus;

/* Private define ------------------------------------------------------------*/
#define BLOCK_SIZE 512      /* Block Size in Bytes */
#define NUMBER_OF_BLOCKS 10 /* For Multi Blocks operation (Read/Write) */
#define MULTI_BUFFER_SIZE (BLOCK_SIZE * NUMBER_OF_BLOCKS)

typedef struct SDStatus
{
    uint8_t SD_init_result;     // SD卡初始化
    uint8_t SD_FS_Write_result; // 写结果
    uint8_t SD_FS_Read_result;  // 读结果
    uint8_t SD_FS_Open_result;  // 打开文件结果
    uint8_t SD_FS_Mount_result; // 挂载文件系统结果
    uint8_t SDCard_task_init;   // SD卡任务成功初始化结果

    SD_Error Status;
    FATFS fs;
    FIL fnew;
    FRESULT res_sd; /* 文件操作结果 */
    UINT fnum;      /* 文件成功读写数量 */

    int32_t count;
} SDStatus;

void SDCard_task(void *pvParameters);

#endif
