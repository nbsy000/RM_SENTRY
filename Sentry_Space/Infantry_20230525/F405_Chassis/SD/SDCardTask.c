/**********************************************************************************************************
 * @文件     SDCardTask.c
 * @说明     读写SD卡，记录日志
 * @版本  	 V1.0
 * @作者     段康晟
 * @日期     2022.7
 **********************************************************************************************************/
#include "main.h"
#include "stdlib.h"

char WriteBuffer[128] = "";              /* 写缓冲区*/
TCHAR DataFile[] = "0:infantry.csv"; //文件名
SDStatus sd_status;
extern uint8_t Anomalies_tag;

/*这里放记录的变量*/
extern JudgeReceive_t JudgeReceive;
extern F405_typedef F405;
extern INA260 INA260_1;
extern float AD_actual_value;
extern ChassisSpeedCal_t RealCarSpeed;
int test = 1;

/*时间戳*/
float Second = 0.0f;
uint32_t Recorder,Minute,Hour = 0;
char TimeBuffer[64];
void TimeRecord()
{
    Second += GetDeltaT(&Recorder);
    if(Second >= 60.0f && Minute >= 59)
    {
        Hour += 1;
        Minute = 0;
        Second = 0.0f;
    }
    else if(Second >= 60.0f)
    {
        Minute += 1;
        Second = 0.0f;
    }
    sprintf(TimeBuffer,"%d:%02d:%5.2f\n",Hour,Minute,Second);
}


float temp1 = 2.0, temp2 = 3.0;
void SDLOG(enum SDWRITE write_type, const char *str)
{
    memset(WriteBuffer, '\0', sizeof(WriteBuffer)); //清空缓存区
    if (SD_START == write_type)
    {
        sprintf(WriteBuffer, "Spin,Mode,Debug,CapV,RealPower,Remain,PMax,SpeedXSet,SpeedX,SpeedYSet,SpeedY,SpeedWSet,SpeedW,Time\n");
    }
    else if (SD_WARNING == write_type)
    {
        sprintf(WriteBuffer, "[WARNING]:  ");
        sprintf(WriteBuffer + 13, str);
    }
    else if (SD_INFO == write_type)
    {
        sprintf(WriteBuffer + 10, str);
    }
	else if (SD_CSV == write_type)
	{
        TimeRecord();
        sprintf(WriteBuffer, "%d,%d,%d,%.2f,%.2f,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
                ,F405.Chassis_Flag&2
                ,F405.Chassis_Flag
                ,test
                ,AD_actual_value
                ,JudgeReceive.realChassispower
                ,JudgeReceive.remainEnergy
                ,JudgeReceive.MaxPower
                ,RealCarSpeed.SpeedX.SetPoint
                ,RealCarSpeed.SpeedX.ActualValue
                ,RealCarSpeed.SpeedY.SetPoint
                ,RealCarSpeed.SpeedY.ActualValue
                ,RealCarSpeed.SpeedW.SetPoint
                ,RealCarSpeed.SpeedW.ActualValue
                );
        strcat(WriteBuffer,TimeBuffer);
	}
    else
    {
        sprintf(WriteBuffer, "[ERROR]:  ");
        sprintf(WriteBuffer + 11, str);
    }

    if (sd_status.SD_FS_Open_result) //确保已经打开再写
    {
        sd_status.res_sd = f_write(&sd_status.fnew, WriteBuffer, strlen(WriteBuffer), &sd_status.fnum);
        if (sd_status.res_sd == FR_OK)
            sd_status.SD_FS_Write_result = 1; //写入成功
        else
            sd_status.SD_FS_Write_result = 0;
    }
}
void OpenSDCard()
{
    sd_status.res_sd = f_open(&sd_status.fnew, (const TCHAR *)DataFile, FA_OPEN_ALWAYS | FA_WRITE); //打开文件，如果文件不存在则创建它
    uint32_t file_size = 0;

    if (sd_status.res_sd == FR_OK)
    {
        file_size = f_size(&sd_status.fnew); //获得文件已经写入的长度
        sd_status.SD_FS_Open_result = 1;     //打开文件成功
        if (file_size > 0)                   //判断文件非空（大于零的数），并寻找添加头（即文件尾）
            sd_status.res_sd = f_lseek(&sd_status.fnew, file_size);
    }
    else
        sd_status.SD_FS_Open_result = 0;
}

void CloseSDCard()
{
	sd_status.res_sd = f_sync(&sd_status.fnew);
	if(sd_status.res_sd != FR_OK)
	{
		f_close(&sd_status.fnew);
		sd_status.SD_FS_Open_result = 0;

	}
}

void SDCard_task(void *pvParameters)
{
//    int random = 0;
//    random = rand();
//    sprintf(DataFile,"0:infantry_%d.csv",random%30);
    // SD卡初始化
    while ((sd_status.Status = SD_Init()) != SD_RESPONSE_NO_ERROR)
        sd_status.SD_init_result = 0; // printf("SD卡初始化失败，请确保SD卡已正确接入开发板，或换一张SD卡测试！\n");
    sd_status.SD_init_result = 1;     // printf("SD卡初始化成功！\n");

    //挂载文件系统
    sd_status.res_sd = f_mount(&sd_status.fs, "0:", 1); //在外部SPI Flash挂载文件系统，文件系统挂载时会对SPI设备初始化

    if (sd_status.res_sd != FR_OK)
        sd_status.SD_FS_Mount_result = 0; //挂载失败
    else
        sd_status.SD_FS_Mount_result = 1; //挂载成功

    OpenSDCard();
    if (sd_status.SDCard_task_init == 0)
    {
        //任务初始化
        sd_status.SDCard_task_init = 1;
        SDLOG(SD_START, "");
    }

    while (1)
    {

		temp1++;
        sd_status.count++;
        if (sd_status.count % 1000 == 0) // 1hz
        {
			if(sd_status.SD_FS_Open_result == 0)
				OpenSDCard(); 
        }
        if (sd_status.count % 2 == 0) // 500hz
        {

        }

        if (sd_status.count % 5 == 0) // 200hz
        {
        }

        if (sd_status.count % 100 == 0) // 10HZ
        {
            SDLOG(SD_CSV, "TEST");
        }
        if (sd_status.count % 1000 == 999) // 1hz
        {
            CloseSDCard(); //每一秒将所有数据保存下来
        }



		vTaskDelay(1);
		
    }

    //关闭
    // f_mount(NULL, "0:", 1); /* 不再使用文件系统，取消挂载文件系统 */
}
