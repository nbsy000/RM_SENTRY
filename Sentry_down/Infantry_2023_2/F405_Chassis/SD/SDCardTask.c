/**********************************************************************************************************
 * @�ļ�     SDCardTask.c
 * @˵��     ��дSD������¼��־
 * @�汾  	 V1.0
 * @����     �ο���
 * @����     2022.7
 **********************************************************************************************************/
#include "main.h"
#include "stdlib.h"

char WriteBuffer[128] = "";              /* д������*/
TCHAR DataFile[] = "0:infantry_down.csv"; //�ļ���
SDStatus sd_status;
extern uint8_t Anomalies_tag;
extern JudgeReceive_t JudgeReceive;
extern F405_typedef F405;
extern INA260 INA260_1;
float temp1 = 2.0, temp2 = 3.0;
void SDLOG(enum SDWRITE write_type, const char *str)
{
    memset(WriteBuffer, '\0', sizeof(WriteBuffer)); //��ջ�����
    if (SD_START == write_type)
    {
        sprintf(WriteBuffer, "Plimit,Power,RemainEnergy,SelfProtect,SuperCap\n");
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
        sprintf(WriteBuffer, "%d,%.1f,%d,%d,%d\n" ,JudgeReceive.MaxPower,JudgeReceive.realChassispower ,JudgeReceive.remainEnergy,F405.Chassis_Flag,F405.SuperPowerLimit);
	}
    else
    {
        sprintf(WriteBuffer, "[ERROR]:  ");
        sprintf(WriteBuffer + 11, str);
    }

    if (sd_status.SD_FS_Open_result) //ȷ���Ѿ�����д
    {
        sd_status.res_sd = f_write(&sd_status.fnew, WriteBuffer, strlen(WriteBuffer), &sd_status.fnum);
        if (sd_status.res_sd == FR_OK)
            sd_status.SD_FS_Write_result = 1; //д��ɹ�
        else
            sd_status.SD_FS_Write_result = 0;
    }
}
void OpenSDCard()
{
    sd_status.res_sd = f_open(&sd_status.fnew, (const TCHAR *)DataFile, FA_OPEN_ALWAYS | FA_WRITE); //���ļ�������ļ��������򴴽���
    uint32_t file_size = 0;

    if (sd_status.res_sd == FR_OK)
    {
        file_size = f_size(&sd_status.fnew); //����ļ��Ѿ�д��ĳ���
        sd_status.SD_FS_Open_result = 1;     //���ļ��ɹ�
        if (file_size > 0)                   //�ж��ļ��ǿգ����������������Ѱ�����ͷ�����ļ�β��
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
//    sprintf(DataFile,"0:infantry_%d.csv",random);
    // SD����ʼ��
    while ((sd_status.Status = SD_Init()) != SD_RESPONSE_NO_ERROR)
        sd_status.SD_init_result = 0; // printf("SD����ʼ��ʧ�ܣ���ȷ��SD������ȷ���뿪���壬��һ��SD�����ԣ�\n");
    sd_status.SD_init_result = 1;     // printf("SD����ʼ���ɹ���\n");

    //�����ļ�ϵͳ
    sd_status.res_sd = f_mount(&sd_status.fs, "0:", 1); //���ⲿSPI Flash�����ļ�ϵͳ���ļ�ϵͳ����ʱ���SPI�豸��ʼ��

    if (sd_status.res_sd != FR_OK)
        sd_status.SD_FS_Mount_result = 0; //����ʧ��
    else
        sd_status.SD_FS_Mount_result = 1; //���سɹ�

    OpenSDCard();
    if (sd_status.SDCard_task_init == 0)
    {
        //�����ʼ��
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
            CloseSDCard(); //ÿһ�뽫�������ݱ�������
        }



		vTaskDelay(1);
		
    }

    //�ر�
    // f_mount(NULL, "0:", 1); /* ����ʹ���ļ�ϵͳ��ȡ�������ļ�ϵͳ */
}
