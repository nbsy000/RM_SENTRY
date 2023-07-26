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
char ReadBuffer[100] = "";							/* ��������*/
float sd_read_var[VAR_NUMBER+1];

//ɾ���ļ��Ľṹ����� 3��������ֹ�����
uint8_t delete_flag1,delete_flag2,delete_flag3;
TCHAR DataFile[] = "0:SENTRY.csv"; //�ļ���
SDStatus sd_status;

extern uint8_t Anomalies_tag;
extern JudgeReceive_t JudgeReceive;
extern F405_typedef F405;
extern INA260 INA260_1;
extern TaskHandle_t User_Tasks[TASK_NUM];
float temp1 = 2.0, temp2 = 3.0;
int count = 0;
void SDLOG(enum SDWRITE write_type, const char *str)
{
    memset(WriteBuffer, '\0', sizeof(WriteBuffer)); //��ջ�����
    if (SD_START == write_type)
    {
        sprintf(WriteBuffer, "Plimit,Power,RemainEnergy,HeatCool17,HeatMax17,shooterHeat17,self_outpost_hp,self_base_hp,Chasis_State,Count\n");
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
				count++;
        sprintf(WriteBuffer, "%d,%.1f,%d,%d,%d,%d,%d,%d,%d,%d\n" , JudgeReceive.MaxPower, JudgeReceive.realChassispower, JudgeReceive.remainEnergy,
                                                    JudgeReceive.HeatCool17, JudgeReceive.HeatMax17, JudgeReceive.shooterHeat17,
                                                    JudgeReceive.self_outpost_hp, JudgeReceive.self_base_hp, F405.Chassis_Flag,count);
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
    sd_status.res_sd = f_open(&sd_status.fnew, (const TCHAR *)DataFile, FA_OPEN_ALWAYS | FA_WRITE | FA_READ); //���ļ�������ļ��������򴴽���
    uint32_t file_size = 0;

    if (sd_status.res_sd == FR_OK)
    {
        file_size = f_size(&sd_status.fnew); //����ļ��Ѿ�д��ĳ���
        sd_status.SD_FS_Open_result = 1;     //���ļ��ɹ�

#ifndef SD_READ_MODE //д��ģʽ
        if (file_size > 0)                   //�ж��ļ��ǿգ����������������Ѱ�����ͷ�����ļ�β��
            sd_status.res_sd = f_lseek(&sd_status.fnew, file_size);
#endif
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


uint32_t bytesRead;
void ReadSDCard()
{
		static int i = 0;
		// ��ȡ�ļ��е�����
		f_read(&sd_status.fnew,ReadBuffer,100,&bytesRead);
		if(bytesRead)
		{
			VOFA_Send();
			memset(ReadBuffer, 0, sizeof(ReadBuffer));
		}
}
double start_time_SD = 0;
double now_time = 0;
void SDCard_task(void *pvParameters)
{
//    int random = 0;
//    random = rand();
//    sprintf(DataFile,"0:infantry_%d.csv",random);
    // SD����ʼ��
	start_time_SD = GetTime_s();
    while ((sd_status.Status = SD_Init()) != SD_RESPONSE_NO_ERROR)	
	{
		now_time = GetTime_s();
		if( now_time - start_time_SD > 5.0)
		{
			vTaskDelete(User_Tasks[SDCARD_TASK]);
			return;
		}
        sd_status.SD_init_result = 0; // printf("SD����ʼ��ʧ�ܣ���ȷ��SD������ȷ���뿪���壬��һ��SD�����ԣ�\n");
    }
	if(sd_status.Status == SD_RESPONSE_NO_ERROR)
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
			
#ifndef SD_READ_MODE
        SDLOG(SD_START, "");
#endif
    }

    while (1)
    {
				temp1++;
        sd_status.count++;
#ifndef SD_READ_MODE

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


#else

				if(delete_flag1&delete_flag2&delete_flag3)
				{
					sd_status.SD_FS_DELETE_result	 = f_unlink(DataFile);
				}
				else
				{
					if (sd_status.count % 10 == 0) // 100HZ
					{
							ReadSDCard();
					}
				}
					
#endif

			vTaskDelay(1);
		
    }

    //�ر�
    // f_mount(NULL, "0:", 1); /* ����ʹ���ļ�ϵͳ��ȡ�������ļ�ϵͳ */
}
