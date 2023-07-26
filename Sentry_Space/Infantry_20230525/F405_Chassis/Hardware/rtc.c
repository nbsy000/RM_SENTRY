#include "rtc.h"
void RTC_Configuration(void)
{
  uint16_t retry = 0x1fff;
  RTC_InitTypeDef RTC_InitStructure;
  RTC_DateTypeDef RTC_Date;
  RTC_TimeTypeDef RTC_Time;

  //打开时钟和后备寄存器访问
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
  PWR_BackupAccessCmd(ENABLE);
  
  if(RTC_ReadBackupRegister(RTC_BKP_DR0)!=0x5050)//如果为第一次配置
	{
		RCC_LSEConfig(RCC_LSE_ON);//3. 那么开启LSE
        //并检查指定的RCC标志位设置是否完毕,等待低速晶振就绪
		while(RCC_GetFlagStatus(RCC_FLAG_LSERDY)==RESET)
			{
				retry++;
				delay_ms(10);
			}
		if(retry==0)//如果等待时间过长
            return;//LSE 开启失败.
  
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
  RCC_RTCCLKCmd(ENABLE);
  
  //设定分频系数
  RTC_InitStructure.RTC_AsynchPrediv = 0X7F;  //异步分频
  RTC_InitStructure.RTC_SynchPrediv = 0XFF;   //同步分频
  RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24; //设置为24小时计时法
  
  RTC_Init (&RTC_InitStructure);
    
  //设置日期和时间
  RTC_Date.RTC_Year = 23;
  RTC_Date.RTC_Month = 4;
  RTC_Date.RTC_Date = 22;
  RTC_Date.RTC_WeekDay = RTC_Weekday_Saturday;
    
  RTC_Time.RTC_Hours = 12;
  RTC_Time.RTC_Minutes = 0;
  RTC_Time.RTC_Seconds = 0;
  RTC_Time.RTC_H12 = RTC_H12_AM;
    
  RTC_SetTime (RTC_Format_BIN,&RTC_Time);
  RTC_SetDate(RTC_Format_BIN,&RTC_Date);
  
  //标记为已初始化过
  RTC_WriteBackupRegister(RTC_BKP_DR0,0X5050);
  }
  return ;

}