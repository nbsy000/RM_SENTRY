/**********************************************************************************************************
 * @文件     usart2.c
 * @说明     uart初始化,VOFA
 * @版本  	 V1.0
**********************************************************************************************************/
#include "main.h"


//#if VOFA

//unsigned char DataScopeSend_Buf[DataScopeSend_BufSize];
#define	VOFA_MAX_CHANNEL 10
float 	VOFA_justfloat[VOFA_MAX_CHANNEL];
uint8_t VOFA_send_Data[VOFA_MAX_CHANNEL*4+4];
DMA_InitTypeDef   dma;
extern float INA_Power;
extern ChassisState_t chassis;
/**********************************************************************************************************
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void VOFA_USART_Configuration(void)
{
	USART_InitTypeDef usart;
	GPIO_InitTypeDef  gpio;
	NVIC_InitTypeDef  nvic;
	
	//写入结尾数据
	VOFA_send_Data[VOFA_MAX_CHANNEL*4] = 0x00;                    
	VOFA_send_Data[VOFA_MAX_CHANNEL*4+1] = 0x00;
	VOFA_send_Data[VOFA_MAX_CHANNEL*4+2] = 0x80;
	VOFA_send_Data[VOFA_MAX_CHANNEL*4+3] = 0x7f;
	
	nvic.NVIC_IRQChannel = VOFA_DMA_TX_IRQ;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
	{
		VOFA_DMA_TX_AHBxClock_FUN(VOFA_DMA_TX_CLK,ENABLE);
		DMA_DeInit(VOFA_DMA_TX_STREAM);
		dma.DMA_Channel= VOFA_DMA_TX_CHANNEL;
		dma.DMA_PeripheralBaseAddr = (uint32_t)&(VOFA_USARTx->DR);
		dma.DMA_Memory0BaseAddr = (uint32_t)VOFA_send_Data;
		dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
		dma.DMA_BufferSize = (VOFA_MAX_CHANNEL*4+4);
		dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
		dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		dma.DMA_Mode = DMA_Mode_Normal;
		dma.DMA_Priority = DMA_Priority_VeryHigh;
		dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
		dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
		dma.DMA_MemoryBurst = DMA_Mode_Normal;
		dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	
		DMA_Init(VOFA_DMA_TX_STREAM,&dma);
		DMA_ClearFlag(VOFA_DMA_TX_STREAM, VOFA_DMA_FLAG_TCIF);  // clear all DMA flags
		DMA_ITConfig(VOFA_DMA_TX_STREAM,DMA_IT_TC,ENABLE);  //open DMA send inttrupt
		DMA_Cmd(VOFA_DMA_TX_STREAM,DISABLE);
	}	
	
	VOFA_USART_GPIO_APBxClock_FUN(VOFA_USART_GPIO_CLK,ENABLE);
	VOFA_USART_APBxClock_FUN(VOFA_USART_CLK,ENABLE);

	GPIO_PinAFConfig(VOFA_USART_TX_PORT,VOFA_USART_TX_SOURCE,VOFA_USART_TX_AF);
	GPIO_PinAFConfig(VOFA_USART_RX_PORT,VOFA_USART_RX_SOURCE,VOFA_USART_RX_AF); 

	gpio.GPIO_Pin = VOFA_USART_TX_PIN;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(VOFA_USART_TX_PORT,&gpio);
	
	gpio.GPIO_Pin = VOFA_USART_RX_PIN;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(VOFA_USART_RX_PORT,&gpio);

	usart.USART_BaudRate = VOFA_USART_BAUD_RATE;
	usart.USART_WordLength = USART_WordLength_8b;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(VOFA_USARTx,&usart);
	
	USART_Cmd(VOFA_USARTx,ENABLE);
	
	USART_DMACmd(VOFA_USARTx,USART_DMAReq_Tx,ENABLE);	

	DMA_Cmd(VOFA_DMA_TX_STREAM,ENABLE);
}
void VOFA_DMA_TX_INT_FUN(void)
{
	if(DMA_GetITStatus(VOFA_DMA_TX_STREAM,VOFA_DMA_IT_STATUS  ))
	{
		DMA_ClearITPendingBit(VOFA_DMA_TX_STREAM,VOFA_DMA_IT_STATUS);
		DMA_Cmd(VOFA_DMA_TX_STREAM, DISABLE); 
	}
}

//将要输出的变量声明extern
void VOFA_Send(void)
{
	//需转为float型数据存储
	//功率测试参数：
	extern float K_Power;
	extern float test_W_Chassis_t1;
	extern float test_W_Chassis_t2;
	extern float test_KP;
//	VOFA_justfloat[0]=(float)(JudgeReceive.MaxPower);
	VOFA_justfloat[0]=(float)(JudgeReceive.remainEnergy);
	VOFA_justfloat[1]=(float)(JudgeReceive.realChassispower);
//	VOFA_justfloat[3]=(float)(chassis.carSpeedw);	
//	VOFA_justfloat[4]=(float)(chassis.carSpeedx/1000.0f);
//	VOFA_justfloat[7]=(float)(chassis.carSpeedy/1000.0f);		
//	
//	VOFA_justfloat[0] = (float)chassis.CurrentState.X;
//	VOFA_justfloat[1] = (float)chassis.CurrentState.Y;	
//	VOFA_justfloat[2] = (float)PCReceive.now_x;
//	VOFA_justfloat[3] = (float)PCReceive.now_y;
	
//	VOFA_justfloat[0] = Gyro_Chassis.YAW_ABS;
//	VOFA_justfloat[1] = Gyro_ChassisYaw.YAW_ABS;
//	VOFA_justfloat[2] = (float)PCReceive.now_x;
//	VOFA_justfloat[3] = (float)PCReceive.now_y;	
//	VOFA_justfloat[2] = (float)PCReceive.now_x;
//	VOFA_justfloat[3] = (float)PCReceive.now_y;	

	//大Yaw轴参数：
//	extern Motor_9025_t Motor_9025;
//	extern Gyro_Typedef Gyro_ChassisYaw;//大Yaw轴数据
//	VOFA_justfloat[0]=(float)(Motor_9025.PidPos.SetPoint);
//	VOFA_justfloat[1]=(float)(-Gyro_ChassisYaw.YAW_ABS);
//	VOFA_justfloat[2]=(float)(Motor_9025.PidSpeed.SetPoint);
//	VOFA_justfloat[3]=(float)(-Gyro_ChassisYaw.GZ);

	//底盘各电机编码值
//	extern int dif_encoder[];
//	extern int average; 
//	extern Chassis_Motor_t ChassisMotor[];
//	VOFA_justfloat[0]=(float)(ChassisMotor[0].total_x);
//	VOFA_justfloat[1]=(float)(ChassisMotor[1].total_x);
//	VOFA_justfloat[2]=(float)(ChassisMotor[2].total_x);
//	VOFA_justfloat[3]=(float)(ChassisMotor[3].total_x);	
//	VOFA_justfloat[4]=(float)(average);	
//	VOFA_justfloat[5]=(float)(dif_encoder[0]);	
//	VOFA_justfloat[6]=(float)(dif_encoder[1]);	
//	VOFA_justfloat[7]=(float)(dif_encoder[2]);	
//	VOFA_justfloat[8]=(float)(dif_encoder[3]);	
	
	//拷贝到传输变量
	memcpy(VOFA_send_Data, (uint8_t *)VOFA_justfloat, sizeof(VOFA_justfloat));
	
	VOFA_DMA_TX_STREAM->NDTR = VOFA_MAX_CHANNEL*4+4; 
	DMA_Cmd(VOFA_DMA_TX_STREAM, ENABLE);   

}

//重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{   
    USART_SendData(USART2, (uint8_t)ch);  // 发送一个字节数据到串口
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) ;// 等待发送完毕
    return (ch);
}


//#elseif PC_ROS

//uint8_t PCbuffer[PC_RECVBUF_SIZE] = {0, 0, 0};
//uint8_t PC_SendBuf[PC_SENDBUF_SIZE];

///**
//  * @brief  串口2配置，PC通信
//  * @param  None
//  * @retval None
//  */
//void USART2_Configuration(void)
//{
//    USART_InitTypeDef usartInit;
//    GPIO_InitTypeDef  gpioInit;
//    NVIC_InitTypeDef  nvicInit;

//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);

//    gpioInit.GPIO_Pin = GPIO_Pin_2;
//    gpioInit.GPIO_Mode = GPIO_Mode_AF;
//    gpioInit.GPIO_OType = GPIO_OType_PP;
//    gpioInit.GPIO_Speed = GPIO_Speed_100MHz;
//    gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
//    GPIO_Init(GPIOA, &gpioInit);

//    gpioInit.GPIO_Pin = GPIO_Pin_3;
//    gpioInit.GPIO_Mode = GPIO_Mode_AF;
//    gpioInit.GPIO_OType = GPIO_OType_PP;
//    gpioInit.GPIO_Speed = GPIO_Speed_100MHz;
//    gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
//    GPIO_Init(GPIOA, &gpioInit);

//    gpioInit.GPIO_Pin = GPIO_Pin_1;
//    gpioInit.GPIO_Mode = GPIO_Mode_OUT;
//    gpioInit.GPIO_OType = GPIO_OType_PP;
//    gpioInit.GPIO_Speed = GPIO_Speed_100MHz;
//    gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
//    GPIO_Init(GPIOA, &gpioInit);

//    usartInit.USART_BaudRate = 115200;
//    usartInit.USART_WordLength = USART_WordLength_8b;
//    usartInit.USART_StopBits = USART_StopBits_1;
//    usartInit.USART_Parity = USART_Parity_No;
//    usartInit.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
//    usartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//    USART_Init(USART2, &usartInit);

//    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
//    
//    USART_Cmd(USART2, ENABLE);
//    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
//    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);

//    nvicInit.NVIC_IRQChannel = USART2_IRQn;
//    nvicInit.NVIC_IRQChannelPreemptionPriority = 1;
//    nvicInit.NVIC_IRQChannelSubPriority = 0;
//    nvicInit.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&nvicInit);

//    {
//        DMA_InitTypeDef dmaInit;
//        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
//        DMA_DeInit(DMA1_Stream5);
//        dmaInit.DMA_Channel = DMA_Channel_4;
//        dmaInit.DMA_PeripheralBaseAddr = (uint32_t) & (USART2->DR);
//        dmaInit.DMA_Memory0BaseAddr = (uint32_t)PCbuffer;
//        dmaInit.DMA_DIR = DMA_DIR_PeripheralToMemory;
//        dmaInit.DMA_BufferSize = PC_RECVBUF_SIZE;
//        dmaInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//        dmaInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
//        dmaInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//        dmaInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//        dmaInit.DMA_Mode = DMA_Mode_Circular;
//        dmaInit.DMA_Priority = DMA_Priority_VeryHigh;
//        dmaInit.DMA_FIFOMode = DMA_FIFOMode_Disable;
//        dmaInit.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//        dmaInit.DMA_MemoryBurst = DMA_Mode_Normal;
//        dmaInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

//        nvicInit.NVIC_IRQChannel = DMA1_Stream5_IRQn;
//        nvicInit.NVIC_IRQChannelPreemptionPriority = 1;
//        nvicInit.NVIC_IRQChannelSubPriority = 3;
//        nvicInit.NVIC_IRQChannelCmd = ENABLE;
//        NVIC_Init(&nvicInit);

//        DMA_Init(DMA1_Stream5, &dmaInit);
//        DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);
//        DMA_Cmd(DMA1_Stream5, ENABLE);
//    }

//    {
//        DMA_InitTypeDef dmaInit;
//        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
//        DMA_DeInit(DMA1_Stream6);
//        dmaInit.DMA_Channel = DMA_Channel_4;
//        dmaInit.DMA_PeripheralBaseAddr = (uint32_t) & (USART2->DR);
//        dmaInit.DMA_Memory0BaseAddr = (uint32_t)SendToPC_Buff;
//        dmaInit.DMA_DIR = DMA_DIR_MemoryToPeripheral;
//        dmaInit.DMA_BufferSize = PC_SENDBUF_SIZE;
//        dmaInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//        dmaInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
//        dmaInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//        dmaInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//        dmaInit.DMA_Mode = DMA_Mode_Normal;
//        dmaInit.DMA_Priority = DMA_Priority_VeryHigh;
//        dmaInit.DMA_FIFOMode = DMA_FIFOMode_Disable;
//        dmaInit.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//        dmaInit.DMA_MemoryBurst = DMA_Mode_Normal;
//        dmaInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

//        nvicInit.NVIC_IRQChannel = DMA1_Stream6_IRQn;
//        nvicInit.NVIC_IRQChannelPreemptionPriority = 1;
//        nvicInit.NVIC_IRQChannelSubPriority = 1;
//        nvicInit.NVIC_IRQChannelCmd = ENABLE;
//        NVIC_Init(&nvicInit);

//        DMA_Init(DMA1_Stream6, &dmaInit);
//        DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);
//        DMA_Cmd(DMA1_Stream6, DISABLE);
//    }
//}

///**
//  * @brief  串口2空闲中断配置
//  * @param  None
//  * @retval None
//  */
//void USART2_IRQHandler(void)
//{
//    if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
//    {
//        (void)USART2->SR; //clear the IDLE int
//        (void)USART2->DR;
//    }
//}


//uint8_t tempPC[PC_RECVBUF_SIZE]; //这里要改为全局变量，不然crc不通过
//int16_t Crcpass, crcNopass;
////PC_Recv_t PC_Recv_D;
//uint8_t ErrorBuff[PC_RECVBUF_SIZE * 4];
//int16_t buffindex;
///**
//  * @brief  串口2 DMA接收中断
//  * @param  None
//  * @retval None
//  */
//void DMA1_Stream5_IRQHandler(void)
//{
//    static uint8_t temptemp[2 * PC_RECVBUF_SIZE];
//    int16_t PackPoint, n;
//    if (DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5))
//    {
//        DMA_Cmd(DMA1_Stream5,DISABLE);
//        memcpy(temptemp + PC_RECVBUF_SIZE, PCbuffer, PC_RECVBUF_SIZE);
//        for (PackPoint = 0; PackPoint < PC_RECVBUF_SIZE; PackPoint++) //防止错位，不一定数组元素的第一个就为
//        {
//            if (temptemp[PackPoint] == '!')
//            {
//                for (n = 0; n < PC_RECVBUF_SIZE; n++)
//                {
//                    tempPC[n] = temptemp[(n + PackPoint)];
//                }
//                crcNopass++;
////                if (Verify_CRC8_Check_Sum(tempPC, PC_RECVBUF_SIZE))
//                    PC_Receive(tempPC);
////                else
////                {
////                    buffindex++;
////                    buffindex = buffindex % 4;
////                }
//                break;
//            }
//        }
//        DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
//        DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
//        memcpy(temptemp, temptemp + PC_RECVBUF_SIZE, PC_RECVBUF_SIZE);
//        DMA_Cmd(DMA1_Stream5,ENABLE);
//    }
//}

///**
//  * @brief  串口2 DMA发送中断
//  * @param  None
//  * @retval None
//  */
//void DMA1_Stream6_IRQHandler(void)
//{
//    if (DMA_GetFlagStatus(DMA1_Stream6, DMA_IT_TCIF6) == SET)
//    {
//        DMA_Cmd(DMA1_Stream6, DISABLE);
//        DMA_SetCurrDataCounter(DMA1_Stream6, PC_SEND_BUF_SIZE);
//        DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6);
//        DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);
//    }
//}

//#else

////PA3-USART2_RX
////PA2-USART2_TX

//volatile unsigned char mmWave_RxBuffer[40];
//float distancex = 0,distancey = 0,distancez = 0, angle = 0;
//float distancex_sum = 0,distancey_sum = 0,distancez_sum = 0,angle_sum = 0;

///*
// *
// * 将毫米波传感器板IWR1443BOOST传送过来的数据进行解析,并且送到上位机PC端显示.
// * 在PC端打开串口调试助手(115200,无校验位,1停止位,LSB),可以显示如下数据:
// * System is running!
// * index 485    range 0.4902    speed 0.1270    angle -30    x -0.2461    y 0.4219    z 0.0547
// * index 486    range 0.4902    speed 0.1270    angle -28    x -0.2305    y 0.4238    z 0.0957
// * index 489    range 0.4902    speed 0.1270    angle -34    x -0.2754    y 0.3984    z 0.0742
// * index 490    range 0.4902    speed 0.1270    angle -32    x -0.2617    y 0.4121    z 0.0527
// * index 548    range 0.0000    speed 0.0000    angle 0      x 0.0000     y 0.0000    z 0.0000
// * index 549    range 0.0000    speed 0.0000    angle 0      x 0.0000     y 0.0000    z 0.0000
// *
// * ------------------------------------------------------------------
// * MCU的基本配置描述:
// * (1)MSP432启用外部高频晶振48M;  MCLK=48MHz,HSMCLK=48MHz,SMCLK=24MHz
// * (2)使用到了两个UART模块:
// *   a. EUSCI_A0 (P1.2/RXD, P1.3/TXD) -->XDS110调试器 --> PC
// *      EUSCI_A2 (P3.2/RXD, P3.3/TXD) -->mmWave传感器
// *   b. 在配置上,时钟来自SMCLK,115200,无校验位,1停止位,LSB
// *   c. 接收采用中断的方式,开启了相应的中断函数,用来接收数据
// *   d. 往PC发送使用了printf函数,修改了底层重定向,具体可参考源代码;
// * (3)红色LED指示灯(P1.0)
// *    uart每收到一个字节,在中断中设置标志位.
// *    while(1)主循环里对每个字节进行处理.开始处理数据时会点亮,处理完之后熄灭.
// *    可以理解为,只要UART收到毫米波传感器的数据,则红色闪烁指示;
// * ------------------------------------------------------------------
// * 毫米波传感器输出的数据格式:
// * 数据帧的举例如下:
// * (1)目标点0个:0B 0A 0D 0C 00 00 09 00
// * (2)目标点1个:0B 0A 0D 0C 01 00 09 00 3F 09 41 00 1B 00 0B 04 B0 07 2A 03
// * (3)目标点2个:0B 0A 0D 0C 02 00 09 00 3F 09 41 00 1B 00 0B 04 DF 07 AD 02 3F 09 BF FF 1C 00 55 04 00 08 A3 01
// *
// * 补充:
// * a.每秒更新10次?
// * b.每次检测最多输出多少个目标?
// *
// * 硬件连接示意图:
// * 波特率:115200,无校验位,1停止位,LSB
// *   mmWave           MSP432P401
// * +--------+      +--------------+
// * |        |      |              |
// * | 第5个引脚| -->  |P3.2/UCA2RXD  |   XDS110调试器
// * |Gnd旁一列 |      |              |    +---+
// * |        |      |  P1.2/UCA0RXD|<---|   |  ----USB线接PC机
// * |        |      |  P1.3/UCA0TXD|--->|   |
// * |    Gnd | ---  |Gnd           |    +---+
// * +--------+      +--------------+
// *
// *
// *******************************************************************************/

////=========================================================
////
//// 变量声明和定义
////
////=========================================================
//volatile uint8_t RXData                   = 0;
//volatile uint8_t state_machine            = 0;
//volatile uint8_t RXDataflag               = 0;
//volatile uint8_t parseMagicCodeflag       = 1;
//volatile uint8_t parseHeaderflag          = 0;
//volatile uint8_t parseDetectedObjectsflag = 0;
//volatile uint8_t uart1_cnt                = 0;
//volatile uint8_t uart1_buf[UART1_BUF_LEN];
//volatile uint8_t DetectedObjects_cnt      = 0;

//volatile uint8_t objOutIndx;
//uint16_t Indx = 1;   // index of serial output data

//typedef volatile struct My_output_message_header_t
//{
//    /*! @brief   Output buffer magic word (sync word). It is initialized to  {0x0102,0x0304,0x0506,0x0708} */
//    uint16_t    magicWord[2];

//    /*! @brief   Number of detected objects */
//    uint16_t    numDetectedObj;

//    uint16_t     xyzQFormat;

//} My_output_message_header;

//My_output_message_header my_header;

///*!
// *  @brief    Detected object estimated parameters
// *
// */
//typedef volatile struct MmwDemo_detectedObj_t
//{
//    uint16_t   range;     /*!< @brief Range index */
//    int16_t   speed;   /*!< @brief speed index. Note that it is changed
//                                 to signed integer in order to handle extended maximum velocity.
//                                 Neagative values correspond to the object moving toward
//                                 sensor, and positive values correspond to the
//                                 object moving away from the sensor */
//    int16_t  angle;      /*!< @brief Peak value */
//    int16_t  x;             /*!< @brief x - coordinate in meters. Q format depends on the range resolution */
//    int16_t  y;             /*!< @brief y - coordinate in meters. Q format depends on the range resolution */
//    int16_t  z;             /*!< @brief z - coordinate in meters. Q format depends on the range resolution */
//} MmwDemo_detectedObj;

//MmwDemo_detectedObj detectedObj[maxDetectedObjectsNum];

////=========================================================
////
//// 函数声明
////
////=========================================================

////uint8_t parseMagicCode(void);
////uint8_t parseDetectedObjects(void);
////void mmWavesensor_IWR1443BOOST_Data_lite_version_receiveData(void);

////---------------------------------------------------------
////printf函数重定向
//int fputc(int ch, FILE *f);
//int fputs(const char *ptr, FILE *f);
//int fgetc(FILE *f);
////int fputc(int _c, register FILE *_fp);
////int fputs(const char *_ptr, register FILE *_fp);
////---------------------------------------------------------

//void USART2_Configuration(void)
//{
//	USART_InitTypeDef usart;
//	GPIO_InitTypeDef  gpio;
//	NVIC_InitTypeDef  nvic;

//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

//  GPIO_PinAFConfig(GPIOA,GPIO_PinSource2, GPIO_AF_USART2); 
//  GPIO_PinAFConfig(GPIOA,GPIO_PinSource3, GPIO_AF_USART2); 
//	
//  gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
//  gpio.GPIO_Mode = GPIO_Mode_AF;
//  gpio.GPIO_OType = GPIO_OType_PP;
//  gpio.GPIO_Speed = GPIO_Speed_100MHz;
//  gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOA,&gpio);
//	
//	USART_DeInit(USART2);
//	usart.USART_BaudRate = 115200;
//	usart.USART_WordLength = USART_WordLength_8b;
//	usart.USART_StopBits = USART_StopBits_1;
//	usart.USART_Parity = USART_Parity_No;
//	usart.USART_Mode = USART_Mode_Rx;
//	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   
//	USART_Init(USART2, &usart);
//	 
//	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);   //开启串口中断 
//	USART_Cmd(USART2, ENABLE);
//	//USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE); 

//	nvic.NVIC_IRQChannel = USART2_IRQn;
//	nvic.NVIC_IRQChannelPreemptionPriority = 0;
//	nvic.NVIC_IRQChannelSubPriority = 3;
//	nvic.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&nvic);
//		
////		{
////        DMA_InitTypeDef   dma;
////        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
////        DMA_DeInit(DMA2_Stream2);//stream0重设为缺省值
////        dma.DMA_Channel= DMA_Channel_4;
////        dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);/* UART5接收数据地址 */
////        dma.DMA_Memory0BaseAddr = (uint32_t)mmWave_RxBuffer;//接收缓存区首地址
////        dma.DMA_DIR = DMA_DIR_PeripheralToMemory;//数据存储方向
////        dma.DMA_BufferSize = mmWaveBufferSize;
////        dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
////        dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
////        dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
////        dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
////        dma.DMA_Mode = DMA_Mode_Circular;
////        dma.DMA_Priority = DMA_Priority_VeryHigh;
////        dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
////        dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
////        dma.DMA_MemoryBurst = DMA_Mode_Normal;
////        dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

////  			DMA_Init(DMA2_Stream2, &dma);
////        DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);
////        DMA_Cmd(DMA2_Stream2, ENABLE);
////		}

//}


///**
//  * @brief  DMA接收中断
//  * @param  None
//  * @retval None
//  */
////void DMA2_Stream2_IRQHandler(void)
////{
////	if(DMA_GetFlagStatus(DMA2_Stream2, DMA_IT_TCIF4) == SET)
////	{
////		DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF4);
////    DMA_ClearITPendingBit(DMA2_Stream2, DMA_FLAG_TCIF4);
////	}
////}

///**
//  * @brief  串口的接收中断
//  * @param  None
//  * @retval None
//  */
//void USART2_IRQHandler(void)
//{
//	uint32_t status = USART_GetITStatus(USART2, USART_IT_RXNE);
////	USART_ClearFlag(USART2, USART_FLAG_RXNE);
//	//接收中断,uart接收到字节,接收中断标志位会置1
//	if(USART_GetITStatus(USART2,USART_IT_RXNE) !=RESET)
//	{
//		(void) USART2->SR;
//		(void) USART2->DR;

//		RXData = USART_ReceiveData(USART2);//接串口数据
//		if(parseHeaderflag || parseDetectedObjectsflag)
//		{
//			uart1_buf[uart1_cnt] = RXData;
//			uart1_cnt++;
//		}
//		RXDataflag = 1;
//		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
//	}
//}


////=========================================================
//// 帧头解析函数
//// 采用状态机的形式每次判断一个字节, 判断到完整的帧头时,返回1. 否则返回0.
//// 即连续的4个字节是: 0x0b,0x0a,0x0d,0x0c
////=========================================================
//uint8_t parseMagicCode(void)
//{
//    if(state_machine == 0)         // 协议解析魔法数状态机
//    {
//          if(RXData == 0x0b)       // 接收到帧头第一个魔法数
//              state_machine = 1;
//          else
//              state_machine = 0;   // 状态机复位
//    }
//    else if(state_machine == 1)
//    {
//          if(RXData == 0x0a)       // 接收到帧头第二个魔法数
//              state_machine = 2;
//          else
//              state_machine = 0;   // 状态机复位
//    }
//    else if(state_machine == 2)
//    {
//          if(RXData == 0x0d)       // 接收到帧头第三个魔法数
//              state_machine = 3;
//         else
//              state_machine = 0;   // 状态机复位
//    }
//    else if(state_machine == 3)
//    {
//          if(RXData == 0x0c)       // 接收到帧头第四个魔法数
//          {
//              state_machine = 0;
//              return 1;
//          }
//          state_machine = 0;        // 状态机复位
//    }
//    return 0;
//}

////=========================================================
//// 数据解析函数
//// UART每接收到一个字节,都会执行一次调用这个函数;
//// 即连续的4个字节是: 0x0b,0x0a,0x0d,0x0c
////=========================================================
//void mmWavesensor_IWR1443BOOST_Data_lite_version_receiveData(void)
//{
//    if(RXDataflag == 1)//串口接收数据标志位，串口中断每收到一个字节数，该位置1；
//    {
//        RXDataflag = 0;

//        //设置LED指示灯亮
//        //MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);

//        if(parseMagicCodeflag)
//        {
//            //解析数据帧，获取到帧头, 返回1，开始解析数据头
//            parseHeaderflag = parseMagicCode();
//            if(parseHeaderflag)
//            {
//                parseMagicCodeflag = 0;
//            }
//            uart1_cnt = 0;
//        }

//        if(parseHeaderflag) //解析数据头标志位
//        {
//            if(uart1_cnt == 4) //Header的4个字节数据；    numDetectedObj 2字节 xyzQFormat 2字节
//            {
//                my_header.numDetectedObj = uart1_buf[1] << 8 |  uart1_buf[0];          //LSB串口模式，高低字节组成一个16bit数
//                if(my_header.numDetectedObj > maxDetectedObjectsNum)
//                    my_header.numDetectedObj = 0;
//                my_header.xyzQFormat     = uart1_buf[3] << 8 |  uart1_buf[2];
//                DetectedObjects_cnt = my_header.numDetectedObj * 12;                   //每个目标结构有6个参数，每个参数2字节，共12字节；
//                parseHeaderflag = 0;
//                parseDetectedObjectsflag = 1;
//                uart1_cnt = 0;
//            }
//        }
//        if(parseDetectedObjectsflag)  //解析检测目标数据结构标志位
//        {
//            if(uart1_cnt == DetectedObjects_cnt)
//            {
//                if(my_header.numDetectedObj == 0)
//                {
//                    memset((void *)uart1_buf, 0, my_header.numDetectedObj * 12);
//                    memset((void *)detectedObj, 0, sizeof(MmwDemo_detectedObj)*maxDetectedObjectsNum);
//                }

//                uint8_t i=0;
//								int y_cnt = 0;
//								int angle_cnt = 0;
//								distancey_sum = 0;
//								angle_sum = 0;
//								distancez_sum = 0;
//                for(i=0;i<my_header.numDetectedObj;i++)
//                {
//                    detectedObj[i].range      = uart1_buf[i*12+1]  << 8 | uart1_buf[i*12+0];
//                    detectedObj[i].speed    = uart1_buf[i*12+3]  << 8 | uart1_buf[i*12+2];
//                    detectedObj[i].angle    = uart1_buf[i*12+5]  << 8 | uart1_buf[i*12+4];
//                    detectedObj[i].x          = uart1_buf[i*12+7]  << 8 | uart1_buf[i*12+6];
//                    detectedObj[i].y          = uart1_buf[i*12+9]  << 8 | uart1_buf[i*12+8];
//                    detectedObj[i].z          = uart1_buf[i*12+11] << 8 | uart1_buf[i*12+10];
//										if(detectedObj[i].y > MIN_DST_Y*512 && detectedObj[i].y < MAX_DST_Y*512)  //y轴选定范围
//										{
//											if((detectedObj[i].z > MIN_DST_Z*512 && detectedObj[i].z < MAX_DST_Y*512))  //z轴选定范围
//											{
//												if(detectedObj[i].x > MIN_DST_X*512 && detectedObj[i].x < MAX_DST_X*512)  //x轴选定范围
//												{
//													distancey_sum += detectedObj[i].y / 512.0f;  //求和
//													y_cnt ++;  //计数
//													if(detectedObj[i].angle > MIN_ANGLE && detectedObj[i].angle < MAX_ANGLE && detectedObj[i].angle != 0)  //角度选定范围
//													{
//														angle_sum += detectedObj[i].angle;
//														angle_cnt ++;
//													}													
//												}
//											}
//										}
//                }
//								if(distancey_sum != 0)
//									distancey = (float) 1.0 * distancey_sum / y_cnt;  //取平均
//								else
//									distancey = DST_Y;
//								if(angle_sum != 0)
//									angle = (float) 1.0 * angle_sum / angle_cnt;  //取平均
//								else 
//									angle = 0;
//								
//                for(i=my_header.numDetectedObj;i<maxDetectedObjectsNum;i++)
//                {
//                    detectedObj[i].range      = 0;
//                    detectedObj[i].speed    = 0;
//                    detectedObj[i].angle    = 0;
//                    detectedObj[i].x          = 0;
//                    detectedObj[i].y          = 0;
//                    detectedObj[i].z          = 0;
//                }
//								Indx++;
//                //将数据发送到PC端显示
//                //printf("index %d    range %.4f    speed %.4f    angle %d    x %.4f    y %.4f    z %.4f\n\r",  Indx++, detectedObj[0].range/512.0, detectedObj[0].speed/512.0, detectedObj[0].angle, detectedObj[0].x/512.0, detectedObj[0].y/512.0, detectedObj[0].z/512.0);  //串口打印输出X,Y,Z的十进制数值

//                parseDetectedObjectsflag = 0;
//                parseMagicCodeflag = 1;
//                uart1_cnt = 0;
//             }
//         }

//        //LED灯熄灭
//        //GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
//     }

//}

////=========================================================
//// printf重定向的相关操作
//// 已经修改好,可以不用动.
//// 在正式的代码中,如果不需要使用printf函数,可以直接注释
////=========================================================
////int fputc(int ch, FILE *f)
////{
////	/*发送一个字节数据到串口*/
////	USART_SendData(USART2, (uint8_t) ch);
////	
////	/*等待发送完毕*/
////	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
////	return (ch);
////}

////int fputs(const char *ptr, FILE *p)
////{
////	unsigned int i, len;
////	len = strlen(ptr);
////  for(i = 0; i < len; i++)
////  {
////    USART_SendData(USART2, (uint8_t)ptr[i]);
////  }

////  return len;
////}

////int fgetc(FILE *f)
////{
////	/*等待串口输入数据*/
////	while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);
////	return (int)USART_ReceiveData(USART2);
////}

////void Usart_SendByte(USART_TypeDef* pUSARTx, uint8_t data)
////{
////	USART_SendData(pUSARTx, data);
////	while(USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
////}

////void Usart_SendArray(USART_TypeDef* pUSARTx, uint8_t *array, uint8_t num)
////{
////	uint8_t i;
////	for(i = 0; i < num; i++)
////	{
////		Usart_SendByte(pUSARTx, array[i]);
////	}
////	while(USART_GetFlagStatus(pUSARTx, USART_FLAG_TC) == RESET);
////}
////int fputc(int _c, register FILE *_fp)
////{
////  UART_transmitData(EUSCI_A0_BASE, (unsigned char) _c);
////  return((unsigned char)_c);
////}

////int fputs(const char *_ptr, register FILE *_fp)
////{
////  unsigned int i, len;

////  len = strlen(_ptr);

////  for(i=0 ; i<len ; i++)
////  {
////    UART_transmitData(EUSCI_A0_BASE, (unsigned char) _ptr[i]);
////  }

////  return len;
////}

//#endif


