/******************************************************************************
 ** \file hc32_debug.c
 **
 **  Copyright Fuzhou Internet of Things Open Lab.
 **
 **	 Author :fiot-lab
 **
 **  DESCRIPTION: print Log information
 **
 ******************************************************************************/
 
/******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_debug.h"
#include "hc32_bc35.h"
#include "hc32_pcnt.h"
/*******************************************************************************
 * global variable - (global)  and static variable - (static) 
 ******************************************************************************/
static fiot_lab_log_e g_fiot_lab_log_level = LOG_DEBUG;//Initialize Debug Level
volatile static uint8_t u8RxData[48];//Serial Receive Log Data
volatile static uint8_t u8TxData[]={0x6D,0xAC,0x00,0xFF,0x0A,0x00,0x26,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCF,0x68};//Use Serial DownLoad
volatile static uint8_t u8TxCnt=0;	//Send Length
volatile static uint8_t u8RxCnt=0;	//Receive Length
uint8_t u8LogData[32];			 	//Serial Receive Log Data Buff
uint16_t LOG_RX_STA=0;       		//Serial Receive Log Data Status
uint8_t u8DeviceLogData = 0; 		//Serial Receive PC Data Flag
//Log Print Infomation Level,Initialize Debug Level.
static const char *g_log_names[] =
{
    "FIOT-DEBUG",
    "FIOT-INFO",
    "FIOT-WARN",
    "FIOT-ERROR",
    "FIOT-FATAL",
};

/******************************************************************************
  ** \brief  UART 初始化配置
  ** 
  ** @param  NULL
  ** 
  ** \retval NULL
******************************************************************************/ 
void HC32_DEBUG_Init(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    stc_uart_cfg_t    stcCfg;
    DDL_ZERO_STRUCT(stcCfg);    
    DDL_ZERO_STRUCT(stcGpioCfg);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE); //使能GPIO时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart2,TRUE);///<使能uart2时钟 
    ///<TX
    stcGpioCfg.enDir = GpioDirOut;
    Gpio_Init(GpioPortD, GpioPin14, &stcGpioCfg);
    Gpio_SetAfMode(GpioPortD, GpioPin14, GpioAf1);    //PD14 URART2_TX
    ///<RX
    stcGpioCfg.enDir = GpioDirIn;
    Gpio_Init(GpioPortD, GpioPin13, &stcGpioCfg);
    Gpio_SetAfMode(GpioPortD, GpioPin13, GpioAf1);    //PD13 URART2_RX	
	
    ///<UART Init
    stcCfg.enRunMode        = UartMskMode1;          ///<模式1
    stcCfg.enStopBit        = UartMsk1bit;           ///<1bit停止位
    stcCfg.enMmdorCk        = UartMskDataOrAddr;     ///<偶校验
    stcCfg.stcBaud.u32Baud  = 115200;                ///<波特率115200
    stcCfg.stcBaud.enClkDiv = UartMsk8Or16Div;       ///<通道采样分频配置
    stcCfg.stcBaud.u32Pclk  = Sysctrl_GetHClkFreq(); ///<获取外设时钟(HCLK)频率值
    Uart_Init(M0P_UART2, &stcCfg);                   ///<串口初始化 
	
    ///<UART中断使能	
    Uart_ClrStatus(M0P_UART2,UartRC);                ///<清接收请求
    Uart_ClrStatus(M0P_UART2,UartTC);                ///<清除发送请求
    Uart_EnableIrq(M0P_UART2,UartRxIrq);             ///<使能接收中断    
    EnableNvic(UART0_2_IRQn, IrqLevel1, TRUE);       ///<中断使能	
}
/******************************************************************************
  ** \brief  UART 清除配置
  ** 
  ** @param  NULL
  ** 
  ** \retval NULL
******************************************************************************/ 
void HC32_DEBUG_Deinit(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    stc_uart_cfg_t    stcCfg;	
	
    stcGpioCfg.enDir = GpioDirIn;
    Gpio_Init(GpioPortD, GpioPin14, &stcGpioCfg);	
	
    stcGpioCfg.enDir = GpioDirIn;
    Gpio_Init(GpioPortD, GpioPin13, &stcGpioCfg);	
    ///<UART Init
    Uart_Init(M0P_UART2, &stcCfg);  
    Uart_DisableIrq(M0P_UART2,UartRxIrq);             ///<使能接收中断    
    EnableNvic(UART0_2_IRQn, IrqLevel1, FALSE);       ///<中断使能		
}
/********************************************************************************
  ** \brief  FIOT_LAB_LOG_PARSER
  ** 
  ** @param  uint8_t *p_data, uint16_t length
  ** 
  ** \retval null
******************************************************************************/ 
uint8_t FIOT_LAB_LOG_PARSER(void)
{
	if(LOG_RX_STA &0x8000){
		u8RxCnt = 0;
		LOG_RX_STA = 0;
		if(0 != strstr((char *)u8RxData,"ATI")){
			return 2;
		}else if(0 != strstr((char *)u8RxData,"AT+NRB")){//NB reboot
			return 3;
		}else if(0 != strstr((char *)u8RxData,"AT+NBAND?")){//Get NB Band
			return 4;
		}else if(0 != strstr((char *)u8RxData,"AT+CCLK")){	//DATATIME
			return 5;
		}else if(0 != strstr((char *)u8RxData,"AT+NUESTATS")){//Get NB Status	
			return 6;
		}else if(0 != strstr((char *)u8RxData,"AT+NCCID")){//Get NB CCID	
			return 7;
		}else if(0 != strstr((char *)u8RxData,"AT+CGSN")){//Get NB IMEI	
			return 8;
		}else if(0 != strstr((char *)u8RxData,"AT+CIMI")){//Get NB IMSI
			return 9;
		}else if(0 != strstr((char *)u8RxData,"AT+CGATT?")){//Get NB Network Status
			return 10;
		}else if(0 != strstr((char *)u8RxData ,"AT+CSQ")){//Get NB Signal Strength
			return 11;
		}else if(0 != strstr((char *)u8RxData ,"AT+CEREG?")){//Get NB Network Register Status
			return 12;
		}else if(0 != strstr((char *)u8RxData,"AT+BATTERY?")){//Get Device Battery
			return 50;
		}else if(0 != strstr((char *)u8RxData,"AT+REDEVICE")){//Device Reboot
			return 51;
		}else if(0 != strstr((char *)u8RxData,"AT+NCDP")){//Set PM Mode
			return 52;		
		}else if(0 != strstr((char *)u8RxData,"AT+CTM2MREG")){//Register Plat Info
			return 53;
		}else if(0 != strstr((char *)u8RxData,"AT+CTM2MUPDATE")){//Updata Plat Info
			return 54;		
		}else if(0 != strstr((char *)u8RxData,"AT+NMGS")){//Send Data to Oc Plat
			return 55;			
		}else if(0 != strstr((char *)u8RxData,"AT+CTM2MDEREG")){//Delete Plat Info
			return 56;
		}else if(0 != strstr((char *)u8RxData,"AT+ECBCINFO")){//Get NB Neighbor info
			return 57;
		}else if(0 != strstr((char *)u8RxData,"AT+BEEP=1")){//Open BEEP
			return 58;
		}else if(0 != strstr((char *)u8RxData,"AT+BEEP=0")){//Close Beep
			return 59;
		}else if(0 != strstr((char *)u8RxData,"AT+NBPWR=1")){//Open NB Power
			return 60;
		}else if(0 != strstr((char *)u8RxData,"AT+NBPWR=0")){//Close NB Power
			return 61;			
		}else if(0 != strstr((char *)u8RxData,"AT+SERST=1")){//Open SE Rst
			return 62;
		}else if(0 != strstr((char *)u8RxData,"AT+SERST=0")){//Close SE Rst
			return 63;
		}else if(0 != strstr((char *)u8RxData,"AT+NBRST=1")){//Open NB Rst
			return 64;
		}else if(0 != strstr((char *)u8RxData,"AT+NBRST=0")){//Close NB Rst
			return 65;
		}else if(0 != strstr((char *)u8RxData,"AT+WP=1")){//Open WP 
			return 66;
		}else if(0 != strstr((char *)u8RxData,"AT+WP=0")){//Close WP
			return 67;
		}else if(0 != strstr((char *)u8RxData,"AT+BAT=1")){//Open Bat 
			return 68;
		}else if(0 != strstr((char *)u8RxData,"AT+BAT=0")){//Close Bat
			return 69;
		}else if(0 != strstr((char *)u8RxData,"AT+WAKE=1")){//Open Wake
			return 70;
		}else if(0 != strstr((char *)u8RxData,"AT+WAKE=0")){//Close Wake
			return 71;			
		}else if(0 != strstr((char *)u8RxData,"AT+PWM=1")){//Open Pwm
			return 72;
		}else if(0 != strstr((char *)u8RxData,"AT+PWM=0")){//Close Pwm
			return 73;
		}else if(0 != strstr((char *)u8RxData,"AT+EEPROM=CHECK")){//Check EEprom
			return 74;
		}else if(0 != strstr((char *)u8RxData,"AT+KEY=READ")){//Read Key
			return 75;			
		}else if(0 != strstr((char *)u8RxData,"AT+SECLK=1")){//Open CLK
			return 76;
		}else if(0 != strstr((char *)u8RxData,"AT+SECLK=0")){//Close CLK
			return 77;			
		}else if(0 != strstr((char *)u8RxData,"AT+BATTERY=READ")){//Read Battery
			return 78;
		}else if(0 != strstr((char *)u8RxData,"AT+SLEEP=2")){//Read Battery
			return 79;	
		}else if(0 != strstr((char *)u8RxData,"AT+PCNT")){//Read PCNT
			return 80;	
		}else if(0 != strstr((char *)u8RxData,"AT+ECCGSN")){//Modify IMEI
			char *p = 0,*p1 = 0;
			char tmp[16] = {0};
			if(0 != (p = strstr((char *)u8RxData,"AT+ECCGSN=\"IMEI\","))){
				if(0 != (p1 = strstr((char *)p,"\r\n"))){
					if(p && p1){
						p+=strlen("AT+ECCGSN=\"IMEI\",");
						sscanf((char *)p,"\"%15s\"",tmp);
						if(sizeof(tmp) == 16){
							if(Bc35UserModifyIMEISend(tmp)){
								FIOT_LAB_LOG(LOG_INFO,"HC32 Modify IMEI Succeed!\r\n");
							}else{
								FIOT_LAB_LOG(LOG_INFO,"HC32 Modify IMEI False!\r\n");					
							}
						}
					}
				}
			}
			return 81;				
		}else if(0 != strstr((char *)u8RxData,"AT+WARN")){//Send DATA
			return 82;
		}else if(0 != strstr((char *)u8RxData,"AT")){//Test OK
			gPcntWaterAmount+=10;
			return 1;
		}
	}
	return 0;
}
/******************************************************************************
  ** \brief  UART2中断服务
  ** 
  ** @param  null
  ** 
  ** \retval null
******************************************************************************/ 
void Uart2_IRQHandler(void)
{
	uint8_t Res = 0;
    if(Uart_GetStatus(M0P_UART2, UartRC))         //UART2数据接收
    {
        Uart_ClrStatus(M0P_UART2, UartRC);        //清中断状态位
		Res = Uart_ReceiveData(M0P_UART2);
        u8RxData[u8RxCnt] = Res;   				//接收数据字节
        u8RxCnt++; 
        if(u8RxCnt >= 18)
        {    
            if ((u8RxData[0]==0x6D)&&(u8RxData[1]==0xAC)&&(u8RxData[6]==0x26)&&(u8RxData[16]==0xA6) 
										&&(u8RxData[17]==0xDA)) //是APP更新帧
            {
			u8RxCnt = 0;
			for(uint32_t i=0;i<18;i++)
			{
				Uart_SendDataPoll(M0P_UART2,u8TxData[i]); //查询方式发送数据
			}
			//boot para区域标记值,通知BootLoader更新程序
			Flash_SectorErase(0xF00);
			Flash_WriteWord(0xF00, 0x67890123);
							NVIC_SystemReset();  //复位软件MCU
			}                    
		}
		//用于接收上位机命令
		if((LOG_RX_STA&0x8000)==0)//接收未完成
		{
			if(LOG_RX_STA&0x4000)//接收到了0x0d
			{
				if(Res!=0x0a)LOG_RX_STA=0;//接收错误,重新开始
				else 
				{
					LOG_RX_STA|=0x8000;	//接收完成了 
					u8DeviceLogData = 1;
				}
			}
			else //还没收到0X0D
			{	
				if(Res==0x0d)LOG_RX_STA|=0x4000;
				else
				{
					u8LogData[LOG_RX_STA&0X3FFF]=Res ;
					LOG_RX_STA++;
					if(LOG_RX_STA>(FIOT_LAB_LOG_BUF_SIZE-1))LOG_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}
	}
}

/********************************************************************************
  ** \brief  UartDebugSend
  ** 
  ** @param  (Want to Send Data,Data Length)
  ** 
  ** \retval Null
******************************************************************************/ 
void UartDebugSend(uint8_t *pBuf, uint32_t gLen){
	for(uint32_t t = 0;t < gLen;t++)
	{
		 M0P_UART2->SBUF_f.DATA = pBuf[t];//Send Data
		 while(!Uart_GetStatus(M0P_UART2,UartTC));//Wati UartTC Set Bit 
		 Uart_ClrStatus(M0P_UART2, UartTC);//Clear UartTC Bit
	}	 
}
/********************************************************************************
  ** \brief  Set Log Print Level
  ** 
  ** @param  Set Print Log Level
  ** 
  ** \retval Null
******************************************************************************/ 
void fiot_lab_set_log_level(fiot_lab_log_e level)
{
    g_fiot_lab_log_level = level;
}
/********************************************************************************
  ** \brief  Get Log Print Level
  ** 
  ** @param  Null
  ** 
  ** \retval Return Now Log Print Level
******************************************************************************/ 
fiot_lab_log_e fiot_lab_get_log_level(void)
{
    return g_fiot_lab_log_level;
}
/********************************************************************************
  ** \brief  Get Log String Print Level
  ** 
  ** @param  Want to Query Log Print Level
  ** 
  ** \retval Want to Query Log Print Level OR "UNKNOW"
******************************************************************************/ 
static const char *fiot_lab_get_log_level_name(fiot_lab_log_e log_level)
{
    if (log_level >= LOG_MAX)
    {
        return "UNKOWN";
    }

    return g_log_names[log_level];
}
/********************************************************************************
  ** \brief  Set Log Print Level Hearder
  ** 
  ** @param  Set Print Log level
  ** 
  ** \retval Return Set Log level Hearder
******************************************************************************/ 
char *fiot_lab_log_format_prefix(fiot_lab_log_e level)
{
//	stc_rtc_time_t readtime;	
	
	static char prefix_buf[FIOT_LAB_LOG_BUF_SIZE];

	memset(prefix_buf, 0, FIOT_LAB_LOG_BUF_SIZE);
	
//	Rtc_ReadDateTime(&readtime);
	
//	snprintf(prefix_buf, FIOT_LAB_LOG_BUF_SIZE, "[%s][%04d-%02d-%02d %02d:%02d:%02d]", \
//																							fiot_lab_get_log_level_name(level), \
//																							BCD2DEC(readtime.u8Year) + 2000, BCD2DEC(readtime.u8Month), BCD2DEC(readtime.u8Day),  \
//																							BCD2DEC(readtime.u8Hour), BCD2DEC(readtime.u8Minute), BCD2DEC(readtime.u8Second));
	snprintf(prefix_buf, FIOT_LAB_LOG_BUF_SIZE, "[%s]",fiot_lab_get_log_level_name(level));	
	
	return prefix_buf;
}
/********************************************************************************
  ** \brief  String to HEX
  ** 
  ** @param  (Data Buff, Data Length)
  ** 
  ** \retval null
******************************************************************************/ 
void fiot_lab_log_hexdump(uint8_t *p_data, uint16_t length)
{
	uint16_t i, j, line = 0;
	
	uint8_t print_buf[48 + 1];
		
	if (NULL == p_data || length == 0)
		return;

	line = length / 16;
	if (line != 0)
	{
		for (i = 0; i < line; i++)
		{
			memset(print_buf, 0, sizeof(print_buf));
			
			for (j = 0; j < 16; j++)
			{
				snprintf((char *)print_buf + j * 3, sizeof(print_buf), "%02X ", p_data[i * 16 + j]);
			}
			FIOT_LAB_LOG(LOG_INFO, "%s\r\n", print_buf);
		}
	}

	if (length % 16 != 0)
	{
		memset(print_buf, 0, sizeof(print_buf));
				
		for (j = 0; j < length % 16; j++)
		{
			snprintf((char *)print_buf + j * 3, sizeof(print_buf), "%02X ", p_data[line * 16 + j]);
		}
		FIOT_LAB_LOG(LOG_INFO, "%s\r\n", print_buf);
	}
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/










