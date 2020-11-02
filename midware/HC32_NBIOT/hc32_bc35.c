/******************************************************************************
 ** \file hc32_bc35.c
 **
 **  Copyright Fuzhou Internet of Things Open Lab.
 **
 **	 Author :Fiot-lab
 **
 **  DESCRIPTION: This file contains NB Configure and Control.
 **
 ******************************************************************************/
#include "hc32_bc35.h"
#include "hc32_key.h"
#include "hc32_lptimer.h"
#include "hc32_crc.h"
#include "hc32_beep.h"
#include "hc32_se.h"
#include "hc32_eeprom.h"
#include "hc32_battery.h"
#include "hc32_valve.h"
#include "hc32_pcnt.h"

uint8_t gBc35UartRecvBuf[Bc35UartRecvBufMaxSize];//Serial Receive Buff

uint8_t gBc35RecvBuf[Bc35UartRecvBufMaxSize];//NBiot Receive Buff

uint8_t gBc35UartSendBuf[Bc35UartRecvBufMaxSize];//NBiot Send Buff

uint8_t gBc35UserLeakPack[64] = {0};//存储补包的数据

Enum_BC35STATUS gBc35UserPrevStatus = BC35STATUS_NONE;//NBiot Status Init

Enum_BC35STATUS gBc35UserNextStatus = BC35STATUS_NONE;//NBiot Status Init

static char gBc35UserStatusIndex = 0;//NBiot Status Index

int gBc35UserQlwConnectStatus = 0;//NBiot Connect Status

int gBc35UserWaitResponse = 0;//Wait Response 20 Sec

char gBc35UserGetDataStatus = 0;//NBiot Get Data Status Flag

char gBc35UserGetDataError = 0;//NBiot Get Data Error Flag

char gBc35UserSendDataRepetition = 0;//NBiot Send Data to OC.if Send Data Faile,Send Data Repetition.

char gBc35UserSendDataStatus = 0;//NBiot Send Data to OC Status Flag

char u8Nb35SleepStatus = 0;//Nbiot Enter Sleep Status 1:Sleep 0:Normal

boolean_t gBc35UserImmediatelyWarn = 0;//Immediately Warn Mode   0111   3:OverCurrent  2:FanCurrent  1:devicewarn

boolean_t gNormalOverCurrentWarn = 0;//Immediately Warn Mode

char gBc35UserReceiveCmdFromOc = 0;//Receive Cmd From Oc Plat

uint32_t gBc35UserOverCurrentTime = 0;//OverCurrent Check Time

uint32_t gBc35UserConCurrentTime = 30;//ConCurrent Check Time

uint32_t gBc35UserSendDataFrequency = 10;//Send Ok DATA Frequency,MIN:minute

uint32_t gBc35UserLeakPackage = 0;//记录补包数

BC35_DateTime_T gBc35DataTime = {2010, 1, 1, 0, 0, 0, 0};

BC35_Qccid_T gBc35Qccid = {0};

BC35_Nngr_T gBc35Nngr = {0};

BC35_Qimei_T gBc35Qimei = {0};

BC35_Qimsi_T gBc35Qimsi = {0};

BC35_Version_T gBc35Version = {0};

BC35_NueStats_T gBc35NueStats = {0};

BC35_Serv_T gBc35Server = {0};

BC35_Cmd_Parser_T gBc35CmdParser = {0};

BC35_Band_T gBc35Band = {0};

BC35_Dat_Send gBc35Data = {0};

BC35_BcInfo_T gBc35InfoNeighbor = {0};

#ifdef HC32_BC28_SUPPORTS

char gBc28UserWaterDevNum[4] = {0};//Water Device Num addr

static char gBc28UserQlwOcip[4] = {0};

static short gBc28UserQlwPort = 5683;

BC28_NueStats_T gBc28NueStats = {0};

BC28_period_T gBc28period = {0};

#endif
/******************************************************************************
 * Local variable definitions ('static')                                      *
 ******************************************************************************/
volatile static uint8_t u8RxData;
volatile static uint8_t u8TxCnt=0;
volatile static uint16_t u16RxCnt=0;
uint8_t u8OcData[128];
uint16_t gBC35_RX_STA=0;       //接收状态标记

/********************************************************************************
  ** \brief  BC35配置函数
  ** 
  ** @param  null
  ** 
  ** \retval null
******************************************************************************/ 
void HC32_BC35_GPIO(void){
    stc_gpio_cfg_t stcGpioCfg;
    
    ///< 打开GPIO外设时钟门控
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); 
    
    ///< 端口方向配置
    stcGpioCfg.enDir = GpioDirOut;
    ///< 配置上下拉
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
    
    ///<  NB Power关闭
    Gpio_SetIO(USER_NBPWR_PORT, USER_NBPWR_PIN);
    ///<  NB RST关闭
    Gpio_ClrIO(USER_NBRST_PORT, USER_NBRST_PIN);
    ///<  NB WAKE关闭
    Gpio_ClrIO(USER_NBWAKE_PORT, USER_NBWAKE_PIN);
	
    ///< GPIO IO  NB Power端口初始化
    Gpio_Init(USER_NBPWR_PORT, USER_NBPWR_PIN, &stcGpioCfg);
    ///< GPIO IO  NB RST端口初始化
    Gpio_Init(USER_NBRST_PORT, USER_NBRST_PIN, &stcGpioCfg);
    ///< GPIO IO  NB WAKE端口初始化
    Gpio_Init(USER_NBWAKE_PORT, USER_NBWAKE_PIN, &stcGpioCfg);

	HC32_NBPWR_Hight();
	
	HC32_NBWAKE_Low();
	
	HC32_NBRST_Low();
	delay1ms(100);
	HC32_NBRST_Hight();	
	delay1ms(100);
	HC32_NBRST_Low();		
}
void HC32_TXRX_Out(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    ///< 打开GPIO外设时钟门控
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); 
    
    ///< 端口方向配置
    stcGpioCfg.enDir = GpioDirOut;
    ///< 配置上下拉
    stcGpioCfg.enPu = GpioPuEnable;
    stcGpioCfg.enPd = GpioPdDisable;
    
//    ///< LED关闭
    Gpio_ClrIO(GpioPortD, GpioPin5);
    Gpio_ClrIO(GpioPortD, GpioPin6);
	
//    ///< GPIO IO LED端口初始化
    Gpio_Init(GpioPortD, GpioPin5, &stcGpioCfg);
    Gpio_Init(GpioPortD, GpioPin6, &stcGpioCfg);
	
	Gpio_WriteOutputIO(GpioPortD,GpioPin5,1);
	Gpio_WriteOutputIO(GpioPortD,GpioPin6,1);	
}
void HC32_BC35_Init(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    stc_uart_cfg_t    stcCfg;

    DDL_ZERO_STRUCT(stcCfg);    
    DDL_ZERO_STRUCT(stcGpioCfg);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE); //使能GPIO时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart1,TRUE);///<使能uart1时钟 
	
    ///<TX
    stcGpioCfg.enDir = GpioDirOut;
    Gpio_Init(GpioPortD, GpioPin5, &stcGpioCfg);
    Gpio_SetAfMode(GpioPortD, GpioPin5, GpioAf1);    //PD05 URART1_TX
    
    ///<RX
    stcGpioCfg.enDir = GpioDirIn;
    Gpio_Init(GpioPortD, GpioPin6, &stcGpioCfg);
    Gpio_SetAfMode(GpioPortD, GpioPin6, GpioAf1);    //PD06 URART1_RX	

//    ///<UART Init
    stcCfg.enRunMode        = UartMskMode1;          ///<模式3
    stcCfg.enStopBit        = UartMsk1bit;           ///<1bit停止位
    stcCfg.enMmdorCk        = UartMskOdd;     			 ///<无偶校验
	#ifdef HC32_BC28_SUPPORTS
		stcCfg.stcBaud.u32Baud  = 9600;                ///<波特率9600
	#endif
	#ifdef HC32_L161_SUPPORTS
		stcCfg.stcBaud.u32Baud  = 115200;                ///<波特率115200
	#else
		stcCfg.stcBaud.u32Baud  = 9600;  		
	#endif
    stcCfg.stcBaud.enClkDiv = UartMsk8Or16Div;       ///<通道采样分频配置
    stcCfg.stcBaud.u32Pclk  = Sysctrl_GetHClkFreq(); ///<获取外设时钟(PCLK)频率值
    Uart_Init(M0P_UART1, &stcCfg);                   ///<串口初始化

    ///<UART中断使能	
    Uart_ClrStatus(M0P_UART1,UartRC);                ///<清接收请求
    Uart_ClrStatus(M0P_UART1,UartTC);                ///<清除发送请求
    Uart_EnableIrq(M0P_UART1,UartRxIrq);             ///<使能接收中断
    EnableNvic(UART1_3_IRQn, IrqLevel1, TRUE);       ///<中断使能
		
	Bc35UserStatusInit();

//#ifdef HC32_LOG_SUPPORTS 
//		FIOT_LAB_LOG(LOG_INFO,"NBIOT(TX:PD5 RX:PD6) Baud: %d StopBit:%d MmdorCk:%d\r\n",stcCfg.stcBaud.u32Baud,stcCfg.enStopBit,stcCfg.enMmdorCk);
//#endif
}
void HC32_BC35_Deinit(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    stc_uart_cfg_t    stcCfg;

    DDL_ZERO_STRUCT(stcCfg);    
    DDL_ZERO_STRUCT(stcGpioCfg);
   
    ///<TX
    stcGpioCfg.enDir = GpioDirIn;
    Gpio_Init(GpioPortD, GpioPin5, &stcGpioCfg);
    
    ///<RX
    stcGpioCfg.enDir = GpioDirIn;
    Gpio_Init(GpioPortD, GpioPin6, &stcGpioCfg);
	
    Uart_Init(M0P_UART1, &stcCfg);                    ///<串口初始化
    Uart_DisableIrq(M0P_UART1,UartRxIrq);             ///<使能接收中断    		
    EnableNvic(UART1_3_IRQn, IrqLevel1, FALSE);       ///<中断使能
}
/********************************************************************************
  ** \brief  UART1中断服务函数
  ** 
  ** @param  null
  ** 
  ** \retval null
******************************************************************************/ 
void Uart1_IRQHandler(void)
{
    if(Uart_GetStatus(M0P_UART1, UartRC))         //UART1接收状态
    {
        Uart_ClrStatus(M0P_UART1, UartRC);        //清除接收标志
		gBc35UartRecvBuf[u16RxCnt++] = M0P_UART1->SBUF_f.DATA;
		if(u16RxCnt >= (Bc35UartRecvBufMaxSize-1)) u16RxCnt = 0;				
    }
}
#ifdef HC32_TIMESTAMP_SUPPORTS
/********************************************************************************
  ** \brief  UserDataTimeCheck(时间戳)
  ** 
  ** @param  null
  ** 
  ** \retval Unix Time
******************************************************************************/ 
static unsigned int Bc35UserDataTimeCheck(void){
	
    unsigned char month_day[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    unsigned char month_day_leap[] = {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
		unsigned int monDays[] = {0,31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    unsigned int gSecCount = 0;
		
    if(gBc35DataTime.sec >= 60){
        gBc35DataTime.sec -= 60;
        gBc35DataTime.min++;
    }
    if(gBc35DataTime.min >= 60){
        gBc35DataTime.min -= 60;
        gBc35DataTime.hour++;
    }
    if(gBc35DataTime.hour >= 24){
        gBc35DataTime.hour -= 24;
        gBc35DataTime.day++;
    }
    if((!(gBc35DataTime.year % 4) && (gBc35DataTime.year % 100)) || !(gBc35DataTime.year % 400)){
        if(gBc35DataTime.day > month_day_leap[gBc35DataTime.mouth]){
            gBc35DataTime.day -= month_day_leap[gBc35DataTime.mouth];
            gBc35DataTime.mouth++;
        }
    }else{
        if(gBc35DataTime.day > month_day[gBc35DataTime.mouth]){
            gBc35DataTime.day -= month_day[gBc35DataTime.mouth];
            gBc35DataTime.mouth++;
        }
    }
    if(gBc35DataTime.mouth > 12){
        gBc35DataTime.mouth -= 12;
        gBc35DataTime.year++;
    }
		gSecCount = (gBc35DataTime.year - 1970) * 365 * 24 * 3600 + (monDays[gBc35DataTime.mouth-1] + \
								 gBc35DataTime.day - 1) * 24 * 3600 + (gBc35DataTime.hour-8) * 3600 + \
								 gBc35DataTime.min * 60 + gBc35DataTime.sec;
		
		gSecCount += (gBc35DataTime.mouth > 2 && (gBc35DataTime.year % 4 == 0) && \
									(gBc35DataTime.year % 100 != 0 || gBc35DataTime.year % 400 == 0))*24*3600;	//闰月
		
		gSecCount += ((gBc35DataTime.year - 1970)/4 - (gBc35DataTime.year - 1970)/100 + \
									(gBc35DataTime.year - 1970)/400)*24*3600;		//闰年
		
		#ifdef HC32_LOG_SUPPORTS
		FIOT_LAB_LOG(LOG_INFO,"Unix Time:%u\r",gSecCount);
		#endif
    return gSecCount;
}
#endif
/********************************************************************************
  ** \brief  *Bc35UserModule
  ** 
  ** @param  char *buffer, char *RePly
  ** 
  ** \retval 返回查询的地址
******************************************************************************/ 
static char *Bc35UserModule(char *buffer, char *RePly){
    char *p = 0, *p1 = 0;
    if(!RePly) p = buffer;
    else if(0 != (p = strstr(buffer, RePly))){
        p += strlen(RePly);
        if(0 != (p1 = strstr(p, "\r"))) *p1 = 0;
        if(0 != (p1 = strstr(p, "\n"))) *p1 = 0;
        while(*p == ' ') p++;
    }
    return p;
}
/********************************************************************************
  ** \brief  UartIntRecvReset(初始化)
  ** 
  ** @param  null
  ** 
  ** \retval null
******************************************************************************/ 
void UartIntRecvReset(void){
	u16RxCnt = 0;
	memset(gBc35UartRecvBuf, 0, sizeof(gBc35UartRecvBuf));
	memset(gBc35RecvBuf, 0, sizeof(gBc35RecvBuf));
}
/********************************************************************************
  ** \brief  UartIntSend函数(发送数据函数)
  ** 
  ** @param  uint8_t *pBuf, uint32_t gLen
  ** 
  ** \retval null
******************************************************************************/ 
void UartIntSend(uint8_t *pBuf, uint32_t gLen){

		uint32_t t;
		for(t = 0;t < gLen;t++)		//循环发送数据
		{
#ifdef HC32_UART_NB_SUPPORTS
			M0P_UART1->SBUF_f.DATA = pBuf[t];
			while(!Uart_GetStatus(M0P_UART1,UartTC));
			Uart_ClrStatus(M0P_UART1, UartTC);
#else
      LPUart_SendDataIt(M0P_LPUART1, pBuf[t]);
			while(!LPUart_GetStatus(M0P_LPUART1,LPUartTC));
			LPUart_ClrStatus(M0P_LPUART1,LPUartTC);			
#endif
		}	 
}
/********************************************************************************
  ** \brief  Bc35UartUserSend函数(发送AT命令函数)
  ** 
  ** @param  想要发送的AT指令
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/ 
static int Bc35UartUserSend(char *cmd){
    if(cmd && strlen(cmd)){
        memset(gBc35UartSendBuf, 0, sizeof(gBc35UartSendBuf));
        strcpy((char *)gBc35UartSendBuf, cmd);
		UartIntRecvReset();
		UartIntSend(gBc35UartSendBuf, strlen(cmd));
		return 1;
    }
    return 0;
}
/********************************************************************************
  ** \brief  AT命令函数
  ** 
  ** @param  null
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/ 
static int Bc35UserATSend(void){
    return Bc35UartUserSend(BC35_AT);
}
static int Bc35UserAT(void){
	Bc35UserATSend();
	delay1ms(100);
    memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
    if(0 != strstr((char *)gBc35RecvBuf, BC35_OK_REPLY)){
			#ifdef	HC32_LOG_SUPPORTS
			FIOT_LAB_LOG(LOG_INFO,"OK\r");
			#endif
			return 1;
    }
    return 0;
}
/********************************************************************************
  ** \brief  AT重启命令函数
  ** 
  ** @param  null
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/
static int Bc35UserRebootSend(void){
		#ifdef HC32_L161_SUPPORTS	
    return Bc35UartUserSend(BC35_REBOOT_CRL);
		#endif
		#ifdef HC32_BC28_SUPPORTS	
    return Bc35UartUserSend(BC28_REBOOT_CRL);	
		#endif	
}
static int Bc35UserReboot(void){
    char *p = 0;
	Bc35UserRebootSend();
	delay1ms(100);
    memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
	#ifdef HC32_L161_SUPPORTS
    if(0 != (p = strstr((char *)gBc35RecvBuf, BC35_REBOOT_REPLY))){
   
    }
	#endif
	#ifdef HC32_BC28_SUPPORTS
    if(0 != (p = strstr((char *)gBc35RecvBuf, BC28_REBOOT_REPLY))){

    }		
	#endif
    if(p){
		#ifdef	HC32_LOG_SUPPORTS
		FIOT_LAB_LOG(LOG_INFO,"REBOOTING OK\r\n");
		#endif			
        return 1;
    }
    return 0;
}
/********************************************************************************
  ** \brief  AT进入SLEEP命令函数
  ** 
  ** @param  null
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/
#ifdef HC32_L161_SUPPORTS
static int Bc35UserEnterSleepSend(uint8_t mode_t){
	if(mode_t == 0)
		//Hibernate2 status
		return Bc35UartUserSend(BC35_SLEEP_WRITE("0"));
	else if(mode_t == 1)
		//Hibernate1 status
		return Bc35UartUserSend(BC35_SLEEP_WRITE("1"));
	else if(mode_t == 2)
		//Sleep2 status
		return Bc35UartUserSend(BC35_SLEEP_WRITE("2"));
	else if(mode_t == 3)
		//Sleep1 status
		return Bc35UartUserSend(BC35_SLEEP_WRITE("3"));
	return 0;
}
int Bc35UserEnterSleep(uint8_t mode_t){
    char *p = 0;
		Bc35UserEnterSleepSend(mode_t);
		delay1ms(50);
    memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
		FIOT_LAB_LOG(LOG_DEBUG,"Sleep:%s\r\n",gBc35RecvBuf);
		if(0 != (p = strstr(p, BC35_OK_REPLY))){
				return 1;
		}
    return 0;
}
#endif
/********************************************************************************
  ** \brief  AT获取IMEI命令函数
  ** 
  ** @param  null
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/
static int Bc35UserGetQimeiSend(void){
    return Bc35UartUserSend(BC35_IMEI_READ);
}
static int Bc35UserGetQimei(void){
    char *p = 0, *p1 = 0;
	if(gBc35Qimei.update == 1)
	{
		#ifdef	HC32_LOG_SUPPORTS
		FIOT_LAB_LOG(LOG_INFO,"CGSN:%s\r\n", gBc35Qimei.imei);
		#endif
		return 1;
	}
	Bc35UserGetQimeiSend();
	delay1ms(100);
    memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
    if(0 != (p = strstr((char *)gBc35RecvBuf, BC35_IMEI_REPLY))){
        if(0 != (p1 = strstr(p, BC35_OK_REPLY))){
            p = Bc35UserModule((char *)gBc35RecvBuf, BC35_IMEI_REPLY);
        }
    }	
	#ifdef HC32_BC28_SUPPORTS
    if(p && p1 && strlen(p) == 15){
		if(strcmp(p, gBc35Qimei.imei)){
			memset(gBc35Qimei.imei, 0, sizeof(gBc35Qimei.imei));
			strcpy(gBc35Qimei.imei, p);	
			gBc35Qimei.update = 1;
			#ifdef	HC32_LOG_SUPPORTS
			FIOT_LAB_LOG(LOG_INFO,"CGSN:%s\r\n", gBc35Qimei.imei);
			#endif
			return 1;
		}
	}		
	#endif
	#ifdef HC32_L161_SUPPORTS
    if(p && p1 && strlen(p) == 17){
		sscanf((char *)p,"\"%15s\"",gBc35Qimei.imei);
		gBc35Qimei.update = 1;
		#ifdef	HC32_LOG_SUPPORTS
		FIOT_LAB_LOG(LOG_INFO,"CGSN:%s\r\n", gBc35Qimei.imei);
		#endif
		return 1;
    }
	#endif
    return 0;
}
/********************************************************************************
  ** \brief  AT获取临近小区命令函数
  ** 
  ** @param  null
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/
static int Bc35UserGetBcInfoSend(void){
    return Bc35UartUserSend(BC35_BCINFO_READ);
}
static int Bc35UserGetBcInfo(void){
    char *p = 0, *p1 = 0;
    int dst[15] = {0};
    unsigned char num = 0, i = 0;
		Bc35UserGetBcInfoSend();
		delay1ms(200);
		memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
    if(0 != (p = strstr((char *)gBc35RecvBuf, BC35_BCINFO_REPLY0))){
        if(0 != (p1 = strstr(p, BC35_OK_REPLY))){
        }
    }	
		if(p && p1)
		{
			FIOT_LAB_LOG(LOG_INFO, "%s\r", gBc35RecvBuf);
			while(0 != (p = strstr(p, BC35_BCINFO_REPLY1))){
			p += strlen(BC35_BCINFO_REPLY1);
      while(*p == ' '||*p == '\r'||*p == '\n') p++;
			if((0 != (p1 = strstr(p, "\r")))||(0 != (p1 = strstr(p, "\n")))){
					*p1 = 0;
					GetDstAndNumFromStrByCs(p, ",", &num, dst);				
					if(num >= 3){
							gBc35InfoNeighbor.Neighbor[i].Earfcn = atoi(p + dst[0]);
							gBc35InfoNeighbor.Neighbor[i].Pci = atoi(p + dst[1]);
							gBc35InfoNeighbor.Neighbor[i].Rsrp = atoi(p + dst[2]);
							gBc35InfoNeighbor.Neighbor[i].Rsrq = atoi(p + dst[3]);
							#ifdef	HC32_LOG_SUPPORTS
							FIOT_LAB_LOG(LOG_INFO,"Find Neighbor%d Earfcn %d Pci %d Rsrp %d Rsrq %d\r\n", i, gBc35InfoNeighbor.Neighbor[i].Earfcn, \
																																									     gBc35InfoNeighbor.Neighbor[i].Pci, \
																																											 gBc35InfoNeighbor.Neighbor[i].Rsrp, \
																																											 gBc35InfoNeighbor.Neighbor[i].Rsrq);	
							#endif
							i++;
							p = p1 + 1;
							if(i > sizeof(gBc35InfoNeighbor.Neighbor)/sizeof(gBc35InfoNeighbor.Neighbor[0])){
									break;
							}						
						}	
					}
				}
				gBc35InfoNeighbor.NeighborNo = i;
				if(gBc35InfoNeighbor.NeighborNo == 0)
				{
						memset(gBc35InfoNeighbor.Neighbor, 0, sizeof(gBc35InfoNeighbor.Neighbor));
				}
				return 1;
		}
		return 0;
}
/********************************************************************************
  ** \brief  AT获取CCID命令函数
  ** 
  ** @param  null
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/
static int Bc35UserGetQccidSend(void){
	#ifdef HC32_L161_SUPPORTS
    return Bc35UartUserSend(BC35_QCCID_READ);
	#endif
	#ifdef HC32_BC28_SUPPORTS
    return Bc35UartUserSend(BC28_QCCID_READ);	
	#endif
}
static int Bc35UserGetQccid(void){
    char *p = 0, *p1 = 0;
	if(gBc35Qccid.update == 1){
		#ifdef	HC32_LOG_SUPPORTS
		FIOT_LAB_LOG(LOG_INFO,"CCID:%s\r\n", gBc35Qccid.ccid);
		#endif
		return 1;
	}
	Bc35UserGetQccidSend();
	delay1ms(100);
	memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
	#ifdef HC32_L161_SUPPORTS		
    if(0 != (p = strstr((char *)gBc35RecvBuf, BC35_QCCID_REPLY))){
        if(0 != (p1 = strstr(p, BC35_OK_REPLY))){
            p = Bc35UserModule((char *)gBc35RecvBuf, BC35_QCCID_REPLY);
        }
    }	
	#endif
	#ifdef HC32_BC28_SUPPORTS
    if(0 != (p = strstr((char *)gBc35RecvBuf, BC28_QCCID_REPLY))){
        if(0 != (p1 = strstr(p, BC28_OK_REPLY))){
            p = Bc35UserModule((char *)gBc35RecvBuf, BC28_QCCID_REPLY);
        }
    }	
	#endif		
    if(p && p1){
        if(strcmp(p, gBc35Qccid.ccid)){
            memset(gBc35Qccid.ccid, 0, sizeof(gBc35Qccid.ccid));
            strcpy(gBc35Qccid.ccid, p);
			gBc35Qccid.update = 1;
			#ifdef	HC32_LOG_SUPPORTS
			FIOT_LAB_LOG(LOG_INFO,"CCID:%s\r\n", gBc35Qccid.ccid);
			#endif
        }
        return 1;
    }
    return 0;
}
/********************************************************************************
  ** \brief  AT获取IMSI命令函数
  ** 
  ** @param  null
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/
static int Bc35UserGetQimsiSend(void){
    return Bc35UartUserSend(BC35_IMSI_READ);
}
static int Bc35UserGetQimsi(void){
    char *p = 0, *p1 = 0;
	if(gBc35Qimsi.update == 1)
	{
		#ifdef	HC32_LOG_SUPPORTS
		FIOT_LAB_LOG(LOG_INFO,"IMSI:%s\r\n", gBc35Qimsi.imsi);
		#endif
		return 1;
	}
	Bc35UserGetQimsiSend();
	delay1ms(100);
	memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
    if(0 != (p = strstr((char *)gBc35RecvBuf, BC35_OK_REPLY))){
        p = (char *)gBc35RecvBuf;
        while((*p < 0x30)||(*p > 0x39)) p++;
        if(0 != (p1 = strstr(p, "\r"))) *p1 = 0;
        if(0 != (p1 = strstr(p, "\n"))) *p1 = 0;
    }	
    if(p && strlen(p) == 15){
        if(strcmp(p, gBc35Qimsi.imsi)){
            memset(gBc35Qimsi.imsi, 0, sizeof(gBc35Qimsi.imsi));
            strcpy(gBc35Qimsi.imsi, p);
			gBc35Qimsi.update = 1;
			#ifdef	HC32_LOG_SUPPORTS
			FIOT_LAB_LOG(LOG_INFO,"IMSI:%s\r\n", gBc35Qimsi.imsi);
			#endif
        }
        return 1;
    }
    return 0;
}
/********************************************************************************
  ** \brief  AT获取版本号命令函数
  ** 
  ** @param  null
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/
static int Bc35UserGetVerisonSend(void){
	#ifdef HC32_L161_SUPPORTS
    return Bc35UartUserSend(BC35_VERSION_READ);
	#endif
	#ifdef HC32_BC28_SUPPORTS
    return Bc35UartUserSend(BC28_VERSION_READ);	
	#endif
}
static int Bc35UserGetVerison(void){
    char *p = 0, *p1 = 0;
		if(gBc35Version.update == 1){
			#ifdef	HC32_LOG_SUPPORTS
			FIOT_LAB_LOG(LOG_INFO,"Revision:%s\r\n", gBc35Version.ver);
			#endif
			return 1;
		}
		Bc35UserGetVerisonSend();
		delay1ms(100);
		memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
		#ifdef HC32_L161_SUPPORTS
    if(0 != (p = strstr((char *)gBc35RecvBuf, BC35_VERSION_REPLY))){
        if(0 != (p1 = strstr(p, BC35_OK_REPLY))){
            p = Bc35UserModule((char *)gBc35RecvBuf, BC35_VERSION_REPLY);
        }
    }	
		#endif
		#ifdef HC32_BC28_SUPPORTS
    if(0 != (p = strstr((char *)gBc35RecvBuf, BC28_VERSION_REPLY))){
        if(0 != (p1 = strstr(p, BC28_OK_REPLY))){
            p = Bc35UserModule((char *)gBc35RecvBuf, BC28_VERSION_REPLY);
        }
    }			
		#endif
    if(p && p1){
        if(strcmp(p, gBc35Version.ver)){
            memset(gBc35Version.ver, 0, sizeof(gBc35Version.ver));
            strcpy(gBc35Version.ver, p);
						gBc35Version.update = 1;
						#ifdef	HC32_LOG_SUPPORTS
						FIOT_LAB_LOG(LOG_INFO,"Revision:%s\r\n", gBc35Version.ver);
						#endif
        }
        return 1;
    }
    return 0;
}
/********************************************************************************
  ** \brief  AT获取CEREG状态命令函数
  ** 
  ** @param  null
  ** 
  ** \retval Ok or Faile
******************************************************************************/
static int Bc35UserGetCeRegSend(void){
	  return Bc35UartUserSend(BC35_CEREG_READ);
}
static int Bc35UserGetCeReg(void){
    char *p = 0, *p1 = 0;
    int dst[10] = {0};
    unsigned char num = 0;
		Bc35UserGetCeRegSend();
		delay1ms(50);
		memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
    if(0 != (p = strstr((char *)gBc35RecvBuf, BC35_CEREG_REPLY))){
        if(0 != (p1 = strstr(p, BC35_OK_REPLY))){
            p = Bc35UserModule((char *)gBc35RecvBuf, BC35_CEREG_REPLY);
        }
		}
    if(p && p1){
			if(0 != (p1 = strstr(p, "+CEREG:"))){
            *p1 = 0;
            p1 += 7;
        }
        GetDstAndNumFromStrByCs(p, ",", &num, dst);		
				if(num == 1)
				{
					char n = atoi(p + dst[0]);
					char stat = atoi(p + dst[1]);
					#ifdef	HC32_LOG_SUPPORTS
					//network stat
					FIOT_LAB_LOG(LOG_INFO,"CEREG:%d,%d\r\n",n,stat);
					#endif
					return 1;
				}
		}
		return 0;
}
/********************************************************************************
  ** \brief  AT获取PSM状态命令函数
  ** 
  ** @param  null
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/
static int Bc35UserGetPsmModeSend(void){
    return Bc35UartUserSend(BC35_CPSMS_READ);
}
static int Bc35UserGetPsmMode(void){
    char *p = 0, *p1 = 0;
	Bc35UserGetPsmModeSend();
	delay1ms(50);
	memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
    if(0 != (p = strstr((char *)gBc35RecvBuf, BC35_CPSMS_REPLY))){
        if(0 != (p1 = strstr(p, BC35_OK_REPLY))){
            p = Bc35UserModule((char *)gBc35RecvBuf, BC35_CPSMS_REPLY);
        }
    }	
    if(p && p1){
        if(*p == '0'){
			#ifdef	HC32_LOG_SUPPORTS
			FIOT_LAB_LOG(LOG_INFO,"PSM:Disable the use of PSM\r");
			#endif
            return 1;
        }else if(*p == '1'){
			#ifdef	HC32_LOG_SUPPORTS
			FIOT_LAB_LOG(LOG_INFO,"PSM:Enable the use of PSM\r\n");
			#endif
            return 2;
        }else if(*p == '2'){
			#ifdef	HC32_LOG_SUPPORTS
			FIOT_LAB_LOG(LOG_INFO,"PSM:Disable the use of PSM and discard all parameters for PSM\r\n");	
			#endif					
            return 3;
        }else{
			#ifdef	HC32_LOG_SUPPORTS
			FIOT_LAB_LOG(LOG_ERROR,"PSM:Error the use of PSM\r\n");
			#endif
            return 0;
        }
    }
    return 0;
}
/********************************************************************************
  ** \brief  AT设置PSM状态命令函数
  ** 
	** @param  mode 0:Disable the use of PSM
	**         mode 1:Enable the use of PSM
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/
static int Bc35UerSetPsmModeSend(uint8_t mode_t){
	if(mode_t == 0)
		//Disable the use of PSM
		return Bc35UartUserSend(BC35_CPSMS_WRITE("0"));
	else if(mode_t == 1)
		//Enable the use of PSM
		return Bc35UartUserSend(BC35_CPSMS_WRITE("1"));
	else if(mode_t == 2)
		//Disable the use of PSM and discard all parameters for PSM or,
		//if available, reset to the default values.
		return Bc35UartUserSend(BC35_CPSMS_WRITE("2"));
	return 0;
}
static int Bc35UerSetPsmMode(uint8_t SetPsmMode){
	Bc35UerSetPsmModeSend(SetPsmMode);
	delay1ms(100);
	memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
    if(0 != strstr((char *)gBc35RecvBuf, BC35_OK_REPLY)){	
			return Bc35UserGetPsmMode();
    }	
	return 0;
}
static int Bc35UerSetPsmTimeSend(uint8_t time_t){
	
	char buf[36] = {0};
	sprintf(buf,"AT+CPSMS=1,,,01000001,00000001");
	//Disable the use of PSM
	return Bc35UartUserSend(buf);

	//return 0;
}
static int Bc35UerSetPsmTime(uint8_t SetPsmTime){
	Bc35UerSetPsmTimeSend(SetPsmTime);
	delay1ms(100);
	memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
    if(0 != strstr((char *)gBc35RecvBuf, BC35_OK_REPLY)){	
			return 1;
    }	
	return 0;
}


/********************************************************************************
  ** \brief  AT获取eDRX状态命令函数
  ** 
  ** @param  null
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/
static int Bc35UserGeteDRXSend(void){
    return Bc35UartUserSend(BC35_EDRX_READ);
}
static int Bc35UserGeteDRX(void){
    char *p = 0, *p1 = 0;
    int dst[10] = {0};
    unsigned char num = 0;
	Bc35UserGeteDRXSend();
	delay1ms(50);
	memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
    if(0 != (p = strstr((char *)gBc35RecvBuf, BC35_EDRX_REPLY))){
        if(0 != (p1 = strstr(p, BC35_OK_REPLY))){
            p = Bc35UserModule((char *)gBc35RecvBuf, BC35_EDRX_REPLY);
        }
    }
	if(p && p1){
		GetDstAndNumFromStrByCs(p, ",\"", &num, dst);	
		if(num == 3){
			char mode = atoi(p + dst[0]);
			short value = (atoi(p + dst[1]) << 2) | atoi(p + dst[2]);
			#ifdef HC32_LOG_SUPPORTS
			//edrx status
			FIOT_LAB_LOG(LOG_INFO,"eDRX:%d,%04d\r\n",mode,value);
			#endif
			return 1;
		}
	}
	return 0;
}
/********************************************************************************
  ** \brief  AT设置eDRX状态命令函数
  ** 
  ** @param  uint8_t mode, uint8_t adt, uint8_t value
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/
static int Bc35UserSeteDRXSend(uint8_t mode, uint8_t adt, uint8_t value){
	if(adt == BC35_EDRX_NBS1)
	{
		if(mode == BC35_EDRX_DISABLE)
			return Bc35UartUserSend(BC35_EDRX_WRITE("0", "5", "0010"));
		else if(mode == BC35_EDRX_ENABLE){
			if(value == BC35_EDRX_20P48SEC){
				return Bc35UartUserSend(BC35_EDRX_WRITE("1", "5", "0010"));				
			}else if(value == BC35_EDRX_40P96SEC){
				return Bc35UartUserSend(BC35_EDRX_WRITE("1", "5", "0011"));			
			}else if(value == BC35_EDRX_81P92SEC){
				return Bc35UartUserSend(BC35_EDRX_WRITE("1", "5", "0101"));			
			}else if(value == BC35_EDRX_163P84SEC){
				return Bc35UartUserSend(BC35_EDRX_WRITE("1", "5", "1001"));			
			}else if(value == BC35_EDRX_327P68SEC){
				return Bc35UartUserSend(BC35_EDRX_WRITE("1", "5", "1010"));			
			}else if(value == BC35_EDRX_655P36SEC){
				return Bc35UartUserSend(BC35_EDRX_WRITE("1", "5", "1011"));			
			}
		}
	}else{	
		#ifdef HC32_LOG_SUPPORTS
		FIOT_LAB_LOG(LOG_ERROR,"eDRX AcT-Type is Error!\r\n");	
		#endif
	}
	return 0;
}
static int Bc35UserSeteDRX(uint8_t mode, uint8_t adt, uint8_t value){
	Bc35UserSeteDRXSend(mode, adt, value);
	delay1ms(50);
	memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
    if(0 != strstr((char *)gBc35RecvBuf, BC35_OK_REPLY)){	
			return Bc35UserGeteDRX();
    }	
	return 0;	
}
/********************************************************************************
  ** \brief  AT获取Nnmi状态命令函数
  ** 
  ** @param  null
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/
static int Bc35UserGetNnmiSend(void){
    return Bc35UartUserSend(BC28_NNMI_READ);
}
static int Bc35UserGetNnmi(void){
    char *p = 0, *p1 = 0;
	Bc35UserGetNnmiSend();
	delay1ms(100);
	memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
    if(0 != (p = strstr((char *)gBc35RecvBuf, BC28_NNMI_REPLY))){
        if(0 != (p1 = strstr(p, BC35_OK_REPLY))){
            p = Bc35UserModule((char *)gBc35RecvBuf, BC28_NNMI_REPLY);
        }
    }	
    if(p && p1){	
        if(*p == '0'){
			#ifdef HC32_LOG_SUPPORTS
			FIOT_LAB_LOG(LOG_INFO,"NNMI:No indications\r\n");
			#endif
            return 1;
        }else if(*p == '1'){
			#ifdef HC32_LOG_SUPPORTS
			FIOT_LAB_LOG(LOG_INFO,"NNMI:Indications and message\r\n");
			#endif
            return 2;
        }else if(*p == '2'){
			#ifdef HC32_LOG_SUPPORTS
			FIOT_LAB_LOG(LOG_INFO,"NNMI:Indications only\r\n");	
			#endif
            return 3;
        }else{
			#ifdef HC32_LOG_SUPPORTS
			FIOT_LAB_LOG(LOG_ERROR,"NNMI:Error the use of Indications\r\n");
			#endif
            return 0;
        }
    }
	return 0;
}
/********************************************************************************
  ** \brief  AT设置Nnmi状态命令函数
  ** 
  ** @param  null
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/
static int Bc35UserSetNnmiSend(uint8_t SetNnmiMode){
	if(SetNnmiMode == 0)
		//No indications
		return Bc35UartUserSend(BC28_NNMI_WRITE("0"));
	else if(SetNnmiMode == 1)
		//Indications and message
		return Bc35UartUserSend(BC28_NNMI_WRITE("1"));
	else if(SetNnmiMode == 2)
		//Indications only
		return Bc35UartUserSend(BC28_NNMI_WRITE("2"));
	return 0;
}
static int Bc35UserSetNnmi(uint8_t SetNnmiMode){

	Bc35UserSetNnmiSend(SetNnmiMode);
	delay1ms(50);
	memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
    if(0 != strstr((char *)gBc35RecvBuf, BC35_OK_REPLY)){	
		return Bc35UserGetNnmi();
    }	
	return 0;	
}
/********************************************************************************
  ** \brief  AT获取CFUN状态命令函数
  ** 
  ** @param  null
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/
static int Bc35UserGetCfunSend(void){
    return Bc35UartUserSend(BC35_CFUN_READ);
}
static int Bc35UserGetCfun(void){
    char *p = 0, *p1 = 0;
	Bc35UserGetCfunSend();
	delay1ms(50);
	memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
    if(0 != (p = strstr((char *)gBc35RecvBuf, BC35_CFUN_REPLY))){
        if(0 != (p1 = strstr(p, BC35_OK_REPLY))){
            p = Bc35UserModule((char *)gBc35RecvBuf, BC35_CFUN_REPLY);
        }
    }
	if(p && p1){
		if(*p == '0'){
			#ifdef HC32_LOG_SUPPORTS
			FIOT_LAB_LOG(LOG_INFO,"CFUN:Minimum functionality\r\n");
			#endif
			return 1;
		}else if(*p == '1'){
			#ifdef HC32_LOG_SUPPORTS
			FIOT_LAB_LOG(LOG_INFO,"CFUN:Full functionality\r\n");
			#endif
			return 2;
		}else{
			#ifdef HC32_LOG_SUPPORTS
			FIOT_LAB_LOG(LOG_ERROR,"CFUN:Error the use of functionality\r\n");
			#endif
			return 0;
		}		
	}
	return 0;
}
/********************************************************************************
  ** \brief  AT设置CFUN状态命令函数
  ** 
  ** @param  uint8_t functionality
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/
static int Bc35UserSetCfunSend(uint8_t functionality){
	if(functionality == 0)
		//Minimum functionality
		return Bc35UartUserSend(BC35_CFUN_WRITE("0"));
	else if(functionality == 1)
		//Full functionality
		return Bc35UartUserSend(BC35_CFUN_WRITE("1"));
	return 0;
}
static int Bc35UserSetCfun(uint8_t functionality){
	Bc35UserSetCfunSend(functionality);
	delay1ms(50);
	memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
    if(0 != strstr((char *)gBc35RecvBuf, BC35_OK_REPLY)){	
			return Bc35UserGetCfun();
    }	
	return 0;	
}
/********************************************************************************
  ** \brief  AT获取时间状态命令函数
  ** 
  ** @param  null
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/
static int Bc35UserGetDateTimeSend(void){
    return Bc35UartUserSend(BC35_DATE_TIME_READ);
}
static int Bc35UserGetDateTime(void){
    char *p = 0, *p1 = 0;
    int dst[16] = {0};
    unsigned char num = 0;
	stc_rtc_time_t readtime;
	Bc35UserGetDateTimeSend();
	delay1ms(100);
	memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));		
    if(0 != (p = strstr((char *)gBc35RecvBuf, BC35_DATE_TIME_REPLY))){
        if(0 != (p1 = strstr(p, BC35_OK_REPLY))){
            p = Bc35UserModule((char *)gBc35RecvBuf, BC35_DATE_TIME_REPLY);
        }
    }
    if(p && p1){
		if(0 != (p1 = strstr(p, "+CCLK: "))){
            *p1 = 0;
            p1 += 7;
        }
		#ifdef HC32_L161_SUPPORTS
        GetDstAndNumFromStrByCs(p, "\"/,:+", &num, dst);
		#endif
		#ifdef HC32_BC28_SUPPORTS	
        GetDstAndNumFromStrByCs(p, "/,:+", &num, dst);
		#endif				
		if(num == 8 || num == 6){					
			#ifdef HC32_L161_SUPPORTS
            short year = atoi(p + dst[1]);
            char sec = atoi(p + dst[6]);	
			#endif
			#ifdef HC32_BC28_SUPPORTS	
            short year = atoi(p + dst[0])+2000;
            char sec = atoi(p + dst[5]);	
			#endif					
            if(year >= 2020 && ((sec <= (gBc35DataTime.sec - 1))||(sec >= (gBc35DataTime.sec + 1)))){
                gBc35DataTime.year = year;
				readtime.u8Year = DEC2BCD(gBc35DataTime.year-2000);
				#ifdef HC32_L161_SUPPORTS
                gBc35DataTime.mouth = atoi(p + dst[2]);
				#endif
				#ifdef HC32_BC28_SUPPORTS				
                gBc35DataTime.mouth = atoi(p + dst[1]);
				#endif							
				readtime.u8Month = DEC2BCD(gBc35DataTime.mouth);
				#ifdef HC32_L161_SUPPORTS							
                gBc35DataTime.day = atoi(p + dst[3]);
				#endif
				#ifdef HC32_BC28_SUPPORTS								
                gBc35DataTime.day = atoi(p + dst[2]);
				#endif							
				readtime.u8Day = DEC2BCD(gBc35DataTime.day);
				#ifdef HC32_L161_SUPPORTS								
                gBc35DataTime.hour = atoi(p + dst[4]);
				#endif
				#ifdef HC32_BC28_SUPPORTS		
                gBc35DataTime.hour = atoi(p + dst[3]);
				#endif
                if(p1 && (*p1 == '+')){
                    p1++;
                    gBc35DataTime.hour += atoi(p1);
                }
				#ifdef HC32_L161_SUPPORTS								
                gBc35DataTime.min = atoi(p + dst[5]);
				#endif
				#ifdef HC32_BC28_SUPPORTS								
                gBc35DataTime.min = atoi(p + dst[4]);
				#endif								
				readtime.u8Minute = DEC2BCD(gBc35DataTime.min);
				#ifdef HC32_L161_SUPPORTS									
				gBc35DataTime.zone = atoi(p + dst[7]);
				#endif
				#ifdef HC32_BC28_SUPPORTS									
				gBc35DataTime.zone = atoi(p + dst[6]);
				#endif
				if((gBc35DataTime.zone == 8) || (gBc35DataTime.zone == 32) ){
					gBc35DataTime.hour += 8;
					if(gBc35DataTime.hour >= 24)
						gBc35DataTime.hour -= 24;
				}
				readtime.u8Hour = DEC2BCD(gBc35DataTime.hour);
                gBc35DataTime.sec = sec;
				readtime.u8Second = DEC2BCD(gBc35DataTime.sec);
				if(Rtc_SetTime(&readtime) != Ok){
					#ifdef HC32_LOG_SUPPORTS
					FIOT_LAB_LOG(LOG_ERROR,"RTC Setting IS ERROR\r\n");
					#endif
				}else{
					#ifdef HC32_LOG_SUPPORTS
					FIOT_LAB_LOG(LOG_ERROR,"RTC Time Already Updata\r\n");
					#endif									
				}
				#ifdef HC32_TIMESTAMP_SUPPORTS
                Bc35UserDataTimeCheck();//转成时间戳
				#endif
				#ifdef HC32_LOG_INFO
				FIOT_LAB_LOG(LOG_INFO,"%04d-%02d-%02d %02d:%02d:%02d\r\n",gBc35DataTime.year, gBc35DataTime.mouth, \
																		gBc35DataTime.day, gBc35DataTime.hour,	\
																		gBc35DataTime.min, gBc35DataTime.sec);
				#endif
			}
         return 1;
        }
    }
    return 0;
}
/********************************************************************************
  ** \brief  AT获取band状态命令函数
  ** 
  ** @param  null
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/
static int Bc35UserGetBandSend(void){
	#ifdef HC32_L161_SUPPORTS
    return Bc35UartUserSend(BC35_BAND_READ);
	#endif
	#ifdef HC32_BC28_SUPPORTS
    return Bc35UartUserSend(BC28_BAND_READ);	
	#endif
}
static int Bc35UserGetBand(void){
    char *p = 0, *p1 = 0;
	if(gBc35Band.update == 1){
		#ifdef	HC32_LOG_SUPPORTS
		FIOT_LAB_LOG(LOG_INFO,"NBAND:%s\r\n", gBc35Band.band);
		#endif
		return 1;
	}
	Bc35UserGetBandSend();
	delay1ms(100);
	memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
	#ifdef HC32_L161_SUPPORTS		
    if(0 != (p = strstr((char *)gBc35RecvBuf, BC35_BAND_REPLY))){
        if(0 != (p1 = strstr(p, BC35_OK_REPLY))){
            p = Bc35UserModule((char *)gBc35RecvBuf, BC35_BAND_REPLY);
        }
	}
	#endif
	#ifdef HC32_BC28_SUPPORTS
    if(0 != (p = strstr((char *)gBc35RecvBuf, BC28_BAND_REPLY))){
        if(0 != (p1 = strstr(p, BC28_OK_REPLY))){
            p = Bc35UserModule((char *)gBc35RecvBuf, BC28_BAND_REPLY);
        }
	}		
	#endif		
    if(p && p1){
		if(strcmp(p, gBc35Band.band)){
				memset(gBc35Band.band, 0, sizeof(gBc35Band.band));
				strcpy(gBc35Band.band, p);
				#ifdef	HC32_LOG_SUPPORTS
				FIOT_LAB_LOG(LOG_INFO,"NBAND:%s\r\n", gBc35Band.band);
				#endif
				gBc35Band.update = 1;
		}
		return 1;			
	}
	return 0;
}
/********************************************************************************
  ** \brief  AT获取CGATT状态命令函数
  ** 
  ** @param  null
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/
static int Bc35UserGetCgattSend(void){
    return Bc35UartUserSend(BC35_CGATT_READ);
}
static int Bc35UserGetCgatt(void){
    char *p = 0, *p1 = 0;
		Bc35UserGetCgattSend();
		delay1ms(50);
		memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
    if(0 != (p = strstr((char *)gBc35RecvBuf, BC35_CGATT_REPLY))){
        if(0 != (p1 = strstr(p, BC35_OK_REPLY))){
            p = Bc35UserModule((char *)gBc35RecvBuf, BC35_CGATT_REPLY);
        }
    }	
    if(p && p1){
				if(*p == '1'){
					#ifdef HC32_LOG_SUPPORTS
					FIOT_LAB_LOG(LOG_INFO,"CGATT:1\r\n");
					#endif
					return 1;
				}else if(*p == '0'){
					#ifdef HC32_LOG_SUPPORTS
					FIOT_LAB_LOG(LOG_INFO,"CGATT:0\r\n");	
					#endif
					return 0;
				}else{
					#ifdef HC32_LOG_SUPPORTS
					FIOT_LAB_LOG(LOG_ERROR,"CGATT:error\r\n");
					#endif					
					return 0;
				}
    }
    return 0;
}
/********************************************************************************
  ** \brief  AT设置CGATT状态命令函数
  ** 
  ** @param  null
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/
static int Bc35UerSetCgattSend(uint8_t isAttached){
		if(isAttached == 0)
			//Detached
			return Bc35UartUserSend(BC35_CGATT_WRITE("0"));
		else if(isAttached == 1)
			//Attached
			return Bc35UartUserSend(BC35_CGATT_WRITE("1"));
		return 0;
}
static int Bc35UerSetCgatt(uint8_t isAttached){
		Bc35UerSetCgattSend(isAttached);
		delay1ms(50);
		memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
    if(0 != strstr((char *)gBc35RecvBuf, BC35_OK_REPLY)){	
			return Bc35UserGetCgatt();
    }	
		return 0;	
}
/********************************************************************************
  ** \brief  AT修改IMEI状态命令函数
  ** 
  ** @param  需要修改的IMEI
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/
int Bc35UserModifyIMEISend(char *str){
	char gSendbuf[64] = {0};
	sprintf(gSendbuf,"AT+ECCGSN=\"IMEI\",\"%s\"\r\n",str);
	if(Bc35UartUserSend(gSendbuf)){
		delay1ms(300);
		memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
		if(0 != strstr((char *)gBc35RecvBuf, BC35_OK_REPLY)){
			gBc35Qimei.update = 0;
			return 1;
		}
	}	
	return 0;
}
/********************************************************************************
  ** \brief  AT获取CSQ状态命令函数
  ** 
  ** @param  null
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/
static int Bc35UserGetCsqSend(void){
    return Bc35UartUserSend(BC35_CSQ_READ);
}
static int BC35UserGetCsq(void){
    char *p = 0, *p1 = 0;
    int dst[10] = {0};
    unsigned char num = 0;
		Bc35UserGetCsqSend();
		delay1ms(50);
		memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
    if(0 != (p = strstr((char *)gBc35RecvBuf, BC35_CSQ_REPLY))){
        if(0 != (p1 = strstr(p, BC35_OK_REPLY))){
            p = Bc35UserModule((char *)gBc35RecvBuf, BC35_CSQ_REPLY);
        }
    }	
    if(p && p1){
        GetDstAndNumFromStrByCs(p, ",", &num, dst);		
				if(num == 1)
				{
					char rssi = atoi(p + dst[0]);
					char ber = atoi(p + dst[1]);
					#ifdef HC32_LOG_SUPPORTS
					//network status
					FIOT_LAB_LOG(LOG_INFO,"CSQ:%d,%d\r\n",rssi,ber);
					#endif
				}				
        return 1;
    }
    return 0;	
}
/********************************************************************************
  ** \brief  AT+ECSTATUS命令解析函数
  ** 
  ** @param  char *buf
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/
#ifdef HC32_L161_SUPPORTS
static int Data_parser(char *buf){
	char *p = 0,*p1 = 0;
	uint8_t i = 0;
	char data[][20] = {{"DlEarfcn:"},{"UlEarfcn:"},{"PCI:"},{"Band:"},{"RSRP:"},{"RSRQ:"}, \
									{"SNR:"},{"CeLevel:"},{"DlBler:"},{"UlBler:"},{"DataInactTimerS:"},{"RetxBSRTimerP:"},{"NBMode:\""}};
	
	uint8_t glen = sizeof(data)/20;
	memset(&gBc35NueStats, 0, sizeof(gBc35NueStats));								
	for(i = 0; i <= glen; i++){
		p = strstr((char *)buf, data[i]);
		p += strlen(data[i]);
    if(0 != (p1 = strstr((char *)p, ","))){
		}
		else{
			p1 = strstr((char *)p, "\"");		
		}
		if(p && p1){
			memcpy(gBc35NueStats.DlEarfcn + (i*10), p, p1 - p);
		}else{
			return 0;
		}
	}
	return 1;
}
#endif

/********************************************************************************
  ** \brief  AT+ECSTATUS命令函数
  ** 
  ** @param  null
  ** 
  ** \retval Ok or Faile
******************************************************************************/
#ifdef HC32_L161_SUPPORTS
static int Bc35UserGetNueStatsSend(void){
    return Bc35UartUserSend(BC35_NUESTATS_READ);
}
static int Bc35UserGetNueStatus(void){
    char *p = 0, *p1 = 0, *p2 = 0;
	char tmpbuf[256] = {0};
	Bc35UserGetNueStatsSend();
	delay1ms(500);
	memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
    if(0 != (p = strstr((char *)gBc35RecvBuf, BC35_OK_REPLY))){//只处理了PHY数据
				if(0 != (p1 = strstr((char *)gBc35RecvBuf, BC35_NUESTATS_REPLYPHY))){
					if(p && p1){
							p2 = strstr((char *)p1, "\r\n");
							if(p1 && p2){
								p1 += strlen(BC35_NUESTATS_REPLYPHY);
								memcpy((char *)tmpbuf, p1, p2 - p1);
								Data_parser(tmpbuf);
								#ifdef HC32_LOG_SUPPORTS								
								FIOT_LAB_LOG(LOG_DEBUG,"DlEarfcn:%s\r",gBc35NueStats.DlEarfcn);
								FIOT_LAB_LOG(LOG_DEBUG,"UlEarfcn:%s\r",gBc35NueStats.UlEarfcn);
								FIOT_LAB_LOG(LOG_DEBUG,"PCI:%s\r",gBc35NueStats.PCI);
								FIOT_LAB_LOG(LOG_DEBUG,"Band:%s\r",gBc35NueStats.Band);
								FIOT_LAB_LOG(LOG_DEBUG,"RSRP:%s\r",gBc35NueStats.RSRP);
								FIOT_LAB_LOG(LOG_DEBUG,"RSRQ:%s\r",gBc35NueStats.RSRQ);
								FIOT_LAB_LOG(LOG_DEBUG,"SNR:%s\r",gBc35NueStats.SNR);													
								FIOT_LAB_LOG(LOG_DEBUG,"CeLevel:%s\r",gBc35NueStats.CeLevel);
								FIOT_LAB_LOG(LOG_DEBUG,"DlBler:%s\r",gBc35NueStats.DlBler);
								FIOT_LAB_LOG(LOG_DEBUG,"UlBler:%s\r",gBc35NueStats.UlBler);
								FIOT_LAB_LOG(LOG_DEBUG,"DataInactTimerS:%s\r",gBc35NueStats.DataInactTimerS);
								FIOT_LAB_LOG(LOG_DEBUG,"RetxBSRTimerP:%s\r",gBc35NueStats.RetxBSRTimerP);
								FIOT_LAB_LOG(LOG_DEBUG,"NBMode:%s\r",gBc35NueStats.NBMode);
								#endif
					}	
				}
			}
			return 1;
    }	
		return 0;
}
#endif
#ifdef HC32_BC28_SUPPORTS
static int Bc28UserGetNueStatsSend(void){
    return Bc35UartUserSend(BC28_NUESTATS_READ);
}
static int Bc28UserGetNueStatus(void){
    char *p = 0;
	Bc28UserGetNueStatsSend();
	delay1ms(200);
	memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
    if(0 != (p = strstr((char *)gBc35RecvBuf, BC35_OK_REPLY))){
		if(0 == strstr((char *)gBc35RecvBuf, "-32768")){		
			if(p){
			memset(&gBc28NueStats,0,sizeof(gBc28NueStats));
			sscanf((char *)gBc35RecvBuf, "\rSignal power:%s\rTotal power:%s\rTX power:%s\rTX time:%s\rRX time:%s\rCell ID:%s\rECL:%s\rSNR:%s\rEARFCN:%s\rPCI:%s\rRSRQ:%s\r", \
		                          gBc28NueStats.Signal_power, gBc28NueStats.Total_power, gBc28NueStats.TX_power, gBc28NueStats.TX_time, gBc28NueStats.RX_time, \
									gBc28NueStats.SCell_ID, gBc28NueStats.ECL, gBc28NueStats.SNR, gBc28NueStats.EARFCN, gBc28NueStats.PCI, gBc28NueStats.RSRQ); 
			#ifdef HC32_LOG_SUPPORTS
			FIOT_LAB_LOG(LOG_INFO,"Signal Power:%s \r",gBc28NueStats.Signal_power);
			FIOT_LAB_LOG(LOG_INFO,"Total Power:%s \r",gBc28NueStats.Total_power);
			FIOT_LAB_LOG(LOG_INFO,"TX Power:%s \r",gBc28NueStats.TX_power);
			FIOT_LAB_LOG(LOG_INFO,"TX_time:%s \r",gBc28NueStats.TX_time);
			FIOT_LAB_LOG(LOG_INFO,"RX time:%s \r",gBc28NueStats.RX_time);
			FIOT_LAB_LOG(LOG_INFO,"Cell ID:%s \r",gBc28NueStats.SCell_ID);
			FIOT_LAB_LOG(LOG_INFO,"ECL:%s \r",gBc28NueStats.ECL);
			FIOT_LAB_LOG(LOG_INFO,"SNR:%s \r",gBc28NueStats.SNR);
			FIOT_LAB_LOG(LOG_INFO,"EARFCN:%s \r",gBc28NueStats.EARFCN);
			FIOT_LAB_LOG(LOG_INFO,"PCI:%s \r",gBc28NueStats.PCI);
			FIOT_LAB_LOG(LOG_INFO,"RSRQ:%s \r",gBc28NueStats.RSRQ);
			#endif
			}
			return 1;
		}
    }	
	return 0;
}
#endif
/********************************************************************************
  ** \brief  连接华为OC平台命令函数
  ** 
  ** @param  null
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/
#ifdef HC_BC35_OCEANCONNECT_SUPPORT
/********************************************************************************
  ** \brief  AT获取南向地址函数
  ** 
  ** @param  null
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/
static int Bc35UserGetQlwServSend(void){
    return Bc35UartUserSend(BC35_NCDP_READ);
}
static int Bc35UserGetQlwServ(void){
    char *p = 0;
    int dst[10] = {0};
    unsigned char num = 0;
	Bc35UserGetQlwServSend();
	delay1ms(50);
	memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
    if(0 != (p = strstr((char *)gBc35RecvBuf, BC35_NCDP_REPLY))){
       p = Bc35UserModule((char *)gBc35RecvBuf, BC35_NCDP_REPLY);
    }
	if(p){
	GetDstAndNumFromStrByCs(p, ".,", &num, dst);
		if(num == 6){
			gBc35Server.ippart1 = atoi(p + dst[0]);
			gBc35Server.ippart2 = atoi(p + dst[1]);
			gBc35Server.ippart3 = atoi(p + dst[2]);
			gBc35Server.ippart4 = atoi(p + dst[3]);
			gBc35Server.ipport = atoi(p + dst[4]);
			gBc35Server.lifetime = atoi(p + dst[5]);
			gBc35Server.instance = atoi(p + dst[6]);
			//judge ip address set succeed?
			#ifdef HC32_LOG_SUPPORTS
			//ip address
			FIOT_LAB_LOG(LOG_INFO,"IP ADDRESS:%d.%d.%d.%d:%d,%d,%d\r\n", gBc35Server.ippart1, gBc35Server.ippart2, gBc35Server.ippart3, gBc35Server.ippart4,
																																	gBc35Server.ipport, gBc35Server.lifetime, gBc35Server.instance);
			#endif
			return 1;
		}
	}
	return 0;
}
/********************************************************************************
  ** \brief  AT删除注册信息函数
  ** 
  ** @param  null
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/
static int Bc35UserSetCtm2mERegSend(void){
    return Bc35UartUserSend(BC35_CTM2MEREG_READ);
}
static int Bc35UserSetCtm2mEReg(void){
    char *p = 0, *p1 = 0;
		if(gBc35UserGetDataStatus == 4){
			gBc35UserGetDataStatus = 0;
			return 1;
		}
		Bc35UserSetCtm2mERegSend();
		delay1ms(200);
		memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
		if(0 != (p = strstr((char *)gBc35RecvBuf, BC35_OK_REPLY))){
				if(0 != (p1 = strstr((char *)gBc35RecvBuf, BC35_OK_REPLY)))
					if(p && p1)
						return 1;
		}else if(0 != (p = strstr((char *)gBc35RecvBuf, "+CTM2M ERROR: 958"))){//已经删除注册信息
					return 1;
		}
		return 0;
}
/********************************************************************************
  ** \brief  AT设置注册信息函数
  ** 
  ** @param  null
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/
static int Bc35UserSetCtm2mRegSend(void){
    return Bc35UartUserSend(BC35_CTM2MREG_READ);
}
static int Bc35UserSetCtm2mReg(void){
		if(gBc35UserGetDataStatus == 3){
			gBc35UserGetDataStatus = 0;
			return 1;
		}
		Bc35UserSetCtm2mRegSend();
		delay1ms(200);
		memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
		FIOT_LAB_LOG(LOG_DEBUG,"m2mReg:%s\r\n",gBc35RecvBuf);
		if(0 != strstr((char *)gBc35RecvBuf, BC35_OK_REPLY)){//等待注册信息
				return 0;
			}else if(0 != strstr((char *)gBc35RecvBuf, "+CTM2M ERROR:")){//未删除注册信息
				Bc35UserSetCtm2mEReg();
				return 0;
		}
		return 0;
}
/********************************************************************************
  ** \brief  AT设置更新数据信息函数
  ** 
  ** @param  null
  ** 
  ** \retval Ok or Faile
******************************************************************************/
static int Bc35UserSetCtm2mUpDataSend(void){
    return Bc35UartUserSend(BC35_CTM2MUPDATE_READ);
}
static int Bc35UserSetCtm2mUpData(void){
		Bc35UserSetCtm2mUpDataSend();
		delay1ms(100);
		memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
		FIOT_LAB_LOG(LOG_DEBUG,"BC35_CTM2MUPDATE_READ:%s\r",gBc35RecvBuf);
		if(0 != strstr((char *)gBc35RecvBuf, BC35_OK_REPLY)){
				return 1;
		}else{
		
		}
		return 0;
}
/********************************************************************************
  ** \brief  AT设置南向地址函数
  ** 
  ** @param  null
  ** 
  ** \retval Ok or Faile
******************************************************************************/
static int Bc28UserGetQlwServSend(void){
    return Bc35UartUserSend(BC28_NCDP_READ);
}
static int Bc28UserGetQlwServ(void){
    char *p = 0, *p1 = 0;
    int dst[10] = {0};
    unsigned char num = 0;
	Bc28UserGetQlwServSend();
	delay1ms(50);
	memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
    if(0 != (p = strstr((char *)gBc35RecvBuf, BC28_NCDP_REPLY))){
        if(0 != (p1 = strstr(p, BC28_OK_REPLY))){
            p = Bc35UserModule((char *)gBc35RecvBuf, BC28_NCDP_REPLY);
        }
    }
	if(p && p1){
	GetDstAndNumFromStrByCs(p, ".,", &num, dst);
			if(num == 4){
				gBc35Server.ippart1 = atoi(p + dst[0]);
				gBc35Server.ippart2 = atoi(p + dst[1]);
				gBc35Server.ippart3 = atoi(p + dst[2]);
				gBc35Server.ippart4 = atoi(p + dst[3]);
				gBc35Server.ipport = atoi(p + dst[4]);
				//judge ip address set succeed?
				#ifdef HC32_LOG_SUPPORTS
				//ip address
				FIOT_LAB_LOG(LOG_INFO,"Setting Network Ip Address:%d.%d.%d.%d,%d\r\n", gBc35Server.ippart1, gBc35Server.ippart2, gBc35Server.ippart3, gBc35Server.ippart4, gBc35Server.ipport);
				#endif
				return 1;
			}
	}
	return 0;
}
static int Bc28UserQlwServSend(void){
    char gBc35ServerIps[][16] = {
        //"139.159.140.34",
		"49.4.85.232",
    };
    char tmp[50] = {0};
    #ifdef HC_BC35_OCEANCONNECT_SERVER_IP_SUPPORT
    if((gBc28UserQlwOcip[0] != 0)&&(gBc28UserQlwOcip[1] != 0)&&(gBc28UserQlwOcip[2] != 0)&&(gBc28UserQlwOcip[3] != 0)){
        sprintf(tmp, "AT+NCDP=%d.%d.%d.%d,%d\r\n", gBc28UserQlwOcip[0], gBc28UserQlwOcip[1], gBc28UserQlwOcip[2], gBc28UserQlwOcip[3],gBc28UserQlwPort);
    #ifdef HC_FLASH_SUPPORT
    }else if((gNvData.Ocip[0] != 0)&&(gNvData.Ocip[1] != 0)&&(gNvData.Ocip[2] != 0)&&(gNvData.Ocip[3] != 0)){
        sprintf(tmp, "AT+NCDP=%d.%d.%d.%d,5683\r\n", gNvData.Ocip[0], gNvData.Ocip[1], gNvData.Ocip[2], gNvData.Ocip[3]);
    #endif
    }else{
        sprintf(tmp, "AT+NCDP=%s,5683\r\n", gBc35ServerIps[0]);
    }
    #else
    sprintf(tmp, "AT+NCDP=%s,5683\r", gBc35ServerIps[0]);
    #endif	
	#ifdef HC32_LOG_SUPPORTS
	FIOT_LAB_LOG(LOG_INFO,"%s",tmp);
	#endif
    if(Bc35UartUserSend(tmp)){
		delay1ms(50);
		memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
		if(0 != strstr((char *)gBc35RecvBuf, BC35_OK_REPLY)){						
			return Bc28UserGetQlwServ();
		}
	}
	return 0;
}
/********************************************************************************
  ** \brief  AT发送数据函数
  ** 
  ** @param  null
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/
#ifdef HC32_BC28_SUPPORTS
int Bc28UserQlwSendDataSend(uint8_t mesg, uint8_t ctrlcode, uint8_t errorcode, uint8_t secret){
    static char gSendBuf[512] = {0}, buf[512] = {0};
    int gLength = 0;
	float BatteryValue = 0;
	uint16_t u16RefCrc16 = 0;
	
	stc_rtc_time_t readtime;
	uint8_t ret = 0,CtrlCode = ctrlcode;
	uint8_t Secret = secret,ecode = errorcode;
    memset(gSendBuf, 0, sizeof(gSendBuf));
    memset(buf, 0, sizeof(buf));
	memset(&gBc35Data,0,sizeof(gBc35Data));
	
	gBc35Data.Header[0] = 0x36;//固定头 68H
	gBc35Data.Header[1] = 0x38;//固定头 68H
    strcat(buf, gBc35Data.Header);
	ret = Bc35UserGetQimei();
	if(ret != 1) return 0;
	else{
		sprintf(gBc35Data.Imei,"0%s",gBc35Qimei.imei);
		strcat(buf, gBc35Data.Imei);			
	}
	sprintf(gBc35Data.VerCode,"0B");//VERSIONCODE
    strcat(buf, gBc35Data.VerCode);
		
	switch(CtrlCode)
	{
		case DEV_CCODE_METERDATA:
			sprintf(gBc35Data.ConCode,"%02d",DEV_CCODE_METERDATA);//Meter Water CONTROLCODE 01
			break;
		case DEV_CCODE_PHOTODATA://无图片
			sprintf(gBc35Data.ConCode,"%02d",DEV_CCODE_PHOTODATA);//Photo CONTROLCODE 02
			break;
		case DEV_CCODE_PARAMETERDATA://参数码
			sprintf(gBc35Data.ConCode,"%02d",DEV_CCODE_PARAMETERDATA);//PARAMETER CONTROLCODE 03
			break;
		case DEV_CCODE_ALARMDATA://报警码
			sprintf(gBc35Data.ConCode,"%02d",DEV_CCODE_ALARMDATA);//ALARM CONTROLCODE 04
			break;
		case DEV_CCODE_AGAINMETERDATA:
			sprintf(gBc35Data.ConCode,"%02d",DEV_CCODE_AGAINMETERDATA);//AGAINMETER CONTROLCODE 05
			break;
		case DEV_CCODE_AGAINPHOTODATA://无图片
			sprintf(gBc35Data.ConCode,"%02d",DEV_CCODE_AGAINPHOTODATA);//AGAINPHOTO CONTROLCODE 06
			break;
		default://error CONTROLCODE
			sprintf(gBc35Data.ConCode,"%02d",DEV_CCODE_METERDATA);//CONTROLCODE
			break;
	}
    strcat(buf, gBc35Data.ConCode);
		
	//设备名字：666(FIOT(公司编号)) + (MCU唯一标识)DEVUID(9位)		
	sprintf(gBc35Data.Mid,"666%s","123456789");//MID 
    strcat(buf, gBc35Data.Mid);
		
	if(CtrlCode == DEV_CCODE_METERDATA || CtrlCode == DEV_CCODE_PHOTODATA  \
									|| CtrlCode == DEV_CCODE_AGAINMETERDATA){								
										
//		sprintf(gBc35Data.SNR,"%04x",(unsigned char)atoi(gBc28NueStats.SNR));//SNR
		sprintf(gBc35Data.SNR," %04s ",gBc28NueStats.SNR);//SNR											
		FIOT_LAB_LOG(LOG_DEBUG,"gBc35Data.SNR:%s\r",gBc35Data.SNR);	 																	
		strcat(buf, gBc35Data.SNR);
//		sprintf(gBc35Data.RSRP,"%04x",(unsigned short)atoi(gBc28NueStats.Signal_power));//RSRP
		sprintf(gBc35Data.RSRP," %04s ",gBc28NueStats.Signal_power);//RSRP									
		FIOT_LAB_LOG(LOG_DEBUG,"gBc35Data.RSRP:%s\r",gBc35Data.RSRP);																		
		strcat(buf, gBc35Data.RSRP);	
		sprintf(gBc35Data.PCI,"%04x",(unsigned char)atoi(gBc28NueStats.PCI));//PCI
		FIOT_LAB_LOG(LOG_DEBUG,"gBc35Data.PCI:%s\r",gBc35Data.PCI);																		
		strcat(buf, gBc35Data.PCI);	
		sprintf(gBc35Data.ECL,"%02d",(unsigned char)atoi(gBc28NueStats.ECL));//ECL
		FIOT_LAB_LOG(LOG_DEBUG,"gBc35Data.ECL:%s\r",gBc35Data.ECL);																			
		strcat(buf, gBc35Data.ECL);
	}							
	Rtc_ReadDateTime(&readtime);																				
	sprintf(gBc35Data.ReportTime, "%02d%02d%02d%02d%02d%02d",BCD2DEC(readtime.u8Year),BCD2DEC(readtime.u8Month),BCD2DEC(readtime.u8Day), \
														BCD2DEC(readtime.u8Hour),BCD2DEC(readtime.u8Minute),BCD2DEC(readtime.u8Second));//ReportTime		
	strcat(buf, gBc35Data.ReportTime);
			
	sprintf(gBc35Data.DevId,"%s","01");//Instrument ID 	
	strcat(buf, gBc35Data.DevId);	
	FIOT_LAB_LOG(LOG_DEBUG,"gBc35Data.DevId:%2s\r\n",gBc35Data.DevId);	
	
	if(Secret == 1){
		gBc35Data.SecFlag[0] = 0x31;//加密SrcFlag
	}else{
		gBc35Data.SecFlag[0] = 0x30;//不加密SrcFlag		
	}
	sprintf(gBc35Data.SecFlag,"%2s","00");		
	strcat(buf, gBc35Data.SecFlag);	
	FIOT_LAB_LOG(LOG_DEBUG,"gBc35Data.SecFlag:%2s\r\n",gBc35Data.SecFlag);	

	gBc35Data.Len[0] = 32;		
	sprintf(gBc35Data.Len,"0%03x",gBc35Data.Len[0]);		
	strcat(buf, gBc35Data.Len);
	FIOT_LAB_LOG(LOG_DEBUG,"gBc35Data.Len:%2s\r\n",gBc35Data.Len);
	
	sprintf(gBc35Data.Count,"0%s","1");
	strcat(buf, gBc35Data.Count);
	FIOT_LAB_LOG(LOG_DEBUG,"gBc35Data.Count:%2s\r\n",gBc35Data.Count);		

	HC32_BATTERY_DeInit();
	HC32_BATTERY_Init();		
	BatteryValue = BatteryObtain();	
	if(BatteryValue <= 3.65 && BatteryValue >= 3.3){
		gBc35UserBatteryValueLevel = 3;
	}else if(BatteryValue >= 3.15 && BatteryValue < 3.3){
		gBc35UserBatteryValueLevel = 2;
	}else if(BatteryValue < 3.15 && BatteryValue > 2.8){
		gBc35UserBatteryValueLevel = 1;
		ecode = DEV_ECODE_BATTERY;//Low Vol Warn 
	}else{
		ecode = DEV_ECODE_BATTERY;
		gBc35UserBatteryValueLevel = 0;
	}	

	if(gNormalOverCurrentWarn == 1){// Normal OverCurrent Warn
		ecode = DEV_ECODE_OVERCURRENT;
	}
	
	//Data Field
	switch(ecode)
	{
		case DEV_ECODE_NORMAL:
			sprintf(gBc35Data.Ecode,"%02d",DEV_ECODE_NORMAL);//Ecode	Normal 01
			break;
		case DEV_ECODE_BATTERY:
			sprintf(gBc35Data.Ecode,"%02d",DEV_ECODE_BATTERY);//Ecode	Battery 02
			break;
		case DEV_ECODE_GAINDATA:
			sprintf(gBc35Data.Ecode,"%02d",DEV_ECODE_GAINDATA);//Ecode	Gaindata 03
			break;
		case DEV_ECODE_DEVDESTROY:
			sprintf(gBc35Data.Ecode,"%02d",DEV_ECODE_DEVDESTROY);//Ecode	DevDestroy 04
			break;
		case DEV_ECODE_VALVA:
			sprintf(gBc35Data.Ecode,"%02d",DEV_ECODE_VALVA);//Ecode	Valva 05
			break;
		case DEV_ECODE_OVERCURRENT:
			sprintf(gBc35Data.Ecode,"%02d",DEV_ECODE_OVERCURRENT);//Ecode OverCurrent
			break;
		case DEV_ECODE_CONCURRENT:
			sprintf(gBc35Data.Ecode,"%02d",DEV_ECODE_CONCURRENT);//Ecode ConCurrent
			break;
		default:
			sprintf(gBc35Data.Ecode,"%02d",DEV_ECODE_NORMAL);//Ecode	Normal
			break;
	}
	strcat(buf, gBc35Data.Ecode);

	if(CtrlCode == DEV_CCODE_METERDATA){//抄表数据
		gBc35Data.Valve[0] = 0x31;
		sprintf(gBc35Data.Valve, "0%s", gBc35Data.Valve);
		FIOT_LAB_LOG(LOG_DEBUG,"gBc35Data.Valve:%s\r",gBc35Data.Valve);	
		strcat(buf, gBc35Data.Valve);
		sprintf(gBc35Data.WaterNum, "%08x", "88088233");
		FIOT_LAB_LOG(LOG_DEBUG,"gBc35Data.WaterNum:%s\r",gBc35Data.WaterNum);			
		strcat(buf, gBc35Data.WaterNum);
		sprintf(gBc35Data.IntervalFlow, "%08x", "50233");	
		FIOT_LAB_LOG(LOG_DEBUG,"gBc35Data.IntervalFlow:%s\r",gBc35Data.IntervalFlow);				
		strcat(buf, gBc35Data.IntervalFlow);
		sprintf(gBc35Data.InverseNum, "%08x", "10000003");
		FIOT_LAB_LOG(LOG_DEBUG,"gBc35Data.InverseNum:%s\r",gBc35Data.InverseNum);				
		strcat(buf, gBc35Data.InverseNum);
		sprintf(gBc35Data.AcquisitionTime, "%02d%02d%02d%02d%02d%02d",gBc35DataTime.year,gBc35DataTime.mouth, \
																		gBc35DataTime.day,gBc35DataTime.hour, \
																		gBc35DataTime.min,gBc35DataTime.sec);	
		strcat(buf, gBc35Data.AcquisitionTime);
		
		sprintf(gBc35Data.Vol, "%04x",(int)(BatteryValue*100));//保留两位数据  x100	
		FIOT_LAB_LOG(LOG_DEBUG,"gBc35Data.Vol:%s  %d\r",gBc35Data.Vol, (int)(BatteryValue*100));				
		strcat(buf, gBc35Data.Vol);
		sprintf(gBc35Data.Temperature, "%04x","201");			
		FIOT_LAB_LOG(LOG_DEBUG,"gBc35Data.Temperature:%s\r",gBc35Data.Temperature);				
		strcat(buf, gBc35Data.Temperature);		
		sprintf(gBc35Data.Press, "%04x","0");			
		FIOT_LAB_LOG(LOG_DEBUG,"gBc35Data.Press:%s\r",gBc35Data.Press);				
		strcat(buf, gBc35Data.Press);	
		sprintf(gBc35Data.Ph, "%04x","80");			
		FIOT_LAB_LOG(LOG_DEBUG,"gBc35Data.PH:%s\r",gBc35Data.Ph);				
		strcat(buf, gBc35Data.Ph);	
		sprintf(gBc35Data.Chlorine, "%04x","300");			
		FIOT_LAB_LOG(LOG_DEBUG,"gBc35Data.Chlorine:%s\r",gBc35Data.Chlorine);				
		strcat(buf, gBc35Data.Chlorine);
		sprintf(gBc35Data.Turbidity, "%04x","003");			
		FIOT_LAB_LOG(LOG_DEBUG,"gBc35Data.Turbidity:%s\r",gBc35Data.Turbidity);				
		strcat(buf, gBc35Data.Turbidity);
	}else if(CtrlCode == DEV_CCODE_ALARMDATA){//报警数据
		switch(ecode){
			case DEV_ECODE_DEVDESTROY:
				gBc35Data.DevDestroy[0] = 0x31;
				strcat(buf, gBc35Data.DevDestroy);
				break;
		}
	}
	
	//做CRC校验
	u16RefCrc16 = HC32_CRC16_Get16((uint16_t *)buf, sizeof(buf),16);	
	sprintf(gBc35Data.CRC,"%04x",u16RefCrc16);
	
	strcat(buf, gBc35Data.CRC);
	FIOT_LAB_LOG(LOG_DEBUG, "u16RefCrc16:0x%x CRC:0x%x 0x%x\r\n",u16RefCrc16,gBc35Data.CRC[0],gBc35Data.CRC[1]);
	FIOT_LAB_LOG(LOG_DEBUG, "BUF:%s\r\n",buf);		
	gBc35Data.End[0] = 0X31;//固定 68H
	gBc35Data.End[1] = 0X36;//固定头 68H
	strcat(buf, gBc35Data.End);
	gLength = strlen(buf);		
    sprintf(gSendBuf, "AT+NMGS=%d,%02d%02x", gLength+2, mesg, gLength);
    if(AsciiStrToHexStr(buf, gLength, &gSendBuf[strlen(gSendBuf)], sizeof(gSendBuf) - strlen(gSendBuf))){
		strcat(gSendBuf, "\r\n");
		FIOT_LAB_LOG(LOG_DEBUG, "gSendBuf:%s",gSendBuf);
        if(Bc35UartUserSend(gSendBuf)){				
		delay1ms(1000);
		// be likely to not rec NNMI.
		memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
		FIOT_LAB_LOG(LOG_DEBUG,"gBc35RecvBuf:%s\r\n",gBc35RecvBuf);
		if(0 != strstr((char *)gBc35RecvBuf, BC35_OK_REPLY)){//send ok,start record time
			FIOT_LAB_LOG(LOG_DEBUG,"Send OK to OC \r\n");
			if(u8DeviceWarning == 1)
				u8DeviceWarning = 0;
			else if(u8DeviceAlarm == 1)
				u8DeviceAlarm = 0;	
			else if(u8DeviceOverCurrent == 1)
				u8DeviceOverCurrent = 0;				
			//Start Lptimer
			LpTimer_Enable();
			//Wati 20s Flag
			gBc35UserWaitResponse = 1;	
			//Send Data to OC  succeed, Wait OC Cmd.
			gBc35UserReceiveCmdFromOc = 1;
			//clear time count
			gLptimSecRecord = 0;
			gBc35UserSendDataRepetition++;
			gBc35UserSendDataStatus = 1;
			return 1;
			}				
		}
	}
	return 0;
}
#endif
static uint8_t OceanConnect_cmd_parser(uint8_t *pu8Data, uint16_t u16DataLen){

	uint8_t tmp_state = 0,j = 0;
	char crcbuf[128] = {0};
	uint16_t u16RefCrc16 = 0;
	char tmp[5] = {0};
	char cmptmp[5] = {0};	
	uint16_t i = 1;	//i = 0  是messageID
	
  if ((NULL == pu8Data) || (u16DataLen == NULL)) return 0;

	tmp_state = CMD_PARSER_STATE_Header;//初始状态
	
	#ifdef HC32_LOG_DEBUG	
	FIOT_LAB_LOG(LOG_DEBUG,"Start Parser!\r");	
	#endif
	
	for (i = 1; i < u16DataLen; i += 2)//两位处理一次
	{
		switch (tmp_state)
		{		
			case CMD_PARSER_STATE_Header:
				if(((pu8Data[i]-48)*10 + (pu8Data[i+1]-48)) == CMD_SYNC_HEADER){
					gBc35CmdParser.Header = CMD_SYNC_HEADER;
					#ifdef HC32_LOG_DEBUG
					FIOT_LAB_LOG(LOG_DEBUG,"Find Header = 0x%d\r",gBc35CmdParser.Header);
					#endif
					crcbuf[0] = gBc35CmdParser.Header;
					tmp_state = CMD_PARSER_STATE_Version;
				}else{
					tmp_state = CMD_PARSER_STATE_Error;
				}
				break;
			case CMD_PARSER_STATE_Version:
				if(((pu8Data[i]-48)*10 + (pu8Data[i+1]-48)) == CMD_SYNC_VERSION1){
					gBc35CmdParser.AgreementVersion = CMD_SYNC_VERSION1;
					#ifdef HC32_LOG_DEBUG
					FIOT_LAB_LOG(LOG_DEBUG,"Find VERSION1 = 0x%02d\r",gBc35CmdParser.AgreementVersion);
					#endif
					crcbuf[1] = gBc35CmdParser.AgreementVersion;
					tmp_state = CMD_PARSER_STATE_Control;
				}else{
					tmp_state = CMD_PARSER_STATE_Error;
				}				
				break;
			case CMD_PARSER_STATE_Control:
				if(((pu8Data[i]-48)*10 + (pu8Data[i+1]-48)) == CMD_SYNC_CONTROLCODE){
					gBc35CmdParser.ControlCode = CMD_SYNC_CONTROLCODE;
					#ifdef HC32_LOG_DEBUG
					FIOT_LAB_LOG(LOG_DEBUG,"Find CONTROLCODE = 0x%02d\r",gBc35CmdParser.ControlCode);
					#endif
					crcbuf[2] = gBc35CmdParser.ControlCode;
					tmp_state = CMD_PARSER_STATE_Secret;
				}else{
					tmp_state = CMD_PARSER_STATE_Error;
				}	
				break;
			case CMD_PARSER_STATE_Secret:
				if(((pu8Data[i]-48)*10 + (pu8Data[i+1]-48)) == CMD_SYNC_PULSSECRECT){
					gBc35CmdParser.SecretFlag = CMD_SYNC_PULSSECRECT;//加密,加密处理的协议还未完成
					#ifdef HC32_LOG_DEBUG
					FIOT_LAB_LOG(LOG_DEBUG,"Find SECRECT = 0x%02d\r",gBc35CmdParser.SecretFlag);
					#endif
					crcbuf[3] = gBc35CmdParser.SecretFlag;
					tmp_state = CMD_PARSER_STATE_Lenth;
				}else if(((pu8Data[i]-48)*10 + (pu8Data[i+1]-48)) == CMD_SYNC_NOTSECRECT){
					gBc35CmdParser.SecretFlag = CMD_SYNC_NOTSECRECT;//不加密
					#ifdef HC32_LOG_DEBUG
					FIOT_LAB_LOG(LOG_DEBUG,"Find SECRECT = 0x%02d\r",gBc35CmdParser.SecretFlag);
					#endif
					crcbuf[3] = gBc35CmdParser.SecretFlag;
					tmp_state = CMD_PARSER_STATE_Lenth;
				}
				else{
					tmp_state = CMD_PARSER_STATE_Error;
				}
				break;
			case CMD_PARSER_STATE_Lenth:
				if((gBc35CmdParser.DataLenth = ((pu8Data[i]-48)*1000) + ((pu8Data[i+1]-48)*100) + ((pu8Data[i+2]-48)*10) + (pu8Data[i+3]-48)) <= 9999){
					#ifdef HC32_LOG_DEBUG					
					FIOT_LAB_LOG(LOG_DEBUG,"Find LENTH = 0x%d\r",gBc35CmdParser.DataLenth);
					#endif
					crcbuf[4] = gBc35CmdParser.DataLenth;
					tmp_state = CMD_PARSER_STATE_Data;
					i += 2;
				}else{
					tmp_state = CMD_PARSER_STATE_Error;
				}
				break;
			case CMD_PARSER_STATE_Data:
				//Valve Control
				if((gBc35CmdParser.Data.ControlNum = ((pu8Data[i]-48)*1000) + ((pu8Data[i+1]-48)*100) + ((pu8Data[i+2]-48)*10) + (pu8Data[i+3]-48)) == CMD_CODE_VALVE){
					gBc35CmdParser.Data.ControlNum = CMD_CODE_VALVE;
					gBc35CmdParser.Data.Valve = (pu8Data[i+4]-48) * 10 + (pu8Data[i+5]-48);
					#ifdef HC32_LOG_DEBUG
					FIOT_LAB_LOG(LOG_DEBUG,"Find Valve = %d-%d\r",gBc35CmdParser.Data.ControlNum,gBc35CmdParser.Data.Valve);
					#endif
					i += 4;
					crcbuf[5] = gBc35CmdParser.Data.ControlNum;
					crcbuf[6] = gBc35CmdParser.Data.Valve;
					tmp_state = CMD_PARSER_STATE_CRC;
				//Send Data Frequency
				}else if((gBc35CmdParser.Data.ControlNum = ((pu8Data[i]-48)*1000) + ((pu8Data[i+1]-48)*100) + ((pu8Data[i+2]-48)*10) + (pu8Data[i+3]-48)) == CMD_CODE_UPDATAFRE){
					gBc35CmdParser.Data.ControlNum = CMD_CODE_UPDATAFRE;
					//gBc35CmdParser.Data.SdataFrequency = (pu8Data[i+4]-48) * 1000 + (pu8Data[i+5]-48) * 100 + (pu8Data[i+6]-48) * 10 + (pu8Data[i+7]-48);
					gBc35UserSendDataFrequency = (pu8Data[i+4]-48) * 1000 + (pu8Data[i+5]-48) * 100 + (pu8Data[i+6]-48) * 10 + (pu8Data[i+7]-48);
					#ifdef HC32_LOG_DEBUG
					FIOT_LAB_LOG(LOG_DEBUG,"Find Send Data Frequency = %d-%04d\r",gBc35CmdParser.Data.ControlNum,gBc35UserSendDataFrequency);
					#endif
					for(char j = 0; j<4; j += 2){						
						AT24CXX_WriteOneByte(ADDR_DATAFREQUENCY_RECORD+j/2,(pu8Data[i+4+j]-48)*10+(pu8Data[i+4+j+1]-48));
					}				
					crcbuf[5] = gBc35CmdParser.Data.ControlNum;
					crcbuf[6] = gBc35CmdParser.Data.SdataFrequency;
					i += 6;
					tmp_state = CMD_PARSER_STATE_CRC;
				//Gather Frequency
				}else if((gBc35CmdParser.Data.ControlNum = ((pu8Data[i]-48)*1000) + ((pu8Data[i+1]-48)*100) + ((pu8Data[i+2]-48)*10) + (pu8Data[i+3]-48)) == CMD_CODE_GATHERFRE){
					gBc35CmdParser.Data.ControlNum = CMD_CODE_GATHERFRE;
					gBc35CmdParser.Data.GatherFrequency = (pu8Data[i+4]-48) * 1000 + (pu8Data[i+5]-48) * 100 + (pu8Data[i+6]-48) * 10 + (pu8Data[i+7]-48);
					#ifdef HC32_LOG_DEBUG
					FIOT_LAB_LOG(LOG_DEBUG,"Find Gather Frequency = %d-%04d\r",gBc35CmdParser.Data.ControlNum,gBc35CmdParser.Data.GatherFrequency);
					#endif
					crcbuf[5] = gBc35CmdParser.Data.ControlNum;
					crcbuf[6] = gBc35CmdParser.Data.GatherFrequency;
					i += 6;
					tmp_state = CMD_PARSER_STATE_CRC;
				//OvercurrentWarn 
				}else if((gBc35CmdParser.Data.ControlNum = ((pu8Data[i]-48)*1000) + ((pu8Data[i+1]-48)*100) + ((pu8Data[i+2]-48)*10) + (pu8Data[i+3]-48)) == CMD_CODE_OVERCURRENTWARN){
					gBc35CmdParser.Data.ControlNum = CMD_CODE_OVERCURRENTWARN;
					gBc35CmdParser.Data.OvercurrentWarn = (pu8Data[i+4]-48) * 1000 + (pu8Data[i+5]-48) * 100 + (pu8Data[i+6]-48) * 10 + (pu8Data[i+7]-48);
					#ifdef HC32_LOG_DEBUG
					FIOT_LAB_LOG(LOG_DEBUG,"Find OvercurrentWarn = %d-%d\r",gBc35CmdParser.Data.ControlNum,gBc35CmdParser.Data.OvercurrentWarn);
					#endif
					for(char j = 0; j<4; j += 2){						
						AT24CXX_WriteOneByte(ADDR_OCURRENTTH_RECORD+j/2,(pu8Data[i+4+j]-48)*10+(pu8Data[i+4+j+1]-48));
					}					
					AT24CXX_Read(ADDR_OCURRENTTH_RECORD,(uint8_t *)tmp,2);					
					SetOverCurrentValue(tmp[0],tmp[1]);
					#ifdef HC32_LOG_DEBUG
					FIOT_LAB_LOG(LOG_DEBUG,"OVERCURRENTTHRE Num : %02d%02d\r",tmp[0],tmp[1]);
					#endif				
					crcbuf[5] = gBc35CmdParser.Data.ControlNum;
					crcbuf[6] = gBc35CmdParser.Data.OvercurrentWarn;				
					i += 6;
					tmp_state = CMD_PARSER_STATE_CRC;	
				//OvercurrentTime
				}else if((gBc35CmdParser.Data.ControlNum = ((pu8Data[i]-48)*1000) + ((pu8Data[i+1]-48)*100) + ((pu8Data[i+2]-48)*10) + (pu8Data[i+3]-48)) == CMD_CODE_OVERCURRENTTIME){
					gBc35CmdParser.Data.ControlNum = CMD_CODE_OVERCURRENTTIME;
					//gBc35CmdParser.Data.OvercurrentTime = (pu8Data[i+4]-48) * 1000 + (pu8Data[i+5]-48) * 100 + (pu8Data[i+6]-48) * 10 + (pu8Data[i+7]-48);
					gBc35UserOverCurrentTime = (pu8Data[i+4]-48) * 1000 + (pu8Data[i+5]-48) * 100 + (pu8Data[i+6]-48) * 10 + (pu8Data[i+7]-48);					
					for(char j = 0; j<4; j += 2){						
						AT24CXX_WriteOneByte(ADDR_OCURRENTTIME_RECORD+j/2,(pu8Data[i+4+j]-48)*10+(pu8Data[i+4+j+1]-48));
					}					
					#ifdef HC32_LOG_DEBUG
					FIOT_LAB_LOG(LOG_DEBUG,"Find OvercurrentTime = %d-%d \r",gBc35CmdParser.Data.ControlNum,gBc35UserOverCurrentTime);
					#endif					
					crcbuf[5] = gBc35CmdParser.Data.ControlNum;
					crcbuf[6] = gBc35CmdParser.Data.OvercurrentTime;				
					i += 6;
					tmp_state = CMD_PARSER_STATE_CRC;	
				//ConOvercurrentWarn
				}else if((gBc35CmdParser.Data.ControlNum = ((pu8Data[i]-48)*1000) + ((pu8Data[i+1]-48)*100) + ((pu8Data[i+2]-48)*10) + (pu8Data[i+3]-48)) == CMD_CODE_CYOVERCURRENTWARN){
					gBc35CmdParser.Data.ControlNum = CMD_CODE_CYOVERCURRENTWARN;
					gBc35CmdParser.Data.ConOvercurrentWarn = (pu8Data[i+4]-48) * 1000 + (pu8Data[i+5]-48) * 100 + (pu8Data[i+6]-48) * 10 + (pu8Data[i+7]-48);
					for(char j = 0; j<4; j += 2){						
						AT24CXX_WriteOneByte(ADDR_CCURRENTTH_RECORD+j/2,(pu8Data[i+4+j]-48)*10+(pu8Data[i+4+j+1]-48));
					}	
					AT24CXX_Read(ADDR_CCURRENTTH_RECORD,(uint8_t *)tmp,2);					
					SetOverCurrentValue(tmp[0],tmp[1]);	
					#ifdef HC32_LOG_DEBUG
					FIOT_LAB_LOG(LOG_DEBUG,"Find ConOvercurrentWarn = %d-%d\r",gBc35CmdParser.Data.ControlNum,gBc35CmdParser.Data.ConOvercurrentWarn);
					#endif					
					crcbuf[5] = gBc35CmdParser.Data.ControlNum;
					crcbuf[6] = gBc35CmdParser.Data.ConOvercurrentWarn;				
					i += 6;
					tmp_state = CMD_PARSER_STATE_CRC;	
				//ConOvercurrentTime
				}else if((gBc35CmdParser.Data.ControlNum = ((pu8Data[i]-48)*1000) + ((pu8Data[i+1]-48)*100) + ((pu8Data[i+2]-48)*10) + (pu8Data[i+3]-48)) == CMD_CODE_CYOVERCURRENTTIME){
					gBc35CmdParser.Data.ControlNum = CMD_CODE_CYOVERCURRENTTIME;
					//gBc35CmdParser.Data.ConOvercurrentTime = (pu8Data[i+4]-48) * 1000 + (pu8Data[i+5]-48) * 100 + (pu8Data[i+6]-48) * 10 + (pu8Data[i+7]-48);
					gBc35UserConCurrentTime = (pu8Data[i+4]-48) * 1000 + (pu8Data[i+5]-48) * 100 + (pu8Data[i+6]-48) * 10 + (pu8Data[i+7]-48);
					for(char j = 0; j<4; j += 2){						
						AT24CXX_WriteOneByte(ADDR_CCURRENTTIME_RECORD+j/2,(pu8Data[i+4+j]-48)*10+(pu8Data[i+4+j+1]-48));
					}									
					#ifdef HC32_LOG_DEBUG
					FIOT_LAB_LOG(LOG_DEBUG,"Find ConOvercurrentTime = %d-%d\r",gBc35CmdParser.Data.ControlNum,gBc35UserConCurrentTime);
					#endif
					crcbuf[5] = gBc35CmdParser.Data.ControlNum;
					crcbuf[6] = gBc35CmdParser.Data.ConOvercurrentTime;					
					i += 6;
					tmp_state = CMD_PARSER_STATE_CRC;					
				}else if((gBc35CmdParser.Data.ControlNum = ((pu8Data[i]-48)*1000) + (pu8Data[i+1]-48)*100 + (pu8Data[i+2]-48)*10 + (pu8Data[i+3]-48)) == CMD_CODE_MODIFYSERVER){
					//修改服务器
					gBc35CmdParser.Data.ControlNum = CMD_CODE_MODIFYSERVER;
					#if 0
					gBc35CmdParser.Data.Server.ServerPast1 = (pu8Data[i+4]-48) * 100 + (pu8Data[i+5]-48) * 10 + (pu8Data[i+6]-48); 
					gBc35CmdParser.Data.Server.ServerPast2 = (pu8Data[i+7]-48) * 100 + (pu8Data[i+8]-48) * 10 + (pu8Data[i+9]-48); 
					gBc35CmdParser.Data.Server.ServerPast3 = (pu8Data[i+10]-48) * 100 + (pu8Data[i+11]-48) * 10 + (pu8Data[i+12]-48); 
					gBc35CmdParser.Data.Server.ServerPast4 = (pu8Data[i+13]-48) * 100 + (pu8Data[i+14]-48) * 10 + (pu8Data[i+15]-48); 
					gBc35CmdParser.Data.Server.ServerPort = (pu8Data[i+16]-48) * 1000 + (pu8Data[i+17]-48) * 100 + (pu8Data[i+18]-48) * 10 + (pu8Data[i+19]-48);
					#ifdef HC32_LOG_DEBUG
					FIOT_LAB_LOG(LOG_DEBUG,"Find Server = %d-%d.%d.%d.%d:%d\r",gBc35CmdParser.Data.ControlNum, gBc35CmdParser.Data.Server.ServerPast1, gBc35CmdParser.Data.Server.ServerPast2, \
																														gBc35CmdParser.Data.Server.ServerPast3, gBc35CmdParser.Data.Server.ServerPast4, gBc35CmdParser.Data.Server.ServerPort);
					#endif				
					#else
				
					gBc28UserQlwOcip[0] = (pu8Data[i+4]-48) * 100 + (pu8Data[i+5]-48) * 10 + (pu8Data[i+6]-48);
					gBc28UserQlwOcip[1] = (pu8Data[i+7]-48) * 100 + (pu8Data[i+8]-48) * 10 + (pu8Data[i+9]-48); 
					gBc28UserQlwOcip[2] = (pu8Data[i+10]-48) * 100 + (pu8Data[i+11]-48) * 10 + (pu8Data[i+12]-48); 
					gBc28UserQlwOcip[3] = (pu8Data[i+13]-48) * 100 + (pu8Data[i+14]-48) * 10 + (pu8Data[i+15]-48); 
					gBc28UserQlwPort = (pu8Data[i+16]-48) * 1000 + (pu8Data[i+17]-48) * 100 + (pu8Data[i+18]-48) * 10 + (pu8Data[i+19]-48);	
					AT24CXX_Write(ADDR_IPSER_RECORD,(uint8_t *)gBc28UserQlwOcip,4);
					AT24CXX_WriteLenByte(ADDR_IPSER_RECORD+4,gBc28UserQlwPort,2);
					
					#ifdef HC32_LOG_DEBUG
					FIOT_LAB_LOG(LOG_DEBUG,"IP Server Already Store to Flash\r\n");
					#endif
					#ifdef HC32_LOG_DEBUG
					FIOT_LAB_LOG(LOG_DEBUG,"Find Server = %d.%d.%d.%d:%d\r\n",gBc28UserQlwOcip[0],gBc28UserQlwOcip[1],gBc28UserQlwOcip[2],gBc28UserQlwOcip[3],gBc28UserQlwPort);
					#endif
					#endif
					//CRC
					crcbuf[5] = gBc35CmdParser.Data.ControlNum;
					crcbuf[6] = gBc35CmdParser.Data.ConOvercurrentTime;					
					i += 18;
					tmp_state = CMD_PARSER_STATE_CRC;					
				}else if((gBc35CmdParser.Data.ControlNum = ((pu8Data[i]-48)*1000) + ((pu8Data[i+1]-48)*100) + ((pu8Data[i+2]-48)*10) + (pu8Data[i+3]-48)) == CMD_CODE_MODIFYSYSTIME){
					//修改时间
					
					gBc35CmdParser.Data.ControlNum = CMD_CODE_MODIFYSYSTIME;
					#if 0
					gBc35CmdParser.Data.Clock.Year = (pu8Data[i+4]-48) * 1000 + (pu8Data[i+5]-48) * 100 + (pu8Data[i+6]-48)*10 + (pu8Data[i+7]-48); 
					gBc35CmdParser.Data.Clock.Month = (pu8Data[i+8]-48)*10 + (pu8Data[i+9]-48); 
					gBc35CmdParser.Data.Clock.Day = (pu8Data[i+10]-48)*10 + (pu8Data[i+11]-48); 
					gBc35CmdParser.Data.Clock.Hour = (pu8Data[i+12]-48)*10 + (pu8Data[i+13]-48); 
					gBc35CmdParser.Data.Clock.Min = (pu8Data[i+14]-48)*10 + (pu8Data[i+15]-48);			
					gBc35CmdParser.Data.Clock.Sec = (pu8Data[i+16]-48)*10 + (pu8Data[i+17]-48);
					#ifdef HC32_LOG_DEBUG
					FIOT_LAB_LOG(LOG_DEBUG,"Find Time = %d,%d-%02d-%02d %02d:%02d:%02d\r",gBc35CmdParser.Data.ControlNum, gBc35CmdParser.Data.Clock.Year, gBc35CmdParser.Data.Clock.Month, \
																					gBc35CmdParser.Data.Clock.Day, gBc35CmdParser.Data.Clock.Hour, gBc35CmdParser.Data.Clock.Min, gBc35CmdParser.Data.Clock.Sec);
					#endif				
					#else
					stc_rtc_time_t readtime;
					readtime.u8Year = DEC2BCD((pu8Data[i+4]-48) * 1000 + (pu8Data[i+5]-48) * 100 + (pu8Data[i+6]-48)*10 + (pu8Data[i+7]-48)-2000); 
					readtime.u8Month = DEC2BCD((pu8Data[i+8]-48)*10 + (pu8Data[i+9]-48)); 
					readtime.u8Day = DEC2BCD((pu8Data[i+10]-48)*10 + (pu8Data[i+11]-48)); 
					readtime.u8Hour = DEC2BCD((pu8Data[i+12]-48)*10 + (pu8Data[i+13]-48)); 
					readtime.u8Minute = DEC2BCD((pu8Data[i+14]-48)*10 + (pu8Data[i+15]-48));			
					readtime.u8Second = DEC2BCD((pu8Data[i+16]-48)*10 + (pu8Data[i+17]-48));
					if(Rtc_SetTime(&readtime) != Ok){
						#ifdef HC32_LOG_SUPPORTS
						FIOT_LAB_LOG(LOG_ERROR,"RTC Setting IS ERROR\r\n");
						#endif
					}else{
						#ifdef HC32_LOG_SUPPORTS
						FIOT_LAB_LOG(LOG_ERROR,"RTC Time Already Updata\r\n");
						#endif									
					}				
					#endif

					crcbuf[5] = gBc35CmdParser.Data.ControlNum;
					crcbuf[6] = gBc35CmdParser.Data.Clock.Year;		
					crcbuf[7] = gBc35CmdParser.Data.Clock.Month;	
					crcbuf[8] = gBc35CmdParser.Data.Clock.Day;
					crcbuf[9] = gBc35CmdParser.Data.Clock.Hour;		
					crcbuf[10] = gBc35CmdParser.Data.Clock.Min;	
					crcbuf[11] = gBc35CmdParser.Data.Clock.Sec;					
					i += 16;
					tmp_state = CMD_PARSER_STATE_CRC;					
				}else if((gBc35CmdParser.Data.ControlNum = ((pu8Data[i]-48)*1000) + ((pu8Data[i+1]-48)*100) + ((pu8Data[i+2]-48)*10) + (pu8Data[i+3]-48)) == CMD_CODE_BATTERYTHRELD){
					gBc35CmdParser.Data.ControlNum = CMD_CODE_BATTERYTHRELD;
					gBc35CmdParser.Data.Battery = (pu8Data[i+4]-48) * 1000 + (pu8Data[i+5]-48) * 100 + (pu8Data[i+6]-48) * 10 + (pu8Data[i+7]-48);
					#ifdef HC32_LOG_DEBUG
					FIOT_LAB_LOG(LOG_DEBUG,"Find Battery = %d-%d\r",gBc35CmdParser.Data.ControlNum,gBc35CmdParser.Data.Battery);
					#endif
					crcbuf[5] = gBc35CmdParser.Data.ControlNum;
					crcbuf[6] = gBc35CmdParser.Data.Battery;				
					i += 6;
					tmp_state = CMD_PARSER_STATE_CRC;					
				}else if((gBc35CmdParser.Data.ControlNum = ((pu8Data[i]-48)*1000) + ((pu8Data[i+1]-48)*100) + ((pu8Data[i+2]-48)*10) + (pu8Data[i+3]-48)) == CMD_CODE_MODIFYAPN){
					//未实现修改APN
					gBc35CmdParser.Data.ControlNum = CMD_CODE_MODIFYAPN;
					#ifdef HC32_LOG_DEBUG
					FIOT_LAB_LOG(LOG_DEBUG,"Find APN = %d\r",gBc35CmdParser.Data.ControlNum);
					#endif
					//CRC未实现
					crcbuf[5] = gBc35CmdParser.Data.ControlNum;
					crcbuf[6] = gBc35CmdParser.Data.Battery;					
					i += 6;
					tmp_state = CMD_PARSER_STATE_CRC;					
				}else if((gBc35CmdParser.Data.ControlNum = ((pu8Data[i]-48)*1000) + ((pu8Data[i+1]-48)*100) + ((pu8Data[i+2]-48)*10) + (pu8Data[i+3]-48)) == CMD_CODE_MODIFYWNUM){
					//Modify Water Num
					gBc35CmdParser.Data.ControlNum = CMD_CODE_MODIFYWNUM;
					#ifdef HC32_LOG_DEBUG
					FIOT_LAB_LOG(LOG_DEBUG,"Find Water Num = %d\r",gBc35CmdParser.Data.ControlNum);
					#endif
					for(char j = 0; j<8; j += 2){						
						AT24CXX_WriteOneByte(ADDR_WNUM_RECORD+j/2,(pu8Data[i+4+j]-48)*10+(pu8Data[i+4+j+1]-48));
					}				
					//CRC未实现
					crcbuf[5] = gBc35CmdParser.Data.ControlNum;
					crcbuf[6] = gBc35CmdParser.Data.Battery;					
					i += 6;
					tmp_state = CMD_PARSER_STATE_CRC;					
				}else if((gBc35CmdParser.Data.ControlNum = ((pu8Data[i]-48)*1000) + ((pu8Data[i+1]-48)*100) + ((pu8Data[i+2]-48)*10) + (pu8Data[i+3]-48)) == CMD_CODE_WATERAMOUNT){
					char tmp[8] = {0};
					//Modify 水量
					gBc35CmdParser.Data.ControlNum = CMD_CODE_WATERAMOUNT;
					for(char j = 0; j<8; j += 2){						
						AT24CXX_WriteOneByte(ADDR_StorePcnt_START+j/2,(pu8Data[i+4+j]-48)*10+(pu8Data[i+4+j+1]-48));
					}	
					AT24CXX_Read(ADDR_StorePcnt_START,(uint8_t *)tmp,4);
					#ifdef HC32_LOG_DEBUG
					FIOT_LAB_LOG(LOG_DEBUG,"CMD_CODE_WATERAMOUNT Num : %02d%02d%02d%02d\r",tmp[0],tmp[1],tmp[2],tmp[3]);
					#endif	
					WaterPcntClear(tmp[0]*100 + tmp[1],tmp[2]*10 + tmp[3]/10);					
					//CRC未实现
					crcbuf[5] = gBc35CmdParser.Data.ControlNum;
					crcbuf[6] = gBc35CmdParser.Data.Battery;					
					i += 6;
					tmp_state = CMD_PARSER_STATE_CRC;					
				}else if((gBc35CmdParser.Data.ControlNum = ((pu8Data[i]-48)*1000) + ((pu8Data[i+1]-48)*100) + ((pu8Data[i+2]-48)*10) + (pu8Data[i+3]-48)) == CMD_CODE_IMMEDWARN){
					gBc35CmdParser.Data.ControlNum = CMD_CODE_IMMEDWARN;
					#ifdef HC32_LOG_DEBUG
					FIOT_LAB_LOG(LOG_DEBUG,"Find CMD_CODE_IMMEDWARN = %d\r",gBc35CmdParser.Data.ControlNum);
					#endif
					
					//0111  0000 0001
					AT24CXX_WriteOneByte(ADDR_IMWARN_RECORD,(pu8Data[i+4]-48)<<2 | (pu8Data[i+5]-48)<<1 | (pu8Data[i+6]-48));
					FIOT_LAB_LOG(LOG_DEBUG,"%d\r\n",(pu8Data[i+4]-48) << 2 | (pu8Data[i+5]-48)<<1 | (pu8Data[i+6]-48));
					gBc35UserImmediatelyWarn = (pu8Data[i+4]-48)<<2 | (pu8Data[i+5]-48)<<1 | (pu8Data[i+6]-48);
					
					#ifdef HC32_LOG_DEBUG
					FIOT_LAB_LOG(LOG_DEBUG,"CMD_CODE_IMMEDWARN Num : %d\r",gBc35UserImmediatelyWarn);
					#endif	
					
					//CRC未实现
					crcbuf[5] = gBc35CmdParser.Data.ControlNum;
					crcbuf[6] = gBc35CmdParser.Data.Battery;					
					i += 6;
					tmp_state = CMD_PARSER_STATE_CRC;	
				}else if((gBc35CmdParser.Data.ControlNum = ((pu8Data[i]-48)*1000) + ((pu8Data[i+1]-48)*100) + ((pu8Data[i+2]-48)*10) + (pu8Data[i+3]-48)) == CMD_CODE_DENSEPERIOD){
					gBc35CmdParser.Data.ControlNum = CMD_CODE_DENSEPERIOD;
					#ifdef HC32_LOG_DEBUG
					FIOT_LAB_LOG(LOG_DEBUG,"Find CMD_CODE_DENSEPERIOD = %d\r",gBc35CmdParser.Data.ControlNum);
					#endif
					
					gBc28period.Year =  DEC2BCD((pu8Data[i+4]-48)*10 + (pu8Data[i+5]-48));
					gBc28period.Month = DEC2BCD((pu8Data[i+6]-48)*10 + (pu8Data[i+7]-48));
					gBc28period.Day = DEC2BCD((pu8Data[i+8]-48)*10 + (pu8Data[i+9]-48));
					gBc28period.Hour =  DEC2BCD((pu8Data[i+10]-48)*10 + (pu8Data[i+11]-48));
					gBc28period.Min = DEC2BCD((pu8Data[i+12]-48)*10 + (pu8Data[i+13]-48));
					gBc28period.PriodStartFlag = 1;//open
					// gBc28period.Sec = (pu8Data[i+16]-48)*10 + (pu8Data[i+17]-48);	
					
					#ifdef HC32_LOG_DEBUG
					FIOT_LAB_LOG(LOG_DEBUG,"Find time for centralized reporting = %d.%d.%d.%d:%d\r",BCD2DEC(gBc28period.Year), BCD2DEC(gBc28period.Month),BCD2DEC(gBc28period.Day), \
																														BCD2DEC(gBc28period.Hour),BCD2DEC(gBc28period.Min));
					#endif					
					//CRC未实现
					crcbuf[5] = gBc35CmdParser.Data.ControlNum;
					crcbuf[6] = gBc35CmdParser.Data.Battery;					
					i += 6;
					tmp_state = CMD_PARSER_STATE_CRC;					
				}else{
					tmp_state = CMD_PARSER_STATE_Error;
				}				
				break;
			case CMD_PARSER_STATE_CRC:
				//CRC 未处理,代码未写
				u16RefCrc16 = HC32_CRC16_Get16((uint16_t *)crcbuf,sizeof(crcbuf), 16);
				sprintf(cmptmp,"%x",u16RefCrc16);	
				for(j = 0;j < 4;j++){
					tmp[j] = pu8Data[i+j];
				}			
				if(strcmp(tmp,cmptmp) == 0){
					#ifdef HC32_LOG_DEBUG
					tmp_state = CMD_PARSER_STATE_End;
					FIOT_LAB_LOG(LOG_DEBUG,"CRC Check Succeed!\r\n");	
					#endif
				}else{
					#ifdef HC32_LOG_DEBUG	
					tmp_state = CMD_PARSER_STATE_Error;
					FIOT_LAB_LOG(LOG_DEBUG,"CRC Check False!\r\n");	
					#endif
				}
				i += 2;
				break;
			case CMD_PARSER_STATE_End:
				if(((pu8Data[i]-48)*10 + (pu8Data[i+1]-48)) == CMD_SYNC_END){
					gBc35CmdParser.End = CMD_SYNC_END;
					tmp_state = CMD_PARSER_STATE_Ok;
					#ifdef HC32_LOG_DEBUG
					FIOT_LAB_LOG(LOG_DEBUG,"Find End = 0x%d\r",gBc35CmdParser.End);
					#endif
				}else{
					tmp_state = CMD_PARSER_STATE_Error;
				}				
				break;
			case CMD_PARSER_STATE_Error:
				#ifdef HC32_LOG_DEBUG	
				FIOT_LAB_LOG(LOG_DEBUG,"Parser Faile!\r");	
				#endif
				i += u16DataLen;//exit
				break;
		}
		if(tmp_state == CMD_PARSER_STATE_Error)
			return 0;
	}
	return 1;
}

#endif
static void Bc35UserStatusInit(void){
	gBc35UserStatusIndex = 0;
    gBc35UserNextStatus = BC35STATUS_NONE;
    gBc35UserPrevStatus = BC35STATUS_NONE;
	Bc35UserATSend();
	//Warn Mode Read IP Serv Configure from Flash.	
	AT24CXX_Read(ADDR_IPSER_RECORD,(unsigned char *)gBc28UserQlwOcip,4);
	gBc28UserQlwPort = AT24CXX_ReadLenByte(ADDR_IPSER_RECORD+4,2);
	if((gBc28UserQlwOcip[0] == 0xFF) && (gBc28UserQlwOcip[1] == 0xFF) && (gBc28UserQlwOcip[2] == 0xFF) && (gBc28UserQlwOcip[3] == 0xFF)){
		memset(gBc28UserQlwOcip, 0, sizeof(gBc28UserQlwOcip));
		#ifdef HC32_LOG_INFO
		FIOT_LAB_LOG(LOG_INFO,"IP Server Default Setting\r\n");
		#endif		
	}else{
		#ifdef HC32_LOG_INFO
		FIOT_LAB_LOG(LOG_INFO,"IP Server : %d.%d.%d.%d:%d\r\n",gBc28UserQlwOcip[0],gBc28UserQlwOcip[1],\
														   gBc28UserQlwOcip[2],gBc28UserQlwOcip[3],gBc28UserQlwPort);
		#endif
	}
	//Warn Mode Read Water Amount from Flash.
	AT24CXX_Read(ADDR_WNUM_RECORD,(uint8_t *)gBc28UserWaterDevNum,4);
	if((gBc28UserWaterDevNum[0] == 0xFF) && (gBc28UserWaterDevNum[1] == 0XFF) && (gBc28UserWaterDevNum[2] == 0XFF) && (gBc28UserWaterDevNum[3] == 0XFF)){
		memset(gBc28UserWaterDevNum, 0, sizeof(gBc28UserWaterDevNum));
		#ifdef HC32_LOG_DEBUG
		FIOT_LAB_LOG(LOG_DEBUG,"Water Num Not Setting\r");
		#endif			
	}else{
		#ifdef HC32_LOG_DEBUG
		FIOT_LAB_LOG(LOG_DEBUG,"Water Num : %02d%02d%02d%02d\r",gBc28UserWaterDevNum[0],gBc28UserWaterDevNum[1],gBc28UserWaterDevNum[2],gBc28UserWaterDevNum[3]);
		#endif	
	}		
	//Warn Mode Read Configure from Flash.
	if(AT24CXX_ReadOneByte(ADDR_IMWARN_RECORD) != 1){
		gBc35UserImmediatelyWarn = 0;
		#ifdef HC32_LOG_DEBUG
		FIOT_LAB_LOG(LOG_DEBUG,"Normol Warn Mode(default) : %d\r\n",gBc35UserImmediatelyWarn);
		#endif		
	}else{
		gBc35UserImmediatelyWarn = 1;
		#ifdef HC32_LOG_DEBUG
		FIOT_LAB_LOG(LOG_DEBUG,"Immediatelt Warn Mode : %d\r\n",gBc35UserImmediatelyWarn);
		#endif		
	}					
	if(AT24CXX_ReadOneByte(ADDR_DATAFREQUENCY_RECORD) != 0xff && AT24CXX_ReadOneByte(ADDR_DATAFREQUENCY_RECORD+1) != 0xff){
		gBc35UserSendDataFrequency = AT24CXX_ReadOneByte(ADDR_DATAFREQUENCY_RECORD)* 100 + AT24CXX_ReadOneByte(ADDR_DATAFREQUENCY_RECORD+1);
	}else{	
	}
	#ifdef HC32_LOG_DEBUG
	FIOT_LAB_LOG(LOG_DEBUG,"SendDataFrequency : %d Minute\r\n",gBc35UserSendDataFrequency);
	#endif			
}
/********************************************************************************
  ** \brief  LOG命令解析函数函数
  ** 
  ** @param  null
  ** 
  ** \retval Ok or Faile
******************************************************************************/
void Bc35UserLogMsgParser(void){
	uint8_t ret = 0;
	uint8_t index = FIOT_LAB_LOG_PARSER();
	switch(index)
	{
		case BC35STATUS_AT:	
				ret = Bc35UserAT();
				break;
		case BC35STATUS_VERSION:
				ret = Bc35UserGetVerison();			
				break;
		case BC35STATUS_REBOOT:
				ret = Bc35UserReboot();
				break;
		case BC35STATUS_BAND:
				ret = Bc35UserGetBand();
				break;
		case BC35STATUS_DATATIME:
				ret = Bc35UserGetDateTime();			
				break;
		case BC35STATUS_NUESTATUS:
		#ifdef HC32_L161_SUPPORTS
				ret = Bc35UserGetNueStatus();			
		#endif
		#ifdef HC32_BC28_SUPPORTS		
				ret = Bc28UserGetNueStatus();	
		#endif		
				break;
		case BC35STATUS_QCCID:
				ret = Bc35UserGetQccid();			
				break;
		case BC35STATUS_QIMEI:		
				ret = Bc35UserGetQimei();		
				break;
		case BC35STATUS_QIMSI:
				ret = Bc35UserGetQimsi();			
				break;
		case BC35STATUS_CGATT:
				ret = Bc35UserGetCgatt();			
				break;
		case BC35STATUS_CSQ:
				ret = BC35UserGetCsq();			
				break;
		case BC35STATUS_CEREG:
				ret = Bc35UserGetCeReg();			
				break;
		case 51://rebootDevice
			#ifdef HC32_LOG_SUPPORTS
			FIOT_LAB_LOG(LOG_FATAL,"DEV REBOOTING...\r\n");
			#endif
			NVIC_SystemReset();  //复位MCU				
		  break;
		case 52:
		#ifdef HC32_L161_SUPPORTS			
				ret = Bc35UserQlwServSend();
		#endif
		#ifdef HC32_BC28_SUPPORTS
				ret = Bc28UserQlwServSend();
		#endif		
			break;
		case 53:
				ret = Bc35UserSetCtm2mReg();
			break;
		case 54:
				ret = Bc35UserSetCtm2mUpData();
			break;
		case 55:
		FIOT_LAB_LOG(LOG_INFO,"Send Data OC\r\n");
		gBc35UserGetDataStatus = 0;
		gBc35UserSendDataStatus = 0;
		#ifdef HC32_L161_SUPPORTS			
				ret = Bc35UserQlwSendDataSend(OceanFlatSendMessId, DEV_CCODE_ALARMDATA, \
												DEV_ECODE_DEVDESTROY, 0);
		#endif
		#ifdef HC32_BC28_SUPPORTS
				ret = Bc28UserQlwSendDataSend(OceanFlatSendMessId, DEV_CCODE_ALARMDATA, \
												DEV_ECODE_DEVDESTROY, 0);
		#endif		
			break;
		case 56:
				ret = Bc35UserSetCtm2mEReg();
			break;
		case 57:
				ret = Bc35UserGetBcInfo();
			break;	
		case 58://Open BEEP
			HC32_BEEP_Hight();
			FIOT_LAB_LOG(LOG_DEBUG,"HC32 Beep(PF03) Already Draw High(1)!\r\n");
			break;
		case 59://Close Beep
			HC32_BEEP_Low();
			FIOT_LAB_LOG(LOG_DEBUG,"HC32 Beep(PF03) Already Draw Low(0)!\r\n");
			break;	
		case 60://Open NB Power
			HC32_NBPWR_Hight();
			FIOT_LAB_LOG(LOG_DEBUG,"HC32 NB PWR(PC11) Already Draw Hght(1)!\r\n");
			break;	
		case 61://Close NB Power
			HC32_NBPWR_Low();
			FIOT_LAB_LOG(LOG_DEBUG,"HC32 NB PWR(PC11) Already Draw Low(0)!\r\n");
			break;	
		case 62://Open SE Rst
			SE_REST_Hight();
			FIOT_LAB_LOG(LOG_DEBUG,"HC32 SE RST(PD12) Already Draw High(1)!\r\n");
			break;	
		case 63://Close SE Rst
			SE_REST_Low();
			FIOT_LAB_LOG(LOG_DEBUG,"HC32 SE RST(PD12) Already Draw Low(0)!\r\n");
			break;	
		case 64://Open NB Rst
			HC32_NBRST_Hight();
			FIOT_LAB_LOG(LOG_DEBUG,"HC32 NB RST(PC10) Already Draw High(1)!\r\n");
			break;	
		case 65://Close NB Rst
			HC32_NBRST_Low();
			FIOT_LAB_LOG(LOG_DEBUG,"HC32 NB RST(PC10) Already Draw Low(0)!\r\n");
			break;	
		case 66://Open WP 
			HC32_WP_Hight();
			FIOT_LAB_LOG(LOG_DEBUG,"HC32 WP(PF06) Already Draw High(1)!\r\n");
			break;	
		case 67://Close WP
			HC32_WP_Low();
			FIOT_LAB_LOG(LOG_DEBUG,"HC32 WP(PF06) Already Draw Low(0)!\r\n");
			break;	
		case 68://Open Bat 
			HC32_BATTERY_Hight();
			FIOT_LAB_LOG(LOG_DEBUG,"HC32 Battery(PA15) Already Draw High(1)!\r\n");
			break;	
		case 69://Close Bat
			HC32_BATTERY_Low();
			FIOT_LAB_LOG(LOG_DEBUG,"HC32 Battery(PA15) Already Draw Low(0)!\r\n");
			break;	
		case 70://Open Wake
			HC32_NBWAKE_Hight();
			FIOT_LAB_LOG(LOG_DEBUG,"HC32 NB WAKE(PD03) Already Draw High(1)!\r\n");
			break;	
		case 71://Close Wake
			HC32_NBWAKE_Low();
			FIOT_LAB_LOG(LOG_DEBUG,"HC32 NB WAKE(PD03) Already Draw Low(0)!\r\n");	
			break;	
		case 72://Open Pwm
			HC32_Valve_Open();
			FIOT_LAB_LOG(LOG_DEBUG,"HC32 PWM(PE8-9) Already Open!\r\n");	
			break;	
		case 73://Close Pwm
			HC32_Valve_Close();
			FIOT_LAB_LOG(LOG_DEBUG,"HC32 PWM(PE8-9) Already Close!\r\n");	
			break;	
		case 74://Check EEprom
			ret = AT24CXX_Check();
			if(ret)
				FIOT_LAB_LOG(LOG_DEBUG,"HC32 EEPROM(PB10-11) Already Check,Result:Fail\r\n");	
			else
				FIOT_LAB_LOG(LOG_DEBUG,"HC32 EEPROM(PB10-11) Already Check,Result:Succeed\r\n");				
			break;	
		case 75://Read Key
			FIOT_LAB_LOG(LOG_DEBUG,"HC32 Read Key(PC07) Level : %d\r\n",HC32_Read_Key());	
			break;	
		case 76://Open CLK
			SE_CLK_ON();
			FIOT_LAB_LOG(LOG_DEBUG,"HC32 Se Clk Already Open!\r\n");	
			break;	
		case 77://Close CLK
			SE_CLK_OFF();
			FIOT_LAB_LOG(LOG_DEBUG,"HC32 Se Clk Already Close!\r\n");	
			break;	
		case 78://Read Battery 
			FIOT_LAB_LOG(LOG_DEBUG,"HC32 Read Battery Value: %f  BatteryLevel : %d!\r\n",BatteryObtain(),BatteryLevel());	
			break;	
		case 79://NB Enter Sleep
			Bc35UerSetPsmMode(1);
			#ifdef HC32_L161_SUPPORTS
			Bc35UserEnterSleep(0);
			FIOT_LAB_LOG(LOG_DEBUG,"HC32 Enter Sleep mode 0\r\n");			
			#endif
			break;	
		case 80://NB Get Pcnt
			FIOT_LAB_LOG(LOG_DEBUG,"HC32 Read Pcnt is %d,Ton is %d.%d\r\n",WaterFlowStatistics(),WaterReturnInt(),WaterReturnFloat());	
			break;	
		case 81://Modify IMEI	
			break;	
		case 82://Send Warn
			u8DeviceAlarm = 1;
			break;			
		}
		if(!ret && index < 58){
				#ifdef HC32_LOG_SUPPORTS				
				FIOT_LAB_LOG(LOG_FATAL,"ERROR\r\n");	
				#endif				
		}
}

static int Bc35UserGetNMGRSend(void){
    return Bc35UartUserSend(BC35_NMGR_READ);
}
static int Bc28UserGetNMGR(void){
    char *p = 0,*p1 = 0,*p2 = 0;
	char tmpbuf[64] = {0};
	char ret = 0;
	Bc35UserGetNMGRSend();
	delay1ms(200);
	memcpy(gBc35RecvBuf, gBc35UartRecvBuf, sizeof(gBc35UartRecvBuf));
    if(0 != (p = strstr((char *)gBc35RecvBuf, BC35_OK_REPLY))){
        if(0 != (p1 = strstr((char *)gBc35RecvBuf, ","))){
			//receive NNMI data	
			if((p = strstr((char *)gBc35RecvBuf, "\r\n")) != 0){
				p += strlen("\r\n");
				while(*p == '\r'||*p == '\n'||*p == 'N'||*p == '+'||*p == 'M'||*p == 'I' || *p == ':'||*p == ' ') p++;
				memset(gBc35Nngr.len,0,sizeof(gBc35Nngr.len));
				memcpy(gBc35Nngr.len, p, p1 - p);
				if(0 != (p2 = strstr((char *)p, "\r\n"))){
					if(*p1 == ',') p1++;
					memset(gBc35Nngr.rec,0,sizeof(gBc35Nngr.rec));
					memcpy(gBc35Nngr.rec, p1, p2 - p1);
					#ifdef HC32_LOG_SUPPORTS
					FIOT_LAB_LOG(LOG_DEBUG,"OceanFlat Rec:%s,%s\r", gBc35Nngr.len, gBc35Nngr.rec);
					#endif
					if(0 != strstr((char *)gBc35Nngr.rec, BC35_NNMI_RESPONSE)){
						#ifdef HC32_LOG_SUPPORTS
						FIOT_LAB_LOG(LOG_DEBUG,"OceanFlat Response!\r");
						#endif	
						gBc35UserSendDataRepetition = 0;
					}
					else
					{
						HexStrToAsciiStr(gBc35Nngr.rec,(char *)tmpbuf, sizeof(tmpbuf));	
						#ifdef HC32_LOG_DEBUG								
						FIOT_LAB_LOG(LOG_DEBUG,"Hander:%s-%d\r\n",tmpbuf,strlen(tmpbuf));
						#endif	
						//command manage
						ret = OceanConnect_cmd_parser((unsigned char *)tmpbuf,(uint16_t)gBc35Nngr.len);
						if(ret == 1){
							//cmd parser succeed
							#ifdef HC32_LOG_DEBUG								
							FIOT_LAB_LOG(LOG_DEBUG,"Oc Cmd Parser Succeed = %d\r\n",ret);
							#endif	
						}else{
							//cmd parser false
							#ifdef HC32_LOG_DEBUG								
							FIOT_LAB_LOG(LOG_DEBUG,"Oc Cmd Parser Faile = %d\r\n",ret);
							#endif										
						}
					}
					*p = 0;*p1 = 0;*p2 = 0;
					//logical handler
				}							
			}
        }
		return 1;
    }		
	return 0;
}

/********************************************************************************
  ** \brief  AT命令流程函数函数
  ** 
  ** @param  null
  ** 
  ** \retval 0:Faile  1:Succeed
******************************************************************************/
static void Bc35UserUartMsgSend(void){
    int ret = 0;
    Enum_BC35STATUS gBc35UserStatusListOnce[] = {
		BC35STATUS_VERSION,
		BC35STATUS_QIMEI,
		BC35STATUS_BAND,
		BC35STATUS_CGATT,
		BC35STATUS_QIMSI,		
		BC35STATUS_DATATIME,
		BC35STATUS_NUESTATUS,
		BC35STATUS_CSQ,
		BC35STATUS_EDRX,
		BC35STATUS_CPSMS,
		BC35STATUS_CFUN,
		BC35STATUS_CEREG,
		BC35STATUS_QCCID,		
		BC35STATUS_NNMI,
    };
	
	#ifdef HC32_BC28_SUPPORTS		
    Enum_BC35STATUS gBc35UserStatusList[] = {
        BC35STATUS_NMGR,
    };	
	#endif		
	
	#ifdef HC_BC35_OCEANCONNECT_SUPPORT
	
	#ifdef HC32_L161_SUPPORTS
    Enum_BC35STATUS gBc35UserStatusListOceanConnet[] = {
		BC35STATUS_CGATT,
		BC35STATUS_DATATIME,
		BC35STATUS_NUESTATUS,
		BC35STATUS_NCDP,
		BC35STATUS_CTM2MREG,
		BC35STATUS_NMGS,
		BC35STATUS_CTM2MDEREG,
    };
	#endif
	
	#ifdef HC32_BC28_SUPPORTS	
    Enum_BC35STATUS gBc35UserStatusListOceanConnet[] = {
		BC35STATUS_CGATT,
		BC35STATUS_NNMI,		
		BC35STATUS_DATATIME,
		BC35STATUS_NUESTATUS,
		BC35STATUS_NCDP,
		BC35STATUS_NMGS,
    };
	#endif	
	
	#endif
	
    if(gBc35UserNextStatus == BC35STATUS_NONE){
		//小于状态流程gBc35UserStatusListOnce
        if(gBc35UserStatusIndex < HcArraySize(gBc35UserStatusListOnce)){
            gBc35UserNextStatus = gBc35UserStatusListOnce[gBc35UserStatusIndex];
		#ifdef HC_BC35_OCEANCONNECT_SUPPORT
		//小于状态流程gBc35UserStatusListOceanConnet				
		}else if(gBc35UserStatusIndex < (HcArraySize(gBc35UserStatusListOnce) + HcArraySize(gBc35UserStatusList))){
            gBc35UserNextStatus = gBc35UserStatusList[gBc35UserStatusIndex - HcArraySize(gBc35UserStatusListOnce)];
        }else if(gBc35UserStatusIndex < (HcArraySize(gBc35UserStatusListOnce) + HcArraySize(gBc35UserStatusList) + HcArraySize(gBc35UserStatusListOceanConnet))){
            gBc35UserNextStatus = gBc35UserStatusListOceanConnet[gBc35UserStatusIndex - HcArraySize(gBc35UserStatusListOnce) - HcArraySize(gBc35UserStatusList)];		
		#endif
		}else{//超过最大值
			gBc35UserStatusIndex = 0;
			gBc35UserNextStatus = gBc35UserStatusListOnce[gBc35UserStatusIndex];				
		}			
				
		//状态遍历
		gBc35UserStatusIndex++;		
				
		#ifdef HC_BC35_OCEANCONNECT_SUPPORT
		if(u8DeviceAlarm == 1 || u8DeviceWarning == 1 || u8DeviceOverCurrent == 1){//日上报数据
		//执行完成 		
		if(gBc35UserStatusIndex >= (HcArraySize(gBc35UserStatusListOnce) + HcArraySize(gBc35UserStatusList) + HcArraySize(gBc35UserStatusListOceanConnet))){
				gBc35UserStatusIndex = HcArraySize(gBc35UserStatusListOnce);
				gBc35UserQlwConnectStatus = 0;
			}					
		}else
		if(gBc35UserStatusIndex >= (HcArraySize(gBc35UserStatusListOnce) + HcArraySize(gBc35UserStatusList))){
			gBc35UserStatusIndex = HcArraySize(gBc35UserStatusListOnce);
		}
		#endif		
	}
		
	switch(gBc35UserNextStatus){
		case BC35STATUS_REBOOT:
			ret = Bc35UserReboot();
			break;
		case BC35STATUS_VERSION:
			ret = Bc35UserGetVerison();
			break;
		case BC35STATUS_DATATIME:
			ret = Bc35UserGetDateTime();	
			break;
		case BC35STATUS_NUESTATUS:
			ret = Bc28UserGetNueStatus();		
			break;
		case BC35STATUS_QCCID:
			ret = Bc35UserGetQccid();
			break;
		case BC35STATUS_BAND:
			ret = Bc35UserGetBand();
			break;
		case BC35STATUS_QIMEI:
			ret = Bc35UserGetQimei();
			break;
		case BC35STATUS_QIMSI:
			ret = Bc35UserGetQimsi();
			break;
		case BC35STATUS_CGATT:
			ret = Bc35UserGetCgatt();
			break;
		case BC35STATUS_CSQ:
			ret = BC35UserGetCsq();
			break;
		case BC35STATUS_CEREG:
			ret = Bc35UserGetCeReg();
			break;
		case BC35STATUS_EDRX:
			ret = Bc35UserGeteDRX();
			break;
		case BC35STATUS_CPSMS:
			ret = Bc35UserGetPsmMode();
			break;
		case BC35STATUS_CFUN:
			ret = Bc35UserGetCfun();
			break;
		case BC35STATUS_NNMI:
			ret = Bc35UserSetNnmi(BC35_NNMI_TWO);
			break;
			#ifdef HC_BC35_OCEANCONNECT_SUPPORT
		case BC35STATUS_NCDP:
			#ifdef HC32_BC28_SUPPORTS
			ret = Bc28UserQlwServSend();
			#endif		
			break;		
		case BC35STATUS_CTM2MREG:
			ret = Bc35UserSetCtm2mReg();
			break;
		case BC35STATUS_CTM2MUPDATE:
			ret = Bc35UserSetCtm2mUpData();
			break;
		case BC35STATUS_NMGS:		
			if(u8DeviceAlarm == 1){
			#ifdef HC32_BC28_SUPPORTS
				ret = Bc28UserQlwSendDataSend(OceanFlatSendMessId, DEV_CCODE_METERDATA, \
											DEV_ECODE_NORMAL, 0);
			#endif
			}else if(u8DeviceWarning == 1){
			#ifdef HC32_BC28_SUPPORTS
				ret = Bc28UserQlwSendDataSend(OceanFlatSendMessId, DEV_CCODE_ALARMDATA, \
											DEV_ECODE_DEVDESTROY, 0);
			#endif
			}else if(u8DeviceOverCurrent == 1){
			#ifdef HC32_BC28_SUPPORTS
				ret = Bc28UserQlwSendDataSend(OceanFlatSendMessId, DEV_CCODE_ALARMDATA, \
											DEV_ECODE_OVERCURRENT, 0);
			#endif			
			}				
			break;
		case BC35STATUS_CTM2MDEREG:	
			FIOT_LAB_LOG(LOG_FATAL,"Bc35UserSetCtm2mReg\r");					
			ret = Bc35UserSetCtm2mEReg();
			break;
		#endif
		case BC35STATUS_NMGR:
			ret = Bc28UserGetNMGR();
			break;		
		case BC35STATUS_SETEDRX:
			ret = Bc35UserSeteDRX(BC35_EDRX_ENABLE,BC35_EDRX_NBS1,BC35_EDRX_20P48SEC);
			break;
		case BC35STATUS_SETCPSMS:
			ret = Bc35UerSetPsmMode(BC35_PSM_MODE_ENABLE);
			break;
		case BC35STATUS_SETCFUN:
			ret = Bc35UserSetCfun(1);
			break;
		case BC35STATUS_SETCGATT:
			ret = Bc35UerSetCgatt(1);
			break;
		default:
			break;
	}
		
    if(ret){//send ok clear status
		if(gBc35UserNextStatus != BC35STATUS_BUSY){
			gBc35UserGetDataError = 0;
			gBc35UserNextStatus = BC35STATUS_NONE;
		}
		if(gBc35UserGetDataStatus){
			LpTimer_Disable();
			gBc35UserGetDataStatus = 0;
			gLptimSecRecord = 0;			
		}
    }
	else //Return error
	{
//		
		gBc35UserWaitResponse = 1;
//		
//		//Start Lptimer
		LpTimer_Enable();
//		
		if(gBc35UserGetDataStatus == 2){
			
			if(u8DeviceWarning == 1)
				u8DeviceWarning = 0;
			
			if(u8DeviceAlarm == 1)
				u8DeviceAlarm = 0;
			
			if(u8DeviceOverCurrent == 1)
				u8DeviceOverCurrent = 0;
			
			gBc35UserNextStatus = BC35STATUS_NONE;
			if(gBc35UserStatusIndex < (HcArraySize(gBc35UserStatusListOnce)))
				gBc35UserStatusIndex = 0;
			else if(gBc35UserStatusIndex < (HcArraySize(gBc35UserStatusListOnce) + HcArraySize(gBc35UserStatusListOceanConnet)))
				gBc35UserStatusIndex = HcArraySize(gBc35UserStatusListOnce);			
		}else{
			//get NB data failed
			gBc35UserGetDataStatus = 1;//AT CMD Send Faile
		}
	}
}	
/******************************************************************************
  ** \brief  异常处理函数
  ** 
  ** @param  NULL
  ** 
  ** \retval NULL
******************************************************************************/
void Bc35UserExceptionHandle(void){
	if((gBc35UserSendDataRepetition != 0 || gBc35UserGetDataError != 0)){
		FIOT_LAB_LOG(LOG_DEBUG,"-------------ExceptionHandle:%d %d-------------\r\n",gBc35UserSendDataRepetition,gBc35UserGetDataError);								
		if(gBc35UserSendDataRepetition != 0 && gBc35UserSendDataRepetition <= 3){
			u8DeviceAlarm = 1;
			FIOT_LAB_LOG(LOG_DEBUG,"Repetition Send Data OC\r\n");
		}else if(gBc35UserGetDataError != 0 && gBc35UserGetDataError <= 3){
			u8DeviceAlarm = 1;
			FIOT_LAB_LOG(LOG_DEBUG,"Repetition Send AT CMD \r\n");				
		}else{
			//发送N次不成功,进入补包
			if(gBc35UserSendDataRepetition >= 3){
				gBc35UserSendDataRepetition = 0;
				FIOT_LAB_LOG(LOG_DEBUG,"Repetition Send Data OC n time\r\n");
			}
			if(gBc35UserGetDataError >= 3){
				gBc35UserGetDataError = 0;
				FIOT_LAB_LOG(LOG_DEBUG,"Repetition Send AT CMD n time\r\n");												
			}
			gBc35UserLeakPackage++;//补包开始
		}
	}		
}
/******************************************************************************
  ** \brief  OC平台接收命令函数
  ** 
  ** @param  null
  ** 
  ** \retval Ok or Faile
******************************************************************************/
void Bc35UartUserHandle(void){
	if(gBc35UserGetDataStatus != 2 && gBc35UserSendDataStatus != 2)
		Bc35UserUartMsgSend();
	//send data OC,handle OC's cmd.
	if(gBc35UserWaitResponse == 1){	
		#ifdef HC32_LOG_SUPPORTS				
		//FIOT_LAB_LOG(LOG_FATAL,"Wait Record Time : %d %d %d!\r",gLptimSecRecord,gBc35UserReceiveCmdFromOc,u8DeviceAlarm);	
		#endif
		//wait 20s time
		if((gLptimSecRecord >= 10 && (gBc35UserSendDataStatus != 0 || gBc35UserGetDataStatus != 0))){
			if(gBc35UserReceiveCmdFromOc == 0){	//AT Send Time Out
				LpTimer_Disable();				//Stop Lptimer
				gBc35UserWaitResponse = 0;		//clear wait 20s flag	
				#ifdef HC32_LOG_SUPPORTS
					FIOT_LAB_LOG(LOG_FATAL,"AT Send End\r");
				#endif	
				if(gBc35UserGetDataStatus){	
					//gBc35UserStatusIndex = 0;//重新发送
					u8DeviceAlarm = 0;
					gBc35UserGetDataError ++;//发送AT命令未成功,进入补发操作
					gBc35UserGetDataStatus = 2;//time out Send AT
					#ifdef HC32_LOG_SUPPORTS
					FIOT_LAB_LOG(LOG_FATAL,"AT Send Faile :%d %d\r",gBc35UserGetDataError,gBc35UserGetDataStatus);
					#endif					
				}					
			}else if(gBc35UserReceiveCmdFromOc < 1){
				gBc35UserReceiveCmdFromOc++;
			}else if(gBc35UserReceiveCmdFromOc >= 1){	
				gBc35UserReceiveCmdFromOc = 0;
				LpTimer_Disable();				
				gBc35UserWaitResponse = 0;		//clear wait 20s flag
				if(gBc35UserSendDataStatus){
					u8DeviceAlarm = 0;//Cancel this delivery
					gBc35UserSendDataStatus = 2;//time out Send AT
					u8DeviceDeepSleep = 1;//enter sleep
				}				
				#ifdef HC32_LOG_SUPPORTS
					FIOT_LAB_LOG(LOG_FATAL,"SendData End\r");
				#endif				
			}
			gLptimSecRecord = 0;				//clear time count	
		}
	}	
}
/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/












