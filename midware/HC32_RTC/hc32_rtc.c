/******************************************************************************
 ** \file hc32_rtc.c
 **
 **  Copyright Fuzhou Internet of Things Open Lab.
 **
 **	 Author :Fiot-lab
 **
 **  DESCRIPTION: This file contains RTC Configure.
 **
 ******************************************************************************/
 
/******************************************************************************
* Include files
******************************************************************************/
#include "hc32_rtc.h"
#include "hc32_bc35.h"
#include "hc32_wdt.h"
#include "hc32_debug.h"
#include "common.h"
#include "hc32_eeprom.h"
/*******************************************************************************
 * global variable - (global)  and static variable - (static) 
 ******************************************************************************/
uint8_t u8RecordPcntHalfHour = 0; //Use to Record Pcnt Times,Every half an hour.

uint8_t u8DeviceAlarm = 0;//Use to Device Alarm Wake Up,Once a day.

uint8_t u8DeviceWdtdog = 0;//Wdt Feed Operate,Every 30 seconds

boolean_t u8DeviceDeepSleep = 1;//Device DeepSleep States

boolean_t u8DevicePriod = 0;

RTC_Alarm_T gRtcUserAlarm = {0};//Used to store random escalation times,which reporting time every day.

/********************************************************************************
  ** \brief  RTC Configure Init
  ** 
  ** @param  Null
  ** 
  ** \retval Null
******************************************************************************/ 
void HC32_RTC_Init(void)
{

    stc_rtc_initstruct_t RtcInitStruct;
    stc_rtc_alarmtime_t RtcAlmStruct;
    
    DDL_ZERO_STRUCT(RtcInitStruct);                      //变量初始化为0
    DDL_ZERO_STRUCT(RtcAlmStruct);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralRtc,TRUE);//RTC时钟模块打开   
    
    Sysctrl_ClkSourceEnable(SysctrlClkRCL, TRUE);        //使能外部XTL时钟作为RTC时钟
		
	RtcInitStruct.rtcAmpm = RtcPm;                       //24小时制
	RtcInitStruct.rtcClksrc = RtcClkRcl;                 //外部低俗时钟
	RtcInitStruct.rtcPrdsel.rtcPrdsel = RtcPrdx;         //周期中断类型PRDS
	RtcInitStruct.rtcPrdsel.rtcPrdx = 59;				//30S中断一次(BCD码)		
	RtcInitStruct.rtcTime.u8Second = 0x55;               //配置RTC时间2019年4月17日10:01:55
	RtcInitStruct.rtcTime.u8Minute = 0x01;
	RtcInitStruct.rtcTime.u8Hour   = 0x10;
	RtcInitStruct.rtcTime.u8Day    = 0x17;
	RtcInitStruct.rtcTime.u8DayOfWeek = 0x04;
	RtcInitStruct.rtcTime.u8Month  = 0x04;
	RtcInitStruct.rtcTime.u8Year   = 0x19;
	RtcInitStruct.rtcCompen = RtcCompenEnable;           //使能时钟补偿误差
	RtcInitStruct.rtcCompValue = 5;                      //补偿值,根据实际补偿(未设置)
	Rtc_Init(&RtcInitStruct);    

	//获取凌晨0-6点随机上报时间
	//Sec 根据地址号设置最小估值(10s)
	gRtcUserAlarm.Sec = DEC2BCD((gBc28UserWaterDevNum[3]%10/2) * 12);
	gRtcUserAlarm.Min = DEC2BCD(Random(59));
	gRtcUserAlarm.Hour = DEC2BCD(Random(8));
	
	RtcAlmStruct.RtcAlarmSec = gRtcUserAlarm.Sec;
	RtcAlmStruct.RtcAlarmMinute = gRtcUserAlarm.Min;
	RtcAlmStruct.RtcAlarmHour = gRtcUserAlarm.Hour;
	RtcAlmStruct.RtcAlarmWeek = 0x7f;                    //周一到周日,每天10:02:05启动一次闹钟    
	Rtc_SetAlarmTime(&RtcAlmStruct);                     //配置闹钟时间  
	Rtc_AlmIeCmd(TRUE);                                  //使能闹钟中断
		
	EnableNvic(RTC_IRQn, IrqLevel3, TRUE);               //使能RTC中断向量
	Rtc_Cmd(TRUE);                                       //使能RTC开始计数

	#ifdef HC32_LOG_INFO
	FIOT_LAB_LOG(LOG_INFO,"AlarmRandomTime:%02d:%02d:%02d\r",BCD2DEC(RtcAlmStruct.RtcAlarmHour),
																											BCD2DEC(RtcAlmStruct.RtcAlarmMinute),
																											BCD2DEC(RtcAlmStruct.RtcAlarmSec));		
	#endif
}
/********************************************************************************
    ** \brief  RTC中断服务函数
    ** 
		** @param  null
    ** 
    ** \retval null
******************************************************************************/  
void Rtc_IRQHandler(void)
{
    if (Rtc_GetAlmfItStatus() == TRUE) //闹钟中断
    {
        Rtc_ClearAlmfItStatus();       //清除标志位
		//everyday send message
		//FIOT_LAB_LOG(LOG_FATAL,"Alarm Happend!\r\n");
		u8DeviceAlarm = 1;
		WaterRecordDay++;//记录天数
		WaterRecordindex = 0;//索引清零
		gBc35UserGetDataStatus = 0; 
		u8Nb35SleepStatus = 0;
    }
	else if(Rtc_GetPridItStatus() == TRUE)
    {
		//RTC 32s Logical Handler
		Wdt_Feed();											// 喂狗
        Rtc_ClearPrdfItStatus();       //清除标志位
		u8DeviceWdtdog = 1;
		u8RecordPcntHalfHour++;	
		//FIOT_LAB_LOG(LOG_FATAL,"Rtc Happend = %d!\r",u8RecordPcntHalfHour);
	}
	if(gBc28period.PriodStartFlag == 1){
		stc_rtc_time_t readtime;
		Rtc_ReadDateTime(&readtime);
		if((readtime.u8Year == gBc28period.Year) && (readtime.u8Month == gBc28period.Month) && (readtime.u8Day == gBc28period.Day) && \
											(readtime.u8Hour == gBc28period.Hour) && (readtime.u8Minute == gBc28period.Min)){
			FIOT_LAB_LOG(LOG_FATAL,"-----------Period Send------------\r");
			u8DevicePriod = 1;
		}
	}
}
/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
























