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
    
    DDL_ZERO_STRUCT(RtcInitStruct);                      //������ʼ��Ϊ0
    DDL_ZERO_STRUCT(RtcAlmStruct);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralRtc,TRUE);//RTCʱ��ģ���   
    
    Sysctrl_ClkSourceEnable(SysctrlClkRCL, TRUE);        //ʹ���ⲿXTLʱ����ΪRTCʱ��
		
	RtcInitStruct.rtcAmpm = RtcPm;                       //24Сʱ��
	RtcInitStruct.rtcClksrc = RtcClkRcl;                 //�ⲿ����ʱ��
	RtcInitStruct.rtcPrdsel.rtcPrdsel = RtcPrdx;         //�����ж�����PRDS
	RtcInitStruct.rtcPrdsel.rtcPrdx = 59;				//30S�ж�һ��(BCD��)		
	RtcInitStruct.rtcTime.u8Second = 0x55;               //����RTCʱ��2019��4��17��10:01:55
	RtcInitStruct.rtcTime.u8Minute = 0x01;
	RtcInitStruct.rtcTime.u8Hour   = 0x10;
	RtcInitStruct.rtcTime.u8Day    = 0x17;
	RtcInitStruct.rtcTime.u8DayOfWeek = 0x04;
	RtcInitStruct.rtcTime.u8Month  = 0x04;
	RtcInitStruct.rtcTime.u8Year   = 0x19;
	RtcInitStruct.rtcCompen = RtcCompenEnable;           //ʹ��ʱ�Ӳ������
	RtcInitStruct.rtcCompValue = 5;                      //����ֵ,����ʵ�ʲ���(δ����)
	Rtc_Init(&RtcInitStruct);    

	//��ȡ�賿0-6������ϱ�ʱ��
	//Sec ���ݵ�ַ��������С��ֵ(10s)
	gRtcUserAlarm.Sec = DEC2BCD((gBc28UserWaterDevNum[3]%10/2) * 12);
	gRtcUserAlarm.Min = DEC2BCD(Random(59));
	gRtcUserAlarm.Hour = DEC2BCD(Random(8));
	
	RtcAlmStruct.RtcAlarmSec = gRtcUserAlarm.Sec;
	RtcAlmStruct.RtcAlarmMinute = gRtcUserAlarm.Min;
	RtcAlmStruct.RtcAlarmHour = gRtcUserAlarm.Hour;
	RtcAlmStruct.RtcAlarmWeek = 0x7f;                    //��һ������,ÿ��10:02:05����һ������    
	Rtc_SetAlarmTime(&RtcAlmStruct);                     //��������ʱ��  
	Rtc_AlmIeCmd(TRUE);                                  //ʹ�������ж�
		
	EnableNvic(RTC_IRQn, IrqLevel3, TRUE);               //ʹ��RTC�ж�����
	Rtc_Cmd(TRUE);                                       //ʹ��RTC��ʼ����

	#ifdef HC32_LOG_INFO
	FIOT_LAB_LOG(LOG_INFO,"AlarmRandomTime:%02d:%02d:%02d\r",BCD2DEC(RtcAlmStruct.RtcAlarmHour),
																											BCD2DEC(RtcAlmStruct.RtcAlarmMinute),
																											BCD2DEC(RtcAlmStruct.RtcAlarmSec));		
	#endif
}
/********************************************************************************
    ** \brief  RTC�жϷ�����
    ** 
		** @param  null
    ** 
    ** \retval null
******************************************************************************/  
void Rtc_IRQHandler(void)
{
    if (Rtc_GetAlmfItStatus() == TRUE) //�����ж�
    {
        Rtc_ClearAlmfItStatus();       //�����־λ
		//everyday send message
		//FIOT_LAB_LOG(LOG_FATAL,"Alarm Happend!\r\n");
		u8DeviceAlarm = 1;
		WaterRecordDay++;//��¼����
		WaterRecordindex = 0;//��������
		gBc35UserGetDataStatus = 0; 
		u8Nb35SleepStatus = 0;
    }
	else if(Rtc_GetPridItStatus() == TRUE)
    {
		//RTC 32s Logical Handler
		Wdt_Feed();											// ι��
        Rtc_ClearPrdfItStatus();       //�����־λ
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
























