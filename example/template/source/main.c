/******************************************************************************/
/** \file main.c
 **
 ** A detailed description is available at
 ** @link Sample Group Some description @endlink
 **
 **   - 2020-07-10  1.0  Lux First version for Device Driver Library of Module.
 **
 **	Author ：fiot-lab
 **
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "ddl_device.h"
#include "system.h"
#include "Common.h"
#include "hc32_led.h"
#include "hc32_bc35.h"
#include "hc32_debug.h"
#include "hc32_rtc.h"
#include "hc32_wdt.h"
#include "hc32_key.h"
#include "hc32_battery.h"
#include "hc32_valve.h"
#include "hc32_lptimer.h"
#include "hc32_pcnt.h"
#include "hc32_lcd.h"
#include "hc32_se.h"
#include "hc32_eeprom.h"
#include "hc32_beep.h"
#include "hc32_flash.h"
#include "hc32_lmode.h"
/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
/******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/


/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

void HC32_HardWare(void){

#ifdef HC32_SYSTEM32M_SUPPORTS
	//32MHz Clock Init
	SystemInit();	
	#endif
#ifdef HC32_LED_SUPPORTS	
	//LED Init
	HC32_LED_Init();
#endif	
#ifdef HC32_LOG_SUPPORTS  
    ///UART Init
    HC32_DEBUG_Init();
	FIOT_LAB_LOG(LOG_INFO,"Sysctrl_GetHClkFreq: %d\r\n",Sysctrl_GetHClkFreq());
#endif	
#ifdef HC32_EEPROM_SUPPORTS 
	#ifdef HC32_IIC_HARDWARE_SUPPORTS
		//EEPROM Init(hardware)
		HC32_EEPROM_Init(0x55,25);
	#else
		//EEPROM Init(software)
		AT24CXX_Init();
		while(AT24CXX_Check())
			FIOT_LAB_LOG(LOG_ERROR,"HC32 EEPROM Init Error\r");
		#ifdef HC32_LOG_SUPPORTS 
		FIOT_LAB_LOG(LOG_INFO,"EEPROM Init Succeed!\r\n");
		#endif				
	#endif
#endif		
#ifdef HC32_BC35_SUPPORTS
	//GPIO Init
	 HC32_BC35_GPIO();
	//NBIOT Init
	HC32_BC35_Init();
#endif  
#ifdef HC32_RTC_SUPPORTS	
	//RTC Init
	HC32_RTC_Init();
#endif
#ifdef HC32_WDT_SUPPORTS	
	//WDT Init <52.4s
	HC32_WDT_Init(WdtResetEn,WdtT52s4);
#endif	
#ifdef HC32_KEY_SUPPORTS	
	//KEY Init
	HC32_KEY_Init();
#endif
#ifdef HC32_BATTERY_SUPPORTS	
	//Battery Init
	HC32_BATTERY_GPIO();
	HC32_BATTERY_Init();
#endif
#ifdef HC32_VALVE_SUPPORTS	
	//nSleep GPIO Init
	HC32_VALVE_GPIO();
	//Valve Init (Period:0x9000, CHA-Duty:0x6000，CHB-Duty:0x3000)
	HC32_VALVE_Init(0x9000, 0x3000, 0x3000);
	HC32_Valve_Open();
#endif
#ifdef HC32_LPTIMER_SUPPORTS	
	//Lptimer Init
	HC32_LPTIMER_Init();
	//Stop Lptimer
	LpTimer_Disable();
#endif	
#ifdef HC32_LCD_SUPPORTS
	//Lcd Init
	HC32_LCD_Init();
	for(uint8_t i =0; i<=3; i++){
		Lcd_FullDisp();		
		delay1ms(500);
		Lcd_ClearDisp();
		delay1ms(500);
	}
#endif
#ifdef HC32_SE_SUPPORTS
	//SE Init(not finish)	
	HC32_SE_Init();		
#endif		
		
#ifdef HC32_BEEP_SUPPORTS
	//Beep Init
	HC32_BEEP_Init();
#endif
#ifdef HC32_PCNT_SUPPORTS	
	//Pcnt Init
	HC32_PCNT_Init();
	//Start Pcnt
	ReedPipe_Enable();
#endif				
#ifdef HC32_FLASH_SUPPORTS 
	//Flash Init 
	while(Ok != HC32_FLASH_Init())
		FIOT_LAB_LOG(LOG_ERROR,"HC32 Flash Init Error\r");
	#ifdef HC32_LOG_SUPPORTS 
		FIOT_LAB_LOG(LOG_INFO,"Flash Init Succeed!\r\n");
	#endif
#endif
}

/*******************************************************************************
 ** \brief  Main function of project
 **
 ** \return uint32_t return value, if needed
 **
 ** This Main
 ** 
 ******************************************************************************/
int32_t main(void)
{ 
	uint8_t ret = 0;
	uint8_t priodCount = 0;
	uint32_t u32min = 0;
	HC32_HardWare();
    while (1)
	{	
		#ifdef HC32_PCNT_SUPPORTS
		WaterFlowFloat();
		#endif
		#ifdef HC32_LCD_SUPPORTS
		HC32_LCD_ShowNum(WaterReturnInt(), WaterReturnFloat(), LCDImageFlow, 3);	
		#endif			
		//send data every day
		if(u8DeviceAlarm == 1){
			gBc35UserGetDataStatus = 0;
			gBc35UserSendDataStatus = 0;
			gBc35UserQlwConnectStatus = 1;
		//send Waring data
		}else if(u8DeviceWarning == 1){
			gBc35UserGetDataStatus = 0;
			gBc35UserSendDataStatus = 0;
			gBc35UserQlwConnectStatus = 2;
		//send OverCurrent data
		}else if(u8DeviceOverCurrent == 1){
			gBc35UserGetDataStatus = 0;
			gBc35UserSendDataStatus = 0;			
			gBc35UserQlwConnectStatus = 3;			
		//Get Board information
		}else if((((u8DeviceWdtdog == 1) || (u8DeviceDeepSleep == 1) || (u8DevicePcnt == 1)) && gBc35UserWaitResponse == 0)){
			u8DevicePcnt = 0;
			u8DeviceWdtdog = 0;
			if(u8RecordPcntHalfHour >= 2){
				//Half Hour,record Pcnt
				//eeprom
				u32min++;
				FIOT_LAB_LOG(LOG_DEBUG,"u32min:%d\r\n",u32min);
				
				if(u32min%2 == 0)//发送函数异常处理
					Bc35UserExceptionHandle();		
				
				if((u8DevicePriod == 1) && (u32min % 5) == 0){//密集周期上报
					u8DeviceAlarm = 1;
					priodCount++;
					FIOT_LAB_LOG(LOG_DEBUG,"PriodStartFlag Send Data OC,:%d priodCount:%d\r\n",u8DevicePriod,priodCount); 
					if(priodCount >= 48){
						u8DevicePriod = 0;
						priodCount = 0;
					}
					FIOT_LAB_LOG(LOG_DEBUG,"-------------Dense Period Send Data-------------\r\n");
				}
				if((u32min % gBc35UserSendDataFrequency) == 0){//高频周期上报
					u8DeviceAlarm = 1;
					FIOT_LAB_LOG(LOG_DEBUG,"-------------Frequency Send Data:%d -----------\r\n",gBc35UserSendDataFrequency);
				}
				if((u32min % gBc35UserOverCurrentTime) == 0){//过流检测		
					ret = JudgeNowOverCurrent();
					FIOT_LAB_LOG(LOG_DEBUG,"-------------OverCurrent Check:%d-------------\r\n",gBc35UserOverCurrentTime);					
					if(ret == 1){
						//Record OverCurrent Water
						if((gBc35UserImmediatelyWarn & (1<<2)) != 0){
							#ifdef HC32_LOG_SUPPORTS
							FIOT_LAB_LOG(LOG_FATAL,"Immediately OverCurrent\r\n");
							#endif
							u8DeviceOverCurrent = 1;
						}else{
							#ifdef HC32_LOG_SUPPORTS
							FIOT_LAB_LOG(LOG_FATAL,"Normal OverCurrent\r\n");
							#endif
						}						
					}
					StoreLastWaterNum(WaterReturnInt(),WaterReturnFloat());	
				}
				if((u32min % gBc35UserConCurrentTime) == 0){//反流检测
					FIOT_LAB_LOG(LOG_DEBUG,"-------------ConCurrent Check:%d-------------\r\n",gBc35UserConCurrentTime);					
				
				
				}
				if((u32min % 30) == 0){//30分钟存一次水量
					#ifdef HC32_LOG_SUPPORTS
					FIOT_LAB_LOG(LOG_DEBUG,"----------------EEPROM STORE WATER-----------------\r\n");
					#endif				
					HC32_StoreWaterNum(WaterReturnInt(),WaterReturnFloat());					
				}
				u8RecordPcntHalfHour = 0;
				if(u32min >= 1440){
					u32min = 0;
				}
			}
			if(gBc35UserWaitResponse == 0 && u8DeviceOverCurrent == 0 && u8DeviceAlarm == 0){	
				//FIOT_LAB_LOG(LOG_INFO,"Enter DeepSleep\r\n");
//					#ifdef HC32_BC35_SUPPORTS
//						#ifdef HC32_L161_SUPPORTS
//						if(u8Nb35SleepStatus == 0){
//							Bc35UserEnterSleep(0);
//							u8Nb35SleepStatus = 1;
//						}
//						#endif
//					#endif	
//					#ifdef HC32_LOG_SUPPORTS					
//						HC32_DEBUG_Deinit();
//					#endif
//					#ifdef HC32_BATTERY_SUPPORTS						
//						HC32_BATTERY_DeInit();
//					#endif
//					#ifdef HC32_VALVE_SUPPORTS	
//						HC32_VALVE_Deinit();
//					#endif
//					#ifdef HC32_VALVE_SUPPORTS						
//						HC32_SE_Deinit();
//					#endif					
//					HC32_DeepSleepCfg();//Configure All IO (Input PD Digital) 
//					Lpm_GotoDeepSleep(FALSE);
////					#ifdef HC32_BC35_SUPPORTS					
////						HC32_BC35_Init();
////					#endif			
//					#ifdef HC32_LOG_SUPPORTS						
//						HC32_DEBUG_Init();
//					#endif	
////					#ifdef HC32_BC35_SUPPORTS					
////						Bc35UserEnterSleep(0);	
////					#endif	
////					#ifdef HC32_BATTERY_SUPPORTS						
////						HC32_BATTERY_Init();
////					#endif					
			}					
		}		
		if((u8DeviceAlarm == 1 || u8DeviceWarning == 1 || u8DeviceOverCurrent == 1 || gBc35UserWaitResponse == 1)){			
			Bc35UartUserHandle();	
		}
		#ifdef HC32_LOG_PARSER
		//Debug cmd Parser
		if(u8DeviceLogData == 1){
			u8DeviceLogData = 0;
			Bc35UserLogMsgParser();
		}	
		#endif
	}
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/































