/******************************************************************************
 ** \file hc32_wdt.c
 **
 **  Copyright Fuzhou Internet of Things Open Lab.
 **
 **	 Author :Fiot-lab
 **
 **  DESCRIPTION: This file contains WDT Configure.
 **
 ******************************************************************************/
 
/******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_wdt.h"
#include "hc32_debug.h"
#include "pca.h"
/********************************************************************************
  ** \brief  WDT配置
  ** 
  ** @param  (使能源,喂狗周期)
  ** 
  ** \retval null
******************************************************************************/ 
void HC32_WDT_Init(en_wdt_func_t enFunc, en_wdt_time_t enTime)
{
    ///开启WDT外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralWdt,TRUE);
    ///WDT 初始化
    Wdt_Init(enFunc, enTime);
	  ///< 启动WDT 
    Wdt_Start();
	
#ifdef HC32_LOG_SUPPORTS 
	FIOT_LAB_LOG(LOG_INFO,"WDT:%d\r\n",enTime);
#endif	


}
/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/























