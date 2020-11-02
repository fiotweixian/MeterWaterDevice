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
  ** \brief  WDT����
  ** 
  ** @param  (ʹ��Դ,ι������)
  ** 
  ** \retval null
******************************************************************************/ 
void HC32_WDT_Init(en_wdt_func_t enFunc, en_wdt_time_t enTime)
{
    ///����WDT����ʱ��
    Sysctrl_SetPeripheralGate(SysctrlPeripheralWdt,TRUE);
    ///WDT ��ʼ��
    Wdt_Init(enFunc, enTime);
	  ///< ����WDT 
    Wdt_Start();
	
#ifdef HC32_LOG_SUPPORTS 
	FIOT_LAB_LOG(LOG_INFO,"WDT:%d\r\n",enTime);
#endif	


}
/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/























