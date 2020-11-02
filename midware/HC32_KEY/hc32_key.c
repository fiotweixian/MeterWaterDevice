/******************************************************************************
 ** \file hc32_key.c
 **
 **  Copyright Fuzhou Internet of Things Open Lab.
 **
 **	 Author :Fiot-lab
 **
 **  DESCRIPTION: This file contains KEY Configure.
 **
 ******************************************************************************/
 /******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_key.h"
#include "hc32_led.h"
#include "hc32_lcd.h"
#include "hc32_bc35.h"

/*******************************************************************************
 * global variable - (global)  and static variable - (static) 
 ******************************************************************************/
uint8_t u8DeviceWarning = 0;

/********************************************************************************
  ** \brief  KEY配置函数
  ** 
  ** @param  null
  ** 
  ** \retval null
******************************************************************************/ 
void HC32_KEY_Init(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    ///< 时钟使能
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    
    ///< 输入
    stcGpioCfg.enDir = GpioDirIn;
    ///< 高驱动能力
    stcGpioCfg.enDrv = GpioDrvL;
    ///< 无上下拉
    stcGpioCfg.enPu = GpioPuEnable;
    stcGpioCfg.enPd = GpioPdDisable;
    ///< 开漏输出关闭
    stcGpioCfg.enOD = GpioOdDisable;
    ///< 输入输出控制总线配置->AHB
    stcGpioCfg.enCtrlMode = GpioAHB;
    ///< GPIO初始化
    Gpio_Init(KEY0_USER_PORT, KEY0_USER_PIN, &stcGpioCfg);	
    ///< Set USER KEY irqFalling
    Gpio_EnableIrq(KEY0_USER_PORT, KEY0_USER_PIN, GpioIrqFalling);
    ///< 使能PORTC中断
    EnableNvic(PORTC_E_IRQn, IrqLevel1, TRUE);
		
#ifdef HC32_LOG_SUPPORTS 
	FIOT_LAB_LOG(LOG_INFO,"KEY GpioIrqFalling\r\n");
#endif			
}
/********************************************************************************
  ** \brief  KEY_PORTA中断服务函数
  ** 
  ** @param  null
  **  
  ** \retval null
******************************************************************************/ 
void PortC_IRQHandler(void)
{
    if(TRUE == Gpio_GetIrqStatus(KEY0_USER_PORT, KEY0_USER_PIN))//判断IO
    {       			
		u8DeviceAlarm = 1;
		gBc35UserGetDataStatus = 0;
		gBc35UserGetDataError = 0;
		#ifdef HC32_LOG_SUPPORTS						
			HC32_DEBUG_Init();
		#endif			
		#ifdef HC32_BC35_SUPPORTS					
			HC32_BC35_Init();
		#endif		
		u8Nb35SleepStatus = 0;
		FIOT_LAB_LOG(LOG_INFO,"KEY Event Happend!\r\n");
        Gpio_ClearIrq(KEY0_USER_PORT, KEY0_USER_PIN);  //清除标志位		
    }
}
/********************************************************************************
  ** \brief  Read Key Level
  ** 
  ** @param  null
  **  
  ** \retval null
******************************************************************************/ 
boolean_t HC32_Read_Key(void){
	return Gpio_GetInputIO(KEY0_USER_PORT,KEY0_USER_PIN);
}
/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/

















