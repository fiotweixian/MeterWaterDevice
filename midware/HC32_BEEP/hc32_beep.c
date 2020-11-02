/******************************************************************************
 ** \file hc32_beep.c
 **
 **  Copyright Fuzhou Internet of Things Open Lab.
 **
 **	 Author :Fiot-lab
 **
 **  DESCRIPTION: This file contains BEEP Configure and Beep Control.
 **
 ******************************************************************************/
/******************************************************************************
* Include files
******************************************************************************/
#include "hc32_beep.h"

/*******************************************************************************
 * global variable - (global)  and static variable - (static) 
 ******************************************************************************/
 
 
/********************************************************************************
  ** \brief  BEEP GPIO 配置函数
  ** 
  ** @param  NULL
  ** 
  ** \retval NULL
******************************************************************************/ 
void HC32_BEEP_Init(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    ///< 打开GPIO外设时钟门控
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); 
    
    ///< 端口方向配置
    stcGpioCfg.enDir = GpioDirOut;
    ///< 配置上下拉
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
    
    ///< BEEP关闭
    Gpio_ClrIO(USER_BEEP_PORT, USER_BEEP_PIN);
  
    ///< GPIO IO BEEP端口初始化
    Gpio_Init(USER_BEEP_PORT, USER_BEEP_PIN, &stcGpioCfg);
	
	///< 输出低电平,关闭蜂鸣器
	HC32_BEEP_Low();	
}
/********************************************************************************
  ** \brief  BEEP GPIO 清除配置
  ** 
  ** @param  NULL
  ** 
  ** \retval NULL
******************************************************************************/ 
void HC32_BEEP_Deinit(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    ///< 打开GPIO外设时钟门控
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); 
    
    ///< 端口方向配置
    stcGpioCfg.enDir = GpioDirIn;
    ///< 配置上下拉
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
  
    ///< GPIO IO BEEP端口初始化
    Gpio_Init(USER_BEEP_PORT, USER_BEEP_PIN, &stcGpioCfg);
}
/********************************************************************************
  ** \brief  BEEP_Switch状态切换
  ** 
  ** @param  高电平:1/低电平:0
  ** 
  ** \retval NULL
******************************************************************************/ 
void BEEP_Switch(uint8_t switch_t)
{
	if(switch_t){
     Gpio_SetIO(USER_BEEP_PORT, USER_BEEP_PIN);		
	}
	else{
     Gpio_ClrIO(USER_BEEP_PORT, USER_BEEP_PIN);	
	}
}
/********************************************************************************
  ** \brief  BEEP 状态翻转
  ** 
  ** @param  状态翻转时间
  ** 
  ** \retval NULL
******************************************************************************/ 
void BEEP_OverTurn(int32_t delay_t)
{
	Gpio_SetIO(USER_BEEP_PORT, USER_BEEP_PIN);	
	delay1ms(delay_t);
	Gpio_ClrIO(USER_BEEP_PORT, USER_BEEP_PIN);	
	delay1ms(delay_t);
}
/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/









