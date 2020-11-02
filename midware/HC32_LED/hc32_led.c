/******************************************************************************
 ** \file hc32_led.c
 **
 **  Copyright Fuzhou Internet of Things Open Lab.
 **
 **	 Author :Fiot-lab
 **
 **  DESCRIPTION: This file contains LED Configure and Control.
 **
 ******************************************************************************/
 
/******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_led.h"

/********************************************************************************
  ** \brief  LED Configure Init
  ** 
  ** @param  Null
  ** 
  ** \retval Null
******************************************************************************/ 
void HC32_LED_Init(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    ///< 打开GPIO外设时钟门控
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); 
    
    ///< 端口方向配置
    stcGpioCfg.enDir = GpioDirOut;
    ///< 配置上下拉
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdEnable;
    
    ///< LED关闭
    Gpio_ClrIO(USER_LED_PORT, USER_LED_PIN);
    
    ///< GPIO IO LED端口初始化
    Gpio_Init(USER_LED_PORT, USER_LED_PIN, &stcGpioCfg);

}
/********************************************************************************
  ** \brief  LED_Switch状态切换
  ** 
	** @param  0:Off 1:On
  ** 
  ** \retval null
******************************************************************************/ 
void LED_Switch(uint8_t switch_t)
{
	if(switch_t){
     Gpio_SetIO(USER_LED_PORT, USER_LED_PIN);		
	}
	else{
     Gpio_ClrIO(USER_LED_PORT, USER_LED_PIN);	
	}
}
/********************************************************************************
  ** \brief  LED_OverTurn状态翻转
  ** 
  ** @param  OverTurn Time(ms)
  ** 
  ** \retval null
******************************************************************************/ 
void LED_OverTurn(int32_t delay_t)
{
   Gpio_SetIO(USER_LED_PORT, USER_LED_PIN);	
	 delay1ms(delay_t);
   Gpio_ClrIO(USER_LED_PORT, USER_LED_PIN);	
	 delay1ms(delay_t);
}
/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


