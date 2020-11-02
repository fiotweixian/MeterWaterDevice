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
  ** \brief  BEEP GPIO ���ú���
  ** 
  ** @param  NULL
  ** 
  ** \retval NULL
******************************************************************************/ 
void HC32_BEEP_Init(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    ///< ��GPIO����ʱ���ſ�
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); 
    
    ///< �˿ڷ�������
    stcGpioCfg.enDir = GpioDirOut;
    ///< ����������
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
    
    ///< BEEP�ر�
    Gpio_ClrIO(USER_BEEP_PORT, USER_BEEP_PIN);
  
    ///< GPIO IO BEEP�˿ڳ�ʼ��
    Gpio_Init(USER_BEEP_PORT, USER_BEEP_PIN, &stcGpioCfg);
	
	///< ����͵�ƽ,�رշ�����
	HC32_BEEP_Low();	
}
/********************************************************************************
  ** \brief  BEEP GPIO �������
  ** 
  ** @param  NULL
  ** 
  ** \retval NULL
******************************************************************************/ 
void HC32_BEEP_Deinit(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    ///< ��GPIO����ʱ���ſ�
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); 
    
    ///< �˿ڷ�������
    stcGpioCfg.enDir = GpioDirIn;
    ///< ����������
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
  
    ///< GPIO IO BEEP�˿ڳ�ʼ��
    Gpio_Init(USER_BEEP_PORT, USER_BEEP_PIN, &stcGpioCfg);
}
/********************************************************************************
  ** \brief  BEEP_Switch״̬�л�
  ** 
  ** @param  �ߵ�ƽ:1/�͵�ƽ:0
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
  ** \brief  BEEP ״̬��ת
  ** 
  ** @param  ״̬��תʱ��
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









