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
  ** \brief  KEY���ú���
  ** 
  ** @param  null
  ** 
  ** \retval null
******************************************************************************/ 
void HC32_KEY_Init(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    ///< ʱ��ʹ��
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    
    ///< ����
    stcGpioCfg.enDir = GpioDirIn;
    ///< ����������
    stcGpioCfg.enDrv = GpioDrvL;
    ///< ��������
    stcGpioCfg.enPu = GpioPuEnable;
    stcGpioCfg.enPd = GpioPdDisable;
    ///< ��©����ر�
    stcGpioCfg.enOD = GpioOdDisable;
    ///< �������������������->AHB
    stcGpioCfg.enCtrlMode = GpioAHB;
    ///< GPIO��ʼ��
    Gpio_Init(KEY0_USER_PORT, KEY0_USER_PIN, &stcGpioCfg);	
    ///< Set USER KEY irqFalling
    Gpio_EnableIrq(KEY0_USER_PORT, KEY0_USER_PIN, GpioIrqFalling);
    ///< ʹ��PORTC�ж�
    EnableNvic(PORTC_E_IRQn, IrqLevel1, TRUE);
		
#ifdef HC32_LOG_SUPPORTS 
	FIOT_LAB_LOG(LOG_INFO,"KEY GpioIrqFalling\r\n");
#endif			
}
/********************************************************************************
  ** \brief  KEY_PORTA�жϷ�����
  ** 
  ** @param  null
  **  
  ** \retval null
******************************************************************************/ 
void PortC_IRQHandler(void)
{
    if(TRUE == Gpio_GetIrqStatus(KEY0_USER_PORT, KEY0_USER_PIN))//�ж�IO
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
        Gpio_ClearIrq(KEY0_USER_PORT, KEY0_USER_PIN);  //�����־λ		
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

















