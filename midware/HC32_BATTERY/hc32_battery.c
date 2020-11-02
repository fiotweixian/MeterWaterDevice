/******************************************************************************
 ** \file hc32_battery.c
 **
 **  Copyright Fuzhou Internet of Things Open Lab.
 **
 **	 Author :Fiot-lab
 **
 **  DESCRIPTION: This file contains ADC Configure and Battery Value.
 **
 ******************************************************************************/
 
 /******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_battery.h"

/*******************************************************************************
 * global variable - (global)  and static variable - (static) 
 ******************************************************************************/
uint32_t u32AdcRestult = 0;//ADC����ֵ

/*******************************************************************************
��ѹ�ȼ���Ϊ4��:0 1 2 3.�ȼ���,��ѹ��.
******************************************************************************/
char gBc35UserBatteryValueLevel = 0;//��ѹֵ�ȼ�

/********************************************************************************
  ** \brief  BATTERY GPIO ��ʼ��
  ** 
  ** @param  NULL
  ** 
  ** \retval NULL
******************************************************************************/ 
void HC32_BATTERY_GPIO(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    ///< ��GPIO����ʱ���ſ�
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); 
    
    ///< �˿ڷ�������
    stcGpioCfg.enDir = GpioDirOut;
    ///< ����������ʧ��
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
    
    ///< Battery IO �ر�
    Gpio_ClrIO(USER_BATTERY_PORT, USER_BATTERY_PIN);
    
    ///< GPIO IO Battery�˿ڳ�ʼ��
    Gpio_Init(USER_BATTERY_PORT, USER_BATTERY_PIN, &stcGpioCfg);	
}
/********************************************************************************
  ** \brief  BATTERY ADC ��ʼ��
  ** 
  ** @param  NULL
  ** 
  ** \retval NULL
******************************************************************************/ 
void HC32_BATTERY_Init(void)
{
	float BatteryValue = 0;
	
    stc_adc_cfg_t   stcAdcCfg;
    DDL_ZERO_STRUCT(stcAdcCfg);
    ///< ���� GPIOʱ��
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    ///< ����ADC/BGR ʱ��
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE); 
    Bgr_BgrEnable();        ///<����BGR	
	Gpio_SetAnalogMode(GpioPortC, GpioPin1);        		//PC01 (AIN11)
    ///< ADC ����
    stcAdcCfg.enAdcMode         = AdcSglMode;               ///<���β���ģʽ
    stcAdcCfg.enAdcClkDiv       = AdcMskClkDiv8;            ///<������Ƶ-8
    stcAdcCfg.enAdcSampCycleSel = AdcMskSampCycle12Clk;     ///<����������-12
    stcAdcCfg.enAdcRefVolSel    = AdcMskRefVolSelInBgr2p5;  ///<�ο���ѹѡ��-�ڲ�2.5V
    stcAdcCfg.enAdcOpBuf        = AdcMskBufEnable;         	///<OP BUF����-ON
    stcAdcCfg.enInRef           = AdcMskInRefEnable;        ///<�ڲ��ο���ѹʹ��-ON
    stcAdcCfg.enAdcAlign        = AdcAlignRight;            ///<ת��������뷽ʽ-��
    Adc_Init(&stcAdcCfg);	
    ///< ADC ����ͨ������
    Adc_CfgSglChannel(AdcExInputCH11);
   
	///<��ʼ�ɼ�
    Adc_SGL_Always_Start(); 		
	
	///��ȡ��ѹֵ
	BatteryValue = BatteryObtain();
	//�жϵ�ѹ�ȼ�
	if(BatteryValue <= 3.65 && BatteryValue >= 3.3){
		gBc35UserBatteryValueLevel = 3;
	}else if(BatteryValue >= 3.15 && BatteryValue < 3.3){
		gBc35UserBatteryValueLevel = 2;
	}else if(BatteryValue < 3.15 && BatteryValue > 2.8){
		gBc35UserBatteryValueLevel = 1;
	}else{
		gBc35UserBatteryValueLevel = 0;
	}	
	#ifdef HC32_LOG_SUPPORTS 
	FIOT_LAB_LOG(LOG_INFO,"HC32 BATTERY ADC(PC01) Configure Succeed! BATTERY Vol:%f\r\n",BatteryValue);
	#endif			
}
/********************************************************************************
  ** \brief  BATTERY ADC �������
  ** 
  ** @param  NULL
  ** 
  ** \retval NULL
******************************************************************************/ 
void HC32_BATTERY_DeInit(void)
{	
    Bgr_BgrDisable();//�ر�bgr ���뿪��ʱ��
	
	delay10us(2);//�ȴ��ر�

	Adc_DeInit();//ADC �������

	delay10us(2);//�ȴ�����
	
	Adc_ClrIrqStatus(AdcMskIrqSgl);///< ����жϱ�־λ
	
	Adc_SGL_Always_Stop();///ֹͣADC�ɼ�	
}
/********************************************************************************
  ** \brief  �����ɼ�
  ** 
  ** @param  null
  ** 
  ** \retval null
******************************************************************************/ 
void HC32_BATTERY_Enable(void)
{
    ///< �������βɼ�
    Adc_SGL_Always_Start();   	
}
/********************************************************************************
  ** \brief  �رղɼ�
  ** 
  ** @param  null
  ** 
  ** \retval null
******************************************************************************/ 
void HC32_BATTERY_Disable(void)
{
    ///< ֹͣ���βɼ�
    Adc_SGL_Always_Stop();   	
}
/********************************************************************************
  ** \brief  �õ���ѹֵ
  ** 
  ** @param  null
  ** 
  ** \retval ���ص�ѹֵ
******************************************************************************/ 
float BatteryObtain(void)
{
	uint32_t u32BatteryRestult = 0;
	char i = 0,j = 0;
	HC32_BATTERY_Hight();	
	do{		
		if(TRUE == Adc_GetIrqStatus(AdcMskIrqSgl)){
			if(i >= 5){//�˲���ǰ�漸�β��ȶ�����
				u32BatteryRestult += Adc_GetSglResult();
				Adc_ClrIrqStatus(AdcMskIrqSgl);
				j++;
			}
		}
		delay1ms(5);
		i++;
		if(i > BATTERY_AVERAGE_TIME + 5)
			break;
	}while(1);
	u32BatteryRestult /= j;
	HC32_BATTERY_Low();	
	return (float)u32BatteryRestult / 4096 * 2.5 * 2;		
}

/********************************************************************************
  ** \brief  �õ���ѹ�ȼ�
  ** 
  ** @param  null
  ** 
  ** \retval ���ص�ѹ�ȼ�
******************************************************************************/ 
uint8_t BatteryLevel(void)
{
	float Vol =  BatteryObtain();
	if(Vol <= 3.65 && Vol >= 3.3)
		return 3;
	else if(Vol >= 3.15 && Vol < 3.3)
		return 2;
	else if(Vol < 3.15 && Vol > 2.9)
		return 1;		
	else
		return 0;
}
/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
















