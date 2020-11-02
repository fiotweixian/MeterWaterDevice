/******************************************************************************
 ** \file hc32_valve.c
 **
 **  Copyright Fuzhou Internet of Things Open Lab.
 **
 **	 Author :Fiot-lab
 **
 **  DESCRIPTION: This file contains PWM Configure and Valva Control .
 **
 ******************************************************************************/
/******************************************************************************
* Include files
******************************************************************************/
#include "hc32_valve.h"
/********************************************************************************
  ** \brief  Valve sleep GPIO
  ** 
  ** @param  uint16_t u16Period, uint16_t u16CHA_PWMDuty, uint16_t u16CHB_PWMDuty
  ** 
  ** \retval null
******************************************************************************/ 
void HC32_VALVE_GPIO(void){
	
    stc_gpio_cfg_t stcGpioCfg;
    
    ///< ��GPIO����ʱ���ſ�
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); 
    
    ///< �˿ڷ�������
    stcGpioCfg.enDir = GpioDirOut;
    ///< ����������
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdEnable;
    
    ///<  SLEEP�ر�
    Gpio_ClrIO(USER_SLEEP_PORT, USER_SLEEP_PIN);
    
    ///< GPIO IO SLEEP�˿ڳ�ʼ��
    Gpio_Init(USER_SLEEP_PORT, USER_SLEEP_PIN, &stcGpioCfg);

		HC32_SLEEP_Hight();
}
/********************************************************************************
  ** \brief  Valve��ʼ��
  ** 
  ** @param  uint16_t u16Period, uint16_t u16CHA_PWMDuty, uint16_t u16CHB_PWMDuty
  ** 
  ** \retval null
******************************************************************************/ 
void HC32_VALVE_Init(uint16_t u16Period, uint16_t u16CHxACompare, uint16_t u16CHxBCompare)
{
    uint16_t                     u16CntValue;
    uint8_t                      u8ValidPeriod;	
	
    stc_gpio_cfg_t               stcTIM3Port;
    stc_tim3_mode23_cfg_t        stcTim3BaseCfg;
    stc_tim3_m23_compare_cfg_t   stcTim3PortCmpCfg;
	
    DDL_ZERO_STRUCT(stcTIM3Port);
    DDL_ZERO_STRUCT(stcTim3BaseCfg);
    DDL_ZERO_STRUCT(stcTim3PortCmpCfg);
	
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); 	//ʱ��
    Sysctrl_SetPeripheralGate(SysctrlPeripheralTim3, TRUE);   //Timer3ʱ��
	
    stcTIM3Port.enDir  = GpioDirOut;
	
    Gpio_Init(GpioPortE, GpioPin9, &stcTIM3Port);
    Gpio_SetAfMode(GpioPortE,GpioPin9,GpioAf1);               //PE09����TIM3_CH0A
    
    Gpio_Init(GpioPortE, GpioPin10, &stcTIM3Port);
    Gpio_SetAfMode(GpioPortE,GpioPin10,GpioAf1);               //PE08����TIM3_CH0B
	
    stcTim3BaseCfg.enWorkMode    = Tim3WorkMode2;             //��ݲ�
    stcTim3BaseCfg.enCT          = Tim3Timer;                 //��ʱ������,�ڲ���PCLK
    stcTim3BaseCfg.enPRS         = Tim3PCLKDiv256;            //PCLK
    stcTim3BaseCfg.enCntDir      = Tim3CntUp;                 //���ϼ���
    stcTim3BaseCfg.enPWMTypeSel  = Tim3IndependentPWM;        //�������PWM
    stcTim3BaseCfg.enPWM2sSel    = Tim3SinglePointCmp;        //����ȽϹ���
    stcTim3BaseCfg.bOneShot      = FALSE;                     //ѭ������
    stcTim3BaseCfg.bURSSel       = FALSE;                     //�����������
    
    Tim3_Mode23_Init(&stcTim3BaseCfg);                        //TIM3 ģʽ23��ʼ��
    
    Tim3_M23_ARRSet(u16Period, TRUE);                       //������װֵ,��ʹ�ܻ���
		
    Tim3_M23_CCR_Set(Tim3CCR0A, u16CHxACompare);            //����CH0�Ƚ�ֵA
//    Tim3_M23_CCR_Set(Tim3CCR1B, u16CHxBCompare);            //����CH0�Ƚ�ֵB  

    stcTim3PortCmpCfg.enCHxACmpCtrl   = Tim3PWMMode2;         //OCREFA�������OCMA:PWMģʽ2
    stcTim3PortCmpCfg.enCHxAPolarity  = Tim3PortPositive;     //�������
    stcTim3PortCmpCfg.bCHxACmpBufEn   = TRUE;                 //A�������
    stcTim3PortCmpCfg.enCHxACmpIntSel = Tim3CmpIntNone;       //A�жϱȽϿ���:��
		
    Tim3_M23_PortOutput_Cfg(Tim3CH0, &stcTim3PortCmpCfg);     //�Ƚ�����˿�����
    
    Tim3_M23_CCR_Set(Tim3CCR1B, u16CHxBCompare);            //����CH0�Ƚ�ֵB 
	 
    stcTim3PortCmpCfg.enCHxBCmpCtrl   = Tim3PWMMode2;         //OCREFB�������OCMA:PWMģʽ2
    stcTim3PortCmpCfg.enCHxBPolarity  = Tim3PortOpposite;     //�������
    stcTim3PortCmpCfg.bCHxBCmpBufEn   = TRUE;                 //B�������
    stcTim3PortCmpCfg.enCHxBCmpIntSel = Tim3CmpIntNone;       //B�жϱȽϿ���:��
    
    Tim3_M23_PortOutput_Cfg(Tim3CH1, &stcTim3PortCmpCfg);     //�Ƚ�����˿����� 
    
    u8ValidPeriod = 0;                                        //ʱ�������������
    Tim3_M23_SetValidPeriod(u8ValidPeriod);                   //�����������
    
    u16CntValue = 0;
    
    Tim3_M23_Cnt16Set(u16CntValue);                           //���ü�����ֵ
    
    Tim3_ClearAllIntFlag();                                   //���жϱ�־
    Tim3_Mode23_EnableIrq(Tim3UevIrq);                        //ʹ��TIM3 UEV�����ж�
    EnableNvic(TIM3_IRQn, IrqLevel3, TRUE);                   //TIM3�ж�
		
    Tim3_M23_EnPWM_Output(TRUE, FALSE);        //���ʹ��
                         
}
void HC32_VALVE_Deinit(void)
{
	
	stc_gpio_cfg_t               stcTIM3Port;

	DDL_ZERO_STRUCT(stcTIM3Port);

	Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); 	//ʱ��

	stcTIM3Port.enDir  = GpioDirOut;

	Gpio_Init(GpioPortE, GpioPin9, &stcTIM3Port);
	
	Gpio_Init(GpioPortE, GpioPin10, &stcTIM3Port);

	Gpio_WriteOutputIO(GpioPortE,GpioPin9,0);
	Gpio_WriteOutputIO(GpioPortE,GpioPin9,0);
	
	Tim3_M23_ARRSet(0, FALSE);                       //������װֵ,��ʹ�ܻ���	
	HC32_Valve_Close();
	Tim3_M23_EnPWM_Output(FALSE, FALSE);        //���ʹ��
}
void Tim3_IRQHandler(void)
{
    //Timer3 ģʽ23 �����ж�
    if(TRUE == Tim3_GetIntFlag(Tim3UevIrq))
    {
				Tim3_M23_CCR_Set(Tim3CCR0A, 0x3000); //����CH0 ͨ��A�Ƚ�ֵ
				Tim3_M23_CCR_Set(Tim3CCR0B, 0x3000); //����CH0 ͨ��B�Ƚ�ֵ		
        
        Tim3_ClearIntFlag(Tim3UevIrq);  //����ж�
		}
}
/********************************************************************************
  ** \brief  ��������
  ** 
  ** @param  null
  ** 
  ** \retval null
******************************************************************************/ 
void HC32_Valve_Open(void)
{
		Tim3_M23_Run();    //Timer3����
}
/********************************************************************************
  ** \brief  �رշ���
  ** 
  ** @param  null
  ** 
  ** \retval null
******************************************************************************/ 
void HC32_Valve_Close(void)
{
		Tim3_M23_Stop();    //Timer3ֹͣ
}





















