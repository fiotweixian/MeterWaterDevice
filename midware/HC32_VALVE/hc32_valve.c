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
    
    ///< 打开GPIO外设时钟门控
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); 
    
    ///< 端口方向配置
    stcGpioCfg.enDir = GpioDirOut;
    ///< 配置上下拉
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdEnable;
    
    ///<  SLEEP关闭
    Gpio_ClrIO(USER_SLEEP_PORT, USER_SLEEP_PIN);
    
    ///< GPIO IO SLEEP端口初始化
    Gpio_Init(USER_SLEEP_PORT, USER_SLEEP_PIN, &stcGpioCfg);

		HC32_SLEEP_Hight();
}
/********************************************************************************
  ** \brief  Valve初始化
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
	
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); 	//时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralTim3, TRUE);   //Timer3时钟
	
    stcTIM3Port.enDir  = GpioDirOut;
	
    Gpio_Init(GpioPortE, GpioPin9, &stcTIM3Port);
    Gpio_SetAfMode(GpioPortE,GpioPin9,GpioAf1);               //PE09配置TIM3_CH0A
    
    Gpio_Init(GpioPortE, GpioPin10, &stcTIM3Port);
    Gpio_SetAfMode(GpioPortE,GpioPin10,GpioAf1);               //PE08配置TIM3_CH0B
	
    stcTim3BaseCfg.enWorkMode    = Tim3WorkMode2;             //锯齿波
    stcTim3BaseCfg.enCT          = Tim3Timer;                 //定时器功能,内部的PCLK
    stcTim3BaseCfg.enPRS         = Tim3PCLKDiv256;            //PCLK
    stcTim3BaseCfg.enCntDir      = Tim3CntUp;                 //向上计数
    stcTim3BaseCfg.enPWMTypeSel  = Tim3IndependentPWM;        //独立输出PWM
    stcTim3BaseCfg.enPWM2sSel    = Tim3SinglePointCmp;        //单点比较功能
    stcTim3BaseCfg.bOneShot      = FALSE;                     //循环计数
    stcTim3BaseCfg.bURSSel       = FALSE;                     //上下溢出更新
    
    Tim3_Mode23_Init(&stcTim3BaseCfg);                        //TIM3 模式23初始化
    
    Tim3_M23_ARRSet(u16Period, TRUE);                       //设置重装值,并使能缓存
		
    Tim3_M23_CCR_Set(Tim3CCR0A, u16CHxACompare);            //设置CH0比较值A
//    Tim3_M23_CCR_Set(Tim3CCR1B, u16CHxBCompare);            //设置CH0比较值B  

    stcTim3PortCmpCfg.enCHxACmpCtrl   = Tim3PWMMode2;         //OCREFA输出控制OCMA:PWM模式2
    stcTim3PortCmpCfg.enCHxAPolarity  = Tim3PortPositive;     //正常输出
    stcTim3PortCmpCfg.bCHxACmpBufEn   = TRUE;                 //A缓存机制
    stcTim3PortCmpCfg.enCHxACmpIntSel = Tim3CmpIntNone;       //A中断比较控制:无
		
    Tim3_M23_PortOutput_Cfg(Tim3CH0, &stcTim3PortCmpCfg);     //比较输出端口配置
    
    Tim3_M23_CCR_Set(Tim3CCR1B, u16CHxBCompare);            //设置CH0比较值B 
	 
    stcTim3PortCmpCfg.enCHxBCmpCtrl   = Tim3PWMMode2;         //OCREFB输出控制OCMA:PWM模式2
    stcTim3PortCmpCfg.enCHxBPolarity  = Tim3PortOpposite;     //正常输出
    stcTim3PortCmpCfg.bCHxBCmpBufEn   = TRUE;                 //B缓存机制
    stcTim3PortCmpCfg.enCHxBCmpIntSel = Tim3CmpIntNone;       //B中断比较控制:无
    
    Tim3_M23_PortOutput_Cfg(Tim3CH1, &stcTim3PortCmpCfg);     //比较输出端口配置 
    
    u8ValidPeriod = 0;                                        //时间更新周期设置
    Tim3_M23_SetValidPeriod(u8ValidPeriod);                   //间隔周期设置
    
    u16CntValue = 0;
    
    Tim3_M23_Cnt16Set(u16CntValue);                           //设置计数初值
    
    Tim3_ClearAllIntFlag();                                   //清中断标志
    Tim3_Mode23_EnableIrq(Tim3UevIrq);                        //使能TIM3 UEV更新中断
    EnableNvic(TIM3_IRQn, IrqLevel3, TRUE);                   //TIM3中断
		
    Tim3_M23_EnPWM_Output(TRUE, FALSE);        //输出使能
                         
}
void HC32_VALVE_Deinit(void)
{
	
	stc_gpio_cfg_t               stcTIM3Port;

	DDL_ZERO_STRUCT(stcTIM3Port);

	Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); 	//时钟

	stcTIM3Port.enDir  = GpioDirOut;

	Gpio_Init(GpioPortE, GpioPin9, &stcTIM3Port);
	
	Gpio_Init(GpioPortE, GpioPin10, &stcTIM3Port);

	Gpio_WriteOutputIO(GpioPortE,GpioPin9,0);
	Gpio_WriteOutputIO(GpioPortE,GpioPin9,0);
	
	Tim3_M23_ARRSet(0, FALSE);                       //设置重装值,并使能缓存	
	HC32_Valve_Close();
	Tim3_M23_EnPWM_Output(FALSE, FALSE);        //输出使能
}
void Tim3_IRQHandler(void)
{
    //Timer3 模式23 更新中断
    if(TRUE == Tim3_GetIntFlag(Tim3UevIrq))
    {
				Tim3_M23_CCR_Set(Tim3CCR0A, 0x3000); //设置CH0 通道A比较值
				Tim3_M23_CCR_Set(Tim3CCR0B, 0x3000); //设置CH0 通道B比较值		
        
        Tim3_ClearIntFlag(Tim3UevIrq);  //清除中断
		}
}
/********************************************************************************
  ** \brief  开启阀门
  ** 
  ** @param  null
  ** 
  ** \retval null
******************************************************************************/ 
void HC32_Valve_Open(void)
{
		Tim3_M23_Run();    //Timer3运行
}
/********************************************************************************
  ** \brief  关闭阀门
  ** 
  ** @param  null
  ** 
  ** \retval null
******************************************************************************/ 
void HC32_Valve_Close(void)
{
		Tim3_M23_Stop();    //Timer3停止
}





















