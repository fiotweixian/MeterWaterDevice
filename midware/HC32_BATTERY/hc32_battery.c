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
uint32_t u32AdcRestult = 0;//ADC采样值

/*******************************************************************************
电压等级分为4级:0 1 2 3.等级高,电压高.
******************************************************************************/
char gBc35UserBatteryValueLevel = 0;//电压值等级

/********************************************************************************
  ** \brief  BATTERY GPIO 初始化
  ** 
  ** @param  NULL
  ** 
  ** \retval NULL
******************************************************************************/ 
void HC32_BATTERY_GPIO(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    ///< 打开GPIO外设时钟门控
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); 
    
    ///< 端口方向配置
    stcGpioCfg.enDir = GpioDirOut;
    ///< 配置上下拉失能
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
    
    ///< Battery IO 关闭
    Gpio_ClrIO(USER_BATTERY_PORT, USER_BATTERY_PIN);
    
    ///< GPIO IO Battery端口初始化
    Gpio_Init(USER_BATTERY_PORT, USER_BATTERY_PIN, &stcGpioCfg);	
}
/********************************************************************************
  ** \brief  BATTERY ADC 初始化
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
    ///< 开启 GPIO时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    ///< 开启ADC/BGR 时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE); 
    Bgr_BgrEnable();        ///<开启BGR	
	Gpio_SetAnalogMode(GpioPortC, GpioPin1);        		//PC01 (AIN11)
    ///< ADC 配置
    stcAdcCfg.enAdcMode         = AdcSglMode;               ///<单次采样模式
    stcAdcCfg.enAdcClkDiv       = AdcMskClkDiv8;            ///<采样分频-8
    stcAdcCfg.enAdcSampCycleSel = AdcMskSampCycle12Clk;     ///<采样周期数-12
    stcAdcCfg.enAdcRefVolSel    = AdcMskRefVolSelInBgr2p5;  ///<参考电压选择-内部2.5V
    stcAdcCfg.enAdcOpBuf        = AdcMskBufEnable;         	///<OP BUF配置-ON
    stcAdcCfg.enInRef           = AdcMskInRefEnable;        ///<内部参考电压使能-ON
    stcAdcCfg.enAdcAlign        = AdcAlignRight;            ///<转换结果对齐方式-右
    Adc_Init(&stcAdcCfg);	
    ///< ADC 采样通道配置
    Adc_CfgSglChannel(AdcExInputCH11);
   
	///<开始采集
    Adc_SGL_Always_Start(); 		
	
	///获取电压值
	BatteryValue = BatteryObtain();
	//判断电压等级
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
  ** \brief  BATTERY ADC 清除配置
  ** 
  ** @param  NULL
  ** 
  ** \retval NULL
******************************************************************************/ 
void HC32_BATTERY_DeInit(void)
{	
    Bgr_BgrDisable();//关闭bgr 必须开启时钟
	
	delay10us(2);//等待关闭

	Adc_DeInit();//ADC 清除配置

	delay10us(2);//等待配置
	
	Adc_ClrIrqStatus(AdcMskIrqSgl);///< 清除中断标志位
	
	Adc_SGL_Always_Stop();///停止ADC采集	
}
/********************************************************************************
  ** \brief  开启采集
  ** 
  ** @param  null
  ** 
  ** \retval null
******************************************************************************/ 
void HC32_BATTERY_Enable(void)
{
    ///< 启动单次采集
    Adc_SGL_Always_Start();   	
}
/********************************************************************************
  ** \brief  关闭采集
  ** 
  ** @param  null
  ** 
  ** \retval null
******************************************************************************/ 
void HC32_BATTERY_Disable(void)
{
    ///< 停止单次采集
    Adc_SGL_Always_Stop();   	
}
/********************************************************************************
  ** \brief  得到电压值
  ** 
  ** @param  null
  ** 
  ** \retval 返回电压值
******************************************************************************/ 
float BatteryObtain(void)
{
	uint32_t u32BatteryRestult = 0;
	char i = 0,j = 0;
	HC32_BATTERY_Hight();	
	do{		
		if(TRUE == Adc_GetIrqStatus(AdcMskIrqSgl)){
			if(i >= 5){//滤波掉前面几次不稳定数据
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
  ** \brief  得到电压等级
  ** 
  ** @param  null
  ** 
  ** \retval 返回电压等级
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
















