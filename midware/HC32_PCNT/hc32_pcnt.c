/******************************************************************************
 ** \file hc32_pcnt.c
 **
 **  Copyright Fuzhou Internet of Things Open Lab.
 **
 **	 Author :Fiot-lab
 **
 **  DESCRIPTION: This file contains PCNT Configure.
 **
 ******************************************************************************/
/******************************************************************************
* Include files
******************************************************************************/
#include "hc32_pcnt.h"
#include "hc32_bc35.h"
#include "hc32_debug.h"
#include "common.h"
#include "hc32_eeprom.h"
/*******************************************************************************
 * global variable - (global)  and static variable - (static) 
 ******************************************************************************/
unsigned int gPcntWaterAmount = 0;

unsigned int gPcntHardNum = 0;

unsigned int gPcntWaterTon = 0;

unsigned int gPcntWaterPulse = 0;

unsigned char u8DevicePcnt = 0;

unsigned char u8DeviceOverCurrent = 0;

PCNT_NUM_T gUserWaterNum = {0};

PCNT_LAST_T gUserWaterLastNum = {0};

OVERCURRENT_THRE_T gUserOverCurrentThre = {1,25};

CONCURRENT_THRE_T gUserConCurrentThre = {1,25};

void HC32_PCNT_DebOut(void)
{
	stc_gpio_cfg_t stcGpioCfg;
	
	DDL_ZERO_STRUCT(stcGpioCfg);                 ///< 初始化结构体变量的值为0	
	
	Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);  ///< 开启GPIO时钟 	
	
	stcGpioCfg.enDir = GpioDirOut;               						///< 输出     
	stcGpioCfg.enOD = GpioOdEnable;              						///< 开漏输出     
	stcGpioCfg.enPu = GpioPuEnable;              						///< 上拉
	stcGpioCfg.enPd = GpioPdDisable;             						///< 不下拉	
	
	Gpio_Init(GpioPortC, GpioPin11,&stcGpioCfg); 						///< 初始化PC11	
	
	Gpio_SetAfMode(GpioPortC, GpioPin11,GpioAf4);						///< 复用PC11为PCNT_S0FO
	
	
}

/********************************************************************************
 ** \brief  PCNT计数配置
 ** 
 ** @param  null
 **
 ** \return null
 ******************************************************************************/
void HC32_PCNT_Init(void)
{

    stc_gpio_cfg_t GpioInitStruct;
    stc_pcnt_initstruct_t PcntInitStruct;
	
    DDL_ZERO_STRUCT(GpioInitStruct);
    DDL_ZERO_STRUCT(PcntInitStruct);
	
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralPcnt, TRUE);   
	
    GpioInitStruct.enDrv = GpioDrvH;
    GpioInitStruct.enDir = GpioDirIn;
    Gpio_Init(GpioPortC,GpioPin0,&GpioInitStruct);
	//Gpio_Init(GpioPortA,GpioPin12,&GpioInitStruct);
	
    Gpio_SetAfMode(GpioPortC,GpioPin0,GpioAf2);   //PC00作为PCNT_S0
	//Gpio_SetAfMode(GpioPortA,GpioPin12,GpioAf7);//PC00作为PCNT_S0
	
    PcntInitStruct.Pcnt_S0Sel = PcntS0PNoinvert;//S0输入极性不取反
    PcntInitStruct.Pcnt_Clk   = PcntCLKRcl;     //采样时钟
    PcntInitStruct.Pcnt_Mode  = PcntSingleMode; //双通道正交脉冲计数模式
    PcntInitStruct.Pcnt_FltEn = TRUE;          	//滤波使能
    PcntInitStruct.Pcnt_DebTop = 5;            	//滤波计数阈值
    PcntInitStruct.Pcnt_ClkDiv = 5;         	//滤波计数分频系数
    PcntInitStruct.Pcnt_TocrEn = FALSE;        	//超时控制使能
	PcntInitStruct.Pcnt_TocrTh = 2000;         	//超时阈值
    PcntInitStruct.Pcnt_Dir = PcntDirUp;		//向上计数
	#ifdef HC32_PCNTDEBUG_SUPPORTS			
	PcntInitStruct.Pcnt_Dbg = PcntDBGSxP;		//DebugOut
	#endif
    Pcnt_Init(&PcntInitStruct);
		
	Pcnt_SetB2T(99);							//计数值
    Pcnt_ClrItStatus(PcntOV);                   //清除PCNT溢出中断
    Pcnt_ItCfg(PcntOV, TRUE);                   //使能溢出中断
    EnableNvic(PCNT_IRQn, IrqLevel0, TRUE);     //中断优先级
		
	if((AT24CXX_ReadOneByte(ADDR_StorePcnt_START) == 0xFF) && (AT24CXX_ReadOneByte(ADDR_StorePcnt_START+3) == 0xFF)){
		for(uint8_t i = 0;i<=4;i++){
			AT24CXX_WriteLenByte(ADDR_StorePcnt_START+i,0,1);	
		}
		#ifdef HC32_LOG_SUPPORTS 
		FIOT_LAB_LOG(LOG_DEBUG,"Because First Power On , Erase Water Store Succeed!");
		#endif
	}
	else{
		//gPcntWaterTon = AT24CXX_ReadOneByte(ADDR_StorePcnt_START)*100 + AT24CXX_ReadOneByte(ADDR_StorePcnt_START+1);//获取吨数
		//gPcntWaterAmount = AT24CXX_ReadOneByte(ADDR_StorePcnt_START+4)*100 + AT24CXX_ReadOneByte(ADDR_StorePcnt_START+5);
		//gPcntWaterPulse = gPcntWaterAmount*100 + AT24CXX_ReadOneByte(ADDR_StorePcnt_START+2)*10 + AT24CXX_ReadOneByte(ADDR_StorePcnt_START+3)/10 + AT24CXX_ReadOneByte(ADDR_StorePcnt_START+6);//获取脉冲数
		
		FIOT_LAB_LOG(LOG_INFO,"gPcntWaterTon=%d gPcntWaterAmount=%d gPcntWaterPulse=%d cnt:%d\r\n",gPcntWaterTon,gPcntWaterAmount,gPcntWaterPulse,AT24CXX_ReadOneByte(ADDR_StorePcnt_START+6));
		
		gUserOverCurrentThre.u32OverCurrentThreH = AT24CXX_ReadOneByte(ADDR_OCURRENTTH_RECORD);//过流阈值
		gUserOverCurrentThre.u32OverCurrentThreL = AT24CXX_ReadOneByte(ADDR_OCURRENTTH_RECORD+1);
		
		gUserConCurrentThre.u32ConCurrentThreH = AT24CXX_ReadOneByte(ADDR_CCURRENTTH_RECORD);//反流阈值
		gUserConCurrentThre.u32ConCurrentThreL = AT24CXX_ReadOneByte(ADDR_CCURRENTTH_RECORD+1);
		
	}	
	FIOT_LAB_LOG(LOG_INFO,"OverCurrentThreshold : %02d.%02d \r\n",gUserOverCurrentThre.u32OverCurrentThreH,gUserOverCurrentThre.u32OverCurrentThreL);	
	FIOT_LAB_LOG(LOG_INFO,"ConCurrentThreshold : %02d.%02d \r\n",gUserConCurrentThre.u32ConCurrentThreH,gUserConCurrentThre.u32ConCurrentThreL);	
	
	if(AT24CXX_ReadOneByte(ADDR_OCURRENTTIME_RECORD) != 0xff && AT24CXX_ReadOneByte(ADDR_OCURRENTTIME_RECORD+1) != 0xFF){
		gBc35UserOverCurrentTime = AT24CXX_ReadOneByte(ADDR_OCURRENTTIME_RECORD) * 100 + AT24CXX_ReadOneByte(ADDR_OCURRENTTIME_RECORD+1);	
	}else{
		gBc35UserOverCurrentTime = 30;
	}
	#ifdef HC32_LOG_DEBUG
	FIOT_LAB_LOG(LOG_DEBUG,"OverCurrentTime : %d Minute\r\n",gBc35UserOverCurrentTime);
	#endif		

	if(AT24CXX_ReadOneByte(ADDR_CCURRENTTIME_RECORD) != 0xff && AT24CXX_ReadOneByte(ADDR_CCURRENTTIME_RECORD+1) != 0xFF){
		gBc35UserConCurrentTime = AT24CXX_ReadOneByte(ADDR_CCURRENTTIME_RECORD) * 100 + AT24CXX_ReadOneByte(ADDR_CCURRENTTIME_RECORD+1);	
	}else{
		gBc35UserConCurrentTime = 30;
	}
	#ifdef HC32_LOG_DEBUG
	FIOT_LAB_LOG(LOG_DEBUG,"ConCurrentTime : %d Minute\r\n",gBc35UserConCurrentTime);
	#endif
	
	#ifdef HC32_LOG_SUPPORTS 
		FIOT_LAB_LOG(LOG_INFO,"PCNT(PC0) DebTop:%d ClkDiv:%d PcntClk:%d\r\n",PcntInitStruct.Pcnt_DebTop, PcntInitStruct.Pcnt_ClkDiv,Sysctrl_GetPClkFreq());
	#endif
	#ifdef HC32_PCNTDEBUG_SUPPORTS		
		HC32_PCNT_DebOut();
	#endif
	
	WaterFlowFloat();
	
	gUserWaterLastNum.u32WaterLastH = WaterReturnInt();
	
	gUserWaterLastNum.u32WaterLastL = WaterReturnFloat();	
}
/********************************************************************************
 ** \brief  中断函数入口
 ** 
 ** @param  null
 **
 ** \return null
 ******************************************************************************/
void Pcnt_IRQHandler(void)
{
    Pcnt_ClrItStatus(PcntOV);
	gPcntWaterAmount++;
	u8DevicePcnt = 1;
}
/********************************************************************************
 ** \brief  开启计数
 ** 
 ** @param  null
 **
 ** \return null
 ******************************************************************************/
void ReedPipe_Enable(void)
{
    Pcnt_Cmd(TRUE);        //使能PCNT
}
/********************************************************************************
 ** \brief  关闭计数
 ** 
 ** @param  null
 **
 ** \return null
 ******************************************************************************/
void ReedPipe_Disable(void)
{
    Pcnt_Cmd(FALSE);        //使能PCNT
}

/********************************************************************************
 ** \brief  PCNT统计
 ** 
 ** @param  null
 **
 ** \return 返回总PCNT
 ******************************************************************************/
unsigned int WaterFlowStatistics(void){
	static uint32_t tmp = 0,cnt = 0;
	if(gPcntWaterAmount == 10){//100 * 0.001m3  0.1m
		gPcntWaterAmount = 0;//extern 
		gPcntWaterTon++;
		if(gPcntWaterTon >=65535){
			//最大计数65535吨
		}	
		AT24CXX_WriteLenByte(ADDR_StorePcnt_START+0,gPcntWaterTon%10000/100,1);
		AT24CXX_WriteLenByte(ADDR_StorePcnt_START+1,gPcntWaterTon%10000%100,1);		
	}
//	if(gPcntWaterAmount != tmp){//每100个脉冲存储一次
//		tmp = gPcntWaterAmount;
//		AT24CXX_WriteLenByte(ADDR_StorePcnt_START+4,gPcntWaterAmount,1);				
//		#ifdef HC32_LOG_DEBUG
//		FIOT_LAB_LOG(LOG_INFO,"Now (100Num)Pulse Cnt is %d ,Ton is %.4f\r\n",gPcntWaterAmount,gPcntWaterTon + gPcntWaterAmount*0.03);
//		#endif
//	}
//	if(Pcnt_GetCnt() != cnt){
//		cnt = Pcnt_GetCnt();
//		AT24CXX_WriteLenByte(ADDR_StorePcnt_START+6,Pcnt_GetCnt(),1);
//		#ifdef HC32_LOG_DEBUG
//		FIOT_LAB_LOG(LOG_INFO,"cnt:%d\r\n",Pcnt_GetCnt());
//		#endif		
//	}
	return gPcntWaterAmount*100 + Pcnt_GetCnt();
}	
/********************************************************************************
 ** \brief  水量统计
 ** 
 ** @param  null
 **
 ** \return 返回水量pcnt
 ******************************************************************************/
uint32_t WaterFlowFloat(void){
	double floatWaterFlow = WaterFlowStatistics() * EVERYM3WATER + gPcntWaterTon + gPcntWaterPulse*0.001;
	double intpart;
	double fractpart = modf(floatWaterFlow, &intpart);				
	gUserWaterNum.u32WaterL = fractpart * 10000;
	gUserWaterNum.u32WaterH = intpart;	
	return 0;
}
uint32_t WaterReturnInt(void){
	return gUserWaterNum.u32WaterH;
}
uint32_t WaterReturnFloat(void){
	return gUserWaterNum.u32WaterL;
}
void WaterPcntClear(uint32_t Ton_t,uint32_t Pulse_t)
{
	gPcntWaterTon = Ton_t;
	gPcntWaterPulse = Pulse_t - Pcnt_GetCnt();
}
boolean_t JudgeNowOverCurrent(void){
	FIOT_LAB_LOG(LOG_DEBUG,"OverCurrentThre:%d.%d\r\n",gUserOverCurrentThre.u32OverCurrentThreH,gUserOverCurrentThre.u32OverCurrentThreL);
	if(WaterReturnInt() - gUserWaterLastNum.u32WaterLastH > gUserOverCurrentThre.u32OverCurrentThreH){ //大于1.25 m3 超流
		return 1;
	}else if(WaterReturnInt() - gUserWaterLastNum.u32WaterLastH == gUserOverCurrentThre.u32OverCurrentThreH){
		if((WaterReturnFloat() - gUserWaterLastNum.u32WaterLastL) >= gUserOverCurrentThre.u32OverCurrentThreL)
			return 1;
		else
			return 	0;
	}else{
		return 	0;
	}
}
void StoreLastWaterNum(uint32_t WaterLastH,uint32_t WaterLastL)
{
	gUserWaterLastNum.u32WaterLastH = WaterLastH;
	
	gUserWaterLastNum.u32WaterLastL = WaterLastL;	
	
	FIOT_LAB_LOG(LOG_DEBUG,"u32WaterLastH:%d %d \r\n",gUserWaterLastNum.u32WaterLastH,gUserWaterLastNum.u32WaterLastL);
}
void SetOverCurrentValue(uint32_t WaterThreH,uint32_t WaterThreL){
	
	gUserOverCurrentThre.u32OverCurrentThreH = WaterThreH;
	
	gUserOverCurrentThre.u32OverCurrentThreL = WaterThreL;
}
void SetConCurrentValue(uint32_t WaterThreH,uint32_t WaterThreL){
	
	gUserOverCurrentThre.u32OverCurrentThreH = WaterThreH;
	
	gUserOverCurrentThre.u32OverCurrentThreL = WaterThreL;
}
/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/



