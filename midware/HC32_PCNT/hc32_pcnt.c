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
	
	DDL_ZERO_STRUCT(stcGpioCfg);                 ///< ��ʼ���ṹ�������ֵΪ0	
	
	Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);  ///< ����GPIOʱ�� 	
	
	stcGpioCfg.enDir = GpioDirOut;               						///< ���     
	stcGpioCfg.enOD = GpioOdEnable;              						///< ��©���     
	stcGpioCfg.enPu = GpioPuEnable;              						///< ����
	stcGpioCfg.enPd = GpioPdDisable;             						///< ������	
	
	Gpio_Init(GpioPortC, GpioPin11,&stcGpioCfg); 						///< ��ʼ��PC11	
	
	Gpio_SetAfMode(GpioPortC, GpioPin11,GpioAf4);						///< ����PC11ΪPCNT_S0FO
	
	
}

/********************************************************************************
 ** \brief  PCNT��������
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
	
    Gpio_SetAfMode(GpioPortC,GpioPin0,GpioAf2);   //PC00��ΪPCNT_S0
	//Gpio_SetAfMode(GpioPortA,GpioPin12,GpioAf7);//PC00��ΪPCNT_S0
	
    PcntInitStruct.Pcnt_S0Sel = PcntS0PNoinvert;//S0���뼫�Բ�ȡ��
    PcntInitStruct.Pcnt_Clk   = PcntCLKRcl;     //����ʱ��
    PcntInitStruct.Pcnt_Mode  = PcntSingleMode; //˫ͨ�������������ģʽ
    PcntInitStruct.Pcnt_FltEn = TRUE;          	//�˲�ʹ��
    PcntInitStruct.Pcnt_DebTop = 5;            	//�˲�������ֵ
    PcntInitStruct.Pcnt_ClkDiv = 5;         	//�˲�������Ƶϵ��
    PcntInitStruct.Pcnt_TocrEn = FALSE;        	//��ʱ����ʹ��
	PcntInitStruct.Pcnt_TocrTh = 2000;         	//��ʱ��ֵ
    PcntInitStruct.Pcnt_Dir = PcntDirUp;		//���ϼ���
	#ifdef HC32_PCNTDEBUG_SUPPORTS			
	PcntInitStruct.Pcnt_Dbg = PcntDBGSxP;		//DebugOut
	#endif
    Pcnt_Init(&PcntInitStruct);
		
	Pcnt_SetB2T(99);							//����ֵ
    Pcnt_ClrItStatus(PcntOV);                   //���PCNT����ж�
    Pcnt_ItCfg(PcntOV, TRUE);                   //ʹ������ж�
    EnableNvic(PCNT_IRQn, IrqLevel0, TRUE);     //�ж����ȼ�
		
	if((AT24CXX_ReadOneByte(ADDR_StorePcnt_START) == 0xFF) && (AT24CXX_ReadOneByte(ADDR_StorePcnt_START+3) == 0xFF)){
		for(uint8_t i = 0;i<=4;i++){
			AT24CXX_WriteLenByte(ADDR_StorePcnt_START+i,0,1);	
		}
		#ifdef HC32_LOG_SUPPORTS 
		FIOT_LAB_LOG(LOG_DEBUG,"Because First Power On , Erase Water Store Succeed!");
		#endif
	}
	else{
		//gPcntWaterTon = AT24CXX_ReadOneByte(ADDR_StorePcnt_START)*100 + AT24CXX_ReadOneByte(ADDR_StorePcnt_START+1);//��ȡ����
		//gPcntWaterAmount = AT24CXX_ReadOneByte(ADDR_StorePcnt_START+4)*100 + AT24CXX_ReadOneByte(ADDR_StorePcnt_START+5);
		//gPcntWaterPulse = gPcntWaterAmount*100 + AT24CXX_ReadOneByte(ADDR_StorePcnt_START+2)*10 + AT24CXX_ReadOneByte(ADDR_StorePcnt_START+3)/10 + AT24CXX_ReadOneByte(ADDR_StorePcnt_START+6);//��ȡ������
		
		FIOT_LAB_LOG(LOG_INFO,"gPcntWaterTon=%d gPcntWaterAmount=%d gPcntWaterPulse=%d cnt:%d\r\n",gPcntWaterTon,gPcntWaterAmount,gPcntWaterPulse,AT24CXX_ReadOneByte(ADDR_StorePcnt_START+6));
		
		gUserOverCurrentThre.u32OverCurrentThreH = AT24CXX_ReadOneByte(ADDR_OCURRENTTH_RECORD);//������ֵ
		gUserOverCurrentThre.u32OverCurrentThreL = AT24CXX_ReadOneByte(ADDR_OCURRENTTH_RECORD+1);
		
		gUserConCurrentThre.u32ConCurrentThreH = AT24CXX_ReadOneByte(ADDR_CCURRENTTH_RECORD);//������ֵ
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
 ** \brief  �жϺ������
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
 ** \brief  ��������
 ** 
 ** @param  null
 **
 ** \return null
 ******************************************************************************/
void ReedPipe_Enable(void)
{
    Pcnt_Cmd(TRUE);        //ʹ��PCNT
}
/********************************************************************************
 ** \brief  �رռ���
 ** 
 ** @param  null
 **
 ** \return null
 ******************************************************************************/
void ReedPipe_Disable(void)
{
    Pcnt_Cmd(FALSE);        //ʹ��PCNT
}

/********************************************************************************
 ** \brief  PCNTͳ��
 ** 
 ** @param  null
 **
 ** \return ������PCNT
 ******************************************************************************/
unsigned int WaterFlowStatistics(void){
	static uint32_t tmp = 0,cnt = 0;
	if(gPcntWaterAmount == 10){//100 * 0.001m3  0.1m
		gPcntWaterAmount = 0;//extern 
		gPcntWaterTon++;
		if(gPcntWaterTon >=65535){
			//������65535��
		}	
		AT24CXX_WriteLenByte(ADDR_StorePcnt_START+0,gPcntWaterTon%10000/100,1);
		AT24CXX_WriteLenByte(ADDR_StorePcnt_START+1,gPcntWaterTon%10000%100,1);		
	}
//	if(gPcntWaterAmount != tmp){//ÿ100������洢һ��
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
 ** \brief  ˮ��ͳ��
 ** 
 ** @param  null
 **
 ** \return ����ˮ��pcnt
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
	if(WaterReturnInt() - gUserWaterLastNum.u32WaterLastH > gUserOverCurrentThre.u32OverCurrentThreH){ //����1.25 m3 ����
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



