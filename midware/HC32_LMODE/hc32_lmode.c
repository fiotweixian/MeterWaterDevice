/******************************************************************************
 ** \file hc32_lmode.c
 **
 **  Copyright Fuzhou Internet of Things Open Lab.
 **
 **	 Author :Fiot-lab
 **
 **  DESCRIPTION: This file contains LowPower Configure.
 **
 ******************************************************************************/
#include "hc32_lmode.h"
#include "hc32_key.h"
#include "hc32_debug.h"
#include "hc32_led.h"
#include "hc32_battery.h"
#include "hc32_rtc.h"
#include "hc32_bc35.h"
//默认不关闭
//#define HC32_SWD_LOAD_ENABLE

/*******************************************************************************
 * global variable - (global)  and static variable - (static) 
 ******************************************************************************/
 
 

/********************************************************************************
  ** \brief  LMODE配置函数
  ** 
  ** @param  null
  ** 
  ** \retval null
******************************************************************************/ 
void HC32_DeepSleepCfg(void)
{  
		//说明：进入深度睡眠模式，需要保留的IO:
		//报警唤醒IO口	
    //初始化IO配置(follow STK)

	 ///< 不用的引脚全部配置成 数字端口
	
//LCD COM : PA9 PA10 PA11 PA12 
//LCD VLCD:	PB03 PB04 PB05 PB06	
//SEG:
//	 A:PA0 PA1 PA2 PA3 PA4	PA5 PA6
//   B:PB00 PB01 PB02 PB12 PB13 PB15
//	 C:PC04 PC05
//	 D:PD10 PD11
//   E:PE12 PE13 PE14 PE15
	
//    M0P_GPIO->PAADS = 0; 	
//		M0P_GPIO->PAADS_f.PA00 = 0;
//		M0P_GPIO->PAADS_f.PA01 = 0;
//		M0P_GPIO->PAADS_f.PA02 = 0;
//		M0P_GPIO->PAADS_f.PA03 = 0;
//		M0P_GPIO->PAADS_f.PA04 = 0;
//		M0P_GPIO->PAADS_f.PA05 = 0;
//		M0P_GPIO->PAADS_f.PA06 = 0; //LCD SEG 不配置
//		M0P_GPIO->PAADS_f.PA07 = 0;
		M0P_GPIO->PAADS_f.PA08 = 0;
//		M0P_GPIO->PAADS_f.PA09 = 0;
//		M0P_GPIO->PAADS_f.PA10 = 0;
//		M0P_GPIO->PAADS_f.PA11 = 0;
//		M0P_GPIO->PAADS_f.PA12 = 0;	//LCD COM 不配置
		M0P_GPIO->PAADS_f.PA13 = 0;
		M0P_GPIO->PAADS_f.PA14 = 0;
		M0P_GPIO->PAADS_f.PA15 = 0;		
		
//    M0P_GPIO->PBADS = 0;
//		M0P_GPIO->PBADS_f.PB00 = 0;
//		M0P_GPIO->PBADS_f.PB01 = 0;
//		M0P_GPIO->PBADS_f.PB02 = 0;//LCD SEG 不配置
//		M0P_GPIO->PBADS_f.PB03 = 0;
//		M0P_GPIO->PBADS_f.PB04 = 0;
//		M0P_GPIO->PBADS_f.PB05 = 0;
//		M0P_GPIO->PBADS_f.PB06 = 0;//LCD VLCD 不配置
		M0P_GPIO->PBADS_f.PB07 = 0;
		M0P_GPIO->PBADS_f.PB08 = 0;
		M0P_GPIO->PBADS_f.PB09 = 0;
		M0P_GPIO->PBADS_f.PB10 = 0;
		M0P_GPIO->PBADS_f.PB11 = 0;
//		M0P_GPIO->PBADS_f.PB12 = 0;
//		M0P_GPIO->PBADS_f.PB13 = 0;//LCD SEG 不配置
		M0P_GPIO->PBADS_f.PB14 = 0;
//		M0P_GPIO->PBADS_f.PB15 = 0;//LCD SEG 不配置		
		
		
//    M0P_GPIO->PCADS = 0;	//LCD SEG 不配置
		M0P_GPIO->PCADS_f.PC00 = 0;
		M0P_GPIO->PCADS_f.PC01 = 0;
		M0P_GPIO->PCADS_f.PC02 = 0;
		M0P_GPIO->PCADS_f.PC03 = 0;
//		M0P_GPIO->PCADS_f.PC04 = 0;
//		M0P_GPIO->PCADS_f.PC05 = 0;//LCD SEG 不配置
		M0P_GPIO->PCADS_f.PC06 = 0;
		M0P_GPIO->PCADS_f.PC07 = 0;
		M0P_GPIO->PCADS_f.PC08 = 0;
		M0P_GPIO->PCADS_f.PC09 = 0;
		M0P_GPIO->PCADS_f.PC10 = 0;
		M0P_GPIO->PCADS_f.PC11 = 0;
		M0P_GPIO->PCADS_f.PC12 = 0;
		M0P_GPIO->PCADS_f.PC13 = 0;
		M0P_GPIO->PCADS_f.PC14 = 0;
		M0P_GPIO->PCADS_f.PC15 = 0;

//    M0P_GPIO->PDADS = 0; //LCD SEG 不配置	
		M0P_GPIO->PDADS_f.PD00 = 0;
		M0P_GPIO->PDADS_f.PD01 = 0;
		M0P_GPIO->PDADS_f.PD02 = 0;
		M0P_GPIO->PDADS_f.PD03 = 0;
		M0P_GPIO->PDADS_f.PD04 = 0;
//		M0P_GPIO->PDADS_f.PD05 = 0;
//		M0P_GPIO->PDADS_f.PD06 = 0;//NB 不配置
		M0P_GPIO->PDADS_f.PD07 = 0;
		M0P_GPIO->PDADS_f.PD08 = 0;
		M0P_GPIO->PDADS_f.PD09 = 0;
//		M0P_GPIO->PDADS_f.PD10 = 0;
//		M0P_GPIO->PDADS_f.PD11 = 0;//LCD SEG 不配置
		M0P_GPIO->PDADS_f.PD12 = 0;
		M0P_GPIO->PDADS_f.PD13 = 0;
		M0P_GPIO->PDADS_f.PD14 = 0;
		M0P_GPIO->PDADS_f.PD15 = 0;


//    M0P_GPIO->PEADS = 0; //LCD SEG 不配置	
		M0P_GPIO->PEADS_f.PE00 = 0;
		M0P_GPIO->PEADS_f.PE01 = 0;
		M0P_GPIO->PEADS_f.PE02 = 0;
		M0P_GPIO->PEADS_f.PE03 = 0;
		M0P_GPIO->PEADS_f.PE04 = 0;
		M0P_GPIO->PEADS_f.PE05 = 0;
		M0P_GPIO->PEADS_f.PE06 = 0;
		M0P_GPIO->PEADS_f.PE07 = 0;
		M0P_GPIO->PEADS_f.PE08 = 0;
		M0P_GPIO->PEADS_f.PE09 = 0;
		M0P_GPIO->PEADS_f.PE10 = 0;
		M0P_GPIO->PEADS_f.PE11 = 0;
//		M0P_GPIO->PEADS_f.PE12 = 0;
//		M0P_GPIO->PEADS_f.PE13 = 0;
//		M0P_GPIO->PEADS_f.PE14 = 0;
//		M0P_GPIO->PEADS_f.PE15 = 0;//LCD SEG 不配置

    M0P_GPIO->PFADS = 0;
		
		
    ///< 不用的引脚全部配置成 输入IO
    //M0P_GPIO->PADIR = 0XFFFF;  // LCD COM
		M0P_GPIO->PADIR_f.PA07 = 1;
		M0P_GPIO->PADIR_f.PA08 = 1;		
		M0P_GPIO->PADIR_f.PA13 = 1;
		M0P_GPIO->PADIR_f.PA14 = 1;
		M0P_GPIO->PADIR_f.PA15 = 1;	


		
//    M0P_GPIO->PBDIR = 0XF0FF;   // 0XF087 0000 0000
		M0P_GPIO->PBDIR_f.PB07 = 1;
		M0P_GPIO->PBDIR_f.PB08 = 0;
		M0P_GPIO->PBDIR_f.PB09 = 0;
		M0P_GPIO->PBDIR_f.PB10 = 0;
		M0P_GPIO->PBDIR_f.PB11 = 0;		
		M0P_GPIO->PBDIR_f.PB14 = 1;


    M0P_GPIO->PCDIR = 0XF7CF;//
		
		
//    M0P_GPIO->PDDIR = 0XF3DF;//9F   1101  0000
		M0P_GPIO->PDDIR_f.PD00 = 1;
		M0P_GPIO->PDDIR_f.PD01 = 1;
		M0P_GPIO->PDDIR_f.PD02 = 1;
		M0P_GPIO->PDDIR_f.PD03 = 1;
		M0P_GPIO->PDDIR_f.PD04 = 1;
//		M0P_GPIO->PDDIR_f.PD05 = 1;
//		M0P_GPIO->PDDIR_f.PD06 = 1;
		M0P_GPIO->PDDIR_f.PD07 = 1;
		M0P_GPIO->PDDIR_f.PD08 = 1;
		M0P_GPIO->PDDIR_f.PD09 = 1;		
		M0P_GPIO->PDDIR_f.PD10 = 0;
		M0P_GPIO->PDDIR_f.PD11 = 0;		
		M0P_GPIO->PDDIR_f.PD12 = 1;		
		M0P_GPIO->PDDIR_f.PD13 = 1;	
		M0P_GPIO->PDDIR_f.PD14 = 1;
		M0P_GPIO->PDDIR_f.PD15 = 1;	
		
    M0P_GPIO->PEDIR = 0X0FFF;
		
    M0P_GPIO->PFDIR = 0XFFFF;
		
		///< 不用的引脚全部配置成 下拉    
    M0P_GPIO->PAPD = 0XFFFF;  
//////////////////////////////////////		
    M0P_GPIO->PBPD = 0xF0FF;//0xF08787
/////////////////////////////////		
    M0P_GPIO->PCPD = 0xF77E;//按键上拉DF5F
////////////////////////////////////////		
 //   M0P_GPIO->PDPD = 0xFFBF;//9f   1011  
		M0P_GPIO->PDPD_f.PD00 = 1;
		M0P_GPIO->PDPD_f.PD01 = 1;
		M0P_GPIO->PDPD_f.PD02 = 1;
		M0P_GPIO->PDPD_f.PD03 = 1;
		M0P_GPIO->PDPD_f.PD04 = 1;
//		M0P_GPIO->PDPD_f.PD05 = 1;
//		M0P_GPIO->PDPD_f.PD06 = 1;
		M0P_GPIO->PDPD_f.PD07 = 1;
		M0P_GPIO->PDPD_f.PD08 = 1;
		M0P_GPIO->PDPD_f.PD09 = 1;		
		M0P_GPIO->PDPD_f.PD10 = 1;
		M0P_GPIO->PDPD_f.PD11 = 1;	
		M0P_GPIO->PDPD_f.PD12 = 1;
		M0P_GPIO->PDPD_f.PD13 = 1;	
		M0P_GPIO->PDPD_f.PD14 = 1;
		M0P_GPIO->PDPD_f.PD15 = 1;	 
//////////////////////////////////////
    M0P_GPIO->PEPD = 0xFFFF;
/////////////////////////////////////		
    M0P_GPIO->PFPD = 0xFFFF;
		

#ifdef HC32_SWD_LOAD_ENABLE
    //swd接口配置gpio,使无法进入debug调试
    M0P_SYSCTRL->SYSCTRL2 = 0X5A5A;
    M0P_SYSCTRL->SYSCTRL2 = 0XA5A5;
    M0P_SYSCTRL->SYSCTRL1_f.SWD_USE_IO = 1;
#endif
}









