/******************************************************************************
 ** \file hc32_lcd.c
 **
 **  Copyright Fuzhou Internet of Things Open Lab.
 **
 **	 Author :Fiot-lab
 **
 **  DESCRIPTION: This file contains LCD Configure and Display.
 **
 ******************************************************************************/
/******************************************************************************
* Include files
******************************************************************************/ 
#include "hc32_lcd.h"
#include "hc32_debug.h"

/*******************************************************************************
 * global variable - (global)  and static variable - (static) 
 ******************************************************************************/
// 显示数字：0 1 2 3 4 5 6 7 9
unsigned int Num9Select[11][2] = {{0x00000700,0x00000D00}, {0x00000600,0x00000000}, 
																	{0x00000300,0x00000E00}, {0x00000700,0x00000A00}, 
																	{0x00000600,0x00000300}, {0x00000500,0x00000B00},  
																	{0x00000500,0x00000F00}, {0x00000700,0x00000000}, 
																	{0x00000700,0x00000F00}, {0x00000700,0x00000B00},
																	{0x00000000,0x00000000}};

unsigned int Num8Select[11][2] = {{0x07000000,0x0000000D}, {0x06000000,0x00000000}, 
																	{0x03000000,0x0000000E}, {0x07000000,0x0000000A}, 
																	{0x06000000,0x00000003}, {0x05000000,0x0000000B},  
																	{0x05000000,0x0000000F}, {0x07000000,0x00000000}, 
																	{0x07000000,0x0000000F}, {0x07000000,0x0000000B},
																	{0x00000000,0x00000000}};

unsigned int Num7Select[11][2] = {{0x00070000,0x000D0000}, {0x00060000,0x00000000}, 
																	{0x00030000,0x000E0000}, {0x00070000,0x000A0000}, 
																	{0x00060000,0x00030000}, {0x00050000,0x000B0000},  
																	{0x00050000,0x000F0000}, {0x00070000,0x00000000}, 
																	{0x00070000,0x000F0000}, {0x00070000,0x000B0000},
																	{0x00000000,0x00000000}};

unsigned int Num6Select[11][2] = {{0x00070000,0x000D0000}, {0x00060000,0x00000000}, 
																	{0x00030000,0x000E0000}, {0x00070000,0x000A0000}, 
																	{0x00060000,0x00030000}, {0x00050000,0x000B0000},  
																	{0x00050000,0x000F0000}, {0x00070000,0x00000000}, 
																	{0x00070000,0x000F0000}, {0x00070000,0x000B0000},
																	{0x00000000,0x00000000}};

unsigned int Num5Select[11][2] = {{0x07000000,0x0000000D}, {0x06000000,0x00000000}, 
																	{0x03000000,0x0000000E}, {0x07000000,0x0000000A}, 
																	{0x06000000,0x00000003}, {0x05000000,0x0000000B},  
																	{0x05000000,0x0000000F}, {0x07000000,0x00000000}, 
																	{0x07000000,0x0000000F}, {0x07000000,0x0000000B},
																	{0x00000000,0x00000000}};
																																																														
unsigned int Num4Select[11][2] = {{0x00000700,0x000D0000}, {0x00000600,0x00000000}, 
																	{0x00000300,0x000E0000}, {0x00000700,0x000A0000}, 
																	{0x00000600,0x00030000}, {0x00000500,0x000B0000},  
																	{0x00000500,0x000F0000}, {0x00000700,0x00000000}, 
																	{0x00000700,0x000F0000}, {0x00000700,0x000B0000},
																	{0x00000000,0x00000000}};	

																																																														
unsigned int Num3Select[11][2] = {{0x00000D00,0x07000000}, {0x00000000,0x06000000}, 
																	{0x00000E00,0x03000000}, {0x00000A00,0x07000000}, 
																	{0x00000300,0x06000000}, {0x00000B00,0x05000000},  
																	{0x00000F00,0x05000000}, {0x00000000,0x07000000}, 
																	{0x00000F00,0x07000000}, {0x00000B00,0x07000000},
																	{0x00000000,0x00000000}};	
																																																														
unsigned int Num2Select[11][2] = {{0x000D0000,0x00000700}, {0x00000000,0x00000600}, 
																	{0x000E0000,0x00000300}, {0x000A0000,0x00000700}, 
																	{0x00030000,0x00000600}, {0x000B0000,0x00000500},  
																	{0x000F0000,0x00000500}, {0x00000000,0x00000700}, 
																	{0x000F0000,0x00000700}, {0x000B0000,0x00000700},
																	{0x00000000,0x00000000}};

unsigned int Num1Select[11][2] = {{0x0D000000,0x00000007}, {0x00000000,0x00000006}, 
																	{0x0E000000,0x00000003}, {0x0A000000,0x00000007}, 
																	{0x03000000,0x00000006}, {0x0B000000,0x00000005},  
																	{0x0F000000,0x00000005}, {0x00000000,0x00000007}, 
																	{0x0F000000,0x00000007}, {0x0B000000,0x00000007},
																	{0x00000000,0x00000000}};	
/********************************************************************************
  ** \brief  LCD配置函数
  ** 
  ** @param  null
  ** 
  ** \retval null
******************************************************************************/ 
void HC32_LCD_Init(void)
{
    stc_lcd_cfg_t LcdInitStruct;
    stc_lcd_segcom_t LcdSegCom;
    Sysctrl_ClkSourceEnable(SysctrlClkRCL,TRUE);            ///< 打开RCL时钟	
    Sysctrl_SetPeripheralGate(SysctrlPeripheralLcd,TRUE);   ///< 打开LCD时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);  ///< 打开GPIO时钟
    Gpio_SetAnalogMode(GpioPortA, GpioPin9);  //COM0
    Gpio_SetAnalogMode(GpioPortA, GpioPin10); //COM1
    Gpio_SetAnalogMode(GpioPortA, GpioPin11); //COM2
    Gpio_SetAnalogMode(GpioPortA, GpioPin12); //COM3   

	//图形
	Gpio_SetAnalogMode(GpioPortA, GpioPin0);  //SEG23 - S0
	Gpio_SetAnalogMode(GpioPortA, GpioPin1);  //SEG22 - S1
	Gpio_SetAnalogMode(GpioPortA, GpioPin2);  //SEG21 - S2
	
	//第一位数字
	Gpio_SetAnalogMode(GpioPortA, GpioPin4); 	//SEG19	- S3		
	Gpio_SetAnalogMode(GpioPortA, GpioPin3);  //SEG20 - S4
	
	//第二位数字
	Gpio_SetAnalogMode(GpioPortA, GpioPin5); 	//SEG18 - S5
	Gpio_SetAnalogMode(GpioPortA, GpioPin6); 	//SEG17 - S6

	//第三位数字
    Gpio_SetAnalogMode(GpioPortB, GpioPin15); //SEG05 - S7		
	Gpio_SetAnalogMode(GpioPortC, GpioPin4);  //SEG15 - S8
	
	//第四位数字
	Gpio_SetAnalogMode(GpioPortC, GpioPin5);  //SEG14 - S9
	Gpio_SetAnalogMode(GpioPortB, GpioPin0);  //SEG13 - S10
	
	//第五位数字
	Gpio_SetAnalogMode(GpioPortB, GpioPin1);  //SEG12 - S11
    Gpio_SetAnalogMode(GpioPortB, GpioPin2); 	//SEG11 - S12
		
	//第六位数字
    Gpio_SetAnalogMode(GpioPortE, GpioPin12); //SEG51	- S13
    Gpio_SetAnalogMode(GpioPortE, GpioPin13); //SEG50 - S14

	//第七位数字
    Gpio_SetAnalogMode(GpioPortE, GpioPin14); //SEG49 - S15
    Gpio_SetAnalogMode(GpioPortE, GpioPin15); //SEG48 - S16
		
	//第八位数字
    Gpio_SetAnalogMode(GpioPortB, GpioPin12); //SEG08 - S17
    Gpio_SetAnalogMode(GpioPortB, GpioPin13); //SEG07 - S18	
		
	//第九位数字
    Gpio_SetAnalogMode(GpioPortD, GpioPin10); //SEG45 - S19			
    Gpio_SetAnalogMode(GpioPortD, GpioPin11); //SEG44 - S20	
    
    Gpio_SetAnalogMode(GpioPortB, GpioPin3);  //VLCDH
    Gpio_SetAnalogMode(GpioPortB, GpioPin4);  //VLCD3
    Gpio_SetAnalogMode(GpioPortB, GpioPin5);  //VLCD2
    Gpio_SetAnalogMode(GpioPortB, GpioPin6);  //VLCD1	
    LcdSegCom.u32Seg0_31 = 0xFF01065F;                             ///< 配置LCD_POEN0寄存器 开启SEG0~SEG7                                     
    LcdSegCom.stc_seg32_51_com0_8_t.seg32_51_com0_8 = 0xFFF0CFFF;  ///< 初始化LCD_POEN1寄存器 全部关闭输出端口
    LcdSegCom.stc_seg32_51_com0_8_t.segcom_bit.Com0_3 = 0;         ///< 使能COM0~COM3
    LcdSegCom.stc_seg32_51_com0_8_t.segcom_bit.Mux = 0;            ///< Mux=0,Seg32_35=0,BSEL=1选择外部电容工作模式，内部电阻断路
    LcdSegCom.stc_seg32_51_com0_8_t.segcom_bit.Seg32_35 = 0;
    LcdSegCom.stc_seg32_51_com0_8_t.segcom_bit.Seg51 = 0;
    LcdSegCom.stc_seg32_51_com0_8_t.segcom_bit.Seg50 = 0;
    LcdSegCom.stc_seg32_51_com0_8_t.segcom_bit.Seg49 = 0;		
    LcdSegCom.stc_seg32_51_com0_8_t.segcom_bit.Seg48 = 0;
    LcdSegCom.stc_seg32_51_com0_8_t.segcom_bit.Seg47 = 1;	
    LcdSegCom.stc_seg32_51_com0_8_t.segcom_bit.Seg46 = 1;				
    LcdSegCom.stc_seg32_51_com0_8_t.segcom_bit.Seg45 = 0;
    LcdSegCom.stc_seg32_51_com0_8_t.segcom_bit.Seg44 = 0;				
    Lcd_SetSegCom(&LcdSegCom);                                     ///< LCD COMSEG端口配置
		
		
    LcdInitStruct.LcdBiasSrc = LcdExtCap;                   	   ///< 电路分压模式,需要外部电路配合
    LcdInitStruct.LcdDuty = LcdDuty4;                              ///< 1/4 duty
    LcdInitStruct.LcdBias = LcdBias3;                              ///< 1/3 BIAS
    LcdInitStruct.LcdCpClk = LcdClk2k;                             ///< 电压泵时钟频率选择2kHz
    LcdInitStruct.LcdScanClk = LcdClk128hz;                        ///< LCD扫描频率选择128Hz
    LcdInitStruct.LcdMode = LcdMode0;                              ///< 模式0
    LcdInitStruct.LcdClkSrc = LcdRCL;                              ///< LCD时钟选择RCL
    LcdInitStruct.LcdEn   = LcdEnable;                             ///< 使能LCD
		
    Lcd_Init(&LcdInitStruct);	
	HC32_LCD_Clear();																							 ///清屏
	
	#ifdef HC32_LOG_SUPPORTS 
		FIOT_LAB_LOG(LOG_INFO,"LCD LcdDuty:%d LcdBias:%d \r\n",LcdInitStruct.LcdDuty, LcdInitStruct.LcdBias);
	#endif	
		
}
void HC32_LCD_Deinit(void)
{
    stc_lcd_cfg_t LcdInitStruct;	
    LcdInitStruct.LcdBiasSrc = LcdExtCap;                   			 ///< 电路分压模式,需要外部电路配合
    LcdInitStruct.LcdDuty = LcdStatic;                              ///< 1/4 duty
    LcdInitStruct.LcdBias = LcdBias3;                              ///< 1/3 BIAS
    LcdInitStruct.LcdCpClk = LcdClk2k;                             ///< 电压泵时钟频率选择2kHz
    LcdInitStruct.LcdScanClk = LcdClk128hz;                        ///< LCD扫描频率选择128Hz
    LcdInitStruct.LcdMode = LcdMode0;                              ///< 模式0
    LcdInitStruct.LcdClkSrc = LcdRCL;                              ///< LCD时钟选择RCL
    LcdInitStruct.LcdEn   = LcdEnable;                             ///< 使能LCD
		
    Lcd_Init(&LcdInitStruct);	
}
/********************************************************************************
  ** \brief  LCD清屏函数
  ** 
  ** @param  null
  ** 
  ** \retval null
******************************************************************************/ 
void HC32_LCD_Clear(void){
	
	Lcd_ClearDisp();
}
/********************************************************************************
  ** \brief  LCD全显函数
  ** 
  ** @param  null
  ** 
  ** \retval null
******************************************************************************/ 
void HC32_LCD_FullDisp(void){
	
	Lcd_FullDisp();
}
/********************************************************************************
  ** \brief  LCD显示数字函数
  ** 
  ** @param  uint8_t u8Row, uint8_t u8NumH, uint8_t u8NumL, uint8_t u8Point
  ** 
  ** \retval ok and error
******************************************************************************/

en_result_t HC32_LCD_ShowNum(uint32_t u32ValueH,uint32_t u32ValueL, uint8_t u8Point, uint8_t u8Level){
	
	uint8_t TenThousand = 0,Thousand = 0,Hundred = 0,Ten = 0,Ge = 0,PointOne = 0,PointTwo = 0,PointThree = 0,PointFour = 0;
	boolean_t Flow = 0,Battery = 0,M3 = 0;
	boolean_t BatteryLevelMax = 0,BatteryLevelMid = 0,BatteryLevelMin = 0;
	switch(u8Point){
		case 0:
			Flow = 1;
			Battery = 1;
			M3 = 1;
			if(u8Level == 3){
				BatteryLevelMax = 1;
				BatteryLevelMid = 1;
				BatteryLevelMin = 1;
			}else if(u8Level == 2){
				BatteryLevelMid = 1;
				BatteryLevelMin = 1; 
			}else if(u8Level == 1){
				BatteryLevelMin = 1; 
			}
			break;
		default:
			break;
	}
	TenThousand = u32ValueH/10000;
	Thousand = u32ValueH%10000/1000;
	Hundred	 = u32ValueH%1000/100;
	Ten = u32ValueH%100/10;
	Ge = u32ValueH%10;
	if(TenThousand == 0){
		TenThousand = 10;
		if(Thousand == 0){
			Thousand = 10;
			if(Hundred == 0){
				Hundred = 10;
				if(Ten == 0)
					Ten = 10;
			}
		}
	}
	if(Flow == 0){
		if(u32ValueL/1000==0){//千
			if(u32ValueL%1000/100==0){//百
				if(u32ValueL%100/10==0){//时
					u32ValueL *= 1000;//个
				}else{
					u32ValueL *= 100;
				}
			}else{
				u32ValueL *= 10;
			}	
		}		
		PointOne = u32ValueL/1000;
		PointTwo = u32ValueL%1000/100;	
		PointThree = u32ValueL%100/10;	
		PointFour = u32ValueL%10;		
		if(PointFour == 0){
			PointFour = 10;
			if(PointThree == 0){
				PointThree = 10;
				if(PointTwo == 0){
					PointTwo = 10;
				}
			}
		}
	}else{
		PointOne = u32ValueL/1000;
		PointTwo = u32ValueL%1000/100;	
		PointThree = u32ValueL%100/10;	
		PointFour = u32ValueL%10;	
	}
	Lcd_WriteRam(5,Num1Select[TenThousand][1] | (Flow << 8) | (Battery << 10) | (M3 << 26) | (BatteryLevelMid << 3));			
	Lcd_WriteRam(4,Num2Select[Thousand][0]|Num2Select[Thousand][1] | Num1Select[TenThousand][0] | (BatteryLevelMin << 11));
	Lcd_WriteRam(3,Num5Select[Ge][1]|Num4Select[Ten][1]|Num4Select[Ten][0]|Num3Select[Hundred][1] | (BatteryLevelMax << 27));		
	Lcd_WriteRam(10,Num6Select[PointOne][0]);  
	Lcd_WriteRam(11,Num6Select[PointOne][1]);  			
	Lcd_WriteRam(8,Num7Select[PointTwo][0]);   
	Lcd_WriteRam(9,Num7Select[PointTwo][1]);   	
	Lcd_WriteRam(1,Num8Select[PointThree][0] | Num3Select[Hundred][0]);  
	Lcd_WriteRam(2,Num8Select[PointThree][1] | Num5Select[Ge][0]|(/*Default Point*/1<<27));  
//	Lcd_WriteRam(12,Num9Select[PointFour][0]);  
//	Lcd_WriteRam(13,Num9Select[PointFour][1]);  
	Lcd_WriteRam(12,0x0000);  
	Lcd_WriteRam(13,0x0000); 	
	return Ok;
}












