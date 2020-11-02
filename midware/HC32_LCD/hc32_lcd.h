#ifndef __HC32_LCD_H__
#define __HC32_LCD_H__
#include "gpio.h"
#include "lcd.h"
/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

#define LCDRAM0 		0
#define LCDRAM1 		1	
#define LCDRAM2 		2
#define LCDRAM3 		3		
#define LCDRAM4 		4
#define LCDRAM5 		5	
#define LCDRAM6 		6
#define LCDRAM7 		7	
#define LCDRAM8 		8
#define LCDRAM9 		9	
#define LCDRAMA 		10
#define LCDRAMB 		11		
#define LCDRAMC 		12
#define LCDRAMD 		13

/*
Flow Lock Battery Signal Temp Kpa M3 Lh Fan	
	1 	1 		1 			1 		1 	 1 	1 	1	 1
*/	
#define LCDImageFlow 								0
#define LCDImageSignal	 						3		
#define LCDImageTemperature			 		4
#define LCDImageKpa					 				5	
#define LCDImageLh							 		7	
#define LCDImageFan							 		8
	
void HC32_LCD_Init(void);
void HC32_LCD_Deinit(void);
void HC32_LCD_Clear(void);
void HC32_LCD_FullDisp(void);
en_result_t HC32_LCD_ShowNum(uint32_t u32ValueH, uint32_t u32ValueL, uint8_t u8Point, uint8_t u8Level);
	
#ifdef __cplusplus
}
#endif

#endif /* __HC32_LCD_H__ */


