#ifndef __HC32_BEEP_H__
#define __HC32_BEEP_H__
#include "gpio.h"
/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
#define ON 		1
#define OFF 	0
#define USER_BEEP_PORT        GpioPortE
#define USER_BEEP_PIN         GpioPin5
#define HC32_BEEP_Low()           {Gpio_WriteOutputIO(USER_BEEP_PORT,USER_BEEP_PIN,0);}
#define HC32_BEEP_Hight()         {Gpio_WriteOutputIO(USER_BEEP_PORT,USER_BEEP_PIN,1);}	
void HC32_BEEP_Init(void);
void HC32_BEEP_Deinit(void);
void BEEP_Switch(uint8_t switch_t);	
void BEEP_OverTurn(int32_t delay_t);	
#ifdef __cplusplus
}
#endif

#endif /* __HC32_BEEP_H__ */


