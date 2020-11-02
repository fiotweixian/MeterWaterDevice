#ifndef __HC32_LED_H__
#define __HC32_LED_H__
#include "gpio.h"
/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
#define ON 		1
#define OFF 	0
#define USER_LED_PORT        GpioPortD
#define USER_LED_PIN         GpioPin14
void HC32_LED_Init(void);
void LED_Switch(uint8_t switch_t);	
void LED_OverTurn(int32_t delay_t);	
#ifdef __cplusplus
}
#endif

#endif /* __HC32_LED_H__ */


