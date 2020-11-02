#ifndef __HC32_VALVE_H__
#define __HC32_VALVE_H__
#include "gpio.h"
#include "adt.h"
#include "timer3.h"
/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
	
#define USER_SLEEP_PORT        GpioPortE
#define USER_SLEEP_PIN         GpioPin11
#define HC32_SLEEP_Low()           {Gpio_WriteOutputIO(USER_SLEEP_PORT,USER_SLEEP_PIN,0);}
#define HC32_SLEEP_Hight()         {Gpio_WriteOutputIO(USER_SLEEP_PORT,USER_SLEEP_PIN,1);}		
//void HC32_VALVE_Init(uint16_t u16Period, uint16_t u16CHA_PWMDuty, uint16_t u16CHB_PWMDuty);
void HC32_Valve_Open(void);
void HC32_Valve_Close(void);
void HC32_VALVE_Init(uint16_t u16Period, uint16_t u16CHxACompare, uint16_t u16CHxBCompare);
void HC32_VALVE_Deinit(void);
void HC32_VALVE_GPIO(void);	
#ifdef __cplusplus
}
#endif

#endif /* __HC32_VALVE_H__ */


