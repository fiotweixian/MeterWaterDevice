#ifndef __HC32_BATTERY_H__
#define __HC32_BATTERY_H__
#include "gpio.h"
#include "adc.h"
#include "bgr.h"
#include "hc32_debug.h"
/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
	
#define BATTERY_AVERAGE_TIME		20
	
#define USER_BATTERY_PORT        	GpioPortA
#define USER_BATTERY_PIN         	GpioPin15
#define HC32_BATTERY_Low()          {Gpio_WriteOutputIO(USER_BATTERY_PORT,USER_BATTERY_PIN,0);}
#define HC32_BATTERY_Hight()        {Gpio_WriteOutputIO(USER_BATTERY_PORT,USER_BATTERY_PIN,1);}	

extern char gBc35UserBatteryValueLevel;

void HC32_BATTERY_Init(void);
void HC32_BATTERY_DeInit(void);
void HC32_BATTERY_GPIO(void);
void HC32_BATTERY_Enable(void);
void HC32_BATTERY_Disable(void);
float BatteryObtain(void);	
uint8_t BatteryLevel(void);


#ifdef __cplusplus
}
#endif

#endif /* __HC32_BATTERY_H__ */


