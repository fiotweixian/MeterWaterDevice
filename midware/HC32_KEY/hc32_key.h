#ifndef __HC32_KEY_H__
#define __HC32_KEY_H__
#include "gpio.h"
/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

extern int Key_Test_Value;	
extern uint8_t u8DeviceWarning, u8DeviceInfomation;
	
///< USER KEY
#define KEY0_USER_PORT       GpioPortC
#define KEY0_USER_PIN        GpioPin7
  
void HC32_KEY_Init(void);
boolean_t HC32_Read_Key(void);
#ifdef __cplusplus
}
#endif

#endif /* __HC32_LED_H__ */


