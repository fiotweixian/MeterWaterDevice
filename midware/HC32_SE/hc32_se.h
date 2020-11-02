#ifndef __HC32_SE_H__
#define __HC32_SE_H__
#include "gpio.h"
#include "ddl.h"
#include "uart.h"
#include "lpuart.h"
#include "pca.h"
/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

 #define   SE_REST_Low()           {Gpio_WriteOutputIO(GpioPortD,GpioPin12,0);}
#define   SE_REST_Hight()         {Gpio_WriteOutputIO(GpioPortD,GpioPin12,1);}
 
#define   SE_CLK_ON()             { M0P_PCA->CCON = 1;}
#define   SE_CLK_OFF()            { M0P_PCA->CCON = 0;}


en_result_t HC32_S7816_Recv(uint8_t *str,uint8_t len,uint32_t timeout);
void HC32_S7816_Send(uint8_t *str,uint8_t len);
void S7816_Config(void);
void HC32_SE_Init(void);
void HC32_SE_Deinit(void);
void HC32_SE_Send(uint8_t *str,uint8_t len);
en_result_t HC32_SE_Recv(uint8_t *str,uint8_t len,uint32_t timeout);
uint8_t HC32_SE_SM4WriteData(uint8_t u8SencSize, uint8_t u8RecSize, uint8_t *u8RecvBuf , uint8_t * u8ScdData);
en_result_t HC32_SE_ATR( uint8_t *str, uint8_t len, uint32_t timeout );
#ifdef __cplusplus
}
#endif

#endif /* __HC32_SE_H__ */


