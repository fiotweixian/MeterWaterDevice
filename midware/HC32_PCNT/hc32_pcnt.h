#ifndef __HC32_PCNT_H__
#define __HC32_PCNT_H__
#include "ddl.h"
#include "gpio.h"
#include "pcnt.h"
#include "hc32_led.h"
/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

#define EVERYM3WATER  0.001	
extern unsigned int gPcntWaterAmount;	
	
extern unsigned char u8DeviceOverCurrent;	
//#define HC32_PCNTDEBUG_SUPPORTS
typedef struct {
	uint32_t u32WaterH;
	uint32_t u32WaterL;	
} PCNT_NUM_T;

typedef struct {
	uint32_t u32WaterLastH;
	uint32_t u32WaterLastL;	
} PCNT_LAST_T;

typedef struct {
	uint32_t u32OverCurrentThreH;
	uint32_t u32OverCurrentThreL;	
} OVERCURRENT_THRE_T;//OVER CURRENT THRESHOLD 

typedef struct {
	uint32_t u32ConCurrentThreH;
	uint32_t u32ConCurrentThreL;	
} CONCURRENT_THRE_T;//Con CURRENT THRESHOLD 


void HC32_PCNT_Init(void);
void ReedPipe_Disable(void);
void ReedPipe_Enable(void);
unsigned int WaterFlowStatistics(void);
uint32_t WaterReturnInt(void);
uint32_t WaterReturnFloat(void);

uint32_t WaterFlowFloat(void);
void WaterPcntClear(uint32_t Ton_t,uint32_t Pulse_t);
void HC32_PCNT_DebOut(void);
boolean_t JudgeNowOverCurrent(void);
void StoreLastWaterNum(uint32_t WaterLastH,uint32_t WaterLastL);
void SetOverCurrentValue(uint32_t WaterThreH,uint32_t WaterThreL);
void SetConCurrentValue(uint32_t WaterThreH,uint32_t WaterThreL);
extern unsigned char u8DevicePcnt;
#ifdef __cplusplus
}
#endif

#endif /* __HC32_PCNT_H__ */


