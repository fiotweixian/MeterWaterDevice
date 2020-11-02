#ifndef __HC32_RTC_H__
#define __HC32_RTC_H__
#include "rtc.h"
/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
typedef struct _RTC_Alarm_T {
	uint8_t Hour;
	uint8_t Min;
	uint8_t Sec;
} RTC_Alarm_T;
	
void HC32_RTC_Init(void);
extern uint8_t u8DeviceAlarm;
extern uint8_t u8DeviceWdtdog;
extern uint8_t u8RecordPcntHalfHour;
extern boolean_t u8DeviceDeepSleep;
extern boolean_t u8DevicePriod;
#ifdef __cplusplus
}
#endif

#endif /* __HC32_RTC_H__ */


