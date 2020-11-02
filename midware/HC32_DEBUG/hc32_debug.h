#ifndef __HC32_DEBUG_H__
#define __HC32_DEBUG_H__
#include "gpio.h"
#include "lpuart.h"
#include "uart.h"
#include "flash.h"
#include "stdio.h"	
/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

//#define HC32_LPUART_DEBUG_SUPPORTS	

#define FIOT_LAB_LOG_PORT         LPUART0
#define FIOT_LAB_LOG_BUF_SIZE     (1024)	
	
extern uint8_t u8DeviceLogData;
	
typedef enum
{
    LOG_DEBUG = 0,
    LOG_INFO,
    LOG_WARNING,
    LOG_ERROR,
    LOG_FATAL,
    LOG_MAX
} fiot_lab_log_e;

char * fiot_lab_log_format_prefix(fiot_lab_log_e level);

void fiot_lab_log_hexdump(uint8_t *p_data, uint16_t length);

fiot_lab_log_e fiot_lab_get_log_level(void);

uint8_t FIOT_LAB_LOG_PARSER(void);

#define FIOT_LAB_LOG(level, fmt, ...) \
		do \
		{ \
			if (level >= fiot_lab_get_log_level()) \
			{ \
				char * log_buf = fiot_lab_log_format_prefix(level); \
				snprintf(log_buf + strlen(log_buf), FIOT_LAB_LOG_BUF_SIZE, fmt, ##__VA_ARGS__); \
				UartDebugSend((unsigned char *)(log_buf), strlen((const char *)(log_buf)));\
			} \
		} while (0)
	
void HC32_DEBUG_Init(void);
void HC32_DEBUG_Deinit(void);		
void UartDebugSend(uint8_t *pBuf, uint32_t gLen);
		
#ifdef __cplusplus
}
#endif

#endif /* __HC32_DEBUG_H__ */


