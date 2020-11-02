#ifndef __HC32_CRC_H__
#define __HC32_CRC_H__
#include "crc.h"
/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
	
	
uint16_t HC32_CRC16_Get16(uint16_t *CrcData, uint16_t len,uint8_t row);
	
	
	
#ifdef __cplusplus
}
#endif

#endif /* __HC32_CRC_H__ */


