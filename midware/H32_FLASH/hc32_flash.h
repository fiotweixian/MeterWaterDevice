#ifndef __HC32_FLASH_H__
#define __HC32_FLASH_H__
#include "gpio.h"
#include "flash.h"
#include "ddl.h"
/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
//用户根据自己的需要设置
#define HC32_FLASH_SIZE 128 	 					//所选HC32的FLASH容量大小(单位为K)
#define HC32_FLASH_WREN 1              //使能FLASH写入(0，不是能;1，使能)
//FLASH起始地址
#define HC32_FLASH_BASE 0x3ff00 		//HC32 FLASH的起始地址
	
	
uint8_t HC32_FLASH_Init(void);
uint8_t HC32FLASH_ReadHalfWord(uint8_t faddr);	
void HC32FLASH_Write_NoCheck(uint32_t WriteAddr,uint16_t *pBuffer,uint16_t NumToWrite);
void HC32FLASH_Read(uint32_t ReadAddr,uint16_t *pBuffer,uint16_t NumToRead);
void Test_Write(uint32_t WriteAddr,uint16_t WriteData);
void HC32FLASH_Write(uint32_t WriteAddr,uint16_t *pBuffer,uint16_t NumToWrite);
	
#ifdef __cplusplus
}
#endif

#endif /* __HC32_FLASH_H__ */


