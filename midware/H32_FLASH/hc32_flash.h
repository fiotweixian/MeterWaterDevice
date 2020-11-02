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
//�û������Լ�����Ҫ����
#define HC32_FLASH_SIZE 128 	 					//��ѡHC32��FLASH������С(��λΪK)
#define HC32_FLASH_WREN 1              //ʹ��FLASHд��(0��������;1��ʹ��)
//FLASH��ʼ��ַ
#define HC32_FLASH_BASE 0x3ff00 		//HC32 FLASH����ʼ��ַ
	
	
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


