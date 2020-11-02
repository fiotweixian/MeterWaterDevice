#ifndef __HC32_EEPROM_H__
#define __HC32_EEPROM_H__
#include "gpio.h"
#include "rtc.h"
/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
//#define HC32_IIC_HARDWARE_SUPPORTS
#ifdef HC32_IIC_HARDWARE_SUPPORTS	
	
#define I2C_SLAVEADDR 0xA0
#define I2C_BAUD100K 	100
#define I2C_BAUD400K 	400
	
void HC32_EEPROM_Init(uint8_t Adddress,uint32_t Baund);
en_result_t HC32_EEPROM_Write(uint8_t u8Addr,uint8_t *pu8Data,uint32_t u32Len);
en_result_t HC32_EEPROM_Read(uint8_t u8Addr,uint8_t *pu8Data,uint32_t u32Len);
void Test_IIc(void);
#else 
	
extern uint8_t WaterRecordDay;
	
extern uint8_t WaterRecordindex;	

	
#define ADDR_RTC_RECORD   			0x01 		// RTC time mark
#define	ADDR_IPSER_RECORD 			0x02		// IP Address Store 0x02.0x03.0x04.0x05:0x06 0x07
#define ADDR_WNUM_RECORD  			0x08		// 0x08 0x09 0x0A 0x0B 0X0A 0X0B 0X0C/WaterNum Max < 99 99 99 99
#define ADDR_IMWARN_RECORD 			0x0E 		// Immediately Warn Default:FFH 
#define ADDR_OCURRENTTH_RECORD 		0x0F		// 0x0F 0x10 /Set OverCurrentThreshold Value Max < 99.99
#define ADDR_OCURRENTTIME_RECORD 	0x11		// 0x11 0X12 /OverCurrent Time Record
#define ADDR_CCURRENTTH_RECORD 		0x13		// 0x13 0x14 /ConCurrent Threshold
#define ADDR_CCURRENTTIME_RECORD 	0x15		// 0X15 0X16 /ConCurrent Time Record
#define	ADDR_DATAFREQUENCY_RECORD	0x17		// 0x17 0x18 /Send Data Frequency

#define ADDR_StorePcnt_START 0X30
	
#define ADDR_ONEDAY_START 0X40
#define ADDR_ONEWATERNUM_START 0X50
	
#define ADDR_SECONDDAY_START 0X1D0	
#define ADDR_SECWATERNUM_START 0X1E0

#define ADDR_THREEDAY_START 0X360	
#define ADDR_THREWATERNUM_START 0X370

//IO方向设置
#define USER_IIC_PORT GpioPortB
#define USER_SCL_PIN 	GpioPin10
#define USER_SDA_PIN 	GpioPin11
#define HC32_SCL_SDA_Low()           {Gpio_WriteOutputIO(USER_IIC_PORT,USER_SCL_PIN,0);Gpio_WriteOutputIO(USER_IIC_PORT,USER_SDA_PIN,0);}
#define HC32_SCL_SDA_Hight()         {Gpio_WriteOutputIO(USER_IIC_PORT,USER_SCL_PIN,1);Gpio_WriteOutputIO(USER_IIC_PORT,USER_SDA_PIN,1);}	
#define USER_WP_PORT GpioPortF
#define USER_WP_PIN 	GpioPin6
#define HC32_WP_Low()           {Gpio_WriteOutputIO(USER_WP_PORT,USER_WP_PIN,0);}
#define HC32_WP_Hight()         {Gpio_WriteOutputIO(USER_WP_PORT,USER_WP_PIN,1);}		
#define AT24C01		127
#define AT24C02		255
#define AT24C04		511
#define AT24C08		1023
#define AT24C16		2047
#define AT24C32		4095
#define AT24C64	  8191
#define AT24C128	16383
#define AT24C256	32767  
//定义EE_TYPE为AT24C02
#define EE_TYPE AT24C02
uint8_t AT24CXX_ReadOneByte(uint16_t ReadAddr);							//指定地址读取一个字节
void AT24CXX_WriteOneByte(uint16_t WriteAddr,uint8_t DataToWrite);		//指定地址写入一个字节
void AT24CXX_WriteLenByte(uint16_t WriteAddr,uint32_t DataToWrite,uint8_t Len);//指定地址开始写入指定长度的数据
uint32_t AT24CXX_ReadLenByte(uint16_t ReadAddr,uint8_t Len);					//指定地址开始读取指定长度数据
void AT24CXX_Write(uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite);	//从指定地址开始写入指定长度的数据
void AT24CXX_Read(uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead);   	//从指定地址开始读出指定长度的数据
uint8_t AT24CXX_Check(void);  //检查器件	
void AT24CXX_Init(void);
void AT24CXX_Deinit(void);
void HC32_StoreWaterNum(uint32_t StoreWater_H,uint32_t StoreWater_L);
#endif
#ifdef __cplusplus
}
#endif

#endif /* __HC32_EEPROM_H__ */


