/******************************************************************************
 ** \file hc32_crc.c
 **
 **  Copyright Fuzhou Internet of Things Open Lab.
 **
 **	 Author :Fiot-lab
 **
 **  DESCRIPTION: This file contains CRC Configure and Check.
 **
 ******************************************************************************/
/******************************************************************************
* Include files
******************************************************************************/
#include "hc32_crc.h"

/*******************************************************************************
 * global variable - (global)  and static variable - (static) 
 ******************************************************************************/

/********************************************************************************
  ** \brief  CRC转换函数
  ** 
  ** @param  (带校验数据, 数据长度, 编码位宽:8,16,32)
  ** 
  ** \retval 返回校验结果
******************************************************************************/ 
uint16_t HC32_CRC16_Get16(uint16_t *CrcData, uint16_t len,uint8_t row)
{
    uint16_t	u16CrcResult = 0;
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralCrc, TRUE);       ///< 使能CRC时钟	
	
	if(row == 8){
		/* 8位位宽 */
		u16CrcResult =  CRC16_Get16((uint16_t *)CrcData, len);  ///< 产生CRC16编码
	}else if(row == 16){
		/* 16位位宽 */
		u16CrcResult =  CRC16_Get16((uint16_t *)CrcData, len/2);  ///< 产生CRC16编码
	}else if(row == 32){
		/* 32位位宽 */
		u16CrcResult =  CRC16_Get16((uint16_t *)CrcData, len/4);  ///< 产生CRC16编码
	}	
    Sysctrl_SetPeripheralGate(SysctrlPeripheralCrc, FALSE);       ///< 失能CRC时钟	
	
	return u16CrcResult;
}

