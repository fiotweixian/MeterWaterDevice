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
  ** \brief  CRCת������
  ** 
  ** @param  (��У������, ���ݳ���, ����λ��:8,16,32)
  ** 
  ** \retval ����У����
******************************************************************************/ 
uint16_t HC32_CRC16_Get16(uint16_t *CrcData, uint16_t len,uint8_t row)
{
    uint16_t	u16CrcResult = 0;
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralCrc, TRUE);       ///< ʹ��CRCʱ��	
	
	if(row == 8){
		/* 8λλ�� */
		u16CrcResult =  CRC16_Get16((uint16_t *)CrcData, len);  ///< ����CRC16����
	}else if(row == 16){
		/* 16λλ�� */
		u16CrcResult =  CRC16_Get16((uint16_t *)CrcData, len/2);  ///< ����CRC16����
	}else if(row == 32){
		/* 32λλ�� */
		u16CrcResult =  CRC16_Get16((uint16_t *)CrcData, len/4);  ///< ����CRC16����
	}	
    Sysctrl_SetPeripheralGate(SysctrlPeripheralCrc, FALSE);       ///< ʧ��CRCʱ��	
	
	return u16CrcResult;
}

