/******************************************************************************
 ** \file hc32_se.c
 **
 **  Copyright Fuzhou Internet of Things Open Lab.
 **
 **	 Author :Fiot-lab
 **
 **  DESCRIPTION: This file contains SE Configure.
 **
 ******************************************************************************/
/******************************************************************************
* Include files
******************************************************************************/
#include "hc32_se.h"
#include "hc32_debug.h"
/*******************************************************************************
 * global variable - (global)  and static variable - (static) 
 ******************************************************************************/

/********************************************************************************
  ** \brief  SE配置函数
  ** 
  ** @param  null
  ** 
  ** \retval null
******************************************************************************/ 
void HC32_SE_Init(void)
{
	uint8_t  S7816Buff[64]={0};

//	uint8_t rec[64] = {0};
	
  stc_gpio_cfg_t stcGpioCfg;
  stc_pcacfg_t  PcaInitStruct;	

	DDL_ZERO_STRUCT(stcGpioCfg);
	
	Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE); 
  Sysctrl_SetPeripheralGate(SysctrlPeripheralPca, TRUE);
	
	stcGpioCfg.enDir = GpioDirOut;
	Gpio_Init(GpioPortD,GpioPin12,&stcGpioCfg);	 //RES
	
	SE_REST_Low();      
	
	stcGpioCfg.enDrv  = GpioDrvH;
	stcGpioCfg.enDir  = GpioDirOut;
	Gpio_Init(GpioPortA, GpioPin7, &stcGpioCfg);
	Gpio_SetAfMode(GpioPortA, GpioPin7, GpioAf2);//clk

	PcaInitStruct.pca_clksrc = PcaPclkdiv2;
	PcaInitStruct.pca_cidl   = FALSE;
	PcaInitStruct.pca_ecom   = PcaEcomEnable;       
	PcaInitStruct.pca_capp   = PcaCappDisable;      
	PcaInitStruct.pca_capn   = PcaCapnDisable;      
	PcaInitStruct.pca_mat    = PcaMatEnable;        
	PcaInitStruct.pca_tog    = PcaTogEnable;        
	PcaInitStruct.pca_pwm    = PcaPwm8bitDisable;    
	PcaInitStruct.pca_epwm   = PcaEpwmEnable; 
	PcaInitStruct.pca_ccapl  = 2;
	PcaInitStruct.pca_ccaph  = 0;	    
	PcaInitStruct.pca_ccap   = 2;
	PcaInitStruct.pca_carr   = 3;           
	Pca_M1Init(&PcaInitStruct);	
	Pca_SetCnt(0);
	Pca_StartPca(TRUE);
	
	M0P_SYSCTRL->PERI_CLKEN0_f.LPUART0 = 1;	
	M0P_GPIO->PDPU_f.PD08 = 1;
	M0P_GPIO->PDDIR_f.PD08 = 1;  //TXD
	M0P_GPIO->PD08_SEL = 1;

	M0P_LPUART0->ICR = 0x00;
	M0P_LPUART0->SCON = 0x00;
	M0P_LPUART0->SCON_f.HDSEL = 1;  //单线
	M0P_LPUART0->SCON_f.OVER = 0;  ///*16分频 *///
	M0P_LPUART0->SCON_f.SM = 3;  ///*mode 3*///
	M0P_LPUART0->SCON_f.B8CONT = 1;  ///*偶校验*///
	M0P_LPUART0->SCON_f.STOPBIT = 2;  //2bit
	M0P_LPUART0->SCON_f.REN = 1;
	M0P_LPUART0->SCNT = 186;        ///*以5.375kbps @16M*///
	
	delay1ms( 4 ); //至少512个7816CLK
	SE_REST_Hight();
		
	if( ErrorTimeout  ==  HC32_SE_ATR( S7816Buff, 16, 0xFFFF ) ) //ATR
	{
		FIOT_LAB_LOG(LOG_DEBUG,"Receive ATR\r\n");
	}		
	fiot_lab_log_hexdump(S7816Buff,16);
	
	
//	memset(S7816Buff, 0 , sizeof(S7816Buff));
//	
//	S7816Buff[0] = 0x80;//Send Header
//	S7816Buff[1] = 0xF0;
//	S7816Buff[2] = 0x00;
//	S7816Buff[3] = 0x01;
//	S7816Buff[4] = 0x09;//data lenth
//	HC32_S7816_Send(S7816Buff,5);
//	if( ErrorTimeout  ==  HC32_S7816_Recv(rec, 16, 0xFFFF ) ) //Rec INS
//	{

//	}		
//	fiot_lab_log_hexdump(rec,16);
//	memset(rec, 0 ,sizeof(rec));
//	
//	S7816Buff[5] = 0x05;//Send Data
//	S7816Buff[6] = 0x12;
//	S7816Buff[7] = 0x34;	
//	S7816Buff[8] = 0x56;
//	S7816Buff[9] = 0x78;
//	S7816Buff[10] = 0x12;
//	S7816Buff[11] = 0x34;	
//	S7816Buff[12] = 0x56;
//	S7816Buff[13] = 0x78;		
//	HC32_S7816_Send(&S7816Buff[5],9);
//	if( ErrorTimeout  ==  HC32_S7816_Recv(rec, 16, 0xFFFF ) ) //Rec 61 Licc
//	{

//	}
//	fiot_lab_log_hexdump(rec,16);
//	memset(rec, 0 ,sizeof(rec));
//	
//	S7816Buff[14] = 0x00;//Send Response
//	S7816Buff[15] = 0xC0;
//	S7816Buff[16] = 0x00;	
//	S7816Buff[17] = 0x00;
//	S7816Buff[18] = 0x20;
//	HC32_S7816_Send(&S7816Buff[14],5);
//	if( ErrorTimeout  ==  HC32_S7816_Recv(rec, 64, 0xFFFF ) ) //Rec Data
//	{

//	}
//	fiot_lab_log_hexdump(rec,64);
//		S7816Buff[0] = 0x05;//Send Data
//		S7816Buff[1] = 0x12;
//		S7816Buff[2] = 0x34;	
//		S7816Buff[3] = 0x56;
//		S7816Buff[4] = 0x78;
//		S7816Buff[5] = 0x12;
//		S7816Buff[6] = 0x34;	
//		S7816Buff[7] = 0x56;
//		S7816Buff[8] = 0x78;
//		HC32_SE_SM4WriteData(9,32,&S7816Buff[10],S7816Buff);
//		fiot_lab_log_hexdump(S7816Buff,64);		
}

void HC32_SE_Deinit(void)
{
  Sysctrl_SetPeripheralGate(SysctrlPeripheralPca, FALSE);
}
/********************************************************************************
  ** \brief  Se Send Data buf
  ** 
  ** @param  Data Send Buf,Data length
  ** 
  ** \retval NULL
******************************************************************************/ 
void HC32_S7816_Send( uint8_t *str, uint8_t len )
{
    uint32_t i;
    for( i = 0; i < len; i++ )
    {
        M0P_LPUART0->SBUF = *( str + i );
        while( TRUE != M0P_LPUART0->ISR_f.TC )
        {
            ;
        }
        M0P_LPUART0->ICR = 0;
    }
}
/********************************************************************************
  ** \brief  Se Receive Data
  ** 
  ** @param  Data Receive Buf,Data length,Data timeout
  ** 
  ** \retval return result
******************************************************************************/ 
en_result_t HC32_S7816_Recv( uint8_t *str, uint8_t len, uint32_t timeout )
{
    uint8_t i;
    uint32_t temp;
    for( i = 0; i < len; i++ )
    {
        temp = timeout;
        while( temp > 0 )
        {
            if( TRUE != M0P_LPUART0->ISR_f.RC )
            {
                temp--;
                if( temp < 1 )
                {
                    return  ErrorTimeout;
                }
            }
            else
            {
                break;
            }
        }
        *( str + i ) = M0P_LPUART0->SBUF;
        M0P_LPUART0->ICR = 0;
    }
    return  Ok;
}
/********************************************************************************
  ** \brief  Se Receive ATR
  ** 
  ** @param  Data Receive Buf,Data length,Data timeout
  ** 
  ** \retval return result
******************************************************************************/ 
en_result_t HC32_SE_ATR( uint8_t *str, uint8_t len, uint32_t timeout ){
	
	return HC32_S7816_Recv(str, len, timeout );
}
/********************************************************************************
  ** \brief  Se Send Data Buf
  ** 
  ** @param  Data Receive Buf,Data length,Data timeout
  ** 
  ** \retval return result
******************************************************************************/ 
uint8_t HC32_SE_SM4WriteData(uint8_t u8SencSize, uint8_t u8RecSize, uint8_t *u8RecvBuf , uint8_t * u8ScdData)
{
    uint8_t temp[5] = {0};
    uint8_t Header[5] = {0x80,0xf0,0x00,0x01,0};
    uint8_t Response[5] = {0x00,0xC0,0x00,0x00,0x20};		
		Header[4] = u8SencSize;
	  if(u8SencSize <= 5)
	  {
	    HC32_S7816_Send( u8ScdData, 5);
	  }
	  else
		{
			HC32_S7816_Send( Header, 5 ); 
      if( ErrorTimeout  ==  HC32_S7816_Recv( &temp[0], 1, 0xFFFFFFFF ) )//Rec INS
      {			
             //return FALSE;
      }			
			fiot_lab_log_hexdump(temp,16);
      if(temp[0] != u8ScdData[1])
      {
					   //return FALSE;
		  }				
			HC32_S7816_Send( u8ScdData, u8SencSize ); 
      if( ErrorTimeout  ==  HC32_S7816_Recv( &temp[2], 2, 0xFFFFFFFF ) )////Rec 61 Licc
      {			
             //return FALSE;
      }
			fiot_lab_log_hexdump(temp,16);				
      if(temp[2] != u8ScdData[1])
      {
					   //return FALSE;
		  }				
			HC32_S7816_Send( Response, 5 ); 
      if( ErrorTimeout  ==  HC32_S7816_Recv( u8RecvBuf, u8RecSize, 0xFFFFFFFF ) )////Rec Data
      {			
             //return FALSE;
      }			
		}
    return TRUE;	
}
/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/












