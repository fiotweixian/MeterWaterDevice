/******************************************************************************
 ** \file hc32_lptimer.c
 **
 **  Copyright Fuzhou Internet of Things Open Lab.
 **
 **	 Author :Fiot-lab
 **
 **  DESCRIPTION: This file contains Lptim Configure and Lptim Control .
 **
 ******************************************************************************/
/******************************************************************************
* Include files
******************************************************************************/
#include "hc32_lptimer.h"
#include "hc32_led.h"
#include "hc32_debug.h"

/*******************************************************************************
 * global variable - (global)  and static variable - (static) 
 ******************************************************************************/
int gLptimSecRecord = 0;

/********************************************************************************
  ** \brief  Lptimer��ʼ��
  ** 
  ** @param  null
  ** 
  ** \retval null
******************************************************************************/ 
void HC32_LPTIMER_Init(void)
{
    stc_lptim_cfg_t    stcLptCfg;
	
    DDL_ZERO_STRUCT(stcLptCfg);

    ///< LPTIM0 ʱ��
    Sysctrl_SetPeripheralGate(SysctrlPeripheralLpTim0, TRUE);
    //ʹ��RCL
    Sysctrl_ClkSourceEnable(SysctrlClkRCL, TRUE);
	
    stcLptCfg.enGate   = LptimGateLow;
    stcLptCfg.enGatep  = LptimGatePLow;
    stcLptCfg.enTcksel = LptimPclk;
    stcLptCfg.enTogen  = LptimTogEnLow;
    stcLptCfg.enCt     = LptimTimerFun;         //��������
    stcLptCfg.enMd     = LptimMode2;            //����ģʽ2���Զ���װ��16λ������/��ʱ��
    stcLptCfg.u16Arr   = 0;                 //Ԥװ�ؼĴ���ֵ,�����ֵ
		stcLptCfg.enPrs		 = LptimPrsDiv256;
	
    Lptim_Init(M0P_LPTIMER0, &stcLptCfg);
    
    Lptim_ClrItStatus(M0P_LPTIMER0);            //����жϱ�־
    Lptim_ConfIt(M0P_LPTIMER0, TRUE);           //����LPTIMER�ж�   
    EnableNvic(LPTIM_0_1_IRQn, IrqLevel3, TRUE); 
		
		#ifdef HC32_LOG_SUPPORTS 
			FIOT_LAB_LOG(LOG_INFO,"Lptim Configure Succeed!\r\n");
		#endif	

}
/********************************************************************************
  ** \brief  LPT��ʼ����
  ** 
  ** @param  null
  ** 
  ** \retval null
******************************************************************************/ 
void LpTimer_Enable(void)
{
    Lptim_Cmd(M0P_LPTIMER0, TRUE);    //LPT ����
}
/********************************************************************************
  ** \brief  LPT�ر�����
  ** 
  ** @param  null
  ** 
  ** \retval null
******************************************************************************/ 
void LpTimer_Disable(void)
{
    Lptim_Cmd(M0P_LPTIMER0, FALSE);    //LPT ֹͣ
}
/********************************************************************************
  ** \brief  Lptimer�жϷ�����
  ** 
  ** @param  null
  ** 
  ** \retval null
******************************************************************************/ 
void LpTim0_IRQHandler(void)
{
    if (TRUE == Lptim_GetItStatus(M0P_LPTIMER0))
    {
			gLptimSecRecord++;
			Lptim_ClrItStatus(M0P_LPTIMER0);//LPTIMER0�����־λ    
    }
}
/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/










