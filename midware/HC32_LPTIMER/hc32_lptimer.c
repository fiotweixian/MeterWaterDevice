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
  ** \brief  Lptimer初始化
  ** 
  ** @param  null
  ** 
  ** \retval null
******************************************************************************/ 
void HC32_LPTIMER_Init(void)
{
    stc_lptim_cfg_t    stcLptCfg;
	
    DDL_ZERO_STRUCT(stcLptCfg);

    ///< LPTIM0 时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralLpTim0, TRUE);
    //使能RCL
    Sysctrl_ClkSourceEnable(SysctrlClkRCL, TRUE);
	
    stcLptCfg.enGate   = LptimGateLow;
    stcLptCfg.enGatep  = LptimGatePLow;
    stcLptCfg.enTcksel = LptimPclk;
    stcLptCfg.enTogen  = LptimTogEnLow;
    stcLptCfg.enCt     = LptimTimerFun;         //计数功能
    stcLptCfg.enMd     = LptimMode2;            //工作模式2：自动重装载16位计数器/定时器
    stcLptCfg.u16Arr   = 0;                 //预装载寄存器值,计算初值
		stcLptCfg.enPrs		 = LptimPrsDiv256;
	
    Lptim_Init(M0P_LPTIMER0, &stcLptCfg);
    
    Lptim_ClrItStatus(M0P_LPTIMER0);            //清除中断标志
    Lptim_ConfIt(M0P_LPTIMER0, TRUE);           //允许LPTIMER中断   
    EnableNvic(LPTIM_0_1_IRQn, IrqLevel3, TRUE); 
		
		#ifdef HC32_LOG_SUPPORTS 
			FIOT_LAB_LOG(LOG_INFO,"Lptim Configure Succeed!\r\n");
		#endif	

}
/********************************************************************************
  ** \brief  LPT开始运行
  ** 
  ** @param  null
  ** 
  ** \retval null
******************************************************************************/ 
void LpTimer_Enable(void)
{
    Lptim_Cmd(M0P_LPTIMER0, TRUE);    //LPT 运行
}
/********************************************************************************
  ** \brief  LPT关闭运行
  ** 
  ** @param  null
  ** 
  ** \retval null
******************************************************************************/ 
void LpTimer_Disable(void)
{
    Lptim_Cmd(M0P_LPTIMER0, FALSE);    //LPT 停止
}
/********************************************************************************
  ** \brief  Lptimer中断服务函数
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
			Lptim_ClrItStatus(M0P_LPTIMER0);//LPTIMER0清除标志位    
    }
}
/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/










