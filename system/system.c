#include "system.h"
#include "flash.h"
void HC32_System32_Init(void)
{
	
    ///< 外部低速时钟配置(for LCD)
    Sysctrl_XTLDriverCfg(SysctrlXtlAmp2, SysctrlXtalDriver2);
    Sysctrl_SetXTLStableTime(SysctrlXtlStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkXTL,TRUE);
    
    ///< 外部高速时钟初始化配置
    ///< 32MHz
    Sysctrl_SetXTHFreq(SysctrlXthFreq24_32MHz);
    Sysctrl_XTHDriverCfg(SysctrlXtalDriver3);
    Sysctrl_SetXTHStableTime(SysctrlXthStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkXTH, TRUE);

}
void HC32_System48_Init(void)
{
    en_flash_waitcycle_t      enFlashWait;
    stc_sysctrl_pll_cfg_t     stcPLLCfg;
    
    DDL_ZERO_STRUCT(stcPLLCfg);
    
    enFlashWait = FlashWaitCycle1;                      //????????1(?HCLK??24MHz??????1)
    Flash_WaitCycle(enFlashWait);                       // Flash ??1???
    
    stcPLLCfg.enInFreq    = SysctrlPllInFreq4_6MHz;     //RCH 4MHz
    stcPLLCfg.enOutFreq   = SysctrlPllOutFreq36_48MHz;  //PLL ??48MHz
    stcPLLCfg.enPllClkSrc = SysctrlPllRch;              //???????RCH
    stcPLLCfg.enPllMul    = SysctrlPllMul12;            //4MHz x 12 = 48MHz
    Sysctrl_SetPLLFreq(&stcPLLCfg);
    Sysctrl_SetPLLStableTime(SysctrlPllStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkPLL, TRUE);
    
    Sysctrl_SysClkSwitch(SysctrlClkPLL);   ///< ????
}
void HC32_System_Change_4M(void)
{
			M0P_SYSCTRL->SYSCTRL2 = 0X5A5A;
			M0P_SYSCTRL->SYSCTRL2 = 0XA5A5;
			M0P_SYSCTRL->SYSCTRL0_f.HCLK_PRS = 7;
			M0P_SYSCTRL->RCH_CR = *((unsigned short *)( 0X00100C00 ) ); //24M
			M0P_SYSCTRL->RCH_CR = *((unsigned short *)( 0X00100C04 ) ); //16M
			M0P_SYSCTRL->RCH_CR = *((unsigned short *)( 0X00100C06 ) ); //8M
			M0P_SYSCTRL->RCH_CR = *((unsigned short *)( 0X00100C08 ) ); //4M
			M0P_SYSCTRL->SYSCTRL2 = 0X5A5A;
			M0P_SYSCTRL->SYSCTRL2 = 0XA5A5;
			M0P_SYSCTRL->SYSCTRL0_f.HCLK_PRS = 0;
}











