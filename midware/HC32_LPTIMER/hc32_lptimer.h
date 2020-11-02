#ifndef __HC32_LPTIMER_H__
#define __HC32_LPTIMER_H__
#include "gpio.h"
#include "lptim.h"
/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
extern int gLptimSecRecord;
void HC32_LPTIMER_Init(void);
void LpTimer_Enable(void);
void LpTimer_Disable(void);
#ifdef __cplusplus
}
#endif

#endif /* __HC32_LPTIMER_H__ */


