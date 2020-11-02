#ifndef __HC32_SYSTEM_H__
#define __HC32_SYSTEM_H__
#include "sysctrl.h"
/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
void HC32_System32_Init(void);
void HC32_System48_Init(void);
void HC32_System_Change_4M(void);
#ifdef __cplusplus
}
#endif

#endif /* __HC32_SYSTEM_H__ */

