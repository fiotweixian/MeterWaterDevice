#ifndef __HC32_WDT_H__
#define __HC32_WDT_H__
#include "wdt.h"
#include "ddl.h"
/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

void HC32_WDT_Init(en_wdt_func_t enFunc, en_wdt_time_t enTime);

#ifdef __cplusplus
}
#endif

#endif /* __HC32_WDT_H__ */


