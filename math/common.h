
#ifndef __COMMON_H
#define __COMMON_H
#include "stdio.h"
#include "math.h"
#include "string.h"
#include "ddl.h"
#include "stdlib.h"
#include "trng.h"
#include "ctype.h"

void GetDstAndNumFromStrByCs(char *str,char *cs,unsigned char *num,int dst[]);
void GetDstAndNumFromStrByCsSize(char *str,char *cs,unsigned char *num,int dst[], int size);
int GetNumFromStrByCs(char *str,char *cs);
char *StrRStr(char *str1, char *str2);
int AsciiStrToHexStr(char *asc, int asc_size, char *dst, int dst_size);
int HexStrToAsciiStr(char *hex, char *dst, int size);
int HexStrToInt(char *hex, char *dst, int size);
uint8_t Random(uint8_t Max);
unsigned int Float2hexRepr(float* a);

#endif


