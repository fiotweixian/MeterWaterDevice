#include "Common.h"

void GetDstAndNumFromStrByCs(char *str,char *cs,unsigned char *num,int dst[]){
    char *p = str, *pcs = cs;
    if(!cs) return;
    for(*num = 0,dst[0] = 0; (p != NULL && *p != 0) && (cs != NULL && *cs != 0); p++){
        for(pcs = cs; pcs < cs + strlen(cs)  && (cs != NULL && *cs != 0) && *p != 0; pcs++){
            if(*p == *pcs){
                *p = 0;
                (*num)++;
                dst[*num] = (int)(p - str + 1);
            }
        }
    }
}
int neul_bc95_str_to_hex(const unsigned char *bufin, int len, char *bufout)
{
    int i = 0;
    #if 0
    int tmp = 0;
    #endif
    if (NULL == bufin || len <= 0 || NULL == bufout)
    {
        return -1;
    }
    for(i = 0; i < len; i++)
    {
        #if 0
        tmp = bufin[i]>>4;
        bufout[i*2] = tmp > 0x09?tmp+0x37:tmp+0x30;
        tmp = bufin[i]&0x0F;
        bufout[i*2+1] = tmp > 0x09?tmp+0x37:tmp+0x30;
        #else
        sprintf(bufout+i*2, "%02X", bufin[i]);
        #endif
    }
    return 0; 
}
void GetDstAndNumFromStrByCsSize(char *str,char *cs,unsigned char *num,int dst[], int size){
    char *p = str, *pcs = cs;
    if(!cs) return;
    for(*num = 0,dst[0] = 0; (p != NULL && *p != 0) && (cs != NULL && *cs != 0); p++){
        for(pcs = cs; pcs < cs + strlen(cs)  && (cs != NULL && *cs != 0) && *p != 0; pcs++){
            if(*p == *pcs){
                (*num)++;
                if((*num) < size){
                    *p = 0;
                    dst[*num] = (int)(p - str + 1);
                }
            }
        }
    }
}
int GetNumFromStrByCs(char *str,char *cs){
    char *p = str, *pcs = cs;
    int num = 0;
    if(!cs) return num;
    for(; (p != NULL && *p != 0) && (cs != NULL && *cs != 0); p++){
        for(pcs = cs; pcs < cs + strlen(cs)  && (cs != NULL && *cs != 0) && *p != 0; pcs++){
            if(*p == *pcs){
                num++;
            }
        }
    }
    return num;
}

char *StrRStr(char *str1, char *str2){
    char *p = str1, *ret = NULL; 
    if(!str1 || !str2) return ret;
    while(0 != (p = strstr(p, str2))){
        ret = p;
        p++;
    }
    return ret;
}

int AsciiStrToHexStr(char *asc, int asc_size, char *dst, int dst_size){

		char value = 0;
    int i = 0;
    if(asc && dst && (asc_size > 0)){
        if((asc_size + asc_size) < dst_size){
            memset(dst, 0, dst_size);
            for( i = 0; i < asc_size; i++){
                value = *(asc + i);
                if((value / 16) >= 10){
                    *(dst+i+i) = 'A' + (value / 16)-10;
                }else{
                    *(dst+i+i) = '0' + (value / 16);
                }
                if((value % 16) >= 10){
                    *(dst+i+i+1) = 'A' + (value % 16)-10;
                }else{
                    *(dst+i+i+1) = '0' + (value % 16);
                }
            }
            return 1;
        }
    }
    return 0;
}
int HexStrToAsciiStr(char *hex, char *dst, int size){
    char value = 0, value1 = 0;
    int length = 0, i = 0;
    if(hex && dst && (strlen(hex) > 0)){
        length = strlen(hex);
        if((length/2) < size){
            memset(dst, 0, size);
            for( i = 0; i < length; i += 2){
                value = *(hex + i);
                value1 = *(hex + i + 1);
                if((value >= '0') && (value <= '9')){
                    value = value - '0';
		   }else if((value >= 'A') && (value <= 'F')){
                    value = value - 'A' + 10;
		   }else if((value >= 'a') && (value <= 'f')){
                    value = value - 'a' + 10;
		   }else{
		       continue;
		   }
                if((value1 >= '0') && (value1 <= '9')){
                    value1 = value1 - '0';
		   }else if((value1 >= 'A') && (value1 <= 'F')){
                    value1 = value1 - 'A' + 10;
		   }else if((value1 >= 'a') && (value1 <= 'f')){
                    value1 = value1 - 'a' + 10;
		   }else{
		       continue;
		   }
                *dst = value * 16 + value1;
                dst++;
            }
            return 1;
        }
    }
    return 0;
}
int HexStrToInt(char *hex, char *dst, int size){
    char value = 0, value1 = 0;
    int length = 0, i = 0;
    if(hex && dst && (strlen(hex) > 0)){
        length = strlen(hex);
        if((length/2) < size){
            memset(dst, 0, size);
            for( i = 0; i < length; i += 2){
                value = *(hex + i);
                value1 = *(hex + i + 1);
                if((value >= '0') && (value <= '9')){
                    value = value - '0';
		   }else{
		       continue;
		   }
                if((value1 >= '0') && (value1 <= '9')){
                    value1 = value1 - '0';
		   }else{
		       continue;
		   }
                *dst = value * 16 + value1 - 48;
                dst++;
            }
            return 1;
        }
    }
    return 0;
}

uint8_t Random(uint8_t Max){
	uint8_t value = 0;
	Sysctrl_SetPeripheralGate(SysctrlPeripheralRng, TRUE);
	Trng_Init();
	Trng_Generate();
	value =  Trng_GetData0()%(Max + 1- 0) + 0;
	Sysctrl_SetPeripheralGate(SysctrlPeripheralRng, FALSE);
	return value;
}
unsigned int Float2hexRepr(float* a){
      unsigned int c;  
     c= ((unsigned int*)a)[0];   
   return c;
}



