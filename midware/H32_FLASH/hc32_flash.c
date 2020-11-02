/******************************************************************************
 ** \file hc32_flash.c
 **
 **  Copyright Fuzhou Internet of Things Open Lab.
 **
 **	 Author :Fiot-lab
 **
 **  DESCRIPTION: This file contains Flash Configure.
 **
 ******************************************************************************/
#include "hc32_flash.h"

/*******************************************************************************
 * global variable - (global)  and static variable - (static) 
 ******************************************************************************/
 
 

uint8_t HC32_FLASH_Init(void)
{
    return Flash_Init(6, TRUE);
}
#if 0
uint8_t HC32FLASH_ReadHalfWord(uint8_t faddr)
{
	return *((volatile uint8_t*)faddr); 
}
#if HC32_FLASH_WREN	//如果使能了写   
void HC32FLASH_Write_NoCheck(uint32_t WriteAddr,uint16_t *pBuffer,uint16_t NumToWrite)   
{ 			 		 
	uint16_t i;
	for(i=0;i<NumToWrite;i++)
	{
			Flash_WriteHalfWord(WriteAddr,pBuffer[i]);//写两个字节
	    WriteAddr+=2;//地址增加2.
	}  
} 
#if HC32_FLASH_SIZE<256
#define HC_SECTOR_SIZE 512 //字节
#else 
#define HC_SECTOR_SIZE	2048
#endif
uint16_t STMFLASH_BUF[HC_SECTOR_SIZE/2];//最多是2K字节
void HC32FLASH_Write(uint32_t WriteAddr,uint16_t *pBuffer,uint16_t NumToWrite)	
{
	uint32_t secpos;	   //扇区地址
	uint16_t secoff;	   //扇区内偏移地址(16位字计算)
	uint16_t secremain; //扇区内剩余地址(16位字计算)	   
 	uint16_t i;    
	uint32_t offaddr;   //去掉0X08000000后的地址
	if(WriteAddr<HC32_FLASH_BASE||(WriteAddr>=(HC32_FLASH_BASE+512*HC32_FLASH_SIZE)))return;//非法地址
	Flash_UnlockAll();						//解锁
	offaddr=WriteAddr-HC32_FLASH_BASE;		//实际偏移地址.
	secpos=offaddr/HC_SECTOR_SIZE;			//扇区地址  0~127 for STM32F103RBT6
	secoff=(offaddr%HC_SECTOR_SIZE)/2;		//在扇区内的偏移(2个字节为基本单位.)
	secremain=HC_SECTOR_SIZE/2-secoff;		//扇区剩余空间大小   
	if(NumToWrite<=secremain)secremain=NumToWrite;//不大于该扇区范围
	while(1) 
	{	
		HC32FLASH_Read(secpos*HC_SECTOR_SIZE+HC32_FLASH_BASE,STMFLASH_BUF,HC_SECTOR_SIZE/2);//读出整个扇区的内容
		for(i=0;i<secremain;i++)//校验数据
		{
			if(STMFLASH_BUF[secoff+i]!=0XFFFF)break;//需要擦除  	  
		}
		if(i<secremain)//需要擦除
		{
			Flash_SectorErase(secpos*HC_SECTOR_SIZE+HC32_FLASH_BASE);//擦除这个扇区
			for(i=0;i<secremain;i++)//复制
			{
				STMFLASH_BUF[i+secoff]=pBuffer[i];	  
			}
			HC32FLASH_Write_NoCheck(secpos*HC_SECTOR_SIZE+HC32_FLASH_BASE,STMFLASH_BUF,HC_SECTOR_SIZE/2);//写入整个扇区  
		}else HC32FLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//写已经擦除了的,直接写入扇区剩余区间. 				   
		if(NumToWrite==secremain)break;//写入结束了
		else//写入未结束
		{
			secpos++;				//扇区地址增1
			secoff=0;				//偏移位置为0 	 
		   	pBuffer+=secremain;  	//指针偏移
			WriteAddr+=secremain;	//写地址偏移	   
		   	NumToWrite-=secremain;	//字节(16位)数递减
			if(NumToWrite>(HC_SECTOR_SIZE/2))secremain=HC_SECTOR_SIZE/2;//下一个扇区还是写不完
			else secremain=NumToWrite;//下一个扇区可以写完了
		}	 
	};	
	Flash_LockAll();//上锁
}

#endif

void HC32FLASH_Read(uint32_t ReadAddr,uint16_t *pBuffer,uint16_t NumToRead)   	
{
	uint16_t i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=HC32FLASH_ReadHalfWord(ReadAddr);//读取2个字节.
		ReadAddr+=2;//偏移2个字节.	
	}
}
void Test_Write(uint32_t WriteAddr,uint16_t WriteData)   	
{
	HC32FLASH_Write(WriteAddr,&WriteData,1);//写入一个字 
	
}
#else 



#endif














