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
#if HC32_FLASH_WREN	//���ʹ����д   
void HC32FLASH_Write_NoCheck(uint32_t WriteAddr,uint16_t *pBuffer,uint16_t NumToWrite)   
{ 			 		 
	uint16_t i;
	for(i=0;i<NumToWrite;i++)
	{
			Flash_WriteHalfWord(WriteAddr,pBuffer[i]);//д�����ֽ�
	    WriteAddr+=2;//��ַ����2.
	}  
} 
#if HC32_FLASH_SIZE<256
#define HC_SECTOR_SIZE 512 //�ֽ�
#else 
#define HC_SECTOR_SIZE	2048
#endif
uint16_t STMFLASH_BUF[HC_SECTOR_SIZE/2];//�����2K�ֽ�
void HC32FLASH_Write(uint32_t WriteAddr,uint16_t *pBuffer,uint16_t NumToWrite)	
{
	uint32_t secpos;	   //������ַ
	uint16_t secoff;	   //������ƫ�Ƶ�ַ(16λ�ּ���)
	uint16_t secremain; //������ʣ���ַ(16λ�ּ���)	   
 	uint16_t i;    
	uint32_t offaddr;   //ȥ��0X08000000��ĵ�ַ
	if(WriteAddr<HC32_FLASH_BASE||(WriteAddr>=(HC32_FLASH_BASE+512*HC32_FLASH_SIZE)))return;//�Ƿ���ַ
	Flash_UnlockAll();						//����
	offaddr=WriteAddr-HC32_FLASH_BASE;		//ʵ��ƫ�Ƶ�ַ.
	secpos=offaddr/HC_SECTOR_SIZE;			//������ַ  0~127 for STM32F103RBT6
	secoff=(offaddr%HC_SECTOR_SIZE)/2;		//�������ڵ�ƫ��(2���ֽ�Ϊ������λ.)
	secremain=HC_SECTOR_SIZE/2-secoff;		//����ʣ��ռ��С   
	if(NumToWrite<=secremain)secremain=NumToWrite;//�����ڸ�������Χ
	while(1) 
	{	
		HC32FLASH_Read(secpos*HC_SECTOR_SIZE+HC32_FLASH_BASE,STMFLASH_BUF,HC_SECTOR_SIZE/2);//������������������
		for(i=0;i<secremain;i++)//У������
		{
			if(STMFLASH_BUF[secoff+i]!=0XFFFF)break;//��Ҫ����  	  
		}
		if(i<secremain)//��Ҫ����
		{
			Flash_SectorErase(secpos*HC_SECTOR_SIZE+HC32_FLASH_BASE);//�����������
			for(i=0;i<secremain;i++)//����
			{
				STMFLASH_BUF[i+secoff]=pBuffer[i];	  
			}
			HC32FLASH_Write_NoCheck(secpos*HC_SECTOR_SIZE+HC32_FLASH_BASE,STMFLASH_BUF,HC_SECTOR_SIZE/2);//д����������  
		}else HC32FLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//д�Ѿ������˵�,ֱ��д������ʣ������. 				   
		if(NumToWrite==secremain)break;//д�������
		else//д��δ����
		{
			secpos++;				//������ַ��1
			secoff=0;				//ƫ��λ��Ϊ0 	 
		   	pBuffer+=secremain;  	//ָ��ƫ��
			WriteAddr+=secremain;	//д��ַƫ��	   
		   	NumToWrite-=secremain;	//�ֽ�(16λ)���ݼ�
			if(NumToWrite>(HC_SECTOR_SIZE/2))secremain=HC_SECTOR_SIZE/2;//��һ����������д����
			else secremain=NumToWrite;//��һ����������д����
		}	 
	};	
	Flash_LockAll();//����
}

#endif

void HC32FLASH_Read(uint32_t ReadAddr,uint16_t *pBuffer,uint16_t NumToRead)   	
{
	uint16_t i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=HC32FLASH_ReadHalfWord(ReadAddr);//��ȡ2���ֽ�.
		ReadAddr+=2;//ƫ��2���ֽ�.	
	}
}
void Test_Write(uint32_t WriteAddr,uint16_t WriteData)   	
{
	HC32FLASH_Write(WriteAddr,&WriteData,1);//д��һ���� 
	
}
#else 



#endif














