/******************************************************************************
 ** \file hc32_eeprom.c
 **
 **  Copyright Fuzhou Internet of Things Open Lab.
 **
 **	 Author :Fiot-lab
 **
 **  DESCRIPTION: This file contains EERPROM, Configure and Read/write operate.
 **
 ******************************************************************************/
#include "hc32_eeprom.h"
#include "hc32_debug.h"

uint8_t WaterRecordindex = 1;//����
uint8_t WaterRecordDay = 1;//��¼����

void IIC_Init(void){
	
    stc_gpio_cfg_t stcGpioCfg;
    
    ///< ��GPIO����ʱ���ſ�
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); 
    
    ///< �˿ڷ�������
    stcGpioCfg.enDir = GpioDirOut;
    ///< ����������
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;

//    ///< SDA�ر�
    Gpio_SetIO(USER_IIC_PORT, USER_SDA_PIN);
    Gpio_SetIO(USER_IIC_PORT, USER_SCL_PIN);	
    Gpio_ClrIO(USER_WP_PORT, USER_WP_PIN);	
	
    ///< GPIO IO �˿ڳ�ʼ��
    Gpio_Init(USER_IIC_PORT, USER_SDA_PIN, &stcGpioCfg);
    Gpio_Init(USER_IIC_PORT, USER_SCL_PIN, &stcGpioCfg);
    Gpio_Init(USER_WP_PORT, USER_WP_PIN, &stcGpioCfg);	

		HC32_SCL_SDA_Hight();
}

void AT24CXX_Deinit(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    ///< ��GPIO����ʱ���ſ�
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, FALSE); 
    
    ///< �˿ڷ�������
    stcGpioCfg.enDir = GpioDirIn;
    ///< ����������
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
	
    ///< GPIO IO �˿ڳ�ʼ��
    Gpio_Init(USER_IIC_PORT, USER_SDA_PIN, &stcGpioCfg);
    Gpio_Init(USER_IIC_PORT, USER_SCL_PIN, &stcGpioCfg);
}
void SDA_OUT(void)
{
    stc_gpio_cfg_t stcGpioCfg;

    ///< �˿ڷ�������
    stcGpioCfg.enDir = GpioDirOut;
    ///< ����������
    stcGpioCfg.enPu = GpioPuEnable;
    stcGpioCfg.enPd = GpioPdDisable;	
    Gpio_Init(USER_IIC_PORT, USER_SDA_PIN, &stcGpioCfg);
}
void SDA_IN(void)
{
    stc_gpio_cfg_t stcGpioCfg;
   
    ///< �˿ڷ�������
    stcGpioCfg.enDir = GpioDirIn;
    ///< ��������
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
	
    Gpio_Init(USER_IIC_PORT, USER_SDA_PIN, &stcGpioCfg);
}
void IIC_SDA(boolean_t level)
{
		if(level)
			Gpio_SetIO(USER_IIC_PORT, USER_SDA_PIN);
		else
			Gpio_ClrIO(USER_IIC_PORT, USER_SDA_PIN);			
}
void IIC_SCL(boolean_t level)
{
		if(level)
			Gpio_SetIO(USER_IIC_PORT, USER_SCL_PIN);
		else
			Gpio_ClrIO(USER_IIC_PORT, USER_SCL_PIN);			
}
boolean_t READ_SDA(void)
{
	return Gpio_GetInputIO(USER_IIC_PORT, USER_SDA_PIN);
}
//����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA(1);	  	  
	IIC_SCL(1);
	delay1us(4);
 	IIC_SDA(0);//START:when CLK is high,DATA change form high to low 
	delay1us(4);
	IIC_SCL(0);//ǯסI2C���ߣ�׼�����ͻ�������� 
}	
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL(0);
	IIC_SDA(0);//STOP:when CLK is high DATA change form low to high
 	delay1us(4);
	IIC_SCL(1); 
	IIC_SDA(1);//����I2C���߽����ź�
	delay1us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	IIC_SDA(1);delay1us(1);	   
	IIC_SCL(1);delay1us(1);	 
	while(READ_SDA())
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL(0);//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)
{
	IIC_SCL(0);
	SDA_OUT();
	IIC_SDA(0);
	delay1us(1);
	IIC_SCL(1);
	delay1us(2);
	IIC_SCL(0);
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	IIC_SCL(0);
	SDA_OUT();
	IIC_SDA(1);
	delay1us(2);
	IIC_SCL(1);
	delay1us(2);
	IIC_SCL(0);
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
		SDA_OUT(); 	    
    IIC_SCL(0);//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
      IIC_SDA((txd&0x80)>>7);
      txd<<=1; 	  
			delay1us(2);   //��TEA5767��������ʱ���Ǳ����
			IIC_SCL(1);
			delay1us(2); 
			IIC_SCL(0);	
			delay1us(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
uint8_t IIC_Read_Byte(unsigned char ack)
{
		unsigned char i,receive=0;
		SDA_IN();//SDA����Ϊ����
		for(i=0;i<8;i++ )
		{
      IIC_SCL(0); 
      delay1us(2);
			IIC_SCL(1);
      receive<<=1;
      if(READ_SDA())receive++;   
			delay1us(1); 
    }					 
    if(!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}
//��ʼ��IIC�ӿ�
void AT24CXX_Init(void)
{
	IIC_Init();//IIC��ʼ��
}
//��AT24CXXָ����ַ����һ������
//ReadAddr:��ʼ�����ĵ�ַ  
//����ֵ  :����������
uint8_t AT24CXX_ReadOneByte(uint16_t ReadAddr)
{				  
		uint8_t temp=0;		  	    																 
		IIC_Start();  
		if(EE_TYPE>AT24C16)
		{
			IIC_Send_Byte(0XA0);	   //����д����
			IIC_Wait_Ack();
			IIC_Send_Byte(ReadAddr>>8);//���͸ߵ�ַ	    
		}else IIC_Send_Byte(0XA0+((ReadAddr/256)<<1));   //����������ַ0XA0,д���� 	   
		IIC_Wait_Ack(); 
		IIC_Send_Byte(ReadAddr%256);   //���͵͵�ַ
		IIC_Wait_Ack();	    
		IIC_Start();  	 	   
		IIC_Send_Byte(0XA1);           //�������ģʽ			   
		IIC_Wait_Ack();	 
		temp=IIC_Read_Byte(0);		   
		IIC_Stop();//����һ��ֹͣ����	    
		return temp;
}
//��AT24CXXָ����ַд��һ������
//WriteAddr  :д�����ݵ�Ŀ�ĵ�ַ    
//DataToWrite:Ҫд�������
void AT24CXX_WriteOneByte(uint16_t WriteAddr,uint8_t DataToWrite)
{				   	  	    																 
  IIC_Start();  
	if(EE_TYPE>AT24C16)
	{
		IIC_Send_Byte(0XA0);	    //����д����
		IIC_Wait_Ack();
		IIC_Send_Byte(WriteAddr>>8);//���͸ߵ�ַ	  
	}else IIC_Send_Byte(0XA0+((WriteAddr/256)<<1));   //����������ַ0XA0,д���� 	 
	IIC_Wait_Ack();	   
  IIC_Send_Byte(WriteAddr%256);   //���͵͵�ַ
	IIC_Wait_Ack(); 	 										  		   
	IIC_Send_Byte(DataToWrite);     //�����ֽ�							   
	IIC_Wait_Ack();  		    	   
  IIC_Stop();//����һ��ֹͣ���� 
	delay1ms(10);	 
}
//��AT24CXX�����ָ����ַ��ʼд�볤��ΪLen������
//�ú�������д��16bit����32bit������.
//WriteAddr  :��ʼд��ĵ�ַ  
//DataToWrite:���������׵�ַ
//Len        :Ҫд�����ݵĳ���2,4
void AT24CXX_WriteLenByte(uint16_t WriteAddr,uint32_t DataToWrite,uint8_t Len)
{  	
	uint8_t t;
	for(t=0;t<Len;t++)
	{
		AT24CXX_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}												    
}

//��AT24CXX�����ָ����ַ��ʼ��������ΪLen������
//�ú������ڶ���16bit����32bit������.
//ReadAddr   :��ʼ�����ĵ�ַ 
//����ֵ     :����
//Len        :Ҫ�������ݵĳ���2,4
uint32_t AT24CXX_ReadLenByte(uint16_t ReadAddr,uint8_t Len)
{  	
	uint8_t t;
	uint32_t temp=0;
	for(t=0;t<Len;t++)
	{
		temp<<=8;
		temp+=AT24CXX_ReadOneByte(ReadAddr+Len-t-1); 	 				   
	}
	return temp;												    
}
//���AT24CXX�Ƿ�����
//��������24XX�����һ����ַ(255)���洢��־��.
//���������24Cϵ��,�����ַҪ�޸�
//����1:���ʧ��
//����0:���ɹ�
uint8_t AT24CXX_Check(void)
{
	uint8_t temp;
	temp=AT24CXX_ReadOneByte(0);//����ÿ�ο�����дAT24CXX			   
	if(temp==0X55)return 0;		   
	else//�ų���һ�γ�ʼ�������
	{
		AT24CXX_WriteOneByte(0,0X55);
	  temp=AT24CXX_ReadOneByte(0);	  
		if(temp==0X55){
			WaterRecordindex = AT24CXX_ReadOneByte(ADDR_ONEDAY_START+4);//����������
			WaterRecordDay = AT24CXX_ReadOneByte(ADDR_ONEDAY_START+3);//��ȡ��������
			return 0;
		}
	}

	return 1;											  
}

//��AT24CXX�����ָ����ַ��ʼ����ָ������������
//ReadAddr :��ʼ�����ĵ�ַ ��24c02Ϊ0~255
//pBuffer  :���������׵�ַ
//NumToRead:Ҫ�������ݵĸ���
void AT24CXX_Read(uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++=AT24CXX_ReadOneByte(ReadAddr++);	
		NumToRead--;
	}
}  
//��AT24CXX�����ָ����ַ��ʼд��ָ������������
//WriteAddr :��ʼд��ĵ�ַ ��24c02Ϊ0~255
//pBuffer   :���������׵�ַ
//NumToWrite:Ҫд�����ݵĸ���
void AT24CXX_Write(uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite)
{
	while(NumToWrite--)
	{
		AT24CXX_WriteOneByte(WriteAddr,*pBuffer);
		WriteAddr++;
		pBuffer++;
	}
}	
void HC32_StoreWaterNum(uint32_t StoreWater_H,uint32_t StoreWater_L){
	uint8_t tmpbuf[10] = {0};
	static uint8_t Dayindex = 4;
	uint8_t i = 0;
	stc_rtc_time_t readtime;
	Rtc_ReadDateTime(&readtime);
	if(Dayindex != WaterRecordDay){
		if(WaterRecordDay >= 4)
			WaterRecordDay = 1;
		if(WaterRecordDay == 1){
			AT24CXX_WriteLenByte(ADDR_ONEDAY_START, BCD2DEC(readtime.u8Year), 1);
			AT24CXX_WriteLenByte(ADDR_ONEDAY_START+1, BCD2DEC(readtime.u8Month), 1);
			AT24CXX_WriteLenByte(ADDR_ONEDAY_START+2, BCD2DEC(readtime.u8Day), 1);
		}else if(WaterRecordDay == 2){
			AT24CXX_WriteLenByte(ADDR_SECONDDAY_START, BCD2DEC(readtime.u8Year), 1);
			AT24CXX_WriteLenByte(ADDR_SECONDDAY_START+1, BCD2DEC(readtime.u8Month), 1);
			AT24CXX_WriteLenByte(ADDR_SECONDDAY_START+2, BCD2DEC(readtime.u8Day), 1);				
		}else if(WaterRecordDay == 3){
			AT24CXX_WriteLenByte(ADDR_THREEDAY_START, BCD2DEC(readtime.u8Year), 1);
			AT24CXX_WriteLenByte(ADDR_THREEDAY_START+1, BCD2DEC(readtime.u8Month), 1);
			AT24CXX_WriteLenByte(ADDR_THREEDAY_START+2, BCD2DEC(readtime.u8Day), 1);				
		}
		AT24CXX_WriteLenByte(ADDR_ONEDAY_START+3, WaterRecordDay, 1);			
		Dayindex = WaterRecordDay;		
	}
	tmpbuf[0] = BCD2DEC(readtime.u8Hour);
	tmpbuf[1] = BCD2DEC(readtime.u8Minute);
	tmpbuf[2] = BCD2DEC(readtime.u8Second);
	tmpbuf[3] = StoreWater_H/10000;
	tmpbuf[4] = StoreWater_H%10000/100;
	tmpbuf[5] = StoreWater_H%10000%100;	
	tmpbuf[6] = StoreWater_L/100;
	tmpbuf[7] = StoreWater_L%100;		
	
	for(i = 0; i<8 ;i++){
		AT24CXX_WriteLenByte(ADDR_ONEWATERNUM_START + WaterRecordindex * 8 + i,tmpbuf[i],1);
	}
	if(WaterRecordindex<=48)
		WaterRecordindex++;
	
	AT24CXX_WriteLenByte(ADDR_ONEDAY_START+4,WaterRecordindex,1);
}


























