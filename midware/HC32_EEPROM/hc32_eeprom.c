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

uint8_t WaterRecordindex = 1;//索引
uint8_t WaterRecordDay = 1;//记录天数

void IIC_Init(void){
	
    stc_gpio_cfg_t stcGpioCfg;
    
    ///< 打开GPIO外设时钟门控
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); 
    
    ///< 端口方向配置
    stcGpioCfg.enDir = GpioDirOut;
    ///< 配置上下拉
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;

//    ///< SDA关闭
    Gpio_SetIO(USER_IIC_PORT, USER_SDA_PIN);
    Gpio_SetIO(USER_IIC_PORT, USER_SCL_PIN);	
    Gpio_ClrIO(USER_WP_PORT, USER_WP_PIN);	
	
    ///< GPIO IO 端口初始化
    Gpio_Init(USER_IIC_PORT, USER_SDA_PIN, &stcGpioCfg);
    Gpio_Init(USER_IIC_PORT, USER_SCL_PIN, &stcGpioCfg);
    Gpio_Init(USER_WP_PORT, USER_WP_PIN, &stcGpioCfg);	

		HC32_SCL_SDA_Hight();
}

void AT24CXX_Deinit(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    ///< 打开GPIO外设时钟门控
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, FALSE); 
    
    ///< 端口方向配置
    stcGpioCfg.enDir = GpioDirIn;
    ///< 配置上下拉
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
	
    ///< GPIO IO 端口初始化
    Gpio_Init(USER_IIC_PORT, USER_SDA_PIN, &stcGpioCfg);
    Gpio_Init(USER_IIC_PORT, USER_SCL_PIN, &stcGpioCfg);
}
void SDA_OUT(void)
{
    stc_gpio_cfg_t stcGpioCfg;

    ///< 端口方向配置
    stcGpioCfg.enDir = GpioDirOut;
    ///< 配置上下拉
    stcGpioCfg.enPu = GpioPuEnable;
    stcGpioCfg.enPd = GpioPdDisable;	
    Gpio_Init(USER_IIC_PORT, USER_SDA_PIN, &stcGpioCfg);
}
void SDA_IN(void)
{
    stc_gpio_cfg_t stcGpioCfg;
   
    ///< 端口方向配置
    stcGpioCfg.enDir = GpioDirIn;
    ///< 无上下拉
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
//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA(1);	  	  
	IIC_SCL(1);
	delay1us(4);
 	IIC_SDA(0);//START:when CLK is high,DATA change form high to low 
	delay1us(4);
	IIC_SCL(0);//钳住I2C总线，准备发送或接收数据 
}	
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL(0);
	IIC_SDA(0);//STOP:when CLK is high DATA change form low to high
 	delay1us(4);
	IIC_SCL(1); 
	IIC_SDA(1);//发送I2C总线结束信号
	delay1us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
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
	IIC_SCL(0);//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
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
//不产生ACK应答		    
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
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
		SDA_OUT(); 	    
    IIC_SCL(0);//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
      IIC_SDA((txd&0x80)>>7);
      txd<<=1; 	  
			delay1us(2);   //对TEA5767这三个延时都是必须的
			IIC_SCL(1);
			delay1us(2); 
			IIC_SCL(0);	
			delay1us(2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t IIC_Read_Byte(unsigned char ack)
{
		unsigned char i,receive=0;
		SDA_IN();//SDA设置为输入
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
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}
//初始化IIC接口
void AT24CXX_Init(void)
{
	IIC_Init();//IIC初始化
}
//在AT24CXX指定地址读出一个数据
//ReadAddr:开始读数的地址  
//返回值  :读到的数据
uint8_t AT24CXX_ReadOneByte(uint16_t ReadAddr)
{				  
		uint8_t temp=0;		  	    																 
		IIC_Start();  
		if(EE_TYPE>AT24C16)
		{
			IIC_Send_Byte(0XA0);	   //发送写命令
			IIC_Wait_Ack();
			IIC_Send_Byte(ReadAddr>>8);//发送高地址	    
		}else IIC_Send_Byte(0XA0+((ReadAddr/256)<<1));   //发送器件地址0XA0,写数据 	   
		IIC_Wait_Ack(); 
		IIC_Send_Byte(ReadAddr%256);   //发送低地址
		IIC_Wait_Ack();	    
		IIC_Start();  	 	   
		IIC_Send_Byte(0XA1);           //进入接收模式			   
		IIC_Wait_Ack();	 
		temp=IIC_Read_Byte(0);		   
		IIC_Stop();//产生一个停止条件	    
		return temp;
}
//在AT24CXX指定地址写入一个数据
//WriteAddr  :写入数据的目的地址    
//DataToWrite:要写入的数据
void AT24CXX_WriteOneByte(uint16_t WriteAddr,uint8_t DataToWrite)
{				   	  	    																 
  IIC_Start();  
	if(EE_TYPE>AT24C16)
	{
		IIC_Send_Byte(0XA0);	    //发送写命令
		IIC_Wait_Ack();
		IIC_Send_Byte(WriteAddr>>8);//发送高地址	  
	}else IIC_Send_Byte(0XA0+((WriteAddr/256)<<1));   //发送器件地址0XA0,写数据 	 
	IIC_Wait_Ack();	   
  IIC_Send_Byte(WriteAddr%256);   //发送低地址
	IIC_Wait_Ack(); 	 										  		   
	IIC_Send_Byte(DataToWrite);     //发送字节							   
	IIC_Wait_Ack();  		    	   
  IIC_Stop();//产生一个停止条件 
	delay1ms(10);	 
}
//在AT24CXX里面的指定地址开始写入长度为Len的数据
//该函数用于写入16bit或者32bit的数据.
//WriteAddr  :开始写入的地址  
//DataToWrite:数据数组首地址
//Len        :要写入数据的长度2,4
void AT24CXX_WriteLenByte(uint16_t WriteAddr,uint32_t DataToWrite,uint8_t Len)
{  	
	uint8_t t;
	for(t=0;t<Len;t++)
	{
		AT24CXX_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}												    
}

//在AT24CXX里面的指定地址开始读出长度为Len的数据
//该函数用于读出16bit或者32bit的数据.
//ReadAddr   :开始读出的地址 
//返回值     :数据
//Len        :要读出数据的长度2,4
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
//检查AT24CXX是否正常
//这里用了24XX的最后一个地址(255)来存储标志字.
//如果用其他24C系列,这个地址要修改
//返回1:检测失败
//返回0:检测成功
uint8_t AT24CXX_Check(void)
{
	uint8_t temp;
	temp=AT24CXX_ReadOneByte(0);//避免每次开机都写AT24CXX			   
	if(temp==0X55)return 0;		   
	else//排除第一次初始化的情况
	{
		AT24CXX_WriteOneByte(0,0X55);
	  temp=AT24CXX_ReadOneByte(0);	  
		if(temp==0X55){
			WaterRecordindex = AT24CXX_ReadOneByte(ADDR_ONEDAY_START+4);//开机读索引
			WaterRecordDay = AT24CXX_ReadOneByte(ADDR_ONEDAY_START+3);//读取天数索引
			return 0;
		}
	}

	return 1;											  
}

//在AT24CXX里面的指定地址开始读出指定个数的数据
//ReadAddr :开始读出的地址 对24c02为0~255
//pBuffer  :数据数组首地址
//NumToRead:要读出数据的个数
void AT24CXX_Read(uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++=AT24CXX_ReadOneByte(ReadAddr++);	
		NumToRead--;
	}
}  
//在AT24CXX里面的指定地址开始写入指定个数的数据
//WriteAddr :开始写入的地址 对24c02为0~255
//pBuffer   :数据数组首地址
//NumToWrite:要写入数据的个数
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


























