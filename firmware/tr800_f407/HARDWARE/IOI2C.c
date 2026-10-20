#include "ioi2c.h"
#include "sys.h"
#include "delay.h"
/**************************************************************************
函数功能：IIC引脚初始化
入口参数：无
返回  值：无
**************************************************************************/
void IIC_Init(void)
{			
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOB时钟
  //GPIOB8,B9初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
  IIC_SCL=1;
  IIC_SDA=1;	
}
/**************************************************************************
函数功能：模拟IIC起始信号
入口参数：无
返回  值：无
**************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();    
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
	//START:when CLK is high,DATA change form high to low 
 	IIC_SDA=0;
	delay_us(4);
	IIC_SCL=0;
}	  
/**************************************************************************
函数功能：模拟IIC结束信号
入口参数：无
返回  值：无
**************************************************************************/
void IIC_Stop(void)
{
	SDA_OUT();
	IIC_SCL=0;
	IIC_SDA=0;
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;
	delay_us(4);							   	
}
/**************************************************************************
函数功能：IIC产生应答信号
入口参数：无
返回  值：无
**************************************************************************/
unsigned char IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;  
	return 0;  
} 
/**************************************************************************
函数功能：IIC应答
入口参数：无
返回  值：无
**************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
/**************************************************************************
函数功能：IIC不应答
入口参数：无
返回  值：无
**************************************************************************/ 
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}					 				     
/**************************************************************************
函数功能：IIC发送一个位
入口参数：无
返回  值：无
**************************************************************************/
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
/**************************************************************************
函数功能：II读取一个位
入口参数：无
返回  值：无
**************************************************************************/
unsigned char IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();
    else
        IIC_Ack();
    return receive;
}

