#include "usartx.h"
SEND_DATA Send_Data;//发送数据的
FLYSKYIBUS FlySkyIBus;
FUTABA_SBUS FutabaSbus;
RINGBUFFER RingBuffer;
RECEIVE_DATA Receive_Data;//接收数据的
extern int Time_count;
extern int RC_STEP_X;
extern int RC_STEP_Y;
extern Encoder OriginalEncoder;
#if TR800_OPEN_LOOP_PWM_TEST
extern volatile u8 tr800_open_loop_state_debug;
#endif

int hand_stop = 0;
int hand_stop1 = 0;
int NoControlstop = 0;

int Flag_serial_cmd;
bool bias_adjust_mode = false;



#if 1
typedef enum {
	ZOOM_IN= 0,
	ZOOM_OUT = 1,
	STOP_Z = 2
}Yuntai_Zoom_TypeDef; 

typedef enum {
	ZOOM_ING= 0,
	ZOOM_PAUSE = 1
	
}Yuntai_ZoomState_TypeDef; 


Yuntai_Zoom_TypeDef  Yuntai_Zoom = STOP_Z;
Yuntai_ZoomState_TypeDef  Yuntai_ZoomState = ZOOM_PAUSE;

char yuntai_up[7] =        {0xFF, 0x01, 0x00, 0x08, 0x20, 0x20, 0x49};
char yuntai_down[7] =      {0xFF, 0x01, 0x00, 0x10, 0x20, 0x20, 0x51};
char yuntai_up_left[7] =   {0xFF, 0x01, 0x00, 0x0C, 0x20, 0x20, 0x49};
char yuntai_up_right[7] =  {0xFF, 0x01, 0x00, 0x0A, 0x20, 0x20, 0x49};

char yuntai_left[7] =      {0xFF, 0x01, 0x00, 0x04, 0x20, 0x20, 0x45};
char yuntai_right[7] =     {0xFF, 0x01, 0x00, 0x02, 0x20, 0x20, 0x43};
char yuntai_down_left[7] = {0xFF, 0x01, 0x00, 0x14, 0x20, 0x20, 0x49};
char yuntai_down_right[7] ={0xFF, 0x01, 0x00, 0x12, 0x20, 0x20, 0x49};

char yuntai_stop[7] = {0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01};
char yuntai_zoom_in[7] =  {0xFF, 0x01, 0x00, 0x20, 0x00, 0x00, 0x21};
char yuntai_zoom_out[7] = {0xFF, 0x01, 0x00, 0x40, 0x00, 0x00, 0x41};

char yuntai_home[7] = {0xFF, 0x01, 0x00, 0x07, 0x00, 0x01, 0x09};

char yuntai_h_speed=10;
char yuntai_h_speed_;
char yuntai_v_speed=10;
char yuntai_v_speed_;
char yuntai_speed = 10;
char yuntai_speed_ = 0;

#endif
#define SBUS_LOW   200
#define SBUS_MID   1000
#define SBUS_HIGH  1800
#define SBUS_BIAS  200
volatile u32 ibus_rx_byte_count = 0;
volatile u32 ibus_valid_packet_count = 0;
volatile u32 ibus_bad_packet_count = 0;
int rc_alive;
int RC_SW1_,RC_Home_;
extern double dl_map(double x, double in_min, double in_max, double out_min, double out_max);

/**************************************************************************
函数功能：串口2发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart2_send(u8 data)
{
	USART2->DR = data;
	while((USART2->SR&0x40)==0);	
}
/**************************************************************************
函数功能：串口2初始化
入口参数：无
返回  值：无
**************************************************************************/
void uart2_init(u32 bound)
{	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock  //使能GPIO时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //Enable the Usart clock //使能USART时钟
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2); 
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6 ,GPIO_AF_USART2);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;			  //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;		  //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; 	  //高速50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;			  //上拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);					  //初始化
	
	//UsartNVIC configuration //UsartNVIC配置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	//Subpriority //子优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
  //Enable the IRQ channel //IRQ通道使能	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //Initialize the VIC register with the specified parameters 
	//根据指定的参数初始化VIC寄存器 	
	NVIC_Init(&NVIC_InitStructure); 
	
	//USART Initialization Settings 初始化设置
	USART_InitStructure.USART_BaudRate = bound; //Port rate //串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //一个停止
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //Sending and receiving mode //收发模式
	USART_Init(USART2, &USART_InitStructure);	   //Initialize serial port 2 //初始化串口2
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
	USART_Cmd(USART2, ENABLE);					   //Enable serial port 2 //使能串口2 
	#if IBUS_EN
	ibus_begin();
    #elif SBUS_EN
    sbus_begin();
	#endif
}

u32 available(void)
{
	return (u32)(SERIAL_BUFFER_SIZE + RingBuffer._iHead - RingBuffer._iTail) % SERIAL_BUFFER_SIZE;
}

void store_char( u8 c ) 
{
   int i = (u32)(RingBuffer._iHead + 1) % SERIAL_BUFFER_SIZE ;
  if ( i != RingBuffer._iTail )
  {
    RingBuffer._aucBuffer[RingBuffer._iHead] = c ;
    RingBuffer._iHead = i ;
  }

}

u8 read(void)
{
  u8 uc;
// if the head isn't ahead of the tail, we don't have any characters
  if ( RingBuffer._iHead == RingBuffer._iTail )
    return 0 ;  
  uc = RingBuffer._aucBuffer[RingBuffer._iTail] ;
  RingBuffer._iTail = (unsigned int)(RingBuffer._iTail + 1) % SERIAL_BUFFER_SIZE ;
  return uc ;
}

u32 write(u8 ch)
{
	USART_SendData(USART2, ch);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); 
	return 1; 
}

void flush()
{
	RingBuffer._iTail = RingBuffer._iHead;
}

void print(const char *format, ...)
{
	va_list args;
	char buf[256];
	va_start (args, format);
	vsprintf (buf, format, args);       
	va_end (args);    
	putstr(buf);

}

/**************************************************************************
函数功能：串口2接收中断
入口参数：无
返回  值：无
**************************************************************************/
int USART2_IRQHandler(void)
{	
    #if IBUS_EN
	u32 now = getSysTickCnt();//millis(void)
	u8 v;
	if (now - FlySkyIBus.last >= PROTOCOL_TIMEGAP)
	{
	  FlySkyIBus.state = GET_LENGTH;
	}
	FlySkyIBus.last = now;
	
	v=USART_ReceiveData(USART2);
	ibus_rx_byte_count++;
	
	switch (FlySkyIBus.state)
	{
	  case GET_LENGTH:
		if (v <= PROTOCOL_LENGTH)
		{
		  FlySkyIBus.ptr = 0;
		  FlySkyIBus.len = v - PROTOCOL_OVERHEAD;
		  FlySkyIBus.chksum = 0xFFFF - v;
		  FlySkyIBus.state = GET_DATA;
		}
		else
		{
		  FlySkyIBus.state = DISCARD;
		}
		break;

	  case GET_DATA:
		FlySkyIBus.buffer[FlySkyIBus.ptr++] = v;
		FlySkyIBus.chksum -= v;
		if (FlySkyIBus.ptr == FlySkyIBus.len)
		{
		  FlySkyIBus.state = GET_CHKSUML;
		}
		break;
		
	  case GET_CHKSUML:
		FlySkyIBus.lchksum = v;
		FlySkyIBus.state = GET_CHKSUMH;
		break;

	  case GET_CHKSUMH:
		if (FlySkyIBus.chksum == (v << 8) + FlySkyIBus.lchksum)
		{
		  u8 i;
		  switch (FlySkyIBus.buffer[0])
		  {
			case PROTOCOL_COMMAND40:
			
			  for (i = 1; i < PROTOCOL_CHANNELS * 2 + 1; i += 2)
			  {
				FlySkyIBus.channel[i / 2] = FlySkyIBus.buffer[i] | (FlySkyIBus.buffer[i + 1] << 8);
			  }
			  tr800_runtime_note_rc_rx(now);
			  ibus_valid_packet_count++;
			  control_sig++;
			  break;
			  default:
			  break;
			  }
		}
		else
		{
		  ibus_bad_packet_count++;
		}
		FlySkyIBus.state = DISCARD;
		break;
	  case DISCARD:
	  default:
		break;
	}
//	USART_ClearITPendingBit(USART3, USART_IT_RXNE);


   #elif SBUS_EN
   u8 data;
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
		
		data = USART_ReceiveData(USART2);
		store_char(data) ;  
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);    
	}
   #endif
  
return 0;	
}

void putstr(const char *str)
{
	int i;
	for(i = 0; i < strlen(str); i++){	
		usart3_send(str[i]);
	}
}

/**************************************************************************
函数功能：串口3发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart3_send(u8 data)
{
	USART3->DR = data;
	while((USART3->SR&0x40)==0);	
}
/**************************************************************************
函数功能：串口1发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}

void uart4_send(u8 data)
{
	UART4->DR = data;
	while((UART4->SR&0x40)==0);	
}


void usart5_send(u8 data)
{
	UART5->DR = data;
	while((UART5->SR&0x40)==0);	
}


/**************************************************************************
函数功能：串口4初始化
入口参数：无
返回  值：无
**************************************************************************/
void uart4_init(u32 bound)
{	 
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	 //Enable the gpio clock  //使能GPIO时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); //Enable the Usart clock //使能USART时钟
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;			  //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;		  //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; 	  //高速50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;			  //上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);				  //初始化
	
    //UsartNVIC configuration //UsartNVIC配置
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQ通道使能	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    //Initialize the VIC register with the specified parameters 
	//根据指定的参数初始化VIC寄存器 	
	NVIC_Init(&NVIC_InitStructure);
	
    //USART Initialization Settings 初始化设置
	USART_InitStructure.USART_BaudRate = bound; //Port rate //串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //一个停止
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //Sending and receiving mode //收发模式
    USART_Init(UART4, &USART_InitStructure); 	 //Initialize serial port 3 //初始化串口3
		
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
	USART_Cmd(UART4, ENABLE);					 //Enable serial port 3 //使能串口3 
  }

void uart3_init(u32 bound)
{	 
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock  //使能GPIO时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //Enable the Usart clock //使能USART时钟
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		      //初始化
	
    //UsartNVIC configuration //UsartNVIC配置
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQ通道使能	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    //Initialize the VIC register with the specified parameters 
	//根据指定的参数初始化VIC寄存器 	
	NVIC_Init(&NVIC_InitStructure);
	
    //USART Initialization Settings 初始化设置
	USART_InitStructure.USART_BaudRate = bound; //Port rate //串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //一个停止
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //Sending and receiving mode //收发模式
    USART_Init(USART3, &USART_InitStructure); 	 //Initialize serial port 3 //初始化串口3
		
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
	USART_Cmd(USART3, ENABLE);					 //Enable serial port 3 //使能串口3 
  }


void uart5_init(u32 bound)
{	 
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	 //Enable the gpio clock  //使能GPIO时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock  //使能GPIO时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //Enable the Usart clock //使能USART时钟
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;			  //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;		  //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; 	  //高速50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;			  //上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);				  //初始化


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;			  //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;		  //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; 	  //高速50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;			  //上拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);				  //初始化
	
    //UsartNVIC configuration //UsartNVIC配置
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQ通道使能	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    //Initialize the VIC register with the specified parameters 
	//根据指定的参数初始化VIC寄存器 	
	NVIC_Init(&NVIC_InitStructure);
	
    //USART Initialization Settings 初始化设置
	USART_InitStructure.USART_BaudRate = bound; //Port rate //串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //一个停止
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //Sending and receiving mode //收发模式
    USART_Init(UART5, &USART_InitStructure); 	 //Initialize serial port 3 //初始化串口3
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
	USART_Cmd(UART5, ENABLE);					 //Enable serial port 3 //使能串口3 
  }

/**************************************************************************
函数功能：串口3接收中断
入口参数：无
返回  值：无
**************************************************************************/

int USART3_IRQHandler(void)
{	
  char v =USART_ReceiveData(USART3);
  return 0;	
}

/**************************************************************************
函数功能：串口4接收中断
入口参数：无
返回  值：无
**************************************************************************/
int UART4_IRQHandler(void)
{	
	static u8 Count=0;
	u8 Usart_Receive;
	u32 now = getSysTickCnt();

	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)  //判断是否接收到数据
	{
		Usart_Receive = USART_ReceiveData(UART4);//读取数据
           Receive_Data.buffer[Count]=Usart_Receive;
		if(Usart_Receive == FRAME_HEADER||Count>0) Count++; else Count=0;
		if (Count == 11)	//验证数据包的长度
		{   
				Count=0;//重新开始接收
				if(Receive_Data.buffer[10] == FRAME_TAIL) //验证数据包的尾部校验信息
				{
					if(Receive_Data.buffer[9] ==Check_Sum(9,0))	 //数据校验位计算，模式0是发送数据校验
				  {		
						tr800_runtime_note_serial_cmd(
							XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]),
							XYZ_Target_Speed_transition(Receive_Data.buffer[5],Receive_Data.buffer[6]),
							XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]),
							now);
						
				  }
			}
		}
	} 
return 0;	
}

int UART5_IRQHandler(void)
{	
	u8 Usart_Receive;
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)  //判断是否接收到数据
	{
		Usart_Receive = USART_ReceiveData(UART5);//读取数据
	} 
  return 0;	
}
/**************************************************************************
*  函数功能：串口1初始化
*
*  入口参数：无
*
*  返 回 值：无
**************************************************************************/
void uart1_init(u32 bound)
{	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	 //Enable the gpio clock //使能GPIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //Enable the Usart clock //使能USART时钟

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10 ,GPIO_AF_USART1);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;			  //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;		  //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; 	  //高速50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;			  //上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);					  //初始化
	
  //UsartNVIC configuration //UsartNVIC配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	//Subpriority //子优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQ通道使能
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
  //Initialize the VIC register with the specified parameters 
	//根据指定的参数初始化VIC寄存器 
	NVIC_Init(&NVIC_InitStructure); 
	
  //USART Initialization Settings 初始化设置
	USART_InitStructure.USART_BaudRate = bound; //Port rate //串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //Sending and receiving mode //收发模式
	USART_Init(USART1, &USART_InitStructure); //Initialize serial port 1 //初始化串口1
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
	USART_Cmd(USART1, ENABLE);					   //Enable serial port 1 //使能串口1
}

/**************************************************************************
*  函数功能：串口1接收中断
*
*  入口参数：无
*
*  返 回 值：无
**************************************************************************/
int USART1_IRQHandler(void)
{	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //接收到数据
	{
		(void)USART_ReceiveData(USART1);
	}
		return 0;	

}
/**************************************************************************
函数功能：串口3发送任务函数
入口参数：无
返回  值：无
**************************************************************************/
void data_task(void *pvParameters)
{
   u32 lastWakeTime = getSysTickCnt();
	while(1)
    {	
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_40_HZ));//此任务以20Hz的频率运行
		data_transition(); //对要进行发送的数据进行赋值
			USART4_SEND();     //UART4 serial status output

	  
	}
}
#if TR800_OPEN_LOOP_PWM_TEST || TR800_MOTOR_TELEMETRY_DEBUG || TR800_CONTROL_TELEMETRY_DEBUG
static short tr800_i16_limit_long(long value)
{
	if(value > 32767) return 32767;
	if(value < -32768) return -32768;
	return (short)value;
}
#endif

/**************************************************************************
函数功能：串口发送的数据进行赋值
入口参数：无
返回  值：无
**************************************************************************/
void data_transition(void)
{
	Send_Data.Sensor_Str.Frame_Header = FRAME_HEADER; //帧头
	Send_Data.Sensor_Str.Frame_Tail = FRAME_TAIL; //帧尾
	Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder+MOTOR_B.Encoder)/2)*1000; //小车x轴速度
	Send_Data.Sensor_Str.Y_speed = 0;
	Send_Data.Sensor_Str.Z_speed = ((MOTOR_B.Encoder-MOTOR_A.Encoder)/(Wheel_spacing)*1000);//小车z轴速度
#if TR800_OPEN_LOOP_PWM_TEST
	Send_Data.Sensor_Str.Accelerometer.X_data = tr800_i16_limit_long(OriginalEncoder.A);
	Send_Data.Sensor_Str.Accelerometer.Y_data = tr800_i16_limit_long(OriginalEncoder.B);
	Send_Data.Sensor_Str.Accelerometer.Z_data = tr800_i16_limit_long((long)MOTOR_A.Motor_Pwm);
	Send_Data.Sensor_Str.Gyroscope.X_data = tr800_i16_limit_long((long)MOTOR_B.Motor_Pwm);
	Send_Data.Sensor_Str.Gyroscope.Y_data = tr800_i16_limit_long((long)RC_pid);
	Send_Data.Sensor_Str.Gyroscope.Z_data = tr800_open_loop_state_debug;
#elif TR800_MOTOR_TELEMETRY_DEBUG
	Send_Data.Sensor_Str.Accelerometer.X_data = tr800_i16_limit_long((long)RC_LX);
	Send_Data.Sensor_Str.Accelerometer.Y_data = tr800_i16_limit_long((long)PWMA);
	Send_Data.Sensor_Str.Accelerometer.Z_data = tr800_i16_limit_long((long)PWMB);
	Send_Data.Sensor_Str.Gyroscope.X_data = (short)(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_2) * 10 +
		GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_3));
	Send_Data.Sensor_Str.Gyroscope.Y_data = (short)(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_5) * 10 +
		GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_4));
	Send_Data.Sensor_Str.Gyroscope.Z_data = tr800_i16_limit_long((long)tr800_runtime_get_fault_flags());
#elif TR800_CONTROL_TELEMETRY_DEBUG
	{
		static u8 control_diag_page = 0;
		control_diag_page++;
		if(control_diag_page > 3) control_diag_page = 1;

		Send_Data.Sensor_Str.Accelerometer.X_data = control_diag_page;
		if(control_diag_page == 1)
		{
			Send_Data.Sensor_Str.Accelerometer.Y_data = tr800_i16_limit_long((long)RC_LX);
			Send_Data.Sensor_Str.Accelerometer.Z_data = tr800_i16_limit_long((long)RC_LY);
			Send_Data.Sensor_Str.Gyroscope.X_data = tr800_i16_limit_long((long)RC_RX);
			Send_Data.Sensor_Str.Gyroscope.Y_data = tr800_i16_limit_long((long)RC_RY);
			Send_Data.Sensor_Str.Gyroscope.Z_data = tr800_i16_limit_long((long)RC_Speed);
		}
		else if(control_diag_page == 2)
		{
			Send_Data.Sensor_Str.Accelerometer.Y_data = tr800_i16_limit_long((long)(Move_X * 1000.0f));
			Send_Data.Sensor_Str.Accelerometer.Z_data = tr800_i16_limit_long((long)(Move_Z * 1000.0f));
			Send_Data.Sensor_Str.Gyroscope.X_data = tr800_i16_limit_long((long)(MOTOR_A.Target * 1000.0f));
			Send_Data.Sensor_Str.Gyroscope.Y_data = tr800_i16_limit_long((long)(MOTOR_B.Target * 1000.0f));
			Send_Data.Sensor_Str.Gyroscope.Z_data = (short)(tr800_runtime_get_system_state() * 100 +
				tr800_runtime_get_requested_mode() * 10 + tr800_runtime_get_active_mode());
		}
		else
		{
			Send_Data.Sensor_Str.Accelerometer.Y_data = tr800_i16_limit_long((long)(MOTOR_A.Encoder * 1000.0f));
			Send_Data.Sensor_Str.Accelerometer.Z_data = tr800_i16_limit_long((long)(MOTOR_B.Encoder * 1000.0f));
			Send_Data.Sensor_Str.Gyroscope.X_data = tr800_i16_limit_long((long)MOTOR_A.Motor_Pwm);
			Send_Data.Sensor_Str.Gyroscope.Y_data = tr800_i16_limit_long((long)MOTOR_B.Motor_Pwm);
			Send_Data.Sensor_Str.Gyroscope.Z_data = tr800_i16_limit_long((long)tr800_runtime_get_fault_flags());
		}
	}
#elif TR800_RC_TELEMETRY_DEBUG
	Send_Data.Sensor_Str.Accelerometer.X_data = RC_RY;  // CH1
	Send_Data.Sensor_Str.Accelerometer.Y_data = RC_LX;  // CH2
	Send_Data.Sensor_Str.Accelerometer.Z_data = RC_RX;  // CH3
	Send_Data.Sensor_Str.Gyroscope.X_data = RC_LY;      // CH4
	Send_Data.Sensor_Str.Gyroscope.Y_data = RC_pid;     // CH5
	Send_Data.Sensor_Str.Gyroscope.Z_data = (short)(tr800_runtime_get_system_state() * 100 +
		tr800_runtime_get_requested_mode() * 10 + tr800_runtime_get_active_mode());
#else
	//加速度计三轴加速度
	Send_Data.Sensor_Str.Accelerometer.X_data= accel[1]; //加速度计Y轴转换到ROS坐标X轴
	Send_Data.Sensor_Str.Accelerometer.Y_data=-accel[0]; //加速度计X轴转换到ROS坐标Y轴
	Send_Data.Sensor_Str.Accelerometer.Z_data= accel[2];
	
	//角速度计三轴角速度
	Send_Data.Sensor_Str.Gyroscope.X_data= gyro[1]; //角速度计Y轴转换到ROS坐标X轴
	Send_Data.Sensor_Str.Gyroscope.Y_data=-gyro[0]; //角速度计X轴转换到ROS坐标Y轴
	if(Flag_Stop==0) Send_Data.Sensor_Str.Gyroscope.Z_data=gyro[2];  //如果电机控制位使能状态，那么正常发送Z轴角速度
	else             Send_Data.Sensor_Str.Gyroscope.Z_data=0;       //如果机器人是静止的（电机控制位失能），那么发送的Z轴角速度为0
#endif
	
	Send_Data.Sensor_Str.Power_Voltage = Voltage*1000; //电池电压(这里将浮点数放大一千倍传输，相应的在接收端在接收到数据后也会缩小一千倍)
	
	Send_Data.buffer[0]=Send_Data.Sensor_Str.Frame_Header; //帧头(固定值)
    Send_Data.buffer[1]=Flag_Stop;//电机状态
	
	Send_Data.buffer[2]=Send_Data.Sensor_Str.X_speed >>8; //小车x轴速度
	Send_Data.buffer[3]=Send_Data.Sensor_Str.X_speed ;    //小车x轴速度
	Send_Data.buffer[4]=Send_Data.Sensor_Str.Y_speed>>8;  //小车y轴速度
	Send_Data.buffer[5]=Send_Data.Sensor_Str.Y_speed;     //小车y轴速度
	Send_Data.buffer[6]=Send_Data.Sensor_Str.Z_speed >>8; //小车z轴速度
	Send_Data.buffer[7]=Send_Data.Sensor_Str.Z_speed ;    //小车z轴速度
	
	Send_Data.buffer[8]=Send_Data.Sensor_Str.Accelerometer.X_data>>8; //加速度计三轴加速度
	Send_Data.buffer[9]=Send_Data.Sensor_Str.Accelerometer.X_data;    //加速度计三轴加速度
	Send_Data.buffer[10]=Send_Data.Sensor_Str.Accelerometer.Y_data>>8;
	Send_Data.buffer[11]=Send_Data.Sensor_Str.Accelerometer.Y_data;
	Send_Data.buffer[12]=Send_Data.Sensor_Str.Accelerometer.Z_data>>8;
	Send_Data.buffer[13]=Send_Data.Sensor_Str.Accelerometer.Z_data;
	
	Send_Data.buffer[14]=Send_Data.Sensor_Str.Gyroscope.X_data>>8; //角速度计三轴角速度
	Send_Data.buffer[15]=Send_Data.Sensor_Str.Gyroscope.X_data; //角速度计三轴角速度
	Send_Data.buffer[16]=Send_Data.Sensor_Str.Gyroscope.Y_data>>8;
	Send_Data.buffer[17]=Send_Data.Sensor_Str.Gyroscope.Y_data;
	Send_Data.buffer[18]=Send_Data.Sensor_Str.Gyroscope.Z_data>>8;
	Send_Data.buffer[19]=Send_Data.Sensor_Str.Gyroscope.Z_data;
	Send_Data.buffer[20]=Send_Data.Sensor_Str.Power_Voltage >>8; //电池电压
	Send_Data.buffer[21]=Send_Data.Sensor_Str.Power_Voltage; //电池电压
	#if PROTOCOL_25
	if(Flag_serial_cmd == 1){ 
	   Send_Data.buffer[22]=Send_Data.Sensor_Str.Serialcmd;
	   Flag_serial_cmd = 0;
	  }
	else Send_Data.buffer[22] = 0;
	Send_Data.buffer[23]=Check_Sum(23,1); //数据校验位计算，模式1是发送数据校验
	Send_Data.buffer[24]=Send_Data.Sensor_Str.Frame_Tail;//帧尾（固定值）
	#else
    Send_Data.buffer[22]=Check_Sum(22,1); //数据校验位计算，模式1是发送数据校验
	Send_Data.buffer[23]=Send_Data.Sensor_Str.Frame_Tail;//帧尾（固定值）
	#endif
	
}
/**************************************************************************
函数功能：串口3发送数据
入口参数：无
返回  值：无
**************************************************************************/
uint32_t previous_mavlink_time = 0;
#define MAVLINK_RATE 1        //hz
extern Encoder OriginalEncoder;     //编码器原始数据
void USART3_SEND(void)
{

#if MAVLINK
 if ((getSysTickCnt() - previous_mavlink_time) >= (1000 / MAVLINK_RATE))
 	{
 
		uint16_t checksum,mav_voltage;
		int32_t alt,relative_alt;
		int8_t i;
	 	float  pitch,roll,ax,ay,az;
		#if 0
        int32_t lat,lon;
		int16_t vx;
		int8_t battery_remaining;
		#endif

	    ax = accel[0]/1671.0f; 
	    ay = accel[1]/1671.0f;
	    az = accel[2]/1671.0f;
		
	    roll   =  atan2f(ax, -az);
        pitch  =  atan2f(ay, -az);

        if(pitch > 0 )pitch = 3.1415926f -pitch;
        else 		 pitch =  -pitch - 3.1415926f;
        
        if(roll > 0 )roll = 3.1415926f -roll;
        else 		 roll = -3.1415926f -roll;
		 
		 memcpy(&imu[10],(char *)&roll,4);
		 memcpy(&imu[14],(char *)&pitch,4);
		 
		 checksum = crc_calculate((const uint8_t*)&imu[1], 5);
		 crc_accumulate_buffer(&checksum, (const char *)&imu[6], imu[1]);
		 crc_accumulate(mavlink_message_crcs[imu[5]], &checksum);
		 memcpy(&imu[34],(char *)&checksum,2);
		 
		  for(i=0;i<sizeof(imu)-1;i++)
		 {

			while((USART3->SR&0X40)==0);
				USART3->DR = imu[i]; 	 
		 }
       Voltage = 12.6;
	   mav_voltage = Voltage*1000;
	   memcpy(&battery[20],(char *)&mav_voltage,2);
	   checksum = crc_calculate((const uint8_t*)&battery[1], 5);
	   crc_accumulate_buffer(&checksum, (const char *)&battery[6], battery[1]);
	   crc_accumulate(mavlink_message_crcs[battery[5]], &checksum);
	   memcpy(&battery[37],(char *)&checksum,2);
	   for(i=0;i<sizeof(battery)-1;i++)
		   {
 
			 while((USART3->SR&0X40)==0);
					USART3->DR = battery[i]; 	 
		   }
          alt = (OriginalEncoder.E - OriginalEncoder.F)*500*Wheel_perimeter/Encoder_precision;
	      relative_alt = ((MOTOR_A.Encoder+MOTOR_B.Encoder)/2)*1000; 
		 //memcpy(&gps[10],(char *)&lat,4);
		 //memcpy(&gps[14],(char *)&lon,4);
		 memcpy(&gps[18],(char *)&alt,4);
		 memcpy(&gps[22],(char *)&relative_alt,4);
		 //memcpy(&gps[26],(char *)&vx,2);
	  
		 checksum = crc_calculate((const uint8_t*)&gps[1], 5);
		 crc_accumulate_buffer(&checksum, (const char *)&gps[6], gps[1]);
		 crc_accumulate(mavlink_message_crcs[gps[5]], &checksum);
		 memcpy(&gps[34],(char *)&checksum,2);
		 
		 for(i=0;i<sizeof(gps)-1;i++)
			 {
	  
			   while((USART3->SR&0X40)==0);
					  USART3->DR = gps[i];	
			 }
	  
			 previous_mavlink_time =getSysTickCnt();
   }
#endif

}
/**************************************************************************
函数功能：串口发送数据
入口参数：无
返回  值：无
**************************************************************************/
void USART1_SEND(void)
{
  unsigned char i = 0;	
	#if PROTOCOL_25
	for(i=0; i<25; i++)
	#else
    for(i=0; i<24; i++)
	#endif
	{
		usart1_send(Send_Data.buffer[i]);
	}	 
}

void USART4_SEND(void)
{
  unsigned char i = 0;	
	#if PROTOCOL_25
	for(i=0; i<25; i++)
	#else
    for(i=0; i<24; i++)
	#endif
	{
	   uart4_send(Send_Data.buffer[i]);
	}	 
}

/**************************************************************************
函数功能：计算发送的数据校验位
入口参数：
返回  值：检验位
**************************************************************************/
u8 Check_Sum(unsigned char Count_Number,unsigned char Mode)
{
	unsigned char check_sum=0,k;
	//发送数据的校验
	if(Mode==1)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Send_Data.buffer[k];
	}
	//接收数据的校验
	if(Mode==0)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Receive_Data.buffer[k];
	}
	return check_sum;
}

/**************************************************************************
函数功能：将上位机发过来的高8位和低8位数据整合成一个short型数据后，再做单位还原换算
入口参数：高8位，低8位
返回  值：机器人X/Y/Z轴的目标速度
**************************************************************************/
float XYZ_Target_Speed_transition(u8 High,u8 Low)
{
		short transition; //数据转换的中间变量
		transition=((High<<8)+Low); //将高8位和低8位整合成一个16位的short型数据
		return transition/1000+(transition%1000)*0.001;    //发送端将数据发送前做了一个*1000的单位换算，这里接收数据后需要还原单位
}

/**************************************************************************
*  函数功能：CAN发送数据
*
*  入口参数：无
*
*  返 回 值：无
**************************************************************************/
void CAN_SEND(void) 
{
	u8 CAN_SENT[8],i;
	
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i];
	}
	CAN1_Send_Num(0x101,CAN_SENT);
	
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i+8];
	}
	CAN1_Send_Num(0x102,CAN_SENT);
	
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i+16];
	}
	CAN1_Send_Num(0x103,CAN_SENT);
}

void ibus_begin(void)
{
  FlySkyIBus.state = DISCARD;
  FlySkyIBus.last = getSysTickCnt();//millis();
  FlySkyIBus.ptr = 0;
  FlySkyIBus.len = 0;
  FlySkyIBus.chksum = 0;
  FlySkyIBus.lchksum = 0;
  //this->Channel = {0};
  memset(FlySkyIBus.Channel,0,sizeof(u16)*16);
  FlySkyIBus.Channel[4] = 1500;
  
}

void loop(void)
{
  {
	u32 now = getSysTickCnt(); // millis();
	u8 v = 0;
	if (now - FlySkyIBus.last >= PROTOCOL_TIMEGAP)
	{
	  FlySkyIBus.state = GET_LENGTH;
	}
	FlySkyIBus.last = now;
	
    v =USART_ReceiveData(USART2);
	
	switch (FlySkyIBus.state)
	{
	  case GET_LENGTH:
		if (v <= PROTOCOL_LENGTH)
		{
		  FlySkyIBus.ptr = 0;
		  FlySkyIBus.len = v - PROTOCOL_OVERHEAD;
		  FlySkyIBus.chksum = 0xFFFF - v;
		  FlySkyIBus.state = GET_DATA;
		}
		else
		{
		  FlySkyIBus.state = DISCARD;
		}
		break;

	  case GET_DATA:
		FlySkyIBus.buffer[FlySkyIBus.ptr++] = v;
		FlySkyIBus.chksum -= v;
		if (FlySkyIBus.ptr == FlySkyIBus.len)
		{
		  FlySkyIBus.state = GET_CHKSUML;
		}
		break;
		
	  case GET_CHKSUML:
		FlySkyIBus.lchksum = v;
		FlySkyIBus.state = GET_CHKSUMH;
		break;

	  case GET_CHKSUMH:
		if (FlySkyIBus.chksum == (v << 8) + FlySkyIBus.lchksum)
		{
			u8 i; 
		  // Execute command - we only know command 0x40
		  switch (FlySkyIBus.buffer[0])
		  {
			case PROTOCOL_COMMAND40:
			  // Valid - extract channel data
			  for ( i = 1; i < PROTOCOL_CHANNELS * 2 + 1; i += 2)
			  {
				FlySkyIBus.channel[i / 2] = FlySkyIBus.buffer[i] | (FlySkyIBus.buffer[i + 1] << 8);
			  }
			  break;

			default:
			  break;
		  }
		}
		FlySkyIBus.state = DISCARD;
		break;

	  case DISCARD:
	  default:
		break;
	}
//	USART_ClearITPendingBit(USART3, USART_IT_RXNE);
  }
}

u16 readChannel(u8 channelNr)
{
  if (channelNr < PROTOCOL_CHANNELS)
  {
    return FlySkyIBus.channel[channelNr];
  }
  else
  {
    return 0;
  }
}

void Relay_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIOB时钟
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7;//LED对应IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化GPIO
  GPIO_ResetBits(GPIOD,GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7);

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOB时钟
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;//LED对应IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIO
  GPIO_ResetBits(GPIOE,GPIO_Pin_3);
}


u32 bias_time = 0;
void RC_Read(void)
{
    #if IBUS_EN
	int i =0;
	for( i = 0;i< 10;i++)   FlySkyIBus.Channel[i] = readChannel(i);
    RC_RY        =   FlySkyIBus.Channel[0];
    RC_LX        =   FlySkyIBus.Channel[1];
	RC_RX        =   FlySkyIBus.Channel[2];
    RC_LY        =   FlySkyIBus.Channel[3];
    RC_power     =   FlySkyIBus.Channel[4];
	RC_Speed     =   FlySkyIBus.Channel[6];
	if(FrontAdjust == 1)     RC_Bias      =   FlySkyIBus.Channel[8];	
	else if(BackAdjust == 1) RC_Bias      =   FlySkyIBus.Channel[9];
	else                     RC_pid       =   RC_power;
	#elif SBUS_EN
	u32 cnt;
	FeedLine();
	if(FutabaSbus.toChannels == 1)
	  {
	   control_sig++;
	   rc_sig_lost = 0;
	   UpdateServos();
	   UpdateChannels();
	   FutabaSbus.toChannels = 0;
	   RC_RY	    =   FutabaSbus.channels[0];
	   RC_RX        =   FutabaSbus.channels[1];
       RC_LX		=   FutabaSbus.channels[2];
       RC_LY        =   FutabaSbus.channels[3];

	   #if 0//MK15
	   RC_pid       =   FutabaSbus.channels[4];
	   RC_Speed     =   FutabaSbus.channels[5];  //SB
	   RC_SW1       =   FutabaSbus.channels[9];
       RC_power     =   FutabaSbus.channels[10];
       RC_S1        =   FutabaSbus.channels[12];
	   RC_S2        =   FutabaSbus.channels[11];
	   RC_Light     =   FutabaSbus.channels[7];
	   #if YUNTAI_EN
	   RC_Zoom      =   FutabaSbus.channels[5];
	   RC_Speed     =   FutabaSbus.channels[6];
	   #else
       RC_Speed     =   FutabaSbus.channels[5];
	   #endif
	   if(FrontAdjust == 1)     RC_Bias      =   map(FutabaSbus.channels[11],200,1800,1000,2000);
	   else if(BackAdjust == 1) RC_Bias      =   map(FutabaSbus.channels[12],200,1800,1000,2000);

       //Velocity_KP =  map(FutabaSbus.channels[11],200,1800,50,1000);
       //Velocity_KI =  map(FutabaSbus.channels[12],200,1800,50,1000);
	   #else
       RC_pid       =   FutabaSbus.channels[4];  //SA
       RC_SB       =   FutabaSbus.channels[5];   //SA
       RC_SD       =   FutabaSbus.channels[7];   //SA
	       // RC_SW1       =   FutabaSbus.channels[9];  //SF
       RC_power     =   FutabaSbus.channels[8];  //SE
       RC_S1        =   FutabaSbus.channels[15]; //LD2
	   RC_S2        =   FutabaSbus.channels[14]; //RD2
       RC_Speed     =   FutabaSbus.channels[9];  //SB
	   RC_Light     =   FutabaSbus.channels[12]; //S2
	   RC_Zoom      =   FutabaSbus.channels[6];  //SC
	   RC_Home      =   FutabaSbus.channels[13];


       #if BIAS_MODE_0
	   if(FrontAdjust == 1)     RC_Bias =  map(FutabaSbus.channels[10],200,1800,1000,2000);
	   else if(BackAdjust == 1) RC_Bias =  map(FutabaSbus.channels[11],200,1800,1000,2000);
       #else
	   if(FrontAdjust == 1)   Pulsewidth_Bias_f = map(FutabaSbus.channels[10],200,1800,1000,2000);
	   if(BackAdjust == 1)	  Pulsewidth_Bias_b = map(FutabaSbus.channels[11],200,1800,1000,2000);
       #endif  
       else{
               RC_Servo1 = map(FutabaSbus.channels[10],272,1712,500,2500);
	           RC_Servo2 = map(FutabaSbus.channels[11],272,1712,500,2500);
			   if(RC_Servo1 > 2500) RC_Servo1 = 2500;
			   else if(RC_Servo1 < 500) RC_Servo1 = 500;
			   if(RC_Servo2 > 2500) RC_Servo2 = 2500;
			   else if(RC_Servo2 < 500) RC_Servo2 = 500;
			   TIM1->CCR1=RC_Servo1; 
			   TIM1->CCR2=RC_Servo2; 

	   }

	   #endif
	   
	   #if YUNTAI_EN
	   Yuntai_move_control();
	   if(abs(RC_Home - RC_Home_ ) > 200)
		  {
              if(RC_Home > RC_MID+RC_ACT)
              	{
                    for(int i = 0; i<7 ;i++ )  usart5_send(yuntai_home[i]);   	
			    }
           RC_Home_ = RC_Home;
		  }
	   #endif
    // Legacy relay/brake side effects are intentionally disabled in the first-stage path.

   
		  }
	else if(rc_sig_lost >=2){
         RC_RY  =  RC_MID;
		 RC_RX	=  RC_MID;
		 RC_LX	=  RC_MID;
		 RC_LY	=  RC_MID;
	}
	#endif
    if( bias_adjust_mode == false && FutabaSbus.channels[14] > 1700  && FutabaSbus.channels[15] > 1700)
       {
           bias_time++;
		   if(bias_time >= 400) bias_adjust_mode = true, bias_time =0,Beep_Control_Flag =1;
	   }
    else if( bias_adjust_mode == true && FutabaSbus.channels[14] < 300  && FutabaSbus.channels[15] < 300)
       {
           bias_time++;
		   if(bias_time >= 100)  bias_adjust_mode = false,bias_time=0,Beep_Control_Flag =1;
		  
	   }
	else bias_time = 0;

    if(bias_adjust_mode == true)
      {
         if(FrontAdjust == 0 && RC_SB > 1700)
     	{
             FrontAdjust = 1;
     		 EnterProjectMode1 = 1; 
			 Beep_Control_Flag =1;			
          }
         else if(FrontAdjust == 1 && RC_SB < 300)  EnterProjectMode1 = 0,Beep_Control_Flag =1;
     
         if(BackAdjust == 0 && RC_SD > 1700)
     	  {
              BackAdjust = 1;
     		  EnterProjectMode2 = 1; 
		      Beep_Control_Flag =1;			
           }
         else if(BackAdjust == 1 && RC_SD < 300)  EnterProjectMode2 = 0,Beep_Control_Flag =1;
     
   }

}  


#if 1
void Yuntai_move_control(void)
{
   yuntai_v_speed = map(abs(992-RC_S1),0,720,1,20);  //20 64
   yuntai_h_speed = map(abs(992-RC_S2),0,720,1,20);  //20 64
   if(yuntai_v_speed >= yuntai_h_speed)  yuntai_speed = yuntai_h_speed;
   else yuntai_speed = yuntai_v_speed;
   if( yuntai_h_speed != yuntai_h_speed_ ||  yuntai_v_speed != yuntai_v_speed_){
       yuntai_down[6]=yuntai_up[6] = yuntai_left[6] = yuntai_right[6]=1;
       yuntai_down_left[6]=yuntai_up_left[6] = yuntai_up_right[6] = yuntai_down_right[6]=1;
       yuntai_down[5] = yuntai_down[4]=yuntai_up[5] = yuntai_up[4] = yuntai_v_speed;
       yuntai_left[5] = yuntai_left[4]=yuntai_right[5] = yuntai_right[4] = yuntai_h_speed;
	   yuntai_down_left[4]=yuntai_up_left[4] = yuntai_up_right[4] = yuntai_down_right[4]=yuntai_speed;
       yuntai_down_left[5]=yuntai_up_left[5] = yuntai_up_right[5] = yuntai_down_right[5]=yuntai_speed;
	  
	   for(int i = 3; i<6 ;i++ ) 
	   	{
	   	    yuntai_up[6]+=yuntai_up[i];
	      	yuntai_down[6]+=yuntai_down[i];
			yuntai_left[6]+=yuntai_left[i];
			yuntai_right[6]+=yuntai_right[i];
			yuntai_up_left[6]+=yuntai_up_left[i];
            yuntai_down_left[6]+=yuntai_down_left[i];
			yuntai_up_right[6]+=yuntai_up_right[i];
            yuntai_down_right[6]+=yuntai_down_right[i];
	   	}
   	}

       if( yuntai_h_speed != yuntai_h_speed_ ||  yuntai_v_speed != yuntai_v_speed_)
        {
              if(RC_S1 > SBUS_MID + 20 )
              	{
	               if(RC_S2 > SBUS_MID + 20 )
				   {
	              	                    
	                  for(int i = 0; i<7 ;i++ )  usart5_send(yuntai_up_left[i]);
	               	}   
				  else if(RC_S2 < SBUS_MID - 20)
	              	{
	                   
					   for(int i = 0; i<7 ;i++ )  usart5_send(yuntai_up_right[i]);   						
				    }

                    else 
                    {					 		                    
					   for(int i = 0; i<7 ;i++ )  usart5_send(yuntai_up[i]);				   
					}
					
			    }
			  
			  else if(RC_S1 < SBUS_MID - 20)
              	{
                   
                  if(RC_S2 > SBUS_MID + 20 )
                    {
	                 
	                  for(int i = 0; i<7 ;i++ )  usart5_send(yuntai_down_left[i]);
                    }
				  else if(RC_S2 < SBUS_MID - 20)
	              	{
	                  
					   for(int i = 0; i<7 ;i++ )  usart5_send(yuntai_down_right[i]);   						 
				    }
                    else 
                    {					 
                      
					   for(int i = 0; i<7 ;i++ )  usart5_send(yuntai_down[i]);				   
					}

			    }
               else
              	{                          
                   if(RC_S2 > SBUS_MID + 20 )
              	   {
              	      
                       for(int i = 0; i<7 ;i++ )  usart5_send(yuntai_left[i]);
			       }
				  else if(RC_S2 < SBUS_MID - 20)
	              	   {
	                     
						   for(int i = 0; i<7 ;i++ )  usart5_send(yuntai_right[i]);   										                    
				        }
				  else
	              	{
	                 
					    for(int i = 0; i<7 ;i++ )  usart5_send(yuntai_stop[i]); 
					
				    }
			    } 
             
        yuntai_h_speed_= yuntai_h_speed;
		yuntai_v_speed_= yuntai_v_speed;

		}  


        if(RC_Zoom > SBUS_MID + SBUS_BIAS)
      	{
            if(Yuntai_Zoom != ZOOM_OUT)
            {
               Yuntai_Zoom = ZOOM_OUT;
          //     for(int i = 0; i<7 ;i++ )  usart5_send(yuntai_zoom_out[i]);
			}

	    }
	  else if(RC_Zoom < SBUS_MID - SBUS_BIAS)
      	{
            if(Yuntai_Zoom != ZOOM_IN)
            {
               Yuntai_Zoom = ZOOM_IN;
			//   for(int i = 0; i<7 ;i++ )  usart5_send(yuntai_zoom_in[i]);   
			}

	    }
	  else
      	{
            if(Yuntai_Zoom != STOP_Z)
            {
               Yuntai_Zoom = STOP_Z;
			   for(int i = 0; i<7 ;i++ )  usart5_send(yuntai_stop[i]); 
			}

	  	}


}
void Yuntai_zoom_control(void)
{	       
    static  uint32_t zooming_time = 0;
      if(Yuntai_Zoom == ZOOM_OUT && getSysTickCnt() - zooming_time < 15000)
      	{
            if(Yuntai_ZoomState == ZOOM_PAUSE) 
               {
                   for(int i = 0; i<7 ;i++ )  usart5_send(yuntai_zoom_out[i]);
                   Yuntai_ZoomState = ZOOM_ING;
			   }
			else if(Yuntai_ZoomState == ZOOM_ING) 
               {
                   for(int i = 0; i<7 ;i++ )  usart5_send(yuntai_stop[i]);
                   Yuntai_ZoomState = ZOOM_PAUSE;
			   }	
	    }
      else if(Yuntai_Zoom == ZOOM_IN && getSysTickCnt() - zooming_time < 15000)
      	{
            if(Yuntai_ZoomState == ZOOM_PAUSE) 
               {
                   for(int i = 0; i<7 ;i++ )  usart5_send(yuntai_zoom_in[i]);
                   Yuntai_ZoomState = ZOOM_ING;
			   }
			else if(Yuntai_ZoomState == ZOOM_ING) 
               {
                   for(int i = 0; i<7 ;i++ )  usart5_send(yuntai_stop[i]);
                   Yuntai_ZoomState = ZOOM_PAUSE;
			   }
	    }

      else if(Yuntai_Zoom == STOP_Z)
      	{
           Yuntai_ZoomState = ZOOM_PAUSE; 
           zooming_time =  getSysTickCnt();

	    }
	 
}
#endif
void sbus_begin(){ 
	u8 loc_sbusData[25] = {
	  0x0f,0x01,0x04,0x20,0x00,0xff,0x07,0x40,0x00,0x02,0x10,0x80,0x2c,0x64,0x21,0x0b,0x59,0x08,0x40,0x00,0x02,0x10,0x80,0x00,0x00};
	u16 loc_channels[18]  = {
	  		1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,0,0};
	u16 loc_servos[18]    = {
  			1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,0,0};
  	
	memcpy(FutabaSbus.sbusData,loc_sbusData,25);
	memcpy(FutabaSbus.channels,loc_channels,18);
	memcpy(FutabaSbus.servos,loc_servos,18);
	FutabaSbus.sbus_passthrough = 1;
	FutabaSbus.toChannels = 0;
	FutabaSbus.bufferIndex=0;
	FutabaSbus.feedState = 0;
}
u16 Channel(u8 ch) {
  // Read channel data
  if ((ch>0)&&(ch<=16)){
    return FutabaSbus.channels[ch-1];
  }
  else{
    return 1023;
  }
}
u8 DigiChannel(u8 ch) {
  // Read digital channel data
  if ((ch>0) && (ch<=2)) {
    return FutabaSbus.channels[15+ch];
  }
  else{
    return 0;
  }
}
void SbusServo(u8 ch, u16 position) {
  if ((ch>0)&&(ch<=16)) {
    if (position>2048) {
      position=2048;
    }
    FutabaSbus.servos[ch-1] = position;
  }
}
void DigiServo(u8 ch, u8 position) {
  // Set digital servo position
  if ((ch>0) && (ch<=2)) {
    if (position>1) {
      position=1;
    }
    FutabaSbus.servos[15+ch] = position;
  }
}
u8 Failsafe(void) {
  return FutabaSbus.failsafe_status;
}

void PassthroughSet(int mode) {
  // Set passtrough mode, if true, received channel data is send to servos
  FutabaSbus.sbus_passthrough = mode;
}

int PassthroughRet(void) {
  // Return current passthrough mode
  return FutabaSbus.sbus_passthrough;
}
void UpdateServos(void) {
  u8 i;
  if (FutabaSbus.sbus_passthrough==0) {
    // clear received channel data
    for (i=1; i<24; i++) {
      FutabaSbus.sbusData[i] = 0;
    }

    // reset counters
    FutabaSbus.ch = 0;
    FutabaSbus.bit_in_servo = 0;
    FutabaSbus.byte_in_sbus = 1;
    FutabaSbus.bit_in_sbus = 0;

    // store servo data
    for (i=0; i<176; i++) {
      if (FutabaSbus.servos[FutabaSbus.ch] & (1<<FutabaSbus.bit_in_servo)) {
        FutabaSbus.sbusData[FutabaSbus.byte_in_sbus] |= (1<<FutabaSbus.bit_in_sbus);
      }
      FutabaSbus.bit_in_sbus++;
      FutabaSbus.bit_in_servo++;

      if (FutabaSbus.bit_in_sbus == 8) {
        FutabaSbus.bit_in_sbus =0;
        FutabaSbus.byte_in_sbus++;
      }
      if (FutabaSbus.bit_in_servo == 11) {
        FutabaSbus.bit_in_servo =0;
        FutabaSbus.ch++;
      }
    }

    // DigiChannel 1
    if (FutabaSbus.channels[16] == 1) {
      FutabaSbus.sbusData[23] |= (1<<0);
    }
    // DigiChannel 2
    if (FutabaSbus.channels[17] == 1) {
      FutabaSbus.sbusData[23] |= (1<<1);
    }

    // Failsafe
    if (FutabaSbus.failsafe_status == SBUS_SIGNAL_LOST) {
      FutabaSbus.sbusData[23] |= (1<<2);
    }

    if (FutabaSbus.failsafe_status == SBUS_SIGNAL_FAILSAFE) {
      FutabaSbus.sbusData[23] |= (1<<2);
      FutabaSbus.sbusData[23] |= (1<<3);
    }
  }
}
void UpdateChannels(void) {
  
  FutabaSbus.channels[0]  = ((FutabaSbus.sbusData[1]|FutabaSbus.sbusData[2]<< 8) & 0x07FF);
  FutabaSbus.channels[1]  = ((FutabaSbus.sbusData[2]>>3|FutabaSbus.sbusData[3]<<5) & 0x07FF);
  FutabaSbus.channels[2]  = ((FutabaSbus.sbusData[3]>>6|FutabaSbus.sbusData[4]<<2|FutabaSbus.sbusData[5]<<10) & 0x07FF);
  FutabaSbus.channels[3]  = ((FutabaSbus.sbusData[5]>>1|FutabaSbus.sbusData[6]<<7) & 0x07FF);
  FutabaSbus.channels[4]  = ((FutabaSbus.sbusData[6]>>4|FutabaSbus.sbusData[7]<<4) & 0x07FF);
  FutabaSbus.channels[5]  = ((FutabaSbus.sbusData[7]>>7|FutabaSbus.sbusData[8]<<1|FutabaSbus.sbusData[9]<<9) & 0x07FF);
  FutabaSbus.channels[6]  = ((FutabaSbus.sbusData[9]>>2|FutabaSbus.sbusData[10]<<6) & 0x07FF);
  FutabaSbus.channels[7]  = ((FutabaSbus.sbusData[10]>>5|FutabaSbus.sbusData[11]<<3) & 0x07FF); // & the other 8 + 2 channels if you need them
  #ifdef ALL_CHANNELS
  FutabaSbus.channels[8]  = ((FutabaSbus.sbusData[12]|FutabaSbus.sbusData[13]<< 8) & 0x07FF);
  FutabaSbus.channels[9]  = ((FutabaSbus.sbusData[13]>>3|FutabaSbus.sbusData[14]<<5) & 0x07FF);
  FutabaSbus.channels[10] = ((FutabaSbus.sbusData[14]>>6|FutabaSbus.sbusData[15]<<2|FutabaSbus.sbusData[16]<<10) & 0x07FF);
  FutabaSbus.channels[11] = ((FutabaSbus.sbusData[16]>>1|FutabaSbus.sbusData[17]<<7) & 0x07FF);
  FutabaSbus.channels[12] = ((FutabaSbus.sbusData[17]>>4|FutabaSbus.sbusData[18]<<4) & 0x07FF);
  FutabaSbus.channels[13] = ((FutabaSbus.sbusData[18]>>7|FutabaSbus.sbusData[19]<<1|FutabaSbus.sbusData[20]<<9) & 0x07FF);
  FutabaSbus.channels[14] = ((FutabaSbus.sbusData[20]>>2|FutabaSbus.sbusData[21]<<6) & 0x07FF);
  FutabaSbus.channels[15] = ((FutabaSbus.sbusData[21]>>5|FutabaSbus.sbusData[22]<<3) & 0x07FF);
  #endif
  
  FutabaSbus.failsafe_status = SBUS_SIGNAL_OK;
  if (FutabaSbus.sbusData[23] & (1<<2)) {
    FutabaSbus.failsafe_status = SBUS_SIGNAL_LOST;
  }
  if (FutabaSbus.sbusData[23] & (1<<3)) {
    FutabaSbus.failsafe_status = SBUS_SIGNAL_FAILSAFE;
  }
}

void FeedLine(void)
{
  if(available() >24){
    while(available() > 0){ 
      FutabaSbus.inData = read();
      switch (FutabaSbus.feedState){
      case 0:
        if (FutabaSbus.inData != 0x0f){
          while(available() > 0){
            FutabaSbus.inData = read();
          }
          return;
        }
        else{
          FutabaSbus.bufferIndex = 0;
          FutabaSbus.inBuffer[FutabaSbus.bufferIndex] = FutabaSbus.inData;
          FutabaSbus.inBuffer[24] = 0xff;
          FutabaSbus.feedState = 1;
        }
        break;
      case 1:
        FutabaSbus.bufferIndex ++;
        FutabaSbus.inBuffer[FutabaSbus.bufferIndex] = FutabaSbus.inData;
        if (FutabaSbus.bufferIndex < 24 && available() == 0){
            FutabaSbus.feedState = 0;
        }
        if (FutabaSbus.bufferIndex == 24){
          FutabaSbus.feedState = 0;
          if (FutabaSbus.inBuffer[0]==0x0f && FutabaSbus.inBuffer[23] == 0x00){
            memcpy(FutabaSbus.sbusData,FutabaSbus.inBuffer,25);
            FutabaSbus.toChannels = 1;
          }
        }
        break;
      }
    }
  }
}

void EX_GPIO_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIOB时钟
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;//LED对应IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化GPIO
  GPIO_SetBits(GPIOD,GPIO_Pin_11 | GPIO_Pin_12| GPIO_Pin_13);

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOB时钟
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;//LED对应IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIO
  GPIO_SetBits(GPIOE,GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

}
void RC_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();
#if YUNTAI_EN
	u32 previous_zoom_time = getSysTickCnt();
#endif
	u32 now = 0;
    while(1)
    {	
      #if SBUS_EN
	  vTaskDelayUntil(&lastWakeTime, F2T(RATE_80_HZ));//此任务以80Hz的频率运行 	
	  #elif IBUS_EN
      vTaskDelayUntil(&lastWakeTime, F2T(RATE_50_HZ));//此任务以80Hz的频率运行 	
	  #endif
      RC_Read();
      now = getSysTickCnt();
      if((RC_pid < TR800_RC_MODE_SELECT_LOW) && tr800_runtime_rc_alive(now))
      {
      	rc_loop_control();
      }
      else
      {
      	tr800_runtime_invalidate_rc_cmd();
      }
      #if YUNTAI_EN
	  if(getSysTickCnt() - previous_zoom_time >= 200)
      	{
             Yuntai_zoom_control();
			 previous_zoom_time = getSysTickCnt();
	    }
	  #endif

   }	
}
