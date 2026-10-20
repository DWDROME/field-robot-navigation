#ifndef __USRATX_H
#define __USRATX_H 

#include "stdio.h"
#include "sys.h"
#include "system.h"
//#include "stm32f10x_conf.h"
#include "string.h"
#include "robot_select_init.h"
#define DATA_STK_SIZE   512 
#define DATA_TASK_PRIO  4

#define RC_TASK_PRIO		4     //ÈÎÎñÓÅÏÈ¼¶
#define RC_STK_SIZE 		256   //ÈÎÎñ¶ÑÕ»´óÐ¡

#define FRAME_HEADER 0X7B //·¢ËÍÊý¾ÝµÄÖ¡Í·
#define FRAME_TAIL 0X7D //·¢ËÍÊý¾ÝµÄÖ¡Í·
#if PROTOCOL_25
#define SEND_DATA_SIZE 25
#else
#define SEND_DATA_SIZE 24
#endif
#define RECEIVE_DATA_SIZE 11


#define RELAY1 PDout(3)
#define RELAY2 PDout(4)
#define RELAY3 PDout(7)


#define cannon_up    PDout(11) 
#define cannon_down  PDout(12) 
#define cannon_left  PEout(14) 
#define cannon_right PEout(15) 
#define cannon_small PEout(13) 
#define cannon_big   PDout(13) 


#if IBUS_EN
#define RC_MID_POS  1525
#define RC_MID_NEG  1475
#define RC_LOW  1000
#define RC_MID  1500
#define RC_HIG  2000
#define RC_ACT  60
#else
#define RC_MID_POS  1017
#define RC_MID_NEG  967
#define RC_LOW  272
#define RC_MID  992
#define RC_HIG  1712
#define RC_ACT  60
#endif

#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03
#define ALL_CHANNELS


#define SERIAL_BUFFER_SIZE      125

//#pragma pack(1)
/*****ÓÃÓÚ´æ·ÅÍÓÂÝÒÇ¼ÓËÙ¶È¼ÆÈýÖáÊý¾Ý½á¹¹Ìå*****/
typedef struct __Mpu6050_Data_ 
{
	short X_data;//2¸ö×Ö½Ú
	short Y_data;//2¸ö×Ö½Ú
	short Z_data;//2¸ö×Ö½Ú
}Mpu6050_Data;
/*******´®¿Ú·¢ËÍÊý¾ÝµÄ½á¹¹Ìå************/
typedef struct _SEND_DATA_  
{
	unsigned char buffer[SEND_DATA_SIZE];
	struct _Sensor_Str_
	{
		unsigned char Frame_Header;//1¸ö×Ö½Ú
		short X_speed;	           //4¸ö×Ö½Ú
		short Y_speed;             //4¸ö×Ö½Ú
		short Z_speed;             //4¸ö×Ö½Ú
		short Power_Voltage;       //4¸ö×Ö½Ú
		
		Mpu6050_Data Accelerometer;//6¸ö×Ö½Ú
		Mpu6050_Data Gyroscope;    //6¸ö×Ö½Ú
		#if PROTOCOL_25
			unsigned char Serialcmd;   //1¸ö×Ö½Ú
		#endif
		unsigned char Frame_Tail;  //1¸ö×Ö½Ú
	}Sensor_Str;

}SEND_DATA;

typedef struct _RINGBUFFER_
{
    u8 _aucBuffer[SERIAL_BUFFER_SIZE];
    int _iHead ;
    int _iTail ;
}RINGBUFFER;

typedef struct _RECEIVE_DATA_  
{
	unsigned char buffer[RECEIVE_DATA_SIZE];
	struct _Control_Str_
	{
		unsigned char Frame_Header;// 1
		float X_speed;	          //4¸ö×Ö½Ú
		float Y_speed;            //4¸ö×Ö½Ú
		float Z_speed;            //4¸ö×Ö½Ú
		unsigned char Frame_Tail;     //1¸ö×Ö½Ú
	}Control_Str;

}RECEIVE_DATA;
#define PROTOCOL_LENGTH 0x20
#define PROTOCOL_OVERHEAD 3  // <len><cmd><data....><chkl><chkh>
#define PROTOCOL_TIMEGAP 3   // Packets are received very ~7ms so use ~half that for the gap
#define PROTOCOL_CHANNELS 10
#define PROTOCOL_COMMAND40 0x40  // Command is always 0x40
typedef enum 
 {
    GET_LENGTH,
    GET_DATA,
    GET_CHKSUML,
    GET_CHKSUMH,
    DISCARD
  }State;
typedef struct _FLYSKYIBUS_
{
  u16 Channel[16];
  u8 state;
  u32 last;
  u8 buffer[PROTOCOL_LENGTH];
  u8 ptr;
  u8 len;
  u16 channel[PROTOCOL_CHANNELS];
  u16 chksum;
  u8 lchksum;
  
}FLYSKYIBUS;

 void ibus_begin(void);
 void loop(void);
 u16 readChannel(u8 channelNr);
 extern volatile u32 ibus_rx_byte_count;
 extern volatile u32 ibus_valid_packet_count;
 extern volatile u32 ibus_bad_packet_count;

typedef struct _FUTABA_SBUS_
{
	u8 sbusData[25];
	u16 channels[18];
	u16 channels_old[18];
	u16 servos[18];
	u8  failsafe_status;
	int sbus_passthrough;
	int toChannels;
	u8 byte_in_sbus;
	u8 bit_in_sbus;
	u8 ch;
	u8 bit_in_channel;
	u8 bit_in_servo;
	u8 inBuffer[25];
	int bufferIndex;
	u8 inData;
	int feedState;

}FUTABA_SBUS;

void sbus_begin(void);
u16 Channel(u8 ch);
u8 DigiChannel(u8 ch);
void SbusServo(u8 ch, u16 position);
void DigiServo(u8 ch, u8 position);
u8 Failsafe(void);
void PassthroughSet(int mode);
int PassthroughRet(void);
void UpdateServos(void);
void UpdateChannels(void);
void FeedLine(void);

//#pragma pack(4)
void data_task(void *pvParameters);
void RC_task(void *pvParameters);
void USART4_SEND(void);
void USART3_SEND(void);
void USART1_SEND(void);
void data_transition(void);
void usart2_send(u8 data);
void uart2_init(u32 bound);
int USART2_IRQHandler(void);
void usart3_send(u8 data);
void usart1_send(u8 data);
void uart3_init(u32 bound);
void uart4_init(u32 bound);

void uart5_init(u32 bound);

int USART3_IRQHandler(void);
u8 Check_Sum(unsigned char Count_Number,unsigned char Mode);
void uart1_init(u32 bound);
int USART1_IRQHandler(void);
float Vz_to_Akm_Angle(float Vx, float Vz);
float XYZ_Target_Speed_transition(u8 High,u8 Low);
void CAN_SEND(void);
void putstr(const char *str);
void Relay_init(void);
void Yuntai_move_control(void);
void water_cannons_control(void);
void EX_GPIO_init(void);

#endif

