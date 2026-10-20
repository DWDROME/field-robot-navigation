#ifndef __SYSTEM_H
#define __SYSTEM_H

// Refer to all header files you need
//引用所有需要用到的头文件
#include "FreeRTOSConfig.h"
//FreeRTOS相关头文件 
//FreeRTOS related header files
#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
//The associated header file for the peripheral 
//外设的相关头文件
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "balance.h"
#include "led.h"
#include "oled.h"
#include "usartx.h"
#include "adc.h"
#include "can.h"
#include "motor.h"
#include "timer.h"
#include "encoder.h"
#include "show.h"								   
#include "key.h"
#include "robot_select_init.h"
//#include "I2C.h"
//#include "ICM20948.h"
//#include "mpu9250.h"
#include "I2C.h"
#include "MPU6050.h"
#include "Public_StdTypes.h"
#include "Public_StdMacros.h"
#include "stmflash.h"
#include "BiasAdjust.h"
#include "mavlink.h"

// Enumeration of car types
//小车型号的枚举定义
typedef enum 
{
	Mec_Car = 0, 
	Omni_Car, 
	Akm_Car, 
	Diff_Car, 
	FourWheel_Car, 
	Tank_Car
} CarMode;

//Motor speed control related parameters of the structure
//电机速度控制相关参数结构体
typedef struct  
{
	float Encoder;     //Read the real time speed of the motor by encoder //编码器数值，读取电机实时速度
	float Motor_Pwm;   //Motor PWM value, control the real-time speed of the motor //电机PWM数值，控制电机实时速度
	float Target;      //Control the target speed of the motor //电机目标速度值，控制电机目标速度
	float Velocity_KP; //Speed control PID parameters //速度控制PID参数
	float	Velocity_KI; //Speed control PID parameters //速度控制PID参数
}Motor_parameter;

//Smoothed the speed of the three axes
//平滑处理后的三轴速度
typedef struct  
{
	float VX;
	float VY;
	float VZ;
}Smooth_Control;

typedef enum
{
	TR800_BOOT_DELAY = 0,
	TR800_READY,
	TR800_RUN,
	TR800_FAULT
} TR800_SystemState;

typedef enum
{
	TR800_MODE_OFF = 0,
	TR800_MODE_SERIAL,
	TR800_MODE_RC
} TR800_ControlMode;

typedef struct
{
	TR800_SystemState system_state;
	TR800_ControlMode requested_mode;
	TR800_ControlMode active_mode;
	u8 enable_state;
	u8 fault_flags;
	u16 boot_ticks;
	u32 serial_last_rx_ms;
	u32 rc_last_rx_ms;
	float voltage;
	TR800_ControlMode active_cmd_source;
	u32 active_cmd_seq;
} TR800_RuntimeStatus;

typedef struct
{
	float x;
	float y;
	float z;
	u32 rx_ms;
	u32 seq;
	u8 valid;
} TR800_CommandSnapshot;

#define TR800_FAULT_ENABLE_LOW      0x01U
#define TR800_FAULT_LOW_VOLTAGE     0x02U
#define TR800_FAULT_HAND_STOP       0x04U
#define TR800_FAULT_HAND_STOP1      0x08U
#define TR800_FAULT_NO_CONTROL      0x10U
#define TR800_FAULT_HALL_STOP       0x20U
#define TR800_FAULT_STOP_MASK       (TR800_FAULT_HAND_STOP | TR800_FAULT_HAND_STOP1 | TR800_FAULT_NO_CONTROL | TR800_FAULT_HALL_STOP)
#define TR800_RC_MODE_SELECT_LOW    1300

/****** external variable definition. When system.h is referenced in other C files, 
        other C files can also use the variable defined by system.c           ******/
/****** 外部变量定义，当其它c文件引用system.h时，也可以使用system.c定义的变量 ******/
extern u8 Flag_Stop;
extern int Divisor_Mode;
extern u8 Car_Mode;
extern int Servo;
extern float RC_Velocity;
extern float Move_X, Move_Y, Move_Z; 
extern float Velocity_KP, Velocity_KI;	
extern Smooth_Control smooth_control;
extern Motor_parameter MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;
extern float Encoder_precision;
extern float Wheel_perimeter;
extern float Wheel_spacing; 
extern float Axle_spacing; 
extern float Omni_turn_radiaus; 
extern u8 PS2_ON_Flag, APP_ON_Flag, Remote_ON_Flag, CAN_ON_Flag, Usart_ON_Flag,Beep_Control_Flag;
extern u8 Flag_Left, Flag_Right, Flag_Direction, Turn_Flag; 
extern u8 PID_Send;                                            										                 
extern float PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY;
extern int RC_LX,RC_LY,RC_RX,RC_RY,RC_power,RC_navigate,RC_slam,RC_reboot,RC_Bias,RC_pid,RC_Servo,RC_S1,RC_S2,RC_Zoom,RC_Home,RC_SW1,RC_UD,RC_LR,RC_Small,RC_Big,RC_Speed,RC_Light;
extern int RC_SA,RC_SB,RC_SC,RC_SD;

extern int RC_Servo1,RC_Servo2;

extern int Check, Checking, Checked, CheckCount, CheckPhrase1, CheckPhrase2;
extern long int ErrorCode; 
extern TR800_RuntimeStatus TR800_Runtime;

void systemInit(void);
void tr800_runtime_init(void);
void tr800_runtime_request_mode(TR800_ControlMode mode);
void tr800_runtime_note_serial_rx(u32 now_ms);
void tr800_runtime_note_rc_rx(u32 now_ms);
void tr800_runtime_note_serial_cmd(float x, float y, float z, u32 now_ms);
void tr800_runtime_note_rc_cmd(float x, float y, float z, u32 now_ms);
void tr800_runtime_invalidate_rc_cmd(void);
u8 tr800_runtime_get_serial_cmd(TR800_CommandSnapshot *out, u32 now_ms);
u8 tr800_runtime_get_rc_cmd(TR800_CommandSnapshot *out, u32 now_ms);
void tr800_runtime_update(u32 now_ms, u16 boot_ticks, u8 enable_state, float voltage);
void tr800_runtime_apply_active_command(TR800_ControlMode mode, u32 cmd_seq);
void tr800_runtime_sync_stop_flags(u8 legacy_hand_stop, u8 legacy_hand_stop1, u8 legacy_no_control_stop, u8 hall_stop);
TR800_SystemState tr800_runtime_get_system_state(void);
TR800_ControlMode tr800_runtime_get_requested_mode(void);
TR800_ControlMode tr800_runtime_get_active_mode(void);
u8 tr800_runtime_get_fault_flags(void);
u8 tr800_runtime_stop_requested(void);
u16 tr800_runtime_get_boot_remaining_ticks(void);
const char *tr800_runtime_get_system_state_text(void);
const char *tr800_runtime_get_control_mode_text(void);
u8 tr800_runtime_serial_alive(u32 now_ms);
u8 tr800_runtime_rc_alive(u32 now_ms);

/***Macros define***/ /***宏定义***/
//After starting the car (1000/100Hz =10) for seconds, it is allowed to control the car to move
//开机(1000/100hz=10)秒后才允许控制小车进行运动
#define CONTROL_DELAY		1000
//The number of robot types to determine the value of Divisor_Mode. There are currently 6 car types
//机器人型号数量，决定Divisor_Mode的值，目前有6种小车类型
#define CAR_NUMBER      6      
#define RATE_1_HZ		1
#define RATE_5_HZ		5
#define RATE_10_HZ		10
#define RATE_20_HZ		20
#define RATE_40_HZ		40
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_80_HZ		80
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000
/***Macros define***/ /***宏定义***/

//C library function related header file
//C库函数的相关头文件
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"
#endif 
