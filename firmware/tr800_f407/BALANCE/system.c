#include "system.h"

//Robot software fails to flag bits
//机器人软件失能标志位
u8 Flag_Stop=1;   

//The ADC value is variable in segments, depending on the number of car models. Currently there are 6 car models
//ADC值分段变量，取决于小车型号数量，目前有6种小车型号
int Divisor_Mode;

// Robot type variable
//机器人型号变量
//0=Mec_Car，1=Omni_Car，2=Akm_Car，3=Diff_Car，4=FourWheel_Car，5=Tank_Car
u8 Car_Mode=5; 

//Servo control PWM value, Ackerman car special
//舵机控制PWM值，阿克曼小车专用
int Servo;  

//Default speed of remote control car, unit: mm/s
//遥控小车的默认速度，单位：mm/s

float RC_Velocity=1000;  

//Vehicle three-axis target moving speed, unit: m/s
//小车三轴目标运动速度，单位：m/s
float Move_X, Move_Y, Move_Z;   

//PID parameters of Speed control
//速度控制PID参数
float Velocity_KP=500,Velocity_KI=200;   // reducer 20    (300 100)  
//float Velocity_KP=300,Velocity_KI=300; 

//Smooth control of intermediate variables, dedicated to omni-directional moving cars
//平滑控制中间变量，全向移动小车专用
Smooth_Control smooth_control;  

//The parameter structure of the motor
//电机的参数结构体
Motor_parameter MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;  

/************ 小车型号相关变量 **************************/
/************ Variables related to car model ************/
//Encoder accuracy
//编码器精度
float Encoder_precision; 
//Wheel circumference, unit: m
//轮子周长，单位：m
float Wheel_perimeter; 
//Drive wheel base, unit: m
//主动轮轮距，单位：m
float Wheel_spacing; 
//The wheelbase of the front and rear axles of the trolley, unit: m
//小车前后轴的轴距，单位：m
float Axle_spacing; 
//All-directional wheel turning radius, unit: m
//全向轮转弯半径，单位：m
float Omni_turn_radiaus; 
/************ 小车型号相关变量 **************************/
/************ Variables related to car model ************/

// Runtime control source display flags; first-stage control only uses SERIAL and RC.
// 运行时控制源显示标志；第一阶段外部控制入口只保留 SERIAL 和 RC。
u8 PS2_ON_Flag=0, APP_ON_Flag=0, Remote_ON_Flag=0, CAN_ON_Flag=0, Usart_ON_Flag=0; 
u8 Beep_Control_Flag;

//Bluetooth remote control associated flag bits
//蓝牙遥控相关的标志位
u8 Flag_Left, Flag_Right, Flag_Direction=0, Turn_Flag; 

//Sends the parameter's flag bit to the Bluetooth APP
//向蓝牙APP发送参数的标志位
u8 PID_Send; 

//The PS2 gamepad controls related variables
//PS2手柄控制相关变量
float PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY; 

//Self-check the relevant flag variables
//自检相关标志变量
int Check=0, Checking=0, Checked=0, CheckCount=0, CheckPhrase1=0, CheckPhrase2=0; 

//Check the result code
//自检结果代码
long int ErrorCode=0; 
int RC_LX=0,RC_LY,RC_RX,RC_RY=0,RC_power,RC_navigate,RC_slam,RC_reboot,RC_Bias,RC_pid,RC_S1,RC_S2,RC_Zoom,RC_Home,RC_SW1,RC_UD,RC_LR,RC_Small,RC_Big,RC_Speed,RC_Light; 
int RC_SA,RC_SB,RC_SC,RC_SD;
int RC_Servo1,RC_Servo2;

TR800_RuntimeStatus TR800_Runtime =
{
	TR800_BOOT_DELAY,
	TR800_MODE_OFF,
	TR800_MODE_OFF,
	1,
	0,
	0,
	0,
	0,
	0.0f,
	TR800_MODE_OFF,
	0
};

#define TR800_SERIAL_ALIVE_TIMEOUT_MS 2000U
#define TR800_RC_ALIVE_TIMEOUT_MS     2000U

static volatile TR800_CommandSnapshot TR800_SerialCmd;
static volatile TR800_CommandSnapshot TR800_RcCmd;

static void tr800_runtime_sync_legacy_flags(void)
{
	PS2_ON_Flag = 0;
	APP_ON_Flag = 0;
	CAN_ON_Flag = 0;

	Remote_ON_Flag = (TR800_Runtime.active_mode == TR800_MODE_RC) ? 1 : 0;
	Usart_ON_Flag = (TR800_Runtime.active_mode == TR800_MODE_SERIAL) ? 1 : 0;
}

void tr800_runtime_init(void)
{
	TR800_Runtime.system_state = TR800_BOOT_DELAY;
	TR800_Runtime.requested_mode = TR800_MODE_OFF;
	TR800_Runtime.active_mode = TR800_MODE_OFF;
	TR800_Runtime.enable_state = 1;
	TR800_Runtime.fault_flags = 0;
	TR800_Runtime.boot_ticks = 0;
	TR800_Runtime.serial_last_rx_ms = 0;
	TR800_Runtime.rc_last_rx_ms = 0;
	TR800_Runtime.voltage = 0.0f;
	TR800_Runtime.active_cmd_source = TR800_MODE_OFF;
	TR800_Runtime.active_cmd_seq = 0;
	TR800_SerialCmd.valid = 0;
	TR800_SerialCmd.seq = 0;
	TR800_SerialCmd.rx_ms = 0;
	TR800_RcCmd.valid = 0;
	TR800_RcCmd.seq = 0;
	TR800_RcCmd.rx_ms = 0;
	Flag_Stop = 1;
	tr800_runtime_sync_legacy_flags();
}

void tr800_runtime_request_mode(TR800_ControlMode mode)
{
	TR800_Runtime.requested_mode = mode;
}

void tr800_runtime_note_serial_rx(u32 now_ms)
{
	TR800_Runtime.serial_last_rx_ms = now_ms;
}

void tr800_runtime_note_rc_rx(u32 now_ms)
{
	TR800_Runtime.rc_last_rx_ms = now_ms;
}

static void tr800_runtime_write_cmd_unlocked(volatile TR800_CommandSnapshot *cmd, float x, float y, float z, u32 now_ms, u8 valid)
{
	cmd->seq++;
	cmd->x = x;
	cmd->y = y;
	cmd->z = z;
	cmd->rx_ms = now_ms;
	cmd->valid = valid;
	cmd->seq++;
}

static void tr800_runtime_write_cmd_from_task(volatile TR800_CommandSnapshot *cmd, float x, float y, float z, u32 now_ms, u8 valid)
{
	taskENTER_CRITICAL();
	tr800_runtime_write_cmd_unlocked(cmd, x, y, z, now_ms, valid);
	taskEXIT_CRITICAL();
}

static u8 tr800_runtime_read_cmd(volatile TR800_CommandSnapshot *cmd, TR800_CommandSnapshot *out, u32 now_ms, u32 timeout_ms)
{
	u32 seq_start;
	u32 seq_end;

	if (out == NULL)
	{
		return 0;
	}

	seq_start = cmd->seq;
	if (seq_start & 1U)
	{
		return 0;
	}
	out->x = cmd->x;
	out->y = cmd->y;
	out->z = cmd->z;
	out->rx_ms = cmd->rx_ms;
	out->valid = cmd->valid;
	out->seq = seq_start;
	seq_end = cmd->seq;
	if ((seq_start != seq_end) || ((seq_end & 1U) != 0U))
	{
		return 0;
	}

	if (!out->valid || out->rx_ms == 0)
	{
		return 0;
	}

	return ((now_ms - out->rx_ms) <= timeout_ms) ? 1 : 0;
}

void tr800_runtime_note_serial_cmd(float x, float y, float z, u32 now_ms)
{
	tr800_runtime_note_serial_rx(now_ms);
	tr800_runtime_write_cmd_unlocked(&TR800_SerialCmd, x, y, z, now_ms, 1);
}

void tr800_runtime_note_rc_cmd(float x, float y, float z, u32 now_ms)
{
	tr800_runtime_note_rc_rx(now_ms);
	tr800_runtime_write_cmd_from_task(&TR800_RcCmd, x, y, z, now_ms, 1);
}

void tr800_runtime_invalidate_rc_cmd(void)
{
	tr800_runtime_write_cmd_from_task(&TR800_RcCmd, 0.0f, 0.0f, 0.0f, 0, 0);
}

u8 tr800_runtime_get_serial_cmd(TR800_CommandSnapshot *out, u32 now_ms)
{
	return tr800_runtime_read_cmd(&TR800_SerialCmd, out, now_ms, TR800_SERIAL_ALIVE_TIMEOUT_MS);
}

u8 tr800_runtime_get_rc_cmd(TR800_CommandSnapshot *out, u32 now_ms)
{
	return tr800_runtime_read_cmd(&TR800_RcCmd, out, now_ms, TR800_RC_ALIVE_TIMEOUT_MS);
}

u8 tr800_runtime_serial_alive(u32 now_ms)
{
	if (TR800_Runtime.serial_last_rx_ms == 0)
	{
		return 0;
	}
	return ((now_ms - TR800_Runtime.serial_last_rx_ms) <= TR800_SERIAL_ALIVE_TIMEOUT_MS) ? 1 : 0;
}

u8 tr800_runtime_rc_alive(u32 now_ms)
{
	if (TR800_Runtime.rc_last_rx_ms == 0)
	{
		return 0;
	}
	return ((now_ms - TR800_Runtime.rc_last_rx_ms) <= TR800_RC_ALIVE_TIMEOUT_MS) ? 1 : 0;
}

void tr800_runtime_update(u32 now_ms, u16 boot_ticks, u8 enable_state, float voltage)
{
	u8 stop_faults = TR800_Runtime.fault_flags & TR800_FAULT_STOP_MASK;
	(void)now_ms;
	TR800_Runtime.boot_ticks = boot_ticks;
	TR800_Runtime.enable_state = enable_state;
	TR800_Runtime.voltage = voltage;
	TR800_Runtime.fault_flags = stop_faults;

	if (boot_ticks < CONTROL_DELAY)
	{
		TR800_Runtime.system_state = TR800_BOOT_DELAY;
	}
	else if (enable_state == 0)
	{
		TR800_Runtime.system_state = TR800_FAULT;
		TR800_Runtime.fault_flags |= TR800_FAULT_ENABLE_LOW;
	}
#if !TR800_DISABLE_LOW_VOLTAGE_FAULT
	else if (voltage < 40.0f)
	{
		TR800_Runtime.system_state = TR800_FAULT;
		TR800_Runtime.fault_flags |= TR800_FAULT_LOW_VOLTAGE;
	}
#endif
	else if (stop_faults)
	{
		TR800_Runtime.system_state = TR800_FAULT;
	}
	else
	{
		TR800_Runtime.system_state = (TR800_Runtime.active_mode == TR800_MODE_OFF) ? TR800_READY : TR800_RUN;
	}
}

void tr800_runtime_apply_active_command(TR800_ControlMode mode, u32 cmd_seq)
{
	if (TR800_Runtime.system_state != TR800_READY && TR800_Runtime.system_state != TR800_RUN)
	{
		TR800_Runtime.requested_mode = TR800_MODE_OFF;
		TR800_Runtime.active_mode = TR800_MODE_OFF;
		TR800_Runtime.active_cmd_source = TR800_MODE_OFF;
		TR800_Runtime.active_cmd_seq = 0;
		Flag_Stop = 1;
		tr800_runtime_sync_legacy_flags();
		return;
	}

	TR800_Runtime.requested_mode = mode;
	TR800_Runtime.active_mode = mode;
	TR800_Runtime.active_cmd_source = mode;
	TR800_Runtime.active_cmd_seq = cmd_seq;
	TR800_Runtime.system_state = (mode == TR800_MODE_OFF) ? TR800_READY : TR800_RUN;
	Flag_Stop = (TR800_Runtime.system_state == TR800_RUN) ? 0 : 1;
	tr800_runtime_sync_legacy_flags();
}

void tr800_runtime_sync_stop_flags(u8 legacy_hand_stop, u8 legacy_hand_stop1, u8 legacy_no_control_stop, u8 hall_stop)
{
	TR800_Runtime.fault_flags &= (u8)~TR800_FAULT_STOP_MASK;
	if (legacy_hand_stop)
	{
		TR800_Runtime.fault_flags |= TR800_FAULT_HAND_STOP;
	}
	if (legacy_hand_stop1)
	{
		TR800_Runtime.fault_flags |= TR800_FAULT_HAND_STOP1;
	}
	if (legacy_no_control_stop)
	{
		TR800_Runtime.fault_flags |= TR800_FAULT_NO_CONTROL;
	}
	if (hall_stop)
	{
		TR800_Runtime.fault_flags |= TR800_FAULT_HALL_STOP;
	}
}

TR800_SystemState tr800_runtime_get_system_state(void)
{
	return TR800_Runtime.system_state;
}

TR800_ControlMode tr800_runtime_get_requested_mode(void)
{
	return TR800_Runtime.requested_mode;
}

TR800_ControlMode tr800_runtime_get_active_mode(void)
{
	return TR800_Runtime.active_mode;
}

u8 tr800_runtime_get_fault_flags(void)
{
	return TR800_Runtime.fault_flags;
}

u8 tr800_runtime_stop_requested(void)
{
	return (TR800_Runtime.fault_flags & TR800_FAULT_STOP_MASK) ? 1 : 0;
}

u16 tr800_runtime_get_boot_remaining_ticks(void)
{
	if (TR800_Runtime.boot_ticks >= CONTROL_DELAY)
	{
		return 0;
	}
	return (u16)(CONTROL_DELAY - TR800_Runtime.boot_ticks);
}

const char *tr800_runtime_get_system_state_text(void)
{
	switch (TR800_Runtime.system_state)
	{
		case TR800_BOOT_DELAY: return "BOOT_DELAY";
		case TR800_READY:      return "READY";
		case TR800_RUN:        return "RUN";
		case TR800_FAULT:      return "FAULT";
		default:               return "UNKNOWN";
	}
}

const char *tr800_runtime_get_control_mode_text(void)
{
	switch (TR800_Runtime.active_mode)
	{
		case TR800_MODE_SERIAL: return "SERIAL";
		case TR800_MODE_RC:     return "RC";
		case TR800_MODE_OFF:
		default:             return "OFF";
	}
}

void systemInit(void)
{       
	//Interrupt priority group setting
	//中断优先级分组设置
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	//Delay function initialization
	//延时函数初始化
	delay_init(168);			                                            
	
	//Initialize the hardware interface connected to the LED lamp
	//初始化与LED灯连接的硬件接口
	LED_Init();                     
	Relay_init();    
    //Initialize the hardware interface connected to the buzzer	
    //初始化与蜂鸣器连接的硬件接口
	Buzzer_Init();  
	
	//Initialize the hardware interface connected to the enable switch
	//初始化与使能开关连接的硬件接口
	Enable_Pin();

	//Initialize the hardware interface connected to the OLED display
  //初始化与OLED显示屏连接的硬件接口	
	OLED_Init();     
	OLED_ShowString(0,0,"OLED BOOT");
	OLED_ShowString(0,10,"PE5 PE6 PE7 PE8");
	OLED_Refresh_Gram();
	
	//Initialize the hardware interface connected to the user's key
	//初始化与用户按键连接的硬件接口
	KEY_Init();	
	
	//Serial port 1 initialization, communication baud rate 115200.
	//串口1初始化，通信波特率115200。
	uart1_init(115200);	  
	//Serial port 2 initialization, communication baud rate 9600, 
	//used to communicate with Bluetooth APP terminal
	//串口2初始化，通信波特率9600，用于与蓝牙APP端通信
	#if IBUS_EN
	uart2_init(115200);             
	#elif SBUS_EN
    uart2_init(100000);
	#else
    uart2_init(9600);              
	#endif
	//Serial port 3 initialization, communication baud rate 115200.
	//串口3初始化，通信波特率115200。
	#if MAVLINK
	uart3_init(57600);
	
	#endif

	uart3_init(115200);
	// UART4 is the first-stage SERIAL command input.
	// UART4 是第一阶段 SERIAL 命令输入。
    uart4_init(115200);
    uart5_init(9600);
	
  //Initialize the CAN communication interface
  //CAN通信接口初始化
	CAN1_Mode_Init(1,7,6,3,0);      
	Servo_PWM_Init(19999,168-1);
	//ADC pin initialization, used to read the battery voltage and potentiometer gear, 
	//potentiometer gear determines the car after the boot of the car model
	//ADC引脚初始化，用于读取电池电压与电位器档位，电位器档位决定小车开机后的小车适配型号
 	Adc_Init();  
  //According to the tap position of the potentiometer, determine which type of car needs to be matched, 
  //and then initialize the corresponding parameters	
  //根据电位器的档位判断需要适配的是哪一种型号的小车，然后进行对应的参数初始化	
	Robot_params();                 
	//Encoder A is initialized to read the real time speed of motor A
	//编码器A初始化，用于读取电机A的实时速度
	Encoder_Init_TIM2(); 
	//Encoder B is initialized to read the real time speed of motor B
  //编码器B初始化，用于读取电机B的实时速度	
	Encoder_Init_TIM3(); 
  //Encoder C is initialized to read the real time speed of motor C  
  //编码器C初始化，用于读取电机C的实时速度	
	Encoder_Init_TIM4();
	//Encoder D is initialized to read the real time speed of motor D
  //编码器D初始化，用于读取电机D的实时速度	
	Encoder_Init_TIM5();  
	//Initialize the motor direction control pin, 
	//which is used to control the motor forward and reverse rotation
	//初始化电机方向控制引脚，用于控制电机正反转
	Dlrobot_Motor_Init();  
  //Initialize motor speed control and, for controlling motor speed, PWM frequency 10kHz
  //初始化电机速度控制以及，用于控制电机速度，PWM频率10KHZ 
	Dlrobot_PWM_Init(16799,9);   //初始化PWM 1KHZ，用于驱动电机
	
    I2C_GPIOInit();
    MPU6050_initialize();    
    Bias_check();
	tr800_runtime_init();
}
