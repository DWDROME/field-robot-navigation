#include "balance.h"

int Time_count=0; //Time variable //计时变量
int protect_count=0; //Time variable //计时变量
int hall_stop = 0;
extern int hand_stop,hand_stop1,NoControlstop;
// Robot mode is wrong to detect flag bits
//机器人模式是否出错检测标志位
int robot_mode_check_flag=0; 




//========== PWM清除使用变量 ==========//
u8 start_check_flag = 0;//标记是否需要清空PWM
u8 wait_clear_times = 0;
u8 start_clear = 0;     //标记开始清除PWM
u8 clear_done_once = 0; //清除完成标志位
u16 clear_again_times = 0;
float debug_show_diff = 0;
void auto_pwm_clear(void);
volatile u8 clear_state = 0x00;
bool slope = 0;
bool pwm_clear = 0;
bool pwmA_clear = 0;
bool pwmB_clear = 0;

bool pwm_clear_en = 0;
bool pwmA_clear_en = 0;
bool pwmB_clear_en = 0;
u16 stable_time,stable_time_show;
bool move = 0;

static u8 Stop_Request_Active(void)
{
	tr800_runtime_sync_stop_flags((u8)(hand_stop != 0), (u8)(hand_stop1 != 0), (u8)(NoControlstop != 0), (u8)(hall_stop != 0));
	return tr800_runtime_stop_requested();
}

static void Sync_Stop_Request(void)
{
	(void)Stop_Request_Active();
}

/*------------------------------------*/

static void Apply_Motor_Pwm_Output(int motor_a, int motor_b)
{
    #if MOTOR_A_DIR_REVERSE
    motor_a = -motor_a;
    #endif

    #if MOTOR_B_DIR_REVERSE
    motor_b = -motor_b;
    #endif

    #if MOTOR_DRIVER_1
    if(motor_a<0)		AIN2=0, 	AIN1=1;
	else if(motor_a > 0)AIN2=0, 	AIN1=0;
	else 				AIN2=1, 	AIN1=1;
	PWMA=abs(motor_a);
	
	if(motor_b<0)		BIN2=0, 	BIN1=1;
	else if(motor_b>0)	BIN2=0, 	BIN1=0;
	else 				BIN2=1, 	BIN1=1;
	PWMB=abs(motor_b);

    #elif MOTOR_DRIVER_2
    if(motor_a<0)		AIN2=0, 	AIN1=1;
	else if(motor_a > 0)AIN2=1, 	AIN1=0;
	else 				AIN2=0, 	AIN1=0;
	PWMA=abs(motor_a);
	
	if(motor_b<0)		BIN2=0, 	BIN1=1;
	else if(motor_b>0)	BIN2=1, 	BIN1=0;
	else 				BIN2=0, 	BIN1=0;
	PWMB=abs(motor_b);

    #elif MOTOR_DRIVER_3
    if(motor_a<0)		AIN2=0, 	AIN1=0;
	else if(motor_a > 0)AIN2=0, 	AIN1=1;
	else 				AIN2=1, 	AIN1=1;
	PWMA=abs(motor_a);
	
	if(motor_b<0)		BIN2=0, 	BIN1=0;
	else if(motor_b>0)	BIN2=0, 	BIN1=1;
	else 				BIN2=1, 	BIN1=1;
	PWMB=abs(motor_b);

	#endif
}

#if TR800_OPEN_LOOP_PWM_TEST
volatile u8 tr800_open_loop_state_debug = 0;

static u8 Open_Loop_Motor_Test(void)
{
	static u8 state = 0;
	static u32 start_ms = 0;
	u32 now = getSysTickCnt();
	int motor_a = 0;
	int motor_b = 0;

	if(state == 4)
	{
		tr800_open_loop_state_debug = state;
		RELAY1 = 0;
		Move_X = 0.0f;
		Move_Y = 0.0f;
		Move_Z = 0.0f;
		MOTOR_A.Target = 0.0f;
		MOTOR_B.Target = 0.0f;
		MOTOR_A.Motor_Pwm = 0;
		MOTOR_B.Motor_Pwm = 0;
		Apply_Motor_Pwm_Output(0, 0);
		if((now - start_ms) >= TR800_OPEN_LOOP_REPEAT_GAP_MS)
		{
			state = 0;
			start_ms = 0;
			tr800_open_loop_state_debug = state;
		}
		return 1;
	}

	if(state == 0)
	{
		tr800_open_loop_state_debug = state;
		if(start_ms == 0)
		{
			start_ms = now;
		}
		if((now - start_ms) < TR800_OPEN_LOOP_START_DELAY_MS)
		{
			Apply_Motor_Pwm_Output(0, 0);
			return 1;
		}
		state = 1;
		start_ms = now;
	}

	if(state == 1)
	{
		tr800_open_loop_state_debug = state;
		if(TR800_OPEN_LOOP_RELAY1_PULSE_MS == 0U)
		{
			RELAY1 = 0;
			state = 2;
			start_ms = now;
			return 1;
		}
		RELAY1 = 1;
		Apply_Motor_Pwm_Output(0, 0);
		if((now - start_ms) >= TR800_OPEN_LOOP_RELAY1_PULSE_MS)
		{
			RELAY1 = 0;
			state = 2;
			start_ms = now;
		}
		return 1;
	}

	if(state == 2)
	{
		tr800_open_loop_state_debug = state;
		RELAY1 = 0;
		Apply_Motor_Pwm_Output(0, 0);
		if((now - start_ms) >= TR800_OPEN_LOOP_AFTER_RELAY_MS)
		{
			state = 3;
			start_ms = now;
		}
		return 1;
	}

	if(state == 3)
	{
		tr800_open_loop_state_debug = state;
		RELAY1 = 0;
		if((TR800_OPEN_LOOP_DURATION_MS == 0U) || ((now - start_ms) < TR800_OPEN_LOOP_DURATION_MS))
		{
			motor_a = TR800_OPEN_LOOP_TEST_PWM;
			motor_b = TR800_OPEN_LOOP_TEST_PWM;
		}
		else
		{
			state = 4;
			start_ms = now;
		}
	}

	Move_X = 0.0f;
	Move_Y = 0.0f;
	Move_Z = 0.0f;
	MOTOR_A.Target = 0.0f;
	MOTOR_B.Target = 0.0f;
	MOTOR_A.Motor_Pwm = motor_a;
	MOTOR_B.Motor_Pwm = motor_b;

	if(state == 4 || Voltage < 40.0f || EN == 0)
	{
		tr800_open_loop_state_debug = state;
		RELAY1 = 0;
		Apply_Motor_Pwm_Output(0, 0);
		return 1;
	}

	Apply_Motor_Pwm_Output(motor_a, motor_b);
	return 1;
}
#endif


Encoder OriginalEncoder; //Encoder raw data //编码器原始数据     
/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
函数功能：运动学逆解，根据三轴目标速度计算各车轮目标转速
入口参数：X和Y、Z轴方向的目标运动速度
返回  值：无
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
	float amplitude=3.5; //Wheel target speed limit //车轮目标速度限幅
	//Inverse kinematics //运动学逆解
	MOTOR_A.Target  = (Vx - Vz * (Wheel_spacing) / 2.0f) * 1.015f;    //计算出左轮的目标速度
	MOTOR_B.Target =  (Vx + Vz * (Wheel_spacing) / 2.0f) * 0.985f;    //计算出右轮的目标速度
	//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
	MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
    MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
	MOTOR_C.Target=0; //Out of use //没有使用到
	MOTOR_D.Target=0; //Out of use //没有使用到
	
}

/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
函数功能：FreeRTOS任务，核心运动控制任务
入口参数：无
返回  值：无
**************************************************************************/
int console_sig_lost,rc_sig_lost;
int RosCmdUpdate,control_sig;

#define TR800_CMD_SLEW_X_STEP      0.025f
#define TR800_CMD_SLEW_Y_STEP      0.025f
#define TR800_CMD_SLEW_Z_STEP      0.060f

static float tr800_slew_towards(float current, float target, float step)
{
	float delta = target - current;

	if(delta > step) return current + step;
	if(delta < -step) return current - step;
	return target;
}

static void tr800_apply_control_command(u32 now)
{
	TR800_CommandSnapshot cmd;
	TR800_ControlMode mode = TR800_MODE_OFF;
	static TR800_ControlMode last_mode = TR800_MODE_OFF;
	static float smooth_x = 0.0f;
	static float smooth_y = 0.0f;
	static float smooth_z = 0.0f;
	u32 cmd_seq = 0;
	float cmd_x = 0.0f;
	float cmd_y = 0.0f;
	float cmd_z = 0.0f;
	u8 rc_selected = (RC_pid < TR800_RC_MODE_SELECT_LOW) ? 1 : 0;
	TR800_SystemState state = tr800_runtime_get_system_state();

	if (state == TR800_READY || state == TR800_RUN)
	{
		if (rc_selected && tr800_runtime_get_rc_cmd(&cmd, now))
		{
			mode = TR800_MODE_RC;
			cmd_seq = cmd.seq;
			cmd_x = cmd.x;
			cmd_y = cmd.y;
			cmd_z = cmd.z;
		}
		else if (!rc_selected && tr800_runtime_get_serial_cmd(&cmd, now))
		{
			mode = TR800_MODE_SERIAL;
			cmd_seq = cmd.seq;
			cmd_x = cmd.x;
			cmd_y = cmd.y;
			cmd_z = cmd.z;
		}
	}

	if(mode == TR800_MODE_OFF)
	{
		smooth_x = 0.0f;
		smooth_y = 0.0f;
		smooth_z = 0.0f;
	}
	else
	{
		if(mode != last_mode)
		{
			smooth_x = 0.0f;
			smooth_y = 0.0f;
			smooth_z = 0.0f;
		}
		smooth_x = tr800_slew_towards(smooth_x, cmd_x, TR800_CMD_SLEW_X_STEP);
		smooth_y = tr800_slew_towards(smooth_y, cmd_y, TR800_CMD_SLEW_Y_STEP);
		smooth_z = tr800_slew_towards(smooth_z, cmd_z, TR800_CMD_SLEW_Z_STEP);
	}
	last_mode = mode;

	Move_X = smooth_x;  // 前后方向按RC/串口命令原始符号输出。
	Move_Y = smooth_y;
	Move_Z = -smooth_z;  // 转向方向与RC/串口命令相反，在统一入口翻转Z轴。
	tr800_runtime_apply_active_command(mode, cmd_seq);
}

void Balance_task(void *pvParameters)
{ 
  u32 lastWakeTime = getSysTickCnt();
  u32 now = 0;
  int j = 0;
  while(1)
  {	
	// This task runs at a frequency of 100Hz (10ms control once)
	//此任务以100Hz的频率运行（10ms控制一次）
	vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); 
	
	//Time count is no longer needed after 30 seconds
	//时间计数，30秒后不再需要
	if(Time_count<3000)Time_count++;

	//Get the encoder data, that is, the real time wheel speed, 
	//and convert to transposition international units
	//获取编码器数据，即车轮实时速度，并转换位国际单位
	Get_Velocity_Form_Encoder();   
#if TR800_OPEN_LOOP_PWM_TEST
	if(Open_Loop_Motor_Test())
	{
		continue;
	}
#endif
	auto_pwm_clear();
	if(((FrontAdjust == 1 || BackAdjust == 1)) && (j++ == 10))
	{
		j = 0;
		BiasAdjust();
		BiasadjustSave();
	}

	now = getSysTickCnt();
	Sync_Stop_Request();
	tr800_runtime_update(now, (u16)Time_count, EN, (float)Voltage);
	tr800_apply_control_command(now);
	Drive_Motor(Move_X, Move_Y, Move_Z);
		
		//Click the user button to update the gyroscope zero
		//单击用户按键更新陀螺仪零点
		Key(); 
		//If there is no abnormity in the battery voltage, and the enable switch is in the ON position,
        //and the software failure flag is 0
		//如果电池电压不存在异常，而且使能开关在ON档位，而且软件失能标志位为0
		if(Turn_Off(Voltage)==0) 
		 { 			
             //Speed closed-loop control to calculate the PWM value of each motor, 
			 //PWM represents the actual wheel speed					 
			 //速度闭环控制计算各电机PWM值，PWM代表车轮实际转速

				 if(Stop_Request_Active()) MOTOR_A.Target = MOTOR_B.Target = MOTOR_C.Target = MOTOR_D.Target = 0;
			 MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
			 MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
             #if TR600 || TS50 || TR600_MINI || TR600_WP
			 auto_pwm_clear();
			 if(int_abs(MOTOR_A.Target*1000) > 50) pwmA_clear_en = 1;
			 if(int_abs(MOTOR_B.Target*1000) > 50) pwmB_clear_en = 1;
			
			 if(int_abs(MOTOR_A.Target*1000) == 0 && int_abs(MOTOR_A.Encoder*1000) == 0 && pwmA_clear_en == 1 && slope == 0)  pwmA_clear = 1;
			 else  pwmA_clear = 0;	
				 
			 if(int_abs(MOTOR_B.Target*1000) == 0 && int_abs(MOTOR_B.Encoder*1000) == 0 && pwmB_clear_en == 1 && slope == 0)  pwmB_clear = 1;		 
			 else  pwmB_clear = 0;
             #if 1
			 if(int_abs(MOTOR_A.Encoder*1000) == 0 && int_abs(MOTOR_B.Encoder*1000) == 0 && move) 
				{
					stable_time_show = stable_time; 
				}
			 if(int_abs(MOTOR_A.Encoder*1000) > 0 || int_abs(MOTOR_B.Encoder*1000) > 0) move = 1;
			 
			 else move = 0;
             #endif
			 
			 if(MOTOR_A.Target == 0 && MOTOR_B.Target == 0) stable_time++;
			 else stable_time = 0;
			 
			 if(stable_time < 200 && slope == 0 )
		   //  if(stable_time < 200 )
			 {
			   if(abs(MOTOR_A.Target*1000) == 0 && abs(MOTOR_A.Encoder*1000) < 150)   MOTOR_A.Motor_Pwm = 0;
			   if(abs(MOTOR_B.Target*1000) == 0 && abs(MOTOR_B.Encoder*1000) < 150 )  MOTOR_B.Motor_Pwm = 0;
			 }					 
             #endif

             #if 0 //TS50
             if(abs(MOTOR_A.Target*1000) == 0 && abs(MOTOR_A.Encoder*1000) < 50)  // 80 120 
            	{
				   MOTOR_A.Motor_Pwm = 0;

            	}
			 if(abs(MOTOR_B.Target*1000) == 0 && abs(MOTOR_B.Encoder*1000) < 50 )   // 120
				{
			    	MOTOR_B.Motor_Pwm = 0;
				}
             #endif
			 
	    	 Limit_Pwm(15960);    //===PWM限幅 最高是16800，考虑到安全，做了限幅，可以在完全调试好之后
			 //Set different PWM control polarity according to different car models
			 //根据不同小车型号设置不同的PWM控制极性
      
			 if((abs(MOTOR_A.Motor_Pwm) == 15960 && (MOTOR_A.Target*MOTOR_A.Encoder <= 0))||(abs(MOTOR_B.Motor_Pwm) == 15960 && (MOTOR_B.Target*MOTOR_B.Encoder <= 0)))
				protect_count++; 
			 else  protect_count = 0;
			 if(protect_count > 100) hall_stop = 1;
			 else if( abs(MOTOR_A.Target*1000) == 0 && abs(MOTOR_B.Target*1000) == 0) hall_stop = 0;
			 
			 Set_Pwm(MOTOR_A.Motor_Pwm,   MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0  );
			}
		 
		 //If Turn_Off(Voltage) returns to 1, the car is not allowed to move, and the PWM value is set to 0
		 //如果Turn_Off(Voltage)返回值为1，不允许控制小车进行运动，PWM值设置为0
		 else	Set_Pwm(0,0,0,0,0);

	}
}
/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
函数功能：赋值给PWM寄存器，控制车轮转速与方向
入口参数：PWM
返回  值：无
**************************************************************************/

void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo)
{
   if(Stop_Request_Active()) motor_a= motor_b =0;
   Apply_Motor_Pwm_Output(motor_a, motor_b);
}


/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
函数功能：限制PWM值 
入口参数：幅值
返回  值：无
**************************************************************************/
void Limit_Pwm(int amplitude)
{	
	    MOTOR_A.Motor_Pwm=target_limit_float(MOTOR_A.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_B.Motor_Pwm=target_limit_float(MOTOR_B.Motor_Pwm,-amplitude,amplitude);
		MOTOR_C.Motor_Pwm=target_limit_float(MOTOR_C.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_D.Motor_Pwm=target_limit_float(MOTOR_D.Motor_Pwm,-amplitude,amplitude);
}	    
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
函数功能：限幅函数
入口参数：幅值
返回  值：无
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
函数功能：检查电池电压、使能开关状态、软件失能标志位状态
入口参数：电压
返回  值：是否允许控制，1：不允许，0允许
**************************************************************************/
u8 Turn_Off( int voltage)
{
  u8 temp;
 if(voltage<40 ||EN==0 || Flag_Stop==1)
   {	                                                
	 
	PWMA=0;
	PWMB=0;		
	PWMC=0;	
	PWMD=0;
	temp=1;   
   }
	else
	temp=0;
	return temp;			
}
/**************************************************************************
Function: Calculate absolute value
Input   : long int
Output  : unsigned int
函数功能：求绝对值
入口参数：long int
返回  值：unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)

函数功能：增量式PI控制器
入口参数：编码器测量值(实际速度)，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
static u8 tr800_target_is_zero(float target)
{
	return (float_abs(target * 1000.0f) < 1.0f) ? 1 : 0;
}

int Incremental_PI_A (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias,Last_Pwm;
	 static int cnt = 0;

     if(Flag_Stop || Stop_Request_Active() || tr800_target_is_zero(Target)) Pwm=Bias=Last_bias=Last_Pwm=0, cnt=0;
     else{
        	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
        	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
        	 if(Pwm>16800)Pwm=16800;
        	 if(Pwm<-16800)Pwm=-16800;
        	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
        	 #if TS50 
        	 if(Target == 0 && Bias == 0)Pwm = 0; 
        	 #elif TR600 || TR600_MINI || TR600_WP
             if(Target == 0 && Bias == 0 && slope == 0 && cnt++ == 300)cnt=0,Pwm = 0; 
        	 if(pwmA_clear == 1 )pwmA_clear_en = 0, Pwm = 0;
        	 #endif
        	 if(Last_Pwm * Pwm < 0) Pwm = 0;
        	 Last_Pwm = Pwm;
      }
	 return Pwm;    
}
int Incremental_PI_B (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias,Last_Pwm;
	 static int cnt = 0;
	 if(Flag_Stop || Stop_Request_Active() || tr800_target_is_zero(Target)) Pwm=Bias=Last_bias=Last_Pwm=0, cnt=0;
	 else{
        	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
        	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
        	 if(Pwm>16800)Pwm=16800;
        	 if(Pwm<-16800)Pwm=-16800;
        	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
        	
             #if TS50  
        	 if(slope == 1 && Target == 0 && Bias == 0) Pwm = 0; 
             else if(Target == 0 && Bias == 0 && slope == 0 && cnt++ == 300)cnt=0,Pwm = 0;
             #elif  TR600 || TR600_MINI || TR600_WP
        	 if(Target == 0 && Bias == 0 && slope == 0 && cnt++ == 300)cnt=0,Pwm = 0; 
        	 if(pwmB_clear == 1 )pwmB_clear_en = 0, Pwm = 0;
             #endif
        	 if(Last_Pwm * Pwm < 0) Pwm = 0;
        	 Last_Pwm = Pwm;
	 	}
	 return Pwm;
}
int Incremental_PI_C (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 static int cnt = 0;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16800)Pwm=16800;
	 if(Pwm<-16800)Pwm=-16800;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
     #if TS50
	 if(Target == 0 && Bias == 0)Pwm = 0; 
     #endif
	
	 return Pwm; 
}
int Incremental_PI_D (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 static int cnt = 0;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>16800)Pwm=16800;
	 if(Pwm<-16800)Pwm=-16800;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差
	 #if TS50
	 if(Target == 0 && Bias == 0)Pwm = 0; 
     #endif
	 return Pwm; 
}

/**************************************************************************
Function: The remote control command of model aircraft is processed
Input   : none
Output  : none
函数功能：对航模遥控控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
long map(long x, long in_min, long in_max, long out_min, long out_max)
	{
	    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}
#define  STABLE_NUM_X  1000
#define  STABLE_NUM_Y  1000
#define  reverse       0
#define  turn_mode_1   1
void Motor_run(int pwm_x,int pwm_y)
{
	unsigned int x = abs(pwm_x);
	short pwm,pwm_l, pwm_b;
	int pwm1,pwm2;
	pwm_b = abs(pwm_y);
   if( pwm_x > STABLE_NUM_X )									   
	 {
		   if( pwm_y > STABLE_NUM_Y) { // front  right	

			   #if turn_mode_1
               pwm_l =	map(16800-abs(pwm_x),1000,15800,-abs(pwm_y)/2,abs(pwm_y));  
			   #else
			   pwm_l =	map(16800-abs(pwm_x),1000,15800,1000,abs(pwm_y));  
			   #endif
           #if reverse
			   pwm1 = pwm_l;
			   pwm2 = -pwm_b;
		   #else
			   pwm1 = pwm_b;
			   pwm2 = -pwm_l;
		   #endif		   
		   }
		   else if(pwm_y < -STABLE_NUM_Y){	 // back right	
			   #if turn_mode_1
               pwm_l =	map(16800-abs(pwm_x),1000,15800,-abs(pwm_y)/2,abs(pwm_y));  
			   #else
			   pwm_l =	map(16800-abs(pwm_x),1000,15800,1000,abs(pwm_y)); 
			   #endif
           #if reverse
			   pwm1 =-pwm_l;
			   pwm2 =pwm_b; 
		   #else
			   pwm1 =-pwm_b;
			   pwm2 =pwm_l; 
		   #endif   
			}
		   else{			  
				  pwm = map(x,1000,15800,1000,15800); 	
				  pwm1 =pwm;
				  pwm2 =pwm;
			   }	 
		}
	else if(pwm_x < -STABLE_NUM_X )
		{	   // Left
		  if( pwm_y > STABLE_NUM_Y) {	//	Left   front
			  #if turn_mode_1
               pwm_l =	map(16800-abs(pwm_x),1000,15800,-abs(pwm_y)/2,abs(pwm_y));  
			   #else
			   pwm_l =	map(16800-abs(pwm_x),1000,15800,1000,abs(pwm_y)); 
			   #endif
         #if reverse
			  pwm1 =pwm_b;
			  pwm2 =-pwm_l;  
         #else
			  pwm1 =pwm_l;
			  pwm2 =-pwm_b; 
         #endif
   
		   }
		   else if(pwm_y < -STABLE_NUM_Y){	//	 Left	back
				  #if turn_mode_1
	              pwm_l = map(16800-abs(pwm_x),1000,15800,-abs(pwm_y)/2,abs(pwm_y));  
				  #else
				  pwm_l = map(16800-abs(pwm_x),1000,15800,1000,abs(pwm_y)); 
				  #endif
	              #if reverse
				  pwm1 = -pwm_b;
				  pwm2 =pwm_l; 
	              #else
				  pwm1 = -pwm_l;
				  pwm2 =pwm_b; 
	              #endif
			}
		   else{
				 pwm = map(x,1000,15800,1000,15800);  // Left 10
				 pwm1 =-pwm;
				 pwm2 =-pwm; 
			}
		 
		}
   else{
		   if( pwm_y > STABLE_NUM_Y)	// 前进
			 {
				int pwm_y0 = pwm_y;
				int pwm_y1 = pwm_y;   //*0.95;	--- 
				pwm1 =pwm_y0 *FL_scale;
				pwm2 =-pwm_y1*FR_scale; 
		
			 }
		   else if(pwm_y < -STABLE_NUM_Y)	 // 后退
			   {
				 int pwm_y0 = pwm_y;
				 int pwm_y1 = pwm_y;
				 pwm1 =  pwm_y0*BL_scale;
				 pwm2 = -pwm_y1*BR_scale;		   
			   }
			   else   
				{
					/* First-stage RC path keeps the last zeroed output behavior from the runtime layer. */
				}
			}

     Set_Pwm(pwm1,-pwm2 ,0 ,0 ,1500 );
	 MOTOR_A.Motor_Pwm = pwm1;
	 MOTOR_B.Motor_Pwm = -pwm2;
	  
}
	
static int RC_Select_Turn_Input(int channel4_turn, int channel1_turn)
{
	return (channel4_turn != 0) ? channel4_turn : -channel1_turn;
}

#define TR800_RC_SPEED_MIN_SCALE       0.85f
#define TR800_RC_SPEED_MAX_SCALE       1.00f
#define TR800_RC_SPEED_TRIM_LIMIT      300
#define TR800_RC_MIN_VELOCITY_MM_S     120
#define TR800_RC_MAX_VELOCITY_MM_S     1100

static int RC_Limit_Int(int value, int low, int high)
{
	if(value < low) return low;
	if(value > high) return high;
	return value;
}

static float RC_Speed_Channel_To_Scale(int raw)
{
	float normalized;

	if(raw < 900 || raw > 2100)
	{
		return 1.0f;
	}

	normalized = ((float)raw - 1000.0f) / 1000.0f;
	normalized = target_limit_float(normalized, 0.0f, 1.0f);
	return TR800_RC_SPEED_MIN_SCALE +
		normalized * (TR800_RC_SPEED_MAX_SCALE - TR800_RC_SPEED_MIN_SCALE);
}

void rc_loop_control(void)
  {
	int Yuzhi=RC_ACT;
	int LX,LY,RX,RY,Remote_RCvelocity;
	int Turn_Input;
	float speed_scale;
	float rc_x;
	float rc_y = 0.0f;
	float rc_z;
    #if IBUS_EN
	LX = RC_LX-1500;
	LY = RC_LY-1500;
	RX = RC_RX-1500;
	RY = RC_RY-1500;
    #elif SBUS_EN

    #if SIYI
	LX = (RC_LX-992)*25/36;
	RY = (RC_RY-992)*25/36;
    RX = (RC_RX-992)*25/36;
	RY = (RC_RY-992)*25/36;
	#elif YUNZHUO
    LX =(RC_LX- 1002)*25/36;
	LY = (RC_LY-1002)*25/36;
	RX = (RC_RX-1002)*25/36;
	RY = (RC_RY-1002)*25/36;
	#endif

    #endif

	if(LX>-Yuzhi&&LX<Yuzhi)LX=0;
	if(LY>-Yuzhi&&LY<Yuzhi)LY=0;
	if(RX>-Yuzhi&&RX<Yuzhi)RX=0;
	if(RY>-Yuzhi&&RY<Yuzhi)RY=0;
		
	speed_scale = RC_Speed_Channel_To_Scale(RC_Speed);
	Remote_RCvelocity = (int)((RC_Velocity + RC_Limit_Int(RX, -TR800_RC_SPEED_TRIM_LIMIT, TR800_RC_SPEED_TRIM_LIMIT)) * speed_scale);
	Remote_RCvelocity = RC_Limit_Int(Remote_RCvelocity, 0, TR800_RC_MAX_VELOCITY_MM_S);
	if(Remote_RCvelocity < TR800_RC_MIN_VELOCITY_MM_S) Remote_RCvelocity = 0;
	
	Turn_Input = RC_Select_Turn_Input(LY, RY);

	rc_x = ((float)LX * (float)Remote_RCvelocity / 500.0f) / 1000.0f; // 统一入口会翻转X轴；这里保持RC通道2原始符号。
	rc_z = ((float)Turn_Input * (PI / 6.0f) / 500.0f) * ((float)Remote_RCvelocity / 200.0f); // CH4 priority, CH1 fallback.

	tr800_runtime_note_rc_cmd(rc_x, rc_y, rc_z, getSysTickCnt());
}

/**************************************************************************
Function: Click the user button to update gyroscope zero
Input   : none
Output  : none
函数功能：单击用户按键更新陀螺仪零点
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{	
	u8 tmp;
	tmp=click_N_Double_MPU6050(50); 
	if(tmp==2)memcpy(Deviation_gyro,Original_gyro,sizeof(gyro));
}
/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
函数功能：读取编码器数值并计算车轮速度，单位m/s
入口参数：无
返回  值：无
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
	  //Retrieves the original data of the encoder
	  //获取编码器的原始数据
		float Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr; 
	  
		OriginalEncoder.A=Read_Encoder(2);	
		OriginalEncoder.B=Read_Encoder(3);	
	//	OriginalEncoder.C=Read_Encoder(4);	
	//	OriginalEncoder.D=Read_Encoder(5);
		
		#if MAVLINK || SHOW_SPEED != 1
        OriginalEncoder.E += OriginalEncoder.A;
        OriginalEncoder.F += OriginalEncoder.B;
    //    OriginalEncoder.G += OriginalEncoder.C;
	//	OriginalEncoder.H += OriginalEncoder.D;
		#endif
	    //Decide the encoder numerical polarity according to different car models
		//根据不同小车型号决定编码器数值极性   

        #if  ENCODE_A_REVERSE
		Encoder_A_pr=-OriginalEncoder.A; 
        #else 
        Encoder_A_pr=OriginalEncoder.A; 
		#endif


		#if  ENCODE_B_REVERSE
		Encoder_B_pr=-OriginalEncoder.B; 
        #else 
        Encoder_B_pr=OriginalEncoder.B; 
		#endif
		
		//The encoder converts the raw data to wheel speed in m/s
		//编码器原始数据转换为车轮速度，单位m/s
		MOTOR_A.Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_B.Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
	//	MOTOR_C.Encoder= Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
	//	MOTOR_D.Encoder= Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
}
/**************************************************************************
Function: Smoothing the three axis target velocity
Input   : Three-axis target velocity
Output  : none
函数功能：对三轴目标速度做平滑处理
入口参数：三轴目标速度
返回  值：无
**************************************************************************/
void Smooth_control(float vx,float vy,float vz)
{
	float step=0.01;

	if	   (vx>0) 	smooth_control.VX+=step;
	else if(vx<0)	smooth_control.VX-=step;
	else if(vx==0)	smooth_control.VX=smooth_control.VX*0.9f;
	
	if	   (vy>0)   smooth_control.VY+=step;
	else if(vy<0)	smooth_control.VY-=step;
	else if(vy==0)	smooth_control.VY=smooth_control.VY*0.9f;
	
	if	   (vz>0) 	smooth_control.VZ+=step;
	else if(vz<0)	smooth_control.VZ-=step;
	else if(vz==0)	smooth_control.VZ=smooth_control.VZ*0.9f;
	
	smooth_control.VX=target_limit_float(smooth_control.VX,-float_abs(vx),float_abs(vx));
	smooth_control.VY=target_limit_float(smooth_control.VY,-float_abs(vy),float_abs(vy));
	smooth_control.VZ=target_limit_float(smooth_control.VZ,-float_abs(vz),float_abs(vz));
}
/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
函数功能：浮点型数据计算绝对值
入口参数：浮点数
返回  值：输入数的绝对值
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}
#if 1

void auto_pwm_clear(void)
{
    //小车姿态简易判断
    static int slope_time = 0;
    float y_accle = (float)(accel[1]/1671.84f);//Y轴加速度实际值
    float z_accle = (float)(accel[2]/1671.84f);//Z轴加速度实际值
    float diff;

    //计算Y、Z加速度融合值，该值越接近9.8，表示小车姿态越水平
    if( y_accle > 0 ) diff  = z_accle - y_accle;
    else diff  = z_accle + y_accle;

	if(diff > 8.0f) slope_time = 0;//,Buzzer =0;
	else slope_time++;             // Buzzer = 1
	if(slope_time > 50) slope = 1;
	else  slope = 0;

//	if(slope == 1) Velocity_KP = 300 ,Velocity_KI = 200;
//	else Velocity_KP = 800 ,Velocity_KI = 600;
    debug_show_diff = diff; 
}


u32 int_abs(int a)
 {
	 u32 temp;
	 if(a<0) temp=-a;
	 else temp = a;
	 return temp;
 }
#endif

