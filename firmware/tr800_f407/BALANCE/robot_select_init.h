#ifndef __ROBOTSELECTINIT_H
#define __ROBOTSELECTINIT_H
#include "sys.h"
#include "system.h"

//Parameter structure of robot
//机器人参数结构体
typedef struct  
{
  float WheelSpacing;      //Wheelspacing, Mec_Car is half wheelspacing //轮距 麦轮车为半轮距
  float AxleSpacing;       //Axlespacing, Mec_Car is half axlespacing //轴距 麦轮车为半轴距	
  float GearRatio;           //Motor_gear_ratio //电机减速比
  int EncoderAccuracy;     //Number_of_encoder_lines //编码器精度(编码器线数)
  float WheelDiameter;     //Diameter of driving wheel //主动轮直径	
  float OmniTurnRadiaus;   //Rotation radius of omnidirectional trolley //全向轮小车旋转半径	
}Robot_Parament_InitTypeDef;

// Encoder structure
//编码器结构体
#define SHOW_SPEED 1
typedef struct  
{
  int A;      
  int B; 
  int C; 
  int D; 
  int64_t E;      
  int64_t F; 
  int64_t G; 
  int64_t H; 
}Encoder;


//遥控器型号
#define IBUS_EN 1
#define SBUS_EN 0
#define FUTABA  0
#define SIYI    1
#define MK15    0

#define YUNZHUO 0
#define YUNTAI_EN    0
#define MAVLINK      0

//底盘型号
#define TR600        0
#define TS50         1
#define TR600_MINI   0
#define TR600_WP     0

#define BIAS_MODE_0  0

#if TS50  
#define ENCODE_A_REVERSE           1
#define ENCODE_B_REVERSE           1
#define MOTOR_A_DIR_REVERSE        0
#define MOTOR_B_DIR_REVERSE        0
#elif TR600
#define ENCODE_A_REVERSE           1
#define ENCODE_B_REVERSE           1
#define MOTOR_A_DIR_REVERSE        0
#define MOTOR_B_DIR_REVERSE        0

#elif TR600_WP
#define ENCODE_A_REVERSE           1
#define ENCODE_B_REVERSE           1
#define MOTOR_A_DIR_REVERSE        0
#define MOTOR_B_DIR_REVERSE        0


#elif TR600_MINI
#define ENCODE_A_REVERSE           0
#define ENCODE_B_REVERSE           1
#define MOTOR_A_DIR_REVERSE        1
#define MOTOR_B_DIR_REVERSE        0

#endif

//底盘参数
#if TR600
#define Tank_wheelSpacing        0.520f  
//#define	Tank_WheelDiameter   0.235f
//#define	Tank_WheelDiameter   0.1471f  // tr600 mini
#define	Tank_WheelDiameter       0.180f  // tr600 mini
#elif TR600_MINI
#define Tank_wheelSpacing        0.490f  
#define	Tank_WheelDiameter       0.1471f  

#elif TR600_WP
#define Tank_wheelSpacing        0.520f  
#define	Tank_WheelDiameter       0.230f  

#elif TS50
#define Tank_wheelSpacing        0.564f  
#define Tank_WheelDiameter       0.1706f
#endif

//Motor_gear_ratio
//电机减速比
#define   HALL_5F     5 
#define   HALL_10F    10.41f 
#define   HALL_15F    16.58f 
#define   HALL_18F    18 
#define   HALL_20F    20 
#define   HALL_30F    30 
#define   HALL_28F    28 
#define   HALL_30     30
#define   HALL_31_4F  31.4f 
#define   HALL_36F    36 
#define   HALL_50F    50 
#define   HALL_56F    56 
#define   HALL_75F    75  

// 电机驱动器型号
#define  MOTOR_DRIVER_1      1   // 750W
#define  MOTOR_DRIVER_2      0   // 1500W
#define  MOTOR_DRIVER_3      0   // 500W

#define  Speed_Threshold     1  // 速度档位

//Number_of_encoder_lines
//编码器精度
#define	  Photoelectric_1000 1000
//The encoder octave depends on the encoder initialization Settings
//编码器倍频数，取决于编码器初始化设置
#define   EncoderMultiples  4
//Encoder data reading frequency
//编码器数据读取频率
#define   CONTROL_FREQUENCY 100
#define   PROTOCOL_25       0

// RC输入诊断：复用状态回传帧的IMU字段输出CH1..CH5和模式码。
// 仅用于确认遥控器数据是否进入MCU；正式运行应关闭。
#define   TR800_RC_TELEMETRY_DEBUG  0

// 电机诊断：复用状态回传帧的IMU字段输出 rawA/rawB/pwmA/pwmB/EN/模式码。
// 用于串口直控时判断单边不动是 PWM 未输出，还是电机/驱动/编码器链路问题。
#define   TR800_MOTOR_TELEMETRY_DEBUG  0

// 控制链路诊断：分页复用状态回传帧的IMU字段，输出 RC输入/命令目标/编码器PWM。
// 用于定位“不及时、不圆滑、速度控制无效”卡在输入、目标、闭环还是输出。
#define   TR800_CONTROL_TELEMETRY_DEBUG  1

// 临时诊断开关：电压检测硬件异常时，仅关闭低电压 fault。
// 保留使能、急停、hand_stop、hall_stop 等其它停机保护；修复电压检测线后应恢复为 0。
#define   TR800_DISABLE_LOW_VOLTAGE_FAULT  1

// OLED 诊断：上电后只运行显示自检，不执行蜂鸣/低压提示逻辑。
// 用于确认 PE5/PE6/PE7/PE8 到 OLED 的硬件链路；确认后改为 0 使用正常状态页。
#ifndef TR800_OLED_SELF_TEST
#define   TR800_OLED_SELF_TEST  0
#endif

// 开环电机诊断：上电后等待一段时间，A/B 两路同时低占空比前进，然后停止。
// 仅用于排查 PWM/方向/驱动器/电机链路，确认后应关闭。
#define   TR800_OPEN_LOOP_PWM_TEST   0
#define   TR800_OPEN_LOOP_TEST_PWM   5000
#define   TR800_OPEN_LOOP_START_DELAY_MS   8000U
#define   TR800_OPEN_LOOP_RELAY1_PULSE_MS  0U
#define   TR800_OPEN_LOOP_AFTER_RELAY_MS   0U
#define   TR800_OPEN_LOOP_DURATION_MS      3000U
#define   TR800_OPEN_LOOP_REPEAT_GAP_MS    5000U
void Robot_params(void);
void Robot_Init(double wheelspacing, float axlespacing, float omni_turn_radiaus, float gearratio,float Accuracy,float tyre_diameter);

#endif
