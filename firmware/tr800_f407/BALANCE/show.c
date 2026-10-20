#include "show.h"

int Voltage_Show;
unsigned char i;
unsigned char Send_Count;
extern SEND_DATA Send_Data;
extern int Time_count;
extern Encoder OriginalEncoder;
extern float debug_show_diff;

int Buzzer_count = 25;
u8 show_ID = 0;
int motor_adjust_timecount;
volatile u32 TR800_OLED_DebugHeartbeat = 0;
volatile u8 TR800_OLED_DebugPhase = 0;

#ifndef TR800_OLED_SELF_TEST
#define TR800_OLED_SELF_TEST 0
#endif

#define OLED_TEXT(text) ((const u8 *)(text))

#if TR800_OLED_SELF_TEST
extern u8 OLED_GRAM[128][8];
#endif

typedef struct
{
	u32 now_ms;
	u32 heartbeat;
	const u8 *system_state;
	const u8 *control_mode;
	u8 fault_flags;
	u8 flag_stop;
	u8 rc_alive;
	int rc_power;
	long target_a_milli;
	long target_b_milli;
	long encoder_a_milli;
	long encoder_b_milli;
	long pwm_a;
	long pwm_b;
	int voltage_centivolt;
} OLED_StatusSnapshot;

#if TR800_OLED_SELF_TEST
static u8 OLED_RenderSelfTest(u32 heartbeat);
#endif
static void OLED_RenderNormalStatus(u32 now_ms, u32 heartbeat);
static void OLED_CaptureStatusSnapshot(OLED_StatusSnapshot *out, u32 now_ms, u32 heartbeat);
static void OLED_RenderStatusSnapshot(const OLED_StatusSnapshot *snapshot);

#if TR800_OLED_SELF_TEST
static void OLED_FillGramLocal(u8 data)
{
	u8 page;
	u8 column;

	for(page = 0; page < 8; page++)
	{
		for(column = 0; column < 128; column++)
		{
			OLED_GRAM[column][page] = data;
		}
	}
}

static void OLED_EntireDisplayOnLocal(void)
{
	OLED_WR_Byte(0xA5, OLED_CMD);
}

static void OLED_EntireDisplayResumeLocal(void)
{
	OLED_WR_Byte(0xA4, OLED_CMD);
}
#endif

void show_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	u8 oled_refresh_div = 0;
	u32 oled_heartbeat = 0;
#if !TR800_OLED_SELF_TEST
	int sample_index = 0;
	static int LowVoltage_1 = 0;
	static int LowVoltage_2 = 0;
#endif

	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_10_HZ));

#if TR800_OLED_SELF_TEST
		if(++oled_refresh_div >= 5)
		{
			oled_refresh_div = 0;
			oled_heartbeat++;
			TR800_OLED_DebugHeartbeat = oled_heartbeat;
			if(OLED_RenderSelfTest(oled_heartbeat))
			{
				OLED_Refresh_Gram();
			}
		}
#else
		if(Time_count < 50) Buzzer = 1;
		else if(Time_count >= 51 && Time_count < 100) Buzzer = 0;

		if(LowVoltage_1 == 1 || LowVoltage_2 == 1) Buzzer_count = 0;
		if(Buzzer_count < 5) Buzzer_count++;
		if(Buzzer_count < 5) Buzzer = 1;
		else if(Buzzer_count == 5) Buzzer = 0;

		if(Long_Press() == 1)
		{
			show_ID++;
			if(show_ID > 2) show_ID = 0;
			OLED_Clear();
		}

		if(Beep_Control_Flag)
		{
			Buzzer = 1;
			motor_adjust_timecount++;
			if(motor_adjust_timecount == 10)
			{
				Beep_Control_Flag = 0;
				Buzzer = 0;
				motor_adjust_timecount = 0;
			}
		}

		for(sample_index = 0; sample_index < 100; sample_index++)
		{
			Voltage_All += Get_battery_volt();
		}
		Voltage = Voltage_All / 100;
		Voltage_All = 0;

		if(LowVoltage_1 == 1) LowVoltage_1++;
		if(LowVoltage_2 == 1) LowVoltage_2++;

#if !TR800_DISABLE_LOW_VOLTAGE_FAULT
		if(Voltage >= 58.8f) Voltage = 58.8f;
		else if(Voltage < 49.0f) Voltage = Voltage - 1.0f;
		if(42.0f <= Voltage && Voltage < 43.0f && LowVoltage_1 < 2) LowVoltage_1++;
		else if(Voltage < 42.0f && LowVoltage_1 < 2) LowVoltage_2++;
#else
		if(Voltage >= 58.8f) Voltage = 58.8f;
#endif

		if(++oled_refresh_div >= 5)
		{
			oled_refresh_div = 0;
			oled_heartbeat++;
			OLED_Clear_GRAM();
			OLED_RenderNormalStatus(getSysTickCnt(), oled_heartbeat);
			OLED_Refresh_Gram();
		}
#endif
	}
}

#if TR800_OLED_SELF_TEST
static u8 OLED_RenderSelfTest(u32 heartbeat)
{
	u8 phase = (u8)((heartbeat / 4U) % 4U);
	TR800_OLED_DebugPhase = phase;

	if(phase == 0)
	{
		OLED_EntireDisplayOnLocal();
		return 0;
	}

	OLED_EntireDisplayResumeLocal();
	OLED_FillGramLocal(0x00);

	if(phase == 1)
	{
		return 1;
	}
	else if(phase == 2)
	{
		OLED_ShowString(0, 0, OLED_TEXT("OLED SELF TEST"));
		OLED_ShowString(0, 10, OLED_TEXT("PE5 PE6 PE7 PE8"));
		OLED_ShowString(0, 20, OLED_TEXT("FULL/CLR/TEXT/HB"));
	}
	else
	{
		OLED_ShowString(0, 0, OLED_TEXT("OLED HEARTBEAT"));
		OLED_ShowString(0, 10, OLED_TEXT("COUNT:"));
		OLED_ShowNumber(48, 10, heartbeat % 10000U, 4, 12);
		OLED_ShowString(0, 20, OLED_TEXT("SWD+SERIAL CHECK"));
	}

	OLED_ShowString(0, 50, OLED_TEXT("PH:"));
	OLED_ShowNumber(24, 50, phase, 1, 12);
	OLED_ShowString(48, 50, OLED_TEXT("HB:"));
	OLED_ShowNumber(72, 50, heartbeat % 1000U, 3, 12);
	return 1;
}
#endif

static const u8 *OLED_SystemStateShort(void)
{
	switch(tr800_runtime_get_system_state())
	{
		case TR800_BOOT_DELAY: return OLED_TEXT("BOOT");
		case TR800_READY:      return OLED_TEXT("RDY");
		case TR800_RUN:        return OLED_TEXT("RUN");
		case TR800_FAULT:      return OLED_TEXT("FLT");
		default:               return OLED_TEXT("UNK");
	}
}

static const u8 *OLED_ControlModeShort(void)
{
	switch(tr800_runtime_get_active_mode())
	{
		case TR800_MODE_SERIAL: return OLED_TEXT("SER");
		case TR800_MODE_RC:     return OLED_TEXT("RC");
		case TR800_MODE_OFF:
		default:                return OLED_TEXT("OFF");
	}
}

static void OLED_ShowSignedInt(u8 x, u8 y, long value, u8 len)
{
	if(value < 0)
	{
		OLED_ShowString(x, y, OLED_TEXT("-"));
		OLED_ShowNumber(x + 10, y, (u32)(-value), len, 12);
	}
	else
	{
		OLED_ShowString(x, y, OLED_TEXT("+"));
		OLED_ShowNumber(x + 10, y, (u32)value, len, 12);
	}
}

static void OLED_ShowVoltageCentivolt(u8 x, u8 y, int centivolt)
{
	if(centivolt < 0)
	{
		OLED_ShowString(x, y, OLED_TEXT("-"));
		centivolt = -centivolt;
		x += 8;
	}

	OLED_ShowNumber(x, y, (u32)(centivolt / 100), 2, 12);
	OLED_ShowString(x + 14, y, OLED_TEXT("."));
	OLED_ShowNumber(x + 22, y, (u32)((centivolt / 10) % 10), 1, 12);
}

static void OLED_CaptureStatusSnapshot(OLED_StatusSnapshot *out, u32 now_ms, u32 heartbeat)
{
	out->now_ms = now_ms;
	out->heartbeat = heartbeat;
	out->system_state = OLED_SystemStateShort();
	out->control_mode = OLED_ControlModeShort();
	out->fault_flags = tr800_runtime_get_fault_flags();
	out->flag_stop = Flag_Stop;
	out->rc_alive = tr800_runtime_rc_alive(now_ms);
	out->rc_power = RC_power;
	out->target_a_milli = (long)(MOTOR_A.Target * 1000.0f);
	out->target_b_milli = (long)(MOTOR_B.Target * 1000.0f);
	out->encoder_a_milli = (long)(MOTOR_A.Encoder * 1000.0f);
	out->encoder_b_milli = (long)(MOTOR_B.Encoder * 1000.0f);
	out->pwm_a = (long)MOTOR_A.Motor_Pwm;
	out->pwm_b = (long)MOTOR_B.Motor_Pwm;
	out->voltage_centivolt = (int)(Voltage * 100.0f);
}

static void OLED_RenderStatusSnapshot(const OLED_StatusSnapshot *snapshot)
{
	OLED_ShowString(0, 0, OLED_TEXT("M:"));
	OLED_ShowString(16, 0, snapshot->control_mode);
	OLED_ShowString(48, 0, snapshot->system_state);
	OLED_ShowString(80, 0, OLED_TEXT("F:"));
	OLED_ShowNumber(96, 0, snapshot->fault_flags, 3, 12);

	OLED_ShowString(0, 10, OLED_TEXT("S:"));
	OLED_ShowNumber(16, 10, snapshot->flag_stop, 1, 12);
	OLED_ShowString(32, 10, OLED_TEXT("RC:"));
	OLED_ShowNumber(56, 10, snapshot->rc_alive, 1, 12);
	OLED_ShowString(72, 10, OLED_TEXT("P:"));
	OLED_ShowNumber(88, 10, (u32)snapshot->rc_power, 4, 12);

	OLED_ShowString(0, 20, OLED_TEXT("TA"));
	OLED_ShowSignedInt(16, 20, snapshot->target_a_milli, 4);
	OLED_ShowString(68, 20, OLED_TEXT("EA"));
	OLED_ShowSignedInt(84, 20, snapshot->encoder_a_milli, 4);

	OLED_ShowString(0, 30, OLED_TEXT("TB"));
	OLED_ShowSignedInt(16, 30, snapshot->target_b_milli, 4);
	OLED_ShowString(68, 30, OLED_TEXT("EB"));
	OLED_ShowSignedInt(84, 30, snapshot->encoder_b_milli, 4);

	OLED_ShowString(0, 40, OLED_TEXT("PA"));
	OLED_ShowSignedInt(14, 40, snapshot->pwm_a, 5);
	OLED_ShowString(62, 40, OLED_TEXT("PB"));
	OLED_ShowSignedInt(76, 40, snapshot->pwm_b, 5);

	OLED_ShowString(0, 50, OLED_TEXT("V"));
	OLED_ShowVoltageCentivolt(8, 50, snapshot->voltage_centivolt);
	OLED_ShowString(60, 50, OLED_TEXT("HB"));
	OLED_ShowNumber(78, 50, snapshot->heartbeat % 1000U, 3, 12);
}

static void OLED_RenderNormalStatus(u32 now_ms, u32 heartbeat)
{
	OLED_StatusSnapshot snapshot;

	OLED_CaptureStatusSnapshot(&snapshot, now_ms, heartbeat);
	Voltage_Show = snapshot.voltage_centivolt;
	OLED_RenderStatusSnapshot(&snapshot);
}

void oled_show(void)
{
	static u32 heartbeat = 0;

	OLED_RenderNormalStatus(getSysTickCnt(), ++heartbeat);
}
