/*
* OmniFlight - 四轴飞控
* 初始化相关外设并校准各个传感器：
* 陀螺仪、磁力计、气压计各五秒，共计飞控需静止10S左右-蓝灯亮

* 校准完毕蓝灯灭
* 全部外设初始化完-蜂鸣器鸣笛蓝灯灭

* 锁定油门-红灯亮
*
* 串口分配：
*   USART1 (PA9/PA10) — 电机1 (MOTOR_ID_1)
*   USART2 (PD5/PD6)  — 电机2 (MOTOR_ID_2)
*   USART3 (PD8/PD9)  — 调试打印 (printf 输出)
*/

/* Enroll 注册层，负责把板级资源注册到 BSP */
#include "Enroll.h"

/*系统sys层*/
#include "sys.h"
#include "Delay.h"

/*API层 MCU片内外设*/
#include "usart.h"
#include "tim.h"
#include "pwm.h"

/*
 * 重定义 printf 输出到 USART3，避免与电机 USART1/2 冲突。
 * 必须在 My_Usart.h 首次被 include 之前定义。
 */
#define PRINTF_USART USART3

/*app应用层*/
#include "My_Usart/My_Usart.h"
#include "API_I2C.h"
#include "API_SPI.h"
#include "PID/PID.h"
#include "Control/Control.h"
#include "Control_Task/Control_Task.h"
#include "AimControl.h"

/* ============================================================
 * PID 调参串口清理开关
 * ============================================================
 *
 * 0：关闭 main.c 里原来的 USART4 流程打印，避免干扰 [PID_X]
 * 1：恢复 main.c 里的流程打印
 *
 * 注意：
 *   这里只关闭 MAIN_UART4_PRINTF( ...)
 *   不影响 STM32 -> K230 的 UI 包发送，也不影响 AimControl.c 里的 [PID_X] 打印。
 */
#define MAIN_UART4_LOG_ENABLE    0U

#if (MAIN_UART4_LOG_ENABLE != 0U)
#define MAIN_UART4_PRINTF(...)   usart_printf(USART4, __VA_ARGS__)
#else
#define MAIN_UART4_PRINTF(...)   do { } while (0)
#endif

/*BSP硬件抽象层*/
#include "LED.h"
#include "KEY.h"
#include "MPU6050.h"
#include "MPU6050_Int.h"
#include "Motor.h"
#include "QMC5883P.h"
#include "BMP280.h"
#include "Dshot.h" /* DShot协议 初始化 */
#include "NRF24L01.h"
#include "Buzzer.h"
#include "IMU.h"
#include "Altitude.h"
#include "Stepmotor.h" /* Emm42 闭环步进电机 */


/* ============================================================
 * 第八步 V11：TASK3 搜索到目标后延时自动切到 TASK2 RUNNING
 * ============================================================
 * 本步骤验证：
 *   1. 上电后等待步进驱动器准备好，再让 Y 轴自动 GoHome 到已保存零点；
 *   2. FIND RUNNING 时，云台跟踪靶心，但激光永远关闭；
 *   3. TASK2 RUNNING 时，云台跟踪靶心，并允许对准稳定后开激光；
 *   4. TASK3 RUNNING 时，Y 轴保持上电 GoHome 后的固定零点姿态，不再额外发 Y 轴绝对运动命令；
 *   5. X 轴先搜索；
 *   6. 搜索识别到目标后立即停止 X/Y；
 *   7. 切入 AimControl PID 锁定；
 *   8. PID 对准稳定后允许自动开激光。
 *
 * 注意：
 *   本步骤开始接入 TASK3 打靶。
 *   搜索速度仍沿用第七步 V3 的慢速参数，先保证锁定和激光逻辑正确。
 *
 * 模式：
 *   FIND  ：后续用于“跟踪靶心但不开激光”的校准/测试模式
 *   TASK2 ：后续用于任务2，稳定对准后开激光
 *   TASK3 ：后续用于任务3，搜索后切到 TASK2 打靶
 *   TASK4 ：发挥部分，小车运动时连续跟踪打靶，激光锁存常亮
 *
 * 状态：
 *   SELECT  ：模式选择中，按键循环切换 FIND -> TASK2 -> TASK3 -> TASK4 -> FIND
 *   READY   ：几秒没按键后自动进入，等待再次按键启动
 *   RUNNING ：当前任务运行中；本步骤 FIND/TASK2 会控制云台
 *   SAFE    ：安全停止状态；会停止电机并关闭激光
 */
typedef enum
{
	MAIN_AIM_MODE_FIND  = 0U,
	MAIN_AIM_MODE_TASK2 = 2U,
	MAIN_AIM_MODE_TASK3 = 3U,
	MAIN_AIM_MODE_TASK4 = 4U
} MainAimMode_t;

typedef enum
{
	MAIN_UI_SELECT  = 0U,
	MAIN_UI_READY   = 1U,
	MAIN_UI_RUNNING = 2U,
	MAIN_UI_SAFE    = 3U
} MainUiState_t;

/* 几秒没按键后自动从 SELECT 进入 READY。
 * 想更从容地选择模式，就把 2500 改大，例如 3000 或 4000。
 */
#define MAIN_UI_READY_DELAY_MS  2500U

/* 第三步：STM32 -> K230 LCD 显示协议。
 *
 * 发送串口：USART3，也就是 K230 接入 STM32 的串口。
 * 调试串口仍然是 USART4。
 *
 * 协议格式：
 *   u<mode>,<state>e
 *
 * mode:
 *   0 = FIND
 *   2 = TASK2
 *   3 = TASK3
 *   4 = TASK4
 *
 * state:
 *   0 = SELECT
 *   1 = READY
 *   2 = RUNNING
 *   3 = SAFE
 *
 * 例子：
 *   u0,0e  FIND SELECT
 *   u2,1e  TASK2 READY
 *   u3,2e  TASK3 RUNNING
 *   u4,2e  TASK4 RUNNING
 *
 * 注意：
 *   这里只在状态变化时发送，不在每一帧发送，避免占用 USART3。
 */
#define MAIN_UI_K230_UART          USART3
#define MAIN_UI_K230_SEND_ENABLE   1U
#define MAIN_UI_K230_SEND_REPEAT   2U

/* ============================================================
 * 上电等待步进驱动器准备 + Y 轴自动回固定零点
 * ============================================================
 *
 * 解决现象：
 *   STM32 上电比步进驱动器快，如果一上电马上扫描/GoHome，
 *   可能出现 scan none。这里先等待驱动器能正常应答，再进入后续初始化。
 */
#define MAIN_DRIVER_READY_TIMEOUT_MS       5000U
#define MAIN_DRIVER_RETRY_INTERVAL_MS      100U
#define MAIN_DRIVER_WAIT_PRINT_INTERVAL_MS 500U

#define MAIN_BOOT_Y_HOME_ENABLE            1U
#define MAIN_BOOT_Y_HOME_TIMEOUT_MS        8000U

/* ============================================================
 * 第七步：TASK3 搜索测试参数
 * ============================================================
 *
 * 本步骤只搜索，不开激光，不做 PID 锁定。
 */
#define TASK3_MOTOR_X                      STEPMOTOR1
#define TASK3_MOTOR_Y                      STEPMOTOR2

/* 第八步：
 * Y 轴已经在上电阶段通过 GoHome 回到保存的固定零点。
 * 搜索阶段不再给 Y 轴发送 MoveTo(0.0, ABS)，避免 0°绝对命令失败后把 X 轴也 Stop 掉。
 *
 * 后续第八步如果要做 Y 小幅摆扫，再重新加入 Y 轴搜索。
 */
#define TASK3_Y_CENTER_DEG                 (0.0f)
#define TASK3_Y_CENTER_RPM                 18U

/* X 轴搜索：相对当前位置旋转一圈多一点，避免刚好差一点扫不到。
 *
 * V3 调整：
 *   原来 22RPM 太快，摄像头从靶纸前扫过去时 K230 来不及稳定识别。
 *   这里先降到 8RPM，并把超时时间放宽到 12000ms。
 *
 * 后续调参：
 *   如果还是太快：8 -> 6 -> 5
 *   如果太慢但能识别：8 -> 10 -> 12
 */
#define TASK3_X_SEARCH_ANGLE_DEG           (-2160.0f)
#define TASK3_X_SEARCH_RPM                 6U

#define TASK3_SEARCH_TIMEOUT_MS            12000U

/* 搜索阶段先用 1 帧确认，确保“扫到一帧就停下”。
 * 这是为了测试能不能抓住目标。
 * 后续如果误识别多，再改回 2 帧。
 */
#define TASK3_SEARCH_CONFIRM_COUNT         1U

/* 第八步 V11：TASK3 找到目标后，先 Stop 停稳，再自动切到 TASK2 RUNNING。
 *
 * 比赛时程序启动后不允许二次操作，所以不能停在 TASK2 READY 等按键。
 * 这里采用非阻塞延时：找到目标 -> Stop X/Y -> 等 300ms -> 自动启动 TASK2。
 */
#define TASK3_FOUND_TO_TASK2_DELAY_MS      300U


static MainAimMode_t s_main_ui_mode = MAIN_AIM_MODE_FIND;
static MainUiState_t s_main_ui_state = MAIN_UI_SELECT;
static uint32_t s_main_ui_last_key_ms = 0U;

/* TASK4 固定周期控制使用的最新视觉缓存。
 * K230 串口来数据时只刷新这里；
 * TIM2 只置 task4_control_flag；
 * 主循环每到周期再调用 AimControl_Task4Periodic()。
 */
static int16_t s_task4_latest_target_x = 0;
static int16_t s_task4_latest_target_y = 0;
static int16_t s_task4_latest_ref_x = 0;
static int16_t s_task4_latest_ref_y = 0;
static int16_t s_task4_latest_ex = 0;
static int16_t s_task4_latest_ey = 0;
static int16_t s_task4_latest_status = 0;
static uint8_t s_task4_has_new_frame = 0U;

#if (MAIN_UART4_LOG_ENABLE != 0U)
static const char *MainUi_ModeName(MainAimMode_t mode)
{
	switch (mode)
	{
	case MAIN_AIM_MODE_FIND:
		return "FIND";
	case MAIN_AIM_MODE_TASK2:
		return "TASK2";
	case MAIN_AIM_MODE_TASK3:
		return "TASK3";
	case MAIN_AIM_MODE_TASK4:
		return "TASK4";
	default:
		return "UNKNOWN";
	}
}

static const char *MainUi_StateName(MainUiState_t state)
{
	switch (state)
	{
	case MAIN_UI_SELECT:
		return "SELECT";
	case MAIN_UI_READY:
		return "READY";
	case MAIN_UI_RUNNING:
		return "RUNNING";
	case MAIN_UI_SAFE:
		return "SAFE";
	default:
		return "UNKNOWN";
	}
}

#endif

static MainAimMode_t MainUi_NextMode(MainAimMode_t mode)
{
	switch (mode)
	{
	case MAIN_AIM_MODE_FIND:
		return MAIN_AIM_MODE_TASK2;
	case MAIN_AIM_MODE_TASK2:
		return MAIN_AIM_MODE_TASK3;
	case MAIN_AIM_MODE_TASK3:
		return MAIN_AIM_MODE_TASK4;
	case MAIN_AIM_MODE_TASK4:
	default:
		return MAIN_AIM_MODE_FIND;
	}
}

static void MainUi_SendToK230(void)
{
#if (MAIN_UI_K230_SEND_ENABLE != 0U)
	uint8_t i;

	/* 状态变化时只发很短的 UI 包，不会持续占用 USART3。
	 * 重复发送 2 次是为了降低 K230 偶尔没读到的概率。
	 */
	for (i = 0U; i < MAIN_UI_K230_SEND_REPEAT; i++)
	{
		usart_printf(MAIN_UI_K230_UART,
		             "u%d,%de",
		             (int)s_main_ui_mode,
		             (int)s_main_ui_state);
	}
#endif
}


/* ============================================================
 * 上电步进驱动器等待 / Y 轴自动回零
 * ============================================================ */
static void Main_WaitMs(uint32_t wait_ms)
{
	uint32_t start_ms;

	start_ms = Control_Task_GetMillis();
	while ((uint32_t)(Control_Task_GetMillis() - start_ms) < wait_ms)
	{
		/* 等待 TIM2 1ms 时间戳推进 */
	}
}

static uint8_t Main_WaitOneDriverReady(uint8_t motor_id,
                                       const char *motor_name,
                                       uint32_t timeout_ms)
{
#if (MAIN_UART4_LOG_ENABLE == 0U)
	(void)motor_name;
#endif
	uint32_t start_ms;
	uint32_t last_print_ms;
	MotorErrCode e;

	start_ms = Control_Task_GetMillis();
	last_print_ms = start_ms;

	MAIN_UART4_PRINTF(
	             "[BOOT] Wait %s driver ready, timeout=%lu ms\r\n",
	             (motor_name != 0) ? motor_name : "MOTOR",
	             (unsigned long)timeout_ms);

	while ((uint32_t)(Control_Task_GetMillis() - start_ms) < timeout_ms)
	{
		e = Stepmotor_SelfTest(motor_id);
		if (e == MOTOR_OK)
		{
			e = Stepmotor_Enable(motor_id);
			if (e == MOTOR_OK)
			{
				MAIN_UART4_PRINTF(
				             "[BOOT] %s ready ret=%d st=0x%02X angle10=%ld ms=%lu\r\n",
				             (motor_name != 0) ? motor_name : "MOTOR",
				             (int)e,
				             Stepmotor_ReadStatus(motor_id),
				             (long)(Stepmotor_ReadAngle(motor_id) * 10.0f),
				             (unsigned long)Control_Task_GetMillis());
				return 1U;
			}
		}

		if ((uint32_t)(Control_Task_GetMillis() - last_print_ms) >= MAIN_DRIVER_WAIT_PRINT_INTERVAL_MS)
		{
			last_print_ms = Control_Task_GetMillis();
			MAIN_UART4_PRINTF(
			             "[BOOT] waiting %s ret=%d st=0x%02X ms=%lu\r\n",
			             (motor_name != 0) ? motor_name : "MOTOR",
			             (int)e,
			             Stepmotor_ReadStatus(motor_id),
			             (unsigned long)Control_Task_GetMillis());
		}

		Main_WaitMs(MAIN_DRIVER_RETRY_INTERVAL_MS);
	}

	MAIN_UART4_PRINTF(
	             "[BOOT] %s driver wait timeout st=0x%02X ms=%lu\r\n",
	             (motor_name != 0) ? motor_name : "MOTOR",
	             Stepmotor_ReadStatus(motor_id),
	             (unsigned long)Control_Task_GetMillis());
	return 0U;
}

static void Main_WaitStepDriversReady(void)
{
	(void)Main_WaitOneDriverReady(STEPMOTOR1,
	                              "X",
	                              MAIN_DRIVER_READY_TIMEOUT_MS);

	(void)Main_WaitOneDriverReady(STEPMOTOR2,
	                              "Y",
	                              MAIN_DRIVER_READY_TIMEOUT_MS);
}

static void Main_BootYHome(void)
{
#if (MAIN_BOOT_Y_HOME_ENABLE != 0U)
	MotorErrCode e;

	MAIN_UART4_PRINTF(
	             "[BOOT] Y GoHome start, timeout=%lu ms\r\n",
	             (unsigned long)MAIN_BOOT_Y_HOME_TIMEOUT_MS);

	e = Stepmotor_GoHome(STEPMOTOR2, MAIN_BOOT_Y_HOME_TIMEOUT_MS);

	MAIN_UART4_PRINTF(
	             "[BOOT] Y GoHome ret=%d st=0x%02X angle10=%ld ms=%lu\r\n",
	             (int)e,
	             Stepmotor_ReadStatus(STEPMOTOR2),
	             (long)(Stepmotor_ReadAngle(STEPMOTOR2) * 10.0f),
	             (unsigned long)Control_Task_GetMillis());

	if (e == MOTOR_OK)
	{
		MAIN_UART4_PRINTF(
		             "[BOOT] Y HOME OK: camera should be at saved zero pose.\r\n");
	}
	else
	{
		MAIN_UART4_PRINTF(
		             "[BOOT] Y HOME FAIL: TASK3 absolute Y may be invalid.\r\n");
	}
#endif
}

/* ============================================================
 * TASK4 视觉缓存
 * ============================================================ */
static void Task4Vision_Clear(void)
{
	s_task4_latest_target_x = 0;
	s_task4_latest_target_y = 0;
	s_task4_latest_ref_x = 0;
	s_task4_latest_ref_y = 0;
	s_task4_latest_ex = 0;
	s_task4_latest_ey = 0;
	s_task4_latest_status = 0;
	s_task4_has_new_frame = 0U;
}

static void Task4Vision_Save(int16_t target_x,
                             int16_t target_y,
                             int16_t ref_x,
                             int16_t ref_y,
                             int16_t ex,
                             int16_t ey,
                             int16_t status)
{
	s_task4_latest_target_x = target_x;
	s_task4_latest_target_y = target_y;
	s_task4_latest_ref_x = ref_x;
	s_task4_latest_ref_y = ref_y;
	s_task4_latest_ex = ex;
	s_task4_latest_ey = ey;
	s_task4_latest_status = status;
	s_task4_has_new_frame = 1U;
}

static void Task4Periodic_UpdateFromFlag(void)
{
	uint8_t has_new_frame;

	if (task4_control_flag == 0U)
	{
		return;
	}

	/* 先清 flag，避免主循环处理期间下一个 tick 丢失太久。 */
	task4_control_flag = 0U;

	if ((s_main_ui_state != MAIN_UI_RUNNING) ||
	    (s_main_ui_mode != MAIN_AIM_MODE_TASK4))
	{
		/* 不在 TASK4 RUNNING 时不消费旧视觉控制，防止重新进入 TASK4 后拿旧数据追。 */
		s_task4_has_new_frame = 0U;
		return;
	}

	has_new_frame = s_task4_has_new_frame;

	AimControl_Task4Periodic(s_task4_latest_target_x,
	                         s_task4_latest_target_y,
	                         s_task4_latest_ref_x,
	                         s_task4_latest_ref_y,
	                         s_task4_latest_ex,
	                         s_task4_latest_ey,
	                         s_task4_latest_status,
	                         has_new_frame);

	/* 一个控制周期只允许用一次“新帧”做测量更新；
	 * 如果下个周期前没有新 K230 包，就进入预测保持。
	 */
	s_task4_has_new_frame = 0U;
}

/* ============================================================
 * 第七步：TASK3 搜索状态机，只搜索，不锁定，不开激光
 * ============================================================ */
typedef enum
{
	TASK3_SEARCH_IDLE = 0U,
	TASK3_SEARCH_RUNNING,
	TASK3_FOUND_DELAY,
	TASK3_FOUND_SWITCHED,
	TASK3_SEARCH_TIMEOUT
} Task3SearchState_t;

static Task3SearchState_t s_task3_search_state = TASK3_SEARCH_IDLE;
static uint32_t s_task3_search_start_ms = 0U;
static uint8_t s_task3_found_count = 0U;
static uint32_t s_task3_found_delay_start_ms = 0U;

static void Task3Search_Cancel(const char *reason)
{
#if (MAIN_UART4_LOG_ENABLE == 0U)
	(void)reason;
#endif
	if ((s_task3_search_state == TASK3_SEARCH_RUNNING) ||
	    (s_task3_search_state == TASK3_FOUND_DELAY) ||
	    (s_task3_search_state == TASK3_FOUND_SWITCHED))
	{
		(void)Stepmotor_Stop(TASK3_MOTOR_X);
		(void)Stepmotor_Stop(TASK3_MOTOR_Y);

		AimControl_SetLaserAllowed(0U);

		MAIN_UART4_PRINTF(
		             "[TASK3] cancel, state=%d, reason=%s, ms=%lu\r\n",
		             (int)s_task3_search_state,
		             (reason != 0) ? reason : "none",
		             (unsigned long)Control_Task_GetMillis());
	}

	s_task3_search_state = TASK3_SEARCH_IDLE;
	s_task3_found_count = 0U;
	s_task3_found_delay_start_ms = 0U;
}

static void Task3Search_Start(void)
{
	MotorErrCode ex_cmd;

	/* TASK3 第八步先只搜索 X，搜索阶段不允许激光亮。
	 *
	 * 注意：
	 *   Y 轴已经在上电阶段通过 Main_BootYHome() 回到保存的固定零点。
	 *   这里不再执行 Stepmotor_MoveTo(Y, 0.0f, MOTOR_MODE_ABS)。
	 *
	 * 搜索到目标后：
	 *   立即 Stop X/Y；
	 *   切换到 TASK3_LOCK_RUNNING；
	 *   允许 AimControl PID 接管并在对准稳定后开激光。
	 */
	AimControl_SetLaserAllowed(0U);
	AimControl_Reset();

	(void)Stepmotor_Stop(TASK3_MOTOR_X);

	s_task3_search_state = TASK3_SEARCH_RUNNING;
	s_task3_search_start_ms = Control_Task_GetMillis();
	s_task3_found_count = 0U;
	s_task3_found_delay_start_ms = 0U;

	MAIN_UART4_PRINTF(
	             "[TASK3] SEARCH START STEP8: Y keep home, X rel10=%ld, rpm=%d, timeout=%lu ms\r\n",
	             (long)(TASK3_X_SEARCH_ANGLE_DEG * 10.0f),
	             (int)TASK3_X_SEARCH_RPM,
	             (unsigned long)TASK3_SEARCH_TIMEOUT_MS);

	/* X 轴开始相对旋转搜索。 */
	ex_cmd = Stepmotor_MoveTo(TASK3_MOTOR_X,
	                          TASK3_X_SEARCH_ANGLE_DEG,
	                          TASK3_X_SEARCH_RPM,
	                          MOTOR_ACCEL_DIRECT,
	                          MOTOR_MODE_REL);

	MAIN_UART4_PRINTF(
	             "[TASK3] X search cmd ret=%d, x_st=0x%02X, x_angle10=%ld\r\n",
	             (int)ex_cmd,
	             Stepmotor_ReadStatus(TASK3_MOTOR_X),
	             (long)(Stepmotor_ReadAngle(TASK3_MOTOR_X) * 10.0f));

	if (ex_cmd != MOTOR_OK)
	{
		(void)Stepmotor_Stop(TASK3_MOTOR_X);
		s_task3_search_state = TASK3_SEARCH_IDLE;

		MAIN_UART4_PRINTF(
		             "[TASK3] SEARCH START FAIL: X motor command error, laser off.\r\n");
	}
}

static void Task3Search_OnVision(int16_t target_x,
                                 int16_t target_y,
                                 int16_t ref_x,
                                 int16_t ref_y,
                                 int16_t ex,
                                 int16_t ey,
                                 int16_t status)
{
	if (s_task3_search_state == TASK3_SEARCH_RUNNING)
	{
		if (status != 0)
		{
			if (s_task3_found_count < 255U)
			{
				s_task3_found_count++;
			}

			if (s_task3_found_count >= TASK3_SEARCH_CONFIRM_COUNT)
			{
				/* V11：
				 *   TASK3 只负责搜索；
				 *   找到靶子后立即停止 X/Y；
				 *   不在这一帧开 PID；
				 *   进入 FOUND_DELAY，等待 TASK3_FOUND_TO_TASK2_DELAY_MS；
				 *   延时结束后由 Task3Search_Update() 自动切到 TASK2 RUNNING。
				 */
				(void)Stepmotor_Stop(TASK3_MOTOR_X);
				(void)Stepmotor_Stop(TASK3_MOTOR_Y);

				AimControl_SetLaserAllowed(0U);
				AimControl_Reset();

				s_task3_search_state = TASK3_FOUND_DELAY;
				s_task3_found_delay_start_ms = Control_Task_GetMillis();

				MAIN_UART4_PRINTF(
				             "[TASK3] FOUND -> DELAY: target=(%d,%d), ref=(%d,%d), e=(%d,%d), cnt=%d, delay=%lu ms, ms=%lu\r\n",
				             (int)target_x,
				             (int)target_y,
				             (int)ref_x,
				             (int)ref_y,
				             (int)ex,
				             (int)ey,
				             (int)s_task3_found_count,
				             (unsigned long)TASK3_FOUND_TO_TASK2_DELAY_MS,
				             (unsigned long)s_task3_found_delay_start_ms);

				MAIN_UART4_PRINTF(
				             "[TASK3] X/Y stopped. PID disabled during delay, then auto start TASK2.\r\n");
			}
		}
		else
		{
			s_task3_found_count = 0U;
		}

		return;
	}

	if ((s_task3_search_state == TASK3_FOUND_DELAY) ||
	    (s_task3_search_state == TASK3_FOUND_SWITCHED))
	{
		/* 延时/已切换阶段，TASK3 不再直接处理视觉控制。 */
		(void)target_x;
		(void)target_y;
		(void)ref_x;
		(void)ref_y;
		(void)ex;
		(void)ey;
		(void)status;
	}
}

static void MainUi_Print(const char *reason);
static void Task3Search_Update(void)
{
	uint32_t now;

	now = Control_Task_GetMillis();

	if (s_task3_search_state == TASK3_SEARCH_RUNNING)
	{
		if ((uint32_t)(now - s_task3_search_start_ms) >= TASK3_SEARCH_TIMEOUT_MS)
		{
			(void)Stepmotor_Stop(TASK3_MOTOR_X);
			(void)Stepmotor_Stop(TASK3_MOTOR_Y);
			AimControl_SetLaserAllowed(0U);

			s_task3_search_state = TASK3_SEARCH_TIMEOUT;

			MAIN_UART4_PRINTF(
			             "[TASK3] SEARCH TIMEOUT: motors stopped before switch, laser off, ms=%lu\r\n",
			             (unsigned long)now);
		}
	}
	else if (s_task3_search_state == TASK3_FOUND_DELAY)
	{
		if ((uint32_t)(now - s_task3_found_delay_start_ms) >= TASK3_FOUND_TO_TASK2_DELAY_MS)
		{
			/* 延时结束后，自动进入 TASK2 RUNNING。
			 * 这里不再需要二次按键，适合比赛启动后无人操作流程。
			 */
			s_task3_search_state = TASK3_FOUND_SWITCHED;

			s_main_ui_mode = MAIN_AIM_MODE_TASK2;
			s_main_ui_state = MAIN_UI_RUNNING;
			s_main_ui_last_key_ms = now;

			Task4Vision_Clear();
			AimControl_SetTask4Mode(0U);
			AimControl_Reset();
			AimControl_SetLaserAllowed(1U);

			MainUi_SendToK230();

			MAIN_UART4_PRINTF(
			             "[TASK3] DELAY DONE -> TASK2 RUNNING: auto tracking and laser allowed, ms=%lu\r\n",
			             (unsigned long)now);

			MainUi_Print("TASK3_FOUND_AUTO_TASK2_RUNNING");
		}
	}
}

static void MainUi_Print(const char *reason)
{
#if (MAIN_UART4_LOG_ENABLE == 0U)
	(void)reason;
#endif
	MAIN_UART4_PRINTF(
	             "[UI] %s %s, ms=%lu, reason=%s\r\n",
	             MainUi_ModeName(s_main_ui_mode),
	             MainUi_StateName(s_main_ui_state),
	             (unsigned long)Control_Task_GetMillis(),
	             (reason != 0) ? reason : "none");

	/* 状态变化时同步发送给 K230，让 K230 LCD 显示当前模式。 */
	MainUi_SendToK230();
}

/*
 * MainUi_StartCurrentMode：从 READY 进入 RUNNING。
 *
 * 第八步接入：
 *   FIND  RUNNING：云台跟踪靶心，但激光强制关闭；
 *   TASK2 RUNNING：云台跟踪靶心，对准稳定后允许自动开激光；
 *   TASK3 RUNNING：Y 轴保持上电固定零点，X 轴开始搜索，找到目标后停止，延时后自动切到 TASK2 RUNNING。
 */
static void MainUi_EnterSafe(const char *reason);
static void MainUi_StartCurrentMode(void)
{
	s_main_ui_state = MAIN_UI_RUNNING;
	MainUi_Print("START");

	if (s_main_ui_mode == MAIN_AIM_MODE_FIND)
	{
		Task3Search_Cancel("START_FIND");
		Task4Vision_Clear();

		/* FIND：只跟踪，不允许激光。 */
		AimControl_SetTask4Mode(0U);
		AimControl_SetLaserAllowed(0U);
		AimControl_Reset();

		MAIN_UART4_PRINTF(
		             "[UI] FIND RUNNING: tracking enabled, laser disabled.\r\n");
	}
	else if (s_main_ui_mode == MAIN_AIM_MODE_TASK2)
	{
		Task3Search_Cancel("START_TASK2");
		Task4Vision_Clear();

		/* TASK2：普通定点打靶。
		 * 激光逻辑保持原来方式：对准稳定才开，偏离/丢失会关。
		 */
		AimControl_SetTask4Mode(0U);
		AimControl_Reset();
		AimControl_SetLaserAllowed(1U);

		MAIN_UART4_PRINTF(
		             "[UI] TASK2 RUNNING: tracking enabled, laser stable-only.\r\n");
	}
	else if (s_main_ui_mode == MAIN_AIM_MODE_TASK3)
	{
		/* TASK3：搜索到靶纸后延时自动切到 TASK2 RUNNING。 */
		Task4Vision_Clear();
		AimControl_SetTask4Mode(0U);
		Task3Search_Start();

		MAIN_UART4_PRINTF(
		             "[UI] TASK3 RUNNING: search only, found -> delay -> TASK2 RUNNING.\r\n");
	}
	else if (s_main_ui_mode == MAIN_AIM_MODE_TASK4)
	{
		Task3Search_Cancel("START_TASK4");
		Task4Vision_Clear();

		/* TASK4：发挥部分。
		 *   - 不搜索，启动时靶纸应已在 K230 画面内；
		 *   - 使用现有 PID + 二状态卡尔曼持续跟踪；
		 *   - 初次开激光条件与 TASK2 相同；
		 *   - 一旦激光打开，进入锁存常亮；
		 *   - 之后 status 短暂丢失/误差变大，都不自动关激光；
		 *   - 只有 SAFE / 退出任务才关激光。
		 */
		AimControl_SetTask4Mode(1U);
		AimControl_Reset();
		AimControl_SetLaserAllowed(1U);

		MAIN_UART4_PRINTF(
		             "[UI] TASK4 RUNNING: tracking enabled, laser latch after lock.\r\n");
	}
	else
	{
		MainUi_EnterSafe("UNKNOWN_MODE");
	}
}

/*
 * MainUi_EnterSafe：进入安全状态。
 *
 * 不管当前是什么模式，只要进入 SAFE：
 *   1. 停止 X/Y 电机；
 *   2. 清 PID / 卡尔曼；
 *   3. 关闭激光。
 */
static void MainUi_EnterSafe(const char *reason)
{
	Task3Search_Cancel(reason);
	Task4Vision_Clear();
	AimControl_Stop();

	s_main_ui_state = MAIN_UI_SAFE;
	MainUi_Print(reason);

	MAIN_UART4_PRINTF(
	             "[UI] SAFE: motors stopped, laser off.\r\n");
}

static void MainUi_Init(void)
{
	s_main_ui_mode = MAIN_AIM_MODE_FIND;
	s_main_ui_state = MAIN_UI_SELECT;
	s_main_ui_last_key_ms = Control_Task_GetMillis();

	MainUi_Print("BOOT");
	MAIN_UART4_PRINTF(
	             "[UI] Step9 TASK4 FIX1: latch laser, forward declarations fixed.\r\n");
}

static void MainUi_Update(uint8_t key_event)
{
	uint32_t now;

	now = Control_Task_GetMillis();

	if (key_event != 0U)
	{
		s_main_ui_last_key_ms = now;

		MAIN_UART4_PRINTF(
		             "[KEY] event=%d, ms=%lu\r\n",
		             (int)key_event,
		             (unsigned long)now);

		switch (s_main_ui_state)
		{
		case MAIN_UI_SELECT:
			/* 选择中：每按一次切换一个模式。 */
			s_main_ui_mode = MainUi_NextMode(s_main_ui_mode);
			MainUi_Print("MODE_NEXT");
			break;

		case MAIN_UI_READY:
			/* READY 后再按一次，进入 RUNNING。 */
			MainUi_StartCurrentMode();
			break;

		case MAIN_UI_RUNNING:
			/* 运行中按一下进入 SAFE，立即停止电机并关闭激光。 */
			MainUi_EnterSafe("SAFE");
			break;

		case MAIN_UI_SAFE:
		default:
			/* SAFE 中按一下回到 SELECT，保留当前模式。
			 * 为安全起见，再执行一次 AimControl_Stop()。
			 */
			Task3Search_Cancel("BACK_SELECT");
			Task4Vision_Clear();
			AimControl_Stop();
			s_main_ui_state = MAIN_UI_SELECT;
			s_main_ui_last_key_ms = now;
			MainUi_Print("BACK_SELECT");
			break;
		}

		return;
	}

	/* SELECT 状态下，超过指定时间没有按键，自动 READY。 */
	if (s_main_ui_state == MAIN_UI_SELECT)
	{
		if ((uint32_t)(now - s_main_ui_last_key_ms) >= MAIN_UI_READY_DELAY_MS)
		{
			s_main_ui_state = MAIN_UI_READY;
			MainUi_Print("AUTO_READY");
		}
	}
}

int main(void)
{
/* 系统时钟配置初始化 */
	SYS_Init();
/* 注册层：注册相关资源，登记资源映射 */
	Enroll_LED_Register();					/* LED 资源注册 */
	Enroll_KEY_Register();					/* KEY 资源注册 */
	Enroll_USART_Register();				/* USART 资源注册 */
	Enroll_PWM_Register();					/* PWM 资源注册（Buzzer: TIM3 CH4） */
	Enroll_TIM_Register();					/* TIM 资源注册 */
	Enroll_I2C_Register();					/* I2C 资源注册 */
	Enroll_SPI_Register();					/* SPI 资源注册 */
	Enroll_NRF24L01_Register();				/* NRF24L01 CE 引脚注册 */

	/* 注册后绑定中断回调*/
	Enroll_USART_RegisterIrqHandler(Control_Task_USART_Callback); 		/* USART 中断回调：注册时自动使能异步 TX */
	API_TIM_RegisterIrqHandler(API_TIM1, Control_Task1_Callback);           	/* TIM1: PID */
	API_TIM_RegisterIrqHandler(API_TIM2, Control_Task2_Callback);               /* TIM2: printf/时间戳 */

/* 初始化层：初始化相关外设，启动硬件功能 */
	API_USART_Init(API_USART1, 115200U); // 初始化 USART1 (电机1)
	API_USART_Init(API_USART2, 115200U); // 初始化 USART2 (电机2)
	API_USART_Init(API_USART3, 115200U); // 初始化 USART3 (调试打印)
	API_USART_Init(API_USART4, 115200U); // 初始化 UART4 (PC10/PC11)
	Stepmotor_Init();                // 关电机 USART 的 RXNEIE + NVIC IRQ

	API_TIM_Init(API_TIM1, 1U); /* TIM1: PID 节拍，每 1ms */
	API_TIM_Init(API_TIM2, 1U); /* TIM2: printf/时间戳  每 1ms */
	API_PWM_Init(API_PWM_TIM3, (1000000U / 2700U) - 1, 84U - 1U);

/* 通信协议初始化 */
	API_I2C_Init();						/* 软件 I2C 初始化 */
	API_SPI_Init();						/* 软件 SPI 初始化 */
	// App_I2C_ScanOnce();				/* 开机执行一次 I2C 扫描 */
	// App_SPI_TestOnce();				/* 开机执行一次 SPI 测试 */

/*BSP硬件抽象层初始化*/
	LED_Init(LED_LOW);	/* LED 初始化-低电平 */
	KEY_Init();	/* KEY 初始化 */
    Main_WaitStepDriversReady();
    AimControl_Init();
    Main_BootYHome();
	/* ═══════════════════════════════════════════════════════════
	 * 闭环步进电机 — 上电自检 + 运动测试
	 *
	 * STEPMOTOR1 → USART1 (PA9/PA10)
	 * STEPMOTOR2 → USART2 (PD5/PD6)
	 * 调试输出  → USART3 (PD8/PD9), 串口助手 115200bps
	 *
	 * 直接改下面函数里的角度和速度即可，后续可替换为变量。
	 *
	 * 首次校准：取消 Calibrate/SetOrigin/GoHome 注释，空载跑一次后注回。
	 * ═══════════════════════════════════════════════════════════ */
	// {
	// 	MotorErrCode e;

	// 	usart_printf(USART3, "[SM] Init\r\n");

	// 	/* ── STEPMOTOR1 (USART1) ── */
	// 	e = Stepmotor_SelfTest(STEPMOTOR1);
	// 	usart_printf(USART3, "[SM] Motor1 scan: %s\r\n",
	// 	             (e == MOTOR_OK) ? "FOUND" : "none");

	// 	if (e == MOTOR_OK)
	// 	{
	// 		e = Stepmotor_Enable(STEPMOTOR1);
	// 		usart_printf(USART3, "[SM] Motor1 En=%d St=0x%02X\r\n",
	// 		             (int)e, Stepmotor_ReadStatus(STEPMOTOR1));

	// 		/* 首次校准（空载！） */
	// 		// e = Stepmotor_Calibrate(STEPMOTOR1, 30000U);
	// 		// usart_printf(USART3, "[SM] Motor1 Cal=%d\r\n", (int)e);
	// 		// if (e == MOTOR_OK) {
	// 		//     e = Stepmotor_SetOrigin(STEPMOTOR1, 1U);
	// 		//     usart_printf(USART3, "[SM] Motor1 Org=%d\r\n", (int)e);
	// 		// }
	// 		// if (e == MOTOR_OK) {
	// 		//     e = Stepmotor_GoHome(STEPMOTOR1, 60000U);
	// 		//     usart_printf(USART3, "[SM] Motor1 Home=%d\r\n", (int)e);
	// 		// }

	// 		/* 运动：+90° @30RPM，直启，相对模式 */
	// 		e = Stepmotor_GoTo(STEPMOTOR1, 360.0f, 3U, 0U,
	// 		                   MOTOR_MODE_REL, 10000U);
	// 		usart_printf(USART3, "[SM] Motor1 +90 =%d ang=%.1f\r\n",
	// 		             (int)e, (double)Stepmotor_ReadAngle(STEPMOTOR1));
	// 		Delay_ms(500U);

	// 		e = Stepmotor_GoTo(STEPMOTOR1, -360.0f, 3U, 0U,
	// 		                   MOTOR_MODE_REL, 10000U);
	// 		usart_printf(USART3, "[SM] Motor1 -90 =%d ang=%.1f\r\n",
	// 		             (int)e, (double)Stepmotor_ReadAngle(STEPMOTOR1));
	// 	}

	// 	/* ── STEPMOTOR2 (USART2) ── */
	// 	e = Stepmotor_SelfTest(STEPMOTOR2);
	// 	usart_printf(USART3, "[SM] Motor2 scan: %s\r\n",
	// 	             (e == MOTOR_OK) ? "FOUND" : "none");

	// 	if (e == MOTOR_OK)
	// 	{
	// 		e = Stepmotor_Enable(STEPMOTOR2);
	// 		usart_printf(USART3, "[SM] Motor2 En=%d St=0x%02X\r\n",
	// 		             (int)e, Stepmotor_ReadStatus(STEPMOTOR2));

	// 		/* 首次校准（空载！） */
	// 		// e = Stepmotor_Calibrate(STEPMOTOR2, 30000U);
	// 		// usart_printf(USART3, "[SM] Motor2 Cal=%d\r\n", (int)e);
	// 		// if (e == MOTOR_OK) {
	// 		//     e = Stepmotor_SetOrigin(STEPMOTOR2, 1U);
	// 		//     usart_printf(USART3, "[SM] Motor2 Org=%d\r\n", (int)e);
	// 		// }
	// 		// if (e == MOTOR_OK) {
	// 		//     e = Stepmotor_GoHome(STEPMOTOR2, 60000U);
	// 		//     usart_printf(USART3, "[SM] Motor2 Home=%d\r\n", (int)e);
	// 		// }

	// 		/* 运动：+90° @30RPM，直启，相对模式 */
	// 		e = Stepmotor_GoTo(STEPMOTOR2, 360.0f, 3U, 0U,
	// 		                   MOTOR_MODE_REL, 10000U);
	// 		usart_printf(USART3, "[SM] Motor2 +90 =%d ang=%.1f\r\n",
	// 		             (int)e, (double)Stepmotor_ReadAngle(STEPMOTOR2));
	// 		Delay_ms(500U);

	// 		e = Stepmotor_GoTo(STEPMOTOR2, -360.0f, 3U, 0U,
	// 		                   MOTOR_MODE_REL, 10000U);
	// 		usart_printf(USART3, "[SM] Motor2 -90 =%d ang=%.1f\r\n",
	// 		             (int)e, (double)Stepmotor_ReadAngle(STEPMOTOR2));
	// 	}
	// }

	MainUi_Init();

	while (1)
	{
		uint8_t key_event;

		/* 读取一次性按键事件。
		 * Step1 中 key_Get() 已经改成：有新事件时 Key=1/2/3/4，无新事件时 Key=0。
		 */
		key_Get();
		key_event = (uint8_t)Key;
		Key = 0U;

		/* 第七步：UI 状态机 + K230 LCD 显示 + FIND/TASK2 + TASK3搜索。 */
		MainUi_Update(key_event);

		/*
		 * K230 数据包格式：
		 *   s target_x,target_y,ref_x,ref_y,ex,ey,status e
		 * 示例：
		 *   s358,259,284,117,74,142,1e
		 *
		 * 本步骤仍然保留串口解析。
		 * 只有在 FIND RUNNING 或 TASK2 RUNNING 时，才调用 AimControl_Update()：
		 *   FIND  ：跟踪靶心，但激光禁止；
		 *   TASK2 ：跟踪靶心，对准稳定后允许开激光；
		 *   TASK3 ：X 轴搜索，识别到目标后停止。
		 */
		if (USART_DataTypeStruct.state == 2U)
		{
			uint8_t i;

			USART_Packet_Count = USART_DataTypeStruct.count;

			for (i = 0U; i < USART_Packet_Count; i++)
			{
				USART_Packet_Data[i] = USART_Deal(&USART_DataTypeStruct, (int8_t)i);
			}

			USART_DataTypeStruct.state = 0U;

			if (USART_Packet_Count == 7U)
			{
				int16_t target_x = USART_Packet_Data[0];
				int16_t target_y = USART_Packet_Data[1];
				int16_t ref_x    = USART_Packet_Data[2];
				int16_t ref_y    = USART_Packet_Data[3];
				int16_t ex       = USART_Packet_Data[4];
				int16_t ey       = USART_Packet_Data[5];
				int16_t status   = USART_Packet_Data[6];

				/* 第五步接入 FIND + TASK2：
				 *   FIND RUNNING：
				 *      调用 AimControl_Update() 跟踪靶心；
				 *      MainUi_StartCurrentMode() 已经设置 AimControl_SetLaserAllowed(0U)，所以不会开激光。
				 *
				 *   TASK2 RUNNING：
				 *      调用 AimControl_Update() 跟踪靶心；
				 *      MainUi_StartCurrentMode() 已经设置 AimControl_SetLaserAllowed(1U)，所以对准稳定后允许开激光。
				 *
				 *   TASK3：
				 *      第七步 V2 只做 X 轴搜索，识别到目标后停止，不开激光。
				 */
				if ((s_main_ui_state == MAIN_UI_RUNNING) &&
				    ((s_main_ui_mode == MAIN_AIM_MODE_FIND) ||
				     (s_main_ui_mode == MAIN_AIM_MODE_TASK2)))
				{
					AimControl_Update(target_x,
					                  target_y,
					                  ref_x,
					                  ref_y,
					                  ex,
					                  ey,
					                  status);
				}
				else if ((s_main_ui_state == MAIN_UI_RUNNING) &&
				         (s_main_ui_mode == MAIN_AIM_MODE_TASK4))
				{
					/* TASK4 不再“来一帧就控制一次”。
					 * 这里仅缓存最新视觉数据，由固定周期 flag 触发控制。
					 */
					Task4Vision_Save(target_x,
					                 target_y,
					                 ref_x,
					                 ref_y,
					                 ex,
					                 ey,
					                 status);
				}
				else if ((s_main_ui_state == MAIN_UI_RUNNING) &&
				         (s_main_ui_mode == MAIN_AIM_MODE_TASK3))
				{
					Task3Search_OnVision(target_x,
					                     target_y,
					                     ref_x,
					                     ref_y,
					                     ex,
					                     ey, 
					                     status);
				}
				else
				{
					(void)target_x;
					(void)target_y;
					(void)ref_x;
					(void)ref_y;
					(void)ex;
					(void)ey;
					(void)status;
				}
			}
			else
			{
				/* 数据数量不对时，不控制电机。 */
			}
		}

		/* TASK4 固定周期控制：TIM2 只置 flag，主循环里执行控制和串口发送电机命令。 */
		Task4Periodic_UpdateFromFlag();

		Task3Search_Update();
	}
}
