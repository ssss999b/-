#include "AimControl.h"
#include "StepMotor.h"
#include "PID.h"
#include "usart.h"
#include "My_Usart/My_Usart.h"
#include "LED.h"
#include "Control_Task.h"
#include <stdint.h>

/* ============================================================
 * 用户可调参数区
 * ============================================================ */

/* 电机映射：
 * STEPMOTOR1：X 轴，水平左右方向
 * STEPMOTOR2：Y 轴，垂直上下方向
 */
#define AIM_MOTOR_X              STEPMOTOR1
#define AIM_MOTOR_Y              STEPMOTOR2

/* 方向符号：
 * 如果某一轴越追越远，就把对应轴从 1.0f 改成 -1.0f，或者反过来。
 * 注意：KP/KI/KD 建议保持正数，方向只在这里改。
 */
#define AIM_X_DIR_SIGN           (1.0f)
#define AIM_Y_DIR_SIGN           (1.0f)

/* 控制分频：
 * K230 大约 50~60fps。
 * AIM_CMD_DIV = 2 表示每收到 2 帧有效数据，给步进电机发一次运动指令。
 */
#define AIM_CMD_DIV              1U

/* PID 采样周期：
 * 需要和 AIM_CMD_DIV 对应。
 * AIM_CMD_DIV = 1，建议 0.018f
 * AIM_CMD_DIV = 2，建议 0.035f
 * AIM_CMD_DIV = 3，建议 0.055f
 * AIM_CMD_DIV = 4，建议 0.070f
 */
#define AIM_PID_DT_S             (0.018f)

/* X 轴 PID 参数 */
#define AIM_X_KP                 (0.008f)  //0.008f
#define AIM_X_KI                 (0.003f)  //0.002f
#define AIM_X_KD                 (0.00f)

/* Y 轴 PID 参数 */
#define AIM_Y_KP                 (0.01f)
#define AIM_Y_KI                 (0.003f)
#define AIM_Y_KD                 (0.00f)

/* ============================================================
 * TASK4 独立 PID 参数
 * ============================================================
 *
 * TASK2/FIND 继续使用上面的 AIM_X/Y_KP/KI/KD；
 * TASK4 进入 RUNNING 后会自动切换到下面这一套参数。
 *
 * 调参建议：
 *   - 小车运动时激光落后：适当增大 KP，或增大 TASK4_KF_LEAD_TIME；
 *   - 来回摆/超调：减小 KP，减小 LEAD_TIME，或增大 KF_R；
 *   - 长期固定偏差：小幅增加 KI；
 *   - 细碎抖动：减小 KP，增大 KF_R，减小 Q_ACCEL。
 */
#define AIM_TASK4_X_KP           (0.0125f)  //0.012f
#define AIM_TASK4_X_KI           (0.005f)  //0.004f
#define AIM_TASK4_X_KD           (0.0f)

#define AIM_TASK4_Y_KP           (0.010f)   //0.010f
#define AIM_TASK4_Y_KI           (0.003f)   //0.003f
#define AIM_TASK4_Y_KD           (0.00f)

/* 像素死区：
 * 当 |ex| 或 |ey| 小于死区时，对应轴 PID 输出会变为 0。
 */
#define AIM_DEADBAND_X_PX        (1.0f)
#define AIM_DEADBAND_Y_PX        (1.0f)

/* 单次相对运动角度限制 */
#define AIM_OUT_MAX_DEG          (360.00f)
#define AIM_MIN_STEP_DEG         (0.001f)

/* 步进电机运动参数 */
#define AIM_RPM                  12U
#define AIM_ACCEL                MOTOR_ACCEL_DIRECT
#define AIM_MOTOR_MODE           MOTOR_MODE_REL

/* ============================================================
 * TASK4 独立输出限制 / 电机速度参数
 * ============================================================
 *
 * TASK4_OUT_MAX_DEG:
 *   单次 PID 输出最大角度。减小更稳，增大更跟手。
 *
 * TASK4_MIN_STEP_DEG:
 *   最小步进补偿。0 表示不强制补偿；
 *   如果输出太小电机不动，可设到 0.10~0.12。
 *
 * TASK4_RPM:
 *   TASK4 跟踪速度。增大更快但容易过冲，减小更稳但可能跟不上小车。
 *
 * TASK4_OUT_SLEW_MAX_STEP:
 *   输出斜率限制。减小更丝滑但滞后，增大更跟手但可能抖。
 */
#define AIM_TASK4_OUT_MAX_DEG       (360.00f)
#define AIM_TASK4_MIN_STEP_DEG      (0.001f)
#define AIM_TASK4_RPM               12U
#define AIM_TASK4_OUT_SLEW_ENABLE   1U
#define AIM_TASK4_OUT_SLEW_MAX_STEP (1.20f)

/* 目标丢失保护 */
#define AIM_LOST_STOP_COUNT      10U

/* ref 改变自动复位阈值 */
#define AIM_REF_RESET_THRESHOLD_PX   8

/* ============================================================
 * 视觉误差二状态卡尔曼滤波参数
 * ============================================================
 *
 * 这里不是你 Filter.h 里的 Kalman2D_Angle。
 * 这里新写的是“位置 + 速度”的二状态卡尔曼：
 *
 *   state = [误差位置 error_px, 误差速度 error_px_per_s]
 *
 * 用途：
 *   小车运动时，ex/ey 会连续变化。
 *   这个滤波器可以估计误差变化速度，减少一阶低通那种明显滞后。
 *
 * AIM_KF_ENABLE:
 *   1：启用二状态卡尔曼滤波
 *   0：关闭滤波，直接用原始 ex/ey
 *
 * AIM_KF_R:
 *   视觉测量噪声，单位约等于 像素^2。
 *   越大越平滑，但越慢；越小越跟手，但越抖。
 *   16 表示大约认为视觉噪声标准差为 4 像素。
 *
 * AIM_KF_Q_ACCEL:
 *   误差加速度过程噪声。
 *   越大越相信误差会快速变化，也就是更跟手；
 *   越小越平滑，但更慢。
 *
 * AIM_KF_LEAD_TIME_S:
 *   预测提前量。
 *   0.000f：只输出当前滤波误差
 *   0.015~0.030f：略微向前预测，适合小车运动，减少滞后
 *   如果出现超调/来回摆，先改回 0.000f。
 */
#define AIM_KF_ENABLE            1U
#define AIM_KF_R                 (16.0f)
#define AIM_KF_Q_ACCEL           (8000.0f)
#define AIM_KF_LEAD_TIME_S       (0.020f)

/* ============================================================
 * TASK4 独立二状态卡尔曼参数
 * ============================================================
 *
 * TASK4_KF_R:
 *   视觉测量噪声。增大更稳但更慢，减小更跟手但更抖。
 *
 * TASK4_KF_Q_ACCEL:
 *   误差加速度过程噪声。增大更跟手，减小更平滑。
 *
 * TASK4_KF_LEAD_TIME_S:
 *   预测提前量。增大可补偿小车运动滞后，但太大会超前/来回摆。
 *   作用是补偿视觉识别、串口通信、电机响应这些延迟。
 */
#define AIM_TASK4_KF_ENABLE         1U
#define AIM_TASK4_KF_R              (8.0f)                 //R 越大：越不相信当前视觉输出,更平滑,但是反应更慢             R 越小：越相信当前视觉 反应更快,但是更容易抖
#define AIM_TASK4_KF_Q_ACCEL        (11000.0f)               //Q_ACCEL 越大：认为目标变化更快,更跟手,滞后更小,但更容易抖    Q_ACCEL 越小： 更平滑,更稳,但更容易跟不上
#define AIM_TASK4_KF_LEAD_TIME_S    (0.040f)                //LEAD_TIME 越大：越提前,更容易解决滞后                       LEAD_TIME 太大：会超前,会过冲,会来回摆 

/* TASK4 固定周期 / 真实 dt / 短丢失预测参数。
 *
 * TASK4 现在由 main.c 按 task4_control_flag 固定周期调用；
 * 本文件内部仍用 Control_Task_GetMillis() 计算真实 dt，并做限幅保护。
 */
#define AIM_TASK4_DT_DEFAULT_S             (0.015f)
#define AIM_TASK4_DT_MIN_S                 (0.005f)
#define AIM_TASK4_DT_MAX_S                 (0.050f)

/* 短丢失预测保持周期数。
 * 5 个周期 × 15ms ≈ 75ms。
 * 在这段时间内不 Stop，继续用卡尔曼预测误差追一小段。
 */
#define AIM_TASK4_LOST_PREDICT_FRAMES      5U

/* 输出斜率限制：
 * 作用是限制每次电机角度指令的变化速度，让运动更丝滑。
 * 如果感觉响应变慢，可调大；如果还是一顿一顿，可调小。
 */
#define AIM_OUT_SLEW_ENABLE      1U
#define AIM_OUT_SLEW_MAX_STEP    (1.20f)


/* ============================================================
 * 激光控制参数
 * ============================================================
 *
 * 任务 2 定点打靶逻辑：
 *   当 K230 已经识别到靶心，并且靶心 target 与紫色参考点 ref
 *   连续多帧重合到一定误差范围内时，自动打开激光。
 *
 * 当前硬件连接：
 *   激光接在 PE8，对应工程里的 LED2。
 *   LED_Control(LED2, LED_HIGH) 表示打开激光。
 *   LED_Control(LED2, LED_LOW)  表示关闭激光。
 */
#define AIM_LASER_ENABLE             1U
#define AIM_LASER_ID                 LED2
#define AIM_LASER_ON_LEVEL           LED_HIGH
#define AIM_LASER_OFF_LEVEL          LED_LOW

/* 激光开启判定误差阈值，单位：像素 */
#define AIM_LASER_ALIGN_X_PX         7
#define AIM_LASER_ALIGN_Y_PX         7

/* 连续多少帧都对准后，才真正打开激光 */
#define AIM_LASER_STABLE_COUNT       5U

/* TASK4：发挥部分连续打靶，使用独立 PID/KF 参数。
 * 初次开激光条件与 TASK2 共用 AIM_LASER_ALIGN_X/Y 和 AIM_LASER_STABLE_COUNT。
 * 但一旦开灯后会锁存常亮，短暂丢目标/误差变大不自动关灯。
 *
 * Step10：TASK4 不再一丢目标就立即 Stop；
 * 固定周期控制函数会在短丢失期间继续预测，连续丢失太久才 Stop。
 */
#define AIM_TASK4_LOST_STOP_IMMEDIATE 0U

/* 调试串口 */
#define AIM_DEBUG_ENABLE         1U
#define AIM_DEBUG_UART           USART4
#define AIM_DEBUG_DIV            5U

/* MoveTo 高频调试：
 * 调 PID 时建议保持 0，只保留低频状态打印。
 */
#define AIM_MOVE_DEBUG_ENABLE    0U

/* PID 调参专用打印：
 * 1：打开 [PID_X] 打印
 * 0：关闭 [PID_X] 打印
 *
 * 打印内容：
 *   ref_x      = X轴目标值
 *   target_x   = X轴实际值
 *   x_angle    = X轴最终 PID 输出角度，单位用毫度打印
 */
#define AIM_PID_X_DEBUG_ENABLE       1U
#define AIM_PID_X_DEBUG_DIV          5U

/* 旧的 AimControl 串口打印总开关：
 * 0：关闭原来的 [AIM] Init / laser / 状态 等打印，只保留 [PID_X]
 * 1：恢复旧的 [AIM] 打印
 */
#define AIM_OLD_UART4_LOG_ENABLE     0U

#if (AIM_OLD_UART4_LOG_ENABLE != 0U)
#define AIM_OLD_DEBUG_PRINTF(...)    usart_printf(AIM_DEBUG_UART, __VA_ARGS__)
#else
#define AIM_OLD_DEBUG_PRINTF(...)    do { } while (0)
#endif

/* ============================================================
 * 二状态卡尔曼滤波器结构体
 * ============================================================ */

typedef struct
{
    float x;      /* 位置状态：误差，单位 px */
    float v;      /* 速度状态：误差变化速度，单位 px/s */

    float p00;    /* 协方差矩阵 P */
    float p01;
    float p10;
    float p11;

    uint8_t inited;
} AimKalmanCV1D_t;

/* ============================================================
 * 内部状态变量
 * ============================================================ */

/* X/Y 两个 PID 控制器对象 */
static PID_TypeDef s_pid_x;
static PID_TypeDef s_pid_y;

/* 电机在线标志 */
static uint8_t s_motor_x_ok = 0U;
static uint8_t s_motor_y_ok = 0U;

/* 目标丢失状态 */
static uint8_t s_lost_stopped = 0U;
static uint8_t s_lost_cnt = 0U;

/* 控制分频计数器 */
static uint8_t s_cmd_div_cnt = 0U;

/* 调试打印分频计数器 */
static uint8_t s_debug_div_cnt = 0U;

/* ref 参考点记录 */
static uint8_t s_ref_inited = 0U;
static int16_t s_last_ref_x = 0;
static int16_t s_last_ref_y = 0;

/* X/Y 视觉误差二状态卡尔曼滤波器 */
static AimKalmanCV1D_t s_kf_ex;
static AimKalmanCV1D_t s_kf_ey;

/* 输出斜率限制所需的上一帧电机角度 */
static float s_last_x_angle = 0.0f;
static float s_last_y_angle = 0.0f;


/* 激光状态变量：
 * s_laser_on        ：当前软件认为激光是否打开
 * s_laser_align_cnt : 连续对准帧数
 */
static uint8_t s_laser_on = 0U;
static uint8_t s_laser_align_cnt = 0U;

/* 激光允许标志。
 *
 * 1：允许 aim_laser_update() 根据误差稳定情况自动开激光；
 * 0：强制禁止开激光。即使已经对准，也会保持关闭。
 *
 * 第四步 FIND 模式会把它设为 0，用于“云台跟踪但不开激光”的安全测试。
 */
static uint8_t s_laser_allowed = 0U;

/* TASK4 激光锁存模式：
 * s_task4_mode = 1 时：
 *   - 未锁存前，开灯条件与 TASK2 一样；
 *   - 一旦 s_task4_laser_latched = 1，激光保持常亮；
 *   - status 短暂丢失或误差变大都不自动关灯。
 */
static uint8_t s_task4_mode = 0U;
static uint8_t s_task4_laser_latched = 0U;

/* TASK4 固定周期预测控制状态 */
static uint32_t s_task4_last_update_ms = 0U;
static uint8_t s_task4_lost_predict_cnt = 0U;


/* ============================================================
 * 内部辅助函数
 * ============================================================ */

/* 求 float 绝对值 */
static float aim_abs_f(float x)
{
    return (x >= 0.0f) ? x : -x;
}

/* 求 int16_t 绝对值 */
static int16_t aim_abs_i16(int16_t x)
{
    return (x >= 0) ? x : (int16_t)(-x);
}


/* 控制激光开关：
 * on = 1：打开激光
 * on = 0：关闭激光
 *
 * 这里做了状态记忆，避免每一帧都重复写 GPIO。
 * 但是 AimControl_Init() 中会强制写一次关闭电平，保证上电安全。
 */
static void aim_laser_set(uint8_t on)
{
#if (AIM_LASER_ENABLE != 0U)
    if (on != 0U)
    {
        if (s_laser_on == 0U)
        {
            LED_Control(AIM_LASER_ID, AIM_LASER_ON_LEVEL);
            s_laser_on = 1U;

#if (AIM_DEBUG_ENABLE != 0U)
            AIM_OLD_DEBUG_PRINTF( "[AIM] laser ON\r\n");
#endif
        }
    }
    else
    {
        if (s_laser_on != 0U)
        {
            LED_Control(AIM_LASER_ID, AIM_LASER_OFF_LEVEL);
            s_laser_on = 0U;

#if (AIM_DEBUG_ENABLE != 0U)
            AIM_OLD_DEBUG_PRINTF( "[AIM] laser OFF\r\n");
#endif
        }
    }
#else
    (void)on;
#endif
}

/* 激光对准判断：
 * status = 1，且 ex/ey 连续多帧进入阈值，才打开激光。
 * 目标丢失、误差变大时，立即关闭激光。
 *
 * 注意：
 *   这里使用原始 ex/ey 判断，不使用卡尔曼预测后的 ex_f/ey_f。
 *   这样可以避免预测值提前满足条件而导致激光过早打开。
 */
static void aim_laser_update(int16_t ex, int16_t ey, int16_t status)
{
#if (AIM_LASER_ENABLE != 0U)
    /* FIND / SAFE / 禁止激光时，必须强制关灯，并清 TASK4 锁存。 */
    if (s_laser_allowed == 0U)
    {
        s_laser_align_cnt = 0U;
        s_task4_laser_latched = 0U;
        aim_laser_set(0U);
        return;
    }

    /* TASK4：激光锁存常亮模式。
     * 初次开灯条件与 TASK2 一样；
     * 一旦锁存，后续 status=0 或误差变大都不自动关灯。
     */
    if (s_task4_mode != 0U)
    {
        if (s_task4_laser_latched != 0U)
        {
            aim_laser_set(1U);
            return;
        }

        if (status == 0)
        {
            s_laser_align_cnt = 0U;
            aim_laser_set(0U);
            return;
        }

        if ((aim_abs_i16(ex) <= AIM_LASER_ALIGN_X_PX) &&
            (aim_abs_i16(ey) <= AIM_LASER_ALIGN_Y_PX))
        {
            if (s_laser_align_cnt < AIM_LASER_STABLE_COUNT)
            {
                s_laser_align_cnt++;
            }

            if (s_laser_align_cnt >= AIM_LASER_STABLE_COUNT)
            {
                s_task4_laser_latched = 1U;
                aim_laser_set(1U);
            }
        }
        else
        {
            s_laser_align_cnt = 0U;
            aim_laser_set(0U);
        }

        return;
    }

    /* 普通 TASK2 逻辑：目标丢失或偏离阈值就关灯。 */
    if (status == 0)
    {
        s_laser_align_cnt = 0U;
        aim_laser_set(0U);
        return;
    }

    if ((aim_abs_i16(ex) <= AIM_LASER_ALIGN_X_PX) &&
        (aim_abs_i16(ey) <= AIM_LASER_ALIGN_Y_PX))
    {
        if (s_laser_align_cnt < AIM_LASER_STABLE_COUNT)
        {
            s_laser_align_cnt++;
        }

        if (s_laser_align_cnt >= AIM_LASER_STABLE_COUNT)
        {
            aim_laser_set(1U);
        }
    }
    else
    {
        s_laser_align_cnt = 0U;
        aim_laser_set(0U);
    }
#else
    (void)ex;
    (void)ey;
    (void)status;
#endif
}

/* 绝对值限幅 */
static float aim_limit_abs(float x, float max_abs)
{
    max_abs = aim_abs_f(max_abs);

    if (max_abs <= 0.0f)
    {
        return 0.0f;
    }

    if (x > max_abs)
    {
        return max_abs;
    }

    if (x < -max_abs)
    {
        return -max_abs;
    }

    return x;
}

/* 最小步进补偿 */
static float aim_apply_min_step_ex(float x, float min_step)
{
    float a;

    if (x == 0.0f)
    {
        return 0.0f;
    }

    min_step = aim_abs_f(min_step);
    if (min_step <= 0.0f)
    {
        return x;
    }

    a = aim_abs_f(x);

    if (a < min_step)
    {
        return (x > 0.0f) ? min_step : -min_step;
    }

    return x;
}

/* 斜率限制：
 * 限制当前输出相对上一帧输出的最大变化量。
 */
static float aim_slew_rate(float input, float last_output, float max_step)
{
    float diff;

    max_step = aim_abs_f(max_step);
    diff = input - last_output;

    if (diff > max_step)
    {
        diff = max_step;
    }
    else if (diff < -max_step)
    {
        diff = -max_step;
    }

    return last_output + diff;
}

/* 二状态卡尔曼初始化 */
static void aim_kf_init(AimKalmanCV1D_t *kf, float init_pos)
{
    if (kf == 0)
    {
        return;
    }

    kf->x = init_pos;
    kf->v = 0.0f;

    /* 初始不确定性：
     * 位置给 100，速度给 1000，让刚启动时可以较快收敛。
     */
    kf->p00 = 100.0f;
    kf->p01 = 0.0f;
    kf->p10 = 0.0f;
    kf->p11 = 1000.0f;

    kf->inited = 1U;
}

/* 二状态卡尔曼复位 */
static void aim_kf_reset(AimKalmanCV1D_t *kf)
{
    if (kf == 0)
    {
        return;
    }

    kf->x = 0.0f;
    kf->v = 0.0f;
    kf->p00 = 0.0f;
    kf->p01 = 0.0f;
    kf->p10 = 0.0f;
    kf->p11 = 0.0f;
    kf->inited = 0U;
}

/* 二状态卡尔曼更新：
 * measurement：当前 K230 测得的误差，单位 px
 * dt         ：采样周期，单位 s
 * 返回值     ：滤波并可略微预测后的误差
 */
static float aim_kf_update(AimKalmanCV1D_t *kf, float measurement, float dt, float r, float q_accel, float lead_time)
{
    float dt2;
    float dt3;
    float dt4;
    float q00;
    float q01;
    float q11;
    float x_pred;
    float v_pred;
    float p00_pred;
    float p01_pred;
    float p10_pred;
    float p11_pred;
    float s;
    float k0;
    float k1;
    float y;
    float out;

    if (kf == 0)
    {
        return measurement;
    }

    if (dt <= 0.0f)
    {
        dt = AIM_PID_DT_S;
    }

    if (kf->inited == 0U)
    {
        aim_kf_init(kf, measurement);
        return measurement;
    }

    dt2 = dt * dt;
    dt3 = dt2 * dt;
    dt4 = dt2 * dt2;

    /* 常加速度模型过程噪声 Q */
    q00 = 0.25f * dt4 * q_accel;
    q01 = 0.5f  * dt3 * q_accel;
    q11 = dt2 * q_accel;

    /* 预测：x = x + v*dt，v = v */
    x_pred = kf->x + (kf->v * dt);
    v_pred = kf->v;

    /* P = FPF' + Q */
    p00_pred = kf->p00 + dt * (kf->p10 + kf->p01) + dt2 * kf->p11 + q00;
    p01_pred = kf->p01 + dt * kf->p11 + q01;
    p10_pred = kf->p10 + dt * kf->p11 + q01;
    p11_pred = kf->p11 + q11;

    /* 测量更新，H = [1, 0] */
    s = p00_pred + r;
    if (s <= 1e-6f)
    {
        s = 1e-6f;
    }

    k0 = p00_pred / s;
    k1 = p10_pred / s;
    y = measurement - x_pred;

    kf->x = x_pred + k0 * y;
    kf->v = v_pred + k1 * y;

    kf->p00 = (1.0f - k0) * p00_pred;
    kf->p01 = (1.0f - k0) * p01_pred;
    kf->p10 = p10_pred - k1 * p00_pred;
    kf->p11 = p11_pred - k1 * p01_pred;

    /* 轻微提前预测，补偿视觉和串口控制延迟 */
    out = kf->x + (kf->v * lead_time);

    return out;
}


/* 二状态卡尔曼仅预测：
 * 没有新视觉测量时使用。
 * 作用：短暂丢帧/短暂 status=0 时，不拿旧数据重复更新，只根据 x/v 预测。
 */
static float aim_kf_predict_only(AimKalmanCV1D_t *kf,
                                 float dt,
                                 float q_accel,
                                 float lead_time)
{
    float dt2;
    float dt3;
    float dt4;
    float q00;
    float q01;
    float q11;
    float x_pred;
    float v_pred;
    float p00_pred;
    float p01_pred;
    float p10_pred;
    float p11_pred;

    if (kf == 0)
    {
        return 0.0f;
    }

    if (kf->inited == 0U)
    {
        return 0.0f;
    }

    if (dt <= 0.0f)
    {
        dt = AIM_TASK4_DT_DEFAULT_S;
    }

    dt2 = dt * dt;
    dt3 = dt2 * dt;
    dt4 = dt2 * dt2;

    q00 = 0.25f * dt4 * q_accel;
    q01 = 0.5f  * dt3 * q_accel;
    q11 = dt2 * q_accel;

    x_pred = kf->x + (kf->v * dt);
    v_pred = kf->v;

    p00_pred = kf->p00 + dt * (kf->p10 + kf->p01) + dt2 * kf->p11 + q00;
    p01_pred = kf->p01 + dt * kf->p11 + q01;
    p10_pred = kf->p10 + dt * kf->p11 + q01;
    p11_pred = kf->p11 + q11;

    kf->x = x_pred;
    kf->v = v_pred;
    kf->p00 = p00_pred;
    kf->p01 = p01_pred;
    kf->p10 = p10_pred;
    kf->p11 = p11_pred;

    return kf->x + (kf->v * lead_time);
}

/* TASK4 使用真实 dt。
 * 第一次调用或异常时使用默认 15ms，之后用 Control_Task_GetMillis() 计算真实周期。
 */
static float aim_task4_get_dt_s(void)
{
    uint32_t now_ms;
    uint32_t diff_ms;
    float dt;

    now_ms = Control_Task_GetMillis();

    if (s_task4_last_update_ms == 0U)
    {
        s_task4_last_update_ms = now_ms;
        return AIM_TASK4_DT_DEFAULT_S;
    }

    diff_ms = (uint32_t)(now_ms - s_task4_last_update_ms);
    s_task4_last_update_ms = now_ms;

    dt = ((float)diff_ms) / 1000.0f;

    if (dt < AIM_TASK4_DT_MIN_S)
    {
        dt = AIM_TASK4_DT_MIN_S;
    }
    else if (dt > AIM_TASK4_DT_MAX_S)
    {
        dt = AIM_TASK4_DT_MAX_S;
    }

    return dt;
}

/* PID 初始化配置 */
static void aim_pid_setup(PID_TypeDef *pid,
                          float kp,
                          float ki,
                          float kd,
                          float deadband_px)
{
    PID_Init(pid);

    /* 使用位置式 PID */
    PID_SetMode(pid, PID_MODE_POSITION);

    /* 目标值设为 0，表示希望 ex/ey 最终为 0 */
    PID_SetTarget(pid, 0.0f);

    /* 设置 PID 采样周期 */
    PID_SetSampleTime(pid, AIM_PID_DT_S);

    /* 设置像素死区 */
    PID_SetDeadband(pid, deadband_px);

    /* 设置 PID 输出限幅，也就是单次最大运动角度 */
    PID_SetOutputLimit(pid, AIM_OUT_MAX_DEG);

    /* 设置积分限幅，防止积分项无限积累 
     X轴最大积分输出 = 0.20 × 500 = 100°*******************************************
     Y轴最大积分输出 = 0.15 × 500 = 75°********************************************/
    PID_SetIntegralLimit(pid, 500.0f);

    /* 积分分离阈值 */
    PID_SetIntegralSeparation(pid, 1000.0f);

    /* 微分低通滤波系数，当前 KD 为 0 时基本不起作用 */
    PID_SetDerivativeLPF(pid, 0.25f);

    /* 开启抗积分饱和 */
    PID_SetAntiWindup(pid, PID_ENABLE);

    /* 写入 PID 参数 */
    Set_PID(pid, kp, ki, kd);
}

/* 检查 ref 是否变化：
 * 返回 1：ref 发生明显变化，已经自动 Reset PID，本次建议跳过控制。
 * 返回 0：ref 没有明显变化，可以继续控制。
 */
static uint8_t aim_check_ref_changed(int16_t ref_x, int16_t ref_y)
{
    int16_t dx;
    int16_t dy;

    if (s_ref_inited == 0U)
    {
        s_last_ref_x = ref_x;
        s_last_ref_y = ref_y;
        s_ref_inited = 1U;
        return 0U;
    }

    dx = (int16_t)(ref_x - s_last_ref_x);
    dy = (int16_t)(ref_y - s_last_ref_y);

    if ((aim_abs_i16(dx) > AIM_REF_RESET_THRESHOLD_PX) ||
        (aim_abs_i16(dy) > AIM_REF_RESET_THRESHOLD_PX))
    {
        AimControl_Reset();

#if (AIM_DEBUG_ENABLE != 0U)
        AIM_OLD_DEBUG_PRINTF(
                     "[AIM] ref changed, reset PID old:%d,%d new:%d,%d\r\n",
                     s_last_ref_x,
                     s_last_ref_y,
                     ref_x,
                     ref_y);
#endif

        s_last_ref_x = ref_x;
        s_last_ref_y = ref_y;

        return 1U;
    }

    return 0U;
}

/* 低频状态打印：
 * E 是原始误差，F 是卡尔曼滤波/预测后的误差。
 */
static void aim_debug_print(int16_t target_x,
                            int16_t target_y,
                            int16_t ref_x,
                            int16_t ref_y,
                            int16_t ex,
                            int16_t ey,
                            int16_t status,
                            float ex_f,
                            float ey_f,
                            float x_angle,
                            float y_angle)
{
    (void)target_x;
    (void)target_y;
    (void)ref_x;
    (void)ref_y;
    (void)ex;
    (void)ey;
    (void)status;
    (void)ex_f;
    (void)ey_f;
    (void)x_angle;
    (void)y_angle;

#if (AIM_DEBUG_ENABLE != 0U)
    s_debug_div_cnt++;
    if (s_debug_div_cnt < AIM_DEBUG_DIV)
    {
        return;
    }
    s_debug_div_cnt = 0U;

    AIM_OLD_DEBUG_PRINTF(
                 "[AIM] T:%d,%d R:%d,%d E:%d,%d F:%.1f,%.1f S:%d X:%.2f Y:%.2f\r\n",
                 target_x,
                 target_y,
                 ref_x,
                 ref_y,
                 ex,
                 ey,
                 (double)ex_f,
                 (double)ey_f,
                 status,
                 (double)x_angle,
                 (double)y_angle);
#else
    (void)target_x;
    (void)target_y;
    (void)ref_x;
    (void)ref_y;
    (void)ex;
    (void)ey;
    (void)status;
    (void)ex_f;
    (void)ey_f;
    (void)x_angle;
    (void)y_angle;
#endif
}

/* ============================================================
 * 对外接口函数
 * ============================================================ */

/* 清空 PID、卡尔曼滤波器和控制计数器 */
void AimControl_Reset(void)
{
    PID_Reset(&s_pid_x);
    PID_Reset(&s_pid_y);

    aim_kf_reset(&s_kf_ex);
    aim_kf_reset(&s_kf_ey);

    s_last_x_angle = 0.0f;
    s_last_y_angle = 0.0f;

    s_cmd_div_cnt = 0U;
    s_debug_div_cnt = 0U;

    s_task4_last_update_ms = 0U;
    s_task4_lost_predict_cnt = 0U;

    /* 普通模式 Reset 必须关激光。
     * TASK4 已经锁存开灯后，Reset 只清 PID/滤波，不关激光；
     * 否则短暂丢目标/ref 抖动会破坏“连续打靶”。
     */
    if ((s_task4_mode != 0U) && (s_task4_laser_latched != 0U))
    {
        aim_laser_set(1U);
    }
    else
    {
        s_laser_align_cnt = 0U;
        aim_laser_set(0U);
    }
}

/* 设置是否允许自动开激光。
 *
 * allowed = 0：
 *   强制关闭激光，并清除连续对准计数。
 *
 * allowed != 0：
 *   允许 AimControl_Update() 在误差稳定后自动开激光。
 *
 * 第四步只接入 FIND 模式，所以 main.c 会调用 AimControl_SetLaserAllowed(0U)。
 */
void AimControl_SetLaserAllowed(uint8_t allowed)
{
    if (allowed != 0U)
    {
        s_laser_allowed = 1U;
    }
    else
    {
        s_laser_allowed = 0U;
        s_laser_align_cnt = 0U;
        s_task4_laser_latched = 0U;
        aim_laser_set(0U);
    }
}

/* 设置 TASK4 激光锁存模式。
 *
 * enable = 0：
 *   普通 FIND/TASK2/TASK3 逻辑。
 *
 * enable != 0：
 *   TASK4 逻辑：初次开灯条件与 TASK2 相同，开灯后锁存常亮。
 */
void AimControl_SetTask4Mode(uint8_t enable)
{
    if (enable != 0U)
    {
        s_task4_mode = 1U;
        s_task4_laser_latched = 0U;
        s_laser_align_cnt = 0U;
        s_task4_last_update_ms = 0U;
        s_task4_lost_predict_cnt = 0U;

        /* TASK4 使用独立 PID 参数。
         * main.c 随后还会调用 AimControl_Reset() 清状态。
         */
        aim_pid_setup(&s_pid_x,
                      AIM_TASK4_X_KP,
                      AIM_TASK4_X_KI,
                      AIM_TASK4_X_KD,
                      AIM_DEADBAND_X_PX);
        aim_pid_setup(&s_pid_y,
                      AIM_TASK4_Y_KP,
                      AIM_TASK4_Y_KI,
                      AIM_TASK4_Y_KD,
                      AIM_DEADBAND_Y_PX);
    }
    else
    {
        s_task4_mode = 0U;
        s_task4_laser_latched = 0U;
        s_laser_align_cnt = 0U;
        s_task4_last_update_ms = 0U;
        s_task4_lost_predict_cnt = 0U;

        /* 退出 TASK4 后恢复普通 FIND/TASK2 参数。 */
        aim_pid_setup(&s_pid_x,
                      AIM_X_KP,
                      AIM_X_KI,
                      AIM_X_KD,
                      AIM_DEADBAND_X_PX);
        aim_pid_setup(&s_pid_y,
                      AIM_Y_KP,
                      AIM_Y_KI,
                      AIM_Y_KD,
                      AIM_DEADBAND_Y_PX);
    }
}

/* 安全停止：
 *   1. 停止 X/Y 电机；
 *   2. 禁止并关闭激光；
 *   3. 清 PID / 卡尔曼 / 计数器。
 *
 * main.c 进入 SAFE、离开 RUNNING 时调用。
 */
void AimControl_Stop(void)
{
    if (s_motor_x_ok != 0U)
    {
        (void)Stepmotor_Stop(AIM_MOTOR_X);
    }

    if (s_motor_y_ok != 0U)
    {
        (void)Stepmotor_Stop(AIM_MOTOR_Y);
    }

    s_task4_mode = 0U;
    s_task4_laser_latched = 0U;
    s_task4_last_update_ms = 0U;
    s_task4_lost_predict_cnt = 0U;

    AimControl_SetLaserAllowed(0U);
    AimControl_Reset();
}

/* 瞄准控制初始化 */
void AimControl_Init(void)
{
    MotorErrCode e;

    s_motor_x_ok = 0U;
    s_motor_y_ok = 0U;
    s_lost_stopped = 0U;
    s_lost_cnt = 0U;
    s_cmd_div_cnt = 0U;
    s_debug_div_cnt = 0U;

    s_ref_inited = 0U;
    s_last_ref_x = 0;
    s_last_ref_y = 0;

    s_last_x_angle = 0.0f;
    s_last_y_angle = 0.0f;

    /* 上电初始化时，先强制关闭激光，避免误亮。
     * 默认禁止激光，只有后续 TASK2/TASK3 正式接入时才会允许。
     */
    s_laser_allowed = 0U;
    s_task4_mode = 0U;
    s_task4_laser_latched = 0U;
    s_task4_last_update_ms = 0U;
    s_task4_lost_predict_cnt = 0U;
    s_laser_on = 0U;
    s_laser_align_cnt = 0U;
    LED_Control(AIM_LASER_ID, AIM_LASER_OFF_LEVEL);

    aim_kf_reset(&s_kf_ex);
    aim_kf_reset(&s_kf_ey);

    aim_pid_setup(&s_pid_x, AIM_X_KP, AIM_X_KI, AIM_X_KD, AIM_DEADBAND_X_PX);
    aim_pid_setup(&s_pid_y, AIM_Y_KP, AIM_Y_KI, AIM_Y_KD, AIM_DEADBAND_Y_PX);

#if (AIM_DEBUG_ENABLE != 0U)
    AIM_OLD_DEBUG_PRINTF( "[AIM] Init start\r\n");
#endif

    /* 扫描并使能 X 轴电机 */
    e = Stepmotor_SelfTest(AIM_MOTOR_X);
#if (AIM_DEBUG_ENABLE != 0U)
    AIM_OLD_DEBUG_PRINTF( "[AIM] X scan:%s\r\n", (e == MOTOR_OK) ? "FOUND" : "none");
#endif
    if (e == MOTOR_OK)
    {
        e = Stepmotor_Enable(AIM_MOTOR_X);
#if (AIM_DEBUG_ENABLE != 0U)
        AIM_OLD_DEBUG_PRINTF( "[AIM] X enable=%d st=0x%02X\r\n",
                     (int)e,
                     Stepmotor_ReadStatus(AIM_MOTOR_X));
#endif
        if (e == MOTOR_OK)
        {
            s_motor_x_ok = 1U;
        }
    }

    /* 扫描并使能 Y 轴电机 */
    e = Stepmotor_SelfTest(AIM_MOTOR_Y);
#if (AIM_DEBUG_ENABLE != 0U)
    AIM_OLD_DEBUG_PRINTF( "[AIM] Y scan:%s\r\n", (e == MOTOR_OK) ? "FOUND" : "none");
#endif
    if (e == MOTOR_OK)
    {
        e = Stepmotor_Enable(AIM_MOTOR_Y);
#if (AIM_DEBUG_ENABLE != 0U)
        AIM_OLD_DEBUG_PRINTF( "[AIM] Y enable=%d st=0x%02X\r\n",
                     (int)e,
                     Stepmotor_ReadStatus(AIM_MOTOR_Y));
#endif
        if (e == MOTOR_OK)
        {
            s_motor_y_ok = 1U;
        }
    }

#if (AIM_DEBUG_ENABLE != 0U)
    AIM_OLD_DEBUG_PRINTF( "[AIM] Init done X=%d Y=%d\r\n",
                 s_motor_x_ok,
                 s_motor_y_ok);
#endif
}

/* 瞄准控制更新函数：
 * 每当 main.c 从 K230 收到一帧完整数据后，调用一次本函数。
 */
void AimControl_Update(int16_t target_x,
                       int16_t target_y,
                       int16_t ref_x,
                       int16_t ref_y,
                       int16_t ex,
                       int16_t ey,
                       int16_t status)
{
    float ex_f = 0.0f;
    float ey_f = 0.0f;
    float x_angle = 0.0f;
    float y_angle = 0.0f;
    uint16_t motor_rpm = AIM_RPM;
    MotorErrCode x_cmd = MOTOR_OK;
    MotorErrCode y_cmd = MOTOR_OK;

    /* 目标丢失处理 */
    if (status == 0)
    {
        /* TASK4 中：激光已经锁存后不能因为短暂丢帧关灯。
         * 同时停止 X/Y，避免视觉不稳时继续乱追。
         */
        aim_laser_update(ex, ey, status);

        if (s_task4_mode != 0U)
        {
#if (AIM_TASK4_LOST_STOP_IMMEDIATE != 0U)
            if (s_lost_stopped == 0U)
            {
                if (s_motor_x_ok != 0U)
                {
                    (void)Stepmotor_Stop(AIM_MOTOR_X);
                }
                if (s_motor_y_ok != 0U)
                {
                    (void)Stepmotor_Stop(AIM_MOTOR_Y);
                }

                s_lost_stopped = 1U;
            }
#endif
            return;
        }

        /* 普通 FIND/TASK2：沿用原来的目标丢失保护，丢失后关灯并 Reset。 */
        if (s_lost_cnt < 255U)
        {
            s_lost_cnt++;
        }

        if ((s_lost_cnt >= AIM_LOST_STOP_COUNT) && (s_lost_stopped == 0U))
        {
            if (s_motor_x_ok != 0U)
            {
                (void)Stepmotor_Stop(AIM_MOTOR_X);
            }
            if (s_motor_y_ok != 0U)
            {
                (void)Stepmotor_Stop(AIM_MOTOR_Y);
            }

            AimControl_Reset();
            s_lost_stopped = 1U;

#if (AIM_DEBUG_ENABLE != 0U)
            AIM_OLD_DEBUG_PRINTF( "[AIM] target lost, stop\r\n");
#endif
        }

        return;
    }

    /* 当前帧目标有效，清除丢失计数 */
    s_lost_cnt = 0U;
    s_lost_stopped = 0U;

    /* ref 变化时清 PID 和卡尔曼，跳过当前帧 */
    if (aim_check_ref_changed(ref_x, ref_y) != 0U)
    {
        return;
    }

    /* 激光对准判断每帧都执行，不参与 AIM_CMD_DIV 分频。
     * 这样一旦对准或偏离，激光能更快响应。
     */
    aim_laser_update(ex, ey, status);

    /* 控制分频 */
    s_cmd_div_cnt++;
    if (s_cmd_div_cnt < AIM_CMD_DIV)
    {
        return;
    }
    s_cmd_div_cnt = 0U;

    if (s_task4_mode != 0U)
    {
#if (AIM_TASK4_KF_ENABLE != 0U)
        /* TASK4 使用独立二状态卡尔曼滤波/预测误差 */
        ex_f = aim_kf_update(&s_kf_ex,
                             (float)ex,
                             AIM_PID_DT_S,
                             AIM_TASK4_KF_R,
                             AIM_TASK4_KF_Q_ACCEL,
                             AIM_TASK4_KF_LEAD_TIME_S);
        ey_f = aim_kf_update(&s_kf_ey,
                             (float)ey,
                             AIM_PID_DT_S,
                             AIM_TASK4_KF_R,
                             AIM_TASK4_KF_Q_ACCEL,
                             AIM_TASK4_KF_LEAD_TIME_S);
#else
        ex_f = (float)ex;
        ey_f = (float)ey;
#endif
    }
    else
    {
#if (AIM_KF_ENABLE != 0U)
        /* 普通 FIND/TASK2 使用原来的二状态卡尔曼参数 */
        ex_f = aim_kf_update(&s_kf_ex,
                             (float)ex,
                             AIM_PID_DT_S,
                             AIM_KF_R,
                             AIM_KF_Q_ACCEL,
                             AIM_KF_LEAD_TIME_S);
        ey_f = aim_kf_update(&s_kf_ey,
                             (float)ey,
                             AIM_PID_DT_S,
                             AIM_KF_R,
                             AIM_KF_Q_ACCEL,
                             AIM_KF_LEAD_TIME_S);
#else
        ex_f = (float)ex;
        ey_f = (float)ey;
#endif
    }

    /* PID uses error = Target - Actual.
     * Set Target=0 and Actual=-ex_f, then PID error equals ex_f.
     */
    x_angle = PID_CalcDt(&s_pid_x, -ex_f, AIM_PID_DT_S) * AIM_X_DIR_SIGN;
    y_angle = PID_CalcDt(&s_pid_y, -ey_f, AIM_PID_DT_S) * AIM_Y_DIR_SIGN;

    if (s_task4_mode != 0U)
    {
        motor_rpm = AIM_TASK4_RPM;

        x_angle = aim_limit_abs(x_angle, AIM_TASK4_OUT_MAX_DEG);
        y_angle = aim_limit_abs(y_angle, AIM_TASK4_OUT_MAX_DEG);

#if (AIM_TASK4_OUT_SLEW_ENABLE != 0U)
        x_angle = aim_slew_rate(x_angle, s_last_x_angle, AIM_TASK4_OUT_SLEW_MAX_STEP);
        y_angle = aim_slew_rate(y_angle, s_last_y_angle, AIM_TASK4_OUT_SLEW_MAX_STEP);
#endif

        x_angle = aim_apply_min_step_ex(x_angle, AIM_TASK4_MIN_STEP_DEG);
        y_angle = aim_apply_min_step_ex(y_angle, AIM_TASK4_MIN_STEP_DEG);
    }
    else
    {
        motor_rpm = AIM_RPM;

        x_angle = aim_limit_abs(x_angle, AIM_OUT_MAX_DEG);
        y_angle = aim_limit_abs(y_angle, AIM_OUT_MAX_DEG);

#if (AIM_OUT_SLEW_ENABLE != 0U)
        x_angle = aim_slew_rate(x_angle, s_last_x_angle, AIM_OUT_SLEW_MAX_STEP);
        y_angle = aim_slew_rate(y_angle, s_last_y_angle, AIM_OUT_SLEW_MAX_STEP);
#endif

        /* 对非零输出做最小步进补偿 */
        x_angle = aim_apply_min_step_ex(x_angle, AIM_MIN_STEP_DEG);
        y_angle = aim_apply_min_step_ex(y_angle, AIM_MIN_STEP_DEG);
    }

    s_last_x_angle = x_angle;
    s_last_y_angle = y_angle;


#if (AIM_DEBUG_ENABLE != 0U) && (AIM_PID_X_DEBUG_ENABLE != 0U)
    {
        static uint8_t s_pid_y_debug_div_cnt = 0U;
        int32_t y_out_mdeg;

        s_pid_y_debug_div_cnt++;
        if (s_pid_y_debug_div_cnt >= AIM_PID_X_DEBUG_DIV)
        {
            s_pid_y_debug_div_cnt = 0U;

            /* y_angle 是已经经过限幅、斜率限制、最小步进补偿后的最终输出。
             * 单位换成 0.001° 打印，避免 printf 浮点支持问题。
             */
            y_out_mdeg = (int32_t)(y_angle * 1000.0f);

            usart_printf(AIM_DEBUG_UART,
                         "[PID_Y] ref=%d target=%d out_mdeg=%ld\r\n",
                         (int)ref_y,
                         (int)target_y,
                         (long)y_out_mdeg);
        }
    }
#endif

    /* 给 X 轴发送相对运动命令 */
    if ((x_angle != 0.0f) && (s_motor_x_ok != 0U))
    {
        x_cmd = Stepmotor_MoveTo(AIM_MOTOR_X,
                                 x_angle,
                                 motor_rpm,
                                 AIM_ACCEL,
                                 AIM_MOTOR_MODE);

#if (AIM_DEBUG_ENABLE != 0U) && (AIM_MOVE_DEBUG_ENABLE != 0U)
        AIM_OLD_DEBUG_PRINTF(
                     "[AIM] MoveX ret=%d angle=%.2f\r\n",
                     (int)x_cmd,
                     (double)x_angle);
#endif
    }

    /* 给 Y 轴发送相对运动命令 */
    if ((y_angle != 0.0f) && (s_motor_y_ok != 0U))
    {
        y_cmd = Stepmotor_MoveTo(AIM_MOTOR_Y,
                                 y_angle,
                                 motor_rpm,
                                 AIM_ACCEL,
                                 AIM_MOTOR_MODE);

#if (AIM_DEBUG_ENABLE != 0U) && (AIM_MOVE_DEBUG_ENABLE != 0U)
        AIM_OLD_DEBUG_PRINTF(
                     "[AIM] MoveY ret=%d angle=%.2f\r\n",
                     (int)y_cmd,
                     (double)y_angle);
#endif
    }

    /* 低频打印当前状态 */
    aim_debug_print(target_x,
                    target_y,
                    ref_x,
                    ref_y,
                    ex,
                    ey,
                    status,
                    ex_f,
                    ey_f,
                    x_angle,
                    y_angle);

    (void)x_cmd;
    (void)y_cmd;
}


/* TASK4 固定周期预测控制。
 *
 * 与 AimControl_Update() 的区别：
 *   1. 由 main.c 按 task4_control_flag 固定周期调用，不再依赖 K230 帧到达时间；
 *   2. 使用真实 dt；
 *   3. 有新 status=1 帧时做卡尔曼测量更新；
 *   4. 没有新帧或 status=0 时，短时间只做卡尔曼预测，不立即 Stop；
 *   5. 连续丢失太久才 Stop，激光锁存状态仍保持；
 *   6. FIX1：无新帧周期不再清激光对准计数，避免锁定不开灯。
 */
void AimControl_Task4Periodic(int16_t target_x,
                              int16_t target_y,
                              int16_t ref_x,
                              int16_t ref_y,
                              int16_t ex,
                              int16_t ey,
                              int16_t status,
                              uint8_t has_new_frame)
{
    uint8_t measurement_valid;
    float dt;
    float ex_f = 0.0f;
    float ey_f = 0.0f;
    float x_angle = 0.0f;
    float y_angle = 0.0f;
    MotorErrCode x_cmd = MOTOR_OK;
    MotorErrCode y_cmd = MOTOR_OK;

    /* 本函数只服务 TASK4。非 TASK4 时直接退出，防止误调用。 */
    if (s_task4_mode == 0U)
    {
        return;
    }

    dt = aim_task4_get_dt_s();

    measurement_valid = ((has_new_frame != 0U) && (status != 0)) ? 1U : 0U;

    /* 激光逻辑修复 FIX1：
     *
     * Step10 V1 把 TASK4 改成 15ms 固定周期控制后，控制周期可能比 K230 帧率更快。
     * 如果“没有新帧”的周期也调用 aim_laser_update(..., status=0)，
     * 会把 s_laser_align_cnt 清零，导致永远凑不够 AIM_LASER_STABLE_COUNT，
     * 表现为“已经锁定但不开激光”。
     *
     * 正确做法：
     *   - 有新帧时，才用这一帧的 status/ex/ey 更新开灯判定；
     *   - 没有新帧时，不改变激光对准计数；
     *   - 已经锁存开灯后，GPIO 会保持常亮，不需要每个周期重复写。
     */
    if (has_new_frame != 0U)
    {
        if (status != 0)
        {
            aim_laser_update(ex, ey, 1);
        }
        else
        {
            aim_laser_update(ex, ey, 0);
        }
    }

    if (measurement_valid != 0U)
    {
        s_lost_cnt = 0U;
        s_task4_lost_predict_cnt = 0U;
        s_lost_stopped = 0U;

        /* ref 变化时清 PID 和卡尔曼，跳过当前周期。
         * TASK4 已锁存激光时，AimControl_Reset() 不会关灯。
         */
        if (aim_check_ref_changed(ref_x, ref_y) != 0U)
        {
            return;
        }

#if (AIM_TASK4_KF_ENABLE != 0U)
        ex_f = aim_kf_update(&s_kf_ex,
                             (float)ex,
                             dt,
                             AIM_TASK4_KF_R,
                             AIM_TASK4_KF_Q_ACCEL,
                             AIM_TASK4_KF_LEAD_TIME_S);
        ey_f = aim_kf_update(&s_kf_ey,
                             (float)ey,
                             dt,
                             AIM_TASK4_KF_R,
                             AIM_TASK4_KF_Q_ACCEL,
                             AIM_TASK4_KF_LEAD_TIME_S);
#else
        ex_f = (float)ex;
        ey_f = (float)ey;
#endif
    }
    else
    {
        if (s_task4_lost_predict_cnt < 255U)
        {
            s_task4_lost_predict_cnt++;
        }

        s_lost_cnt = s_task4_lost_predict_cnt;

        if (s_task4_lost_predict_cnt > AIM_TASK4_LOST_PREDICT_FRAMES)
        {
            if (s_lost_stopped == 0U)
            {
                if (s_motor_x_ok != 0U)
                {
                    (void)Stepmotor_Stop(AIM_MOTOR_X);
                }

                if (s_motor_y_ok != 0U)
                {
                    (void)Stepmotor_Stop(AIM_MOTOR_Y);
                }

                s_lost_stopped = 1U;
            }

            return;
        }

#if (AIM_TASK4_KF_ENABLE != 0U)
        ex_f = aim_kf_predict_only(&s_kf_ex,
                                   dt,
                                   AIM_TASK4_KF_Q_ACCEL,
                                   AIM_TASK4_KF_LEAD_TIME_S);
        ey_f = aim_kf_predict_only(&s_kf_ey,
                                   dt,
                                   AIM_TASK4_KF_Q_ACCEL,
                                   AIM_TASK4_KF_LEAD_TIME_S);
#else
        ex_f = 0.0f;
        ey_f = 0.0f;
#endif
    }

    /* TASK4 仍然使用独立 PID 参数，但 dt 改为真实控制周期。 */
    x_angle = PID_CalcDt(&s_pid_x, -ex_f, dt) * AIM_X_DIR_SIGN;
    y_angle = PID_CalcDt(&s_pid_y, -ey_f, dt) * AIM_Y_DIR_SIGN;

    x_angle = aim_limit_abs(x_angle, AIM_TASK4_OUT_MAX_DEG);
    y_angle = aim_limit_abs(y_angle, AIM_TASK4_OUT_MAX_DEG);

#if (AIM_TASK4_OUT_SLEW_ENABLE != 0U)
    x_angle = aim_slew_rate(x_angle, s_last_x_angle, AIM_TASK4_OUT_SLEW_MAX_STEP);
    y_angle = aim_slew_rate(y_angle, s_last_y_angle, AIM_TASK4_OUT_SLEW_MAX_STEP);
#endif

    x_angle = aim_apply_min_step_ex(x_angle, AIM_TASK4_MIN_STEP_DEG);
    y_angle = aim_apply_min_step_ex(y_angle, AIM_TASK4_MIN_STEP_DEG);

    s_last_x_angle = x_angle;
    s_last_y_angle = y_angle;

#if (AIM_DEBUG_ENABLE != 0U) && (AIM_PID_X_DEBUG_ENABLE != 0U)
    {
        static uint8_t s_task4_debug_div_cnt = 0U;
        int32_t y_out_mdeg;

        s_task4_debug_div_cnt++;
        if (s_task4_debug_div_cnt >= AIM_PID_X_DEBUG_DIV)
        {
            s_task4_debug_div_cnt = 0U;
            y_out_mdeg = (int32_t)(y_angle * 1000.0f);

            usart_printf(AIM_DEBUG_UART,
                         "[T4_Y] new=%d st=%d lost=%d dt_ms=%ld ref=%d target=%d out_mdeg=%ld\r\n",
                         (int)has_new_frame,
                         (int)status,
                         (int)s_task4_lost_predict_cnt,
                         (long)(dt * 1000.0f),
                         (int)ref_y,
                         (int)target_y,
                         (long)y_out_mdeg);
        }
    }
#endif

    if ((x_angle != 0.0f) && (s_motor_x_ok != 0U))
    {
        x_cmd = Stepmotor_MoveTo(AIM_MOTOR_X,
                                 x_angle,
                                 AIM_TASK4_RPM,
                                 AIM_ACCEL,
                                 AIM_MOTOR_MODE);

#if (AIM_DEBUG_ENABLE != 0U) && (AIM_MOVE_DEBUG_ENABLE != 0U)
        AIM_OLD_DEBUG_PRINTF(
                     "[T4] MoveX ret=%d angle=%.2f\r\n",
                     (int)x_cmd,
                     (double)x_angle);
#endif
    }

    if ((y_angle != 0.0f) && (s_motor_y_ok != 0U))
    {
        y_cmd = Stepmotor_MoveTo(AIM_MOTOR_Y,
                                 y_angle,
                                 AIM_TASK4_RPM,
                                 AIM_ACCEL,
                                 AIM_MOTOR_MODE);

#if (AIM_DEBUG_ENABLE != 0U) && (AIM_MOVE_DEBUG_ENABLE != 0U)
        AIM_OLD_DEBUG_PRINTF(
                     "[T4] MoveY ret=%d angle=%.2f\r\n",
                     (int)y_cmd,
                     (double)y_angle);
#endif
    }

    aim_debug_print(target_x,
                    target_y,
                    ref_x,
                    ref_y,
                    ex,
                    ey,
                    status,
                    ex_f,
                    ey_f,
                    x_angle,
                    y_angle);

    (void)x_cmd;
    (void)y_cmd;
}
