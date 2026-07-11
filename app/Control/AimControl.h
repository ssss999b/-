#ifndef __AIM_CONTROL_H
#define __AIM_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 瞄准控制初始化：
 * 在 main.c 中，完成 USART 初始化、Stepmotor_Init()、LED_Init() 后调用。
 */
void AimControl_Init(void);

/* 瞄准控制更新：
 * main.c 每收到一帧 K230 数据后调用一次。
 *
 * target_x / target_y：K230 识别到的靶心坐标
 * ref_x / ref_y      ：紫色参考点坐标
 * ex / ey            ：误差，ex = target_x - ref_x，ey = target_y - ref_y
 * status             ：1 表示识别成功，0 表示目标丢失
 */
void AimControl_Update(int16_t target_x,
                       int16_t target_y,
                       int16_t ref_x,
                       int16_t ref_y,
                       int16_t ex,
                       int16_t ey,
                       int16_t status);

/* 手动清空 PID、滤波器和内部计数器。
 * 调用后会关闭激光。
 */
void AimControl_Reset(void);

/* 设置是否允许自动开激光。
 *
 * allowed = 0：
 *   强制关闭激光。FIND 模式用这个状态。
 *
 * allowed != 0：
 *   允许 AimControl_Update() 在对准稳定后自动开激光。
 */
void AimControl_SetLaserAllowed(uint8_t allowed);

/* 安全停止：
 *   停止 X/Y 电机，关闭激光，清 PID / 卡尔曼 / 计数器。
 */
void AimControl_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __AIM_CONTROL_H */
