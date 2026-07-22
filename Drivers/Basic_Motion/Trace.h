#ifndef TRACE_H
#define TRACE_H

#include "gray_with_MCU.h"
#include "Motor.h"
#include "clock.h"

/**
 * @brief 用陀螺仪闭环控制小车旋转到目标 Yaw 角
 *
 * PD 控制: 用左右轮差速实现原地旋转，误差越大差速越大。
 *
 *               error > 0 (yaw_target 在 CCW 方向):
 *               → right_speed > left_speed → 小车逆时针转 → yaw ↑
 *               error < 0 (yaw_target 在 CW 方向):
 *               → left_speed > right_speed → 小车顺时针转 → yaw ↓
 *
 * @param dir         预留的旋转方向（当前未使用，方向由 error 符号决定）
 * @param speed       基础旋转速度 (0 ~ 0.6)
 * @param yaw_target  目标 Yaw 角 (度), 自动归一化到 [-180, 180]
 */
void Rotate(Track_Clock dir, float speed, float yaw_target);

void Patrol_Trace(Track_Clock spin_dir,float left_speed, float right_speed, float speed_add, float yaw_current);

//检查是否碰到矩形角或出线
bool meet_Rect(void);

//对矩形框循迹
void Rect_PID_trace(Track_Clock dir, float speed, float speed_add);
//使用占空比循迹
void Rect_DUTY_trace(Track_Clock dir, float left_duty, float right_duty, float duty_add, float yaw_current);

#endif