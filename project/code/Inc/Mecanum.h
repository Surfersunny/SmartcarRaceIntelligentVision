#ifndef __MECANUM_H
#define __MECANUM_H

#include "zf_common_headfile.h"

// ========== 机器人参数 ==========
#define WHEEL_RADIUS_MM      (35.0f)   // 轮子半径 (mm)
#define ROBOT_WIDTH_MM       (175.0f)  // 车宽 (mm)
#define ROBOT_LENGTH_MM      (200.0f)  // 车长 (mm)

// 逆运动学：给定目标速度 (vx, vy, vw)，计算四个轮子转速 (rad/s)
void mecanum_ik(float vx, float vy, float vw, float wheel_speeds[4]);

// 正运动学：给定四个轮子转速 (rad/s)，计算车身速度 (vx, vy, vw)
void mecanum_fk(float wheel_speeds[4], float *vx, float *vy, float *vw);

#endif