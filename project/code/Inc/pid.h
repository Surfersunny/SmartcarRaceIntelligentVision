#ifndef __PID_H
#define __PID_H

#include "zf_common_headfile.h"
#include "math.h"

// PID类型
#define PID_TYPE_INCREMENTAL    0   // 增量式
#define PID_TYPE_POSITIONAL     1   // 位置式（非增量）
#define PID_TYPE_ANGLE          2   // 角度过零处理

// PID结构体
typedef struct {
    // parameters
    float kp;
    float ki;
    float kd;
    float dt;               // 离散PID计算间隔 (s)
    float integral_limit;   // 积分饱和
    float output_limit;     // 输出限幅
    float deadband;         // 死区（error < deadband 则不再计算）
    uint8_t pid_type;       // PID类型: 0=增量式, 1=位置式, 2=角度过零处理

    // target & feedback
    float target;
    float feedback;

    float last_error;
    float integral;

    // output
    float output;
} PID_Controller;

// 初始化
void PID_Init(PID_Controller *pid,
              float kp, float ki, float kd,
              float dt,
              float integral_limit,
              float output_limit,
              float deadband,
              uint8_t pid_type);      

// 设定目标值
void PID_SetTarget(PID_Controller *pid, float target);

// 设定参数（用于中途改变）
static void PID_SetKp(PID_Controller *pid, float kp) { pid->kp = kp; }
static void PID_SetKi(PID_Controller *pid, float ki) { pid->ki = ki; }
static void PID_SetKd(PID_Controller *pid, float kd) { pid->kd = kd; }
static void PID_SetType(PID_Controller *pid, uint8_t type) { pid->pid_type = type; }

// *核心计算函数*
float PID_Calculate(PID_Controller *pid, float feedback); 

// 重置
void PID_Reset(PID_Controller *pid);

// 获取接口函数
static inline float PID_GetOutput(PID_Controller *pid)    { return pid->output; }
static float PID_GetFeedback(PID_Controller *pid)  { return pid->feedback; }
static float PID_GetTarget(PID_Controller *pid)    { return pid->target; }

#endif