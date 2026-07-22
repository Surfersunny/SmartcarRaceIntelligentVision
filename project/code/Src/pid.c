#include "pid.h"

void PID_Init(PID_Controller *pid,
              float kp, float ki, float kd,
              float dt,
              float integral_limit,
              float output_limit,
              float deadband,
              uint8_t pid_type)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    pid->deadband = deadband;
    pid->pid_type = pid_type;

    pid->target = 0.0f;
    pid->feedback = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

void PID_SetTarget(PID_Controller *pid, float target)
{
    pid->target = target;
}

static float normalize_angle(float angle)
{
    while (angle > 3.1415926f) angle -= 6.2831852f;
    while (angle < -3.1415926f) angle += 6.2831852f;
    return angle;
}

float PID_Calculate(PID_Controller *pid, float feedback)
{
    float error;
    float p_term, i_term, d_term;
    float delta;
    
    pid->feedback = feedback;
    
    // ========== 计算误差 ==========
    if (pid->pid_type == PID_TYPE_ANGLE) {
        // 角度：误差归一化到 [-π, π]
        error = pid->target - pid->feedback;
        error = normalize_angle(error);
    } else {
        // 增量式或位置式：直接计算误差
        error = pid->target - pid->feedback;
    }
    
    // ========== 死区判断 ==========
    if (fabsf(error) <= pid->deadband) {
        // 增量式：死区内不累加，输出不变
        // 位置式：死区内输出0
        if (pid->pid_type == PID_TYPE_INCREMENTAL) {
            return pid->output;
        } else {
            pid->output = 0;
            return 0;
        }
    }
    
    // ========== PID 计算 ==========
    p_term = pid->kp * error;
    
    pid->integral += error * pid->dt;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
		
    i_term = pid->ki * pid->integral;
    
    d_term = pid->kd * (error - pid->last_error) / pid->dt;
    
    // ========== 根据类型计算输出 ==========
    if (pid->pid_type == PID_TYPE_INCREMENTAL) {
        // 增量式
        delta = p_term + i_term + d_term;
        pid->output += delta;
    } else {
        // 位置式 / 角度
        pid->output = p_term + i_term + d_term;
    }
    
    // ========== 输出限幅 ==========
    if (pid->output > pid->output_limit) pid->output = pid->output_limit;
    if (pid->output < -pid->output_limit) pid->output = -pid->output_limit;
    
    pid->last_error = error;
    
    return pid->output;
}

void PID_Reset(PID_Controller *pid)
{
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}