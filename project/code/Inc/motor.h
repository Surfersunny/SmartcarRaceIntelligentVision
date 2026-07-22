#ifndef __MOTOR_H
#define __MOTOR_H

#include "zf_common_headfile.h"
#include "pid.h"

// ========== 电机定义 ==========
#define DIR_FORWARD GPIO_HIGH
#define DIR_REVERSE GPIO_LOW

#define MOTOR1_DIR          (C7)
#define MOTOR1_PWM          (PWM2_MODULE0_CHA_C6)
#define MOTOR2_DIR          (C9)
#define MOTOR2_PWM          (PWM2_MODULE1_CHA_C8)
#define MOTOR3_DIR          (D2)
#define MOTOR3_PWM          (PWM2_MODULE3_CHB_D3)
#define MOTOR4_DIR          (C10)
#define MOTOR4_PWM          (PWM2_MODULE2_CHB_C11)

void set_motor(uint8_t id, uint8_t dir, uint32_t pwm_duty);

void stop_all_motors(void);

#endif