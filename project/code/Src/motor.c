#include "motor.h"

// ========== µç»ú¿ØÖÆ ==========
void set_motor(uint8_t id, uint8_t dir, uint32_t pwm_duty)
{
    switch(id)
    {
        case 1: gpio_set_level(MOTOR1_DIR, dir); pwm_set_duty(MOTOR1_PWM, pwm_duty); break;
        case 2: gpio_set_level(MOTOR2_DIR, dir); pwm_set_duty(MOTOR2_PWM, pwm_duty); break;
        case 3: gpio_set_level(MOTOR3_DIR, dir); pwm_set_duty(MOTOR3_PWM, pwm_duty); break;
        case 4: gpio_set_level(MOTOR4_DIR, dir); pwm_set_duty(MOTOR4_PWM, pwm_duty); break;
    }
}

void stop_all_motors(void)
{
    pwm_set_duty(MOTOR1_PWM, 0);
    pwm_set_duty(MOTOR2_PWM, 0);
    pwm_set_duty(MOTOR3_PWM, 0);
    pwm_set_duty(MOTOR4_PWM, 0);
}