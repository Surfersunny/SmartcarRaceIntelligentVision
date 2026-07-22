#include "motion_module.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

extern float omega[4];
extern IMU_TypeDef imu;

// ========== PID ==========
#define PID_KP            5.0f
#define PID_KI            0.0f
#define PID_KD            0.1f
#define MAX_DUTY          8000.0f
#define DEADBAND          0.1f

// ========== 平移梯形规划 ==========
#define TRANS_SPEED       600.0f    // 最高速度 mm/s
#define TRANS_ACCEL       1500.0f   // 加速度 mm/s^2（实测可调）
#define POS_DEADBAND      2.0f      // 到位死区 mm

// ========== 蠕行精确定位 ==========
#define CREEP_SPEED       300.0f    // 蠕行最高速度 mm/s
#define CREEP_GAIN        8.0f      // 蠕行 P 增益

// ========== 打滑补偿模型 ==========
#define SLIP_BASE         0.97f     // 基础打滑系数
#define SLIP_K            0.000125f // 速度相关打滑系数
#define SLIP_MIN          0.75f     // 打滑系数下限

// ========== 旋转 ==========
#define ROTATE_SPEED      120.0f    // 旋转速度 度/s
#define ANGLE_DEADBAND    0.5f      // 旋转到位死区 度

// ========== 平移锁头纠偏 ==========
#define YAW_CORRECT_GAIN  2.0f      // 纠偏 P 增益
#define YAW_CORRECT_MAX   30.0f     // 最大纠偏角速度 度/s

const float DT = 0.001f;  // 1ms 控制周期

static float normalize_angle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static void execute_speed(MotionControl_t *Motion, float vx, float vy, float vw)
{
    float target_omega[4];
    float vw_rad = vw * 0.0174532925f;
    mecanum_ik(vx, vy, vw_rad, target_omega);

    if (Motion->type == MOTION_TYPE_ROTATE) {
        for (int i = 0; i < 4; i++) {
            PID_SetTarget(&Motion->pid[i], target_omega[i]);
            float out = PID_Calculate(&Motion->pid[i], omega[i]);
            set_motor(i + 1, out >= 0 ? GPIO_HIGH : GPIO_LOW, (uint32_t)fabsf(out));
        }
    } else {
        for (int i = 0; i < 4; i++) {
            PID_SetTarget(&Motion->pid[i], target_omega[i]);
            float out = PID_Calculate(&Motion->pid[i], omega[i]);
            set_motor(i + 1, out >= 0 ? GPIO_HIGH : GPIO_LOW, (uint32_t)fabsf(out));
        }
    }
}

void motion_init(MotionControl_t *Motion)
{
    memset(Motion, 0, sizeof(MotionControl_t));
    Motion->state = MOTION_STATE_IDLE;
    Motion->type  = MOTION_TYPE_NONE;
    for (int i = 0; i < 4; i++) {
        PID_Init(&Motion->pid[i], PID_KP, PID_KI, PID_KD, DT, 0, MAX_DUTY, DEADBAND, PID_TYPE_INCREMENTAL);
    }
}

void motion_translate(MotionControl_t *Motion, float dist_mm, int8_t direction)
{
    if (Motion->state != MOTION_STATE_IDLE) return;

    Motion->type        = MOTION_TYPE_TRANSLATE;
    Motion->direction   = direction;
    Motion->target_dist = dist_mm;
    Motion->phase       = PHASE_TRAP;

    // 梯形速度规划
    float t_acc = TRANS_SPEED / TRANS_ACCEL;
    float d_acc = 0.5f * TRANS_ACCEL * t_acc * t_acc;

    if (dist_mm >= 2.0f * d_acc) {
        Motion->t1 = t_acc;
        Motion->t2 = t_acc + (dist_mm - 2.0f * d_acc) / TRANS_SPEED;
        Motion->t3 = Motion->t2 + t_acc;
        Motion->peak_speed = TRANS_SPEED;
    } else {
        Motion->t1 = sqrtf(dist_mm / TRANS_ACCEL);
        Motion->t2 = Motion->t1;
        Motion->t3 = Motion->t1 + Motion->t1;
        Motion->peak_speed = TRANS_ACCEL * Motion->t1;
    }

    Motion->t_run = 0.0f;
    Motion->state = MOTION_STATE_SET;
}

void motion_rotate(MotionControl_t *Motion, float angle_deg)
{
    if (Motion->state != MOTION_STATE_IDLE) return;

    Motion->type         = MOTION_TYPE_ROTATE;
    Motion->rotate_dir   = (angle_deg >= 0) ? 1.0f : -1.0f;
    Motion->target_angle = angle_deg;
    Motion->start_yaw    = imu.yaw;
    Motion->state        = MOTION_STATE_SET;
}

void motion_update(MotionControl_t *Motion)
{
    float vx = 0, vy = 0, vw = 0;

    if (Motion->state == MOTION_STATE_IDLE) {
        stop_all_motors();
        for (int i = 0; i < 4; i++) PID_Reset(&Motion->pid[i]);
        return;
    }

    if (Motion->state == MOTION_STATE_SET) {
        Motion->encoder_pos_x = 0;
        Motion->encoder_pos_y = 0;
        Motion->start_yaw = imu.yaw;
        Motion->t_run = 0.0f;
        Motion->phase = PHASE_TRAP;
        Motion->state = MOTION_STATE_RUN;
        return;
    }

    // ========== 编码器里程计（带打滑补偿）==========
    float evx, evy, evw;
    mecanum_fk(omega, &evx, &evy, &evw);

    // 当前瞬时速度（用于打滑模型）
    float current_speed = sqrtf(evx * evx + evy * evy);
    float slip = SLIP_BASE - SLIP_K * current_speed;
    if (slip < SLIP_MIN) slip = SLIP_MIN;

    // 打滑补偿后的真实位置增量
    Motion->encoder_pos_x += 1.0376f * slip * evx * DT;
    Motion->encoder_pos_y += 0.967f * slip * evy * DT;

    // ==================== 平移 ====================
    if (Motion->type == MOTION_TYPE_TRANSLATE) {

        // 当前运动方向上的位置
        float cur = (Motion->direction == UP || Motion->direction == DOWN)
                        ? fabsf(Motion->encoder_pos_x)
                        : fabsf(Motion->encoder_pos_y);

        float error = Motion->target_dist - cur;

        // 到位停止
        if (error <= POS_DEADBAND) {
            Motion->state = MOTION_STATE_IDLE;
            Motion->type  = MOTION_TYPE_NONE;
            stop_all_motors();
            return;
        }

        // ——— 两段式：梯形规划 → 蠕行补偿 ———
        if (Motion->phase == PHASE_TRAP) {
            Motion->t_run += DT;

            if (Motion->t_run >= Motion->t3) {
                // 梯形走完，切蠕行
                Motion->phase = PHASE_CREEP;
            } else {
                // 梯形速度曲线
                float profile_speed;
                if (Motion->t_run < Motion->t1) {
                    profile_speed = TRANS_ACCEL * Motion->t_run;                   // 加速
                } else if (Motion->t_run < Motion->t2) {
                    profile_speed = Motion->peak_speed;                             // 匀速
                } else {
                    profile_speed = Motion->peak_speed
                                  - TRANS_ACCEL * (Motion->t_run - Motion->t2);     // 减速
                    if (profile_speed < 0.0f) profile_speed = 0.0f;
                }

                if (Motion->direction == UP)        vx =  profile_speed;
                else if (Motion->direction == DOWN) vx = -profile_speed;
                else if (Motion->direction == LEFT) vy =  profile_speed;
                else if (Motion->direction == RIGHT)vy = -profile_speed;
            }
        }

        if (Motion->phase == PHASE_CREEP) {
            // 蠕行：P 控制精确定位
            float creep = error * CREEP_GAIN;
            if (creep > CREEP_SPEED)   creep = CREEP_SPEED;

            if (Motion->direction == UP)        vx =  creep;
            else if (Motion->direction == DOWN) vx = -creep;
            else if (Motion->direction == LEFT) vy =  creep;
            else if (Motion->direction == RIGHT)vy = -creep;
        }

        // ===== IMU 锁头纠偏 =====
        float yaw_err = normalize_angle(0.0f - imu.yaw);
        if (fabsf(yaw_err) > 0.5f) {
            vw = -yaw_err * YAW_CORRECT_GAIN;
            if (vw > YAW_CORRECT_MAX)  vw =  YAW_CORRECT_MAX;
            if (vw < -YAW_CORRECT_MAX) vw = -YAW_CORRECT_MAX;
        }

        execute_speed(Motion, vx, vy, vw);
    }

    // ==================== 旋转 ====================
    else if (Motion->type == MOTION_TYPE_ROTATE) {
        float diff = fabsf(normalize_angle(imu.yaw - Motion->start_yaw));

        if (diff >= fabsf(Motion->target_angle) - ANGLE_DEADBAND) {
            Motion->state = MOTION_STATE_IDLE;
            Motion->type  = MOTION_TYPE_NONE;
            stop_all_motors();
            return;
        }

        vw = ROTATE_SPEED * Motion->rotate_dir;
        execute_speed(Motion, 0, 0, vw);
    }
}
