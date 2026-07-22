#ifndef _MOTION_MODULE_H_
#define _MOTION_MODULE_H_

#include "zf_common_headfile.h"
#include "Mecanum.h"
#include "getspeed.h"
#include "motor.h"
#include "pid.h"
#include "imu_module.h"

#define UP      1
#define DOWN   -1
#define LEFT    2
#define RIGHT  -2

typedef enum {
    MOTION_TYPE_NONE = 0,
    MOTION_TYPE_TRANSLATE,
    MOTION_TYPE_ROTATE
} MotionType_t;

typedef enum {
    MOTION_STATE_IDLE = 0,
    MOTION_STATE_SET,
    MOTION_STATE_RUN
} MotionState_t;

#define PHASE_TRAP  0   // 梯形加减速段
#define PHASE_CREEP 1   // 蠕行补偿段

typedef struct {
    MotionState_t state;
    MotionType_t  type;
    
    int8_t  direction;
    float   rotate_dir;
    float   start_yaw;
    float   target_angle;
    float   target_dist;
    float   encoder_pos_x;
    float   encoder_pos_y;
    
    // 梯形速度规划
    float   t1, t2, t3, t_run, peak_speed;
    uint8_t phase;              // PHASE_TRAP / PHASE_CREEP
    
    PID_Controller pid[4];
} MotionControl_t;

void motion_init(MotionControl_t *Motion);
void motion_translate(MotionControl_t *Motion, float dist_mm, int8_t direction);
void motion_rotate(MotionControl_t *Motion, float angle_deg);
void motion_update(MotionControl_t *Motion);

#endif
