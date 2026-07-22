#include "Mecanum.h"

// ========== 逆运动学 ==========
void mecanum_ik(float vx, float vy, float vw, float wheel_speeds[4])
{
    // w1 = (vx + vy + vw * L) / r
    // w2 = (vx - vy - vw * L) / r
    // w3 = (-vx - vy + vw * L) / r
    // w4 = (-vx + vy - vw * L) / r
    
    float L = (ROBOT_LENGTH_MM + ROBOT_WIDTH_MM) / 2.0f;
    float r = WHEEL_RADIUS_MM;
    
    wheel_speeds[0] = ( vx + vy + vw * L) / r;
    wheel_speeds[1] = ( vx - vy - vw * L) / r;
    wheel_speeds[2] = (-vx - vy + vw * L) / r;
    wheel_speeds[3] = (-vx + vy - vw * L) / r;
}

// ========== 正运动学 ==========
void mecanum_fk(float wheel_speeds[4], float *vx, float *vy, float *vw)
{
    // 从四个轮速反解车身速度
    // vx = (w1 + w2 - w3 - w4) * r / 4
    // vy = (w1 - w2 - w3 + w4) * r / 4
    // vw = (w1 - w2 + w3 - w4) * r / (4 * L)
    
    float r = WHEEL_RADIUS_MM;
    float L = (ROBOT_LENGTH_MM + ROBOT_WIDTH_MM) / 2.0f;
    
    *vx = ( wheel_speeds[0] + wheel_speeds[1] - wheel_speeds[2] - wheel_speeds[3] ) * r / 4.0f;
    *vy = ( wheel_speeds[0] - wheel_speeds[1] - wheel_speeds[2] + wheel_speeds[3] ) * r / 4.0f;
    *vw = ( wheel_speeds[0] - wheel_speeds[1] + wheel_speeds[2] - wheel_speeds[3] ) * r / (4.0f * L);
}
