#include "imu_module.h"

// ========== 初始化 IMU 硬件 ==========
void imu_init(IMU_TypeDef* IMU)
{
    while (imu660rb_init() != 0)
    {
        system_delay_ms(10);
    }

    memset(IMU, 0, sizeof(IMU_TypeDef));	
}

// ========== IMU零漂校准 ==========
void imu_calibrate(IMU_TypeDef* IMU) {
		float sum = 0;
    const int sample_num = 200; 
    
    for(int i = 0; i < sample_num; i++)
    {
        imu660rb_get_gyro();
        sum += imu660rb_gyro_transition(imu660rb_gyro_z);
    }
    
    IMU->gyro_z_offset = sum / sample_num; // 得到准确的静态零漂值
}

// ========== 更新 yaw 角度（每 5ms 调用更新一次）==========
void imu_update(IMU_TypeDef* IMU)
{
    // 1. 读取陀螺仪 Z 轴
    imu660rb_get_gyro();
    
    // 2. 写入角速度，减去零漂：°/s 
    IMU->gyro_z = imu660rb_gyro_transition(imu660rb_gyro_z) - IMU->gyro_z_offset;
    
    // 3. 积分累加 yaw 角
    IMU->yaw += IMU->gyro_z * 0.005f;
    
    // 4. 归一化到 [-180°, 180°]
    if (IMU->yaw > 180.0f) IMU->yaw -= 360.0f;
    if (IMU->yaw < -180.0f) IMU->yaw += 360.0f;
}

// ========== 重置 yaw 角度 ==========
void imu_reset_yaw(IMU_TypeDef* IMU)
{
    IMU->yaw = 0;
}