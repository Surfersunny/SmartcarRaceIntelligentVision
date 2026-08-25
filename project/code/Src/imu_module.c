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
		static float sum = 0;
		static int num = 0;
    const int sample_num = 200;  // 此处确定需要采集多少个值取平均作为零漂
	  const float threshold = 0.25f;// 过大的角速度应被认为是噪声，进行滤除，不加入零漂计算 此处确定噪声阈值
	
		// 如果已经校准完成，直接返回
    if (IMU->imu_calibrated) {
        return;
    }
		
    imu660rb_get_gyro();
	
    float gyro_z_filtered = imu660rb_gyro_transition(imu660rb_gyro_z);
    if (fabsf(gyro_z_filtered) < threshold) {
        sum += gyro_z_filtered;
        num++;
    }
	  
    if (num >= sample_num) {
				IMU->imu_calibrated = 1;
				IMU->gyro_z_offset = sum / sample_num; // 得到准确的静态零漂值
		}
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