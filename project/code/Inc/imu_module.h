#ifndef _IMU_MODULE_H_
#define _IMU_MODULE_H_

#include "zf_common_headfile.h"
#include "math.h"

// ========== IMU 数据结构体 ==========
typedef struct {
    // 角速度（单位：°/s）
    float gyro_z;
	
		// 角速度零漂值（单位：°/s）
    float gyro_z_offset;
	
		// 角速度零漂校准完成标志
		uint8_t imu_calibrated;
    
    // yaw姿态角（单位：°）
    float yaw;
} IMU_TypeDef;

// ========== 函数声明 ==========
void imu_init(IMU_TypeDef* IMU);                    // 初始化 IMU 硬件
void imu_calibrate(IMU_TypeDef* IMU);               // IMU零漂值计算
void imu_update(IMU_TypeDef* IMU);                  // 更新 yaw 角度  在PIT中调用（5ms）
void imu_reset_yaw(IMU_TypeDef* IMU);               // 重置 yaw 角度为 0

// ========== 获取接口函数 ==========
static inline float imu_get_yaw(IMU_TypeDef* IMU) { return IMU->yaw; }

#endif