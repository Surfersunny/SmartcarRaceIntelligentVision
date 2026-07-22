#include "getspeed.h"

// ========== 编码器读取 ==========
void encoder_read_all(int16 encoder[4])
{
    encoder[0] = encoder_get_count(ENCODER_1);
    encoder_clear_count(ENCODER_1);
    encoder[1] = -encoder_get_count(ENCODER_2);
    encoder_clear_count(ENCODER_2);
    encoder[2] = encoder_get_count(ENCODER_3);
    encoder_clear_count(ENCODER_3);
    encoder[3] = -encoder_get_count(ENCODER_4);
    encoder_clear_count(ENCODER_4);
}

// 编码器计数值→实际转速（单位rad/s）
void encoder_to_omega_all(int16 encoder[4], float omega[4])
{
		// 1ms更新一次所以*1000.0f
		for (uint8 i = 0; i < 4; i++) {
			omega[i] = (float)encoder[i] * REDUCTION_RATIO * (2.0f * PI) * 1000.0f / 1024.0;
		}
}