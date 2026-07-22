#ifndef __GET_SPEED_H
#define __GET_SPEED_H

#include "zf_common_headfile.h"

#define REDUCTION_RATIO 0.4 //减速比

#define ENCODER_1           (QTIMER1_ENCODER1)
#define ENCODER_1_LSB       (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_1_DIR       (QTIMER1_ENCODER1_CH2_C1)
#define ENCODER_2           (QTIMER1_ENCODER2)
#define ENCODER_2_LSB       (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_2_DIR       (QTIMER1_ENCODER2_CH2_C24)
#define ENCODER_3           (QTIMER2_ENCODER1)
#define ENCODER_3_LSB       (QTIMER2_ENCODER1_CH1_C3)
#define ENCODER_3_DIR       (QTIMER2_ENCODER1_CH2_C4)
#define ENCODER_4           (QTIMER2_ENCODER2)
#define ENCODER_4_LSB       (QTIMER2_ENCODER2_CH1_C5)
#define ENCODER_4_DIR       (QTIMER2_ENCODER2_CH2_C25)

// 编码器计数值读取（20ms一次）
void encoder_read_all(int16 encoder[4]);
	
// 编码器计数值→实际转速（单位rad/s）
void encoder_to_omega_all(int16 encoder[4], float omega[4]);

#endif