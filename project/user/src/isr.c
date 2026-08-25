/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library 即（RT1064DVL6A 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 RT1064DVL6A 开源库的一部分
* 
* RT1064DVL6A 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          isr
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 8.32.4 or MDK 5.33
* 适用平台          RT1064DVL6A
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/

#include "isr.h"
#include "all.h"

#define MAP_SIZE (ROWS * COLS)

// ========== 识别相关全局变量 ==========
uint8_t map_buffer[MAP_SIZE];
uint16_t map_recv_cnt;
uint8_t map_complete;

uint8_t recog_ready = 0;
uint8_t recog_target_idx = 0;
uint8_t recog_box_idx = 0;
uint8_t recog_phase = 0;
uint8_t recog_total = 0;
uint8_t data;

// ========== 视觉定位数据 ==========
static uint8_t vision_parse_state = 0;  // 0=等待X, 1=解析X, 2=等待Y, 3=解析Y, 4=等待A
uint8_t vision_ready = 0;
float vision_x = 0.0f;
float vision_y = 0.0f;

// ========== 外部函数声明 ==========
extern void bfs_set_target_id(int idx, int id);
extern void bfs_set_box_id(int idx, int id);
extern int bfs_get_target_count(void);
extern int bfs_get_box_count(void);

// ========== 中断处理 ==========

void CSI_IRQHandler(void) {
    CSI_DriverIRQHandler();
    __DSB();
}

void PIT_IRQHandler(void) {
    if(pit_flag_get(PIT_CH0)) {
        void pit_handler (void);
        pit_handler();
        pit_flag_clear(PIT_CH0);
    }
    
    if(pit_flag_get(PIT_CH1)) {
        pit_flag_clear(PIT_CH1);
    }
    
    if(pit_flag_get(PIT_CH2)) {
        pit_flag_clear(PIT_CH2);
    }
    
    if(pit_flag_get(PIT_CH3)) {
        pit_flag_clear(PIT_CH3);
    }

    __DSB();
}

void LPUART1_IRQHandler(void) {
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART1)) {
    #if DEBUG_UART_USE_INTERRUPT
        debug_interrupr_handler();
    #endif
    }
    LPUART_ClearStatusFlags(LPUART1, kLPUART_RxOverrunFlag);
}

void LPUART2_IRQHandler(void) {
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART2)) {}
    LPUART_ClearStatusFlags(LPUART2, kLPUART_RxOverrunFlag);
}

void LPUART3_IRQHandler(void) {
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART3)) {}
    LPUART_ClearStatusFlags(LPUART3, kLPUART_RxOverrunFlag);
}

void LPUART4_IRQHandler(void) {
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART4)) {
        if(NULL != flexio_camera_uart_handler) flexio_camera_uart_handler();
        gnss_uart_callback();
        
        while (uart_query_byte(UART_4, &data)) {
            // ============================================================
            // 1. 地图接收（'S' 响应，192字节）
            // ============================================================
            if (!map_complete && map_recv_cnt < MAP_SIZE && data >= 0 && data <= 5) {
                map_buffer[map_recv_cnt++] = data;
                if (map_recv_cnt >= MAP_SIZE) {
                    map_complete = 1;
                }
                continue;
            }
            
            // ============================================================
            // 2. 视觉定位数据接收（X{x}Y{y}A 格式）
            // ============================================================
            if (data == 'X' && vision_parse_state == 0) {
                // 开始解析视觉数据
                vision_parse_state = 1;
                vision_x = 0.0f;
                vision_y = 0.0f;
                continue;
            }
            
            if (vision_parse_state == 1) {
                // 解析X值
                if (data >= '0' && data <= '9') {
                    vision_x = vision_x * 10 + (data - '0');
                    continue;
                } else if (data == 'Y') {
                    vision_parse_state = 2;
                    continue;
                } else {
                    // 格式错误，重置
                    vision_parse_state = 0;
                    continue;
                }
            }
            
            if (vision_parse_state == 2) {
                // 解析Y值
                if (data >= '0' && data <= '9') {
                    vision_y = vision_y * 10 + (data - '0');
                    continue;
                } else if (data == 'A') {
                    // 视觉数据完成
                    vision_ready = 1;
                    vision_parse_state = 0;
                    continue;
                } else {
                    // 格式错误，重置
                    vision_parse_state = 0;
                    continue;
                }
            }
            
            // ============================================================
            // 3. 识别结果（'t' / 'b' 响应，1字节）
            // ============================================================
            if (recog_phase == 1 && data >= 0 && data <= 9) {
                bfs_set_target_id(recog_target_idx, data);
                recog_target_idx++;
                recog_ready = 1;
                continue;
            } else if (recog_phase == 2 && data >= 0 && data <= 9) {
                bfs_set_box_id(recog_box_idx, data);
                recog_box_idx++;
                recog_ready = 1;
                continue;
            }
        }
    }
    LPUART_ClearStatusFlags(LPUART4, kLPUART_RxOverrunFlag);
} 

void LPUART5_IRQHandler(void) {
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART5)) {
        if(NULL != camera_uart_handler) {
            camera_uart_handler();
        }
    }
    LPUART_ClearStatusFlags(LPUART5, kLPUART_RxOverrunFlag);
}

void LPUART6_IRQHandler(void) {
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART6)) {}
    LPUART_ClearStatusFlags(LPUART6, kLPUART_RxOverrunFlag);
}

void LPUART8_IRQHandler(void) {
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART8)) {
        if(NULL != wireless_module_uart_handler) {
            wireless_module_uart_handler();
        }
    }
    LPUART_ClearStatusFlags(LPUART8, kLPUART_RxOverrunFlag);
}

void GPIO1_Combined_0_15_IRQHandler(void) {
    if(exti_flag_get(B0)) {
        exti_flag_clear(B0);
    }
}

void GPIO1_Combined_16_31_IRQHandler(void) {
    if(exti_flag_get(B16)) {
        exti_flag_clear(B16);
    }
}

void GPIO2_Combined_0_15_IRQHandler(void) {
    if(NULL != flexio_camera_vsync_handler) {
        flexio_camera_vsync_handler();
    }
    
    if(exti_flag_get(C0)) {
        exti_flag_clear(C0);
    }
}

void GPIO2_Combined_16_31_IRQHandler(void) {
    tof_module_exti_handler();
    
    if(exti_flag_get(C16)) {
        exti_flag_clear(C16);
    }
}

void GPIO3_Combined_0_15_IRQHandler(void) {
    if(exti_flag_get(IMU660RC_INT2_PIN)) {
        imu660rc_callback();
        exti_flag_clear(IMU660RC_INT2_PIN);
    }
    if(exti_flag_get(D4)) {
        exti_flag_clear(D4);
    }
}