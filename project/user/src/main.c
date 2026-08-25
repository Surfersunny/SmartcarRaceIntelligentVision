#include "all.h"

// ========== 宏定义 ==========
#define PWM_FREQ (17000)
#define OPENART_UART UART_4
#define OPENART_BAUD 115200
#define OPENART_TX_PIN UART4_TX_C16
#define OPENART_RX_PIN UART4_RX_C17
#define MAP_SIZE (ROWS * COLS)

#define STAGE_1 1
#define STAGE_2 2
#define STAGE_3 3
#define TOTAL_STAGES 3

// ========== 外部变量声明 ==========
extern uint8_t map_buffer[MAP_SIZE];
extern uint16_t map_recv_cnt;
extern uint8_t map_complete;

extern uint8_t recog_ready;
extern uint8_t recog_target_idx;
extern uint8_t recog_box_idx;
extern uint8_t recog_phase;
extern uint8_t recog_total;
extern uint8_t data;

extern uint8_t vision_ready;
extern float vision_x;
extern float vision_y;

// ========== 本地全局变量 ==========
uint8_t current_stage = STAGE_1;
uint8_t all_done = 0;
uint8_t success = 0;
uint8_t is_initialized = 0;
uint8_t correction_x = 0;
uint8_t correction_y = 0;

volatile uint32_t sys_cnt = 0;
int16 encoder_data[4] = {0};
float omega[4] = {0};

MotionControl_t motion = {0};
IMU_TypeDef imu = {0};

BFS_MotionQueue_t motion_queue;
uint16_t cmd_index = 0;
uint8_t sequence_running = 0;
uint8_t planning = 0;
uint8_t recognize_done = 0;     // 识别阶段是否完成
uint8_t map_loaded = 0;         // 地图是否已载入
uint8_t awaiting_push_plan = 0; // 识别完成等待推送规划

float realtime_px = 120.0f; 
float realtime_py = 10.0f; // 实时理想位置 最开始在（120，10） 单位：cm

// ========== 串口发送 ==========
void send_cmd(uint8_t cmd) {
    uart_write_byte(OPENART_UART, cmd);
}

// ========== 请求地图 ==========
void request_map(void) {
    map_recv_cnt = 0;
    map_complete = 0;
    recognize_done = 0;
    map_loaded = 0;
    awaiting_push_plan = 0;
		//system_delay_ms(1000);
    send_cmd('S');
}

// ========== 执行运动命令 ==========
/*void execute_motion_cmd(BFS_MotionCmd_t *cmd) {
    if (cmd->type == 0) {
        motion_translate(&motion, cmd->value, cmd->dir);
    } else if (cmd->type == 1) {
        motion_rotate(&motion, cmd->value);
    } else if (cmd->type == 2) {
        uint8_t phase = (uint8_t)cmd->value;
        
        recog_ready = 0;
        recog_phase = phase;
        
        if (phase == 1) {
            recog_target_idx = cmd->idx;
            recog_total = bfs_get_target_count();
						system_delay_ms(2000);
            send_cmd('t');
        } else {
            recog_box_idx = cmd->idx;
            recog_total = bfs_get_box_count();
						system_delay_ms(2000);
            send_cmd('b');
        }
        
        while (!recog_ready) {
            system_delay_ms(1);
        }
        recog_ready = 0;
    }
}*/

void execute_motion_cmd(BFS_MotionCmd_t *cmd) {
    if (cmd->type == 0) {  // 平移命令
        motion_translate(&motion, cmd->value, cmd->dir);
				
				// 更新理想位置
				float distance = cmd->value / 10.0f;  // mm -> cm
        switch (cmd->dir) {
            case UP:    realtime_px -= distance; break;
            case DOWN:  realtime_px += distance; break;
            case LEFT:  realtime_py -= distance; break;
            case RIGHT: realtime_py += distance; break;
				}
				
				// 平移完成后进入两步视觉纠偏 此处可注释，从而纯测试路径规划正确与否
				correction_x = 1; 
				correction_y = 1;
    }
    else if (cmd->type == 1) {  // 旋转命令
        motion_rotate(&motion, cmd->value);
    }
    else if (cmd->type == 2) {  // 识别命令
        uint8_t phase = (uint8_t)cmd->value;
        recog_ready = 0;
        recog_phase = phase;

        if (phase == 1) {
            recog_target_idx = cmd->idx;
            recog_total = bfs_get_target_count();
            //system_delay_ms(1000);
            send_cmd('t');
        } else {
            recog_box_idx = cmd->idx;
            recog_total = bfs_get_box_count();
            //system_delay_ms(1000);
            send_cmd('b');
        }

        while (!recog_ready) {
            system_delay_ms(1);
        }
        recog_ready = 0;
    }
}

// ========== 规划并启动 ==========
void plan_and_start(void) {
    if (!map_complete || planning || sequence_running || all_done) return;
    
    planning = 1;
    
    // 第一次规划才载入地图，避免二次载入清掉识别IDs
    if (!map_loaded) {
        uint8_t map_2d[ROWS][COLS];
        for (int i = 0; i < ROWS; i++) {
            for (int j = 0; j < COLS; j++) {
                map_2d[i][j] = map_buffer[i * COLS + j];
            }
        }
        bfs_load_map(map_2d);
        map_loaded = 1;
    }
    
    memset(&motion_queue, 0, sizeof(BFS_MotionQueue_t));
    
    if (current_stage == STAGE_1) {
        success = bfs_plan_stage1(&motion_queue);
    } else if (!recognize_done) {
        // 第2/3关：先识别
        bfs_plan_recognize(&motion_queue);
        recognize_done = 1;
        awaiting_push_plan = 1;
        success = 1;
    } else {
        // 第2/3关：推送规划
        awaiting_push_plan = 0;
        uint8_t has_bomb = (bfs_get_bomb_count() > 0);
        success = bfs_plan_stage23(&motion_queue, has_bomb);
    }
    
    if (success || motion_queue.count > 0) {
        cmd_index = 0;
        sequence_running = 1;
    }
    
    planning = 0;
}

// ========== 硬件初始化 ==========
void hardware_init(void) {
    gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR1_PWM, PWM_FREQ, 0);
    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR2_PWM, PWM_FREQ, 0);
    gpio_init(MOTOR3_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR3_PWM, PWM_FREQ, 0);
    gpio_init(MOTOR4_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR4_PWM, PWM_FREQ, 0);
    
    encoder_dir_init(ENCODER_1, ENCODER_1_LSB, ENCODER_1_DIR);
    encoder_dir_init(ENCODER_2, ENCODER_2_LSB, ENCODER_2_DIR);
    encoder_dir_init(ENCODER_3, ENCODER_3_LSB, ENCODER_3_DIR);
    encoder_dir_init(ENCODER_4, ENCODER_4_LSB, ENCODER_4_DIR);
    
    imu_init(&imu);
    
    uart_init(OPENART_UART, OPENART_BAUD, OPENART_TX_PIN, OPENART_RX_PIN);
    uart_rx_interrupt(OPENART_UART, 1);
    
    pit_ms_init(PIT_CH0, 1);
    interrupt_global_enable(0);
}

// ========== PIT中断处理 ==========
void pit_handler(void) {
    static uint64_t imu_cnt = 0;
    imu_cnt++;
    
		if (!imu.imu_calibrated) {
				if (imu_cnt % 5 == 0)	imu_calibrate(&imu);
		}
		
    else {
        encoder_read_all(encoder_data);
        encoder_to_omega_all(encoder_data, omega);
        motion_update(&motion);
        sys_cnt++;
        
        if (sys_cnt >= 5) {
            imu_update(&imu);
            sys_cnt = 0;
        }
    }
}

// ========== 离开发车区（不做纠偏） ==========
void leave_start_zone(void) {
    memset(&motion_queue, 0, sizeof(BFS_MotionQueue_t));
    
    BFS_MotionCmd_t cmd1;
    cmd1.type = 0;
    cmd1.value = 100.0f;
    cmd1.dir = DOWN;
    motion_queue.cmds[motion_queue.count++] = cmd1;
    
    BFS_MotionCmd_t cmd2;
    cmd2.type = 0;
    cmd2.value = 400.0f;
    cmd2.dir = RIGHT;
    motion_queue.cmds[motion_queue.count++] = cmd2;
    
    cmd_index = 0;
    sequence_running = 1;
    is_initialized = 1;
    
    while (sequence_running) {
        if (motion.state == MOTION_STATE_IDLE) {
            if (cmd_index < motion_queue.count) {
							  if (!correction_x && !correction_y) {
									  execute_motion_cmd(&motion_queue.cmds[cmd_index]);
                    cmd_index++;
								}									
								else if (correction_x && correction_y) {
										vision_ready = 0;
										send_cmd('r');
									
										while (!vision_ready) ;
									
										if (vision_ready && fabsf(vision_x) > 0.1 && fabsf(vision_y) > 0.1) {
												float err_x = vision_x - realtime_px;

												float distx = fabsf(err_x) * 10.0f;
												if (err_x > 0) motion_translate(&motion, 0.0f, UP);
												else           motion_translate(&motion, 0.0f, DOWN);
											
												correction_x = 0;
										}
								}
								else if (!correction_x && correction_y) {
									  if (vision_ready && fabsf(vision_x) > 0.1 && fabsf(vision_y) > 0.1) {
												float err_y = vision_y - realtime_py;

												float disty = fabsf(err_y) * 10.0f;
												if (err_y > 0) motion_translate(&motion, 0.0f, LEFT);
												else           motion_translate(&motion, 0.0f, RIGHT);
											
												correction_y = 0;
										}
								}
						}
						else {
                sequence_running = 0;
            }
			}
	}
}

// ========== 回到发车区（不做纠偏） ==========
void return_to_start_zone(void) {
    memset(&motion_queue, 0, sizeof(BFS_MotionQueue_t));
    
    BFS_MotionCmd_t cmd1;
    cmd1.type = 0;
    cmd1.value = 400.0f;
    cmd1.dir = LEFT;
    motion_queue.cmds[motion_queue.count++] = cmd1;
    
    BFS_MotionCmd_t cmd2;
    cmd2.type = 0;
    cmd2.value = 100.0f;
    cmd2.dir = UP;
    motion_queue.cmds[motion_queue.count++] = cmd2;
    
    cmd_index = 0;
    sequence_running = 1;
    
     while (sequence_running) {
        if (motion.state == MOTION_STATE_IDLE) {
            if (cmd_index < motion_queue.count) {
							  if (!correction_x && !correction_y) {
									  execute_motion_cmd(&motion_queue.cmds[cmd_index]);
                    cmd_index++;
								}									
								else if (correction_x && correction_y) {
										vision_ready = 0;
										send_cmd('r');
									
										while (!vision_ready) ;
									
										if (vision_ready && fabsf(vision_x) > 0.1 && fabsf(vision_y) > 0.1) {
												float err_x = vision_x - realtime_px;

												float distx = fabsf(err_x) * 10.0f;
												if (err_x > 0) motion_translate(&motion, 0.0f, UP);
												else           motion_translate(&motion, 0.0f, DOWN);
											
												correction_x = 0;
										}
								}
								else if (!correction_x && correction_y) {
									  if (vision_ready && fabsf(vision_x) > 0.1 && fabsf(vision_y) > 0.1) {
												float err_y = vision_y - realtime_py;

												float disty = fabsf(err_y) * 10.0f;
												if (err_y > 0) motion_translate(&motion, 0.0f, LEFT);
												else           motion_translate(&motion, 0.0f, RIGHT);
											
												correction_y = 0;
										}
								}
						}
						else {
                sequence_running = 0;
            }
			}
	}
}

// ========== 软件盲盒：最后旋转360°==========
void rotate_360(void) {
    memset(&motion_queue, 0, sizeof(BFS_MotionQueue_t));
    
    BFS_MotionCmd_t cmd1;
    cmd1.type = 1;
    cmd1.value = 180.0f;
    motion_queue.cmds[motion_queue.count++] = cmd1;
    
    BFS_MotionCmd_t cmd2;
    cmd2.type = 1;
    cmd2.value = 180.0f;
    motion_queue.cmds[motion_queue.count++] = cmd2;
    
    cmd_index = 0;
    sequence_running = 1;
    
    while (sequence_running) {
        if (motion.state == MOTION_STATE_IDLE) {
            if (cmd_index < motion_queue.count) {
                execute_motion_cmd(&motion_queue.cmds[cmd_index]);
                cmd_index++;
            } else {
                sequence_running = 0;
            }
        }
    }
}

// ========== 主函数 ==========
int main(void) {
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
    system_delay_ms(2000);
    
    hardware_init();
    motion_init(&motion);
    bfs_init();
    
    recog_ready = 0;
    recog_target_idx = 0;
    recog_box_idx = 0;
    recog_phase = 0;
    recog_total = 0;
    all_done = 0;
    is_initialized = 0;
    
    leave_start_zone();
    request_map();
    
    while (1) {
        if (all_done) {
            system_delay_ms(100);
            continue;
        }
        
        if (map_complete && !planning && !sequence_running) {
            plan_and_start();
        }
        
        if (sequence_running) {
            if (motion.state == MOTION_STATE_IDLE) {
                if (cmd_index < motion_queue.count) {
                    if (!correction_x && !correction_y) {
												execute_motion_cmd(&motion_queue.cmds[cmd_index]);
												cmd_index++;
										}									
										else if (correction_x && correction_y) {
												vision_ready = 0;
												send_cmd('r');
											
												while (!vision_ready) ;
											
												if (vision_ready && fabsf(vision_x) > 0.1 && fabsf(vision_y) > 0.1) {
														float err_x = vision_x - realtime_px;

														float distx = fabsf(err_x) * 10.0f;
														if (err_x > 0) motion_translate(&motion, distx, UP);
														else           motion_translate(&motion, distx, DOWN);
													
														correction_x = 0;
												}
										}
										else if (!correction_x && correction_y) {
												if (vision_ready && fabsf(vision_x) > 0.1 && fabsf(vision_y) > 0.1) {
														float err_y = vision_y - realtime_py;

														float disty = fabsf(err_y) * 10.0f;
														if (err_y > 0) motion_translate(&motion, disty, LEFT);
														else           motion_translate(&motion, disty, RIGHT);
													
														correction_y = 0;
												}
										}
                } else {
                    sequence_running = 0;
                    
                    if (awaiting_push_plan) {
                        // 识别阶段结束，等主循环重新规划推送
                    } else if (current_stage < TOTAL_STAGES) {
                        return_to_start_zone();
                        //system_delay_ms(3000);
                        leave_start_zone();
                        current_stage++;
                        request_map();
                    } else {
                        return_to_start_zone();
												rotate_360(); // 软件盲盒：最后旋转360°
                        all_done = 1;
                        stop_all_motors();
                    }
                }
            }
        }
    }
}

/*#include "all.h"
#include "bfs_planning.h"

// ========== 宏定义 ==========
#define PWM_FREQ (17000)
#define OPENART_UART UART_4
#define OPENART_BAUD 115200
#define OPENART_TX_PIN UART4_TX_C16
#define OPENART_RX_PIN UART4_RX_C17
#define MAP_SIZE (ROWS * COLS)

// ========== 外部变量声明 ==========
extern uint8_t map_buffer[MAP_SIZE];
extern uint16_t map_recv_cnt;
extern uint8_t map_complete;

extern uint8_t recog_ready;
extern uint8_t recog_target_idx;
extern uint8_t recog_box_idx;
extern uint8_t recog_phase;
extern uint8_t recog_total;
extern uint8_t data;

// ========== 本地全局变量 ==========
volatile uint32_t sys_cnt = 0;
int16 encoder_data[4] = {0};
float omega[4] = {0};

MotionControl_t motion = {0};
IMU_TypeDef imu = {0};

uint8_t is_initialized = 0;

// ========== 硬件初始化 ==========
void hardware_init(void) {
    gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR1_PWM, PWM_FREQ, 0);
    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR2_PWM, PWM_FREQ, 0);
    gpio_init(MOTOR3_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR3_PWM, PWM_FREQ, 0);
    gpio_init(MOTOR4_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR4_PWM, PWM_FREQ, 0);
    
    encoder_dir_init(ENCODER_1, ENCODER_1_LSB, ENCODER_1_DIR);
    encoder_dir_init(ENCODER_2, ENCODER_2_LSB, ENCODER_2_DIR);
    encoder_dir_init(ENCODER_3, ENCODER_3_LSB, ENCODER_3_DIR);
    encoder_dir_init(ENCODER_4, ENCODER_4_LSB, ENCODER_4_DIR);
    
    imu_init(&imu);
    
    uart_init(OPENART_UART, OPENART_BAUD, OPENART_TX_PIN, OPENART_RX_PIN);
    uart_rx_interrupt(OPENART_UART, 1);
    
    pit_ms_init(PIT_CH0, 1);
    interrupt_global_enable(0);
}

// ========== PIT中断处理 ==========
void pit_handler(void) {
    static uint64_t imu_cnt = 0;
    imu_cnt++;
    
		if (!imu.imu_calibrated) {
				if (imu_cnt % 5 == 0)	imu_calibrate(&imu);
		}
		
    else {
        encoder_read_all(encoder_data);
        encoder_to_omega_all(encoder_data, omega);
        motion_update(&motion);
        sys_cnt++;
        
        if (sys_cnt >= 5) {
            imu_update(&imu);
            sys_cnt = 0;
        }
    }
}

// ========== 等待运动完成 ==========
void wait_motion_idle(void) {
    while (motion.state != MOTION_STATE_IDLE) {
        system_delay_ms(5);
    }
}

// ========== 测试1：单步移动测试 ==========
void test_single_moves(void) {
    // 前进200mm
    motion_translate(&motion, 20.0f, UP);
    wait_motion_idle();
    system_delay_ms(1000);
    
    // 后退200mm
    motion_translate(&motion, 800.0f, DOWN);
    wait_motion_idle();
    system_delay_ms(1000);
    
    // 左移200mm
    motion_translate(&motion, 800.0f, LEFT);
    wait_motion_idle();
    system_delay_ms(1000);
    
    // 右移200mm
    motion_translate(&motion, 800.0f, RIGHT);
    wait_motion_idle();
    system_delay_ms(1000);
}

// ========== 测试2：正方形路径（回到原点验证） ==========
void test_square_path(void) {
    float side_length = 400.0f;  // 正方形边长400mm
    
    // 走正方形
    for (int i = 0; i < 4; i++) {
        // 前进
        motion_translate(&motion, side_length, UP);
        wait_motion_idle();
        system_delay_ms(500);
        
        // 右转90度
        motion_rotate(&motion, -90.0f);
        wait_motion_idle();
        system_delay_ms(500);
    }
    
    // 最后转回初始朝向
    motion_rotate(&motion, -90.0f);
    wait_motion_idle();
    system_delay_ms(1000);
}

// ========== 测试3：距离精度测试 ==========
void test_distance_accuracy(void) {
    // 测试不同距离
    float distances[] = {100.0f, 200.0f, 300.0f, 400.0f, 500.0f};
    
    for (int i = 0; i < 5; i++) {
        // 前进
        motion_translate(&motion, distances[i], UP);
        wait_motion_idle();
        system_delay_ms(1000);
        
        // 后退（回到起点）
        motion_translate(&motion, distances[i], DOWN);
        wait_motion_idle();
        system_delay_ms(1000);
    }
}

// ========== 测试4：角度精度测试 ==========
void test_angle_accuracy(void) {
    float angles[] = {30.0f, 45.0f, 60.0f, 90.0f, 180.0f};
    
    for (int i = 0; i < 5; i++) {
        // 左转
        motion_rotate(&motion, angles[i]);
        wait_motion_idle();
        system_delay_ms(1000);
        
        // 右转（回到初始朝向）
        motion_rotate(&motion, -angles[i]);
        wait_motion_idle();
        system_delay_ms(1000);
    }
}

// ========== 测试5：连续运动测试 ==========
void test_continuous_motion(void) {
    // 连续前进3格（600mm）
    for (int i = 0; i < 3; i++) {
        motion_translate(&motion, 200.0f, UP);
        wait_motion_idle();
        system_delay_ms(200);
    }
    system_delay_ms(1000);
    
    // 连续后退3格
    for (int i = 0; i < 3; i++) {
        motion_translate(&motion, 200.0f, DOWN);
        wait_motion_idle();
        system_delay_ms(200);
    }
    system_delay_ms(1000);
}

// ========== 主函数 ==========
int main(void) {
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
    system_delay_ms(2000);
    
    hardware_init();
    motion_init(&motion);
    bfs_init();
    
    is_initialized = 0;
    
    system_delay_ms(2000);  // 等待系统稳定
    
    // ===== 测试模式选择 =====
    // 取消注释要运行的测试
    
    // 测试1：单步移动测试
    test_single_moves();
    
    // 测试2：正方形路径（回到原点验证累计误差）
    // test_square_path();
    
    // 测试3：距离精度测试
    // test_distance_accuracy();
    
    // 测试4：角度精度测试
    // test_angle_accuracy();
    
    // 测试5：连续运动测试
    // test_continuous_motion();
    
    // 所有测试完成，停止
    stop_all_motors();
    
    while (1) {
        system_delay_ms(100);
    }
}*/