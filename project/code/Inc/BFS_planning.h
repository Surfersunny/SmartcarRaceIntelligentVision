#ifndef _BFS_PLANNING_H_
#define _BFS_PLANNING_H_

#include "zf_common_headfile.h"
#include "motion_module.h"

#define ROWS 12
#define COLS 16

#define CELL_FREE   0
#define CELL_WALL   1
#define CELL_BOX    2
#define CELL_TARGET 3
#define CELL_PLAYER 4
#define CELL_BOMB   5

#define GRID_SIZE_MM 200.0f
#define MAX_MOTION_CMDS 1000
#define MAX_BOX 20
#define MAX_BOMBS 20

// 运动命令结构体
// type: 0=平移, 1=旋转, 2=识别
typedef struct {
    uint8_t type;
    float value;       // 平移:距离(mm), 旋转:角度(°), 识别:phase(1=目标,2=箱子)
    int8_t dir;        // 方向: UP/DOWN/LEFT/RIGHT
    uint8_t idx;       // 识别对象索引（目标或箱子编号）
} BFS_MotionCmd_t;

// 运动队列
typedef struct {
    BFS_MotionCmd_t cmds[MAX_MOTION_CMDS];
    uint16_t count;
    uint8_t push_count;
    uint16_t total_moves;
    uint8_t found;
} BFS_MotionQueue_t;

// ========== 基础接口 ==========
void bfs_init(void);
void bfs_load_map(uint8_t data[ROWS][COLS]);

// ========== BFS寻路 ==========
uint8_t bfs_find_path(int8_t start_x, int8_t start_y, 
                      int8_t target_x, int8_t target_y,
                      int8_t *actions, int *action_len);

// ========== 第1关：直接推箱子 ==========
uint8_t bfs_plan_stage1(BFS_MotionQueue_t *mq);

// ========== 第2/3关：识别阶段（独立） ==========
void bfs_plan_recognize(BFS_MotionQueue_t *mq);

// ========== 第2/3关：推送阶段 ==========
uint8_t bfs_plan_stage23(BFS_MotionQueue_t *mq, uint8_t has_bomb);

// ========== 识别相关函数 ==========
void bfs_set_target_id(int idx, int id);
void bfs_set_box_id(int idx, int id);
int bfs_get_target_id(int idx);
int bfs_get_box_id(int idx);
int bfs_match_box_to_target(int box_idx);
void bfs_occupy_target(int target_idx);

// ========== 获取地图信息 ==========
int bfs_get_box_count(void);
int bfs_get_target_count(void);
int bfs_get_bomb_count(void);
int bfs_get_box_x(int idx);
int bfs_get_box_y(int idx);
int bfs_get_target_x(int idx);
int bfs_get_target_y(int idx);

// ========== 炸弹相关 ==========
uint8_t bfs_is_bomb_active(int8_t x, int8_t y);
uint8_t bfs_trigger_bomb(int8_t x, int8_t y);
void bfs_clear_walls_in_area(int8_t x, int8_t y, int8_t radius);

// ========== 工具函数 ==========
void bfs_add_move_cmd(BFS_MotionQueue_t *mq, int8_t dir);
void bfs_add_rotate_cmd(BFS_MotionQueue_t *mq, float angle);
void bfs_add_recog_cmd(BFS_MotionQueue_t *mq, uint8_t phase, uint8_t idx);
uint8_t bfs_return_to_start(BFS_MotionQueue_t *mq, int8_t cur_px, int8_t cur_py);

// ========== 外部变量声明 ==========
extern int8_t box_x[MAX_BOX];
extern int8_t box_y[MAX_BOX];
extern int8_t target_x[MAX_BOX];
extern int8_t target_y[MAX_BOX];
extern uint8_t origin_px, origin_py;

// 推送阶段起点（识别结束后位置），若外部需要可使用
extern int8_t g_start_px;
extern int8_t g_start_py;

#endif