#include "BFS_planning.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

// ========== 外部变量声明 ==========
extern uint8_t recog_ready;
extern uint8_t recog_target_idx;
extern uint8_t recog_box_idx;
extern uint8_t recog_phase;
extern uint8_t recog_total;

// ========== 全局变量定义 ==========
int8_t box_x[MAX_BOX];
int8_t box_y[MAX_BOX];
int8_t target_x[MAX_BOX];
int8_t target_y[MAX_BOX];
int box_count = 0;
int target_count = 0;
uint8_t origin_px = 0;
uint8_t origin_py = 0;
// 推送阶段规划起点：识别阶段结束后就地开始，无需回起点
int8_t g_start_px = 0;
int8_t g_start_py = 0;

// ========== 内部状态 ==========
static uint8_t map_data[ROWS][COLS];
static uint8_t visited[ROWS][COLS];
static int16_t parent[ROWS][COLS];
static int target_ids[MAX_BOX];
static int box_ids[MAX_BOX];
static uint8_t target_matched[MAX_BOX];
static uint8_t box_pushed[MAX_BOX];

// ========== 炸弹数据 ==========
static int8_t bomb_x[MAX_BOMBS];
static int8_t bomb_y[MAX_BOMBS];
static uint8_t bomb_count = 0;
static uint8_t bomb_active[MAX_BOMBS];

// 第2步"直线推炸弹"的最大直线距离（超过此距离该炸弹留给第3步处理）
#define MAX_LINE_PUSH 5

// ========== 推箱子BFS状态（优化版：轻量状态 + 附属数组） ==========
#define MAX_PUSH_STATES 28000

typedef struct {
    int16_t player_pos;   // 玩家位置编码
    int16_t prev_idx;     // 父状态索引
} PushState_t;

PushState_t queue[MAX_PUSH_STATES];
static int16_t box_pos_at_state[MAX_PUSH_STATES];
static uint8_t action_at_state[MAX_PUSH_STATES];
static uint8_t is_push_at_state[MAX_PUSH_STATES];

// ========== visited_state（推箱子用） ==========
static uint8_t visited_state[ROWS][COLS][ROWS][COLS];

// ========== 最优解结构体 ==========
static uint8_t g_best_mq[sizeof(BFS_MotionQueue_t)];

// ========== 方向转坐标偏移 ==========
static void get_dir_delta(int8_t dir, int8_t *dr, int8_t *dc) {
    switch(dir) {
        case UP:    *dr = -1; *dc = 0; break;
        case DOWN:  *dr = 1;  *dc = 0; break;
        case LEFT:  *dr = 0;  *dc = -1; break;
        case RIGHT: *dr = 0;  *dc = 1; break;
        default:    *dr = 0;  *dc = 0; break;
    }
}

static int8_t get_dir_from_delta(int8_t dr, int8_t dc) {
    if (dr == -1 && dc == 0) return UP;
    if (dr == 1 && dc == 0) return DOWN;
    if (dr == 0 && dc == -1) return LEFT;
    if (dr == 0 && dc == 1) return RIGHT;
    return 0;
}

// ========== 编码/解码 ==========
static inline int16_t encode_pos(int8_t r, int8_t c) {
    return (int16_t)(r * COLS + c);
}

static inline void decode_pos(int16_t code, int8_t *r, int8_t *c) {
    *r = code / COLS;
    *c = code % COLS;
}

// ========== 炸弹相关 ==========
uint8_t bfs_is_bomb_active(int8_t x, int8_t y) {
    for (int i = 0; i < bomb_count; i++) {
        if (bomb_x[i] == x && bomb_y[i] == y && bomb_active[i]) {
            return 1;
        }
    }
    return 0;
}

static int get_bomb_index(int8_t x, int8_t y) {
    for (int i = 0; i < bomb_count; i++) {
        if (bomb_x[i] == x && bomb_y[i] == y && bomb_active[i]) {
            return i;
        }
    }
    return -1;
}

void bfs_clear_walls_in_area(int8_t x, int8_t y, int8_t radius) {
    // 只清除墙：炸弹引爆不会连锁引爆其他炸弹（炸弹只会让自身和墙消失）
    for (int8_t dx = -radius; dx <= radius; dx++) {
        for (int8_t dy = -radius; dy <= radius; dy++) {
            int8_t nx = x + dx;
            int8_t ny = y + dy;
            if (nx <= 0 || nx >= ROWS - 1 || ny <= 0 || ny >= COLS - 1) continue;
            if (map_data[nx][ny] == CELL_WALL) {
                map_data[nx][ny] = CELL_FREE;
            }
        }
    }
}

uint8_t bfs_trigger_bomb(int8_t x, int8_t y) {
    int idx = get_bomb_index(x, y);
    if (idx < 0) return 0;
    bomb_active[idx] = 0;
    map_data[x][y] = CELL_FREE;
    bfs_clear_walls_in_area(x, y, 1);
    return 1;
}

// ========== 检查位置是否被阻挡 ==========
static uint8_t is_position_blocked(int8_t x, int8_t y) {
    if (x < 0 || x >= ROWS || y < 0 || y >= COLS) return 1;
    if (map_data[x][y] == CELL_WALL) return 1;
    if (bfs_is_bomb_active(x, y)) return 1;
    for (int i = 0; i < box_count; i++) {
        if (box_x[i] == x && box_y[i] == y && !box_pushed[i]) {
            return 1;
        }
    }
    return 0;
}

// ========== BFS寻路（int8_t路径） ==========
uint8_t bfs_find_path(int8_t start_x, int8_t start_y, 
                      int8_t target_x, int8_t target_y,
                      int8_t *actions, int *action_len) {
    
    if (start_x == target_x && start_y == target_y) {
        *action_len = 0;
        return 1;
    }
    
    memset(visited, 0, sizeof(visited));
    memset(parent, -1, sizeof(parent));
    
    int16_t q[ROWS * COLS];
    int head = 0, tail = 0;
    
    q[tail++] = encode_pos(start_x, start_y);
    visited[start_x][start_y] = 1;
    
    while (head < tail) {
        int16_t cur = q[head++];
        int8_t r, c;
        decode_pos(cur, &r, &c);
        
        if (r == target_x && c == target_y) {
            int path_len = 0;
            int8_t path_dirs[ROWS * COLS];
            int16_t pos = cur;
            int8_t cr = r, cc = c;
            
            while (pos != encode_pos(start_x, start_y)) {
                int16_t parent_pos = parent[cr][cc];
                int8_t pr, pc;
                decode_pos(parent_pos, &pr, &pc);
                
                int8_t dir = get_dir_from_delta(cr - pr, cc - pc);
                if (dir != 0) {
                    path_dirs[path_len++] = dir;
                }
                
                pos = parent_pos;
                decode_pos(pos, &cr, &cc);
            }
            
            for (int i = 0; i < path_len; i++) {
                actions[i] = path_dirs[path_len - 1 - i];
            }
            *action_len = path_len;
            return 1;
        }
        
        for (int d = 0; d < 4; d++) {
            int8_t dr_tbl[4] = {-1, 1, 0, 0};
            int8_t dc_tbl[4] = {0, 0, -1, 1};
            int8_t nr = r + dr_tbl[d];
            int8_t nc = c + dc_tbl[d];
            
            if (nr < 0 || nr >= ROWS || nc < 0 || nc >= COLS) continue;
            if (visited[nr][nc]) continue;
            if (is_position_blocked(nr, nc)) continue;
            
            visited[nr][nc] = 1;
            parent[nr][nc] = cur;
            q[tail++] = encode_pos(nr, nc);
        }
    }
    
    return 0;
}

// ========== 推箱子规划（优化版） ==========
static uint8_t bfs_plan_push(int8_t start_x, int8_t start_y,
                             int8_t box_x_pos, int8_t box_y_pos,
                             int8_t target_x_pos, int8_t target_y_pos,
                             uint8_t any_target,
                             int8_t *arrived_target_idx,
                             int8_t *actions, int *action_len,
                             int8_t *push_dirs, int *push_len,
                             int8_t *end_px, int8_t *end_py) {
    
    memset(visited_state, 0, sizeof(visited_state));
    
    int head = 0, tail = 0;
    
    // 初始状态
    queue[tail].player_pos = encode_pos(start_x, start_y);
    queue[tail].prev_idx = -1;
    box_pos_at_state[tail] = encode_pos(box_x_pos, box_y_pos);
    action_at_state[tail] = 0;
    is_push_at_state[tail] = 0;
    tail++;
    visited_state[start_x][start_y][box_x_pos][box_y_pos] = 1;
    
    int8_t dr_tbl[4] = {-1, 1, 0, 0};
    int8_t dc_tbl[4] = {0, 0, -1, 1};
    int8_t dir_tbl[4] = {UP, DOWN, LEFT, RIGHT};
    
    while (head < tail && tail < MAX_PUSH_STATES) {
        PushState_t *s = &queue[head];
        int8_t px, py;
        decode_pos(s->player_pos, &px, &py);
        
        int16_t box_pos = box_pos_at_state[head];
        int8_t bx, by;
        decode_pos(box_pos, &bx, &by);
        
        // 到达目标：第一关(any_target)任一目标格即成功；二三关严格要求配对目标
        uint8_t arrived = 0;
        if (any_target) {
            for (int t = 0; t < target_count; t++) {
                if (target_matched[t]) continue;   // 目标已被使用，与箱子一起消失
                if (bx == target_x[t] && by == target_y[t]) {
                    arrived = 1;
                    if (arrived_target_idx) *arrived_target_idx = t;
                    break;
                }
            }
        } else {
            arrived = (bx == target_x_pos && by == target_y_pos);
        }
        if (arrived) {
            // ========== 回溯路径 ==========
            int total_actions = 0;
            int push_count = 0;
            int8_t action_stack[ROWS * COLS * 2];
            int8_t push_stack[ROWS * COLS];
            
            int idx = head;
            while (idx != -1) {
                uint8_t act = action_at_state[idx];
                if (act != 0) {
                    action_stack[total_actions++] = act;
                    if (is_push_at_state[idx]) {
                        push_stack[push_count++] = act;
                    }
                }
                idx = queue[idx].prev_idx;
            }
            
            // 反转动作序列
            for (int i = 0; i < total_actions; i++) {
                actions[i] = action_stack[total_actions - 1 - i];
            }
            *action_len = total_actions;
            
            for (int i = 0; i < push_count; i++) {
                push_dirs[i] = push_stack[push_count - 1 - i];
            }
            *push_len = push_count;
            
            // 模拟最终位置
            int8_t sim_px = start_x, sim_py = start_y;
            for (int i = 0; i < total_actions; i++) {
                switch(actions[i]) {
                    case UP:    sim_px--; break;
                    case DOWN:  sim_px++; break;
                    case LEFT:  sim_py--; break;
                    case RIGHT: sim_py++; break;
                }
            }
            *end_px = sim_px;
            *end_py = sim_py;
            
            return 1;
        }
        
        // 四个方向探索
        for (uint8_t d = 0; d < 4; d++) {
            int8_t new_px = px + dr_tbl[d];
            int8_t new_py = py + dc_tbl[d];
            
            if (new_px < 0 || new_px >= ROWS || new_py < 0 || new_py >= COLS) continue;
            
            int8_t new_bx = bx, new_by = by;
            uint8_t is_push = 0;
            
            // 检查是否推箱子
            if (new_px == bx && new_py == by) {
                int8_t push_bx = bx + dr_tbl[d];
                int8_t push_by = by + dc_tbl[d];
                
                if (push_bx < 0 || push_bx >= ROWS || push_by < 0 || push_by >= COLS) continue;
                if (map_data[push_bx][push_by] == CELL_WALL) continue;
                if (bfs_is_bomb_active(push_bx, push_by)) continue;
                
                // 检查是否推到其他箱子（不能连推）
                uint8_t hits_other_box = 0;
                for (int i = 0; i < box_count; i++) {
                    if (!box_pushed[i]) {
                        if (box_x[i] == bx && box_y[i] == by) continue;
                        if (box_x[i] == push_bx && box_y[i] == push_by) {
                            hits_other_box = 1;
                            break;
                        }
                    }
                }
                if (hits_other_box) continue;
                
                new_bx = push_bx;
                new_by = push_by;
                is_push = 1;
            } else {
                if (is_position_blocked(new_px, new_py)) continue;
            }
            
            if (visited_state[new_px][new_py][new_bx][new_by]) continue;
            
            visited_state[new_px][new_py][new_bx][new_by] = 1;
            queue[tail].player_pos = encode_pos(new_px, new_py);
            queue[tail].prev_idx = head;
            box_pos_at_state[tail] = encode_pos(new_bx, new_by);
            action_at_state[tail] = dir_tbl[d];
            is_push_at_state[tail] = is_push;
            tail++;
        }
        head++;
    }
    
    return 0;
}

// ========== 路径压缩：合并连续相同方向 ==========
static void compress_path(BFS_MotionCmd_t *cmds, uint16_t *count) {
    if (*count == 0) return;
    
    uint16_t write_idx = 0;
    BFS_MotionCmd_t last_cmd = cmds[0];
    
    for (uint16_t i = 1; i < *count; i++) {
        if (cmds[i].type == 0 && last_cmd.type == 0 && 
            cmds[i].dir == last_cmd.dir) {
            // 相同方向，累加距离
            last_cmd.value += cmds[i].value;
        } else {
            // 不同方向，写入累加的结果
            cmds[write_idx++] = last_cmd;
            last_cmd = cmds[i];
        }
    }
    // 写入最后一个命令
    cmds[write_idx++] = last_cmd;
    *count = write_idx;
}

// ========== 工具函数 ==========
void bfs_add_move_cmd(BFS_MotionQueue_t *mq, int8_t dir) {
    BFS_MotionCmd_t *cmd = &mq->cmds[mq->count++];
    cmd->type = 0;
    cmd->value = GRID_SIZE_MM;
    cmd->dir = dir;
}

void bfs_add_rotate_cmd(BFS_MotionQueue_t *mq, float angle) {
    if (angle == 0.0f) return;
    BFS_MotionCmd_t *cmd = &mq->cmds[mq->count++];
    cmd->type = 1;
    cmd->value = angle;
    cmd->dir = 0;
}

void bfs_add_recog_cmd(BFS_MotionQueue_t *mq, uint8_t phase, uint8_t idx) {
    BFS_MotionCmd_t *cmd = &mq->cmds[mq->count++];
    cmd->type = 2;
    cmd->value = phase;
    cmd->dir = 0;
    cmd->idx = idx;
}

// ========== 回到起点（此时已推完等效空地图！直接最简单曼哈顿路径） ==========
uint8_t bfs_return_to_start(BFS_MotionQueue_t *mq, int8_t cur_px, int8_t cur_py) {
    if (cur_px == origin_px && cur_py == origin_py) return 1;

    // 先沿行方向移动（x 轴）
    while (cur_px != origin_px) {
        if (cur_px < origin_px) {
            bfs_add_move_cmd(mq, DOWN);   // 行号增加，向下
            cur_px++;
        } else {
            bfs_add_move_cmd(mq, UP);     // 行号减少，向上
            cur_px--;
        }
    }

    // 再沿列方向移动（y 轴）
    while (cur_py != origin_py) {
        if (cur_py < origin_py) {
            bfs_add_move_cmd(mq, RIGHT);  // 列号增加，向右
            cur_py++;
        } else {
            bfs_add_move_cmd(mq, LEFT);   // 列号减少，向左
            cur_py--;
        }
    }

    return 1;
}

// ========== 获取旋转角度 ==========
static float get_rotation_angle(int8_t from_x, int8_t from_y, int8_t to_x, int8_t to_y) {
    int8_t dx = to_x - from_x;
    int8_t dy = to_y - from_y;
    
    if (dx == -1 && dy == 0) return 0.0f;
    else if (dx == 1 && dy == 0) return 180.0f;
    else if (dx == 0 && dy == -1) return 90.0f;
    else if (dx == 0 && dy == 1) return -90.0f;
    return 0.0f;
}

// ========== 查找识别位置 ==========
static uint8_t find_recog_position(int8_t obj_x, int8_t obj_y, int8_t cur_px, int8_t cur_py, int8_t *out_x, int8_t *out_y) {
    int8_t positions[4][2] = {
        {obj_x - 1, obj_y},
        {obj_x + 1, obj_y},
        {obj_x, obj_y - 1},
        {obj_x, obj_y + 1}
    };
    
    uint16_t min_dist = 0xFFFF;
    uint8_t found = 0;
    
    for (int i = 0; i < 4; i++) {
        int8_t px = positions[i][0];
        int8_t py = positions[i][1];
        
        if (px < 0 || px >= ROWS || py < 0 || py >= COLS) continue;
        if (is_position_blocked(px, py)) continue;
        
        int8_t actions[ROWS * COLS];
        int action_len = 0;
        if (bfs_find_path(cur_px, cur_py, px, py, actions, &action_len)) {
            if (action_len < min_dist) {
                min_dist = action_len;
                *out_x = px;
                *out_y = py;
                found = 1;
            }
        }
    }
    
    return found;
}

// ========== 移动到位置识别（只添加命令，不发送UART） ==========
static uint8_t move_to_recog(int8_t obj_x, int8_t obj_y, uint8_t phase, int8_t obj_idx,
                             BFS_MotionQueue_t *mq, int8_t *cur_px, int8_t *cur_py) {
    int8_t recog_x, recog_y;
    if (!find_recog_position(obj_x, obj_y, *cur_px, *cur_py, &recog_x, &recog_y)) {
        return 0;
    }
    
    int8_t actions[ROWS * COLS];
    int action_len = 0;
    if (!bfs_find_path(*cur_px, *cur_py, recog_x, recog_y, actions, &action_len)) {
        return 0;
    }
    
    for (int i = 0; i < action_len; i++) {
        bfs_add_move_cmd(mq, actions[i]);
    }
    
    *cur_px = recog_x;
    *cur_py = recog_y;
    
    float angle = get_rotation_angle(recog_x, recog_y, obj_x, obj_y);
    bfs_add_rotate_cmd(mq, angle);
    
    bfs_add_recog_cmd(mq, phase, obj_idx);
    
    bfs_add_rotate_cmd(mq, -angle);
    
    return 1;
}

// ========== 推箱子可行性预判（方案2） ==========
static uint8_t bfs_push_precheck(int box_idx, int8_t cx, int8_t cy) {
    int8_t dirs[4][2] = {{-1,0},{1,0},{0,-1},{0,1}};
    for (int d = 0; d < 4; d++) {
        int8_t px = box_x[box_idx] - dirs[d][0];   // 推动位（箱子后方）
        int8_t py = box_y[box_idx] - dirs[d][1];
        if (px < 0 || px >= ROWS || py < 0 || py >= COLS) continue;
        if (is_position_blocked(px, py)) continue;
        int8_t tx = box_x[box_idx] + dirs[d][0];   // 推入格（箱子前方）
        int8_t ty = box_y[box_idx] + dirs[d][1];
        if (tx < 0 || tx >= ROWS || ty < 0 || ty >= COLS) continue;
        if (is_position_blocked(tx, ty)) continue;
        int8_t acts[ROWS * COLS];
        int alen = 0;
        if (bfs_find_path(cx, cy, px, py, acts, &alen)) return 1;
    }
    return 0;
}

// ========== 尝试推一组配对序列（第一关专用） ==========
static uint8_t try_push_sequence(BFS_MotionQueue_t *mq,
                                  uint8_t *box_order,
                                  uint8_t *target_order,
                                  uint8_t count,
                                  uint8_t *best_mq,
                                  uint16_t *best_count,
                                  int8_t *best_end_px,
                                  int8_t *best_end_py) {
    
    BFS_MotionQueue_t temp_mq;
    memset(&temp_mq, 0, sizeof(BFS_MotionQueue_t));
    
    // 保存箱子位置/推送状态：每个配对尝试相互独立
    int8_t save_bx[MAX_BOX], save_by[MAX_BOX];
    uint8_t save_pushed[MAX_BOX], save_matched[MAX_BOX];
    memcpy(save_bx, box_x, sizeof(box_x));
    memcpy(save_by, box_y, sizeof(box_y));
    memcpy(save_pushed, box_pushed, sizeof(box_pushed));
    memcpy(save_matched, target_matched, sizeof(target_matched));
    
    int8_t temp_px = origin_px;
    int8_t temp_py = origin_py;
    uint8_t success = 1;
    
    for (int i = 0; i < count; i++) {
        int box_idx = box_order[i];
        int target_idx = target_order[i];
        
        int8_t actions[ROWS * COLS * 2];
        int action_len = 0;
        int8_t push_dirs[ROWS * COLS];
        int push_len = 0;
        int8_t end_px, end_py;
        int8_t arrived_target = -1;
        
        // 预判玩家无法推动该箱子 → 此配对直接失败
        if (!bfs_push_precheck(box_idx, temp_px, temp_py)) {
            success = 0;
            break;
        }
        
        if (bfs_plan_push(temp_px, temp_py,
                          box_x[box_idx], box_y[box_idx],
                          target_x[target_idx], target_y[target_idx],
                          1,                 // any_target: 第一关任一未使用目标即成功
                          &arrived_target,   // 实际到达的目标index
                          actions, &action_len, push_dirs, &push_len,
                          &end_px, &end_py)) {
            for (int j = 0; j < action_len; j++) {
                bfs_add_move_cmd(&temp_mq, actions[j]);
            }
            temp_px = end_px;
            temp_py = end_py;
            // 已推箱子：位置更新到目的地并标记消失
            box_x[box_idx] = target_x[target_idx];
            box_y[box_idx] = target_y[target_idx];
            box_pushed[box_idx] = 1;
            // 目的地一次性使用
            if (arrived_target >= 0) target_matched[arrived_target] = 1;
        } else {
            success = 0;
            break;
        }
    }
    
    uint8_t result = 0;
    if (success && temp_mq.count > 0 && temp_mq.count < *best_count) {
        *best_count = temp_mq.count;
        memcpy(best_mq, &temp_mq, sizeof(BFS_MotionQueue_t));
        *best_end_px = temp_px;
        *best_end_py = temp_py;
        result = 1;
    }
    
    // 恢复箱子位置/推送状态/目标使用状态
    memcpy(box_x, save_bx, sizeof(box_x));
    memcpy(box_y, save_by, sizeof(box_y));
    memcpy(box_pushed, save_pushed, sizeof(box_pushed));
    memcpy(target_matched, save_matched, sizeof(target_matched));
    return result;
}

// ========== 递归尝试所有目标排列 ==========
static uint8_t try_all_permutations(uint8_t *box_order,
                                     uint8_t *target_order,
                                     uint8_t depth,
                                     uint8_t *used,
                                     uint8_t count,
                                     uint8_t *best_mq,
                                     uint16_t *best_count,
                                     int8_t *best_end_px,
                                     int8_t *best_end_py) {
    
    if (depth == count) {
        return try_push_sequence(NULL, box_order, target_order, count,
                                  best_mq, best_count, best_end_px, best_end_py);
    }
    
    for (int i = 0; i < count; i++) {
        if (!used[i]) {
            used[i] = 1;
            target_order[depth] = i;
            // 找到第一个可行配对立即返回
            if (try_all_permutations(box_order, target_order, depth + 1,
                                      used, count, best_mq, best_count,
                                      best_end_px, best_end_py)) {
                used[i] = 0;
                return 1;
            }
            used[i] = 0;
        }
    }
    
    return 0;
}

/* ============================================================================
 * 第1关：直接推箱子（无识别，尝试配对，可行则立即执行）
 * ============================================================================ */
uint8_t bfs_plan_stage1(BFS_MotionQueue_t *mq) {
    if (box_count == 0 || target_count == 0) return 0;
    if (box_count != target_count) return 0;
    
    memset(mq, 0, sizeof(BFS_MotionQueue_t));
    memset(target_matched, 0, sizeof(target_matched));
    memset(box_pushed, 0, sizeof(box_pushed));
    
    uint8_t box_order[MAX_BOX];
    uint8_t target_order[MAX_BOX];
    uint8_t used[MAX_BOX] = {0};
    
    uint16_t best_count = 0xFFFF;
    int8_t best_end_px = origin_px;
    int8_t best_end_py = origin_py;
    
    for (int i = 0; i < box_count; i++) {
        box_order[i] = i;
    }
    
    try_all_permutations(box_order, target_order, 0, used, box_count,
                         g_best_mq, &best_count, &best_end_px, &best_end_py);
    
    if (best_count < 0xFFFF) {
        memcpy(mq, g_best_mq, sizeof(BFS_MotionQueue_t));
        bfs_return_to_start(mq, best_end_px, best_end_py);
        compress_path(mq->cmds, &mq->count);
        mq->found = 1;
        return 1;
    }
    
    return 0;
}

// ========== 炸弹辅助函数 ==========
static uint8_t bfs_can_reach_box_adj(int box_idx, int8_t cx, int8_t cy) {
    uint8_t save[ROWS][COLS];
    memcpy(save, map_data, sizeof(map_data));
    
    for (int i = 0; i < box_count; i++) {
        if (!box_pushed[i]) map_data[box_x[i]][box_y[i]] = CELL_WALL;
    }
    for (int i = 0; i < bomb_count; i++) {
        if (bomb_active[i]) map_data[bomb_x[i]][bomb_y[i]] = CELL_WALL;
    }
    
    uint8_t ok = 0;
    int8_t dirs[4][2] = {{-1,0},{1,0},{0,-1},{0,1}};
    for (int d = 0; d < 4; d++) {
        int8_t nx = box_x[box_idx] + dirs[d][0];
        int8_t ny = box_y[box_idx] + dirs[d][1];
        if (nx < 0 || nx >= ROWS || ny < 0 || ny >= COLS) continue;
        if (map_data[nx][ny] == CELL_WALL) continue;
        
        int8_t acts[ROWS * COLS];
        int alen = 0;
        if (bfs_find_path(cx, cy, nx, ny, acts, &alen)) {
            ok = 1;
            break;
        }
    }
    
    memcpy(map_data, save, sizeof(map_data));
    return ok;
}

// ========== 炸弹选择核心：找"能走到炸弹旁 + 能推到墙 + 炸墙后 check_pos 可达"的炸弹 ==========
// check_pos: 需要可达的候选位置（推动位或对象四邻格）
// 炸弹像箱子一样可被推到任意一面内部墙（不限于自身周围一圈），引爆后清该墙及 radius=1 内墙
static int bfs_find_bomb_for_wall(int8_t check_pos[4][2], int num_check,
                                  int8_t cx, int8_t cy,
                                  int *out_bi, int8_t *out_wx, int8_t *out_wy) {
    int best_cost = 9999;
    int found = 0;
    uint8_t save[ROWS][COLS];
    memcpy(save, map_data, sizeof(map_data));
    
    for (int bi = 0; bi < bomb_count; bi++) {
        if (!bomb_active[bi]) continue;
        
        // 玩家必须先能走到炸弹旁（否则无法推它）
        int8_t adj_x, adj_y;
        if (!find_recog_position(bomb_x[bi], bomb_y[bi], cx, cy, &adj_x, &adj_y)) continue;
        int8_t to_bomb_acts[ROWS * COLS];
        int to_bomb_len = 0;
        if (!bfs_find_path(cx, cy, adj_x, adj_y, to_bomb_acts, &to_bomb_len)) continue;
        
        // 遍历所有内部墙作为引爆点（炸弹可被推到任意墙）
        for (int wx = 1; wx < ROWS - 1; wx++) {
            for (int wy = 1; wy < COLS - 1; wy++) {
                if (save[wx][wy] != CELL_WALL) continue; // 必须是原来的墙
                if (wx == bomb_x[bi] && wy == bomb_y[bi]) continue;
                
                // dry-run 1：验证炸弹能否被推到该墙（临时清墙 + 炸弹不挡路）
                map_data[wx][wy] = CELL_FREE;
                bomb_active[bi] = 0;
                int8_t end_px, end_py;
                int8_t push_acts[ROWS * COLS * 2];
                int push_alen = 0;
                int8_t push_dirs[ROWS * COLS];
                int push_plen = 0;
                int pushable = bfs_plan_push(adj_x, adj_y,
                                             bomb_x[bi], bomb_y[bi],
                                             wx, wy, 0, NULL,
                                             push_acts, &push_alen, push_dirs, &push_plen,
                                             &end_px, &end_py);
                map_data[wx][wy] = CELL_WALL;
                bomb_active[bi] = 1;
                if (!pushable) continue;
                
                // dry-run 2：模拟引爆（炸弹消失 + 清引爆点及 radius=1 内墙）后检查可达
                memcpy(map_data, save, sizeof(map_data));
                map_data[bomb_x[bi]][bomb_y[bi]] = CELL_FREE;
                bomb_active[bi] = 0;
                for (int dx = -1; dx <= 1; dx++) {
                    for (int dy = -1; dy <= 1; dy++) {
                        int8_t nx = wx + dx;
                        int8_t ny = wy + dy;
                        if (nx <= 0 || nx >= ROWS - 1 || ny <= 0 || ny >= COLS - 1) continue;
                        if (map_data[nx][ny] == CELL_WALL) map_data[nx][ny] = CELL_FREE;
                    }
                }
                
                uint8_t reachable = 0;
                for (int c = 0; c < num_check; c++) {
                    int8_t nx = check_pos[c][0];
                    int8_t ny = check_pos[c][1];
                    if (nx < 0 || nx >= ROWS || ny < 0 || ny >= COLS) continue;
                    if (map_data[nx][ny] == CELL_WALL) continue;
                    int8_t acts[ROWS * COLS];
                    int alen = 0;
                    if (bfs_find_path(end_px, end_py, nx, ny, acts, &alen)) {
                        reachable = 1;
                        break;
                    }
                }
                
                // 恢复状态，供下一个候选墙 dry-run 使用
                memcpy(map_data, save, sizeof(map_data));
                bomb_active[bi] = 1;
                
                if (reachable) {
                    int cost = to_bomb_len + push_alen;
                    if (cost < best_cost) {
                        best_cost = cost;
                        *out_bi = bi;
                        *out_wx = wx;
                        *out_wy = wy;
                        found = 1;
                    }
                }
            }
        }
    }
    memcpy(map_data, save, sizeof(map_data));
    return found;
}

// 为箱子 box_idx 找最佳炸弹+墙组合（推送阶段）
// 判据：炸掉某墙后，箱子能被真正推到目标（dry-run 推箱子）
static int bfs_find_bomb_for_box(int box_idx, int8_t cx, int8_t cy,
                                  int8_t target_x, int8_t target_y,
                                  int *out_bi, int8_t *out_wx, int8_t *out_wy) {
    int best_cost = 9999;
    int found = 0;
    uint8_t save[ROWS][COLS];
    memcpy(save, map_data, sizeof(map_data));
    
    for (int bi = 0; bi < bomb_count; bi++) {
        if (!bomb_active[bi]) continue;
        
        // 玩家必须先能走到炸弹旁（否则无法推它）
        int8_t adj_x, adj_y;
        if (!find_recog_position(bomb_x[bi], bomb_y[bi], cx, cy, &adj_x, &adj_y)) continue;
        int8_t to_bomb_acts[ROWS * COLS];
        int to_bomb_len = 0;
        if (!bfs_find_path(cx, cy, adj_x, adj_y, to_bomb_acts, &to_bomb_len)) continue;
        
        // 遍历所有内部墙作为引爆点（炸弹可被推到任意墙）
        for (int wx = 1; wx < ROWS - 1; wx++) {
            for (int wy = 1; wy < COLS - 1; wy++) {
                if (save[wx][wy] != CELL_WALL) continue; // 必须是原来的墙
                if (wx == bomb_x[bi] && wy == bomb_y[bi]) continue;
                
                // dry-run 1：验证炸弹能否被推到该墙
                map_data[wx][wy] = CELL_FREE;
                bomb_active[bi] = 0;
                int8_t end_px, end_py;
                int8_t push_acts[ROWS * COLS * 2];
                int push_alen = 0;
                int8_t push_dirs[ROWS * COLS];
                int push_plen = 0;
                int pushable = bfs_plan_push(adj_x, adj_y,
                                             bomb_x[bi], bomb_y[bi],
                                             wx, wy, 0, NULL,
                                             push_acts, &push_alen, push_dirs, &push_plen,
                                             &end_px, &end_py);
                map_data[wx][wy] = CELL_WALL;
                bomb_active[bi] = 1;
                if (!pushable) continue;
                
                // dry-run 2：模拟引爆（炸弹消失 + 清引爆点及 radius=1 内墙），箱子能否推到目标
                memcpy(map_data, save, sizeof(map_data));
                map_data[bomb_x[bi]][bomb_y[bi]] = CELL_FREE;
                bomb_active[bi] = 0;
                for (int dx = -1; dx <= 1; dx++) {
                    for (int dy = -1; dy <= 1; dy++) {
                        int8_t nx = wx + dx;
                        int8_t ny = wy + dy;
                        if (nx <= 0 || nx >= ROWS - 1 || ny <= 0 || ny >= COLS - 1) continue;
                        if (map_data[nx][ny] == CELL_WALL) map_data[nx][ny] = CELL_FREE;
                    }
                }
                
                int8_t box_end_px, box_end_py;
                int8_t box_acts[ROWS * COLS * 2];
                int box_alen = 0;
                int8_t box_push_dirs[ROWS * COLS];
                int box_push_len = 0;
                int box_ok = bfs_plan_push(end_px, end_py,
                                           box_x[box_idx], box_y[box_idx],
                                           target_x, target_y,
                                           0, NULL,
                                           box_acts, &box_alen, box_push_dirs, &box_push_len,
                                           &box_end_px, &box_end_py);
                
                // 恢复状态，供下一个候选墙 dry-run 使用
                memcpy(map_data, save, sizeof(map_data));
                bomb_active[bi] = 1;
                
                if (box_ok) {
                    int cost = to_bomb_len + push_alen;
                    if (cost < best_cost) {
                        best_cost = cost;
                        *out_bi = bi;
                        *out_wx = wx;
                        *out_wy = wy;
                        found = 1;
                    }
                }
            }
        }
    }
    memcpy(map_data, save, sizeof(map_data));
    return found;
}

// ========== 为识别阶段找可引爆的炸弹开路 ==========
// 识别对象四邻格被炸弹/墙挡住时，找"能走到炸弹旁 + 能推到墙 + 炸墙后四邻格可达"的炸弹
static int bfs_find_bomb_for_recog(int8_t obj_x, int8_t obj_y, int8_t cx, int8_t cy,
                                   int *out_bi, int8_t *out_wx, int8_t *out_wy) {
    int8_t check_pos[4][2] = {
        {obj_x - 1, obj_y},
        {obj_x + 1, obj_y},
        {obj_x, obj_y - 1},
        {obj_x, obj_y + 1}
    };
    return bfs_find_bomb_for_wall(check_pos, 4, cx, cy, out_bi, out_wx, out_wy);
}

static uint8_t bfs_execute_bomb(int bomb_idx, int8_t wx, int8_t wy,
                                 BFS_MotionQueue_t *mq,
                                 int8_t *cur_px, int8_t *cur_py) {
    // 1. 走到炸弹旁边的空地（不需要视觉识别）
    int8_t adj_x, adj_y;
    if (!find_recog_position(bomb_x[bomb_idx], bomb_y[bomb_idx], *cur_px, *cur_py, &adj_x, &adj_y)) {
        return 0;
    }
    int8_t acts[ROWS * COLS];
    int alen = 0;
    if (!bfs_find_path(*cur_px, *cur_py, adj_x, adj_y, acts, &alen)) {
        return 0;
    }
    for (int i = 0; i < alen; i++) {
        bfs_add_move_cmd(mq, acts[i]);
    }
    *cur_px = adj_x;
    *cur_py = adj_y;
    
    // 2. 用 bfs_plan_push 把炸弹推到墙的位置
    //    临时把墙位清空，让 bomb 能推过去
    uint8_t save_wall = map_data[wx][wy];
    map_data[wx][wy] = CELL_FREE;
    uint8_t save_bomb_active = bomb_active[bomb_idx];
    bomb_active[bomb_idx] = 0;  // 炸弹不再阻挡路径
    
    int8_t end_px, end_py;
    int8_t push_acts[ROWS * COLS * 2];
    int push_alen = 0;
    int8_t push_dirs[ROWS * COLS];
    int push_plen = 0;
    
    // 炸弹作为"箱子"，推至墙位置作为"目标"
    int ok = bfs_plan_push(*cur_px, *cur_py,
                           bomb_x[bomb_idx], bomb_y[bomb_idx],
                           wx, wy,
                           0,
                           NULL,
                           push_acts, &push_alen, push_dirs, &push_plen,
                           &end_px, &end_py);
    
    map_data[wx][wy] = save_wall;
    bomb_active[bomb_idx] = save_bomb_active;
    
    if (!ok) return 0;
    
    for (int i = 0; i < push_alen; i++) {
        bfs_add_move_cmd(mq, push_acts[i]);
    }
    *cur_px = end_px;
    *cur_py = end_py;
    
    // 3. 引爆：炸弹已被推到墙 (wx,wy)，更新坐标并引爆
    map_data[bomb_x[bomb_idx]][bomb_y[bomb_idx]] = CELL_FREE;
    bomb_x[bomb_idx] = wx;
    bomb_y[bomb_idx] = wy;
    bfs_clear_walls_in_area(wx, wy, 1);
    bomb_active[bomb_idx] = 0;
    
    return 1;
}

// ========== 第2步：直线推炸弹到最近墙（帮助度优先） ==========
// ========== 第2步：直线推炸弹（帮助度优先，距离次要） ==========
// 枚举炸弹 bi 沿上/下/左/右直线扫到的最近墙（路径无障碍、距离<=MAX_LINE_PUSH）。
// 直线可推要求：炸弹行进方向反方向的推动位必须可达（界内且非墙/箱子/炸弹）。
// 填入 walls[][2] 与对应直线距离 dists[]，返回候选墙个数。
static int bfs_line_walls(int bi, int8_t walls[4][2], int dists[4]) {
    int8_t dirs[4][2] = {{-1,0},{1,0},{0,-1},{0,1}};
    int n = 0;
    for (int d = 0; d < 4; d++) {
        int8_t dx = dirs[d][0], dy = dirs[d][1];
        // 推动位 = 炸弹后方一格（玩家站立推的位置）
        int8_t px = bomb_x[bi] - dx, py = bomb_y[bi] - dy;
        if (px < 0 || px >= ROWS || py < 0 || py >= COLS) continue;
        if (is_position_blocked(px, py)) continue;   // 推动位不可达
        int8_t nx = bomb_x[bi] + dx, ny = bomb_y[bi] + dy;
        int dist = 1;
        while (nx > 0 && nx < ROWS - 1 && ny > 0 && ny < COLS - 1) {
            if (map_data[nx][ny] == CELL_WALL) {
                walls[n][0] = nx; walls[n][1] = ny; dists[n] = dist;
                n++;
                break;
            }
            if (is_position_blocked(nx, ny)) break;   // 路径上有箱子/炸弹挡
            if (dist >= MAX_LINE_PUSH) break;
            nx += dx; ny += dy; dist++;
        }
    }
    return n;
}

// 统计当前地图下，能推到自己匹配目标（box id==target id）的箱子数。
// 对每个箱子，从它的四邻格（推动位）中任一空地出发 dry-run 推箱子到目标，
// 只要存在一个推动位能推到目标，就认为该箱子"通道已打通"。
static int bfs_pushable_box_count(void) {
    int cnt = 0;
    int8_t dirs4[4][2] = {{-1,0},{1,0},{0,-1},{0,1}};
    for (int b = 0; b < box_count; b++) {
        if (box_pushed[b]) continue;
        int t = bfs_match_box_to_target(b);
        if (t < 0) continue;
        int ok = 0;
        for (int d = 0; d < 4 && !ok; d++) {
            int8_t px = box_x[b] + dirs4[d][0];
            int8_t py = box_y[b] + dirs4[d][1];
            if (px < 0 || px >= ROWS || py < 0 || py >= COLS) continue;
            if (is_position_blocked(px, py)) continue;
            int8_t acts[ROWS * COLS * 2]; int alen = 0;
            int8_t pdirs[ROWS * COLS]; int plen = 0;
            int8_t epx, epy;
            if (bfs_plan_push(px, py, box_x[b], box_y[b],
                              target_x[t], target_y[t], 0, NULL,
                              acts, &alen, pdirs, &plen, &epx, &epy)) ok = 1;
        }
        if (ok) cnt++;
    }
    return cnt;
}

// 当前有多少激活炸弹"可直线推"（至少能扫到一面可达墙）
static int bfs_pushable_bomb_count(void) {
    int cnt = 0;
    for (int bi = 0; bi < bomb_count; bi++) {
        if (!bomb_active[bi]) continue;
        int8_t walls[4][2]; int dists[4];
        if (bfs_line_walls(bi, walls, dists) > 0) cnt++;
    }
    return cnt;
}

// 引爆炸弹 bi（推到墙 sx,sy）后，"可推箱子数 + 可直线推炸弹数"的总效益。
// 模拟：炸弹 bi 消失（不再挡路/不再参与统计），并清 (sx,sy) radius=1 内墙。
static int bfs_line_benefit(int bi, int sx, int sy) {
    uint8_t save_map[ROWS][COLS];
    memcpy(save_map, map_data, sizeof(map_data));
    uint8_t save_active = bomb_active[bi];
    // 炸弹引爆后消失
    bomb_active[bi] = 0;
    map_data[bomb_x[bi]][bomb_y[bi]] = CELL_FREE;
    for (int dx = -1; dx <= 1; dx++)
        for (int dy = -1; dy <= 1; dy++) {
            int8_t nx = sx + dx, ny = sy + dy;
            if (nx <= 0 || nx >= ROWS - 1 || ny <= 0 || ny >= COLS - 1) continue;
            if (map_data[nx][ny] == CELL_WALL) map_data[nx][ny] = CELL_FREE;
        }
    int v = bfs_pushable_box_count() + bfs_pushable_bomb_count();
    memcpy(map_data, save_map, sizeof(map_data));
    bomb_active[bi] = save_active;
    return v;
}

// 第2步：贪心引爆所有"可直线推(距离<=MAX_LINE_PUSH)"的炸弹。
// 选墙：优先"引爆后(可推箱子数+可直线推炸弹数)最大"，距离作次要；允许引爆协同前置(step 帮助度>=0)。
// 反复直到没有可直线推的炸弹或无可引爆。
static uint8_t bfs_explode_line_bombs(BFS_MotionQueue_t *mq, int8_t *cur_px, int8_t *cur_py) {
    uint8_t exploded_any = 0;
    for (int iter = 0; iter < MAX_BOMBS * 2; iter++) {  // 最多引爆全部炸弹
        int best_bi = -1, best_wx = 0, best_wy = 0;
        int best_benefit = 0, best_dist = MAX_LINE_PUSH + 1;
        uint8_t save_map[ROWS][COLS];
        memcpy(save_map, map_data, sizeof(map_data));
        for (int bi = 0; bi < bomb_count; bi++) {
            if (!bomb_active[bi]) continue;
            int8_t walls[4][2]; int dists[4];
            int nw = bfs_line_walls(bi, walls, dists);
            for (int k = 0; k < nw; k++) {
                int wx = walls[k][0], wy = walls[k][1];
                int benefit = bfs_line_benefit(bi, wx, wy);
                if (benefit > best_benefit || (benefit == best_benefit && dists[k] < best_dist)) {
                    best_benefit = benefit; best_dist = dists[k];
                    best_bi = bi; best_wx = wx; best_wy = wy;
                }
            }
        }
        if (best_bi < 0) break;  // 没有可直线推的炸弹，结束第2步
        if (!bfs_execute_bomb(best_bi, best_wx, best_wy, mq, cur_px, cur_py)) {
            // 该炸弹实际推不动（如推动位被墙），标记失效避免死循环
            bomb_active[best_bi] = 0;
            continue;
        }
        exploded_any = 1;
    }
    return exploded_any;
}

// ========== 识别（被挡时用炸弹开路重试） ==========
// 先直接尝试走到识别位识别；若被炸弹/墙挡住，逐个引爆可用炸弹开路后重试
static uint8_t move_to_recog_with_bomb(int8_t obj_x, int8_t obj_y, uint8_t phase, int8_t obj_idx,
                                       BFS_MotionQueue_t *mq, int8_t *cur_px, int8_t *cur_py) {
    // 1. 先直接尝试识别
    if (move_to_recog(obj_x, obj_y, phase, obj_idx, mq, cur_px, cur_py)) {
        return 1;
    }
    
    // 2. 被挡路：逐个引爆可用炸弹开路，再重试识别
    while (1) {
        int bi;
        int8_t wx, wy;
        if (!bfs_find_bomb_for_recog(obj_x, obj_y, *cur_px, *cur_py, &bi, &wx, &wy)) {
            return 0; // 没有可用炸弹
        }
        if (!bfs_execute_bomb(bi, wx, wy, mq, cur_px, cur_py)) {
            bomb_active[bi] = 0; // 该炸弹无法执行，标记失效，避免死循环
            continue;
        }
        if (move_to_recog(obj_x, obj_y, phase, obj_idx, mq, cur_px, cur_py)) {
            return 1;
        }
        // 炸完仍被挡，继续找下一个炸弹
    }
}

/* ============================================================================
 * 识别阶段（第2/3关专用，在推送规划之前单独执行）
 * 真正的 2n-1 优化：识别所有目标 + 识别前 n-1 个箱子
 * ============================================================================ */
void bfs_plan_recognize(BFS_MotionQueue_t *mq) {
    memset(mq, 0, sizeof(BFS_MotionQueue_t));
    
    int8_t cur_px = origin_px;
    int8_t cur_py = origin_py;
    
    // 识别所有目标 (n 个)
    for (int i = 0; i < target_count; i++) {
        move_to_recog_with_bomb(target_x[i], target_y[i], 1, i, mq, &cur_px, &cur_py);
    }
    
    // 只识别前 n-1 个箱子（最后一个箱子不需要识别）
    for (int i = 0; i < box_count - 1; i++) {
        move_to_recog_with_bomb(box_x[i], box_y[i], 2, i, mq, &cur_px, &cur_py);
    }
    
    // 识别结束位置作为推送阶段起点
    g_start_px = cur_px;
    g_start_py = cur_py;
    
    compress_path(mq->cmds, &mq->count);
    mq->found = 1;
}

/* ============================================================================
 * 视觉关推送规划（第2关/第3关）
 * 利用识别阶段得到的 IDs 进行匹配，对于最后一个未识别箱子使用排除法
 * ============================================================================ */
uint8_t bfs_plan_stage23(BFS_MotionQueue_t *mq, uint8_t has_bomb) {
    if (box_count == 0 || target_count == 0) return 0;
    if (box_count != target_count) return 0;
    
    memset(mq, 0, sizeof(BFS_MotionQueue_t));
    memset(target_matched, 0, sizeof(target_matched));
    memset(box_pushed, 0, sizeof(box_pushed));
    
    int8_t cur_px = g_start_px;
    int8_t cur_py = g_start_py;
    
    // 第2步：推送开始前，先贪心引爆所有"可直线推(距离<=MAX_LINE_PUSH)"的炸弹
    // （这些炸弹沿直线推到最近的墙引爆，清掉沿途障碍；用不完的炸弹留给第3步）
    if (has_bomb) {
        bfs_explode_line_bombs(mq, &cur_px, &cur_py);
    }
    
    // 先推送前 n-1 个箱子（索引 0 到 box_count-2）
    for (int i = 0; i < box_count - 1; i++) {
        int box_idx = i;
        
        // 根据识别到的 ID 匹配目标
        int target_idx = bfs_match_box_to_target(box_idx);
        if (target_idx < 0) {
            // 如果没有直接匹配（理论上不会），选择最近的未占用目标
            uint16_t min_dist = 0xFFFF;
            for (int t = 0; t < target_count; t++) {
                if (target_matched[t]) continue;
                uint16_t dist = abs(box_x[box_idx] - target_x[t]) + 
                                abs(box_y[box_idx] - target_y[t]);
                if (dist < min_dist) {
                    min_dist = dist;
                    target_idx = t;
                }
            }
        }
        if (target_idx < 0) continue;  // 理论上不会发生
        
        // 尝试推箱子
        int8_t end_px, end_py;
        int8_t actions[ROWS * COLS * 2];
        int action_len = 0;
        int8_t push_dirs[ROWS * COLS];
        int push_len = 0;
        
        if (bfs_plan_push(cur_px, cur_py,
                          box_x[box_idx], box_y[box_idx],
                          target_x[target_idx], target_y[target_idx],
                          0,
                          NULL,
                          actions, &action_len, push_dirs, &push_len,
                          &end_px, &end_py)) {
            for (int j = 0; j < action_len; j++) {
                bfs_add_move_cmd(mq, actions[j]);
            }
            cur_px = end_px;
            cur_py = end_py;
            box_pushed[box_idx] = 1;
            target_matched[target_idx] = 1;
        } else if (has_bomb) {
            // 推箱子失败，尝试使用炸弹破墙
            uint8_t bomb_worked = 0;
            while (!bomb_worked) {
                int bi;
                int8_t wx, wy;
                if (!bfs_find_bomb_for_box(box_idx, cur_px, cur_py,
                                           target_x[target_idx], target_y[target_idx],
                                           &bi, &wx, &wy)) {
                    break;  // 没有更多可用炸弹
                }
                if (!bfs_execute_bomb(bi, wx, wy, mq, &cur_px, &cur_py)) {
                    bomb_active[bi] = 0;  // 该炸弹无法执行，标记失效，避免死循环
                    continue;  // 炸弹执行失败，尝试下一个
                }
                
                if (bfs_plan_push(cur_px, cur_py,
                                  box_x[box_idx], box_y[box_idx],
                                  target_x[target_idx], target_y[target_idx],
                                  0,
                                  NULL,
                                  actions, &action_len, push_dirs, &push_len,
                                  &end_px, &end_py)) {
                    for (int j = 0; j < action_len; j++) {
                        bfs_add_move_cmd(mq, actions[j]);
                    }
                    cur_px = end_px;
                    cur_py = end_py;
                    box_pushed[box_idx] = 1;
                    target_matched[target_idx] = 1;
                    bomb_worked = 1;
                }
                // 炸弹用了但箱子仍推不动 → while 循环继续尝试下一个炸弹
            }
            if (!bomb_worked) {
                // 所有炸弹用完仍失败，标记箱子为已推送（放弃）
                box_pushed[box_idx] = 1;
            }
        } else {
            // 无炸弹可用，标记失败
            box_pushed[box_idx] = 1;
        }
    }
    
    // 处理最后一个箱子：它的目标就是剩余未匹配的那个
    int last_box_idx = box_count - 1;
    int last_target = -1;
    for (int t = 0; t < target_count; t++) {
        if (!target_matched[t]) {
            last_target = t;
            break;
        }
    }
    
    if (last_target >= 0) {
        int8_t end_px, end_py;
        int8_t actions[ROWS * COLS * 2];
        int action_len = 0;
        int8_t push_dirs[ROWS * COLS];
        int push_len = 0;
        
        // 尝试推最后一个箱子到剩余目标
        if (bfs_plan_push(cur_px, cur_py,
                          box_x[last_box_idx], box_y[last_box_idx],
                          target_x[last_target], target_y[last_target],
                          0,
                          NULL,
                          actions, &action_len, push_dirs, &push_len,
                          &end_px, &end_py)) {
            for (int j = 0; j < action_len; j++) {
                bfs_add_move_cmd(mq, actions[j]);
            }
            cur_px = end_px;
            cur_py = end_py;
            box_pushed[last_box_idx] = 1;
            target_matched[last_target] = 1;
        } else if (has_bomb) {
            // 最后一个箱子也尝试使用炸弹
            uint8_t bomb_worked = 0;
            while (!bomb_worked) {
                int bi;
                int8_t wx, wy;
                if (!bfs_find_bomb_for_box(last_box_idx, cur_px, cur_py,
                                           target_x[last_target], target_y[last_target],
                                           &bi, &wx, &wy)) {
                    break;
                }
                if (!bfs_execute_bomb(bi, wx, wy, mq, &cur_px, &cur_py)) {
                    bomb_active[bi] = 0;  // 该炸弹无法执行，标记失效，避免死循环
                    continue;
                }
                
                if (bfs_plan_push(cur_px, cur_py,
                                  box_x[last_box_idx], box_y[last_box_idx],
                                  target_x[last_target], target_y[last_target],
                                  0,
                                  NULL,
                                  actions, &action_len, push_dirs, &push_len,
                                  &end_px, &end_py)) {
                    for (int j = 0; j < action_len; j++) {
                        bfs_add_move_cmd(mq, actions[j]);
                    }
                    cur_px = end_px;
                    cur_py = end_py;
                    box_pushed[last_box_idx] = 1;
                    target_matched[last_target] = 1;
                    bomb_worked = 1;
                }
            }
        }
    }
    
    // 检查是否所有目标都已完成
    uint8_t all_matched = 1;
    for (int t = 0; t < target_count; t++) {
        if (!target_matched[t]) {
            all_matched = 0;
            break;
        }
    }
    
    if (all_matched && mq->count > 0) {
        bfs_return_to_start(mq, cur_px, cur_py);
        compress_path(mq->cmds, &mq->count);
        mq->found = 1;
        return 1;
    }
    
    return 0;
}

// ========== 识别相关函数 ==========
void bfs_set_target_id(int idx, int id) {
    if (idx < MAX_BOX) {
        target_ids[idx] = id;
        if (idx >= target_count) target_count = idx + 1;
    }
}

void bfs_set_box_id(int idx, int id) {
    if (idx < MAX_BOX) {
        box_ids[idx] = id;
    }
}

int bfs_get_target_id(int idx) {
    if (idx < MAX_BOX) return target_ids[idx];
    return -1;
}

int bfs_get_box_id(int idx) {
    if (idx < MAX_BOX) return box_ids[idx];
    return -1;
}

int bfs_match_box_to_target(int box_idx) {
    if (box_idx >= box_count) return -1;
    for (int i = 0; i < target_count; i++) {
        if (!target_matched[i] && box_ids[box_idx] == target_ids[i]) {
            return i;
        }
    }
    return -1;
}

void bfs_occupy_target(int target_idx) {
    if (target_idx < target_count) {
        target_matched[target_idx] = 1;
    }
}

// ========== 获取地图信息 ==========
int bfs_get_box_count(void) { return box_count; }
int bfs_get_target_count(void) { return target_count; }
int bfs_get_bomb_count(void) { return bomb_count; }
int bfs_get_box_x(int idx) { return (idx < box_count) ? box_x[idx] : -1; }
int bfs_get_box_y(int idx) { return (idx < box_count) ? box_y[idx] : -1; }
int bfs_get_target_x(int idx) { return (idx < target_count) ? target_x[idx] : -1; }
int bfs_get_target_y(int idx) { return (idx < target_count) ? target_y[idx] : -1; }

// ========== 初始化 ==========
void bfs_init(void) {
    memset(map_data, 0, sizeof(map_data));
    memset(box_x, 0, sizeof(box_x));
    memset(box_y, 0, sizeof(box_y));
    memset(target_x, 0, sizeof(target_x));
    memset(target_y, 0, sizeof(target_y));
    memset(target_ids, 0, sizeof(target_ids));
    memset(box_ids, 0, sizeof(box_ids));
    memset(target_matched, 0, sizeof(target_matched));
    memset(box_pushed, 0, sizeof(box_pushed));
    memset(bomb_x, 0, sizeof(bomb_x));
    memset(bomb_y, 0, sizeof(bomb_y));
    memset(bomb_active, 0, sizeof(bomb_active));
    box_count = 0;
    target_count = 0;
    bomb_count = 0;
    origin_px = 0;
    origin_py = 0;
    g_start_px = 0;
    g_start_py = 0;
    memset(g_best_mq, 0, sizeof(g_best_mq));
}

void bfs_load_map(uint8_t data[ROWS][COLS]) {
    memcpy(map_data, data, ROWS * COLS);
    box_count = 0;
    target_count = 0;
    bomb_count = 0;
    memset(target_matched, 0, sizeof(target_matched));
    memset(box_pushed, 0, sizeof(box_pushed));
    memset(target_ids, 0, sizeof(target_ids));
    memset(box_ids, 0, sizeof(box_ids));
    memset(bomb_active, 0, sizeof(bomb_active));
    
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            if (i == 6 && j == 2) {
                origin_px = i;
                origin_py = j; // 硬编码实际起点为（6，2） 因为出发车区才会刷新出地图 （6，2）确保离开了发车区
            } else if (data[i][j] == CELL_BOX && box_count < MAX_BOX) {
                box_x[box_count] = i;
                box_y[box_count] = j;
                box_count++;
            } else if (data[i][j] == CELL_TARGET && target_count < MAX_BOX) {
                target_x[target_count] = i;
                target_y[target_count] = j;
                target_count++;
            } else if (data[i][j] == CELL_BOMB && bomb_count < MAX_BOMBS) {
                bomb_x[bomb_count] = i;
                bomb_y[bomb_count] = j;
                bomb_active[bomb_count] = 1;
                bomb_count++;
            }
        }
    }
    
    g_start_px = origin_px;
    g_start_py = origin_py;
}		