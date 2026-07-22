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
    for (int8_t dx = -radius; dx <= radius; dx++) {
        for (int8_t dy = -radius; dy <= radius; dy++) {
            int8_t nx = x + dx;
            int8_t ny = y + dy;
            if (nx <= 0 || nx >= ROWS - 1 || ny <= 0 || ny >= COLS - 1) continue;
            // 跳过最外围一圈墙，防止车冲出边界
            if (map_data[nx][ny] == CELL_WALL) {
                map_data[nx][ny] = CELL_FREE;
            }
            if (bfs_is_bomb_active(nx, ny)) {
                if (!(nx == x && ny == y)) {
                    bfs_trigger_bomb(nx, ny);
                }
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
        
        // 到达目标
        if (bx == target_x_pos && by == target_y_pos) {
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

void bfs_add_recog_cmd(BFS_MotionQueue_t *mq, uint8_t phase) {
    BFS_MotionCmd_t *cmd = &mq->cmds[mq->count++];
    cmd->type = 2;
    cmd->value = phase;
    cmd->dir = 0;
}

// ========== 回到起点 ==========
uint8_t bfs_return_to_start(BFS_MotionQueue_t *mq, int8_t cur_px, int8_t cur_py) {
    if (cur_px == origin_px && cur_py == origin_py) return 1;
    
    int8_t actions[ROWS * COLS];
    int action_len = 0;
    
    if (bfs_find_path(cur_px, cur_py, origin_px, origin_py, actions, &action_len)) {
        for (int i = 0; i < action_len; i++) {
            bfs_add_move_cmd(mq, actions[i]);
        }
        return 1;
    }
    
    return 0;
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
static uint8_t move_to_recog(int8_t obj_x, int8_t obj_y, uint8_t phase, 
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
    
    bfs_add_recog_cmd(mq, phase);
    
    bfs_add_rotate_cmd(mq, -angle);
    
    return 1;
}

// ========== 尝试推一组配对序列 ==========
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
        
        if (bfs_plan_push(temp_px, temp_py,
                          box_x[box_idx], box_y[box_idx],
                          target_x[target_idx], target_y[target_idx],
                          actions, &action_len, push_dirs, &push_len,
                          &end_px, &end_py)) {
            for (int j = 0; j < action_len; j++) {
                bfs_add_move_cmd(&temp_mq, actions[j]);
            }
            temp_px = end_px;
            temp_py = end_py;
        } else {
            success = 0;
            break;
        }
    }
    
    if (success && temp_mq.count > 0 && temp_mq.count < *best_count) {
        *best_count = temp_mq.count;
        memcpy(best_mq, &temp_mq, sizeof(BFS_MotionQueue_t));
        *best_end_px = temp_px;
        *best_end_py = temp_py;
        return 1;
    }
    
    return 0;
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
    
    uint8_t found = 0;
    for (int i = 0; i < count; i++) {
        if (!used[i]) {
            used[i] = 1;
            target_order[depth] = i;
            if (try_all_permutations(box_order, target_order, depth + 1,
                                      used, count, best_mq, best_count,
                                      best_end_px, best_end_py)) {
                found = 1;
            }
            used[i] = 0;
        }
    }
    
    return found;
}

/* ============================================================================
 * 第1关：直接推箱子（无识别，尝试所有配对）
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
        // 回到起点
				bfs_return_to_start(mq, best_end_px, best_end_py);
        // 压缩路径
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

// 为箱子 box_idx 找最佳炸弹+墙组合
// 返回：成功=1, out_bi=炸弹索引, out_wx/out_wy=要炸的墙坐标
// target_x/y: 目标坐标（>=0 时检查推动位，<0 时检查箱子四邻格）
static int bfs_find_bomb_for_box(int box_idx, int8_t cx, int8_t cy,
                                  int8_t target_x, int8_t target_y,
                                  int *out_bi, int8_t *out_wx, int8_t *out_wy) {
    int best_cost = 9999;
    int found = 0;
    uint8_t save[ROWS][COLS];
    memcpy(save, map_data, sizeof(map_data));
    
    // 将其他未推箱子视为障碍
    for (int i = 0; i < box_count; i++) {
        if (i != box_idx && !box_pushed[i]) map_data[box_x[i]][box_y[i]] = CELL_WALL;
    }
    
    // 计算需要检查的可达位置列表
    // 有 target 时：推动位（玩家站在哪里才能把箱子往目标方向推）
    // 无 target 时：箱子四个相邻格（旧行为：识别失败场景）
    int8_t check_pos[4][2];
    int num_check = 0;
    
    if (target_x >= 0 && target_y >= 0) {
        int8_t dx = target_x - box_x[box_idx];
        int8_t dy = target_y - box_y[box_idx];
        // 推动位 = 箱子另一侧（推上→站在下面，推下→站在上面，以此类推）
        if (dx != 0) {
            int8_t px = box_x[box_idx] + (dx > 0 ? -1 : 1);
            int8_t py = box_y[box_idx];
            if (px >= 0 && px < ROWS && py >= 0 && py < COLS) {
                check_pos[num_check][0] = px;
                check_pos[num_check][1] = py;
                num_check++;
            }
        }
        if (dy != 0) {
            int8_t px = box_x[box_idx];
            int8_t py = box_y[box_idx] + (dy > 0 ? -1 : 1);
            if (px >= 0 && px < ROWS && py >= 0 && py < COLS) {
                check_pos[num_check][0] = px;
                check_pos[num_check][1] = py;
                num_check++;
            }
        }
    }
    
    // 没有 target 或无推动位时回退到箱子四邻格
    if (num_check == 0) {
        check_pos[0][0] = box_x[box_idx] - 1; check_pos[0][1] = box_y[box_idx];
        check_pos[1][0] = box_x[box_idx] + 1; check_pos[1][1] = box_y[box_idx];
        check_pos[2][0] = box_x[box_idx];     check_pos[2][1] = box_y[box_idx] - 1;
        check_pos[3][0] = box_x[box_idx];     check_pos[3][1] = box_y[box_idx] + 1;
        num_check = 4;
    }
    
    for (int bi = 0; bi < bomb_count; bi++) {
        if (!bomb_active[bi]) continue;
        
        // 炸弹本身在 map_data 中是 FREE，手动标记为障碍以便寻路
        map_data[bomb_x[bi]][bomb_y[bi]] = CELL_WALL;
        
        // 遍历炸弹周围 radius=1 内的墙
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;
                int8_t wx = bomb_x[bi] + dx;
                int8_t wy = bomb_y[bi] + dy;
                if (wx < 0 || wx >= ROWS || wy < 0 || wy >= COLS) continue;
                if (wx == 0 || wx == ROWS - 1 || wy == 0 || wy == COLS - 1) continue; // 跳过最外围墙
                if (save[wx][wy] != CELL_WALL) continue; // 必须是原来的墙
                
                // 临时清除这面墙，看箱子是否可达
                map_data[wx][wy] = CELL_FREE;
                
                uint8_t reachable = 0;
                for (int c = 0; c < num_check; c++) {
                    int8_t nx = check_pos[c][0];
                    int8_t ny = check_pos[c][1];
                    if (nx < 0 || nx >= ROWS || ny < 0 || ny >= COLS) continue;
                    if (map_data[nx][ny] == CELL_WALL) continue;
                    
                    int8_t acts[ROWS * COLS];
                    int alen = 0;
                    if (bfs_find_path(cx, cy, nx, ny, acts, &alen)) {
                        reachable = 1;
                        break;
                    }
                }
                map_data[wx][wy] = CELL_WALL;
                
                if (reachable) {
                    int cost = abs(cx - bomb_x[bi]) + abs(cy - bomb_y[bi])
                             + abs(bomb_x[bi] - wx) + abs(bomb_y[bi] - wy);
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
        map_data[bomb_x[bi]][bomb_y[bi]] = CELL_FREE;
    }
    
    memcpy(map_data, save, sizeof(map_data));
    return found;
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

/* ============================================================================
 * 视觉关规划函数（第2关/第3关）
 * 特点：
 *   1. 2n-1 优化：识别所有目标(n) + 识别前n-1个箱子，最后一个不需识别
 *   2. 炸弹机制：推箱子失败时自动尝试用炸弹破墙
 * ============================================================================ */
uint8_t bfs_plan_stage23(BFS_MotionQueue_t *mq, uint8_t has_bomb) {
    if (box_count == 0 || target_count == 0) return 0;
    if (box_count != target_count) return 0;
    
    memset(mq, 0, sizeof(BFS_MotionQueue_t));
    memset(target_matched, 0, sizeof(target_matched));
    memset(box_pushed, 0, sizeof(box_pushed));
    
    int8_t cur_px = origin_px;
    int8_t cur_py = origin_py;
    
    // ===== 阶段1：识别所有目标 (n 个) =====
    recog_target_idx = 0;
    for (int i = 0; i < target_count; i++) {
        if (bfs_is_bomb_active(target_x[i], target_y[i])) {
            if (has_bomb) continue;
        }
        move_to_recog(target_x[i], target_y[i], 1, mq, &cur_px, &cur_py);
    }
    
    // ===== 阶段2：识别前 n-1 个箱子，识别一个推一个 =====
    uint8_t pushed_count = 0;
    int max_attempts = box_count * 4;
    recog_box_idx = 0;
    
    while (pushed_count < box_count - 1 && max_attempts > 0) {
        max_attempts--;
        
        // 找最近的未推送箱子
        int nearest = -1;
        uint16_t min_dist = 0xFFFF;
        for (int i = 0; i < box_count; i++) {
            if (box_pushed[i]) continue;
            uint16_t dist = abs(cur_px - box_x[i]) + abs(cur_py - box_y[i]);
            if (dist < min_dist) {
                min_dist = dist;
                nearest = i;
            }
        }
        if (nearest < 0) break;
        
        // 尝试移动到箱子旁边识别
        int8_t recog_ok = move_to_recog(box_x[nearest], box_y[nearest], 2, mq, &cur_px, &cur_py);
        
        // 如果无法到达箱子，尝试用炸弹开路
        if (!recog_ok && has_bomb) {
            int bi;
            int8_t wx, wy;
            if (bfs_find_bomb_for_box(nearest, cur_px, cur_py, -1, -1, &bi, &wx, &wy)) {
                if (bfs_execute_bomb(bi, wx, wy, mq, &cur_px, &cur_py)) {
                    // 炸弹执行成功，重新尝试到达箱子
                    max_attempts++;  // 补偿本次尝试
                    continue;
                }
            }
        }
        
        if (!recog_ok) {
            box_pushed[nearest] = 1;
            pushed_count++;
            continue;
        }
        
        // 匹配目标
        int target_idx = -1;
        if (box_ids[nearest] != 0) {
            target_idx = bfs_match_box_to_target(nearest);
        }
        if (target_idx < 0) {
            min_dist = 0xFFFF;
            for (int t = 0; t < target_count; t++) {
                if (target_matched[t]) continue;
                uint16_t dist = abs(box_x[nearest] - target_x[t]) + 
                                abs(box_y[nearest] - target_y[t]);
                if (dist < min_dist) {
                    min_dist = dist;
                    target_idx = t;
                }
            }
        }
        
        if (target_idx < 0) {
            box_pushed[nearest] = 1;
            pushed_count++;
            continue;
        }
        
        // 尝试推箱子
        int8_t end_px, end_py;
        int8_t actions[ROWS * COLS * 2];
        int action_len = 0;
        int8_t push_dirs[ROWS * COLS];
        int push_len = 0;
        
        if (bfs_plan_push(cur_px, cur_py,
                          box_x[nearest], box_y[nearest],
                          target_x[target_idx], target_y[target_idx],
                          actions, &action_len, push_dirs, &push_len,
                          &end_px, &end_py)) {
            for (int i = 0; i < action_len; i++) {
                bfs_add_move_cmd(mq, actions[i]);
            }
            cur_px = end_px;
            cur_py = end_py;
            box_pushed[nearest] = 1;
            target_matched[target_idx] = 1;
            pushed_count++;
        } else if (has_bomb) {
            // 推箱子失败，循环尝试所有可用炸弹（依次使用，每用一颗就重试一次推送）
            uint8_t bomb_worked = 0;
            while (!bomb_worked) {
                int bi;
                int8_t wx, wy;
                if (!bfs_find_bomb_for_box(nearest, cur_px, cur_py,
                                           target_x[target_idx], target_y[target_idx],
                                           &bi, &wx, &wy)) {
                    break;  // 没有更多可用炸弹
                }
                if (!bfs_execute_bomb(bi, wx, wy, mq, &cur_px, &cur_py)) {
                    continue;  // 炸弹执行失败，尝试下一个
                }
                max_attempts++;  // 补偿本次尝试
                
                if (bfs_plan_push(cur_px, cur_py,
                                  box_x[nearest], box_y[nearest],
                                  target_x[target_idx], target_y[target_idx],
                                  actions, &action_len, push_dirs, &push_len,
                                  &end_px, &end_py)) {
                    for (int i = 0; i < action_len; i++) {
                        bfs_add_move_cmd(mq, actions[i]);
                    }
                    cur_px = end_px;
                    cur_py = end_py;
                    box_pushed[nearest] = 1;
                    target_matched[target_idx] = 1;
                    pushed_count++;
                    bomb_worked = 1;
                }
                // 炸弹用了但箱子仍推不动 → while 循环继续尝试下一个炸弹
            }
            if (!bomb_worked) {
                box_pushed[nearest] = 1;
                pushed_count++;
            }
        } else {
            box_pushed[nearest] = 1;
            pushed_count++;
        }
    }
    
    // ===== 阶段3：最后一个箱子，不需要识别，推到剩下的目标 =====
    int last_box = -1;
    int last_target = -1;
    
    for (int i = 0; i < box_count; i++) {
        if (!box_pushed[i]) {
            last_box = i;
            break;
        }
    }
    
    for (int t = 0; t < target_count; t++) {
        if (!target_matched[t]) {
            last_target = t;
            break;
        }
    }
    
    if (last_box >= 0 && last_target >= 0) {
        int8_t end_px, end_py;
        int8_t actions[ROWS * COLS * 2];
        int action_len = 0;
        int8_t push_dirs[ROWS * COLS];
        int push_len = 0;
        
        // 尝试推最后一个箱子到剩下的目标
        if (bfs_plan_push(cur_px, cur_py,
                          box_x[last_box], box_y[last_box],
                          target_x[last_target], target_y[last_target],
                          actions, &action_len, push_dirs, &push_len,
                          &end_px, &end_py)) {
            for (int i = 0; i < action_len; i++) {
                bfs_add_move_cmd(mq, actions[i]);
            }
            cur_px = end_px;
            cur_py = end_py;
            box_pushed[last_box] = 1;
            target_matched[last_target] = 1;
        } else if (has_bomb) {
            // 最后一个箱子也尝试用炸弹 — 循环尝试所有可用炸弹
            uint8_t bomb_worked = 0;
            while (!bomb_worked) {
                int bi;
                int8_t wx, wy;
                if (!bfs_find_bomb_for_box(last_box, cur_px, cur_py,
                                           target_x[last_target], target_y[last_target],
                                           &bi, &wx, &wy)) {
                    break;  // 没有更多可用炸弹
                }
                if (!bfs_execute_bomb(bi, wx, wy, mq, &cur_px, &cur_py)) {
                    continue;
                }
                
                if (bfs_plan_push(cur_px, cur_py,
                                  box_x[last_box], box_y[last_box],
                                  target_x[last_target], target_y[last_target],
                                  actions, &action_len, push_dirs, &push_len,
                                  &end_px, &end_py)) {
                    for (int i = 0; i < action_len; i++) {
                        bfs_add_move_cmd(mq, actions[i]);
                    }
                    cur_px = end_px;
                    cur_py = end_py;
                    box_pushed[last_box] = 1;
                    target_matched[last_target] = 1;
                    bomb_worked = 1;
                }
                // 炸弹用了但推不动 → 继续尝试下一个炸弹
            }
            // bomb_worked=0时：所有炸弹用完仍推不动 → box_pushed保持0，后续返回失败（假成功漏洞修复）
        }
    }
    
    // ===== 阶段4：检查是否所有目标都有箱子推到 =====
    uint8_t all_targets_matched = 1;
    for (int t = 0; t < target_count; t++) {
        if (!target_matched[t]) { all_targets_matched = 0; break; }
    }
    
    if (all_targets_matched && mq->count > 0) {
        // 回到起点
        bfs_return_to_start(mq, cur_px, cur_py);
        // 压缩路径
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
    
    memset(g_best_mq, 0, sizeof(g_best_mq));
}

void bfs_load_map(uint8_t data[ROWS][COLS]) {
    memcpy(map_data, data, ROWS * COLS);
    box_count = 0;
    target_count = 0;
    bomb_count = 0;
    memset(target_matched, 0, sizeof(target_matched));
    memset(box_pushed, 0, sizeof(box_pushed));
    memset(bomb_active, 0, sizeof(bomb_active));
    
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            // 强制写死出发点为（4，1），更稳定一些
            if (i == 4 && j == 1) {
                origin_px = i;
                origin_py = j;
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
}