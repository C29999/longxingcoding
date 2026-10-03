#include "init.hpp"
#include <math.h>    // sqrtf/atan2f/fabsf（边线5步链）
// 二值化图像缓冲区（全局数组，供主循环/算法共用）
uint8 binary_image[UVC_WIDTH * UVC_HEIGHT];
// 边线数组：每行边界 x 坐标，-1 表示该行丢线
int16 left_edge[UVC_HEIGHT];
int16 right_edge[UVC_HEIGHT];
int16 mid_edge[UVC_HEIGHT];
int16 left_trace[POINTS_MAX_LEN][2];
int16 right_trace[POINTS_MAX_LEN][2];
int16 left_trace_num = 0;
int16 right_trace_num = 0;
int16 left_start_x = -1, left_start_y = -1;
int16 right_start_x = -1, right_start_y = -1;

void get_near_trace_tail(int16 *left_x, int16 *left_y,
                         int16 *right_x, int16 *right_y)
{
    if(left_x != NULL)  *left_x = (left_trace_num  > 0) ? left_trace[left_trace_num  - 1][0] : -1;
    if(left_y != NULL)  *left_y = (left_trace_num  > 0) ? left_trace[left_trace_num  - 1][1] : -1;
    if(right_x != NULL) *right_x = (right_trace_num > 0) ? right_trace[right_trace_num - 1][0] : -1;
    if(right_y != NULL) *right_y = (right_trace_num > 0) ? right_trace[right_trace_num - 1][1] : -1;
}

static const int16 dir_front[4][2] = {{0, -1},
                                       {1, 0},
                                       {0, 1},
                                       {-1, 0}};
static const int16 dir_frontleft[4][2] = {{-1, -1},
                                          {1, -1},
                                          {1, 1},
                                          {-1, 1}};
static const int16 dir_frontright[4][2] = {{1, -1},
                                           {1, 1},
                                           {-1, 1},
                                           {-1, -1}};
#define MAZE_BLOCK      3      // 局部阈值邻域尺寸（奇数 3/5/7，越大越抗噪越钝）
#define MAZE_CLIP       20     // 判黑偏置：局部阈值 = 邻域均值 - 偏置。
                               // 调小→更难误判黑，减少反光/阴影处"假墙"导致爬线困住
#define BEGIN_Y_NEAR    104    // 近景起始行（= 底行 119 上移 15，避开贴地干扰区）
#define BEGIN_Y_FAR     10     // 远景起始行（再往上视为远处丢线区）
#define NEAR_STOP_Y     60     // 近端爬线终止行（= 远端起始行 FAR_SCAN_BEGIN：
                               // 近端只爬 [NEAR_STOP_Y, BEGIN_Y_NEAR]，远端爬 [BEGIN_Y_FAR, NEAR_STOP_Y]，不重叠）
#define EDGE_CUT        2      // 起点扫描左右边缘保护带
// 爬线连续转向上限：TC264 用 4（迷宫强规则场地）；我们场地噪声大，
// 偶发"前方误判黑"会连转几次，放宽到 14 让小人能绕出直角/钝角拐弯，
// 配合 step 上限防死循环（原地转圈不落点，最多白转十几步）
#define MAZE_TURN_LIMIT 14
// 迷宫法运行所需（main.cpp 每帧赋值）
uint8 *g_gray_image = NULL;    // 当前帧灰度图指针（爬线在灰度图上做）
uint8  g_bin_threshold = 0;    // 当前帧大津法阈值（起点判黑用：gray < t 视为黑）
uint8  touch_boundary_left = 0;   // 左线触边标志（TC264 touch_boundary0）
uint8  touch_boundary_right = 0;  // 右线触边标志（TC264 touch_boundary1）

// 局部自适应阈值：以 (x,y) 为中心 block×block 邻域均值 - 偏置。
// 相比固定阈值，对光照渐变/反光更鲁棒（TC264 在灰度图上爬线用）。
static inline int16 maze_local_thres(const uint8 *gray, int16 x, int16 y, int16 half)
{
    int32 sum = 0;
    for(int16 dy = -half; dy <= half; dy++)
    {
        for(int16 dx = -half; dx <= half; dx++)
        {
            sum += gray[(y + dy) * UVC_WIDTH + (x + dx)];
        }
    }
    return (int16)(sum / (MAZE_BLOCK * MAZE_BLOCK) - MAZE_CLIP);
}
/**
 * @brief 全局大津法（OTSU）求最佳二值化阈值
 * @param gray    灰度图像指针（uint8，值域 0~255）
 * @param width   图像宽度
 * @param height  图像高度
 * @return 最佳阈值 t（gray > t 判为白，否则黑）；全图同色时返回 0
 */
uint8 otsu_threshold(const uint8 *gray, int width, int height)
{
    uint32 hist[256] = {0};      // 灰度直方图（uint32 防溢出）
    int    total = width * height;

    // ① 统计直方图
    for(int i = 0; i < total; i++)
    {
        hist[gray[i]]++;
    }

    // ② 全局灰度均值 μG（用 sum 累加，避免浮点）
    uint64 sum_total = 0;
    for(int i = 0; i < 256; i++)
    {
        sum_total += (uint64)i * hist[i];
    }

    // ③ 遍历阈值 t=0..255，找类间方差最大
    uint64 sum_fg = 0;           // 前景(>t)灰度和累积
    uint32 cnt_fg = 0;           // 前景像素数累积
    double max_var = -1.0;       // 最大类间方差
    uint8  best_t  = 0;          // 对应阈值

    for(int t = 0; t < 256; t++)
    {
        cnt_fg += hist[t];
        sum_fg += (uint64)t * hist[t];

        if(cnt_fg == 0 || cnt_fg == (uint32)total) continue;   // 全一类，跳过防除零

        uint32 cnt_bg = total - cnt_fg;                // 背景像素数
        uint64 sum_bg = sum_total - sum_fg;            // 背景灰度和

        double w0 = (double)cnt_fg / total;            // 前景占比
        double w1 = 1.0 - w0;                          // 背景占比
        double mu0 = (double)sum_fg / cnt_fg;          // 前景均值
        double mu1 = (double)sum_bg / cnt_bg;          // 背景均值

        // 类间方差 σ² = w0 * w1 * (μ0 - μ1)²
        double var = w0 * w1 * (mu0 - mu1) * (mu0 - mu1);

        if(var > max_var)
        {
            max_var = var;
            best_t  = (uint8)t;
        }
    }
    return best_t;
}
/**
 * @brief 灰度图转二值图（大津法/固定阈值通用）
 * @param gray      输入灰度图（uint8，值域 0~255）
 * @param binary    输出二值图（uint8，255=白/赛道，0=黑/背景）
 * @param width     图像宽度
 * @param height    图像高度
 * @param threshold 二值化阈值（可由 otsu_threshold 动态计算）
 */
void binarize(const uint8 *gray, uint8 *binary, int width, int height, uint8 threshold)
{
    int total = width * height;

    for(int i = 0; i < total; i++)
    {
        binary[i] = (gray[i] > threshold) ? 255 : 0;
    }
}


/**
 * @brief 迷宫法找左边界（TC264 findline_lefthand_adaptive 现用版移植）
 *        起点：从近景行(BEGIN_Y_NEAR)向上逐行扫，每行中部向左找"左邻黑"，
 *             找到黑边的那一行即起点（某行噪声不会导致整线丢失）。
 *        爬线：在灰度图上用"局部自适应阈值(邻域均值-偏置)"判断黑白，
 *             左手扶墙向上爬，每行记录边界 x 到 left_edge[]，丢线行记 -1。
 *        约定：gray < 局部阈值 判为黑(墙)，否则白(路/地面)。
 */
void find_line_left(void)
{
    const uint8 *gray = g_gray_image;
    left_trace_num = 0;
    // ① 全部置丢线
    for(int i = 0; i < UVC_HEIGHT; i++)
    {
        left_edge[i] = -1;
    }
    touch_boundary_left = 0;
    if(gray == NULL)
    {
        left_start_x = -1;   // 记录：未找到
        left_start_y = -1;
        return;
    }

    // ② 起点：从近景行向上逐行扫，每行中部向左找"左邻黑"（TC264 逐行上扫）
    int16 x0 = UVC_WIDTH / 2 - 1;
    int16 y0 = BEGIN_Y_NEAR;
    uint8 found = 0;
    for(; y0 > NEAR_STOP_Y; y0--)   // 近端起点只在 [NEAR_STOP_Y, BEGIN_Y_NEAR] 找，远端管更远段
    {
        for(x0 = UVC_WIDTH / 2 - 1; x0 > EDGE_CUT; x0--)
        {
            if(gray[y0 * UVC_WIDTH + (x0 - 1)] < g_bin_threshold)   // 左邻黑
            {
                found = 1;
                break;
            }
        }
        if(found) break;
    }
    if(!found)   // 全程没找到黑边（整幅无墙/过曝/欠曝）
    {
        left_start_x = -1;   // 记录：未找到
        left_start_y = -1;
        return;
    }
    left_start_x = x0;        // 记录起始点（调试用）
    left_start_y = y0;

    // ③ 左手迷宫法向上爬（灰度图 + 局部自适应阈值）
    int16 x = x0, y = y0;
    int16 dir = 0;        // 初始朝上
    int16 step = 0;       // 已记录行数
    int16 turn = 0;       // 连续转向计数（防原地转圈）
    int16 half = MAZE_BLOCK / 2;

    while(step < UVC_HEIGHT &&
          half < x && x < UVC_WIDTH - half - 1 &&
          half < y && y < UVC_HEIGHT - half - 1 &&
          y > NEAR_STOP_Y &&            // 近端只爬到远端起始行，避免和远端重叠
          turn < MAZE_TURN_LIMIT)
    {
        // 触边停止：左边界(近处)/右边界/顶边界（TC264 touch_boundary0）
        if((x <= 1 && y < UVC_HEIGHT - 20) || x >= UVC_WIDTH - 2 || y <= 1)
        {
            touch_boundary_left = 1;
            break;
        }

        int16 lt        = maze_local_thres(gray, x, y, half);
        int16 front     = gray[(y + dir_front[dir][1]) * UVC_WIDTH + (x + dir_front[dir][0])];
        int16 frontleft = gray[(y + dir_frontleft[dir][1]) * UVC_WIDTH + (x + dir_frontleft[dir][0])];

        if(front < lt)
        {                    // 正前方是黑(墙) → 右转找路
            dir = (dir + 1) % 4;
            turn++;
        }
        else if(frontleft < lt)
        {                    // 左前黑、正前白 → 贴墙直走
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            if(y >= 0 && y < UVC_HEIGHT)
            {
                left_edge[y] = x;
                if(left_trace_num < POINTS_MAX_LEN)
                {
                    left_trace[left_trace_num][0] = x;
                    left_trace[left_trace_num][1] = y;
                    left_trace_num++;
                }
            }
            else break;
            step++;
            turn = 0;
        }
        else
        {                    // 左前白、正前白 → 斜向左前 + 左转（追墙）
            x += dir_frontleft[dir][0];
            y += dir_frontleft[dir][1];
            dir = (dir + 3) % 4;
            if(y >= 0 && y < UVC_HEIGHT)
            {
                left_edge[y] = x;
                if(left_trace_num < POINTS_MAX_LEN)
                {
                    left_trace[left_trace_num][0] = x;
                    left_trace[left_trace_num][1] = y;
                    left_trace_num++;
                }
            }
            else break;
            step++;
            turn = 0;
        }
    }
}
/**
 * @brief 迷宫法找右边界（TC264 findline_righthand_adaptive 现用版移植）
 *        与左手版镜像对称：起点逐行上扫找"右邻黑"，右手扶墙爬线，
 *        右转改为左转（dir+3），探测右前方。
 */
void find_line_right(void)
{
    const uint8 *gray = g_gray_image;
    right_trace_num = 0;
    // ① 全部置丢线
    for(int i = 0; i < UVC_HEIGHT; i++)
    {
        right_edge[i] = -1;
    }
    touch_boundary_right = 0;
    if(gray == NULL)
    {
        right_start_x = -1;  // 记录：未找到
        right_start_y = -1;
        return;
    }

    // ② 起点：从近景行向上逐行扫，每行中部向右找"右邻黑"
    int16 x1 = UVC_WIDTH / 2 + 1;
    int16 y1 = BEGIN_Y_NEAR;
    uint8 found = 0;
    for(; y1 > NEAR_STOP_Y; y1--)   // 近端起点只在 [NEAR_STOP_Y, BEGIN_Y_NEAR] 找，远端管更远段
    {
        for(x1 = UVC_WIDTH / 2 + 1; x1 < UVC_WIDTH - 1 - EDGE_CUT; x1++)
        {
            if(gray[y1 * UVC_WIDTH + (x1 + 1)] < g_bin_threshold)   // 右邻黑
            {
                found = 1;
                break;
            }
        }
        if(found) break;
    }
    if(!found)   // 全程没找到黑边
    {
        right_start_x = -1;  // 记录：未找到
        right_start_y = -1;
        return;
    }
    right_start_x = x1;       // 记录起始点（调试用）
    right_start_y = y1;

    // ③ 右手迷宫法向上爬（灰度图 + 局部自适应阈值）
    int16 x = x1, y = y1;
    int16 dir = 0;        // 初始朝上
    int16 step = 0;
    int16 turn = 0;
    int16 half = MAZE_BLOCK / 2;

    while(step < UVC_HEIGHT &&
          0 < x && x < UVC_WIDTH - 3 &&
          y < UVC_HEIGHT - 1 &&
          y > NEAR_STOP_Y &&            // 近端只爬到远端起始行，避免和远端重叠
          turn < MAZE_TURN_LIMIT)
    {
        // 触边停止：右边界(近处)/左边界/顶边界（TC264 touch_boundary1）
        if((x >= UVC_WIDTH - 2 && y < UVC_HEIGHT - 20) || x <= 1 || y <= 1)
        {
            touch_boundary_right = 1;
            break;
        }

        int16 lt         = maze_local_thres(gray, x, y, half);
        int16 front      = gray[(y + dir_front[dir][1]) * UVC_WIDTH + (x + dir_front[dir][0])];
        int16 frontright = gray[(y + dir_frontright[dir][1]) * UVC_WIDTH + (x + dir_frontright[dir][0])];

        if(front < lt)
        {                    // 正前方是黑(墙) → 左转找路
            dir = (dir + 3) % 4;
            turn++;
        }
        else if(frontright < lt)
        {                    // 右前黑、正前白 → 贴墙直走
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            if(y >= 0 && y < UVC_HEIGHT)
            {
                right_edge[y] = x;
                if(right_trace_num < POINTS_MAX_LEN)
                {
                    right_trace[right_trace_num][0] = x;
                    right_trace[right_trace_num][1] = y;
                    right_trace_num++;
                }
            }
            else break;
            step++;
            turn = 0;
        }
        else
        {                    // 右前白、正前白 → 斜向右前 + 右转（追墙）
            x += dir_frontright[dir][0];
            y += dir_frontright[dir][1];
            dir = (dir + 1) % 4;
            if(y >= 0 && y < UVC_HEIGHT)
            {
                right_edge[y] = x;
                if(right_trace_num < POINTS_MAX_LEN)
                {
                    right_trace[right_trace_num][0] = x;
                    right_trace[right_trace_num][1] = y;
                    right_trace_num++;
                }
            }
            else break;
            step++;
            turn = 0;
        }
    }
}

// ============ 远端边线（远景段独立爬线，无角点检测版） ============
// 从远景段扫描起点，迷宫法向上爬，作为近端线的远端延伸（远处预判用）。
// 后续接入 L 角点检测后，远端起点可改为"近端 L 角点反变换回原图"的位置（TC264 find_farline_*）。
#define FAR_SCAN_BEGIN  60      // 远端起点扫描起始行（图像中部偏上，向远景 BEGIN_Y_FAR 扫）
int16 far_left_edge[UVC_HEIGHT];     // 远端左线（按行存边界 x，-1 = 丢线）
int16 far_right_edge[UVC_HEIGHT];    // 远端右线
int16 far_left_trace[POINTS_MAX_LEN][2];
int16 far_right_trace[POINTS_MAX_LEN][2];
int16 far_left_trace_num = 0;
int16 far_right_trace_num = 0;
int16 far_left_start_x = -1, far_left_start_y = -1;    // 远端左线起始点（初始行）
int16 far_right_start_x = -1, far_right_start_y = -1;  // 远端右线起始点

int16 far_trace_overlap_count(void)
{
    int16 overlap = 0;
    for(int i = 0; i < far_left_trace_num; i++)
    {
        for(int j = 0; j < far_right_trace_num; j++)
        {
            if(far_left_trace[i][0] == far_right_trace[j][0] &&
               far_left_trace[i][1] == far_right_trace[j][1])
            {
                overlap++;
                break;
            }
        }
    }
    return overlap;
}

uint8 suppress_duplicate_far_trace(void)
{
    int16 min_num = (far_left_trace_num < far_right_trace_num) ? far_left_trace_num : far_right_trace_num;
    if(min_num <= 0) return 0;

    int16 overlap = far_trace_overlap_count();
    // 超过较短轨迹的 70% 坐标重合，表示两条线实质上在追同一条轮廓。
    if((int32)overlap * 100 < (int32)min_num * 70) return 0;

    if(far_left_trace_num >= far_right_trace_num)
    {
        far_right_trace_num = 0;
        far_right_start_x = -1;
        far_right_start_y = -1;
        for(int i = 0; i < UVC_HEIGHT; i++) far_right_edge[i] = -1;
        return 1;       // 只保留远端左线
    }

    far_left_trace_num = 0;
    far_left_start_x = -1;
    far_left_start_y = -1;
    for(int i = 0; i < UVC_HEIGHT; i++) far_left_edge[i] = -1;
    return 2;           // 只保留远端右线
}

void find_far_line_left(void)
{
    const uint8 *gray = g_gray_image;
    far_left_trace_num = 0;
    for(int i = 0; i < UVC_HEIGHT; i++) far_left_edge[i] = -1;   // 全部置丢线
    if(gray == NULL) { far_left_start_x = -1; far_left_start_y = -1; return; }

    // 起点：从 FAR_SCAN_BEGIN 向上扫，每行中部向左找“左邻黑”（大津阈值判黑）
    int16 x0 = UVC_WIDTH / 2 - 1;
    int16 y0 = FAR_SCAN_BEGIN;
    uint8 found = 0;
    for(; y0 > BEGIN_Y_FAR; y0--)
    {
        for(x0 = UVC_WIDTH / 2 - 1; x0 > EDGE_CUT; x0--)
        {
            if(gray[y0 * UVC_WIDTH + (x0 - 1)] < g_bin_threshold)
            {
                found = 1;
                break;
            }
        }
        if(found) break;
    }
    if(!found)
    {
        far_left_start_x = -1; far_left_start_y = -1;
        return;
    }
    far_left_start_x = x0;
    far_left_start_y = y0;          // 远端初始行

    // 迷宫法向上爬（与近端相同：局部自适应阈值判黑）
    int16 x = x0, y = y0;
    int16 dir = 0, step = 0, turn = 0;
    int16 half = MAZE_BLOCK / 2;
    while(step < UVC_HEIGHT &&
          half < x && x < UVC_WIDTH - half - 1 &&
          half < y && y < UVC_HEIGHT - half - 1 &&
          turn < MAZE_TURN_LIMIT)
    {
        // 触边停止（不置近端 touch_boundary 标志，避免干扰近端逻辑）
        if((x <= 1 && y < UVC_HEIGHT - 20) || x >= UVC_WIDTH - 2 || y <= 1)
            break;

        int16 lt        = maze_local_thres(gray, x, y, half);
        int16 front     = gray[(y + dir_front[dir][1]) * UVC_WIDTH + (x + dir_front[dir][0])];
        int16 frontleft = gray[(y + dir_frontleft[dir][1]) * UVC_WIDTH + (x + dir_frontleft[dir][0])];

        if(front < lt)
        {                    // 正前方是黑(墙) → 右转找路
            dir = (dir + 1) % 4;
            turn++;
        }
        else if(frontleft < lt)
        {                    // 左前黑、正前白 → 贴墙直走
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            if(y >= 0 && y < UVC_HEIGHT)
            {
                far_left_edge[y] = x;
                if(far_left_trace_num < POINTS_MAX_LEN)
                {
                    far_left_trace[far_left_trace_num][0] = x;
                    far_left_trace[far_left_trace_num][1] = y;
                    far_left_trace_num++;
                }
            }
            else break;
            step++;
            turn = 0;
        }
        else
        {                    // 左前白、正前白 → 斜向左前 + 左转（追墙）
            x += dir_frontleft[dir][0];
            y += dir_frontleft[dir][1];
            dir = (dir + 3) % 4;
            if(y >= 0 && y < UVC_HEIGHT)
            {
                far_left_edge[y] = x;
                if(far_left_trace_num < POINTS_MAX_LEN)
                {
                    far_left_trace[far_left_trace_num][0] = x;
                    far_left_trace[far_left_trace_num][1] = y;
                    far_left_trace_num++;
                }
            }
            else break;
            step++;
            turn = 0;
        }
    }
}

void find_far_line_right(void)
{
    const uint8 *gray = g_gray_image;
    far_right_trace_num = 0;
    for(int i = 0; i < UVC_HEIGHT; i++) far_right_edge[i] = -1;   // 全部置丢线
    if(gray == NULL) { far_right_start_x = -1; far_right_start_y = -1; return; }

    // 起点：从 FAR_SCAN_BEGIN 向上扫，每行中部向右找“右邻黑”
    int16 x1 = UVC_WIDTH / 2 + 1;
    int16 y1 = FAR_SCAN_BEGIN;
    uint8 found = 0;
    for(; y1 > BEGIN_Y_FAR; y1--)
    {
        for(x1 = UVC_WIDTH / 2 + 1; x1 < UVC_WIDTH - 1 - EDGE_CUT; x1++)
        {
            if(gray[y1 * UVC_WIDTH + (x1 + 1)] < g_bin_threshold)
            {
                found = 1;
                break;
            }
        }
        if(found) break;
    }
    if(!found)
    {
        far_right_start_x = -1; far_right_start_y = -1;
        return;
    }
    far_right_start_x = x1;
    far_right_start_y = y1;         // 远端初始行

    // 迷宫法向上爬（与近端相同）
    int16 x = x1, y = y1;
    int16 dir = 0, step = 0, turn = 0;
    int16 half = MAZE_BLOCK / 2;
    while(step < UVC_HEIGHT &&
          0 < x && x < UVC_WIDTH - 3 &&
          y < UVC_HEIGHT - 1 &&
          turn < MAZE_TURN_LIMIT)
    {
        // 触边停止
        if((x >= UVC_WIDTH - 2 && y < UVC_HEIGHT - 20) || x <= 1 || y <= 1)
            break;

        int16 lt         = maze_local_thres(gray, x, y, half);
        int16 front      = gray[(y + dir_front[dir][1]) * UVC_WIDTH + (x + dir_front[dir][0])];
        int16 frontright = gray[(y + dir_frontright[dir][1]) * UVC_WIDTH + (x + dir_frontright[dir][0])];

        if(front < lt)
        {                    // 正前方是黑(墙) → 左转找路
            dir = (dir + 3) % 4;
            turn++;
        }
        else if(frontright < lt)
        {                    // 右前黑、正前白 → 贴墙直走
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            if(y >= 0 && y < UVC_HEIGHT)
            {
                far_right_edge[y] = x;
                if(far_right_trace_num < POINTS_MAX_LEN)
                {
                    far_right_trace[far_right_trace_num][0] = x;
                    far_right_trace[far_right_trace_num][1] = y;
                    far_right_trace_num++;
                }
            }
            else break;
            step++;
            turn = 0;
        }
        else
        {                    // 右前白、正前白 → 斜向右前 + 右转（追墙）
            x += dir_frontright[dir][0];
            y += dir_frontright[dir][1];
            dir = (dir + 1) % 4;
            if(y >= 0 && y < UVC_HEIGHT)
            {
                far_right_edge[y] = x;
                if(far_right_trace_num < POINTS_MAX_LEN)
                {
                    far_right_trace[far_right_trace_num][0] = x;
                    far_right_trace[far_right_trace_num][1] = y;
                    far_right_trace_num++;
                }
            }
            else break;
            step++;
            turn = 0;
        }
    }
}

// TC264 同款最后一层：中心线生成后再按固定鸟瞰距离重采样，供误差/前瞻计算使用。
static int16 resample_bird_points(const float input[][2], int16 input_num,
                                  float output[][2], float spacing);

void find_middle_line(void)
{
    // 当前标定：1px = 2.5cm，赛道宽 45cm，因此鸟瞰半宽为 22.5 / 2.5 = 9px。
    // 远端中线暂不参与计算或显示；只生成近端中线。
    const float track_half_width_bird_px = 9.0f;
    const float middle_resample_step_bird_px = 1.0f;
    const float (*base_points)[2] = 0;
    int16 base_num = 0;
    int16 x_offset = 0;
    static float middle_before_resample[POINTS_MAX_LEN][2];
    int16 middle_before_resample_num = 0;

    middle_line_num = 0;
    middle_line_original_num = 0;

    // 近端基准是已完成 7 点平滑、1px 等距重采样和帧间低通的鸟瞰边线。
    if(debug_resampled_left_num >= debug_resampled_right_num)
    {
        base_points = debug_resampled_left;
        base_num = debug_resampled_left_num;
        x_offset = 1;                  // 左线向右偏移到中线
    }
    else
    {
        base_points = debug_resampled_right;
        base_num = debug_resampled_right_num;
        x_offset = -1;                 // 右线向左偏移到中线
    }

    // ① 先得到未做最后一次等距重采样的近端鸟瞰中线。
    for(int16 i = 0; i < base_num && middle_before_resample_num < POINTS_MAX_LEN; i++)
    {
        float mx = base_points[i][0] + x_offset * track_half_width_bird_px;
        float my = base_points[i][1];
        if(mx < 0.0f || mx >= (float)IPM_BIRD_W ||
           my < 0.0f || my >= (float)IPM_BIRD_H)
            continue;
        middle_before_resample[middle_before_resample_num][0] = mx;
        middle_before_resample[middle_before_resample_num][1] = my;
        middle_before_resample_num++;
    }

    // ② 对中线本身再做一次等距重采样，等价于 TC264 的 rpts → rptsn。
    middle_line_num = resample_bird_points(middle_before_resample, middle_before_resample_num,
                                            middle_line, middle_resample_step_bird_px);

    // ③ 最终近端鸟瞰中线反查到原图，保留真实 invx/invy 映射点用于显示。
    for(int16 i = 0; i < middle_line_num && middle_line_original_num < POINTS_MAX_LEN; i++)
    {
        int16 bx = (int16)(middle_line[i][0] + 0.5f);
        int16 by = (int16)(middle_line[i][1] + 0.5f);
        if(bx < 0 || bx >= IPM_BIRD_W || by < 0 || by >= IPM_BIRD_H)
            continue;
        int16 ox = invx[by][bx];
        int16 oy = invy[by][bx];
        if(ox < 0 || ox >= UVC_WIDTH || oy < 0 || oy >= UVC_HEIGHT)
            continue;
        middle_line_original[middle_line_original_num][0] = ox;
        middle_line_original[middle_line_original_num][1] = oy;
        middle_line_original_num++;
    }
}

// 仅用于在上位机标出前瞻点。它不产生误差、不修改 middle_line、也不控制电机。
#define DEBUG_CAR_ANCHOR_X       80.0f
#define DEBUG_CAR_ANCHOR_Y       100.0f
#define DEBUG_LOOKAHEAD_STEP_PX  20   // 20 px × 2.5 cm = 0.5 m
float debug_lookahead_point[2] = {0.0f, 0.0f};
int16 debug_lookahead_index = -1;
uint8 debug_lookahead_valid = 0;

void update_debug_lookahead_point(void)
{
    debug_lookahead_valid = 0;
    debug_lookahead_index = -1;
    if(middle_line_num <= 0) return;

    int16 begin_id = 0;
    float min_d2 = 1.0e20f;
    for(int16 i = 0; i < middle_line_num; i++)
    {
        float dx = middle_line[i][0] - DEBUG_CAR_ANCHOR_X;
        float dy = middle_line[i][1] - DEBUG_CAR_ANCHOR_Y;
        float d2 = dx * dx + dy * dy;
        if(d2 < min_d2)
        {
            min_d2 = d2;
            begin_id = i;
        }
    }

    int16 aim_id = begin_id + DEBUG_LOOKAHEAD_STEP_PX;
    if(aim_id >= middle_line_num) aim_id = middle_line_num - 1;
    debug_lookahead_point[0] = middle_line[aim_id][0];
    debug_lookahead_point[1] = middle_line[aim_id][1];
    debug_lookahead_index = aim_id;
    debug_lookahead_valid = 1;
}

/**
 * @brief 统计边线丢线情况（调试用，上位机显示断点大小和位置）
 * @param edge  边线数组（left_edge / right_edge，-1=丢线）
 * @param valid 输出：有效点数（非 -1 行数）
 * @param segs  输出：丢线段数（连续 -1 的段，最多统计 6 段）
 * @param seg_buf 输出：丢线段表 [seg][0]=起始行, [seg][1]=结束行(含), [seg][2]=长度
 *                无丢线时 seg_buf[0][0]=-1
 */
void edge_loss_stats(const int16 *edge, int16 *valid, int16 *segs, int16 seg_buf[][3])
{
    int16 v = 0, s = 0;
    int16 run = 0;      // 当前连续丢线行数
    int16 run_start = -1;

    for(int y = 0; y < UVC_HEIGHT; y++)
    {
        if(edge[y] >= 0)
        {
            v++;
            if(run > 0)          // 一段丢线结束
            {
                if(s < 6)
                {
                    seg_buf[s][0] = run_start;
                    seg_buf[s][1] = y - 1;
                    seg_buf[s][2] = run;
                }
                s++;
                run = 0;
                run_start = -1;
            }
        }
        else
        {
            if(run == 0) run_start = y;   // 丢线段起点
            run++;
        }
    }
    if(run > 0)                  // 结尾的丢线段
    {
        if(s < 6)
        {
            seg_buf[s][0] = run_start;
            seg_buf[s][1] = UVC_HEIGHT - 1;
            seg_buf[s][2] = run;
        }
        s++;
    }

    *valid = v;
    *segs  = s;
    if(s == 0) seg_buf[0][0] = -1;   // 无丢线标记
}

// 边线→鸟瞰数组（供 net_send_resampled 发送 / 后续中线计算）
float rpts0s[POINTS_MAX_LEN][2];   // 左线鸟瞰坐标
float rpts1s[POINTS_MAX_LEN][2];   // 右线鸟瞰坐标
int16 rpts0s_num = 0;
int16 rpts1s_num = 0;

// 调试专用的“完整轨迹 → 鸟瞰 → 等距重采样”结果。现阶段不替换 rpts0s/rpts1s，
// 因而不会改变中线、控制或现有图传。
#define DEBUG_RESAMPLE_STEP_BIRD_PX  1.0f   // 当前 2.5cm/px 标定下约为 2.5cm：调试显示更密
#define DEBUG_BLUR_KERNEL             7     // 与 TC264 line_blur_kernel 默认值一致，三角加权
// 仅对调试图传做帧间低通：本帧占 0.35，上一帧占 0.65，抑制显示跳动。
#define DEBUG_TEMPORAL_CURRENT_WEIGHT 0.35f
#define DEBUG_TEMPORAL_RESET_DISTANCE 15.0f // 轨迹起点突变时立即重置，避免把两条不同轨迹拖在一起
float debug_resampled_left[POINTS_MAX_LEN][2];
float debug_resampled_right[POINTS_MAX_LEN][2];
float debug_resampled_far_left[POINTS_MAX_LEN][2];
float debug_resampled_far_right[POINTS_MAX_LEN][2];
static float debug_blurred_left[POINTS_MAX_LEN][2];
static float debug_blurred_right[POINTS_MAX_LEN][2];
static float debug_blurred_far_left[POINTS_MAX_LEN][2];
static float debug_blurred_far_right[POINTS_MAX_LEN][2];
static float debug_previous_left[POINTS_MAX_LEN][2];
static float debug_previous_right[POINTS_MAX_LEN][2];
static float debug_previous_far_left[POINTS_MAX_LEN][2];
static float debug_previous_far_right[POINTS_MAX_LEN][2];
static int16 debug_previous_left_num = 0;
static int16 debug_previous_right_num = 0;
static int16 debug_previous_far_left_num = 0;
static int16 debug_previous_far_right_num = 0;
int16 debug_left_mapped_num = 0;
int16 debug_right_mapped_num = 0;
int16 debug_far_left_mapped_num = 0;
int16 debug_far_right_mapped_num = 0;
int16 debug_resampled_left_num = 0;
int16 debug_resampled_right_num = 0;
int16 debug_resampled_far_left_num = 0;
int16 debug_resampled_far_right_num = 0;

static int16 map_trace_to_bird(const int16 trace[][2], int16 trace_num,
                               float mapped[][2])
{
    int16 count = 0;
    for(int16 i = 0; i < trace_num && count < POINTS_MAX_LEN; i++)
    {
        int16 bx, by;
        ipm_map_point(trace[i][0], trace[i][1], &bx, &by);
        if(bx < 0 || bx >= IPM_BIRD_W || by < 0 || by >= IPM_BIRD_H) continue;
        mapped[count][0] = (float)bx;
        mapped[count][1] = (float)by;
        count++;
    }
    return count;
}

// TC264 同款三角加权滑动平均：中心权重最大，端点以边界点复制方式补齐窗口。
static void blur_bird_points(const float input[][2], int16 count,
                             float output[][2], int16 kernel)
{
    if(count <= 0) return;
    int16 half = kernel / 2;
    float weight_sum = (float)((2 * half + 2) * (half + 1) / 2);
    for(int16 i = 0; i < count; i++)
    {
        float sum_x = 0.0f, sum_y = 0.0f;
        for(int16 offset = -half; offset <= half; offset++)
        {
            int16 index = i + offset;
            if(index < 0) index = 0;
            if(index >= count) index = count - 1;
            int16 abs_offset = (offset < 0) ? -offset : offset;
            float weight = (float)(half + 1 - abs_offset);
            sum_x += input[index][0] * weight;
            sum_y += input[index][1] * weight;
        }
        output[i][0] = sum_x / weight_sum;
        output[i][1] = sum_y / weight_sum;
    }
}

static int16 resample_bird_points(const float input[][2], int16 input_num,
                                  float output[][2], float spacing)
{
    if(input_num <= 0 || spacing <= 0.0f) return 0;
    int16 output_num = 0;
    output[output_num][0] = input[0][0];
    output[output_num][1] = input[0][1];
    output_num++;
    float accumulated = 0.0f;

    for(int16 i = 1; i < input_num && output_num < POINTS_MAX_LEN; i++)
    {
        float sx = input[i - 1][0], sy = input[i - 1][1];
        float ex = input[i][0], ey = input[i][1];
        float dx = ex - sx, dy = ey - sy;
        float segment = sqrtf(dx * dx + dy * dy);
        if(segment < 0.001f) continue;

        while(accumulated + segment >= spacing && output_num < POINTS_MAX_LEN)
        {
            float ratio = (spacing - accumulated) / segment;
            sx += dx * ratio;
            sy += dy * ratio;
            output[output_num][0] = sx;
            output[output_num][1] = sy;
            output_num++;
            dx = ex - sx;
            dy = ey - sy;
            segment = sqrtf(dx * dx + dy * dy);
            accumulated = 0.0f;
            if(segment < 0.001f) break;
        }
        accumulated += segment;
    }
    return output_num;
}

// 帧间一阶低通只写回调试重采样数组。点按“距轨迹起点的等距序号”对齐，
// 因此不会触碰原始轨迹、rpts 数组、中线或控制数据。
static void smooth_debug_trace_over_time(float points[][2], int16 count,
                                         float previous[][2], int16 *previous_num)
{
    if(count <= 0)
    {
        *previous_num = 0;
        return;
    }

    int use_previous = (*previous_num > 0);
    if(use_previous)
    {
        float dx = points[0][0] - previous[0][0];
        float dy = points[0][1] - previous[0][1];
        if(dx * dx + dy * dy > DEBUG_TEMPORAL_RESET_DISTANCE * DEBUG_TEMPORAL_RESET_DISTANCE)
            use_previous = 0;
    }

    if(use_previous)
    {
        int16 common = (count < *previous_num) ? count : *previous_num;
        for(int16 i = 0; i < common; i++)
        {
            points[i][0] = previous[i][0] * (1.0f - DEBUG_TEMPORAL_CURRENT_WEIGHT)
                         + points[i][0] * DEBUG_TEMPORAL_CURRENT_WEIGHT;
            points[i][1] = previous[i][1] * (1.0f - DEBUG_TEMPORAL_CURRENT_WEIGHT)
                         + points[i][1] * DEBUG_TEMPORAL_CURRENT_WEIGHT;
        }
    }

    for(int16 i = 0; i < count; i++)
    {
        previous[i][0] = points[i][0];
        previous[i][1] = points[i][1];
    }
    *previous_num = count;
}

void debug_resample_near_traces(void)
{
    float mapped_left[POINTS_MAX_LEN][2];
    float mapped_right[POINTS_MAX_LEN][2];
    debug_left_mapped_num = map_trace_to_bird(left_trace, left_trace_num, mapped_left);
    debug_right_mapped_num = map_trace_to_bird(right_trace, right_trace_num, mapped_right);
    blur_bird_points(mapped_left, debug_left_mapped_num, debug_blurred_left, DEBUG_BLUR_KERNEL);
    blur_bird_points(mapped_right, debug_right_mapped_num, debug_blurred_right, DEBUG_BLUR_KERNEL);
    debug_resampled_left_num = resample_bird_points(debug_blurred_left, debug_left_mapped_num,
                                                     debug_resampled_left, DEBUG_RESAMPLE_STEP_BIRD_PX);
    debug_resampled_right_num = resample_bird_points(debug_blurred_right, debug_right_mapped_num,
                                                      debug_resampled_right, DEBUG_RESAMPLE_STEP_BIRD_PX);
    float mapped_far_left[POINTS_MAX_LEN][2];
    float mapped_far_right[POINTS_MAX_LEN][2];
    debug_far_left_mapped_num = map_trace_to_bird(far_left_trace, far_left_trace_num, mapped_far_left);
    debug_far_right_mapped_num = map_trace_to_bird(far_right_trace, far_right_trace_num, mapped_far_right);
    blur_bird_points(mapped_far_left, debug_far_left_mapped_num, debug_blurred_far_left, DEBUG_BLUR_KERNEL);
    blur_bird_points(mapped_far_right, debug_far_right_mapped_num, debug_blurred_far_right, DEBUG_BLUR_KERNEL);
    debug_resampled_far_left_num = resample_bird_points(debug_blurred_far_left, debug_far_left_mapped_num,
                                                         debug_resampled_far_left, DEBUG_RESAMPLE_STEP_BIRD_PX);
    debug_resampled_far_right_num = resample_bird_points(debug_blurred_far_right, debug_far_right_mapped_num,
                                                          debug_resampled_far_right, DEBUG_RESAMPLE_STEP_BIRD_PX);

    smooth_debug_trace_over_time(debug_resampled_left, debug_resampled_left_num,
                                 debug_previous_left, &debug_previous_left_num);
    smooth_debug_trace_over_time(debug_resampled_right, debug_resampled_right_num,
                                 debug_previous_right, &debug_previous_right_num);
    smooth_debug_trace_over_time(debug_resampled_far_left, debug_resampled_far_left_num,
                                 debug_previous_far_left, &debug_previous_far_left_num);
    smooth_debug_trace_over_time(debug_resampled_far_right, debug_resampled_far_right_num,
                                 debug_previous_far_right, &debug_previous_far_right_num);
}

/**
 * @brief 边线数组处理边线数组变换，把原图边线转换成鸟瞰图边线
 */
// 中线（鸟瞰坐标，浮点点数组），find_middle_line 输出
float middle_line[POINTS_MAX_LEN][2];
int16 middle_line_num = 0;
int16 middle_line_original[POINTS_MAX_LEN][2];
int16 middle_line_original_num = 0;

void edge_array_change(void)
{
    int16 i, bx, by;
    int16 lc = 0, rc = 0;   // 左右有效点数计数器

    // —— 左线 ——
    for(i = 0; i < UVC_HEIGHT && lc < POINTS_MAX_LEN; i++)
    {
        if(left_edge[i] < 0) continue;          // 丢线行跳过（-1）
        ipm_map_point(left_edge[i], i, &bx, &by);   // 原图(x=left_edge[i], y=i) → 鸟瞰
        if(bx < 0 || by < 0) continue;              // 映射越界跳过
        rpts0s[lc][0] = (float)bx;                  // 存鸟瞰 x
        rpts0s[lc][1] = (float)by;                  // 存鸟瞰 y
        lc++;
    }
    rpts0s_num = lc;   // 实际点数
    // —— 右线 ——
    for(i = 0; i < UVC_HEIGHT && rc < POINTS_MAX_LEN; i++)
    {
        if(right_edge[i] < 0) continue;          // 丢线行跳过（-1）
        ipm_map_point(right_edge[i], i, &bx, &by);   // 原图(x=right_edge[i], y=i) → 鸟瞰
        if(bx < 0 || by < 0) continue;              // 映射越界跳过
        rpts1s[rc][0] = (float)bx;                  // 存鸟瞰 x
        rpts1s[rc][1] = (float)by;                  // 存鸟瞰 y
        rc++;
    }
    rpts1s_num = rc;
}



