#ifndef __IMAGE_hpp__
#define __IMAGE_hpp__

#include "zf_common_typedef.hpp"

// 边线→鸟瞰数组容量上限（edge_array_change 输出，供 net_send_resampled 发送）
#define POINTS_MAX_LEN   200

// 二值化图像缓冲区（全局数组，定义在 image.cpp）
extern uint8 binary_image[];

// 边线数组（按行存边界 x 坐标，-1 = 该行丢线）
extern int16 left_edge[];     // 每行左边界 x
extern int16 right_edge[];    // 每行右边界 x
extern int16 mid_edge[];      // 每行中线 x（-1 = 该行无中线）

// 迷宫法实际行走顺序（原图坐标）。保留同一 y 行的横向移动，供原图图传显示直角。
extern int16 left_trace[POINTS_MAX_LEN][2];
extern int16 right_trace[POINTS_MAX_LEN][2];
extern int16 left_trace_num, right_trace_num;
// 近端轨迹最后一个真实点。供远端起点接续使用；对应轨迹为空时返回 (-1,-1)。
void get_near_trace_tail(int16 *left_x, int16 *left_y,
                         int16 *right_x, int16 *right_y);

// 迷宫法起始扫描点（调试用，-1 = 未找到）
extern int16 left_start_x, left_start_y;
extern int16 right_start_x, right_start_y;

// 迷宫法运行所需（main.cpp 每帧赋值：灰度图指针 + 当前大津阈值）
extern uint8 *g_gray_image;
extern uint8  g_bin_threshold;
// 触边标志（TC264 touch_boundary）：爬到图像边界停下时置 1
extern uint8  touch_boundary_left;
extern uint8  touch_boundary_right;

// 全局大津法（OTSU）求最佳二值化阈值，定义在 image.cpp
uint8 otsu_threshold(const uint8 *gray, int width, int height);

// 灰度图转二值图，定义在 image.cpp
void binarize(const uint8 *gray, uint8 *binary, int width, int height, uint8 threshold);

// 迷宫法找边线（参考 TC264 findline_lefthand/righthand_adaptive 移植）
void find_line_left(void);    // 左手迷宫法找左边界
void find_line_right(void);   // 右手迷宫法找右边界
void find_middle_line(void);  // 中间线找宫法找中间线

// 远端边线（远景段独立爬线，无角点检测版；参考 TC264 find_farline_l/r 思路）
extern int16 far_left_edge[];     // 远端左线（按行存边界 x，-1 = 丢线）
extern int16 far_right_edge[];    // 远端右线
// 远端迷宫法实际行走顺序（原图坐标），保留直角处同一行的横向移动。
extern int16 far_left_trace[POINTS_MAX_LEN][2];
extern int16 far_right_trace[POINTS_MAX_LEN][2];
extern int16 far_left_trace_num, far_right_trace_num;
extern int16 far_left_start_x, far_left_start_y;    // 远端左线起始点（初始行，-1 = 未找到）
extern int16 far_right_start_x, far_right_start_y;  // 远端右线起始点
void find_far_line_left(void);    // 远端左手迷宫法找远端左边界
void find_far_line_right(void);   // 远端右手迷宫法找远端右边界
// 远端左右完整轨迹中坐标完全相同的点数。数值大说明两条线可能跟到了同一条边界。
int16 far_trace_overlap_count(void);
// 两条远端轨迹高度重合时，只保留点数更多的一条；返回 0=均保留，1=仅保留左，2=仅保留右。
uint8 suppress_duplicate_far_trace(void);

// 边线丢线统计（调试用）：valid=有效点数, segs=丢线段数,
// seg_buf[][3]=丢线段表 [段][0]=起始行 [段][1]=结束行(含) [段][2]=长度；无丢线时 seg_buf[0][0]=-1
void edge_loss_stats(const int16 *edge, int16 *valid, int16 *segs, int16 seg_buf[][3]);

// 边线→鸟瞰数组（edge_array_change 输出，鸟瞰像素坐标，供 net_send_resampled 发送/中线计算）
extern float rpts0s[POINTS_MAX_LEN][2], rpts1s[POINTS_MAX_LEN][2];
extern int16 rpts0s_num, rpts1s_num;
void edge_array_change(void);
// 仅调试：完整近端轨迹经鸟瞰映射、等距重采样后的结果，不参与现有中线或控制。
extern float debug_resampled_left[POINTS_MAX_LEN][2];
extern float debug_resampled_right[POINTS_MAX_LEN][2];
extern float debug_resampled_far_left[POINTS_MAX_LEN][2];
extern float debug_resampled_far_right[POINTS_MAX_LEN][2];
extern int16 debug_left_mapped_num, debug_right_mapped_num;
extern int16 debug_far_left_mapped_num, debug_far_right_mapped_num;
extern int16 debug_resampled_left_num, debug_resampled_right_num;
extern int16 debug_resampled_far_left_num, debug_resampled_far_right_num;
void debug_resample_near_traces(void);
// 中线（鸟瞰坐标，浮点点数组）：近端 + 远端，find_middle_line 输出。
extern float middle_line[POINTS_MAX_LEN][2];
extern int16 middle_line_num;
// 近端中线的反向逆透视结果（原图像素坐标）；只含 invx/invy 能反查的有效点。
extern int16 middle_line_original[POINTS_MAX_LEN][2];
extern int16 middle_line_original_num;
// 仅调试：从近端最终中线取出的前瞻点和车头基准点，尚未参与电机控制。
extern float debug_lookahead_point[2];
extern int16 debug_lookahead_index;
extern uint8 debug_lookahead_valid;
void update_debug_lookahead_point(void);
#endif
