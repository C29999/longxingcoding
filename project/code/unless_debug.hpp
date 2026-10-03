#ifndef __unless_debug_hpp__
#define __unless_debug_hpp__

#include "zf_common_typedef.hpp"

// ===== TCP 图传（上位机显示图像） =====
#define NET_IP      "192.168.215.248"  // 上位机（Windows）IP
#define NET_PORT    8080                // 上位机"侦听"端口（截图确认是 8080）

void net_init(void);                     // 创建 socket 并连接上位机
void net_send_line(const char *line);    // 发送一行文本（$DBG 等指令帧）
void net_send_image(const uint8 *gray);  // 发送一帧灰度图
void net_send_binary(const uint8 *binary); // 发送一帧二值图（1bit/px，8倍压缩）
void net_send_bird(void);                // 发送一帧鸟瞰二值图（ipm_remap 后 1bit/px 打包）
int  net_connected(void);                // 返回是否已连接

// 发送逐飞 boundary 边线叠加帧（上位机自动叠在图像上，左红/右蓝）
void net_send_edges(const int16 *left_edge, const int16 *right_edge, int height);
// 发送原始边线（黄色 0xFFE0，上位机画虚线）：观察去毛刺前的毛刺
void net_send_edges_raw(const int16 *left_edge, const int16 *right_edge, int height);
// 发送按迷宫实际行走顺序保存的原图边线，保留直角处同一行的横向点。
void net_send_edges_raw_trace(const int16 (*left_trace)[2], int16 left_num,
                              const int16 (*right_trace)[2], int16 right_num);
// 发送四条完整鸟瞰轨迹：近端左红/右蓝，远端左白/右青。
void net_send_edges_bird_trace(const int16 (*near_left)[2], int16 near_left_num,
                               const int16 (*near_right)[2], int16 near_right_num,
                               const int16 (*far_left)[2], int16 far_left_num,
                               const int16 (*far_right)[2], int16 far_right_num);
// 调试用鸟瞰等距重采样点：近左黄、近右橙、远左紫、远右浅蓝。
// 四种协议色独立于原图通道，避免不同坐标系共用上位机缓存。
void net_send_debug_resampled_bird(const float (*near_left)[2], int16 near_left_num,
                                   const float (*near_right)[2], int16 near_right_num,
                                   const float (*far_left)[2], int16 far_left_num,
                                   const float (*far_right)[2], int16 far_right_num);
void net_send_resampled(const float (*left_pts)[2], int16 left_n,
                        const float (*right_pts)[2], int16 right_n);
// 发送中线：鸟瞰坐标（绿 0x07E0 → 上位机第三块）+ 经 invx/invy 反查原图坐标（紫 0xFD20 → 第一块）
void net_send_middle_line(void);
// 调试标记：鸟瞰前瞻点（洋红）和车头基准点（白），不参与控制。
void net_send_guidance_points(const float aim_point[2], uint8 aim_valid);
// 发送远端完整原图轨迹：左紫 0xA81F、右浅蓝 0x05FF。
void net_send_far_edges_raw_trace(const int16 (*left_trace)[2], int16 left_num,
                                  const int16 (*right_trace)[2], int16 right_num);
// 发送边线丢线统计帧（$LOSS：lv ls lm lf rv rs rm rf，上位机数据区显示断点大小）
void net_send_loss(void);

void debug_image(void);
void debug_data(void);
#endif
