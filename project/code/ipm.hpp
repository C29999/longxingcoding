#ifndef __IPM_H__
#define __IPM_H__

#include "zf_common_typedef.hpp"

// 逆透视参数（与 calibrate_ipm_ls2k.py 保持一致）
#define IPM_SRC_W   160
#define IPM_SRC_H   120
#define IPM_BIRD_W  160
#define IPM_BIRD_H  120

// 逆透视查找表（定义在 ipm.cpp，由标定脚本生成）
// invx/invy：鸟瞰坐标 -> 原图坐标（生成鸟瞰图用）
extern const int16 invx[IPM_BIRD_H][IPM_BIRD_W];
extern const int16 invy[IPM_BIRD_H][IPM_BIRD_W];
// mapx/mapy：原图坐标 -> 鸟瞰坐标（TC264 同向，边线点映射用）
extern const int16 mapx[IPM_SRC_H][IPM_SRC_W];
extern const int16 mapy[IPM_SRC_H][IPM_SRC_W];

// 鸟瞰重映射：src 为源灰度图(160x120)，bird 为输出鸟瞰图(160x120)
void ipm_remap(const uint8 *src, uint8 *bird);
// 正向映射：原图坐标 (sx,sy) -> 鸟瞰坐标 (bx,by)，越界填 -1
void ipm_map_point(int16 sx, int16 sy, int16 *bx, int16 *by);

#endif
