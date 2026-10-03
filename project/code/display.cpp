#include "init.hpp"

// 鸟瞰二值图缓冲（IPM_BIRD_W * IPM_BIRD_H = 160x120）
static uint8 bird_binary[IPM_BIRD_W * IPM_BIRD_H];

void display_data(uint8 bin_threshold)
{
    // 图像区域下方（y=110 起）显示 FPS 与阈值
    ips200.show_string(0, 110, "FPS:");
    ips200.show_uint(48, 110, fps, 3);
    ips200.show_string(0, 130, "TH:");
    ips200.show_uint(48, 130, bin_threshold, 3);
    // 起始点（迷宫法底行起点）
    ips200.show_string(0, 150, "LS:");
    ips200.show_int(48, 150, left_start_x, 3);
    ips200.show_string(96, 150, "RS:");
    ips200.show_int(144, 150, right_start_x, 3);
}

void display_image(uint8 bin_threshold)
{
    // 取摄像头灰度图（160*120）
    uint8* gray_image = uvc_dev.get_gray_image_ptr();
    if(NULL == gray_image)
    {
        return;
    }

    // 生成二值鸟瞰图：把二值图逆透视重映射成鸟瞰图（160x120）
    ipm_remap(binary_image, bird_binary);

    // 上半图像区（高度约106px），三块并排各80宽：
    // 左 (0,0) 灰度图 / 中 (80,0) 大津法二值图 / 右 (160,0) 二值化鸟瞰图
    ips200.show_gray_image(0,   0, gray_image,   UVC_WIDTH,  UVC_HEIGHT, 80, 106, 0);
    ips200.show_gray_image(80,  0, binary_image, UVC_WIDTH,  UVC_HEIGHT, 80, 106, 0);
    ips200.show_gray_image(160, 0, bird_binary,  IPM_BIRD_W, IPM_BIRD_H, 80, 106, 0);
}

void display_edge(void)
{
    // 三块显示区：灰度(0,0) / 二值(80,0) / 鸟瞰(160,0)，各80宽、106高
    // 原图坐标(160x120) → 显示坐标：x*80/160, y*106/120；鸟瞰块需先经 ipm_map_point 映射
    // 画点做边界保护：3x3 加粗时可能越过屏幕左/上边界
    for(int y = 0; y < UVC_HEIGHT; y++)
    {
        if(left_edge[y] >= 0)
        {
            int16 lx = left_edge[y];
            uint16 sy  = (uint16)(y * 106 / UVC_HEIGHT);
            // 鸟瞰块：原图边线点映射到鸟瞰坐标（越界跳过）
            int16 bx = -1, by = -1;
            ipm_map_point(lx, y, &bx, &by);
            if(bx >= 0 && by >= 0)
            {
                uint16 sx2 = 160 + (uint16)(bx * 80 / IPM_BIRD_W);
                uint16 sy2 = (uint16)(by * 106 / IPM_BIRD_H);
                for(int dy = -1; dy <= 1; dy++)
                {
                    for(int dx = -1; dx <= 1; dx++)
                    {
                        int16 px = (int16)sx2 + dx, py = (int16)sy2 + dy;
                        if(px >= 0 && px < 240 && py >= 0 && py < 320)
                            ips200.draw_point((uint16)px, (uint16)py, RGB565_RED);
                    }
                }
            }
            // 3x3 加粗（灰度/二值块）
            uint16 sx0 = (uint16)(lx * 80 / UVC_WIDTH);
            uint16 sx1 = 80  + (uint16)(lx * 80 / UVC_WIDTH);
            for(int dy = -1; dy <= 1; dy++)
            {
                for(int dx = -1; dx <= 1; dx++)
                {
                    int16 px0 = (int16)sx0 + dx, px1 = (int16)sx1 + dx, py = (int16)sy + dy;
                    if(px0 >= 0 && py >= 0 && py < 320)
                        ips200.draw_point((uint16)px0, (uint16)py, RGB565_RED);
                    if(px1 >= 0 && px1 < 240 && py >= 0 && py < 320)
                        ips200.draw_point((uint16)px1, (uint16)py, RGB565_RED);
                }
            }
        }
        if(right_edge[y] >= 0)
        {
            int16 rx = right_edge[y];
            uint16 sy  = (uint16)(y * 106 / UVC_HEIGHT);
            // 鸟瞰块：原图边线点映射到鸟瞰坐标（越界跳过）
            int16 bx = -1, by = -1;
            ipm_map_point(rx, y, &bx, &by);
            if(bx >= 0 && by >= 0)
            {
                uint16 sx2 = 160 + (uint16)(bx * 80 / IPM_BIRD_W);
                uint16 sy2 = (uint16)(by * 106 / IPM_BIRD_H);
                for(int dy = -1; dy <= 1; dy++)
                {
                    for(int dx = -1; dx <= 1; dx++)
                    {
                        int16 px = (int16)sx2 + dx, py = (int16)sy2 + dy;
                        if(px >= 0 && px < 240 && py >= 0 && py < 320)
                            ips200.draw_point((uint16)px, (uint16)py, RGB565_BLUE);
                    }
                }
            }
            // 3x3 加粗（灰度/二值块）
            uint16 sx0 = (uint16)(rx * 80 / UVC_WIDTH);
            uint16 sx1 = 80  + (uint16)(rx * 80 / UVC_WIDTH);
            for(int dy = -1; dy <= 1; dy++)
            {
                for(int dx = -1; dx <= 1; dx++)
                {
                    int16 px0 = (int16)sx0 + dx, px1 = (int16)sx1 + dx, py = (int16)sy + dy;
                    if(px0 >= 0 && py >= 0 && py < 320)
                        ips200.draw_point((uint16)px0, (uint16)py, RGB565_BLUE);
                    if(px1 >= 0 && px1 < 240 && py >= 0 && py < 320)
                        ips200.draw_point((uint16)px1, (uint16)py, RGB565_BLUE);
                }
            }
        }
    }
}
