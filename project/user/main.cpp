#include "init.hpp"
#include "image.hpp"    // otsu_threshold 大津法

#include <cstdio>
// IPS200 屏幕对象（240*320 分辨率）
zf_device_ips200 ips200;
// UVC 免驱 USB 摄像头对象
zf_device_uvc uvc_dev;

int main(int, char**)
{
    // 远程 SSH 执行时 stdout 是管道全缓冲，关闭缓冲保证 printf 实时输出
    setvbuf(stdout, NULL, _IONBF, 0);

    // 1. 初始化屏幕 + 摄像头 + 1秒中断
    init_all();

    // 2. TCP 连接上位机（失败不阻塞，上位机未开也能继续跑）
    net_init();

    // 3. 主循环：采集 + 大津法二值化显示 + FPS 打印
    uint8 cur_threshold = 0;     // 当前大津法阈值（供数据区显示）
    int   send_cnt = 0;          // 图传节流计数（每 5 帧发 1 帧）
    uint8 far_keep_mode = 0;     // 0=两条保留，1=仅左，2=仅右（重合抑制）
    while(1)
    {
        redy_go();                     // 检测 P77 发车按键（按下置 key_start_flag=1）
        motor_pwm_out(2000, 2000);     // 测试：两电机 20% 正转（直行）

        if(uvc_dev.wait_image_refresh() == 0)
        {
            frame_count++;        // 主循环每取到一帧计一次

            uint8* gray_image = uvc_dev.get_gray_image_ptr();   // 取灰度图
            if(gray_image != NULL)
            {
                uint8 t = otsu_threshold(gray_image, UVC_WIDTH, UVC_HEIGHT); // 大津法动态阈值
                binarize(gray_image, binary_image, UVC_WIDTH, UVC_HEIGHT, t);
                // 迷宫法在灰度图上爬线（局部自适应阈值）+ 大津阈值判起点
                g_gray_image = gray_image;
                g_bin_threshold = t;

                find_line_left();      // 迷宫法找左边界
                find_line_right();     // 迷宫法找右边界
                find_far_line_left();  // 远端左线（远景段)）
                find_far_line_right(); // 远端右线（远景段）
                far_keep_mode = suppress_duplicate_far_trace();
                edge_array_change();   // 原图边线→鸟瞰数组（供上位机/中线用）
                debug_resample_near_traces(); // 仅打印验证：完整近端轨迹的鸟瞰等距重采样
                find_middle_line();    // 最终近端鸟瞰中线（供后续误差计算）
                update_debug_lookahead_point(); // 仅标出前瞻点，不参与电机控制
                cur_threshold = t;     // 记录阈值供数据区显示
                // 图传上位机：每 5 帧发 1 帧（send 阻塞，节流保帧率）
                if(net_connected() && (++send_cnt % 5 == 0))
                {
                    display_image(t);      // 灰度 + 大津法二值化上屏
                    display_edge();        // 边线叠加（左红/右蓝）
                    net_send_binary(binary_image);                          // 二值图（1bit/px）
                    net_send_edges_bird_trace(left_trace, left_trace_num, right_trace, right_trace_num,
                                              far_left_trace, far_left_trace_num, far_right_trace, far_right_trace_num); // 四条完整鸟瞰边线
                    net_send_edges_raw_trace(left_trace, left_trace_num, right_trace, right_trace_num); // 原图完整轨迹（保留直角横向点）
                    net_send_far_edges_raw_trace(far_left_trace, far_left_trace_num, far_right_trace, far_right_trace_num); // 远端完整原图轨迹（左紫/右青）
                    net_send_debug_resampled_bird(debug_resampled_left, debug_resampled_left_num,
                                                   debug_resampled_right, debug_resampled_right_num,
                                                   debug_resampled_far_left, debug_resampled_far_left_num,
                                                   debug_resampled_far_right, debug_resampled_far_right_num);
                    net_send_middle_line();                                  // 中线（鸟瞰绿+原图紫）
                    net_send_guidance_points(debug_lookahead_point, debug_lookahead_valid);
                    net_send_bird();                                         // 鸟瞰二值图（上位机综合界面第三块）
                }
            }

            if(fps_updated)       // 中断每秒置位：刷新 FPS + 阈值显示
            {
                display_data(cur_threshold);
                // 每秒打印爬线统计：有效点数 + 起点y + 触边标志，定位边线长度跳变（近端 + 远端）
                int16 l_valid = 0, r_valid = 0, fl_valid = 0, fr_valid = 0;
                for(int i = 0; i < UVC_HEIGHT; i++)
                {
                    if(left_edge[i] >= 0) l_valid++;
                    if(right_edge[i] >= 0) r_valid++;
                    if(far_left_edge[i] >= 0) fl_valid++;
                    if(far_right_edge[i] >= 0) fr_valid++;
                }
                printf("FPS = %d | Ln=%d Ly0=%d Lbd=%d | Rn=%d Ry0=%d Rbd=%d | FLn=%d FLy0=%d | FRn=%d FRy0=%d\n",
                       fps,
                       l_valid, left_start_y, touch_boundary_left,
                       r_valid, right_start_y, touch_boundary_right,
                       fl_valid, far_left_start_y,
                       fr_valid, far_right_start_y);
                // 完整轨迹数组的真实长度：与中间原图边线帧逐点发送的数量对应。
                int16 nl_tail_x, nl_tail_y, nr_tail_x, nr_tail_y;
                get_near_trace_tail(&nl_tail_x, &nl_tail_y, &nr_tail_x, &nr_tail_y);
                int16 far_overlap = far_trace_overlap_count();
                printf("TRACE = NL=%d NR=%d | FL=%d FR=%d | Ntail L=(%d,%d) R=(%d,%d) | F-overlap=%d F-keep=%d\n",
                       left_trace_num, right_trace_num,
                       far_left_trace_num, far_right_trace_num,
                       nl_tail_x, nl_tail_y, nr_tail_x, nr_tail_y, far_overlap, far_keep_mode);
                printf("RESAMPLE = mapped NL=%d NR=%d FL=%d FR=%d | blur=7 | time=0.35 | step=1.0px | equal NL=%d NR=%d FL=%d FR=%d\n",
                       debug_left_mapped_num, debug_right_mapped_num,
                       debug_far_left_mapped_num, debug_far_right_mapped_num,
                       debug_resampled_left_num, debug_resampled_right_num,
                       debug_resampled_far_left_num, debug_resampled_far_right_num);
                printf("RS-L:");
                for(int16 i = 0; i < debug_resampled_left_num; i++)
                    printf(" (%.1f,%.1f)", debug_resampled_left[i][0], debug_resampled_left[i][1]);
                printf("\nRS-R:");
                for(int16 i = 0; i < debug_resampled_right_num; i++)
                    printf(" (%.1f,%.1f)", debug_resampled_right[i][0], debug_resampled_right[i][1]);
                const char *middle_near_base = (debug_resampled_left_num == 0 && debug_resampled_right_num == 0) ? "none" :
                                               ((debug_resampled_left_num >= debug_resampled_right_num) ? "left" : "right");
                printf("\nMIDLINE = near=%s NL=%d NR=%d | base=smooth+1px → center-resample=1px | half=9.0px | count=%d | far=disabled\n",
                       middle_near_base, debug_resampled_left_num, debug_resampled_right_num,
                       middle_line_num);
                printf("MID:");
                for(int16 i = 0; i < middle_line_num; i++)
                    printf(" (%.1f,%.1f)", middle_line[i][0], middle_line[i][1]);
                printf("\nMID-RAW:");
                for(int16 i = 0; i < middle_line_original_num; i++)
                    printf(" (%d,%d)", middle_line_original[i][0], middle_line_original[i][1]);
                printf("\n");
                // 上位机综合调参界面数据区（$DBG 文本帧：fps 阈值）
                char dbg[64];
                sprintf(dbg, "$DBG %d %d", (int)fps, (int)cur_threshold);
                net_send_line(dbg);
                // 上位机起点坐标帧（$START：原图 lx ly rx ry + 鸟瞰 blx bly brx bry，未找到为 -1）
                char st[96];
                int16 blx, bly, brx, bry;
                if(left_start_x >= 0) ipm_map_point(left_start_x, left_start_y, &blx, &bly);
                else { blx = -1; bly = -1; }
                if(right_start_x >= 0) ipm_map_point(right_start_x, right_start_y, &brx, &bry);
                else { brx = -1; bry = -1; }
                sprintf(st, "$START %d %d %d %d %d %d %d %d",
                        left_start_x, left_start_y,
                        right_start_x, right_start_y,
                        blx, bly, brx, bry);
                net_send_line(st);
                // 上位机远端起点帧（$FSTART：原图 flx fly frx fry + 鸟瞰 fblx fbly fbrx fbry，未找到为 -1）
                char fst[96];
                int16 fblx, fbly, fbrx, fbry;
                if(far_left_start_x >= 0) ipm_map_point(far_left_start_x, far_left_start_y, &fblx, &fbly);
                else { fblx = -1; fbly = -1; }
                if(far_right_start_x >= 0) ipm_map_point(far_right_start_x, far_right_start_y, &fbrx, &fbry);
                else { fbrx = -1; fbry = -1; }
                sprintf(fst, "$FSTART %d %d %d %d %d %d %d %d",
                        far_left_start_x, far_left_start_y,
                        far_right_start_x, far_right_start_y,
                        fblx, fbly, fbrx, fbry);
                net_send_line(fst);
                // 上位机远端点数帧（$FCNT：fln frn，数据区第二行显示近端/远端点个数）
                char fcnt[32];
                sprintf(fcnt, "$FCNT %d %d", fl_valid, fr_valid);
                net_send_line(fcnt);
                net_send_loss();             // 边线丢线统计帧（$LOSS）
                fps_updated = 0;
            }
        }
    }

    return 0;
}
