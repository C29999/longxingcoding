#include "unless_debug.hpp"
#include "init.hpp"   // UVC_WIDTH / UVC_HEIGHT

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>

static int sock_fd = -1;

void net_init(void)
{
    sock_fd = socket(AF_INET, SOCK_STREAM, 0);
    if(sock_fd < 0) { perror("socket"); return; }

    struct sockaddr_in addr = {0};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(NET_PORT);
    inet_pton(AF_INET, NET_IP, &addr.sin_addr);

    if(connect(sock_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
        perror("connect");
        close(sock_fd);
        sock_fd = -1;
    }
    else
    {
        printf("[NET] connected to %s:%d\n", NET_IP, NET_PORT);
    }
}

int net_connected(void)
{
    return sock_fd >= 0;
}

// 发送一行文本（上位机按 '$' 或 \r\n 切分文本帧；$DBG 等指令帧走这里）
void net_send_line(const char *line)
{
    if(sock_fd < 0 || line == NULL) return;
    send(sock_fd, line, (int)strlen(line), 0);
    send(sock_fd, "\r\n", 2, 0);
}

void net_send_image(const uint8 *gray)
{
    if(sock_fd < 0) return;

    // 逐飞协议帧头（上位机 parse_seekfree_binary_frames 支持）：
    // AA 02 camera_type header_len width_LE height_LE
    // camera_type=0 → 灰度图（1字节/像素）
    uint8 header[8];
    header[0] = 0xAA;
    header[1] = 0x02;
    header[2] = 0x00;                       // camera_type = 0（灰度）
    header[3] = 0x08;                       // header_len
    header[4] = UVC_WIDTH  & 0xFF;          // width  LE
    header[5] = (UVC_WIDTH  >> 8) & 0xFF;
    header[6] = UVC_HEIGHT & 0xFF;          // height LE
    header[7] = (UVC_HEIGHT >> 8) & 0xFF;

    send(sock_fd, header, 8, 0);
    send(sock_fd, gray, UVC_WIDTH * UVC_HEIGHT, 0);
}

// 发送二值图：camera_type=1（编码为 0x20，高位3位），1bit/px（MSB在前）
// 每行 W/8 字节紧密排列；binary_image 约定：255=白/赛道，0=黑/背景 → 打包 255→1
void net_send_binary(const uint8 *binary)
{
    if(sock_fd < 0) return;

    uint8 header[8];
    header[0] = 0xAA;
    header[1] = 0x02;
    header[2] = 0x20;                       // camera_type=1 << 5（上位机取高3位）
    header[3] = 0x08;                       // header_len
    header[4] = UVC_WIDTH  & 0xFF;          // width  LE
    header[5] = (UVC_WIDTH  >> 8) & 0xFF;
    header[6] = UVC_HEIGHT & 0xFF;          // height LE
    header[7] = (UVC_HEIGHT >> 8) & 0xFF;

    static uint8 bits[UVC_WIDTH * UVC_HEIGHT / 8];   // 160*120/8 = 2400 字节
    int byte_idx = 0;
    for(int y = 0; y < UVC_HEIGHT; y++)
    {
        for(int x = 0; x < UVC_WIDTH; x += 8)
        {
            uint8 byte = 0;
            for(int b = 0; b < 8; b++)
            {
                if(binary[y * UVC_WIDTH + x + b] != 0)
                {
                    byte |= (0x80 >> b);   // MSB 在前（与上位机 np.unpackbits 一致）
                }
            }
            bits[byte_idx++] = byte;
        }
    }

    send(sock_fd, header, 8, 0);
    send(sock_fd, bits, UVC_WIDTH * UVC_HEIGHT / 8, 0);
}

// 发送鸟瞰二值图：camera_type=2（编码为 0x40，高位3位），1bit/px（MSB在前）
// 用 ipm_remap 把二值图逆透视成鸟瞰图再打包发送（160x120 → 2400 字节）
void net_send_bird(void)
{
    if(sock_fd < 0) return;

    static uint8 bird[IPM_BIRD_W * IPM_BIRD_H];   // 160x120 鸟瞰缓冲
    extern uint8 binary_image[];                    // 二值图（255=白/赛道）
    ipm_remap(binary_image, bird);

    uint8 header[8];
    header[0] = 0xAA;
    header[1] = 0x02;
    header[2] = 0x40;                       // camera_type=2 << 5（上位机取高3位）
    header[3] = 0x08;                       // header_len
    header[4] = UVC_WIDTH  & 0xFF;          // width  LE（160）
    header[5] = (UVC_WIDTH  >> 8) & 0xFF;
    header[6] = UVC_HEIGHT & 0xFF;          // height LE（120）
    header[7] = (UVC_HEIGHT >> 8) & 0xFF;

    static uint8 bits[UVC_WIDTH * UVC_HEIGHT / 8];   // 2400 字节
    int byte_idx = 0;
    for(int y = 0; y < UVC_HEIGHT; y++)
    {
        for(int x = 0; x < UVC_WIDTH; x += 8)
        {
            uint8 byte = 0;
            for(int b = 0; b < 8; b++)
            {
                if(bird[y * UVC_WIDTH + x + b] != 0)
                {
                    byte |= (0x80 >> b);   // MSB 在前
                }
            }
            bits[byte_idx++] = byte;
        }
    }

    send(sock_fd, header, 8, 0);
    send(sock_fd, bits, UVC_WIDTH * UVC_HEIGHT / 8, 0);
}

// 逐飞 CAMERA BOUNDARY 边线叠加帧：
// AA checksum 03 data_type count(LE16) color(LE16,RGB565) + 点列表
// data_type=0 → 每点2字节 (x,y 各1B)
// 校验和 = (AA + buffer[2..7]) & 0xFF（与上位机 calc_bs 一致）
// 颜色约定：0xF800 红=处理后左线实线 / 0x001F 蓝=处理后右线实线
//          0xFFE0 黄=原始边线虚线（上位机对 0xFFE0 画虚线，用于观察毛刺）
static void net_send_edge_color(const int16 *edge, int height, int side, uint16 color)
{
    if(sock_fd < 0 || height > 256) return;

    // 收集有效点（丢线行 -1 跳过），最多 height 个
    static int16  pts_x[256];
    static uint8  pts_y[256];
    int  cnt = 0;

    for(int y = 0; y < height; y++)
    {
        if(edge[y] >= 0 && cnt < 256)
        {
            pts_x[cnt] = edge[y];
            pts_y[cnt] = (uint8)y;
            cnt++;
        }
    }
    if(cnt == 0) return;

    uint8 frame[8 + 2 * 256];   // data_type=0：每点2字节
    frame[0] = 0xAA;
    frame[1] = 0x00;                       // checksum 占位
    frame[2] = 0x03;                       // cmd = boundary
    frame[3] = 0x00;                       // data_type = 0 (1B/pt)
    frame[4] = cnt & 0xFF;                 // count LE16
    frame[5] = (cnt >> 8) & 0xFF;
    frame[6] = color & 0xFF;               // color LE16
    frame[7] = (color >> 8) & 0xFF;

    int idx = 8;
    for(int i = 0; i < cnt; i++)
    {
        frame[idx++] = (uint8)(pts_x[i]);  // x 低8位
        frame[idx++] = pts_y[i];           // y
    }

    frame[1] = (0xAA + frame[2] + frame[3] + frame[4] + frame[5]
                + frame[6] + frame[7]) & 0xFF;    // 校验和

    send(sock_fd, frame, 8 + 2 * cnt, 0);
}

void net_send_edges(const int16 *left_edge, const int16 *right_edge, int height)
{
    net_send_edge_color(left_edge, height, 0, 0xF800);   // 左线红
    net_send_edge_color(right_edge, height, 1, 0x001F);  // 右线蓝
}

// 原始边线虚线通道：观察去毛刺前的毛刺
// 左右用不同颜色，避免上位机同色覆盖：左=黄(0xFFE0)，右=橙黄(0xFDC0)，都画虚线
void net_send_edges_raw(const int16 *left_edge, const int16 *right_edge, int height)
{
    net_send_edge_color(left_edge, height, 0, 0xFFE0);   // 左线黄虚线
    net_send_edge_color(right_edge, height, 1, 0xFDC0);  // 右线橙黄虚线
}

static void net_send_trace_color(const int16 (*trace)[2], int16 trace_num, uint16 color)
{
    if(sock_fd < 0) return;

    uint8 frame[8 + 2 * 256];
    int cnt = 0;
    for(int i = 0; i < trace_num && cnt < 256; i++)
    {
        int16 x = trace[i][0];
        int16 y = trace[i][1];
        if(x < 0 || x >= UVC_WIDTH || y < 0 || y >= UVC_HEIGHT) continue;
        frame[8 + 2 * cnt] = (uint8)x;
        frame[8 + 2 * cnt + 1] = (uint8)y;
        cnt++;
    }
    frame[0] = 0xAA;
    frame[2] = 0x03;
    frame[3] = 0x00;
    frame[4] = cnt & 0xFF;
    frame[5] = (cnt >> 8) & 0xFF;
    frame[6] = color & 0xFF;
    frame[7] = (color >> 8) & 0xFF;
    frame[1] = (0xAA + frame[2] + frame[3] + frame[4] + frame[5]
                + frame[6] + frame[7]) & 0xFF;
    send(sock_fd, frame, 8 + 2 * cnt, 0);
}

void net_send_edges_raw_trace(const int16 (*left_trace)[2], int16 left_num,
                              const int16 (*right_trace)[2], int16 right_num)
{
    net_send_trace_color(left_trace, left_num, 0xFFE0);
    net_send_trace_color(right_trace, right_num, 0xFDC0);
}

static int net_send_bird_trace_color(const int16 (*trace)[2], int16 trace_num, uint16 color)
{
    if(sock_fd < 0) return 0;

    uint8 frame[8 + 2 * 256];
    int cnt = 0;
    for(int i = 0; i < trace_num && cnt < 256; i++)
    {
        int16 bx, by;
        ipm_map_point(trace[i][0], trace[i][1], &bx, &by);
        if(bx < 0 || bx >= IPM_BIRD_W || by < 0 || by >= IPM_BIRD_H) continue;
        frame[8 + 2 * cnt] = (uint8)bx;
        frame[8 + 2 * cnt + 1] = (uint8)by;
        cnt++;
    }
    frame[0] = 0xAA;
    frame[2] = 0x03;
    frame[3] = 0x00;
    frame[4] = cnt & 0xFF;
    frame[5] = (cnt >> 8) & 0xFF;
    frame[6] = color & 0xFF;
    frame[7] = (color >> 8) & 0xFF;
    frame[1] = (0xAA + frame[2] + frame[3] + frame[4] + frame[5]
                + frame[6] + frame[7]) & 0xFF;
    send(sock_fd, frame, 8 + 2 * cnt, 0);
    return cnt;
}

void net_send_edges_bird_trace(const int16 (*near_left)[2], int16 near_left_num,
                               const int16 (*near_right)[2], int16 near_right_num,
                               const int16 (*far_left)[2], int16 far_left_num,
                               const int16 (*far_right)[2], int16 far_right_num)
{
    int nl = net_send_bird_trace_color(near_left, near_left_num, 0xF800);
    int nr = net_send_bird_trace_color(near_right, near_right_num, 0x001F);
    int fl = net_send_bird_trace_color(far_left, far_left_num, 0xFFFF);
    int fr = net_send_bird_trace_color(far_right, far_right_num, 0x07FF);
    printf("[bird] NL=%d NR=%d FL=%d FR=%d\n", nl, nr, fl, fr);
}

static void net_send_bird_float_color(const float (*points)[2], int16 point_num, uint16 color)
{
    if(sock_fd < 0) return;

    uint8 frame[8 + 2 * 256];
    int cnt = 0;
    for(int i = 0; i < point_num && cnt < 256; i++)
    {
        int16 x = (int16)(points[i][0] + 0.5f);
        int16 y = (int16)(points[i][1] + 0.5f);
        if(x < 0 || x >= IPM_BIRD_W || y < 0 || y >= IPM_BIRD_H) continue;
        frame[8 + 2 * cnt] = (uint8)x;
        frame[8 + 2 * cnt + 1] = (uint8)y;
        cnt++;
    }
    // 即使 cnt=0 也发送空帧，让上位机清除上一帧的同色重采样点。
    frame[0] = 0xAA;
    frame[2] = 0x03;
    frame[3] = 0x00;
    frame[4] = cnt & 0xFF;
    frame[5] = (cnt >> 8) & 0xFF;
    frame[6] = color & 0xFF;
    frame[7] = (color >> 8) & 0xFF;
    frame[1] = (0xAA + frame[2] + frame[3] + frame[4] + frame[5]
                + frame[6] + frame[7]) & 0xFF;
    send(sock_fd, frame, 8 + 2 * cnt, 0);
}

void net_send_debug_resampled_bird(const float (*near_left)[2], int16 near_left_num,
                                   const float (*near_right)[2], int16 near_right_num,
                                   const float (*far_left)[2], int16 far_left_num,
                                   const float (*far_right)[2], int16 far_right_num)
{
    // 使用独立色值作为上位机 overlay 缓存键，视觉颜色仍与上方原图一致。
    net_send_bird_float_color(near_left, near_left_num, 0xFFE1);   // 黄
    net_send_bird_float_color(near_right, near_right_num, 0xFDC1); // 橙
    net_send_bird_float_color(far_left, far_left_num, 0xA83F);     // 紫
    net_send_bird_float_color(far_right, far_right_num, 0x05DF);   // 浅蓝
}

void net_send_guidance_points(const float aim_point[2], uint8 aim_valid)
{
    // 独立协议色：洋红=前瞻点，浅灰白=车头基准；避免与远端左线白色冲突。
    static const float car_anchor[1][2] = {{80.0f, 100.0f}};
    if(aim_valid) net_send_bird_float_color((const float (*)[2])aim_point, 1, 0xF81F);
    else net_send_bird_float_color(NULL, 0, 0xF81F);
    net_send_bird_float_color(car_anchor, 1, 0xBDF7);
}

// 原图按行边线 → 鸟瞰坐标 → boundary 帧（供第三块显示）
// 非 static：避免 -Werror=unused-function 误报（定义未使用警告只针对 static）
void net_send_bird_edge_color(const int16 *edge, uint16 color)
{
    if(sock_fd < 0) return;

    static int16  pts_x[256];
    static uint8  pts_y[256];
    int  cnt = 0;

    for(int y = 0; y < UVC_HEIGHT && cnt < 256; y++)
    {
        if(edge[y] < 0) continue;
        int16 bx, by;
        ipm_map_point(edge[y], y, &bx, &by);   // 原图 → 鸟瞰
        if(bx < 0 || bx >= IPM_BIRD_W || by < 0 || by >= IPM_BIRD_H) continue;
        pts_x[cnt] = bx;
        pts_y[cnt] = (uint8)by;
        cnt++;
    }
    if(cnt == 0) return;

    uint8 frame[8 + 2 * 256];
    frame[0] = 0xAA;
    frame[1] = 0x00;
    frame[2] = 0x03;
    frame[3] = 0x00;
    frame[4] = cnt & 0xFF;
    frame[5] = (cnt >> 8) & 0xFF;
    frame[6] = color & 0xFF;
    frame[7] = (color >> 8) & 0xFF;
    int idx = 8;
    for(int i = 0; i < cnt; i++)
    {
        frame[idx++] = (uint8)(pts_x[i]);
        frame[idx++] = pts_y[i];
    }
    frame[1] = (0xAA + frame[2] + frame[3] + frame[4] + frame[5]
                + frame[6] + frame[7]) & 0xFF;
    send(sock_fd, frame, 8 + 2 * cnt, 0);
}

// 远端完整轨迹按原图坐标发送，保留直角横向点：左紫、右浅蓝。
void net_send_far_edges_raw_trace(const int16 (*left_trace)[2], int16 left_num,
                                  const int16 (*right_trace)[2], int16 right_num)
{
    printf("[far] Ltrace=%d Rtrace=%d\n", left_num, right_num);
    net_send_trace_color(left_trace, left_num, 0xA81F);
    net_send_trace_color(right_trace, right_num, 0x05FF);
}

// 发送边线→鸟瞰数组（edge_array_change 输出，鸟瞰像素坐标 160x120）
// 复用 boundary 帧格式（data_type=0, 每点2字节），颜色沿用红蓝处理线通道
// 左=0xF800 红 / 右=0x001F 蓝：上位机中图画红蓝点云（鸟瞰坐标系）
void net_send_resampled(const float (*left_pts)[2], int16 left_n,
                        const float (*right_pts)[2], int16 right_n)
{
    // 左线（鸟瞰坐标原样发送）
    int cnt = 0;
    static int16  pts_x[256];
    static uint8  pts_y[256];
    for(int i = 0; i < left_n && cnt < 256; i++)
    {
        int16 px = (int16)(left_pts[i][0]);
        int16 py = (int16)(left_pts[i][1]);
        if(px < 0 || px >= IPM_BIRD_W || py < 0 || py >= IPM_BIRD_H) continue;
        pts_x[cnt] = px;
        pts_y[cnt] = (uint8)py;
        cnt++;
    }
    if(cnt > 0)
    {
        uint8 frame[8 + 2 * 256];
        frame[0] = 0xAA;
        frame[1] = 0x00;
        frame[2] = 0x03;
        frame[3] = 0x00;
        frame[4] = cnt & 0xFF;
        frame[5] = (cnt >> 8) & 0xFF;
        frame[6] = 0xF800 & 0xFF;
        frame[7] = (0xF800 >> 8) & 0xFF;
        int idx = 8;
        for(int i = 0; i < cnt; i++)
        {
            frame[idx++] = (uint8)(pts_x[i]);
            frame[idx++] = pts_y[i];
        }
        frame[1] = (0xAA + frame[2] + frame[3] + frame[4] + frame[5]
                    + frame[6] + frame[7]) & 0xFF;
        send(sock_fd, frame, 8 + 2 * cnt, 0);
    }

    // 右线（鸟瞰坐标原样发送）
    cnt = 0;
    for(int i = 0; i < right_n && cnt < 256; i++)
    {
        int16 px = (int16)(right_pts[i][0]);
        int16 py = (int16)(right_pts[i][1]);
        if(px < 0 || px >= IPM_BIRD_W || py < 0 || py >= IPM_BIRD_H) continue;
        pts_x[cnt] = px;
        pts_y[cnt] = (uint8)py;
        cnt++;
    }
    if(cnt > 0)
    {
        uint8 frame[8 + 2 * 256];
        frame[0] = 0xAA;
        frame[1] = 0x00;
        frame[2] = 0x03;
        frame[3] = 0x00;
        frame[4] = cnt & 0xFF;
        frame[5] = (cnt >> 8) & 0xFF;
        frame[6] = 0x001F & 0xFF;
        frame[7] = (0x001F >> 8) & 0xFF;
        int idx = 8;
        for(int i = 0; i < cnt; i++)
        {
            frame[idx++] = (uint8)(pts_x[i]);
            frame[idx++] = pts_y[i];
        }
        frame[1] = (0xAA + frame[2] + frame[3] + frame[4] + frame[5]
                    + frame[6] + frame[7]) & 0xFF;
        send(sock_fd, frame, 8 + 2 * cnt, 0);
    }
}

// 发送中线：鸟瞰坐标（绿 0x07E0，第三块显示）+ 经 invx/invy 反查原图坐标（紫 0xFD20，第一块显示）
// 复用 boundary 帧格式（cmd=0x03, data_type=0, 每点2字节）；middle_line_num<=0 不发
void net_send_middle_line(void)
{
    if(sock_fd < 0 || middle_line_num <= 0) return;

    static int16 pts_x[256];
    static uint8 pts_y[256];

    // ① 鸟瞰中线：middle_line 已是鸟瞰像素坐标，原样发送（第三块画）
    int cnt = 0;
    for(int i = 0; i < middle_line_num && cnt < 256; i++)
    {
        int16 px = (int16)(middle_line[i][0]);
        int16 py = (int16)(middle_line[i][1]);
        if(px < 0 || px >= IPM_BIRD_W || py < 0 || py >= IPM_BIRD_H) continue;
        pts_x[cnt] = px;
        pts_y[cnt] = (uint8)py;
        cnt++;
    }
    if(cnt > 0)
    {
        uint8 frame[8 + 2 * 256];
        frame[0] = 0xAA;
        frame[1] = 0x00;
        frame[2] = 0x03;
        frame[3] = 0x00;
        frame[4] = cnt & 0xFF;
        frame[5] = (cnt >> 8) & 0xFF;
        frame[6] = 0x07E0 & 0xFF;      // 绿
        frame[7] = (0x07E0 >> 8) & 0xFF;
        int idx = 8;
        for(int i = 0; i < cnt; i++)
        {
            frame[idx++] = (uint8)(pts_x[i]);
            frame[idx++] = pts_y[i];
        }
        frame[1] = (0xAA + frame[2] + frame[3] + frame[4] + frame[5]
                    + frame[6] + frame[7]) & 0xFF;
        send(sock_fd, frame, 8 + 2 * cnt, 0);
    }

    // ② 原图中线：直接发送 find_middle_line() 已保存的近端反查数组（第一块画）。
    cnt = 0;
    for(int i = 0; i < middle_line_original_num && cnt < 256; i++)
    {
        int16 sx = middle_line_original[i][0];
        int16 sy = middle_line_original[i][1];
        if(sx < 0 || sx >= UVC_WIDTH || sy < 0 || sy >= UVC_HEIGHT) continue;
        pts_x[cnt] = sx;
        pts_y[cnt] = (uint8)sy;
        cnt++;
    }
    if(cnt > 0)
    {
        uint8 frame[8 + 2 * 256];
        frame[0] = 0xAA;
        frame[1] = 0x00;
        frame[2] = 0x03;
        frame[3] = 0x00;
        frame[4] = cnt & 0xFF;
        frame[5] = (cnt >> 8) & 0xFF;
        frame[6] = 0xFD20 & 0xFF;      // 紫
        frame[7] = (0xFD20 >> 8) & 0xFF;
        int idx = 8;
        for(int i = 0; i < cnt; i++)
        {
            frame[idx++] = (uint8)(pts_x[i]);
            frame[idx++] = pts_y[i];
        }
        frame[1] = (0xAA + frame[2] + frame[3] + frame[4] + frame[5]
                    + frame[6] + frame[7]) & 0xFF;
        send(sock_fd, frame, 8 + 2 * cnt, 0);
    }
}

void debug_image(void)
{


}
void debug_data(void)
{
    

}

// 发送边线丢线统计帧（$LOSS 文本帧）
// 格式：$LOSS lv ls lseg0..5 rv rs rseg0..5
//   lv/rv=有效点数 ls/rs=丢线段数
//   每段 3 个数：起始行 结束行(含) 长度；不足 6 段用 -1 补齐
// 示例：$LOSS 79 2 0 39 40 88 5 8 -1 -1 -1 -1 -1 74 2 0 44 45 ...
void net_send_loss(void)
{
    if(sock_fd < 0) return;

    int16 lv, ls, rv, rs;
    int16 lseg[6][3], rseg[6][3];
    edge_loss_stats(left_edge,  &lv, &ls, lseg);
    edge_loss_stats(right_edge, &rv, &rs, rseg);

    char los[160];
    char *p = los;
    p += sprintf(p, "$LOSS %d %d", lv, ls);
    for(int i = 0; i < 6; i++)
    {
        if(i < ls && lseg[i][0] >= 0)
            p += sprintf(p, " %d %d %d", lseg[i][0], lseg[i][1], lseg[i][2]);
        else
            p += sprintf(p, " -1 -1 -1");
    }
    p += sprintf(p, " %d %d", rv, rs);
    for(int i = 0; i < 6; i++)
    {
        if(i < rs && rseg[i][0] >= 0)
            p += sprintf(p, " %d %d %d", rseg[i][0], rseg[i][1], rseg[i][2]);
        else
            p += sprintf(p, " -1 -1 -1");
    }
    net_send_line(los);
}
