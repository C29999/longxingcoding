#ifndef __init_hpp__
#define __init_hpp__

#include "zf_common_font.hpp"         // RGB565 颜色枚举
#include "zf_driver_delay.hpp"        // system_delay_ms 宏
#include "zf_device_ips200_fb.hpp"    // IPS200 屏幕驱动
#include "zf_device_uvc.hpp"          // UVC 摄像头驱动
#include "zf_driver_pit.hpp"          // PIT 周期定时器（1秒中断）
#include "unless_debug.hpp"           // 无调试输出函数
#include "display.hpp"                // 显示函数
#include "motor.hpp"                  // 电机输出
#include "zf_driver_gpio.hpp"         // GPIO 引脚
#include "zf_driver_pwm.hpp"         // PWM 输出
#include "image.hpp"
#include "ipm.hpp"
// 全局设备对象声明（定义在 main.cpp）
extern zf_device_ips200 ips200;
extern zf_device_uvc    uvc_dev;

// FPS 统计全局变量（定义在 init.cpp，中断与主循环共享）
extern zf_driver_pit   pit_timer;     // 1秒周期定时器
extern volatile uint32 frame_count;   // 主循环计次（每秒清零）
extern volatile uint32 fps;           // 上一秒的帧数
extern volatile uint8  fps_updated;   // fps 已更新标志（中断置1）

// 发车按键（定义在 init.cpp，P77 = ZF_GPIO_KEY_4）
extern uint8 key_start_flag;          // 发车标志：1=已发车

void init_all(void);
void redy_go(void);
#endif
