#include "init.hpp"
#include "motor.hpp"    // motor_init() 电机初始化

#define UVC_EXPOSURE_VALUE   300

zf_driver_pit   pit_timer;            // 1秒周期定时器对象
zf_driver_gpio key_start(ZF_GPIO_KEY_4);    // P77 发车按键
volatile uint32 frame_count  = 0;     // 主循环计次
volatile uint32 fps          = 0;     // 上一秒帧数
volatile uint8  fps_updated  = 0;     // fps 更新标志

uint8  key_start_flag = 0;     // 发车按键标志
void pit_callback(void)
{
    fps          = frame_count;       // 本秒帧数存入 fps
    frame_count  = 0;                 // 清零，开始下一秒计数
    fps_updated  = 1;                 // 通知主循环刷新屏幕
}

// ============ 统一初始化 ============
void init_all(void)
{
    ips200.init(FB_PATH);             // 初始化屏幕
    uvc_dev.init(UVC_PATH);           // 初始化摄像头
    uvc_dev.set_auto_exposure(UVC_AUTO_EXPOSURE_DISABLE);  // 1=关闭自动曝光（手动模式）
    uvc_dev.set_exposure_value(UVC_EXPOSURE_VALUE);        // 固定曝光值
    motor_init();
    // 启动 1 秒周期中断（1000ms），回调 pit_callback
    pit_timer.init_ms(1000, pit_callback);
}
void redy_go(void)
{
    if(key_start.get_level() == 0)              
    {
        system_delay_ms(10);                    
        if(key_start.get_level() == 0)          
        {
            printf("发车!\n");
            while(key_start.get_level() == 0);  
            key_start_flag = 1;                 
        }
    }
}