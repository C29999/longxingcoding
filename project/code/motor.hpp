#ifndef __MOTOR_hpp__
#define __MOTOR_hpp__

#include "zf_driver_gpio.hpp"
#include "zf_driver_pwm.hpp"

// 电机方向引脚（推挽输出）：P73=电机1方向, P76=电机2方向
extern zf_driver_gpio motor_dir_1;
extern zf_driver_gpio motor_dir_2;

// 电机 PWM：P86=电机1, P87=电机2
extern zf_driver_pwm motor_pwm_1;
extern zf_driver_pwm motor_pwm_2;

void motor_init(void);                     // 初始化方向引脚 + PWM 占空比 0
void motor_pwm_out(int16 LPWM, int16 RPWM);// 左右电机独立控制：正=正转 负=反转 0=停（范围-10000~10000）

#endif
