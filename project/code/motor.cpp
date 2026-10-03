#include "motor.hpp"
#include <cstdlib>   // abs()

// ============ 电机对象 ============
// 方向引脚（推挽输出）：P73=电机1, P76=电机2
zf_driver_gpio motor_dir_1(ZF_GPIO_MOTOR_1);
zf_driver_gpio motor_dir_2(ZF_GPIO_MOTOR_2);

// PWM 输出：P86=电机1, P87=电机2
zf_driver_pwm  motor_pwm_1(ZF_PWM_MOTOR_1);
zf_driver_pwm  motor_pwm_2(ZF_PWM_MOTOR_2);

void motor_control(void)
{

    
}
// ============ 初始化 ============
// 方向引脚置低电平（推挽输出），PWM 占空比 0（电机不转）
void motor_init(void)
{
    motor_dir_1.set_level(0);
    motor_dir_2.set_level(0);

    motor_pwm_1.set_duty(0);
    motor_pwm_2.set_duty(0);
}

// ============ 左右电机独立输出 ============
// LPWM: 左电机（电机1），RPWM: 右电机（电机2）
// 范围 -10000 ~ +10000（对应 duty_max=10000）
//   正 = 正转：方向引脚高电平，PWM = 绝对值
//   负 = 反转：方向引脚低电平，PWM = 绝对值
//   0  = 停止：PWM = 0
void motor_pwm_out(int16 LPWM, int16 RPWM)
{
    if(LPWM >  10000) LPWM =  10000;
    if(LPWM < -10000) LPWM = -10000;
    if(RPWM >  10000) RPWM =  10000;
    if(RPWM < -10000) RPWM = -10000;

    // ----- 左电机（电机1）P73 + P86 -----
    if(LPWM >= 0)
    {
        motor_dir_1.set_level(1);
        motor_pwm_1.set_duty((uint16)LPWM);
    }
    else
    {
        motor_dir_1.set_level(0);
        motor_pwm_1.set_duty((uint16)(-LPWM));
    }

    // ----- 右电机（电机2）P76 + P87 -----
    if(RPWM >= 0)
    {
        motor_dir_2.set_level(1);
        motor_pwm_2.set_duty((uint16)RPWM);
    }
    else
    {
        motor_dir_2.set_level(0);
        motor_pwm_2.set_duty((uint16)(-RPWM));
    }
}
