#include "main.h"
#include "motor_control.h"

// 注意：此模块是开环 PWM 控制示例，会直接写 PWM 占空比并设置方向。
// 如果你同时启用了 PIT+PID 的自动控制（ISR 中会写 PWM），两者会互相覆盖，
// 请确保只使用一种控制方式（开环演示时可注释掉 pit_ms_init 或在 ISR 中屏蔽写 PWM）。

void motors_set_all_duty(float duty_percent)
{
    // 限幅到 -100 .. 100
    if (duty_percent > 100.0f) duty_percent = 100.0f;
    if (duty_percent < -100.0f) duty_percent = -100.0f;

    // 计算底层占空比刻度（PWM_DUTY_MAX 为 SDK 提供的最大刻度）
    uint32_t duty_abs = (uint32_t)((float)PWM_DUTY_MAX * (fabsf(duty_percent) / 100.0f));

    if (duty_percent >= 0.0f)
    {
        gpio_set_level(MOTOR1_DIR, MOTOR1_FORWARD_DIR_LEVEL);
        pwm_set_duty(MOTOR1_PWM, duty_abs);

        gpio_set_level(MOTOR2_DIR, MOTOR2_FORWARD_DIR_LEVEL);
        pwm_set_duty(MOTOR2_PWM, duty_abs);

        /* 如果板子有 MOTOR3/MOTOR4，可按需添加宏并设置 */
#ifdef MOTOR3_PWM
        gpio_set_level(MOTOR3_DIR, GPIO_HIGH);
        pwm_set_duty(MOTOR3_PWM, duty_abs);
#endif
#ifdef MOTOR4_PWM
        gpio_set_level(MOTOR4_DIR, GPIO_HIGH);
        pwm_set_duty(MOTOR4_PWM, duty_abs);
#endif
    }
    else
    {
        gpio_set_level(MOTOR1_DIR, !MOTOR1_FORWARD_DIR_LEVEL);
        pwm_set_duty(MOTOR1_PWM, duty_abs);

        gpio_set_level(MOTOR2_DIR, !MOTOR2_FORWARD_DIR_LEVEL);
        pwm_set_duty(MOTOR2_PWM, duty_abs);

#ifdef MOTOR3_PWM
        gpio_set_level(MOTOR3_DIR, GPIO_LOW);
        pwm_set_duty(MOTOR3_PWM, duty_abs);
#endif
#ifdef MOTOR4_PWM
        gpio_set_level(MOTOR4_DIR, GPIO_LOW);
        pwm_set_duty(MOTOR4_PWM, duty_abs);
#endif
    }
}

void motors_stop_all(void)
{
    pwm_set_duty(MOTOR1_PWM, 0);
    pwm_set_duty(MOTOR2_PWM, 0);
#ifdef MOTOR3_PWM
    pwm_set_duty(MOTOR3_PWM, 0);
#endif
#ifdef MOTOR4_PWM
    pwm_set_duty(MOTOR4_PWM, 0);
#endif
}
