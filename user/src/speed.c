/**
 * @brief 计算电机速度（带移动平均滤波）
 * @note 需要在定时中断中调用，建议10ms调用一次
 */

#include "main.h"
#include "speed.h"

// 速度滤波缓冲区
#define SPEED_FILTER_SIZE 5
float speed_left_buffer[SPEED_FILTER_SIZE] = {0};
float speed_right_buffer[SPEED_FILTER_SIZE] = {0};
uint8 speed_buffer_index = 0;

/**
 * 将编码器脉冲增量（delta）转换为速度并做移动平均滤波。
 * 注意：本函数不负责读取或清除编码器计数器，调用方应负责。
 */
void calculate_motor_speed(int16 delta_left, int16 delta_right, float *out_speed_left, float *out_speed_right)
{
    // 计算原始速度（cm/s）
    // 速度 = (Δ脉冲数 / 脉冲每圈) * 轮子周长 / 采样时间
    float raw_speed_left = (float)delta_left * (WHEEL_CIRCUMFERENCE / (float)ENCODER_PULSES_PER_REV) / (float)SPEED_SAMPLE_TIME;
    float raw_speed_right = (float)delta_right * (WHEEL_CIRCUMFERENCE / (float)ENCODER_PULSES_PER_REV) / (float)SPEED_SAMPLE_TIME;

    // 移动平均滤波
    speed_left_buffer[speed_buffer_index] = raw_speed_left;
    speed_right_buffer[speed_buffer_index] = raw_speed_right;
    speed_buffer_index = (speed_buffer_index + 1) % SPEED_FILTER_SIZE;

    float sum_l = 0.0f, sum_r = 0.0f;
    for (int i = 0; i < SPEED_FILTER_SIZE; i++) {
        sum_l += speed_left_buffer[i];
        sum_r += speed_right_buffer[i];
    }
    float filt_l = sum_l / SPEED_FILTER_SIZE;
    float filt_r = sum_r / SPEED_FILTER_SIZE;

    if (out_speed_left) *out_speed_left = filt_l;
    if (out_speed_right) *out_speed_right = filt_r;
}