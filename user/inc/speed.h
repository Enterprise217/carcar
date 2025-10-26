
#ifndef _SPEED_H
#define _SPEED_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 将编码器增量（delta pulses）转换为速度并做移动平均滤波。
 * 说明：为了职责清晰，函数不再读取或清零编码器计数。调用方（如 PIT IRQ）应负责：
 *    1) 读取本次时间窗内的编码器计数（增量）
 *    2) 立即清零计数器（避免丢失下一周期数据）
 * 然后将增量传入本函数。单位：输出速度为 cm/s。
 *
 * Parameters:
 *   delta_left  - 左轮在采样周期内的脉冲增量（可以为负）
 *   delta_right - 右轮在采样周期内的脉冲增量（可以为负）
 *   out_speed_left/out_speed_right - 指向接收滤波后速度的指针（可为 NULL）
 */
void calculate_motor_speed(int16 delta_left, int16 delta_right, float *out_speed_left, float *out_speed_right);

#ifdef __cplusplus
}
#endif

#endif // _SPEED_H