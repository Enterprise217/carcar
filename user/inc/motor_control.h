#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

// 设置所有电机的占空比（百分比 -100..100）
// 正值为前进方向（MOTORx_FORWARD_DIR_LEVEL），负值为反向
void motors_set_all_duty(float duty_percent);

// 停止所有电机（占空比=0）
void motors_stop_all(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H__ */
