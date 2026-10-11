#include "pid.h"
// 初始化PID参数
void CascadePID_Init(CascadePID_t *pid, float kp_a, float ki_g, float max_out) {
    pid->kp_angle = kp_a;
    pid->ki_gyro  = ki_g;
    pid->max_out  = max_out;
    pid->out      = 0.0f;
}

// 串级PID核心计算 (完美复刻ESP32的逻辑)
float CascadePID_Calc(CascadePID_t *pid, float current_angle, float current_gyro) {
    // 1. P环：算出要想回到水平(0度)，你现在必须以多快的角速度转动
    float target_gyro = (0.0f - current_angle) * pid->kp_angle;
    
    // 2. I环：计算目标角速度与实际角速度的误差，并积分累加成物理高度
    float gyro_error = target_gyro - current_gyro;
    pid->out += gyro_error * pid->ki_gyro;
    
    // 3. 限幅保护：防止补偿高度过大导致翻车
    if (pid->out > pid->max_out)  pid->out = pid->max_out;
    if (pid->out < -pid->max_out) pid->out = -pid->max_out;
    
    return pid->out;
}

// 一阶低通滤波器：滤除高频震动毛刺
float LowPassFilter(float current_val, float prev_val, float alpha) {
    return alpha * current_val + (1.0f - alpha) * prev_val;
}


