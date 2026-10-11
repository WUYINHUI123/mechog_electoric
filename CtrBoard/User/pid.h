#ifndef __PID_H
#define __PID_H

// 串级PID控制器结构体
typedef struct {
    float kp_angle;   // 外环：角度P（偏差角度 -> 目标角速度）
    float ki_gyro;    // 内环：角速度I（累加角速度误差 -> 高度补偿）
    float max_out;    // 限幅：最大补偿高度(mm)
    float out;        // 当前积分输出高度
} CascadePID_t;

// 函数声明
void CascadePID_Init(CascadePID_t *pid, float kp_a, float ki_g, float max_out);
float CascadePID_Calc(CascadePID_t *pid, float current_angle, float current_gyro);
float LowPassFilter(float current_val, float prev_val, float alpha);

#endif










































//#ifndef __PID_H__
//#define __PID_H__

//#include "main.h"

//typedef struct
//{
//	volatile float kp;						// 比例
//	volatile float ki;						// 积分
//	volatile float kd;						// 微分
//	
//	volatile float p_term;				// 比例项
//	volatile float i_term;				// 积分项
//	volatile float d_term;				// 微分项
//	
//	volatile float i_term_max;		// 积分累加限幅
//	volatile float i_term_min;		// 积分累加限幅
//	
//	volatile float ctrl_period;		//控制周期
//	
//	volatile float ref_value;			// 目标值
//	volatile float fback_value;		// 实际值
//	
//	volatile float error;					// 误差
//	volatile float pre_err;				// 上一次误差
//	
//	volatile float out_min;				// 输出限幅
//	volatile float out_max;				// 输出限幅
//	
//	volatile float out_value;
//	
//}pid_para_t;

//void pid_para_init(pid_para_t *pid_config);
//void pid_limit_init(pid_para_t *pid_config, float i_term_max, float i_term_min,float out_max, float out_min);
//void pid_clear(pid_para_t *pid_clear);
//void pid_reset(pid_para_t *pid_config, float kp, float ki, float kd);
//float parallel_pid_ctrl(pid_para_t *pi, float ref_value, float fback_value);
//float serial_pid_ctrl(pid_para_t *pid, float ref_value, float fdback_value);
//float serial_pid_ctrl1(pid_para_t *pid, float ref_value, float fdback_value, float i_max, float out_max);
//#endif /* __PID_H__ */















