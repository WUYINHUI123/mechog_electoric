#ifndef __BMI088_H
#define __BMI088_H

#include "main.h"

// 传感器数据结构体 (兼容之前的 PID 和 运动学代码)
typedef struct {
    float pitch;        // 滤波后的俯仰角
    float roll;         // 滤波后的翻滚角
    float yaw;          // 水平航向角
    float gyro_x;       // X轴实时角速度
    float gyro_y;       // Y轴实时角速度
    float gyro_z;       // Z轴实时角速度
    float temperature;  // 芯片内部实时温度 (用于加热器)
} BMI088_Data_t;

extern BMI088_Data_t imu_data;

// 函数声明
void BMI088_Init(SPI_HandleTypeDef *hspi);
void BMI088_Update(SPI_HandleTypeDef *hspi);
void IMU_Heater_Ctrl(void); // 恒温加热器控制任务

#endif