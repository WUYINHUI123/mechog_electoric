#include "bmiO88.h"
#include <math.h>

BMI088_Data_t imu_data = {0};

static float gyro_offsets[3] = {0, 0, 0};
static uint32_t pre_interval = 0;

static const float accCoef = 0.03f;
static const float gyroCoef = 0.97f;

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

extern TIM_HandleTypeDef htim3; // 引用外部定时器3 (用于加热PWM)

// ==========================================
// 1. SPI 底层读写封装 (处理 BMI088 双片选与 Dummy Byte)
// ==========================================
static void CS_Accel(uint8_t state) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static void CS_Gyro(uint8_t state) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// SPI 写寄存器
static void SPI_Write(SPI_HandleTypeDef *hspi, void(*CS_Func)(uint8_t), uint8_t reg, uint8_t data) {
    uint8_t tx[2] = {reg & 0x7F, data};
    CS_Func(0);
    HAL_SPI_Transmit(hspi, tx, 2, 100);
    CS_Func(1);
}

// SPI 读加速度计 (BMI088 的 Accel 坑：读数据必须先接收一个 Dummy 虚拟字节)
static void SPI_Read_Accel(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *data, uint16_t len) {
    uint8_t tx = reg | 0x80;
    uint8_t dummy;
    CS_Accel(0);
    HAL_SPI_Transmit(hspi, &tx, 1, 100);
    HAL_SPI_Receive(hspi, &dummy, 1, 100); // 必须接收 Dummy
    HAL_SPI_Receive(hspi, data, len, 100);
    CS_Accel(1);
}

// SPI 读陀螺仪 (Gyro 没有 Dummy 字节)
static void SPI_Read_Gyro(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *data, uint16_t len) {
    uint8_t tx = reg | 0x80;
    CS_Gyro(0);
    HAL_SPI_Transmit(hspi, &tx, 1, 100);
    HAL_SPI_Receive(hspi, data, len, 100);
    CS_Gyro(1);
}

// ==========================================
// 2. BMI088 初始化与零偏校准
// ==========================================
void BMI088_Init(SPI_HandleTypeDef *hspi) {
    // 1. 加速度计初始化
    SPI_Write(hspi, CS_Accel, 0x7E, 0xB6); // 软复位
    HAL_Delay(50);
    SPI_Write(hspi, CS_Accel, 0x7D, 0x04); // 开启加速度计电源
    HAL_Delay(50);
    SPI_Write(hspi, CS_Accel, 0x41, 0x01); // 设置量程 +-6g
    SPI_Write(hspi, CS_Accel, 0x40, 0xA8); // 设置 OSR4, 100Hz 滤波
    
    // 2. 陀螺仪初始化
    SPI_Write(hspi, CS_Gyro, 0x14, 0xB6);  // 软复位
    HAL_Delay(50);
    SPI_Write(hspi, CS_Gyro, 0x0F, 0x01);  // 设置量程 +-1000dps
    SPI_Write(hspi, CS_Gyro, 0x10, 0x02);  // 设置带宽 116Hz
    
    HAL_Delay(100); // 等待芯片稳定

    // 3. 极速读取 500 次求零偏 (SPI 速度极快，不需要 delay)
    float sum_x = 0, sum_y = 0, sum_z = 0;
    uint8_t buf[6];
    for (int i = 0; i < 500; i++) {
        SPI_Read_Gyro(hspi, 0x02, buf, 6);
        sum_x += (float)((int16_t)(buf[1] << 8 | buf[0])) / 32.768f;
        sum_y += (float)((int16_t)(buf[3] << 8 | buf[2])) / 32.768f;
        sum_z += (float)((int16_t)(buf[5] << 8 | buf[4])) / 32.768f;
        HAL_Delay(1);
    }
    gyro_offsets[0] = sum_x / 500.0f;
    gyro_offsets[1] = sum_y / 500.0f;
    gyro_offsets[2] = sum_z / 500.0f;
    
    pre_interval = HAL_GetTick();
}

// ==========================================
// 3. 姿态更新算法 (由外部定时轮询调用)
// ==========================================
void BMI088_Update(SPI_HandleTypeDef *hspi) {
    uint8_t acc_buf[6], gyr_buf[6], temp_buf[2];
    
    // 分别读取 加速度、陀螺仪、温度
    SPI_Read_Accel(hspi, 0x12, acc_buf, 6);
    SPI_Read_Gyro(hspi, 0x02, gyr_buf, 6);
    SPI_Read_Accel(hspi, 0x22, temp_buf, 2); // 0x22 和 0x23 是温度寄存器

    // 解析温度数据并更新结构体
    int16_t temp_raw = (temp_buf[0] << 3) | (temp_buf[1] >> 5);
    if(temp_raw > 1023) temp_raw -= 2048;
    imu_data.temperature = temp_raw * 0.125f + 23.0f;

    // 解析加速度与角速度
    int16_t ax = (acc_buf[1] << 8 | acc_buf[0]);
    int16_t ay = (acc_buf[3] << 8 | acc_buf[2]);
    int16_t az = (acc_buf[5] << 8 | acc_buf[4]);
    
    int16_t gx = (gyr_buf[1] << 8 | gyr_buf[0]);
    int16_t gy = (gyr_buf[3] << 8 | gyr_buf[2]);
    int16_t gz = (gyr_buf[5] << 8 | gyr_buf[4]);

    float acc[3], gyro[3], angleAcc[2];
    
    // 单位换算: 量程 +-6g -> 32768/6 = 5461 LSB/g
    acc[0] = (float)ax / 5461.0f;
    acc[1] = (float)ay / 5461.0f;
    acc[2] = (float)az / 5461.0f;

    // 单位换算: 量程 +-1000dps -> 32768/1000 = 32.768 LSB/dps
    gyro[0] = (float)gx / 32.768f - gyro_offsets[0];
    gyro[1] = (float)gy / 32.768f - gyro_offsets[1];
    gyro[2] = (float)gz / 32.768f - gyro_offsets[2];

    imu_data.gyro_x = gyro[0];
    imu_data.gyro_y = gyro[1];
    imu_data.gyro_z = gyro[2];

    float dt = (HAL_GetTick() - pre_interval) * 0.001f;
    pre_interval = HAL_GetTick();
    if (dt > 0.1f) dt = 0.001f; 

    // 加速度求倾角
    angleAcc[0] = atan2f(acc[1], acc[2] + fabsf(acc[0])) * 180.0f / M_PI; 
    angleAcc[1] = atan2f(acc[0], acc[2] + fabsf(acc[1])) * -180.0f / M_PI; 

    // 互补滤波融合
    imu_data.pitch = (gyroCoef * (imu_data.pitch + gyro[0] * dt)) + (accCoef * angleAcc[0]);
    imu_data.roll  = (gyroCoef * (imu_data.roll  + gyro[1] * dt)) + (accCoef * angleAcc[1]);
    imu_data.yaw  += gyro[2] * dt;
}

// ==========================================
// 4. 高级功能：IMU恒温控制 (维持在 45℃)
// ==========================================
void IMU_Heater_Ctrl(void) {
    float target_temp = 45.0f; // 设定恒温目标 45度
    float current_temp = imu_data.temperature;
    
    // 极简温度 P 控制器 (仅举例，可后续加积分I)
    float kp_temp = 800.0f; 
    float output = (target_temp - current_temp) * kp_temp;
    
    // 限制PWM输出范围 (假设定时器周期 ARR 为 10000)
    if(output > 9999) output = 9999;
    if(output < 0) output = 0;
    
    // 输出给 PB01 的 TIM3_CH4
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint32_t)output);
}