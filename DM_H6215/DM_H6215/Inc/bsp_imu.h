#ifndef BSP_IMU_H
#define BSP_IMU_H

#include "main.h"
#include "spi.h"
#include "gpio.h"

// ========== BMI088 ���Ŷ��壨��������ʵ��ԭ��ͼ�޸ģ� ==========
// ���� CS_ACC �� CS_GYR ���ӵ��� PE10 �� PE12
// ���ʵ�ʲ�ͬ�����޸�����ĺ궨��
#define IMU_CS_ACC_GPIO_Port   GPIOC
#define IMU_CS_ACC_Pin         GPIO_PIN_0

// ������Ƭѡ��CSB2 �� PC3
#define IMU_CS_GYR_GPIO_Port   GPIOC
#define IMU_CS_GYR_Pin         GPIO_PIN_3
// ========== �������� ==========
/**
 * @brief ��ʼ�� BMI088 IMU
 * @retval 1: �ɹ�, 0: ʧ�ܣ�ID��ȡ����
 */
uint8_t IMU_Init(void);

/**
 * @brief ��ȡ���ٶȼ�ԭʼ���ݣ�16λ�з��ţ�
 */
void IMU_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az);

/**
 * @brief ��ȡ������ԭʼ���ݣ�16λ�з��ţ�
 */
void IMU_ReadGyro(int16_t *gx, int16_t *gy, int16_t *gz);

/**
 * @brief ��ӡ IMU ���ݣ�ͨ�� USB CDC��
 */
void IMU_PrintData(void);

/* ========== Calibration & Data Processing ========== */

/**
 * @brief Run gyro calibration: collect N samples at rest, compute offsets.
 *        Call once after IMU_Init() succeeds, while robot is stationary.
 * @param num_samples Number of samples to collect (recommend 500)
 * @retval 1: success, 0: failed
 */
uint8_t IMU_CalibrateGyro(uint16_t num_samples);

/**
 * @brief Process one IMU reading cycle: read accel+gyro, apply calibration
 *        offsets, low-pass filter, integrate gyro-z for yaw angle,
 *        compute pitch/roll from accelerometer.
 *        Call at your control rate (e.g. 200 Hz = every 5ms).
 */
void IMU_Process(void);

/* ========== Processed Data Accessors ========== */

/** @brief Get calibrated, filtered gyro Z rate in degrees per second. */
float IMU_GetYawRate_dps(void);

/** @brief Get integrated yaw angle in degrees (unwrapped). */
float IMU_GetYawAngle_deg(void);

/** @brief Get pitch angle in degrees (from accelerometer, +nose up). */
float IMU_GetPitch_deg(void);

/** @brief Get roll angle in degrees (from accelerometer, +right down). */
float IMU_GetRoll_deg(void);

/** @brief Get the current accel magnitude in g (for tilt safety check). */
float IMU_GetAccelMagnitude_g(void);

/** @brief Reset yaw angle reference to current heading (for heading hold). */
void IMU_ResetYawReference(void);

/** @brief Print processed IMU data via USB CDC (replaces raw IMU_PrintData). */
void IMU_PrintProcessed(void);

/* ========== Control Functions ========== */

/**
 * @brief Yaw-rate PID controller: computes a differential RPM correction
 *        to add to left/right wheel targets based on yaw rate error.
 *
 * @param target_yaw_rate_dps  Desired yaw rate in deg/s (from RC steer or Pi)
 * @return float  Differential RPM to add to left wheels / subtract from right wheels
 */
float IMU_YawRatePID_Compute(float target_yaw_rate_dps);

/**
 * @brief Heading-hold correction: when target turn is near zero, computes
 *        a small differential RPM to keep yaw angle at the reference.
 *
 * @return float  Differential RPM correction for heading hold.
 */
float IMU_HeadingHold_Compute(void);

/**
 * @brief Check if robot tilt exceeds safe limit.
 * @retval 1: safe, 0: tilt too high -- should stop motors.
 */
uint8_t IMU_IsTiltSafe(void);

/**
 * @brief Set the roll and pitch tilt limits in degrees that trigger safety stop.
 */
void IMU_SetTiltLimits(float max_pitch_deg, float max_roll_deg);

/**
 * @brief Enable or disable IMU closed-loop corrections.
 */
void IMU_SetClosedLoopEnable(uint8_t enable);
uint8_t IMU_IsClosedLoopEnabled(void);

#endif