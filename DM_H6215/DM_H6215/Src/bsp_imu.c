#include "bsp_imu.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <math.h>

extern SPI_HandleTypeDef hspi2;

// ========== �ײ� SPI ��д���� ==========

static uint8_t SPI_ReadWriteByte(uint8_t txData)
{
    uint8_t rxData;
    HAL_SPI_TransmitReceive(&hspi2, &txData, &rxData, 1, HAL_MAX_DELAY);
    return rxData;
}

static uint8_t IMU_ReadReg(uint8_t reg, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    uint8_t cmd = reg | 0x80;  // BMI088 ������ = ��ַ | 0x80
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    SPI_ReadWriteByte(cmd);     // ���Ͷ�����
    uint8_t data = SPI_ReadWriteByte(0x00);  // �������ֽڣ���ȡ����
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    return data;
}

static void IMU_WriteReg(uint8_t reg, uint8_t value, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    uint8_t cmd = reg & 0x7F;  // BMI088 д���� = ��ַ & 0x7F
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    SPI_ReadWriteByte(cmd);
    SPI_ReadWriteByte(value);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
}

// ========== BMI088 ��ʼ�� ==========

uint8_t IMU_Init(void)
{
    // 1. ��ȡ���ٶȼ� WHO_AM_I (�Ĵ��� 0x00)������ֵ 0x1E
    uint8_t acc_id = IMU_ReadReg(0x00, IMU_CS_ACC_GPIO_Port, IMU_CS_ACC_Pin);
    // 2. ��ȡ������ WHO_AM_I (�Ĵ��� 0x00)������ֵ 0x0F
    uint8_t gyr_id = IMU_ReadReg(0x00, IMU_CS_GYR_GPIO_Port, IMU_CS_GYR_Pin);

    // ��� ID ��ƥ�䣬���� 0 ��ʾ��ʼ��ʧ��
    if (acc_id != 0x1E || gyr_id != 0x0F) {
        return 0;
    }

    // 3. ���ü��ٶȼƣ����� ��24g (�Ĵ��� 0x41��ֵ 0x03)
    IMU_WriteReg(0x41, 0x03, IMU_CS_ACC_GPIO_Port, IMU_CS_ACC_Pin);
    // ��ѡ�����ü��ٶȼƴ��� (�Ĵ��� 0x40)���ݲ��޸�

    // 4. ���������ǣ����� 2000��/s (�Ĵ��� 0x0F��ֵ 0x00)
    IMU_WriteReg(0x0F, 0x00, IMU_CS_GYR_GPIO_Port, IMU_CS_GYR_Pin);
    // 5. ʹ�������� (�Ĵ��� 0x0C��ֵ 0x0E)
    IMU_WriteReg(0x0C, 0x0E, IMU_CS_GYR_GPIO_Port, IMU_CS_GYR_Pin);

    return 1;  // ��ʼ���ɹ�
}

// ========== ��ȡ���ٶȼ� ==========

void IMU_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6];

    HAL_GPIO_WritePin(IMU_CS_ACC_GPIO_Port, IMU_CS_ACC_Pin, GPIO_PIN_RESET);
    SPI_ReadWriteByte(0x12 | 0x80);  // ������ӼĴ��� 0x12 ��ʼ
    for (int i = 0; i < 6; i++) {
        buf[i] = SPI_ReadWriteByte(0x00);
    }
    HAL_GPIO_WritePin(IMU_CS_ACC_GPIO_Port, IMU_CS_ACC_Pin, GPIO_PIN_SET);

    // ���ݸ�ʽ��LSB ��ǰ��MSB �ں�
    *ax = (int16_t)((buf[1] << 8) | buf[0]);
    *ay = (int16_t)((buf[3] << 8) | buf[2]);
    *az = (int16_t)((buf[5] << 8) | buf[4]);
}

// ========== ��ȡ������ ==========

void IMU_ReadGyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];

    HAL_GPIO_WritePin(IMU_CS_GYR_GPIO_Port, IMU_CS_GYR_Pin, GPIO_PIN_RESET);
    SPI_ReadWriteByte(0x02 | 0x80);  // ������ӼĴ��� 0x02 ��ʼ
    for (int i = 0; i < 6; i++) {
        buf[i] = SPI_ReadWriteByte(0x00);
    }
    HAL_GPIO_WritePin(IMU_CS_GYR_GPIO_Port, IMU_CS_GYR_Pin, GPIO_PIN_SET);

    *gx = (int16_t)((buf[1] << 8) | buf[0]);
    *gy = (int16_t)((buf[3] << 8) | buf[2]);
    *gz = (int16_t)((buf[5] << 8) | buf[4]);
}

// ========== ��ӡ IMU ���ݣ�ͨ�� USB CDC�� ==========

void IMU_PrintData(void)
{
    int16_t ax, ay, az, gx, gy, gz;

    IMU_ReadAccel(&ax, &ay, &az);
    IMU_ReadGyro(&gx, &gy, &gz);

    char msg[128];
    int len = snprintf(msg, sizeof(msg),
                       "ACC: %6d %6d %6d   GYR: %6d %6d %6d\r\n",
                       ax, ay, az, gx, gy, gz);

    if (len > 0) {
        CDC_Transmit_HS((uint8_t *)msg, (uint16_t)len);
    }
}

/* ========================================================================== */
/*                    Closed-Loop IMU Control Module                          */
/* ========================================================================== */

/* ========== BMI088 Conversion Factors (per datasheet) ========== */
/* Gyro: 2000 dps range => sensitivity 16.384 LSB/(deg/s) */
#define GYRO_SENSITIVITY_LSB_PER_DPS  16.384f
/* Accel: +/-24g range => sensitivity 1365 LSB/g */
#define ACCEL_SENSITIVITY_LSB_PER_G   1365.0f

/* ========== Low-Pass Filter ========== */
#define IMU_LPF_ALPHA                 0.15f

/* ========== Gyro Calibration ========== */
#define IMU_CALIBRATION_SAMPLES       500U

/* ========== Gyro Drift Deadband ========== */
#define IMU_YAW_DRIFT_DEADBAND_DPS    0.5f

/* ========== Tilt Safety Defaults ========== */
#define IMU_TILT_MAX_PITCH_DEG        45.0f
#define IMU_TILT_MAX_ROLL_DEG         45.0f

/* ========== Yaw-Rate PID Gains (tunable) ========== */
#define YAW_PID_KP                    0.8f
#define YAW_PID_KI                    0.05f
#define YAW_PID_KD                    0.15f
#define YAW_PID_I_MAX                 15.0f
#define YAW_PID_OUT_MAX               30.0f

/* ========== Heading-Hold PID Gains ========== */
#define HEADING_PID_KP                2.0f    /* RPM per degree of heading error */
#define HEADING_PID_KI                0.15f   /* integral accumulation rate */
#define HEADING_PID_KD                0.3f    /* derivative damping */
#define HEADING_PID_I_MAX             15.0f   /* integral windup limit (RPM) */
#define HEADING_PID_OUT_MAX           25.0f   /* max correction (RPM) — ~42% of full speed */
#define HEADING_HOLD_DEADBAND_DPS     3.0f

/* ========== Module-Private State ========== */
static struct {
    /* Calibrated gyro offsets (raw LSB) */
    float gyro_offset_x, gyro_offset_y, gyro_offset_z;

    /* Raw readings (latest) */
    int16_t raw_ax, raw_ay, raw_az;
    int16_t raw_gx, raw_gy, raw_gz;

    /* Filtered readings in physical units */
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;
    float accel_x_g, accel_y_g, accel_z_g;

    /* Integrated yaw angle (degrees) */
    float yaw_angle_deg;
    float yaw_reference_deg;

    /* Pitch / roll from accelerometer (degrees) */
    float pitch_deg;
    float roll_deg;

    /* Accel magnitude (g) */
    float accel_mag_g;

    /* Yaw-rate PID state */
    float yaw_pid_integral;
    float yaw_pid_last_error;

    /* Heading-hold PID state */
    float heading_pid_integral;
    float heading_pid_last_error;

    /* Control flags */
    uint8_t closed_loop_enabled;
    float   tilt_max_pitch;
    float   tilt_max_roll;

    /* Timing */
    uint32_t last_process_ms;

    /* Status flags */
    uint8_t gyro_calibrated;
    uint8_t imu_healthy;
} imu_state;

/* ================================================================= */
/*                    Calibration                                    */
/* ================================================================= */

uint8_t IMU_CalibrateGyro(uint16_t num_samples)
{
    if (num_samples == 0) return 0;

    int64_t sum_x = 0, sum_y = 0, sum_z = 0;
    int16_t gx, gy, gz;
    uint16_t collected = 0;

    for (uint16_t i = 0; i < num_samples; i++)
    {
        IMU_ReadGyro(&gx, &gy, &gz);
        sum_x += gx;
        sum_y += gy;
        sum_z += gz;
        collected++;
        HAL_Delay(1);  /* ~1 kHz sampling during calibration */
    }

    imu_state.gyro_offset_x = (float)sum_x / (float)collected;
    imu_state.gyro_offset_y = (float)sum_y / (float)collected;
    imu_state.gyro_offset_z = (float)sum_z / (float)collected;

    imu_state.gyro_calibrated = 1;
    imu_state.yaw_angle_deg   = 0.0f;
    imu_state.yaw_reference_deg = 0.0f;

    return 1;
}

/* ================================================================= */
/*                    Data Processing                                */
/* ================================================================= */

void IMU_Process(void)
{
    /* 1. Read raw sensor data */
    IMU_ReadAccel(&imu_state.raw_ax, &imu_state.raw_ay, &imu_state.raw_az);
    IMU_ReadGyro(&imu_state.raw_gx, &imu_state.raw_gy, &imu_state.raw_gz);

    /* 2. Apply calibration offsets and convert to physical units */
    float gyro_x_raw = ((float)imu_state.raw_gx - imu_state.gyro_offset_x) / GYRO_SENSITIVITY_LSB_PER_DPS;
    float gyro_y_raw = ((float)imu_state.raw_gy - imu_state.gyro_offset_y) / GYRO_SENSITIVITY_LSB_PER_DPS;
    float gyro_z_raw = ((float)imu_state.raw_gz - imu_state.gyro_offset_z) / GYRO_SENSITIVITY_LSB_PER_DPS;

    float accel_x_raw = (float)imu_state.raw_ax / ACCEL_SENSITIVITY_LSB_PER_G;
    float accel_y_raw = (float)imu_state.raw_ay / ACCEL_SENSITIVITY_LSB_PER_G;
    float accel_z_raw = (float)imu_state.raw_az / ACCEL_SENSITIVITY_LSB_PER_G;

    /* 3. Low-pass filter (exponential moving average) */
    imu_state.gyro_x_dps += IMU_LPF_ALPHA * (gyro_x_raw - imu_state.gyro_x_dps);
    imu_state.gyro_y_dps += IMU_LPF_ALPHA * (gyro_y_raw - imu_state.gyro_y_dps);
    imu_state.gyro_z_dps += IMU_LPF_ALPHA * (gyro_z_raw - imu_state.gyro_z_dps);

    imu_state.accel_x_g += IMU_LPF_ALPHA * (accel_x_raw - imu_state.accel_x_g);
    imu_state.accel_y_g += IMU_LPF_ALPHA * (accel_y_raw - imu_state.accel_y_g);
    imu_state.accel_z_g += IMU_LPF_ALPHA * (accel_z_raw - imu_state.accel_z_g);

    /* 4. Drift deadband on gyro Z */
    if (imu_state.gyro_z_dps <  IMU_YAW_DRIFT_DEADBAND_DPS &&
        imu_state.gyro_z_dps > -IMU_YAW_DRIFT_DEADBAND_DPS)
    {
        imu_state.gyro_z_dps = 0.0f;
    }

    /* 5. Compute dt and integrate yaw angle */
    uint32_t now = HAL_GetTick();
    float dt_seconds = 0.0f;

    if (imu_state.last_process_ms != 0U)
    {
        uint32_t dt_ms = now - imu_state.last_process_ms;
        /* Sanity check: ignore gaps > 100ms (e.g. after debugger halt) */
        if (dt_ms <= 100U)
        {
            dt_seconds = (float)dt_ms * 0.001f;
        }
    }
    imu_state.last_process_ms = now;

    imu_state.yaw_angle_deg += imu_state.gyro_z_dps * dt_seconds;

    /* 6. Compute pitch and roll from accelerometer */
    float ax = imu_state.accel_x_g;
    float ay = imu_state.accel_y_g;
    float az = imu_state.accel_z_g;

    /* roll  = atan2(ay, az)
       pitch = atan2(-ax, sqrt(ay^2 + az^2)) */
    imu_state.roll_deg  = atan2f(ay, az) * 57.295779513f;
    imu_state.pitch_deg = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.295779513f;

    /* 7. Compute accel magnitude */
    imu_state.accel_mag_g = sqrtf(ax * ax + ay * ay + az * az);

    imu_state.imu_healthy = 1;
}

/* ================================================================= */
/*                    Data Accessors                                 */
/* ================================================================= */

float IMU_GetYawRate_dps(void)
{
    return imu_state.gyro_z_dps;
}

float IMU_GetYawAngle_deg(void)
{
    return imu_state.yaw_angle_deg;
}

float IMU_GetPitch_deg(void)
{
    return imu_state.pitch_deg;
}

float IMU_GetRoll_deg(void)
{
    return imu_state.roll_deg;
}

float IMU_GetAccelMagnitude_g(void)
{
    return imu_state.accel_mag_g;
}

void IMU_ResetYawReference(void)
{
    imu_state.yaw_reference_deg = imu_state.yaw_angle_deg;
    imu_state.heading_pid_integral = 0.0f;
    imu_state.heading_pid_last_error = 0.0f;
}

/* ================================================================= */
/*                    Yaw-Rate PID Controller                        */
/* ================================================================= */

float IMU_YawRatePID_Compute(float target_yaw_rate_dps)
{
    if (!imu_state.gyro_calibrated || !imu_state.imu_healthy)
    {
        return 0.0f;
    }

    float error = target_yaw_rate_dps - imu_state.gyro_z_dps;

    /* Proportional */
    float P = YAW_PID_KP * error;

    /* Integral (with anti-windup) */
    /* Use a fixed dt of 0.005s (5ms) since this runs synchronously
       with the motor update at 200 Hz */
    imu_state.yaw_pid_integral += YAW_PID_KI * error * 0.005f;
    if (imu_state.yaw_pid_integral >  YAW_PID_I_MAX) imu_state.yaw_pid_integral =  YAW_PID_I_MAX;
    if (imu_state.yaw_pid_integral < -YAW_PID_I_MAX) imu_state.yaw_pid_integral = -YAW_PID_I_MAX;

    /* Derivative (on measurement, not error, to avoid derivative kick) */
    float D = YAW_PID_KD * (imu_state.yaw_pid_last_error - imu_state.gyro_z_dps) / 0.005f;
    imu_state.yaw_pid_last_error = imu_state.gyro_z_dps;

    /* Sum and clamp */
    float output = P + imu_state.yaw_pid_integral + D;
    if (output >  YAW_PID_OUT_MAX) output =  YAW_PID_OUT_MAX;
    if (output < -YAW_PID_OUT_MAX) output = -YAW_PID_OUT_MAX;

    return output;
}

/* ================================================================= */
/*                    Heading-Hold PID Controller                     */
/* ================================================================= */

float IMU_HeadingHold_Compute(void)
{
    if (!imu_state.gyro_calibrated || !imu_state.imu_healthy)
    {
        return 0.0f;
    }

    /* Compute heading error, unwrap to [-180, +180] degrees */
    float error = imu_state.yaw_reference_deg - imu_state.yaw_angle_deg;
    while (error >  180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    /* Proportional */
    float P = HEADING_PID_KP * error;

    /* Integral */
    imu_state.heading_pid_integral += HEADING_PID_KI * error * 0.005f;
    if (imu_state.heading_pid_integral >  HEADING_PID_I_MAX) imu_state.heading_pid_integral =  HEADING_PID_I_MAX;
    if (imu_state.heading_pid_integral < -HEADING_PID_I_MAX) imu_state.heading_pid_integral = -HEADING_PID_I_MAX;

    /* Derivative (use gyro Z directly: if gyro is rotating away, damp it) */
    float D = HEADING_PID_KD * (-imu_state.gyro_z_dps);

    /* Sum and clamp */
    float output = P + imu_state.heading_pid_integral + D;
    if (output >  HEADING_PID_OUT_MAX) output =  HEADING_PID_OUT_MAX;
    if (output < -HEADING_PID_OUT_MAX) output = -HEADING_PID_OUT_MAX;

    imu_state.heading_pid_last_error = error;

    return output;
}

/* ================================================================= */
/*                    Tilt Safety                                    */
/* ================================================================= */

uint8_t IMU_IsTiltSafe(void)
{
    if (!imu_state.imu_healthy) return 1;  /* If no data yet, assume safe */

    float abs_pitch = (imu_state.pitch_deg >= 0.0f) ? imu_state.pitch_deg : -imu_state.pitch_deg;
    float abs_roll  = (imu_state.roll_deg  >= 0.0f) ? imu_state.roll_deg  : -imu_state.roll_deg;

    if (abs_pitch > imu_state.tilt_max_pitch || abs_roll > imu_state.tilt_max_roll)
    {
        return 0;  /* UNSAFE */
    }

    return 1;  /* SAFE */
}

void IMU_SetTiltLimits(float max_pitch_deg, float max_roll_deg)
{
    imu_state.tilt_max_pitch = max_pitch_deg;
    imu_state.tilt_max_roll  = max_roll_deg;
}

/* ================================================================= */
/*                    Control Enable                                 */
/* ================================================================= */

void IMU_SetClosedLoopEnable(uint8_t enable)
{
    imu_state.closed_loop_enabled = (enable != 0U) ? 1 : 0;

    /* Reset PID state on enable transition */
    if (imu_state.closed_loop_enabled)
    {
        imu_state.yaw_pid_integral       = 0.0f;
        imu_state.yaw_pid_last_error     = 0.0f;
        imu_state.heading_pid_integral   = 0.0f;
        imu_state.heading_pid_last_error = 0.0f;
        IMU_ResetYawReference();
    }
}

uint8_t IMU_IsClosedLoopEnabled(void)
{
    return imu_state.closed_loop_enabled;
}

/* ================================================================= */
/*                    Processed Data Print                           */
/* ================================================================= */

void IMU_PrintProcessed(void)
{
    static char msg[192];
    int len = snprintf(msg, sizeof(msg),
                       "IMU yaw=%.1fdps ang=%.1fdeg pitch=%.1f roll=%.1f mag=%.2fg safe=%u cl=%u\r\n",
                       imu_state.gyro_z_dps,
                       imu_state.yaw_angle_deg,
                       imu_state.pitch_deg,
                       imu_state.roll_deg,
                       imu_state.accel_mag_g,
                       IMU_IsTiltSafe(),
                       imu_state.closed_loop_enabled);

    if (len > 0) {
        CDC_Transmit_HS((uint8_t *)msg, (uint16_t)len);
    }
}