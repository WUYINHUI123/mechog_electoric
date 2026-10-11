/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ��������� V7.13 (�ռ������ܷ��棺�ϸ�󶨵����LEG�궨��)
  * LF=1,2(LEG1) | RF=3,4(LEG2) | RB=5,6(LEG3) | LB=7,8(LEG4)
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "fdcan.h"
#include "spi.h"       
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "bsp_fdcan.h"
#include "delay.h"
#include "dm_motor_ctrl.h"
#include "DOG_leg_vmc.h"
#include "gait.h"
#include "bmiO88.h"
#include "pid.h"
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

typedef struct {
    float kp;
    float kd; 
    float pos;
    float tor;
    float vel;
} motor_cfg_t;

/* USER CODE BEGIN PD */
#define CTRL_PERIOD_MS 3  

#define CALIBRATION_MODE 0  
#define USE_IMU_BALANCE 1   

#define VMC_KP  40.0f   
#define VMC_KD  2.0f    

#define VMC_SUPPORT_FORCE 0.0f  

#define TOUCHDOWN_FORCE_TH 15.0f

#define IMU_PITCH_KP    8.0f    
#define IMU_PITCH_KD    0.002f
#define IMU_ROLL_KP     3.0f    
#define IMU_ROLL_KD     0.002f

/* ===== SBUS 遥控配置 (通过 CAN2 从轮毂电机板转发) ===== */
#define SBUS_DEADBAND       200     /* 转向死区 (±20%) */
#define SBUS_ARM_THRESHOLD  500     /* CH5 解锁阈值 */
#define SBUS_TIMEOUT_MS     300     /* CAN2 数据超时 */
#define WHEEL_MAX_RPM       60.0f   /* 轮毂电机最大转速 */

#define TROT_FREQ            0.0035f   /* 周期 ~1.0s, 摆动相 ~400ms */
#define TROT_HEIGHT          85        /* 抬腿高度 85, 离地更明显 */
#define TROT_STRIDE          25        /* 步幅 25, 不易拖地 */
#define TROT_FRONT_Z_OFFSET  0.0f

#define PACE_FREQ            0.0060f  
#define PACE_HEIGHT          30       
#define PACE_STRIDE          50       

#define BOUND_FREQ           0.0075f  
#define BOUND_HEIGHT         40       
#define BOUND_STRIDE         80       

#define WHEELBASE_STRETCH    0.0f    
/* USER CODE END PD */

static const motor_cfg_t leg_motor_kd[4][2] = {
    [0] = {{VMC_KP, VMC_KD, 0, 0, 0}, {VMC_KP, VMC_KD, 0, 0, 0}},
    [1] = {{VMC_KP, VMC_KD, 0, 0, 0}, {VMC_KP, VMC_KD, 0, 0, 0}},
    [2] = {{VMC_KP, VMC_KD, 0, 0, 0}, {VMC_KP, VMC_KD, 0, 0, 0}},
    [3] = {{VMC_KP, VMC_KD, 0, 0, 0}, {VMC_KP, VMC_KD, 0, 0, 0}}
};

extern Node LeftFront, LeftBack, RightFront, RightBack;
float t = 0;  // ?? ��֤����ͨ��

/* ========== 板间通信 (CAN2, 对齐轮毂电机板协议) ========== */
/* 发送: CAN ID 0x100 → [float forward_rpm, float steer_norm] */
static float can2_fwd_rpm   = 0.0f;    /* 目标前进速度 (RPM) */
static float can2_steer_norm = 0.0f;    /* 归一化转向 (-1.0 .. +1.0) */

/* 接收: CAN ID 0x101 ← [int16 ch1, int16 ch3, int16 ch5, int16 ch6] */
static int16_t can2_rx_ch[4] = {0, 0, 0, 0};  /* CH1/CH3/CH5/CH6 */
static uint32_t can2_rx_tick = 0;             /* 最后一次收到 SBUS 的时间 */
uint8_t  board_rx_data[8] = {0};
uint16_t board_rx_id = 0;

void SystemClock_Config(void);
void Targeted_Safe_Damping(int motor1_id, int motor2_id);

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_SPI2_Init();  
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  
  power(1);
  HAL_Delay(1000);
  
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  bsp_fdcan_set_baud(&hfdcan1, CAN_CLASS, CAN_BR_1M);
  bsp_fdcan_set_baud(&hfdcan2, CAN_CLASS, CAN_BR_1M);
  delay_init(400);
  bsp_can_init();
  dm_motor_init();        
  
  BMI088_Init(&hspi2); 

  delay_ms(500);
  /* 先清除所有电机错误状态，否则有历史错误的电机会忽略使能命令（亮红灯不动） */
  for(int i = 1; i <= 8; i++) {
      clear_err(&hfdcan1, i, MIT_MODE);
      delay_ms(1);
  }
  delay_ms(50);
  /* 使能所有电机的 MIT 控制模式 */
  for(int i = 1; i <= 8; i++) {
      enable_motor_mode(&hfdcan1, i, MIT_MODE);
      delay_ms(2);
  }
  delay_ms(100);

  uint32_t next_tick = HAL_GetTick();
  uint32_t imu_tick  = HAL_GetTick();
  float alpha, beta;
  
  CascadePID_t pid_pitch;
  CascadePID_t pid_roll;
  CascadePID_Init(&pid_pitch, IMU_PITCH_KP, IMU_PITCH_KD, 60.0f); 
  CascadePID_Init(&pid_roll,  IMU_ROLL_KP,  IMU_ROLL_KD,  60.0f); 

  float lpf_pitch = 0.0f, lpf_roll = 0.0f;
  float lpf_gyro_x = 0.0f, lpf_gyro_y = 0.0f;
  float comp_pitch = 0.0f, comp_roll = 0.0f;
  
  float startup_progress = 0.0f; 
  float gait_progress    = 0.0f; 
  float smooth_gait      = 0.0f; 
  float start_y = 90.0f; 

  float lpf_Fy[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  /* 板间通信心跳定时器 */
  uint32_t board_comm_tick = 0;
  MODE_SET dynamic_gait = STAND;  /* 由 SBUS 动态决定的当前步态 */
  float    turn_progress = 0.0f;  /* STAND<->TROT 平滑过渡 (0=站立, 1=完全踏步) */

  while (1)
  {
    uint32_t current_tick = HAL_GetTick();

    /* 板间通信：每 50ms 发送运动指令到轮毂电机板 (CAN ID 0x100) */
    if (current_tick - board_comm_tick >= 50) {
        board_comm_tick = current_tick;

        /* ---- 组装运动指令: [float fwd_rpm, float steer_norm] ---- */
        uint8_t tx[8];
        memcpy(&tx[0], &can2_fwd_rpm,   4);
        memcpy(&tx[4], &can2_steer_norm, 4);
        fdcanx_send_data(&hfdcan2, 0x100, tx, 8);
    }

    if (current_tick - imu_tick >= 4) {
        imu_tick = current_tick;
        BMI088_Update(&hspi2);
        IMU_Heater_Ctrl();
        
        lpf_pitch  = LowPassFilter(imu_data.pitch,  lpf_pitch,  0.03f);
        lpf_roll   = LowPassFilter(imu_data.roll,   lpf_roll,   0.03f);
        lpf_gyro_x = LowPassFilter(imu_data.gyro_x, lpf_gyro_x, 0.03f);
        lpf_gyro_y = LowPassFilter(imu_data.gyro_y, lpf_gyro_y, 0.03f);
        
        comp_pitch = CascadePID_Calc(&pid_pitch, lpf_pitch, lpf_gyro_x);
        comp_roll  = CascadePID_Calc(&pid_roll,  lpf_roll,  lpf_gyro_y);

        if (startup_progress < 1.0f) {
            comp_pitch = 0.0f;
            comp_roll  = 0.0f;
            pid_pitch.out = 0.0f; 
            pid_roll.out  = 0.0f; 
        }

#if USE_IMU_BALANCE == 1
        comp_pitch = 0.0f;
        comp_roll  = 0.0f;
#endif
    }

    if (current_tick - next_tick >= CTRL_PERIOD_MS) 
    {
        next_tick = current_tick;
        
        if (t >= Ts) t = 0;

        /* ===== 读取 SBUS 遥控 (CAN2 ID 0x101 从轮毂电机板转发) ===== */
        {
            int16_t sbus_ch1 = can2_rx_ch[0];  /* CH1: 转向 */
            int16_t sbus_ch3 = can2_rx_ch[1];  /* CH3: 油门 */
            int16_t sbus_ch5 = can2_rx_ch[2];  /* CH5: 解锁 */
            uint8_t sbus_alive = ((current_tick - can2_rx_tick) < SBUS_TIMEOUT_MS);
            uint8_t sbus_armed = (sbus_ch5 > SBUS_ARM_THRESHOLD);
            int16_t steer_abs  = (sbus_ch1 > 0) ? sbus_ch1 : -sbus_ch1;

            if (sbus_alive && sbus_armed && startup_progress >= 1.0f)
            {
                /* 转弯 → 缓入 TROT；不转弯 → 缓出回 STAND */
                if (steer_abs > SBUS_DEADBAND) {
                    turn_progress += 0.008f;  /* 约 375ms 缓入 */
                    if (turn_progress > 1.0f) turn_progress = 1.0f;
                } else {
                    turn_progress -= 0.008f;  /* 约 375ms 缓出 */
                    if (turn_progress < 0.0f) turn_progress = 0.0f;
                }
                dynamic_gait = (turn_progress > 0.01f) ? TROT : STAND;

                /* 油门/转向映射到轮毂电机 */
                can2_fwd_rpm    = ((float)sbus_ch3 / 1000.0f) * WHEEL_MAX_RPM;
                can2_steer_norm  = ((float)sbus_ch1 / 1000.0f);
            }
            else
            {
                dynamic_gait     = STAND;
                turn_progress    = 0.0f;
                can2_fwd_rpm     = 0.0f;
                can2_steer_norm  = 0.0f;
            }

            /* ---- 串口调试: 每500ms打印一次SBUS状态 (PA9 TX, 921600bps) ---- */
            {
                static uint32_t last_dbg_tick = 0;
                if (current_tick - last_dbg_tick >= 500) {
                    last_dbg_tick = current_tick;
                    extern UART_HandleTypeDef huart1;
                    char buf[100];
                    int n = sprintf(buf,
                        "SBUS ch1=%d ch3=%d ch5=%d alive=%d armed=%d gait=%d "
                        "fwd=%.1f str=%.2f t=%lu\r\n",
                        can2_rx_ch[0], can2_rx_ch[1], can2_rx_ch[2],
                        ((current_tick - can2_rx_tick) < 300) ? 1 : 0,
                        (can2_rx_ch[2] > 500) ? 1 : 0,
                        dynamic_gait,
                        can2_fwd_rpm, can2_steer_norm,
                        (unsigned long)current_tick);
                    HAL_UART_Transmit(&huart1, (uint8_t *)buf, n, 100);
                }
            }
        }

        if (startup_progress < 1.0f) {
            startup_progress += 0.0015f;
            gaitMode(STAND, 0, 0, 0);
        } else {
            startup_progress = 1.0f; 
#if CALIBRATION_MODE == 1
            gaitMode(STAND, 0, 0, 0); 
            smooth_gait = 0.0f;
#else
            if (gait_progress < 1.0f) {
                gait_progress += 0.0015f; 
            } else {
                gait_progress = 1.0f; 
            }

            smooth_gait = gait_progress * gait_progress * (3.0f - 2.0f * gait_progress);

            gaitMode(STAND, 0, 0, 0); 
            float stand_LF_x = LeftFront.x, stand_LF_y = LeftFront.y;
            float stand_RF_x = RightFront.x, stand_RF_y = RightFront.y;
            float stand_LB_x = LeftBack.x, stand_LB_y = LeftBack.y;
            float stand_RB_x = RightBack.x, stand_RB_y = RightBack.y;

            MODE_SET target_mode  = dynamic_gait;
            float    target_freq  = 0.0f;
            uint8_t  target_height= 0;
            int8_t   target_stride= 0;

            if (target_mode == TROT) {
                target_freq   = TROT_FREQ;
                target_height = (uint8_t)((float)TROT_HEIGHT * turn_progress);
                /* 步幅: 大小跟油门, 方向跟转向, 幅度由 turn_progress 缓入缓出 */
                {
                    float throttle_norm = (float)can2_rx_ch[1] / 1000.0f;
                    float stride_f = throttle_norm * (float)TROT_STRIDE * turn_progress;
                    if (stride_f > (float)TROT_STRIDE)  stride_f = (float)TROT_STRIDE;
                    if (stride_f < -(float)TROT_STRIDE) stride_f = -(float)TROT_STRIDE;
                    target_stride = (int8_t)stride_f;
                }
            } else if (target_mode == PACE) {
                target_freq   = PACE_FREQ;
                target_height = PACE_HEIGHT;
                target_stride = PACE_STRIDE;
            } else if (target_mode == BOUND) {
                target_freq   = BOUND_FREQ;
                target_height = BOUND_HEIGHT;
                target_stride = BOUND_STRIDE;
            } else if (target_mode == STAND) {
                target_freq   = 0.0f;
                target_height = 0;
                target_stride = 0;
            }

            gaitMode(target_mode, target_freq, target_height, target_stride); 

            LeftFront.x = stand_LF_x + (LeftFront.x - stand_LF_x) * smooth_gait;
            LeftFront.y = stand_LF_y + (LeftFront.y - stand_LF_y) * smooth_gait;
            
            RightFront.x = stand_RF_x + (RightFront.x - stand_RF_x) * smooth_gait;
            RightFront.y = stand_RF_y + (RightFront.y - stand_RF_y) * smooth_gait;

            LeftBack.x = stand_LB_x + (LeftBack.x - stand_LB_x) * smooth_gait;
            LeftBack.y = stand_LB_y + (LeftBack.y - stand_LB_y) * smooth_gait;

            RightBack.x = stand_RB_x + (RightBack.x - stand_RB_x) * smooth_gait;
            RightBack.y = stand_RB_y + (RightBack.y - stand_RB_y) * smooth_gait;

          
            if (target_mode == TROT) {
                int16_t steer = can2_rx_ch[0];
                int16_t s_abs = (steer > 0) ? steer : -steer;
                if (s_abs > SBUS_DEADBAND) {
                    float turn_x = ((float)steer / 1000.0f) * (float)TROT_STRIDE * 0.75f * turn_progress;
                    LeftFront.x  += turn_x;
                    LeftBack.x   += turn_x;
                    RightFront.x -= turn_x;
                    RightBack.x  -= turn_x;
                }
            }
#endif
        }

#if CALIBRATION_MODE == 1
        startup_progress = 1.0f;
        comp_pitch = 0.0f;
        comp_roll = 0.0f;
        LeftFront.x = X_ZERO; RightFront.x =X_ZERO; LeftBack.x = -X_ZERO; RightBack.x = -X_ZERO;
#endif

        float smooth_progress = startup_progress * startup_progress * (3.0f - 2.0f * startup_progress);
        float kp_ratio = smooth_progress;
        
        /* TROT 模式下增强刚度 + 支撑力 */
        float trot_kp_scale    = 1.0f;
        float trot_extra_force = 0.0f;
        if (dynamic_gait == TROT) {
            trot_kp_scale    = 1.0f + 0.5f * turn_progress;  /* STAND=1.0x → TROT=1.5x */
            trot_extra_force = 10.0f * turn_progress;        /* 0 → 10N 额外支撑 */
        }

        float applied_force = (VMC_SUPPORT_FORCE + trot_extra_force) * smooth_progress;

        float applied_z_offset = 0.0f;
#if CALIBRATION_MODE == 0
        if (dynamic_gait == TROT) {
            applied_z_offset = TROT_FRONT_Z_OFFSET * smooth_gait;
        }
#endif

        float target_LF_y = start_y + applied_z_offset + (LeftFront.y - start_y) * smooth_progress + (comp_pitch + comp_roll) * smooth_progress;
        float target_RF_y = start_y + applied_z_offset + (RightFront.y - start_y) * smooth_progress + (comp_pitch - comp_roll) * smooth_progress;
        float target_LB_y = start_y + (LeftBack.y - start_y) * smooth_progress + (-comp_pitch + comp_roll) * smooth_progress;
        float target_RB_y = start_y + (RightBack.y - start_y) * smooth_progress + (-comp_pitch - comp_roll) * smooth_progress;
        
        float applied_stretch = 0.0f;
#if CALIBRATION_MODE == 0
        if (dynamic_gait != STAND) {
            applied_stretch = WHEELBASE_STRETCH * smooth_gait;
        }
#endif

        // ��ȡ��̬���λ�� (����������)
        float dx_LF = LeftFront.x - X_ZERO;
        float dx_RF = RightFront.x - X_ZERO;
        float dx_LB = LeftBack.x - (-X_ZERO);
        float dx_RB = RightBack.x - (-X_ZERO);

        // ?? ���������񲹳� (���� -dx, ���� +dx����֤ȫԱ��ͷ����)
        float dynamic_LF_x =  X_ZERO + dx_LF + applied_stretch; 
        float dynamic_LB_x = -X_ZERO+ dx_LB - applied_stretch; 

        float dynamic_RF_x =  X_ZERO - dx_RF + applied_stretch;                  
        float dynamic_RB_x = -X_ZERO- dx_RB - applied_stretch;                      

        // =========================================================================
        // ?? ��VMC ����迹����ִ������
        // =========================================================================

        // -------------------------------------------------------------------------
        // ����1�� ��ǰ�� (LF) -> ?? ��� 1, 2 | ʹ�� LEG1 ��
        // -------------------------------------------------------------------------
        if(solveInverseKinematics(dynamic_LF_x, target_LF_y, &alpha, &beta)) {
            float real_alpha = (motor[Motor1].para.pos - LEG1_ALPHA_OFFSET) / LEG1_ALPHA_DIR;
            float real_beta  = (motor[Motor2].para.pos - LEG1_BETA_OFFSET) / LEG1_BETA_DIR;
            float real_tau1  = motor[Motor1].para.tor / LEG1_ALPHA_DIR;
            float real_tau2  = motor[Motor2].para.tor / LEG1_BETA_DIR;

            float est_Fx = 0.0f, est_Fy_raw = 0.0f;
            estimateFootForce(dynamic_LF_x, target_LF_y, real_alpha, real_beta, real_tau1, real_tau2, &est_Fx, &est_Fy_raw);
            lpf_Fy[0] = LowPassFilter(est_Fy_raw, lpf_Fy[0], 0.05f);
            
            float current_kp = leg_motor_kd[0][0].kp * kp_ratio * trot_kp_scale;
            float tau1_ff = 0.0f, tau2_ff = 0.0f;

            if (lpf_Fy[0] > TOUCHDOWN_FORCE_TH) {
                calcJacobianAndTorque(dynamic_LF_x, target_LF_y, real_alpha, real_beta, 0.0f, VMC_SUPPORT_FORCE * 1.2f, &tau1_ff, &tau2_ff);
            } else {
                calcJacobianAndTorque(dynamic_LF_x, target_LF_y, real_alpha, real_beta, 0.0f, applied_force, &tau1_ff, &tau2_ff);
            }

            float m1_rad = alpha * LEG1_ALPHA_DIR + LEG1_ALPHA_OFFSET;
            float m2_rad = beta  * LEG1_BETA_DIR  + LEG1_BETA_OFFSET;
            mit_ctrl(&hfdcan1, &motor[Motor1], Motor1, m1_rad, 0, current_kp, leg_motor_kd[0][0].kd, tau1_ff * LEG1_ALPHA_DIR);
            mit_ctrl(&hfdcan1, &motor[Motor2], Motor2, m2_rad, 0, current_kp, leg_motor_kd[0][1].kd, tau2_ff * LEG1_BETA_DIR);
        } else { Targeted_Safe_Damping(Motor1, Motor2); }
        delay_us(300); 

        // -------------------------------------------------------------------------
        // ����2�� ��ǰ�� (RF) -> ?? ��� 3, 4 | ʹ�� LEG2 ��
        // -------------------------------------------------------------------------
        if(solveInverseKinematics(dynamic_RF_x, target_RF_y, &alpha, &beta)) {
            float real_alpha = (motor[Motor3].para.pos - LEG2_ALPHA_OFFSET) / LEG2_ALPHA_DIR;
            float real_beta  = (motor[Motor4].para.pos - LEG2_BETA_OFFSET) / LEG2_BETA_DIR;
            float real_tau3  = motor[Motor3].para.tor / LEG2_ALPHA_DIR;
            float real_tau4  = motor[Motor4].para.tor / LEG2_BETA_DIR;

            float est_Fx = 0.0f, est_Fy_raw = 0.0f;
            estimateFootForce(dynamic_RF_x, target_RF_y, real_alpha, real_beta, real_tau3, real_tau4, &est_Fx, &est_Fy_raw);
            lpf_Fy[1] = LowPassFilter(est_Fy_raw, lpf_Fy[1], 0.05f);
            
            float current_kp = leg_motor_kd[1][0].kp * kp_ratio * trot_kp_scale;
            float tau3_ff = 0.0f, tau4_ff = 0.0f;

            if (lpf_Fy[1] > TOUCHDOWN_FORCE_TH) {
                calcJacobianAndTorque(dynamic_RF_x, target_RF_y, real_alpha, real_beta, 0.0f, VMC_SUPPORT_FORCE * 1.2f, &tau3_ff, &tau4_ff);
            } else {
                calcJacobianAndTorque(dynamic_RF_x, target_RF_y, real_alpha, real_beta, 0.0f, applied_force, &tau3_ff, &tau4_ff);
            }

            float m3_rad = alpha * LEG2_ALPHA_DIR + LEG2_ALPHA_OFFSET;
            float m4_rad = beta  * LEG2_BETA_DIR  + LEG2_BETA_OFFSET;
            mit_ctrl(&hfdcan1, &motor[Motor3], Motor3, m3_rad, 0, current_kp, leg_motor_kd[1][0].kd, tau3_ff * LEG2_ALPHA_DIR);
            mit_ctrl(&hfdcan1, &motor[Motor4], Motor4, m4_rad, 0, current_kp, leg_motor_kd[1][1].kd, tau4_ff * LEG2_BETA_DIR);
        } else { Targeted_Safe_Damping(Motor3, Motor4); }
        delay_us(300);

        // -------------------------------------------------------------------------
        // ����3�� �Һ��� (RB) -> ?? ����󶨵�� 5, 6 | ����ʹ�� LEG3 �꣡
        // -------------------------------------------------------------------------
        if(solveInverseKinematics(dynamic_RB_x, target_RB_y, &alpha, &beta)) {
            // ���׾���������ȫ��λʹ�� LEG3 �� Motor5, Motor6
            float real_alpha = (motor[Motor5].para.pos - LEG3_ALPHA_OFFSET) / LEG3_ALPHA_DIR;
            float real_beta  = (motor[Motor6].para.pos - LEG3_BETA_OFFSET) / LEG3_BETA_DIR;
            float real_tau5  = motor[Motor5].para.tor / LEG3_ALPHA_DIR;
            float real_tau6  = motor[Motor6].para.tor / LEG3_BETA_DIR;

            float est_Fx = 0.0f, est_Fy_raw = 0.0f;
            estimateFootForce(dynamic_RB_x, target_RB_y, real_alpha, real_beta, real_tau5, real_tau6, &est_Fx, &est_Fy_raw);
            lpf_Fy[2] = LowPassFilter(est_Fy_raw, lpf_Fy[2], 0.05f);
            
            float current_kp = leg_motor_kd[2][0].kp * kp_ratio * trot_kp_scale;
            float tau5_ff = 0.0f, tau6_ff = 0.0f;

            if (lpf_Fy[2] > TOUCHDOWN_FORCE_TH) {
                calcJacobianAndTorque(dynamic_RB_x, target_RB_y, real_alpha, real_beta, 0.0f, VMC_SUPPORT_FORCE * 1.2f, &tau5_ff, &tau6_ff);
            } else {
                calcJacobianAndTorque(dynamic_RB_x, target_RB_y, real_alpha, real_beta, 0.0f, applied_force, &tau5_ff, &tau6_ff);
            }

            float m5_rad = alpha * LEG3_ALPHA_DIR + LEG3_ALPHA_OFFSET;
            float m6_rad = beta  * LEG3_BETA_DIR  + LEG3_BETA_OFFSET;
            mit_ctrl(&hfdcan1, &motor[Motor5], Motor5, m5_rad, 0, current_kp, leg_motor_kd[2][0].kd, tau5_ff * LEG3_ALPHA_DIR);
            mit_ctrl(&hfdcan1, &motor[Motor6], Motor6, m6_rad, 0, current_kp, leg_motor_kd[2][1].kd, tau6_ff * LEG3_BETA_DIR);
        } else { Targeted_Safe_Damping(Motor5, Motor6); }
        delay_us(300);

       
        if(solveInverseKinematics(dynamic_LB_x, target_LB_y, &alpha, &beta)) {
           
            float real_alpha = (motor[Motor7].para.pos - LEG4_ALPHA_OFFSET) / LEG4_ALPHA_DIR;
            float real_beta  = (motor[Motor8].para.pos - LEG4_BETA_OFFSET) / LEG4_BETA_DIR;
            float real_tau7  = motor[Motor7].para.tor / LEG4_ALPHA_DIR;
            float real_tau8  = motor[Motor8].para.tor / LEG4_BETA_DIR;

            float est_Fx = 0.0f, est_Fy_raw = 0.0f;
            estimateFootForce(dynamic_LB_x, target_LB_y, real_alpha, real_beta, real_tau7, real_tau8, &est_Fx, &est_Fy_raw);
            lpf_Fy[3] = LowPassFilter(est_Fy_raw, lpf_Fy[3], 0.05f);
            
            float current_kp = leg_motor_kd[3][0].kp * kp_ratio * trot_kp_scale;
            float tau7_ff = 0.0f, tau8_ff = 0.0f;

            if (lpf_Fy[3] > TOUCHDOWN_FORCE_TH) {
                calcJacobianAndTorque(dynamic_LB_x, target_LB_y, real_alpha, real_beta, 0.0f, VMC_SUPPORT_FORCE * 1.2f, &tau7_ff, &tau8_ff);
            } else {
                calcJacobianAndTorque(dynamic_LB_x, target_LB_y, real_alpha, real_beta, 0.0f, applied_force, &tau7_ff, &tau8_ff);
            }

            float m7_rad = alpha * LEG4_ALPHA_DIR + LEG4_ALPHA_OFFSET;
            float m8_rad = beta  * LEG4_BETA_DIR  + LEG4_BETA_OFFSET;
            mit_ctrl(&hfdcan1, &motor[Motor7], Motor7, m7_rad, 0, current_kp, leg_motor_kd[3][0].kd, tau7_ff * LEG4_ALPHA_DIR);
            mit_ctrl(&hfdcan1, &motor[Motor8], Motor8, m8_rad, 0, current_kp, leg_motor_kd[3][1].kd, tau8_ff * LEG4_BETA_DIR);
        } else { Targeted_Safe_Damping(Motor7, Motor8); }

    } 
  }
}

void Targeted_Safe_Damping(int motor1_id, int motor2_id) {
    mit_ctrl(&hfdcan1, &motor[motor1_id], motor1_id, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f);
    mit_ctrl(&hfdcan1, &motor[motor2_id], motor2_id, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f);
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  FDCAN2 接收回调：处理来自轮毂电机板的数据
  * @note   CAN ID 0x101 -> SBUS 遥控状态 [int16 ch1, ch3, ch5, ch6]
  */
void fdcan2_rx_callback(void)
{
    fdcanx_receive(&hfdcan2, &board_rx_id, board_rx_data);

    if (board_rx_id == 0x101) {
        /* 解码 SBUS 通道值 (小端序 int16) */
        can2_rx_ch[0] = (int16_t)(board_rx_data[0] | (board_rx_data[1] << 8));
        can2_rx_ch[1] = (int16_t)(board_rx_data[2] | (board_rx_data[3] << 8));
        can2_rx_ch[2] = (int16_t)(board_rx_data[4] | (board_rx_data[5] << 8));
        can2_rx_ch[3] = (int16_t)(board_rx_data[6] | (board_rx_data[7] << 8));
        can2_rx_tick = HAL_GetTick();
    }
}
/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */