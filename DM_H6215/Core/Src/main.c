/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "bsp_imu.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "bsp_can2.h"
#include "Motor.h"
#include "bsp_sbus.h"
#include "usbd_cdc_if.h"
#include <stdio.h>

//#include "sbus.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WHEEL_MOTOR_NUM        4U
#define WHEEL_TX_PERIOD_MS     5U
#define WHEEL_RAMP_STEP_RPM    0.4f
#define WHEEL_TEST_SPEED_RPM   10.0f

/*
 * Remote-control mode configuration.
 * Channel index is 0-based: CH1=0, CH2=1, CH3=2 ...
 */
 
 
#define RC_THROTTLE_CH         2U      /* CH3: 油门 */
#define RC_STEER_CH            0U      /* CH1: 转向 */
#define RC_ARM_CH              4U      /* CH5: arm switch, 2000 side (+1000) = unlocked */
#define RC_MODE_CH             5U      /* CH6: 1000 side = manual RC, 2000 side = Pi auto */
#define RC_VRA_CH              6U      /* CH7: VRa旋钮 → 速度限制器 (0=最慢, 2047=最快) */

#define RC_ONLINE_TIMEOUT_MS   300U
#define RC_DEBUG_PRINT_MS      200U

/* �����־����λ�ᵽ 100 ���ң�������������̫С */
#define RC_DEADBAND_NORM       180
#define RC_ARM_THRESHOLD_NORM  500

/* ��һ�治Ҫ̫�ͣ��ȴ� 45rpm ��ʼ */
#define RC_MAX_SPEED_RPM       60.0f

/* ԭ�� 0.60 ƫС������������ߵ� 1.00 */
#define RC_TURN_GAIN           1.00f
#define TURN_ASSIST_RPM        20.0f   /* CH1 turn assist: wheel differential rpm (low speed) */

/* ��ݮ���ٶ�ָ�� 300ms ����Ч����ʱ�Զ�ͣ */
#define PI_CMD_TIMEOUT_MS      1000U

/* IMU closed-loop control timing */
#define IMU_PROCESS_PERIOD_MS   5U

/* Max yaw rate at full steer stick (deg/s) */
#define RC_YAW_RATE_MAX_DPS     180.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t last_wheel_tx_ms = 0U;
static uint32_t last_rc_debug_ms = 0U;
static uint32_t last_imu_process_ms = 0U;
static uint32_t last_can2_sbus_tx_ms = 0U;
static uint8_t  imu_ready = 0U;   /* set to 1 only after Init+Calibration both succeed */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static const uint16_t WheelMotorId[WHEEL_MOTOR_NUM] = {
    0x09, 0x10, 0x11, 0x12
};

/*
 * Direction correction:
 * +1 means keep command direction.
 * -1 means reverse command direction.
 * 0x11 was reported reversed, so it is set to -1 by default.
 */
static const int8_t WheelMotorDir[WHEEL_MOTOR_NUM] = {
    +1,   // 0x09
    -1,   // 0x10
    -1,   // 0x11
    +1    // 0x12
};

/*
 * Differential-drive wheel grouping assumption:
 * 0x09 = left front, 0x10 = right front, 0x11 = right rear, 0x12 = left rear.
 * If your real wiring is different, only adjust WheelRemoteTargetUpdate().
 */
static float WheelTargetRpm[WHEEL_MOTOR_NUM] = {
    0.0f, 0.0f, 0.0f, 0.0f
};

static float WheelCmdRpm[WHEEL_MOTOR_NUM] = {
    0.0f, 0.0f, 0.0f, 0.0f
};

static volatile float PiTargetRpm[WHEEL_MOTOR_NUM] = {
    0.0f, 0.0f, 0.0f, 0.0f
};

static volatile uint32_t PiCmdLastMs = 0U;
static volatile uint8_t PiCmdValid = 0U;
static volatile uint8_t WheelSourceIsPi = 0U;




static float RampToTarget(float current, float target, float step)
{
    if (current < target)
    {
        current += step;
        if (current > target) current = target;
    }
    else if (current > target)
    {
        current -= step;
        if (current < target) current = target;
    }

    return current;
}

static float ClampFloat(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}
void WheelPiCommandSet(float fl_rpm, float fr_rpm, float rr_rpm, float rl_rpm)
{
    PiTargetRpm[0] = ClampFloat(fl_rpm, -RC_MAX_SPEED_RPM, RC_MAX_SPEED_RPM);
    PiTargetRpm[1] = ClampFloat(fr_rpm, -RC_MAX_SPEED_RPM, RC_MAX_SPEED_RPM);
    PiTargetRpm[2] = ClampFloat(rr_rpm, -RC_MAX_SPEED_RPM, RC_MAX_SPEED_RPM);
    PiTargetRpm[3] = ClampFloat(rl_rpm, -RC_MAX_SPEED_RPM, RC_MAX_SPEED_RPM);

    PiCmdLastMs = HAL_GetTick();
    PiCmdValid = 1U;
}


static int16_t ApplyDeadbandNorm(int16_t value)
{
    if (value > -RC_DEADBAND_NORM && value < RC_DEADBAND_NORM)
    {
        return 0;
    }

    return value;
}

/*
 * VRA Speed Limiter: read CH7 (VRa knob) and return speed scale 0.0f ~ 1.0f.
 * VRA min (full CCW) → 0.0f (motors capped at 0 rpm)
 * VRA max (full CW)  → 1.0f (full RC_MAX_SPEED_RPM available)
 * Applies deadband at both ends to avoid jitter.
 */
static float VRA_GetSpeedScale(void)
{
    int16_t vra = SBUS_GetChannelNorm(RC_VRA_CH);

    /*
     * CH7 is a 2-position switch, not a knob:
     *   1000 side (norm -1000) = HIGH speed -> 1.00
     *   2000 side (norm +1000) = LOW  speed -> 0.50
     * Anything else (unreadable / mid position) falls back to 1.00 so the
     * wheels are never locked out by this switch alone.
     */
    if (vra >= RC_ARM_THRESHOLD_NORM)
    {
        return 0.5f;
    }

    return 1.0f;
}

static void WheelMotorsEnable(void)
{
    for (uint32_t i = 0; i < WHEEL_MOTOR_NUM; i++)
    {
        DM_Motor_Enable(WheelMotorId[i]);
        HAL_Delay(50);
    }
}

static void WheelAllTargetStop(void)
{
    for (uint32_t i = 0; i < WHEEL_MOTOR_NUM; i++)
    {
        WheelTargetRpm[i] = 0.0f;
    }
}

static void WheelRemoteTargetUpdate(void)
{
    const SBUS_Data_t *rc = SBUS_GetData();
    uint8_t online = SBUS_IsOnline(RC_ONLINE_TIMEOUT_MS);

    if ((online == 0U) || (rc->failsafe != 0U))
    {
        WheelSourceIsPi = 0U;
        WheelAllTargetStop();
        return;
    }

    int16_t arm  = SBUS_GetChannelNorm(RC_ARM_CH);
    int16_t mode = SBUS_GetChannelNorm(RC_MODE_CH);

    /*
     * CH5 ���ܰ�ȫ������
     * δ����ʱ������ң����������ݮ�ɣ������������Ӷ���
     */
    if (arm < RC_ARM_THRESHOLD_NORM)
    {
        WheelSourceIsPi = 0U;
        WheelAllTargetStop();
        return;
    }

    uint32_t now = HAL_GetTick();
    uint8_t pi_online = (PiCmdValid != 0U) && ((now - PiCmdLastMs) < PI_CMD_TIMEOUT_MS);

    /*
     * CH6 高：树莓派自动模式。
     * VRA旋钮同样作为全局速度限制器。
     */
    if (mode > RC_ARM_THRESHOLD_NORM)
    {
        if (pi_online != 0U)
        {
            float vra_scale = VRA_GetSpeedScale();
            float effective_max_rpm = RC_MAX_SPEED_RPM * vra_scale;

            WheelSourceIsPi = 1U;

            /* Apply VRA speed limit to Pi targets */
            for (uint32_t i = 0; i < WHEEL_MOTOR_NUM; i++)
            {
                WheelTargetRpm[i] = ClampFloat(PiTargetRpm[i], -effective_max_rpm, effective_max_rpm);
            }

            /* Pi mode: apply heading-hold correction */
            if (IMU_IsClosedLoopEnabled())
            {
                float hh_correction = IMU_HeadingHold_Compute();
                WheelTargetRpm[0] += hh_correction;
                WheelTargetRpm[3] += hh_correction;
                WheelTargetRpm[1] -= hh_correction;
                WheelTargetRpm[2] -= hh_correction;

                for (uint32_t i = 0; i < WHEEL_MOTOR_NUM; i++)
                {
                    WheelTargetRpm[i] = ClampFloat(WheelTargetRpm[i], -effective_max_rpm, effective_max_rpm);
                }
            }
        }
        else
        {
            WheelSourceIsPi = 0U;
            WheelAllTargetStop();
        }

        return;
    }

    /*
     * CH6 低：遥控手动模式。
     * VRA旋钮 (CH7) 作为速度限制器:
     *   VRA=最小 → speed_scale=0.0 → 电机锁死
     *   VRA=50%  → speed_scale=0.5 → 最高速度减半
     *   VRA=最大 → speed_scale=1.0 → 全速
     */
    WheelSourceIsPi = 0U;

    int16_t throttle = ApplyDeadbandNorm(SBUS_GetChannelNorm(RC_THROTTLE_CH));
    int16_t steer    = ApplyDeadbandNorm(SBUS_GetChannelNorm(RC_STEER_CH));

    float vra_scale        = VRA_GetSpeedScale();
    float effective_max_rpm = RC_MAX_SPEED_RPM * vra_scale;
    float forward_rpm       = ((float)throttle / 1000.0f) * effective_max_rpm;
    float steer_norm        = ((float)steer / 1000.0f);

    /*
     * CH3 drives all four wheels at the same speed by itself (no steering).
     * CH1 adds a small counter-rotating pair so the wheels help the in-place
     * turn the legs are doing: low-speed differential, +/- TURN_ASSIST_RPM.
     *   CH1 only   -> pure counter-rotation, assists the in-place spin
     *   CH1 + CH3  -> travel plus the same assist
     */

    float turn_assist_rpm = steer_norm * TURN_ASSIST_RPM;

    float left_rpm  = forward_rpm + turn_assist_rpm;
    float right_rpm = forward_rpm - turn_assist_rpm;

    left_rpm  = ClampFloat(left_rpm,  -effective_max_rpm, effective_max_rpm);
    right_rpm = ClampFloat(right_rpm, -effective_max_rpm, effective_max_rpm);

    /*
     * Ĭ������˳��
     * 0x09 = ��ǰ
     * 0x10 = ��ǰ
     * 0x11 = �Һ�
     * 0x12 = ���
     */
    WheelTargetRpm[0] = left_rpm;
    WheelTargetRpm[1] = right_rpm;
    WheelTargetRpm[2] = right_rpm;
    WheelTargetRpm[3] = left_rpm;

    /* ---- IMU closed-loop yaw-rate correction ---- */
    if (IMU_IsClosedLoopEnabled())
    {
        float target_yaw_rate_dps = steer_norm * RC_YAW_RATE_MAX_DPS;
        float imu_correction_rpm = IMU_YawRatePID_Compute(target_yaw_rate_dps);

        if (steer == 0)
        {
            /* Stick centered: engage heading hold */
            imu_correction_rpm += IMU_HeadingHold_Compute();
        }
        else
        {
            /* Actively steering: update heading reference for later */
            IMU_ResetYawReference();
        }

        /* Apply correction as differential: +left side, -right side */
        WheelTargetRpm[0] += imu_correction_rpm;  /* LF */
        WheelTargetRpm[3] += imu_correction_rpm;  /* LR */
        WheelTargetRpm[1] -= imu_correction_rpm;  /* RF */
        WheelTargetRpm[2] -= imu_correction_rpm;  /* RR */

        /* Re-clamp all targets with VRA speed limit */
        for (uint32_t i = 0; i < WHEEL_MOTOR_NUM; i++)
        {
            WheelTargetRpm[i] = ClampFloat(WheelTargetRpm[i], -effective_max_rpm, effective_max_rpm);
        }
    }
}


static void WheelMotorsVelocityModeUpdate(void)
{
    for (uint32_t i = 0; i < WHEEL_MOTOR_NUM; i++)
    {
        WheelCmdRpm[i] = RampToTarget(WheelCmdRpm[i], WheelTargetRpm[i], WHEEL_RAMP_STEP_RPM);
        DM_Motor_SpeedRpm(WheelMotorId[i], (float)WheelMotorDir[i] * WheelCmdRpm[i]);
    }
}

static void RemoteDebugPrint(void)
{
    const SBUS_Data_t *rc = SBUS_GetData();
    uint8_t online = SBUS_IsOnline(RC_ONLINE_TIMEOUT_MS);

    static char msg[400];
    uint32_t now = HAL_GetTick();
    uint32_t pi_age = now - PiCmdLastMs;

    float vra_scale = VRA_GetSpeedScale();

    int n = snprintf(msg,
                     sizeof(msg),
                     "RC on=%u fs=%u frm=%lu ch1=%d ch2=%d ch3=%d ch4=%d ch5=%d ch6=%d ch7=%d vra=%.2f max=%.1f src=%s pi=%u age=%lu tgt=[%.1f %.1f %.1f %.1f] cmd=[%.1f %.1f %.1f %.1f] CAN2=%.1f,%.1f bc=%lu hc=%lu bad=%lu rst=%lu uerr=%lu\r\n",
                     online,
                     rc->failsafe,
                     (unsigned long)rc->frame_count,
                     SBUS_GetChannelNorm(0),
                     SBUS_GetChannelNorm(1),
                     SBUS_GetChannelNorm(2),
                     SBUS_GetChannelNorm(3),
                     SBUS_GetChannelNorm(4),
                     SBUS_GetChannelNorm(5),
                     SBUS_GetChannelNorm(6),
                     (double)vra_scale,
                     (double)(RC_MAX_SPEED_RPM * vra_scale),
                     WheelSourceIsPi ? "PI" : "RC",
                     PiCmdValid,
                     (unsigned long)pi_age,
                     WheelTargetRpm[0],
                     WheelTargetRpm[1],
                     WheelTargetRpm[2],
                     WheelTargetRpm[3],
                     WheelCmdRpm[0],
                     WheelCmdRpm[1],
                     WheelCmdRpm[2],
                     WheelCmdRpm[3],
                     (double)CAN2_GetMotionCmd()->forward_rpm,
                     (double)CAN2_GetMotionCmd()->steer_norm,
                     (unsigned long)SBUS_DebugByteCount(),
                     (unsigned long)SBUS_DebugHeaderCount(),
                     (unsigned long)SBUS_DebugBadFrameCount(),
                     (unsigned long)SBUS_DebugRestartCount(),
                     (unsigned long)SBUS_DebugUartError());

    if (n > 0)
    {
        CDC_Transmit_HS((uint8_t *)msg, (uint16_t)n);
    }

    /* IMU status (separate print, only when ready) */
    if (imu_ready)
    {
        static char imu_msg[128];
        int m = snprintf(imu_msg, sizeof(imu_msg),
                         "IMU yaw=%.1f yr=%.1f dps p=%.1f r=%.1f safe=%u\r\n",
                         (double)IMU_GetYawAngle_deg(),
                         (double)IMU_GetYawRate_dps(),
                         (double)IMU_GetPitch_deg(),
                         (double)IMU_GetRoll_deg(),
                         IMU_IsTiltSafe());
        if (m > 0)
        {
            CDC_Transmit_HS((uint8_t *)imu_msg, (uint16_t)m);
        }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_USART1_UART_Init();
  MX_UART5_Init();
  MX_USB_DEVICE_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  BSP_FDCAN_Init();
  BSP_CAN2_Init();

  /* Enable 5V power for HT-10A receiver */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_Delay(500);

  SBUS_Init();
  HAL_Delay(1000);

  /* Velocity Mode remote control: enable all four wheel hub motors once, but keep speed target at 0 until RC is online and armed. */
  WheelAllTargetStop();
  WheelMotorsEnable();

  last_wheel_tx_ms = HAL_GetTick();
  last_rc_debug_ms = HAL_GetTick();
  last_imu_process_ms = HAL_GetTick();
  last_can2_sbus_tx_ms = HAL_GetTick();
	
	
	  /* Initialize IMU */
  if (IMU_Init()) {
      /* IMU initialized successfully — run gyro calibration while stationary */
      HAL_Delay(100);
      if (IMU_CalibrateGyro(500)) {
          IMU_SetClosedLoopEnable(0U);
          IMU_SetTiltLimits(45.0f, 45.0f);
          imu_ready = 1U;
      }
  } else {
      /* IMU initialization failed — keep closed-loop disabled */
      imu_ready = 0U;
  }
	
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    /* Poll SBUS DMA buffer every loop; without this, the receiver data will never affect motor targets. */
    SBUS_Process();

    /* Check for incoming CAN2 motion commands (from upper-body board) */
    {
        const CAN2_MotionCmd_t *can2 = CAN2_GetMotionCmd();
        if (can2->updated)
        {
            /* Feed CAN2 command into Pi-target path (reuses Pi-mode infrastructure) */
            float left_rpm  = can2->forward_rpm + can2->steer_norm * RC_MAX_SPEED_RPM;
            float right_rpm = can2->forward_rpm - can2->steer_norm * RC_MAX_SPEED_RPM;
            WheelPiCommandSet(left_rpm, right_rpm, right_rpm, left_rpm);
            ((CAN2_MotionCmd_t *)can2)->updated = 0U;  /* consume */
        }
    }

    WheelRemoteTargetUpdate();

    /* SBUS status feedback to upper-body board via CAN2 (100 Hz) */
    if ((now - last_can2_sbus_tx_ms) >= 10U)
    {
        last_can2_sbus_tx_ms = now;
        CAN2_SendSbusStatus();
    }

    if ((now - last_wheel_tx_ms) >= WHEEL_TX_PERIOD_MS)
    {
        last_wheel_tx_ms = now;
        WheelMotorsVelocityModeUpdate();
    }

    if ((now - last_rc_debug_ms) >= RC_DEBUG_PRINT_MS)
    {
        last_rc_debug_ms = now;
        RemoteDebugPrint();
    }

		
	    /* ---- IMU closed-loop processing (200 Hz, same as motor update) ---- */
    if (imu_ready && ((now - last_imu_process_ms) >= IMU_PROCESS_PERIOD_MS))
    {
        last_imu_process_ms = now;
        IMU_Process();

        /* Tilt safety: stop all motors if robot tilts too far */
        if (!IMU_IsTiltSafe())
        {
            WheelAllTargetStop();
        }
    }

    /* IMU debug print (20 Hz, only when IMU is ready) */
    static uint32_t last_imu_print_ms = 0;
    if (imu_ready && ((now - last_imu_print_ms) >= 50U))
    {
        last_imu_print_ms = now;
        IMU_PrintProcessed();   //��������Ѿ��� USER CODE BEGIN 4 �ж�����
    }
  /* USER CODE END 3 */
}
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}
/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
