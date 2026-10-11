#ifndef __BSP_CAN2_H__
#define __BSP_CAN2_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "fdcan.h"
#include <stdint.h>

/* ========== Inter-board Communication Protocol ========== */
/*
 * CAN2 is used for communication between the upper-body board
 * (8x DM4310 joint motors on CAN1) and the leg board
 * (4x DM hub motors 0x09-0x12 on CAN1).
 *
 * Motion Command (upper-body → legs):  CAN ID 0x100
 *   Data[0..3] = float forward_rpm  (target forward speed in RPM)
 *   Data[4..7] = float steer_norm   (normalised -1.0 .. +1.0)
 *
 * Semantic command (wheel board -> joint board): CAN ID 0x101
 *   Data[0..1] = int16_t turn_cmd    (-1000..+1000, deadbanded by wheel board, 0 = no turn)
 *   Data[2..3] = int16_t stride_cmd  (-1000..+1000, deadbanded; fwd/back stride)
 *   Data[4..5] = int16_t status      (bit0 = armed; CH5 on the 2000 side)
 *   Data[6..7] = int16_t reserved    (always 0)
 */

#define CAN2_CMD_MOTION_ID      0x100U
#define CAN2_SBUS_STATUS_ID     0x101U

/* ========== Received motion command ========== */
typedef struct
{
    float   forward_rpm;      /* target forward speed (RPM) */
    float   steer_norm;       /* normalised steer (-1.0 .. +1.0) */
    uint8_t updated;          /* set to 1 when new data arrives; consumer clears to 0 */
    uint32_t last_update_ms; /* HAL_GetTick() when last valid frame received */
} CAN2_MotionCmd_t;

/* ========== SBUS status to send back to upper-body ========== */
typedef struct
{
    int16_t turn_cmd;         /* -1000..+1000, deadbanded; 0 = no turn */
    int16_t stride_cmd;       /* -1000..+1000, deadbanded; fwd/back stride */
    int16_t status;           /* bit0 = armed */
    int16_t reserved;         /* always 0 */
} CAN2_SbusStatus_t;

/* ========== Public API ========== */

/**
 * @brief  Initialize CAN2 for inter-board communication.
 *         Configures filter to accept CAN ID 0x100, starts peripheral,
 *         activates RX interrupt.
 */
void BSP_CAN2_Init(void);

/**
 * @brief  Process CAN2 events (call periodically from main loop).
 *         Currently a no-op; reception is interrupt-driven.
 */
void BSP_CAN2_Process(void);

/**
 * @brief  Get pointer to the latest received motion command.
 *         Check .updated flag — if 1, new data is available.
 *         Consumer should set .updated = 0 after reading.
 */
const CAN2_MotionCmd_t* CAN2_GetMotionCmd(void);

/**
 * @brief  Send a CAN frame on CAN2.
 * @param  id       Standard CAN ID (11-bit)
 * @param  data     Pointer to 8 data bytes
 * @retval HAL_OK on success
 */
HAL_StatusTypeDef CAN2_SendFrame(uint32_t id, const uint8_t *data);

/**
 * @brief  Send SBUS RC status back to upper-body board on CAN ID 0x101.
 *         Should be called periodically (e.g. every 10 ms).
 */
void CAN2_SendSbusStatus(void);

/**
 * @brief  RX FIFO0 handler for FDCAN2 (called from shared HAL callback).
 */
void CAN2_RxFifo0RxHandler(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_CAN2_H__ */
