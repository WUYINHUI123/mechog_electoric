#include "bsp_can2.h"
#include "bsp_can.h"
#include "bsp_sbus.h"
#include <string.h>

/* Relay timeout for the SBUS status frame sent to the leg board.
   Keep in sync with RC_ONLINE_TIMEOUT_MS in Core/Src/main.c. */
#define SBUS_RELAY_TIMEOUT_MS   300U

/* CH5 arm threshold: the 2000 side (normalised > +500) means "unlocked".
   Keep in sync with RC_ARM_THRESHOLD_NORM in Core/Src/main.c. */
#define RC_ARM_THRESHOLD_NORM   500

/* ========== Local state ========== */
static FDCAN_RxFrame_TypeDef   CAN2_RxFrame;
static FDCAN_TxFrame_TypeDef   CAN2_TxFrame;
static CAN2_MotionCmd_t        can2_motion_cmd;

/* ========== Initialization ========== */

void BSP_CAN2_Init(void)
{
    /* ---- Init TX frame template ---- */
    memset(&CAN2_TxFrame, 0, sizeof(CAN2_TxFrame));
    CAN2_TxFrame.hcan                    = &hfdcan2;
    CAN2_TxFrame.Header.IdType           = FDCAN_STANDARD_ID;
    CAN2_TxFrame.Header.TxFrameType      = FDCAN_DATA_FRAME;
    CAN2_TxFrame.Header.DataLength       = FDCAN_DLC_BYTES_8;
    CAN2_TxFrame.Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    CAN2_TxFrame.Header.BitRateSwitch    = FDCAN_BRS_OFF;
    CAN2_TxFrame.Header.FDFormat         = FDCAN_CLASSIC_CAN;
    CAN2_TxFrame.Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    CAN2_TxFrame.Header.MessageMarker    = 0;

    /* ---- RX state ---- */
    memset(&can2_motion_cmd, 0, sizeof(can2_motion_cmd));

    /* ---- Configure filter for CAN ID 0x100 (motion command) ---- */
    FDCAN_FilterTypeDef filter;
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = 0;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x7FF;          /* mask: match all 11 bits */
    filter.FilterID2    = CAN2_CMD_MOTION_ID; /* accept exactly 0x100 */
    HAL_FDCAN_ConfigFilter(&hfdcan2, &filter);

    /* ---- Global filter: reject non-matching ---- */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
        FDCAN_REJECT, FDCAN_REJECT,
        FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

    /* ---- Activate RX interrupt on FIFO0 ---- */
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    /* ---- Start CAN2 ---- */
    HAL_FDCAN_Start(&hfdcan2);
}

/* ========== Periodic processing (called from main loop) ========== */

void BSP_CAN2_Process(void)
{
    /* Nothing to do — reception is interrupt-driven.
       Add timeout / watchdog check here if needed. */
}

/* ========== Motion command accessor ========== */

const CAN2_MotionCmd_t* CAN2_GetMotionCmd(void)
{
    return &can2_motion_cmd;
}

/* ========== Send a CAN frame on CAN2 ========== */

HAL_StatusTypeDef CAN2_SendFrame(uint32_t id, const uint8_t *data)
{
    uint32_t start = HAL_GetTick();

    /* Wait for a free TX FIFO slot (2 ms timeout) */
    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0U)
    {
        if ((HAL_GetTick() - start) > 2U)
        {
            return HAL_TIMEOUT;
        }
    }

    CAN2_TxFrame.Header.Identifier = id;
    memcpy(CAN2_TxFrame.Data, data, 8);

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2,
        &CAN2_TxFrame.Header, CAN2_TxFrame.Data);
}

/* ========== Send SBUS status back to upper-body board ========== */

void CAN2_SendSbusStatus(void)
{
    const SBUS_Data_t *rc = SBUS_GetData();
    uint8_t data[8];

    int16_t turn_cmd;
    int16_t stride_cmd;
    int16_t status;
    int16_t reserved;

    /*
     * The leg board no longer parses RC channels: everything that depends on
     * the transmitter is decided HERE and sent down as a small semantic set.
     *   turn_cmd   : -1000..+1000, centre-deadbanded by SBUS_GetChannelNorm()
     *                (SBUS_NORM_DEADBAND). 0 = no turn.
     *   stride_cmd : -1000..+1000 forward/backward stride request (CH3).
     *   status     : bit0 = armed (CH5 on the 2000 side).
     *   reserved   : 0.
     * No valid SBUS frame / receiver in failsafe -> all zero, which the leg
     * board reads as "not armed, do not step" with a centred stick.
     */
    if ((SBUS_IsOnline(SBUS_RELAY_TIMEOUT_MS) == 0U) || (rc->failsafe != 0U))
    {
        turn_cmd   = 0;
        stride_cmd = 0;
        status     = 0;
    }
    else
    {
        turn_cmd   = SBUS_GetChannelNorm(0);   /* CH1 -> turn command */
        stride_cmd = SBUS_GetChannelNorm(2);   /* CH3 -> stride command */
        status     = (SBUS_GetChannelNorm(4) > RC_ARM_THRESHOLD_NORM) ? 1 : 0;  /* CH5 -> armed */
    }

    reserved = 0;

    data[0] = (uint8_t)(turn_cmd & 0xFF);
    data[1] = (uint8_t)((turn_cmd >> 8) & 0xFF);
    data[2] = (uint8_t)(stride_cmd & 0xFF);
    data[3] = (uint8_t)((stride_cmd >> 8) & 0xFF);
    data[4] = (uint8_t)(status & 0xFF);
    data[5] = (uint8_t)((status >> 8) & 0xFF);
    data[6] = (uint8_t)(reserved & 0xFF);
    data[7] = (uint8_t)((reserved >> 8) & 0xFF);

    CAN2_SendFrame(CAN2_SBUS_STATUS_ID, data);
}

/* ========== FDCAN2 RX FIFO0 handler (called from bsp_can.c shared callback) ========== */

void CAN2_RxFifo0RxHandler(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;  /* unused */

    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
        &CAN2_RxFrame.Header, CAN2_RxFrame.Data);

    /* Decode motion command (CAN ID 0x100) */
    if (CAN2_RxFrame.Header.Identifier == CAN2_CMD_MOTION_ID)
    {
        /* Data[0..3] = float forward_rpm */
        float fwd;
        memcpy(&fwd, &CAN2_RxFrame.Data[0], 4);

        /* Data[4..7] = float steer_norm */
        float steer;
        memcpy(&steer, &CAN2_RxFrame.Data[4], 4);

        can2_motion_cmd.forward_rpm    = fwd;
        can2_motion_cmd.steer_norm     = steer;
        can2_motion_cmd.updated        = 1U;
        can2_motion_cmd.last_update_ms = HAL_GetTick();
    }
}
