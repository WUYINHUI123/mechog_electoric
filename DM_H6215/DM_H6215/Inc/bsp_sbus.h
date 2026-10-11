#ifndef __BSP_SBUS_H__
#define __BSP_SBUS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usart.h"
#include <stdint.h>

#define SBUS_FRAME_LEN        25
#define SBUS_DMA_BUF_LEN      64
#define SBUS_CHANNEL_COUNT    16

typedef struct
{
  uint16_t raw[SBUS_CHANNEL_COUNT];   // SBUS 原始值，通常约 172~1811
  uint16_t us[SBUS_CHANNEL_COUNT];    // 转换后的 1000~2000us 风格值
  uint8_t  ch17;
  uint8_t  ch18;
  uint8_t  frame_lost;
  uint8_t  failsafe;
  uint32_t frame_count;
  uint32_t last_update_ms;
} SBUS_Data_t;

void SBUS_Init(void);
void SBUS_Process(void);

const SBUS_Data_t* SBUS_GetData(void);
uint8_t SBUS_IsOnline(uint32_t timeout_ms);

/**
 * @brief 获取通道归一化值
 * @param ch_index 0-based channel index, CH1=0, CH2=1 ...
 * @return -1000 ~ +1000, 中位约 0
 */
int16_t SBUS_GetChannelNorm(uint8_t ch_index);
uint16_t SBUS_DebugDmaPos(void);
uint16_t SBUS_DebugOldPos(void);
uint32_t SBUS_DebugByteCount(void);
uint32_t SBUS_DebugHeaderCount(void);
uint32_t SBUS_DebugBadFrameCount(void);
uint8_t  SBUS_DebugLastByte(void);
uint32_t SBUS_DebugUartError(void);
uint32_t SBUS_DebugUartISR(void);
uint32_t SBUS_DebugRestartCount(void);




#ifdef __cplusplus
}
#endif

#endif
