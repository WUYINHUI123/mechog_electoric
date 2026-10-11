#include "bsp_sbus.h"
#include <string.h>

extern UART_HandleTypeDef huart5;

/*
 * Centre dead-band applied inside SBUS_GetChannelNorm().
 * The HT-10A stick centre does not land exactly on 1500us after the raw->us
 * scaling done by sbus_raw_to_us(), so a resting stick still reads about +100.
 * Without this the leg board (SBUS_DEADBAND = 180 there) sees a resting stick
 * as steering input and starts stepping. Keep equal to RC_DEADBAND_NORM in
 * Core/Src/main.c and to SBUS_DEADBAND in the leg board Core/Src/main.c.
 */
#define SBUS_NORM_DEADBAND     180

/*
 * H7 注意：原 DMA 接收在部分 Keil 链接脚本下可能把缓冲区放到 DTCM，
 * DMA1 无法访问，表现为 HAL_UART_ERROR_DMA、byte=0。
 * 本版本改为 UART5 单字节中断接收，先把“有没有进字节”排清楚。
 */
static uint8_t sbus_rx_byte = 0;
static uint8_t sbus_frame[SBUS_FRAME_LEN];

static uint8_t sbus_frame_index = 0;

static SBUS_Data_t sbus_data;
static uint32_t sbus_byte_count = 0;
static uint32_t sbus_header_count = 0;
static uint32_t sbus_bad_frame_count = 0;
static uint8_t sbus_last_byte = 0;
static uint32_t sbus_restart_count = 0;
static uint32_t sbus_last_restart_ms = 0;




static uint16_t sbus_raw_to_us(uint16_t raw)
{
  /*
   * HT-10A SBUS 解码说明：
   * 通道值 = raw * 0.625，范围约 0~1120。
   * 这里转换成 1000~2000 风格，方便原来的 SBUS_GetChannelNorm() 继续使用。
   */
  uint32_t scaled = ((uint32_t)raw * 625U) / 1000U;

  if (scaled > 1120U) scaled = 1120U;

  return (uint16_t)(1000U + (scaled * 1000U) / 1120U);
}

static void sbus_decode_frame(uint8_t *buf)
{
  uint16_t *ch = sbus_data.raw;

  ch[0]  = ((buf[1]       | buf[2]  << 8)                         & 0x07FF);
  ch[1]  = ((buf[2]  >> 3 | buf[3]  << 5)                         & 0x07FF);
  ch[2]  = ((buf[3]  >> 6 | buf[4]  << 2 | buf[5]  << 10)          & 0x07FF);
  ch[3]  = ((buf[5]  >> 1 | buf[6]  << 7)                         & 0x07FF);
  ch[4]  = ((buf[6]  >> 4 | buf[7]  << 4)                         & 0x07FF);
  ch[5]  = ((buf[7]  >> 7 | buf[8]  << 1 | buf[9]  << 9)           & 0x07FF);
  ch[6]  = ((buf[9]  >> 2 | buf[10] << 6)                         & 0x07FF);
  ch[7]  = ((buf[10] >> 5 | buf[11] << 3)                         & 0x07FF);

  ch[8]  = ((buf[12]      | buf[13] << 8)                         & 0x07FF);
  ch[9]  = ((buf[13] >> 3 | buf[14] << 5)                         & 0x07FF);
  ch[10] = ((buf[14] >> 6 | buf[15] << 2 | buf[16] << 10)          & 0x07FF);
  ch[11] = ((buf[16] >> 1 | buf[17] << 7)                         & 0x07FF);
  ch[12] = ((buf[17] >> 4 | buf[18] << 4)                         & 0x07FF);
  ch[13] = ((buf[18] >> 7 | buf[19] << 1 | buf[20] << 9)           & 0x07FF);
  ch[14] = ((buf[20] >> 2 | buf[21] << 6)                         & 0x07FF);
  ch[15] = ((buf[21] >> 5 | buf[22] << 3)                         & 0x07FF);

  for (uint8_t i = 0; i < SBUS_CHANNEL_COUNT; i++)
  {
    sbus_data.us[i] = sbus_raw_to_us(sbus_data.raw[i]);
  }

  /*
   * HT-10A SBUS 解码说明：
   * tmpbuf[23] 为遥控器状态标志：
   *   0x03 = 正常
   *   0x0F = 失控
   */
  uint8_t status = buf[23];

  sbus_data.ch17 = 0;
  sbus_data.ch18 = 0;

  if (status == 0x03U)
  {
    sbus_data.frame_lost = 0;
    sbus_data.failsafe   = 0;
  }
  else if (status == 0x0FU)
  {
    sbus_data.frame_lost = 1;
    sbus_data.failsafe   = 1;
  }
  else
  {
    /* 兼容标准 SBUS flags，避免不同批次接收机状态字略有差异。 */
    sbus_data.frame_lost = (status & 0x04U) ? 1 : 0;
    sbus_data.failsafe   = (status & 0x08U) ? 1 : 0;
  }

  sbus_data.frame_count++;
  sbus_data.last_update_ms = HAL_GetTick();
}

static uint8_t sbus_is_valid_end_byte(uint8_t b)
{
  /*
   * ׼ SBUS  end byte  0x00
   *  S.BUS2 ͬջܳ 0x04/0x14/0x24/0x34
   * ʵſڼݡ
   */
  return (b == 0x00 || b == 0x04 || b == 0x14 || b == 0x24 || b == 0x34);
}

static void sbus_feed_byte(uint8_t byte)
{
  sbus_byte_count++;
  sbus_last_byte = byte;

  if (byte == 0x0F)
  {
    sbus_header_count++;
  }

  if (sbus_frame_index == 0)
  {
    if (byte == 0x0F)
    {
      sbus_frame[sbus_frame_index++] = byte;
    }
    return;
  }

  sbus_frame[sbus_frame_index++] = byte;

  if (sbus_frame_index >= SBUS_FRAME_LEN)
  {
    if (sbus_frame[0] == 0x0F && sbus_is_valid_end_byte(sbus_frame[24]))
    {
      sbus_decode_frame(sbus_frame);
    }
    else
    {
      sbus_bad_frame_count++;
    }

    sbus_frame_index = 0;
  }

  
}

static void sbus_clear_uart_errors(void)
{
  __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_PEF);
  __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_FEF);
  __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_NEF);
  __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_OREF);
  __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_IDLEF);

  huart5.ErrorCode = HAL_UART_ERROR_NONE;
}

static void sbus_restart_it_rx(void)
{
  HAL_UART_AbortReceive(&huart5);

  sbus_clear_uart_errors();

  sbus_frame_index = 0;
  sbus_rx_byte = 0;

  HAL_UART_Receive_IT(&huart5, &sbus_rx_byte, 1);

  sbus_restart_count++;
  sbus_last_restart_ms = HAL_GetTick();
}


void SBUS_Init(void)
{
  memset(&sbus_data, 0, sizeof(sbus_data));

  sbus_frame_index = 0;

  sbus_byte_count = 0;
  sbus_header_count = 0;
  sbus_bad_frame_count = 0;
  sbus_last_byte = 0;

  sbus_restart_count = 0;
  sbus_last_restart_ms = 0;

  sbus_restart_it_rx();
}

void SBUS_Process(void)
{
  uint32_t isr = huart5.Instance->ISR;

  if ((huart5.ErrorCode != HAL_UART_ERROR_NONE) ||
      ((isr & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE)) != 0U))
  {
    if ((HAL_GetTick() - sbus_last_restart_ms) > 20U)
    {
      sbus_restart_it_rx();
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5)
  {
    sbus_feed_byte(sbus_rx_byte);
    HAL_UART_Receive_IT(&huart5, &sbus_rx_byte, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5)
  {
    if ((HAL_GetTick() - sbus_last_restart_ms) > 5U)
    {
      sbus_restart_it_rx();
    }
  }
}

const SBUS_Data_t* SBUS_GetData(void)
{
  return &sbus_data;
}

uint8_t SBUS_IsOnline(uint32_t timeout_ms)
{
  if (sbus_data.frame_count == 0)
  {
    return 0;
  }

  return ((HAL_GetTick() - sbus_data.last_update_ms) <= timeout_ms) ? 1 : 0;
}

int16_t SBUS_GetChannelNorm(uint8_t ch_index)
{
  if (ch_index >= SBUS_CHANNEL_COUNT)
  {
    return 0;
  }

  int32_t v = (int32_t)sbus_data.us[ch_index] - 1500;

  /*
   * 1000~2000us ӳΪ -1000~+1000
   */
  v = v * 2;

  if (v > 1000) v = 1000;
  if (v < -1000) v = -1000;

  /*
   * Centre dead-band: swallow the stick-centre residual so the value relayed
   * to the leg board over CAN2 is a clean 0 when the stick is really centred.
   * Switch channels (CH5/CH6) sit near +/-1000 and are unaffected.
   */
  if (v > -SBUS_NORM_DEADBAND && v < SBUS_NORM_DEADBAND)
  {
    return 0;
  }

  return (int16_t)v;
}

uint16_t SBUS_DebugDmaPos(void)
{
  /* 本版本不用 DMA，这里固定返回 0。 */
  return 0;
}

uint16_t SBUS_DebugOldPos(void)
{
  /* 本版本不用 DMA，这里固定返回 0。 */
  return 0;
}

uint32_t SBUS_DebugByteCount(void)
{
  return sbus_byte_count;
}

uint32_t SBUS_DebugHeaderCount(void)
{
  return sbus_header_count;
}

uint32_t SBUS_DebugBadFrameCount(void)
{
  return sbus_bad_frame_count;
}

uint8_t SBUS_DebugLastByte(void)
{
  return sbus_last_byte;
}
uint32_t SBUS_DebugUartError(void)
{
  return huart5.ErrorCode;
}

uint32_t SBUS_DebugUartISR(void)
{
  return huart5.Instance->ISR;
}


uint32_t SBUS_DebugRestartCount(void)
{
  return sbus_restart_count;
}
