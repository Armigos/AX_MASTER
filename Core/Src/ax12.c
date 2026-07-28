#include "ax12.h"

/* Dynamixel Protocol 1.0 instruction values used by this driver. */
#define AX12_HEADER             0xFFU
#define AX12_INSTRUCTION_PING   0x01U
#define AX12_INSTRUCTION_READ   0x02U
#define AX12_INSTRUCTION_WRITE  0x03U
#define AX12_MAX_PARAMS         16U

static uint8_t AX12_Checksum(const uint8_t *data, uint8_t length)
{
  uint8_t sum = 0U;

  /* Protocol 1.0 checksum is the inverted low byte of the field sum. */
  for (uint8_t i = 0U; i < length; ++i)
  {
    sum = (uint8_t)(sum + data[i]);
  }

  return (uint8_t)(~sum);
}

static uint32_t AX12_RemainingTime(uint32_t started_at, uint32_t timeout_ms)
{
  /* Unsigned subtraction remains valid when the HAL tick counter wraps. */
  uint32_t elapsed = HAL_GetTick() - started_at;

  return (elapsed < timeout_ms) ? (timeout_ms - elapsed) : 0U;
}

static AX12_Result AX12_ReceiveStatus(AX12_Handle *ax12, uint8_t expected_id,
                                     uint8_t *params, uint8_t params_capacity,
                                     uint8_t *received_count)
{
  uint8_t byte = 0U;
  uint8_t previous = 0U;
  uint8_t status[AX12_MAX_PARAMS + 4U];
  uint32_t started_at = HAL_GetTick();
  uint32_t remaining;

  /*
   * Status packet:
   *   FF FF | ID | LENGTH | ERROR | PARAMS... | CHECKSUM
   *
   * Search for the header first so a stale or echoed byte cannot shift every
   * following field.
   */
  do
  {
    remaining = AX12_RemainingTime(started_at, ax12->timeout_ms);
    if ((remaining == 0U) ||
        (HAL_UART_Receive(ax12->uart, &byte, 1U, remaining) != HAL_OK))
    {
      return AX12_ERROR_TIMEOUT;
    }

    if ((previous == AX12_HEADER) && (byte == AX12_HEADER))
    {
      break;
    }
    previous = byte;
  } while (true);

  remaining = AX12_RemainingTime(started_at, ax12->timeout_ms);
  if ((remaining == 0U) ||
      (HAL_UART_Receive(ax12->uart, status, 2U, remaining) != HAL_OK))
  {
    return AX12_ERROR_TIMEOUT;
  }

  uint8_t response_id = status[0];
  uint8_t response_length = status[1];
  /* LENGTH includes ERROR, PARAMS, and CHECKSUM, but not ID or LENGTH. */
  if ((response_id != expected_id) ||
      (response_length < 2U) ||
      (response_length > (AX12_MAX_PARAMS + 2U)))
  {
    return AX12_ERROR_PACKET;
  }

  remaining = AX12_RemainingTime(started_at, ax12->timeout_ms);
  if ((remaining == 0U) ||
      (HAL_UART_Receive(ax12->uart, &status[2], response_length, remaining) != HAL_OK))
  {
    return AX12_ERROR_TIMEOUT;
  }

  uint8_t checksum_data[AX12_MAX_PARAMS + 3U];
  /* The checksum covers ID through the final parameter, not the FF headers. */
  checksum_data[0] = response_id;
  checksum_data[1] = response_length;
  for (uint8_t i = 0U; i < (response_length - 1U); ++i)
  {
    checksum_data[i + 2U] = status[i + 2U];
  }

  if (AX12_Checksum(checksum_data, (uint8_t)(response_length + 1U)) !=
      status[response_length + 1U])
  {
    return AX12_ERROR_CHECKSUM;
  }

  ax12->last_device_error = status[2];
  if (ax12->last_device_error != 0U)
  {
    return AX12_ERROR_DEVICE;
  }

  /* Remove ERROR and CHECKSUM from LENGTH to obtain the payload size. */
  uint8_t param_count = (uint8_t)(response_length - 2U);
  if (received_count != NULL)
  {
    *received_count = param_count;
  }
  if (param_count > 0U)
  {
    if ((params == NULL) || (param_count > params_capacity))
    {
      return AX12_ERROR_PACKET;
    }
    for (uint8_t i = 0U; i < param_count; ++i)
    {
      params[i] = status[i + 3U];
    }
  }

  return AX12_OK;
}

static AX12_Result AX12_SendInstruction(AX12_Handle *ax12, uint8_t id,
                                       uint8_t instruction,
                                       const uint8_t *params,
                                       uint8_t param_count,
                                       uint8_t *response_params,
                                       uint8_t response_capacity,
                                       uint8_t *response_count)
{
  uint8_t packet[AX12_MAX_PARAMS + 6U];
  uint8_t checksum_data[AX12_MAX_PARAMS + 3U];

  if ((ax12 == NULL) || (ax12->uart == NULL) ||
      (param_count > AX12_MAX_PARAMS) ||
      ((param_count > 0U) && (params == NULL)))
  {
    return AX12_ERROR_ARGUMENT;
  }

  /*
   * Instruction packet:
   *   FF FF | ID | LENGTH | INSTRUCTION | PARAMS... | CHECKSUM
   */
  packet[0] = AX12_HEADER;
  packet[1] = AX12_HEADER;
  packet[2] = id;
  packet[3] = (uint8_t)(param_count + 2U);
  packet[4] = instruction;

  checksum_data[0] = packet[2];
  checksum_data[1] = packet[3];
  checksum_data[2] = packet[4];
  for (uint8_t i = 0U; i < param_count; ++i)
  {
    packet[i + 5U] = params[i];
    checksum_data[i + 3U] = params[i];
  }
  packet[param_count + 5U] =
      AX12_Checksum(checksum_data, (uint8_t)(param_count + 3U));

  ax12->last_device_error = 0U;
  /* Discard a stale byte before changing the shared DATA line to TX mode. */
  __HAL_UART_FLUSH_DRREGISTER(ax12->uart);

  /* The single UART pin must drive the bus only while sending the command. */
  if (HAL_HalfDuplex_EnableTransmitter(ax12->uart) != HAL_OK)
  {
    return AX12_ERROR_UART;
  }
  if (HAL_UART_Transmit(ax12->uart, packet, (uint16_t)(param_count + 6U),
                        ax12->timeout_ms) != HAL_OK)
  {
    return AX12_ERROR_UART;
  }

  if (id == AX12_BROADCAST_ID)
  {
    /* Broadcast instructions never produce a status packet. */
    return AX12_OK;
  }

  /* Release the DATA line and listen for the selected motor's status packet. */
  if (HAL_HalfDuplex_EnableReceiver(ax12->uart) != HAL_OK)
  {
    return AX12_ERROR_UART;
  }

  return AX12_ReceiveStatus(ax12, id, response_params,
                            response_capacity, response_count);
}

void AX12_Init(AX12_Handle *ax12, UART_HandleTypeDef *uart, uint32_t timeout_ms)
{
  if (ax12 == NULL)
  {
    return;
  }

  ax12->uart = uart;
  ax12->timeout_ms = timeout_ms;
  ax12->last_device_error = 0U;
}

uint8_t AX12_GetLastDeviceError(const AX12_Handle *ax12)
{
  return (ax12 != NULL) ? ax12->last_device_error : 0U;
}

AX12_Result AX12_Ping(AX12_Handle *ax12, uint8_t id)
{
  return AX12_SendInstruction(ax12, id, AX12_INSTRUCTION_PING, NULL, 0U,
                              NULL, 0U, NULL);
}

AX12_Result AX12_WriteByte(AX12_Handle *ax12, uint8_t id,
                          uint8_t address, uint8_t value)
{
  const uint8_t params[] = {address, value};

  return AX12_SendInstruction(ax12, id, AX12_INSTRUCTION_WRITE,
                              params, sizeof(params), NULL, 0U, NULL);
}

AX12_Result AX12_WriteWord(AX12_Handle *ax12, uint8_t id,
                          uint8_t address, uint16_t value)
{
  /* AX-12 control-table words use low byte first. */
  const uint8_t params[] = {
      address,
      (uint8_t)(value & 0xFFU),
      (uint8_t)((value >> 8U) & 0xFFU)
  };

  return AX12_SendInstruction(ax12, id, AX12_INSTRUCTION_WRITE,
                              params, sizeof(params), NULL, 0U, NULL);
}

AX12_Result AX12_ReadByte(AX12_Handle *ax12, uint8_t id,
                         uint8_t address, uint8_t *value)
{
  const uint8_t params[] = {address, 1U};
  uint8_t received_count = 0U;

  if (value == NULL)
  {
    return AX12_ERROR_ARGUMENT;
  }

  AX12_Result result =
      AX12_SendInstruction(ax12, id, AX12_INSTRUCTION_READ,
                           params, sizeof(params), value, 1U, &received_count);

  return ((result == AX12_OK) && (received_count != 1U))
             ? AX12_ERROR_PACKET
             : result;
}

AX12_Result AX12_ReadWord(AX12_Handle *ax12, uint8_t id,
                         uint8_t address, uint16_t *value)
{
  const uint8_t params[] = {address, 2U};
  uint8_t response[2];
  uint8_t received_count = 0U;

  if (value == NULL)
  {
    return AX12_ERROR_ARGUMENT;
  }

  AX12_Result result =
      AX12_SendInstruction(ax12, id, AX12_INSTRUCTION_READ,
                           params, sizeof(params), response, sizeof(response),
                           &received_count);
  if (result != AX12_OK)
  {
    return result;
  }
  if (received_count != 2U)
  {
    return AX12_ERROR_PACKET;
  }

  /* Reassemble the little-endian control-table word. */
  *value = (uint16_t)response[0] | ((uint16_t)response[1] << 8U);
  return AX12_OK;
}

AX12_Result AX12_SetId(AX12_Handle *ax12, uint8_t id, uint8_t new_id)
{
  return AX12_WriteByte(ax12, id, AX12_ADDR_ID, new_id);
}

AX12_Result AX12_SetBaudRate(AX12_Handle *ax12, uint8_t id, uint8_t baud_value)
{
  return AX12_WriteByte(ax12, id, AX12_ADDR_BAUD_RATE, baud_value);
}

AX12_Result AX12_SetReturnDelayTime(AX12_Handle *ax12, uint8_t id, uint8_t delay_time)
{
  return AX12_WriteByte(ax12, id, AX12_ADDR_RETURN_DELAY, delay_time);
}

AX12_Result AX12_SetPositionMode(AX12_Handle *ax12, uint8_t id)
{
  /* Joint mode uses the full AX-12A position range: 0 through 1023. */
  AX12_Result result = AX12_WriteWord(ax12, id, AX12_ADDR_CW_LIMIT, 0U);
  if (result != AX12_OK)
  {
    return result;
  }

  return AX12_WriteWord(ax12, id, AX12_ADDR_CCW_LIMIT, 1023U);
}

AX12_Result AX12_SetWheelMode(AX12_Handle *ax12, uint8_t id)
{
  /* Setting both angle limits to zero selects continuous wheel mode. */
  AX12_Result result = AX12_WriteWord(ax12, id, AX12_ADDR_CW_LIMIT, 0U);
  if (result != AX12_OK)
  {
    return result;
  }

  return AX12_WriteWord(ax12, id, AX12_ADDR_CCW_LIMIT, 0U);
}

AX12_Result AX12_SetLed(AX12_Handle *ax12, uint8_t id, bool enabled)
{
  return AX12_WriteByte(ax12, id, AX12_ADDR_LED, enabled ? 1U : 0U);
}

AX12_Result AX12_SetTorque(AX12_Handle *ax12, uint8_t id, bool enabled)
{
  return AX12_WriteByte(ax12, id, AX12_ADDR_TORQUE_ENABLE, enabled ? 1U : 0U);
}

AX12_Result AX12_SetGoalPosition(AX12_Handle *ax12, uint8_t id,
                                uint16_t position)
{
  /* AX-12A position values are 10-bit values. */
  if (position > 1023U)
  {
    return AX12_ERROR_ARGUMENT;
  }

  return AX12_WriteWord(ax12, id, AX12_ADDR_GOAL_POSITION, position);
}

AX12_Result AX12_SetMovingSpeed(AX12_Handle *ax12, uint8_t id,
                               uint16_t speed)
{
  /* Moving Speed also uses the AX-12A 10-bit control-table range. */
  if (speed > 1023U)
  {
    return AX12_ERROR_ARGUMENT;
  }

  return AX12_WriteWord(ax12, id, AX12_ADDR_MOVING_SPEED, speed);
}

AX12_Result AX12_GetId(AX12_Handle *ax12, uint8_t id, uint8_t *value)
{
  return AX12_ReadByte(ax12, id, AX12_ADDR_ID, value);
}

AX12_Result AX12_GetBaudRate(AX12_Handle *ax12, uint8_t id, uint8_t *value)
{
  return AX12_ReadByte(ax12, id, AX12_ADDR_BAUD_RATE, value);
}

AX12_Result AX12_GetReturnDelayTime(AX12_Handle *ax12, uint8_t id, uint8_t *value)
{
  return AX12_ReadByte(ax12, id, AX12_ADDR_RETURN_DELAY, value);
}

AX12_Result AX12_GetModelNumber(AX12_Handle *ax12, uint8_t id, uint16_t *model)
{
  return AX12_ReadWord(ax12, id, AX12_ADDR_MODEL_NUMBER, model);
}

AX12_Result AX12_GetFirmwareVersion(AX12_Handle *ax12, uint8_t id, uint8_t *version)
{
  return AX12_ReadByte(ax12, id, AX12_ADDR_FIRMWARE_VER, version);
}

AX12_Result AX12_GetPresentPosition(AX12_Handle *ax12, uint8_t id,
                                   uint16_t *position)
{
  return AX12_ReadWord(ax12, id, AX12_ADDR_PRESENT_POSITION, position);
}
