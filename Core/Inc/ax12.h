#ifndef AX12_H
#define AX12_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* AX-12 / AX-12A uses Dynamixel Protocol 1.0 */
#define AX12_PROTOCOL_1_0          1U

#define AX12_HEADER                0xFFU
#define AX12_BROADCAST_ID          0xFEU
#define AX12_DEFAULT_ID            2U

/* Control table addresses for AX-12A */
#define AX12_ADDR_MODEL_NUMBER     0U
#define AX12_ADDR_FIRMWARE_VER     2U
#define AX12_ADDR_ID               3U
#define AX12_ADDR_BAUD_RATE        4U
#define AX12_ADDR_RETURN_DELAY     5U
#define AX12_ADDR_CW_LIMIT         6U
#define AX12_ADDR_CCW_LIMIT        8U
#define AX12_ADDR_TEMP_LIMIT       11U
#define AX12_ADDR_MIN_VOLTAGE      12U
#define AX12_ADDR_MAX_VOLTAGE      13U
#define AX12_ADDR_MAX_TORQUE       14U
#define AX12_ADDR_STATUS_LEVEL     16U
#define AX12_ADDR_ALARM_LED        17U
#define AX12_ADDR_SHUTDOWN         18U
#define AX12_ADDR_TORQUE_ENABLE    24U
#define AX12_ADDR_LED              25U
#define AX12_ADDR_GOAL_POSITION    30U
#define AX12_ADDR_MOVING_SPEED     32U
#define AX12_ADDR_PRESENT_POSITION 36U

typedef enum
{
  AX12_OK = 0,
  AX12_ERROR_ARGUMENT,
  AX12_ERROR_UART,
  AX12_ERROR_TIMEOUT,
  AX12_ERROR_PACKET,
  AX12_ERROR_CHECKSUM,
  AX12_ERROR_DEVICE
} AX12_Result;

typedef struct
{
  UART_HandleTypeDef *uart;
  uint32_t timeout_ms;
  uint8_t last_device_error;
} AX12_Handle;

void AX12_Init(AX12_Handle *ax12, UART_HandleTypeDef *uart, uint32_t timeout_ms);
uint8_t AX12_GetLastDeviceError(const AX12_Handle *ax12);

AX12_Result AX12_Ping(AX12_Handle *ax12, uint8_t id);

AX12_Result AX12_WriteByte(AX12_Handle *ax12, uint8_t id,
                           uint8_t address, uint8_t value);
AX12_Result AX12_WriteWord(AX12_Handle *ax12, uint8_t id,
                           uint8_t address, uint16_t value);

AX12_Result AX12_ReadByte(AX12_Handle *ax12, uint8_t id,
                          uint8_t address, uint8_t *value);
AX12_Result AX12_ReadWord(AX12_Handle *ax12, uint8_t id,
                          uint8_t address, uint16_t *value);

/* AX-12 convenience wrappers */
AX12_Result AX12_SetId(AX12_Handle *ax12, uint8_t id, uint8_t new_id);
AX12_Result AX12_SetBaudRate(AX12_Handle *ax12, uint8_t id, uint8_t baud_value);
AX12_Result AX12_SetReturnDelayTime(AX12_Handle *ax12, uint8_t id, uint8_t delay_time);
AX12_Result AX12_SetPositionMode(AX12_Handle *ax12, uint8_t id);
AX12_Result AX12_SetWheelMode(AX12_Handle *ax12, uint8_t id);
AX12_Result AX12_SetLed(AX12_Handle *ax12, uint8_t id, bool enabled);
AX12_Result AX12_SetTorque(AX12_Handle *ax12, uint8_t id, bool enabled);
AX12_Result AX12_SetGoalPosition(AX12_Handle *ax12, uint8_t id, uint16_t position);
AX12_Result AX12_SetMovingSpeed(AX12_Handle *ax12, uint8_t id, uint16_t speed);

AX12_Result AX12_GetId(AX12_Handle *ax12, uint8_t id, uint8_t *value);
AX12_Result AX12_GetBaudRate(AX12_Handle *ax12, uint8_t id, uint8_t *value);
AX12_Result AX12_GetReturnDelayTime(AX12_Handle *ax12, uint8_t id, uint8_t *value);
AX12_Result AX12_GetModelNumber(AX12_Handle *ax12, uint8_t id, uint16_t *model);
AX12_Result AX12_GetFirmwareVersion(AX12_Handle *ax12, uint8_t id, uint8_t *version);
AX12_Result AX12_GetPresentPosition(AX12_Handle *ax12, uint8_t id, uint16_t *position);

#ifdef __cplusplus
}
#endif

#endif
