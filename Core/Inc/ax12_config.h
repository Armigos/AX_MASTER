#ifndef AX12_CONFIG_H
#define AX12_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ax12.h"

#define AX12_APP_TIMEOUT_MS        20U
#define AX12_BUS_BAUDRATE     1000000U
#define AX12_LEGACY_BAUDRATE   115200U
#define AX12_BAUD_VALUE_1MBPS       1U
#define HC05_LINK_BAUDRATE    115200U
#define HC05_LINK_PERIOD_MS       10U
#define AX12_MASTER_MOTOR_COUNT       4U
#define AX12_MASTER_1_ID              10U
#define AX12_MASTER_2_ID              11U
#define AX12_MASTER_3_ID              12U
#define AX12_MASTER_4_ID              14U
#define AX12_GOAL_MIN                 0U
#define AX12_GOAL_MAX              1023U
#define AX12_DEFAULT_GOAL           512U
#define AX12_DEFAULT_MOVING_SPEED   256U
#define AX12_SERIAL_LINE_SIZE       48U

typedef struct
{
  uint8_t id;
} AX12_MotorConfig;

extern const AX12_MotorConfig AX12_MASTER_MOTORS[AX12_MASTER_MOTOR_COUNT];

#ifdef __cplusplus
}
#endif

#endif
