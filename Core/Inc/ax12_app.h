#ifndef AX12_APP_H
#define AX12_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ax12_config.h"
#include <stdbool.h>

typedef enum
{
  AX12_MODE_ADMIN = 0,
  AX12_MODE_CONTROL
} AX12_AppMode;

typedef struct
{
  AX12_Handle ax12;
  UART_HandleTypeDef *link_uart;
  AX12_AppMode mode;
  uint16_t motor_goal[AX12_MASTER_MOTOR_COUNT];
  uint16_t motor_present[AX12_MASTER_MOTOR_COUNT];
  bool motor_torque_enabled[AX12_MASTER_MOTOR_COUNT];
  int16_t control_offset;
  bool ready;
  uint8_t link_sequence;
  uint32_t last_link_tx_ms;
  char serial_line[AX12_SERIAL_LINE_SIZE];
  uint8_t serial_line_len;
  volatile bool serial_line_ready;
} AX12_AppState;

bool AX12_AppInit(AX12_AppState *app, UART_HandleTypeDef *ax12_uart,
                  UART_HandleTypeDef *link_uart);
bool AX12_AppStartConsoleRx(AX12_AppState *app, UART_HandleTypeDef *console_uart);
bool AX12_AppProcessSerial(AX12_AppState *app, UART_HandleTypeDef *console_uart);
void AX12_AppUpdate(AX12_AppState *app);
void AX12_AppUartRxCpltCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
