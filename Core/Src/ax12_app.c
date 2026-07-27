#include "ax12_app.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static AX12_AppState *s_console_app = NULL;
static UART_HandleTypeDef *s_console_uart = NULL;
static uint8_t s_console_rx_byte = 0U;

static const char *AX12_SkipSpaces(const char *s)
{
  if (s == NULL)
  {
    return NULL;
  }

  while ((*s == ' ') || (*s == '\t'))
  {
    ++s;
  }

  return s;
}

static bool AX12_ReadToken(const char **line, char *token, size_t token_size)
{
  size_t len = 0U;
  const char *s;

  if ((line == NULL) || (token == NULL) || (token_size == 0U))
  {
    return false;
  }

  s = AX12_SkipSpaces(*line);
  if ((s == NULL) || (*s == '\0'))
  {
    return false;
  }

  while ((*s != '\0') && (*s != ' ') && (*s != '\t'))
  {
    if ((len + 1U) >= token_size)
    {
      return false;
    }
    token[len++] = *s++;
  }

  token[len] = '\0';
  *line = AX12_SkipSpaces(s);
  return true;
}

static bool AX12_ParseUnsigned16(const char *line, uint16_t *value)
{
  char *end = NULL;
  unsigned long parsed;

  if ((line == NULL) || (value == NULL))
  {
    return false;
  }

  line = AX12_SkipSpaces(line);
  if ((line == NULL) || (*line == '\0'))
  {
    return false;
  }

  parsed = strtoul(line, &end, 10);
  if (end == line)
  {
    return false;
  }

  end = (char *)AX12_SkipSpaces(end);
  if ((end == NULL) || (*end != '\0'))
  {
    return false;
  }

  if (parsed > AX12_GOAL_MAX)
  {
    return false;
  }

  *value = (uint16_t)parsed;
  return true;
}

static bool AX12_ParseSigned16(const char *line, int16_t *value)
{
  char *end = NULL;
  long parsed;

  if ((line == NULL) || (value == NULL))
  {
    return false;
  }

  line = AX12_SkipSpaces(line);
  if ((line == NULL) || (*line == '\0'))
  {
    return false;
  }

  parsed = strtol(line, &end, 10);
  if (end == line)
  {
    return false;
  }

  end = (char *)AX12_SkipSpaces(end);
  if ((end == NULL) || (*end != '\0'))
  {
    return false;
  }

  if ((parsed < -1023L) || (parsed > 1023L))
  {
    return false;
  }

  *value = (int16_t)parsed;
  return true;
}

static bool AX12_ParseMotorId(const char *line, uint8_t *motor_id)
{
  char *end = NULL;
  unsigned long parsed;

  if ((line == NULL) || (motor_id == NULL))
  {
    return false;
  }

  line = AX12_SkipSpaces(line);
  if ((line == NULL) || (*line == '\0'))
  {
    return false;
  }

  parsed = strtoul(line, &end, 10);
  if (end == line)
  {
    return false;
  }

  end = (char *)AX12_SkipSpaces(end);
  if ((end == NULL) || (*end != '\0'))
  {
    return false;
  }

  if ((parsed < 1UL) || (parsed > 255UL))
  {
    return false;
  }

  *motor_id = (uint8_t)parsed;
  return true;
}

static bool AX12_ParseOnOff(const char *line, bool *enabled)
{
  char token[16];
  const char *rest = line;

  if ((enabled == NULL) || !AX12_ReadToken(&rest, token, sizeof(token)))
  {
    return false;
  }

  if (strcmp(token, "on") == 0)
  {
    *enabled = true;
    return true;
  }

  if (strcmp(token, "off") == 0)
  {
    *enabled = false;
    return true;
  }

  return false;
}

static uint16_t AX12_ClampGoal(int32_t goal)
{
  if (goal < (int32_t)AX12_GOAL_MIN)
  {
    return AX12_GOAL_MIN;
  }

  if (goal > (int32_t)AX12_GOAL_MAX)
  {
    return AX12_GOAL_MAX;
  }

  return (uint16_t)goal;
}

static bool AX12_GetMotorIndex(uint8_t motor_id, uint8_t *index)
{
  if (index == NULL)
  {
    return false;
  }

  for (uint8_t i = 0U; i < AX12_MASTER_MOTOR_COUNT; ++i)
  {
    if (AX12_MASTER_MOTORS[i].id == motor_id)
    {
      *index = i;
      return true;
    }
  }

  return false;
}

static bool AX12_ReadAllPresentPositions(AX12_AppState *app)
{
  bool all_ok = true;

  if (app == NULL)
  {
    return false;
  }

  for (uint8_t i = 0U; i < AX12_MASTER_MOTOR_COUNT; ++i)
  {
    uint16_t position = 0U;

    if (AX12_GetPresentPosition(&app->ax12, AX12_MASTER_MOTORS[i].id, &position) == AX12_OK)
    {
      app->motor_present[i] = position;
    }
    else
    {
      all_ok = false;
    }
  }

  return all_ok;
}

static bool AX12_SendMasterFrame(AX12_AppState *app)
{
  uint8_t frame[2U + 3U + (AX12_MASTER_MOTOR_COUNT * 3U) + 1U];
  uint8_t idx = 0U;
  uint8_t checksum = 0U;

  if ((app == NULL) || (app->link_uart == NULL))
  {
    return false;
  }

  frame[idx++] = 0xA5U;
  frame[idx++] = 0x5AU;
  frame[idx++] = 0x01U;
  frame[idx++] = app->link_sequence++;
  frame[idx++] = AX12_MASTER_MOTOR_COUNT;

  for (uint8_t i = 0U; i < AX12_MASTER_MOTOR_COUNT; ++i)
  {
    frame[idx++] = AX12_MASTER_MOTORS[i].id;
    frame[idx++] = (uint8_t)(app->motor_present[i] & 0xFFU);
    frame[idx++] = (uint8_t)((app->motor_present[i] >> 8U) & 0xFFU);
  }

  for (uint8_t i = 2U; i < idx; ++i)
  {
    checksum ^= frame[i];
  }

  frame[idx++] = checksum;
  return (HAL_UART_Transmit(app->link_uart, frame, idx, AX12_APP_TIMEOUT_MS) == HAL_OK);
}

static void AX12_PrintStatus(const AX12_AppState *app)
{
  if (app == NULL)
  {
    return;
  }

  printf("master status: mode=%s\r\n",
         (app->mode == AX12_MODE_ADMIN) ? "admin" : "control");
  for (uint8_t i = 0U; i < AX12_MASTER_MOTOR_COUNT; ++i)
  {
    printf("master %u: id=%u, position=%u, torque=%s\r\n",
           (unsigned int)(i + 1U),
           (unsigned int)AX12_MASTER_MOTORS[i].id,
           (unsigned int)app->motor_present[i],
           app->motor_torque_enabled[i] ? "on" : "off");
  }
}

static bool AX12_SetMotorTorque(AX12_AppState *app, uint8_t index, bool enabled)
{
  if ((app == NULL) || (index >= AX12_MASTER_MOTOR_COUNT))
  {
    return false;
  }

  if (AX12_SetTorque(&app->ax12, AX12_MASTER_MOTORS[index].id, enabled ? true : false) != AX12_OK)
  {
    return false;
  }

  app->motor_torque_enabled[index] = enabled;
  return true;
}

static bool AX12_SetAllTorque(AX12_AppState *app, bool enabled)
{
  bool ok = true;

  if (app == NULL)
  {
    return false;
  }

  for (uint8_t i = 0U; i < AX12_MASTER_MOTOR_COUNT; ++i)
  {
    if (!AX12_SetMotorTorque(app, i, enabled))
    {
      ok = false;
    }
  }

  return ok;
}

static bool AX12_SetGoalForIndex(AX12_AppState *app, uint8_t index, uint16_t goal)
{
  int32_t adjusted_goal;

  if ((app == NULL) || (index >= AX12_MASTER_MOTOR_COUNT))
  {
    return false;
  }

  adjusted_goal = (int32_t)goal + (int32_t)app->control_offset;
  app->motor_goal[index] = AX12_ClampGoal(adjusted_goal);

  return (AX12_SetGoalPosition(&app->ax12, AX12_MASTER_MOTORS[index].id,
                               app->motor_goal[index]) == AX12_OK);
}

static bool AX12_SetAllGoals(AX12_AppState *app, const uint16_t goals[AX12_MASTER_MOTOR_COUNT])
{
  bool ok = true;

  if ((app == NULL) || (goals == NULL))
  {
    return false;
  }

  for (uint8_t i = 0U; i < AX12_MASTER_MOTOR_COUNT; ++i)
  {
    if (!AX12_SetGoalForIndex(app, i, goals[i]))
    {
      ok = false;
    }
  }

  return ok;
}

bool AX12_AppStartConsoleRx(AX12_AppState *app, UART_HandleTypeDef *console_uart)
{
  if ((app == NULL) || (console_uart == NULL))
  {
    return false;
  }

  s_console_app = app;
  s_console_uart = console_uart;
  app->serial_line_len = 0U;
  app->serial_line_ready = false;
  app->serial_line[0] = '\0';

  return (HAL_UART_Receive_IT(s_console_uart, &s_console_rx_byte, 1U) == HAL_OK);
}

void AX12_AppUartRxCpltCallback(UART_HandleTypeDef *huart)
{
  AX12_AppState *app = s_console_app;

  if ((huart == NULL) || (app == NULL) || (huart != s_console_uart))
  {
    return;
  }

  if ((s_console_rx_byte == '\r') || (s_console_rx_byte == '\n'))
  {
    if (app->serial_line_len > 0U)
    {
      app->serial_line[app->serial_line_len] = '\0';
      app->serial_line_ready = true;
    }
  }
  else if (app->serial_line_len < (sizeof(app->serial_line) - 1U))
  {
    app->serial_line[app->serial_line_len++] = (char)s_console_rx_byte;
  }
  else
  {
    app->serial_line_len = 0U;
    app->serial_line[0] = '\0';
  }

  (void)HAL_UART_Receive_IT(s_console_uart, &s_console_rx_byte, 1U);
}

static bool AX12_ChangeUartBaud(UART_HandleTypeDef *uart, uint32_t baudrate)
{
  if ((uart == NULL) || (uart->Instance != USART1))
  {
    return false;
  }

  if (HAL_UART_DeInit(uart) != HAL_OK)
  {
    return false;
  }

  uart->Init.BaudRate = baudrate;
  return (HAL_HalfDuplex_Init(uart) == HAL_OK);
}

static bool AX12_ReconfigureBusTo1Mbps(AX12_AppState *app)
{
  bool all_ok = true;

  if ((app == NULL) || (app->ax12.uart == NULL))
  {
    return false;
  }

  printf("bus1m: probing configured IDs at 115200\r\n");
  if (!AX12_ChangeUartBaud(app->ax12.uart, AX12_LEGACY_BAUDRATE))
  {
    printf("bus1m: failed to switch UART to 115200\r\n");
    return false;
  }

  for (uint8_t i = 0U; i < AX12_MASTER_MOTOR_COUNT; ++i)
  {
    uint8_t id = AX12_MASTER_MOTORS[i].id;

    if (AX12_Ping(&app->ax12, id) == AX12_OK)
    {
      printf("bus1m: ID %u found at 115200, changing to 1 Mbps\r\n", id);
      (void)AX12_SetBaudRate(&app->ax12, id, AX12_BAUD_VALUE_1MBPS);
      HAL_Delay(20U);
    }
    else
    {
      printf("bus1m: ID %u not found at 115200 (may already be 1 Mbps)\r\n", id);
    }
  }

  if (!AX12_ChangeUartBaud(app->ax12.uart, AX12_BUS_BAUDRATE))
  {
    printf("bus1m: failed to switch UART to 1 Mbps\r\n");
    return false;
  }
  HAL_Delay(20U);

  for (uint8_t i = 0U; i < AX12_MASTER_MOTOR_COUNT; ++i)
  {
    uint8_t id = AX12_MASTER_MOTORS[i].id;

    if (AX12_Ping(&app->ax12, id) == AX12_OK)
    {
      printf("bus1m: ID %u OK at 1 Mbps\r\n", id);
    }
    else
    {
      printf("bus1m: ID %u FAILED at 1 Mbps\r\n", id);
      all_ok = false;
    }
  }

  printf(all_ok ? "bus1m: success, press RESET\r\n"
                : "bus1m: failed, check power/data/GND/IDs\r\n");
  return all_ok;
}

static bool AX12_HandleCommand(AX12_AppState *app, const char *line)
{
  char cmd[16];
  const char *rest = line;

  if ((app == NULL) || (line == NULL))
  {
    return false;
  }

  if (!AX12_ReadToken(&rest, cmd, sizeof(cmd)))
  {
    return false;
  }

  if (strcmp(cmd, "help") == 0)
  {
    printf("cmds: bus1m, mode admin|control, goal <id> <pos>, all <p1> <p2> <p3> <p4>, torque <id|all> <on|off>, speed <id> <spd>, offset <n>, status\r\n");
    return true;
  }

  if (strcmp(cmd, "bus1m") == 0)
  {
    return AX12_ReconfigureBusTo1Mbps(app);
  }

  if (strcmp(cmd, "mode") == 0)
  {
    char mode_name[16];

    if (!AX12_ReadToken(&rest, mode_name, sizeof(mode_name)))
    {
      return false;
    }

    if (strcmp(mode_name, "admin") == 0)
    {
      app->mode = AX12_MODE_ADMIN;
      return true;
    }

    if (strcmp(mode_name, "control") == 0)
    {
      app->mode = AX12_MODE_CONTROL;
      return true;
    }

    return false;
  }

  if (strcmp(cmd, "status") == 0)
  {
    AX12_PrintStatus(app);
    return true;
  }

  if (strcmp(cmd, "offset") == 0)
  {
    int16_t offset = 0;

    if (!AX12_ParseSigned16(rest, &offset))
    {
      return false;
    }

    app->control_offset = offset;
    return true;
  }

  if (strcmp(cmd, "goal") == 0)
  {
    uint8_t motor_id = 0U;
    uint8_t index = 0U;
    uint16_t goal = 0U;
    char token[16];

    if (!AX12_ReadToken(&rest, token, sizeof(token)) || !AX12_ParseMotorId(token, &motor_id))
    {
      return false;
    }
    if (!AX12_GetMotorIndex(motor_id, &index) || !AX12_ParseUnsigned16(rest, &goal))
    {
      return false;
    }

    return AX12_SetGoalForIndex(app, index, goal);
  }

  if (strcmp(cmd, "all") == 0)
  {
    uint16_t goals[AX12_MASTER_MOTOR_COUNT];
    char token[16];

    for (uint8_t i = 0U; i < AX12_MASTER_MOTOR_COUNT; ++i)
    {
      if (!AX12_ReadToken(&rest, token, sizeof(token)) ||
          !AX12_ParseUnsigned16(token, &goals[i]))
      {
        return false;
      }
    }

    return AX12_SetAllGoals(app, goals);
  }

  if (strcmp(cmd, "speed") == 0)
  {
    uint8_t motor_id = 0U;
    uint8_t index = 0U;
    uint16_t speed = 0U;
    char token[16];

    if (!AX12_ReadToken(&rest, token, sizeof(token)) || !AX12_ParseMotorId(token, &motor_id))
    {
      return false;
    }
    if (!AX12_GetMotorIndex(motor_id, &index) || !AX12_ParseUnsigned16(rest, &speed))
    {
      return false;
    }

    return (AX12_SetMovingSpeed(&app->ax12, AX12_MASTER_MOTORS[index].id, speed) == AX12_OK);
  }

  if (strcmp(cmd, "torque") == 0)
  {
    char arg1[16];
    char arg2[16];
    const char *next = rest;
    bool enabled = false;

    if (!AX12_ReadToken(&next, arg1, sizeof(arg1)))
    {
      return false;
    }

    if (!AX12_ReadToken(&next, arg2, sizeof(arg2)))
    {
      if (!AX12_ParseOnOff(arg1, &enabled))
      {
        return false;
      }

      return AX12_SetAllTorque(app, enabled);
    }

    if (strcmp(arg1, "all") == 0)
    {
      if (!AX12_ParseOnOff(arg2, &enabled))
      {
        return false;
      }

      return AX12_SetAllTorque(app, enabled);
    }

    if (!AX12_ParseOnOff(arg2, &enabled))
    {
      return false;
    }

    {
      uint8_t motor_id = 0U;
      uint8_t index = 0U;

      if (!AX12_ParseMotorId(arg1, &motor_id) || !AX12_GetMotorIndex(motor_id, &index))
      {
        return false;
      }

      return AX12_SetMotorTorque(app, index, enabled);
    }
  }

  return false;
}

bool AX12_AppInit(AX12_AppState *app, UART_HandleTypeDef *ax12_uart,
                  UART_HandleTypeDef *link_uart)
{
  if ((app == NULL) || (ax12_uart == NULL) || (link_uart == NULL))
  {
    printf("AX12 init: invalid argument\r\n");
    return false;
  }

  memset(app, 0, sizeof(*app));
  AX12_Init(&app->ax12, ax12_uart, AX12_APP_TIMEOUT_MS);
  app->link_uart = link_uart;
  app->mode = AX12_MODE_ADMIN;
  app->control_offset = 0;
  app->ready = false;
  app->link_sequence = 0U;
  app->last_link_tx_ms = HAL_GetTick();
  app->serial_line_len = 0U;
  app->serial_line_ready = false;
  app->serial_line[0] = '\0';

  for (uint8_t i = 0U; i < AX12_MASTER_MOTOR_COUNT; ++i)
  {
    app->motor_goal[i] = AX12_DEFAULT_GOAL;
    app->motor_present[i] = AX12_DEFAULT_GOAL;
    app->motor_torque_enabled[i] = false;
  }

  for (uint8_t i = 0U; i < AX12_MASTER_MOTOR_COUNT; ++i)
  {
    uint16_t current = AX12_DEFAULT_GOAL;

    if (AX12_Ping(&app->ax12, AX12_MASTER_MOTORS[i].id) != AX12_OK)
    {
      printf("AX12 init: ping failed (ID=%u)\r\n", AX12_MASTER_MOTORS[i].id);
      return false;
    }

    if (AX12_SetTorque(&app->ax12, AX12_MASTER_MOTORS[i].id, false) != AX12_OK)
    {
      printf("AX12 init: torque off failed (ID=%u)\r\n", AX12_MASTER_MOTORS[i].id);
      return false;
    }

    app->motor_torque_enabled[i] = false;

    if (AX12_GetPresentPosition(&app->ax12, AX12_MASTER_MOTORS[i].id, &current) == AX12_OK)
    {
      app->motor_present[i] = current;
      app->motor_goal[i] = current;
    }
  }

  app->ready = true;
  printf("AX12 init ok: master 4 motors ready (torque off)\r\n");
  AX12_PrintStatus(app);
  return true;
}

bool AX12_AppProcessSerial(AX12_AppState *app, UART_HandleTypeDef *console_uart)
{
  bool handled = false;
  char line_copy[AX12_SERIAL_LINE_SIZE];

  if ((app == NULL) || (console_uart == NULL))
  {
    return false;
  }

  if (!app->serial_line_ready)
  {
    (void)console_uart;
    return false;
  }

  __disable_irq();
  strncpy(line_copy, app->serial_line, sizeof(line_copy));
  line_copy[sizeof(line_copy) - 1U] = '\0';
  app->serial_line_ready = false;
  app->serial_line_len = 0U;
  app->serial_line[0] = '\0';
  __enable_irq();

  handled = AX12_HandleCommand(app, line_copy);
  if (handled)
  {
    printf("CMD OK: %s\r\n", line_copy);
  }
  else
  {
    printf("CMD ERR: %s\r\n", line_copy);
  }

  return handled;
}

void AX12_AppUpdate(AX12_AppState *app)
{
  uint32_t now_ms;

  if ((app == NULL) || !app->ready)
  {
    return;
  }

  now_ms = HAL_GetTick();
  if ((now_ms - app->last_link_tx_ms) < HC05_LINK_PERIOD_MS)
  {
    return;
  }
  app->last_link_tx_ms = now_ms;

  (void)AX12_ReadAllPresentPositions(app);
  (void)AX12_SendMasterFrame(app);
}
