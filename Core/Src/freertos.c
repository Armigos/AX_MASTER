/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file          : freertos.c
  * @brief         : FreeRTOS Tasks & Hardware Control Logic
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "spi.h"
#include "gpio.h"
#include "ili9341.h"
#include "ax12.h"
#include "lcd_font.h"
#include "teaching_storage.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    MODE_TEACHING  = 0,
    MODE_AUTO      = 1,
    MODE_ADMIN_JOG = 2
} SystemMode_t;

typedef enum {
    RUN_STATE_STOPPED = 0,
    RUN_STATE_RUNNING = 1,
    RUN_STATE_COMPLETED = 2
} RunState_t;

typedef enum {
    MOTION_NONE = 0,
    MOTION_AUTO,
    MOTION_HOME
} MotionType_t;

typedef enum {
    MASTER_CTRL_ACTION_NONE = 0,
    MASTER_CTRL_ACTION_HOME,
    MASTER_CTRL_ACTION_ESTOP_SYNC,
    MASTER_CTRL_ACTION_MANUAL
} MasterControllerAction_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BT_CMD_SET_GOAL_POS 0x01
#define BT_CMD_SET_TORQUE   0x02
#define BT_CMD_REQ_STATUS   0x03
#define BT_CMD_HOME_POS     0x04
#define BT_CMD_SET_ALL_POS  0x05
#define BT_CMD_START_AUTO    0x06
#define BT_CMD_RUN_AUTO      0x07
#define BT_CMD_HOLD_CURRENT  0x08
#define BT_CMD_STATUS_REPLY 0x83

#define BT_DEFAULT_JOG_PERIOD_MS       10U
#define BT_DEFAULT_STATUS_PERIOD_MS   200U
/* Faster arrival feedback only while BTN14 sequence execution is active. */
#define BT_AUTO_STATUS_PERIOD_MS        10U /* Auto arrival feedback period. */
#define BT_STATUS_PAYLOAD_LENGTH       17U
#define BT_RX_BUFFER_SIZE              64U
/* Home completion is based on actual slave positions.  Keep this slightly
 * wider than normal settling noise so a mechanically limited axis cannot
 * leave the startup interlock permanently on the MOVING screen. */
#define HOME_POSITION_TOLERANCE        30U
#define TEACHING_STEP_POSITION_TOLERANCE 5U
#define MOTION_STABLE_STATUS_COUNT      3U
#define AX12_DEFAULT_POSITION         512U
#define EMERGENCY_STOP_ENABLED          1U


/* LCD Display Layout & Font Configuration Macros */
#define LCD1_TITLE_SCALE    2     
#define LCD1_BODY_SCALE     3     
#define LCD2_TITLE_SCALE    2     
#define LCD2_BODY_SCALE     2     

#define LCD1_START_X        10    
#define LCD1_START_Y        10    
#define LCD1_LINE_HEIGHT    48    // 수정: 0으로 되어 있어 겹치던 줄 간격을 글자 크기에 맞게 35로 수정

#define LCD2_START_X        10    
#define LCD2_START_Y        10    
#define LCD2_LINE_HEIGHT    30    
#define KEYPAD_SCAN_PERIOD_MS        2U
#define KEYPAD_DEBOUNCE_SAMPLES      3U
#define KEYPAD_EVENT_QUEUE_DEPTH    16U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// 전역 변수 영역에 버퍼 및 포인터 선언 (또는 파일 상단 USER CODE BEGIN PV 영역)
uint8_t pc_to_bt[64];
uint8_t bt_to_pc[64];
uint8_t pc_to_bt_head = 0;
uint8_t pc_to_bt_tail = 0;
uint8_t bt_to_pc_head = 0;
uint8_t bt_to_pc_tail = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint16_t g_robot_axis[4]       = {512, 512, 512, 512};
uint16_t g_prev_axis[4]       = {0, 0, 0, 0};
volatile uint16_t g_slave_axis[4] = {512, 512, 512, 512};
volatile uint16_t g_slave_load[4] = {0, 0, 0, 0};
uint16_t g_motor_speed        = 300;
volatile uint16_t g_motor_load = 0;

SystemMode_t g_system_mode     = MODE_TEACHING;
SystemMode_t g_prev_mode       = 0xFF; 
RunState_t   g_run_state       = RUN_STATE_STOPPED;
RunState_t   g_prev_run_state  = 0xFF;
bool         g_emergency_stop  = false;
bool         g_prev_estop      = false;
uint8_t      g_selected_preset = 0;
uint8_t      g_homing_status   = 0; /* 0:idle, 1:moving, 2:completed */
/* Boot interlock: no keypad motion command is accepted until BTN16 Home
 * reaches the verified slave positions. */
static volatile bool g_home_ready = false;
volatile bool g_admin_jog_enabled = false;
static volatile bool g_estop_request = false;
volatile uint32_t g_slave_status_sequence = 0U;
volatile bool g_auto_motion_released = false;
uint8_t g_teach_save_status = 0U; /* 0:none, 1:saved, 2:flash error */
volatile uint32_t g_teach_save_event_sequence = 0U;
volatile uint32_t g_lcd_event_sequence = 0U;

TeachingSequence_t g_teach_memory[TEACHING_PRESET_COUNT + 1U];
uint8_t g_teach_capture_step = 0U;
uint8_t g_teach_last_saved_step = 0U;
uint8_t g_auto_sequence_step = 0U;
static volatile bool g_auto_step_delay_active = false;
/* True after the final preset target has been confirmed at the robot. */
static volatile bool g_auto_sequence_last_sent = false;
static uint32_t g_auto_step_due_ms = 0U;
static uint16_t g_home_positions[4] = {512U, 512U, 512U, 512U};
static uint32_t g_home_retry_due_ms = 0U;
static uint16_t g_motion_target[4] = {512U, 512U, 512U, 512U};
static uint16_t g_estop_sync_target[4] = {512U, 512U, 512U, 512U};
static volatile bool g_estop_waiting_slave_pose = false;
static volatile bool g_estop_sync_active = false;
static volatile bool g_estop_jog_ready = false;
static uint8_t g_estop_sync_stable_count = 0U;
static volatile MotionType_t g_motion_type = MOTION_NONE;
static volatile MasterControllerAction_t g_master_controller_action =
    MASTER_CTRL_ACTION_NONE;
static volatile uint32_t g_bt_jog_period_ms = BT_DEFAULT_JOG_PERIOD_MS;
static volatile uint32_t g_bt_status_period_ms = BT_DEFAULT_STATUS_PERIOD_MS;
static volatile bool teleplot_tx_busy = false;
static uint8_t bt_rx_byte;
static uint8_t bt_rx_buffer[BT_RX_BUFFER_SIZE];
static volatile uint8_t bt_rx_head = 0U;
static volatile uint8_t bt_rx_tail = 0U;
static volatile uint32_t bt_rx_dropped_count = 0U;

// Keypad GPIO
static GPIO_TypeDef* ROW_PORTS[4] = {GPIOC, GPIOC, GPIOC, GPIOC};
static uint16_t       ROW_PINS[4]  = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3};
static GPIO_TypeDef* COL_PORTS[4] = {GPIOC, GPIOC, GPIOC, GPIOC};
static uint16_t       COL_PINS[4]  = {GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_9};

static const uint8_t KEY_MAP[4][4] = {
    { 1,  2,  3,  4},
    { 5,  6,  7,  8},
    { 9, 10, 11, 12},
    {13, 14, 15, 16}
};
/* USER CODE END Variables */
/* Definitions for Bluetooth */
osThreadId_t BluetoothHandle;
const osThreadAttr_t Bluetooth_attributes = {
  .name = "Bluetooth",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LCD1_Task */
osThreadId_t LCD1_TaskHandle;
const osThreadAttr_t LCD1_Task_attributes = {
  .name = "LCD1_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for LCD2_Task */
osThreadId_t LCD2_TaskHandle;
const osThreadAttr_t LCD2_Task_attributes = {
  .name = "LCD2_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Keypad_TaskHand */
osThreadId_t Keypad_TaskHandHandle;
const osThreadAttr_t Keypad_TaskHand_attributes = {
  .name = "Keypad_TaskHand",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for AX_12 */
osThreadId_t AX_12Handle;
const osThreadAttr_t AX_12_attributes = {
  .name = "AX_12",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Teleplot */
osThreadId_t TeleplotHandle;
const osThreadAttr_t Teleplot_attributes = {
  .name = "Teleplot",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for lcdSpiMutex */
osMutexId_t lcdSpiMutexHandle;
const osMutexAttr_t lcdSpiMutex_attributes = {
  .name = "lcdSpiMutex"
};
/* Definitions for btUartMutex */
osMutexId_t btUartMutexHandle;
const osMutexAttr_t btUartMutex_attributes = {
  .name = "btUartMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
uint8_t Keypad_Scan(void);
void Process_Key_Event(uint8_t key);
void BT_SendPacket(uint8_t cmd, uint8_t *data, uint8_t len);
void Robot_SetGoalPosition(uint8_t motor_id, uint16_t position);
void Robot_SetTorque(uint8_t enable);
void Robot_MoveToHome(void);
void Robot_SetHomePositions(const uint16_t positions[4]);
static void ActivateEmergencyStop(void);
static void CancelHomeMotion(void);
static void MasterController_RequestAction(MasterControllerAction_t action);
static void SetSystemMode(SystemMode_t mode);
static uint8_t TeachingSequence_CountSaved(uint8_t preset);
static uint8_t TeachingSequence_Rank(uint8_t preset, uint8_t step);
void BT_SetJogTransmitPeriod(uint32_t period_ms);
void BT_SetStatusPeriod(uint32_t period_ms);
static void BT_ProcessReceivedByte(uint8_t byte);
static void BT_SendAllPositions(const uint16_t positions[4]);
static void BT_SendAutoPositions(const uint16_t positions[4]);
static void BT_SendAutoStartPositions(const uint16_t positions[4]);
static void BT_SendPositionsCommand(uint8_t cmd, const uint16_t positions[4]);
static void BT_RequestStatus(void);
static bool BT_QueueTransmit(const uint8_t *data, uint16_t length);
static void AutoScheduleFollowingStep(void);
void Update_LCD1_Clean(void);
void Update_LCD2_Clean(void);
/* USER CODE END FunctionPrototypes */

void Start_Bluetooth(void *argument);
void StartLCD1Task(void *argument);
void StartLCD2Task(void *argument);
void Keypad_Task(void *argument);
void Start_AX_12(void *argument);
void Start_Teleplot(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  for(uint8_t p=0U;p<=TEACHING_PRESET_COUNT;p++)
  {
    g_teach_memory[p].saved_mask=0U;
    for(uint8_t s=0U;s<TEACHING_SEQUENCE_STEPS;s++)
      for(uint8_t a=0U;a<4U;a++)
        g_teach_memory[p].step[s].axis[a]=TEACHING_EMPTY_AXIS_VALUE;
  }
  (void)TeachingStorage_Load(g_teach_memory);
  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of lcdSpiMutex */
  lcdSpiMutexHandle = osMutexNew(&lcdSpiMutex_attributes);

  /* creation of btUartMutex */
  btUartMutexHandle = osMutexNew(&btUartMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Bluetooth */
  BluetoothHandle = osThreadNew(Start_Bluetooth, NULL, &Bluetooth_attributes);

  /* creation of LCD1_Task */
  LCD1_TaskHandle = osThreadNew(StartLCD1Task, NULL, &LCD1_Task_attributes);

  /* creation of LCD2_Task */
  LCD2_TaskHandle = osThreadNew(StartLCD2Task, NULL, &LCD2_Task_attributes);

  /* creation of Keypad_TaskHand */
  Keypad_TaskHandHandle = osThreadNew(Keypad_Task, NULL, &Keypad_TaskHand_attributes);

  /* creation of AX_12 */
  AX_12Handle = osThreadNew(Start_AX_12, NULL, &AX_12_attributes);

  /* creation of Teleplot */
  TeleplotHandle = osThreadNew(Start_Teleplot, NULL, &Teleplot_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Start_Bluetooth */
/**
  * @brief  Function implementing the Bluetooth thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Bluetooth */
void Start_Bluetooth(void *argument)
{
  /* USER CODE BEGIN Start_Bluetooth */
  (void)argument;
  (void)HAL_UART_Receive_IT(&huart6, &bt_rx_byte, 1U);
  /* AT bridge: COM7 (USART2) <-> Bluetooth module (USART6). */
  for(;;)
  {
    uint8_t byte;

    if (HAL_UART_Receive(&huart2, &byte, 1U, 1U) == HAL_OK)
    {
      (void)BT_QueueTransmit(&byte, 1U);
    }

    /* USART6 runs in one-byte interrupt mode.  Drain every queued byte here;
     * polling one byte then delaying 1ms loses most of a 115200-bps status
     * frame and prevents AUTO Step completion from being detected. */
    while (bt_rx_tail != bt_rx_head)
    {
      byte = bt_rx_buffer[bt_rx_tail];
      bt_rx_tail = (uint8_t)((bt_rx_tail + 1U) % BT_RX_BUFFER_SIZE);
      BT_ProcessReceivedByte(byte);
    }

    osDelay(1U);
  }
  /* USER CODE END Start_Bluetooth */
}

/* USER CODE BEGIN Header_StartLCD1Task */
/**
* @brief Function implementing the LCD1_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLCD1Task */
void StartLCD1Task(void *argument)
{
  /* USER CODE BEGIN StartLCD1Task */
  LCD1_CS_HIGH(); 
  osDelay(50);
  LCD_RST_LOW(); osDelay(50); LCD_RST_HIGH(); osDelay(150);

  if (osMutexWait(lcdSpiMutexHandle, osWaitForever) == osOK) {
    LCD1_CS_LOW();
    ILI9341_Init(); 
    ILI9341_FillScreen(ILI9341_BLACK);
    LCD1_CS_HIGH();
    osMutexRelease(lcdSpiMutexHandle);
  }

  for(;;)
  {
    Update_LCD1_Clean();
    osDelay(20U); // 최적화: LCD 갱신 주기를 당겨서 반응 속도 향상 (기존 100ms -> 50ms)
  }
  /* USER CODE END StartLCD1Task */
}

/* USER CODE BEGIN Header_StartLCD2Task */
/**
* @brief Function implementing the LCD2_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLCD2Task */
void StartLCD2Task(void *argument)
{
  /* USER CODE BEGIN StartLCD2Task */
  osDelay(200); // LCD1과 초기화 충돌 방지 딜레이
  if (osMutexWait(lcdSpiMutexHandle, osWaitForever) == osOK) {
    LCD2_CS_LOW();
    ILI9341_Init(); 
    ILI9341_FillScreen(ILI9341_BLACK);
    LCD2_CS_HIGH();
    osMutexRelease(lcdSpiMutexHandle);
  }

  for(;;)
  {
    Update_LCD2_Clean();
    osDelay(50U); // 최적화: LCD2 갱신 주기 단축 (기존 150ms -> 80ms)
  }
  /* USER CODE END StartLCD2Task */
}

/* USER CODE BEGIN Header_Keypad_Task */
/**
* @brief Function implementing the Keypad_TaskHand thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Keypad_Task */
void Keypad_Task(void *argument)
{
  /* USER CODE BEGIN Keypad_Task */
  uint8_t key = 0, stable_key = 0, candidate_key = 0, stable_count = 0;
  bool estop_button_was_pressed =
      (HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == GPIO_PIN_SET);

  for(;;)
  {
    /* PB2 is edge-triggered.  Polling is only a backup for EXTI2 and must
     * also detect an edge, never a continuously HIGH input. */
    bool estop_button_pressed =
        (HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == GPIO_PIN_SET);
    if (g_estop_request ||
        (estop_button_pressed && !estop_button_was_pressed))
    {
      g_estop_request = false;
      ActivateEmergencyStop();
    }
    estop_button_was_pressed = estop_button_pressed;

    /* 20 ms stable debounce prevents matrix bounce without missing a short
     * button press to be missed or interpreted as a second press. */
    key = Keypad_Scan();
    if (key == candidate_key)
    {
      if (stable_count < KEYPAD_DEBOUNCE_SAMPLES) ++stable_count;
    }
    else
    {
      candidate_key = key;
      stable_count = 1U;
    }
    if ((stable_count >= KEYPAD_DEBOUNCE_SAMPLES) && (stable_key != candidate_key))
    {
      stable_key = candidate_key;
      if (stable_key != 0U)
      {
        /* The keypad task is the single event consumer.  Dispatch the stable
         * press here; the previous queue handle was removed by CubeMX and had
         * no consumer, which left key events unprocessed. */
        Process_Key_Event(stable_key);
      }
    }

    osDelay(KEYPAD_SCAN_PERIOD_MS);
  }
  /* USER CODE END Keypad_Task */
}

/* USER CODE BEGIN Header_Start_AX_12 */
/**
* @brief Function implementing the AX_12 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_AX_12 */
void Start_AX_12(void *argument)
{
  /* USER CODE BEGIN Start_AX_12 */
  uint8_t motor_index = 0U;
  bool read_pending = false;
  uint32_t read_started_ms = 0U;
  uint32_t last_tx_ms = 0U;
  uint32_t last_status_request_ms = 0U;
  (void)argument;
  osDelay(350U);

  /* Initialization may block once; the continuous position loop below does not. */
  for (uint8_t i = 0U; i < AX12_NUM_MOTORS; ++i)
  {
    uint8_t id = AX12_GetMotorId(i);
    uint8_t status[6];

    if (AX12_Ping(id, status, sizeof(status)) == HAL_OK)
    {
      printf("Master controller AX12 ID %u: detected\r\n", (unsigned)id);
    }
    else
    {
      printf("Master controller AX12 ID %u: no response\r\n", (unsigned)id);
    }

    /* The master is a hand-operated controller.  Always request torque OFF,
     * even when the initial Ping was delayed, so every axis can be moved. */
    (void)AX12_Write1(id, AX12_ADDR_TORQUE_ENABLE, 0U);
  }

  for (;;)
  {
    uint32_t now_ms = HAL_GetTick();

    if (g_auto_step_delay_active && ((int32_t)(now_ms - g_auto_step_due_ms) >= 0))
    {
      uint8_t next_step = (uint8_t)(g_auto_sequence_step + 1U);
      while ((next_step < TEACHING_SEQUENCE_STEPS) &&
             ((g_teach_memory[g_selected_preset].saved_mask &
               (1U << next_step)) == 0U))
      {
        ++next_step;
      }

      g_auto_step_delay_active = false;
      if (next_step < TEACHING_SEQUENCE_STEPS)
      {
        g_auto_sequence_step = next_step;
        for (uint8_t i = 0U; i < 4U; ++i)
          g_motion_target[i] = g_teach_memory[g_selected_preset].step[next_step].axis[i];
        BT_SendAutoPositions(g_motion_target);
        ++g_lcd_event_sequence;
      }
      else { g_auto_sequence_last_sent = true; }
    }

    /* Home is safety-critical.  The first command is sent immediately by
     * BTN16; repeat it while Home is still active so a busy/temporarily lost
     * Bluetooth frame cannot leave only the master controller moving. */
    if ((g_motion_type == MOTION_HOME) &&
        ((int32_t)(now_ms - g_home_retry_due_ms) >= 0))
    {
      BT_SendPositionsCommand(BT_CMD_HOME_POS, g_home_positions);
      g_home_retry_due_ms = now_ms + 250U;
    }

    /* BTN16 home / manual-release commands are queued here rather than sent
     * from a keypad handler, so they never collide with USART1 read IT. */
    if (g_master_controller_action != MASTER_CTRL_ACTION_NONE)
    {
      MasterControllerAction_t action;
      if (read_pending)
      {
        AX12_CancelPositionReadIT();
        read_pending = false;
      }

      __disable_irq();
      action = g_master_controller_action;
      g_master_controller_action = MASTER_CTRL_ACTION_NONE;
      __enable_irq();

      for (uint8_t i = 0U; i < AX12_NUM_MOTORS; ++i)
      {
        uint8_t id = AX12_GetMotorId(i);
        if (action == MASTER_CTRL_ACTION_HOME)
        {
          (void)AX12_Write1(id, AX12_ADDR_TORQUE_ENABLE, 1U);
          (void)AX12_Write2(id, AX12_ADDR_MOVING_SPEED,
                            AX12_MASTER_HOME_SPEED);
          (void)AX12_Write2(id, AX12_ADDR_GOAL_POSITION, 512U);
        }
        else if (action == MASTER_CTRL_ACTION_ESTOP_SYNC)
        {
          (void)AX12_Write1(id, AX12_ADDR_TORQUE_ENABLE, 1U);
          (void)AX12_Write2(id, AX12_ADDR_MOVING_SPEED,
                            AX12_MASTER_ESTOP_SYNC_SPEED);
          (void)AX12_Write2(id, AX12_ADDR_GOAL_POSITION,
                            g_estop_sync_target[i]);
        }
        else
        {
          (void)AX12_Write1(id, AX12_ADDR_TORQUE_ENABLE, 0U);
        }
      }
      osDelay(1U);
      continue;
    }

    if (read_pending)
    {
      uint16_t position;
      HAL_StatusTypeDef result = AX12_GetPositionReadITResult(&position);

      if (result == HAL_OK)
      {
        uint8_t motor_id = AX12_GetMotorId(motor_index);
        uint16_t min_position = AX12_MASTER_4_MIN_POSITION;
        uint16_t max_position = AX12_MASTER_4_MAX_POSITION;

        if (motor_id == AX12_MASTER_1_ID)
        {
          min_position = AX12_MASTER_1_MIN_POSITION;
          max_position = AX12_MASTER_1_MAX_POSITION;
        }
        else if (motor_id == AX12_MASTER_2_ID)
        {
          min_position = AX12_MASTER_2_MIN_POSITION;
          max_position = AX12_MASTER_2_MAX_POSITION;
        }
        else if (motor_id == AX12_MASTER_3_ID)
        {
          min_position = AX12_MASTER_3_MIN_POSITION;
          max_position = AX12_MASTER_3_MAX_POSITION;
        }

        if (position < min_position)
        {
          position = min_position;
        }
        else if (position > max_position)
        {
          position = max_position;
        }
        g_robot_axis[motor_index] = position;

        if (g_estop_sync_active)
        {
          bool aligned = true;
          for (uint8_t axis = 0U; axis < AX12_NUM_MOTORS; ++axis)
          {
            uint16_t error = (g_robot_axis[axis] > g_estop_sync_target[axis]) ?
                             (g_robot_axis[axis] - g_estop_sync_target[axis]) :
                             (g_estop_sync_target[axis] - g_robot_axis[axis]);
            if (error > AX12_MASTER_ESTOP_SYNC_TOLERANCE)
            {
              aligned = false;
              break;
            }
          }
          if (aligned)
          {
            if (g_estop_sync_stable_count < 3U) ++g_estop_sync_stable_count;
            if (g_estop_sync_stable_count >= 3U)
            {
              g_estop_sync_active = false;
              g_estop_jog_ready = true;
              MasterController_RequestAction(MASTER_CTRL_ACTION_MANUAL);
              ++g_lcd_event_sequence;
            }
          }
          else
          {
            g_estop_sync_stable_count = 0U;
          }
        }
        read_pending = false;
        motor_index = (uint8_t)((motor_index + 1U) % AX12_NUM_MOTORS);
      }
      else if ((result == HAL_ERROR) ||
               ((now_ms - read_started_ms) >= 20U))
      {
        AX12_CancelPositionReadIT();
        read_pending = false;
        motor_index = (uint8_t)((motor_index + 1U) % AX12_NUM_MOTORS);
      }
    }
    else if (AX12_StartPositionReadIT(AX12_GetMotorId(motor_index)) == HAL_OK)
    {
      read_pending = true;
      read_started_ms = now_ms;
    }

    if (g_admin_jog_enabled &&
        ((now_ms - last_tx_ms) >= g_bt_jog_period_ms))
    {
      last_tx_ms = now_ms;
      BT_SendAllPositions(g_robot_axis);
    }

    uint32_t status_period_ms =
        (((g_system_mode == MODE_AUTO) && (g_run_state == RUN_STATE_RUNNING)) ||
         (g_motion_type == MOTION_HOME)) ?
        BT_AUTO_STATUS_PERIOD_MS : g_bt_status_period_ms;
    if (!g_emergency_stop &&
        ((now_ms - last_status_request_ms) >= status_period_ms))
    {
      last_status_request_ms = now_ms;
      BT_RequestStatus();
    }

    osDelay(1U);
  }
  /* USER CODE END Start_AX_12 */
}

/* USER CODE BEGIN Header_Start_Teleplot */
/**
* @brief Function implementing the Teleplot thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Teleplot */
void Start_Teleplot(void *argument)
{
  /* USER CODE BEGIN Start_Teleplot */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_Teleplot */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
#define BT_TX_BUFFER_SIZE 32U

static uint8_t bt_tx_active[BT_TX_BUFFER_SIZE];
static uint8_t bt_tx_pending[BT_TX_BUFFER_SIZE];
static volatile bool bt_tx_busy;
static volatile bool bt_tx_pending_ready;
static uint16_t bt_tx_active_length;
static uint16_t bt_tx_pending_length;

static bool BT_IsUrgentFrame(const uint8_t *data, uint16_t length)
{
  return (length >= 5U) &&
         ((data[2] == BT_CMD_HOME_POS) ||
          (data[2] == BT_CMD_START_AUTO) ||
          (data[2] == BT_CMD_RUN_AUTO) ||
          (data[2] == BT_CMD_HOLD_CURRENT) ||
          (data[2] == BT_CMD_SET_TORQUE));
}

static bool BT_IsStatusFrame(const uint8_t *data, uint16_t length)
{
  return (length >= 5U) && (data[2] == BT_CMD_REQ_STATUS);
}

static bool BT_IsJogFrame(const uint8_t *data, uint16_t length)
{
  return (length >= 5U) && (data[2] == BT_CMD_SET_ALL_POS);
}

int __io_putchar(int ch) {
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

static bool BT_QueueTransmit(const uint8_t *data, uint16_t length)
{
  bool start_now = false;

  if ((data == NULL) || (length == 0U) || (length > BT_TX_BUFFER_SIZE))
  {
    return false;
  }

  __disable_irq();
  if (!bt_tx_busy)
  {
    memcpy(bt_tx_active, data, length);
    bt_tx_active_length = length;
    bt_tx_busy = true;
    start_now = true;
  }
  else
  {
    /* Do not let the periodic status request replace a one-shot Home/Auto
     * command while another Bluetooth frame is being transmitted. */
    bool pending_is_urgent = bt_tx_pending_ready &&
                             BT_IsUrgentFrame(bt_tx_pending,
                                              bt_tx_pending_length);
    bool pending_is_jog = bt_tx_pending_ready &&
                          BT_IsJogFrame(bt_tx_pending,
                                        bt_tx_pending_length);
    bool keep_pending =
        (pending_is_urgent && BT_IsStatusFrame(data, length)) ||
        (pending_is_jog && BT_IsStatusFrame(data, length));
    if (!keep_pending)
    {
      memcpy(bt_tx_pending, data, length);
      bt_tx_pending_length = length;
      bt_tx_pending_ready = true;
    }
  }
  __enable_irq();

  if (start_now &&
      (HAL_UART_Transmit_IT(&huart6, bt_tx_active,
                            bt_tx_active_length) != HAL_OK))
  {
    __disable_irq();
    bt_tx_busy = false;
    __enable_irq();
    return false;
  }
  return true;
}

void BT_UartTxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart != &huart6)
  {
    return;
  }

  if (bt_tx_pending_ready)
  {
    memcpy(bt_tx_active, bt_tx_pending, bt_tx_pending_length);
    bt_tx_active_length = bt_tx_pending_length;
    bt_tx_pending_ready = false;
    if (HAL_UART_Transmit_IT(&huart6, bt_tx_active,
                             bt_tx_active_length) == HAL_OK)
    {
      return;
    }
  }
  bt_tx_busy = false;
}

void BT_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart != &huart6)
  {
    return;
  }

  uint8_t next = (uint8_t)((bt_rx_head + 1U) % BT_RX_BUFFER_SIZE);
  if (next != bt_rx_tail)
  {
    bt_rx_buffer[bt_rx_head] = bt_rx_byte;
    bt_rx_head = next;
  }
  else
  {
    ++bt_rx_dropped_count;
  }

  (void)HAL_UART_Receive_IT(&huart6, &bt_rx_byte, 1U);
}

void BT_UartErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart6)
  {
    bt_tx_pending_ready = false;
    bt_tx_busy = false;
    (void)HAL_UART_Receive_IT(&huart6, &bt_rx_byte, 1U);
  }
}

void Teleplot_UartTxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    teleplot_tx_busy = false;
  }
}

void Teleplot_UartErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    teleplot_tx_busy = false;
  }
}

void BT_SendPacket(uint8_t cmd, uint8_t *data, uint8_t len)
{
  uint8_t b[32], n=0U, sum=(uint8_t)(cmd+len);
  if (len>27U || (len && data==NULL)) return;
  b[n++]=0xAAU; b[n++]=0x55U; b[n++]=cmd; b[n++]=len;
  for(uint8_t i=0U;i<len;i++){b[n++]=data[i];sum=(uint8_t)(sum+data[i]);} b[n++]=sum;
  (void)BT_QueueTransmit(b, n);
}
static void BT_SendPositionsCommand(uint8_t cmd,const uint16_t p[4]){uint8_t d[8];for(uint8_t i=0;i<4;i++){d[i*2]=(uint8_t)p[i];d[i*2+1]=(uint8_t)(p[i]>>8);}BT_SendPacket(cmd,d,8);}
static void BT_SendAllPositions(const uint16_t p[4]){BT_SendPositionsCommand(BT_CMD_SET_ALL_POS,p);}
static void BT_SendAutoPositions(const uint16_t p[4]){BT_SendPositionsCommand(BT_CMD_RUN_AUTO,p);}
static void BT_SendAutoStartPositions(const uint16_t p[4]){uint8_t d[9]={1U};for(uint8_t i=0U;i<4U;i++){d[1U+i*2U]=(uint8_t)p[i];d[2U+i*2U]=(uint8_t)(p[i]>>8U);}BT_SendPacket(BT_CMD_START_AUTO,d,9U);}
static void BT_RequestStatus(void){BT_SendPacket(BT_CMD_REQ_STATUS,NULL,0U);}
void BT_SetJogTransmitPeriod(uint32_t ms){g_bt_jog_period_ms=(ms<10U)?10U:((ms>1000U)?1000U:ms);}
void BT_SetStatusPeriod(uint32_t ms){g_bt_status_period_ms=(ms<50U)?50U:((ms>5000U)?5000U:ms);}
static void BT_ProcessReceivedByte(uint8_t x)
{
  static uint8_t st = 0U, cmd = 0U, len = 0U, i = 0U, sum = 0U, d[27];

  if (st == 0U) { if (x == 0xAAU) st = 1U; return; }
  if (st == 1U) { st = (x == 0x55U) ? 2U : 0U; return; }
  if (st == 2U) { cmd = x; sum = x; st = 3U; return; }
  if (st == 3U) { len = x; sum = (uint8_t)(sum + x); i = 0U; st = (len > sizeof(d)) ? 0U : ((len == 0U) ? 5U : 4U); return; }
  if (st == 4U) { d[i++] = x; sum = (uint8_t)(sum + x); if (i >= len) st = 5U; return; }

  if ((x == sum) && (cmd == BT_CMD_STATUS_REPLY) &&
      (len == BT_STATUS_PAYLOAD_LENGTH))
  {
    uint32_t load_sum = 0U;
    bool arrived = (g_motion_type != MOTION_NONE);
    g_auto_motion_released = ((d[16U] & 0x02U) != 0U) || g_auto_motion_released;

    for (uint8_t k = 0U; k < 4U; ++k)
    {
      uint16_t tolerance = (g_motion_type == MOTION_AUTO) ?
                           ((k == AX12_AUTO_GRIPPER_AXIS_INDEX) ?
                            AX12_AUTO_GRIPPER_TOLERANCE :
                            TEACHING_STEP_POSITION_TOLERANCE) :
                           HOME_POSITION_TOLERANCE;
      uint16_t error;
      g_slave_axis[k] = (uint16_t)d[k * 2U] |
                        ((uint16_t)d[k * 2U + 1U] << 8U);
      g_slave_load[k] = (uint16_t)d[8U + k * 2U] |
                        ((uint16_t)d[9U + k * 2U] << 8U);
      load_sum += g_slave_load[k];
      error = (g_slave_axis[k] > g_motion_target[k]) ?
              (g_slave_axis[k] - g_motion_target[k]) :
              (g_motion_target[k] - g_slave_axis[k]);
      if (error > tolerance) arrived = false; /* boundary value is included. */
    }

    g_motor_load = (uint16_t)(load_sum / 4U);
    ++g_slave_status_sequence;
    /* PB2 does not use a previously cached status.  The slave sends this
     * frame after it has latched its held pose; use those values to align the
     * physical master controller. */
    if (g_emergency_stop && g_estop_waiting_slave_pose)
    {
      for (uint8_t axis = 0U; axis < 4U; ++axis)
      {
        g_estop_sync_target[axis] = g_slave_axis[axis];
      }
      g_estop_waiting_slave_pose = false;
      g_estop_sync_active = true;
      g_estop_sync_stable_count = 0U;
      MasterController_RequestAction(MASTER_CTRL_ACTION_ESTOP_SYNC);
      ++g_lcd_event_sequence;
    }
    if (arrived && ((g_motion_type != MOTION_AUTO) || g_auto_motion_released))
    {
      if (g_motion_type == MOTION_HOME)
      {
        g_homing_status = 2U;
        g_home_ready = true;
        g_motion_type = MOTION_NONE;
        ++g_lcd_event_sequence;
        g_prev_mode = (SystemMode_t)0xFF;
      }
      else if (!g_auto_sequence_last_sent)
      {
        /* Advance only after the currently commanded target has physically
         * arrived.  A short, configurable settling time is then applied. */
        AutoScheduleFollowingStep();
      }
      else
      {
        g_run_state = RUN_STATE_COMPLETED;
        g_motion_type = MOTION_NONE;
        g_prev_mode = (SystemMode_t)0xFF;
      }
    }
  }
  st = 0U;
}
void Robot_SetGoalPosition(uint8_t id,uint16_t p){uint8_t d[3]={id,(uint8_t)p,(uint8_t)(p>>8)};BT_SendPacket(BT_CMD_SET_GOAL_POS,d,3);}
void Robot_SetTorque(uint8_t e){uint8_t d[1]={(e!=0U)?1U:0U};BT_SendPacket(BT_CMD_SET_TORQUE,d,1);}
void Robot_SetHomePositions(const uint16_t p[4]){if(p)for(uint8_t i=0;i<4;i++)g_home_positions[i]=(p[i]<=1023U)?p[i]:1023U;}
static void MasterController_RequestAction(MasterControllerAction_t action){__disable_irq();g_master_controller_action=action;__enable_irq();}
static void SetSystemMode(SystemMode_t mode){g_system_mode=mode;g_admin_jog_enabled=(mode==MODE_ADMIN_JOG)||(mode==MODE_TEACHING);++g_lcd_event_sequence;}
static uint8_t TeachingSequence_CountSaved(uint8_t preset){uint8_t count=0U;if(preset<=TEACHING_PRESET_COUNT)for(uint8_t i=0U;i<TEACHING_SEQUENCE_STEPS;i++)count+=(g_teach_memory[preset].saved_mask&(1U<<i))?1U:0U;return count;}
static uint8_t TeachingSequence_Rank(uint8_t preset,uint8_t step){uint8_t rank=0U;if(preset<=TEACHING_PRESET_COUNT)for(uint8_t i=0U;i<=step&&i<TEACHING_SEQUENCE_STEPS;i++)rank+=(g_teach_memory[preset].saved_mask&(1U<<i))?1U:0U;return rank;}
static void AutoScheduleFollowingStep(void)
{
  uint8_t next_step = (uint8_t)(g_auto_sequence_step + 1U);
  while ((next_step < TEACHING_SEQUENCE_STEPS) &&
         ((g_teach_memory[g_selected_preset].saved_mask &
           (1U << next_step)) == 0U))
  {
    ++next_step;
  }
  if (next_step < TEACHING_SEQUENCE_STEPS)
  {
    g_auto_step_due_ms = HAL_GetTick() + AUTO_SEQUENCE_STEP_DELAY_MS;
    g_auto_step_delay_active = true;
  }
  else
  {
    g_auto_sequence_last_sent = true;
    g_auto_step_delay_active = false;
  }
}
void EmergencyStop_Request(void){g_estop_request=true;g_admin_jog_enabled=false;}
static void ActivateEmergencyStop(void){if(g_emergency_stop)return;g_emergency_stop=true;g_run_state=RUN_STATE_STOPPED;g_admin_jog_enabled=false;g_motion_type=MOTION_NONE;g_auto_step_delay_active=false;g_auto_sequence_last_sent=false;g_homing_status=0U;g_estop_waiting_slave_pose=true;g_estop_sync_active=false;g_estop_jog_ready=false;g_estop_sync_stable_count=0U;MasterController_RequestAction(MASTER_CTRL_ACTION_MANUAL);/* Slave replies with its actual latched pose; do not use a cached pose. */BT_SendPacket(BT_CMD_HOLD_CURRENT,NULL,0U);BT_RequestStatus();++g_lcd_event_sequence;printf("[E-STOP] waiting for slave held-pose frame\r\n");g_prev_mode=(SystemMode_t)0xFF;}
static void CancelHomeMotion(void){if((g_motion_type==MOTION_HOME)||(g_homing_status==1U)){g_motion_type=MOTION_NONE;g_homing_status=0U;g_run_state=RUN_STATE_STOPPED;BT_SendPacket(BT_CMD_HOLD_CURRENT,NULL,0U);printf("Home motion cancelled: hold torque ON\r\n");}else if(g_homing_status==2U){g_homing_status=0U;}}
void Robot_MoveToHome(void){static const uint16_t home[4]={512U,512U,512U,512U};g_admin_jog_enabled=false;g_auto_step_delay_active=false;g_auto_sequence_last_sent=false;g_home_ready=false;g_homing_status=1U;g_motion_type=MOTION_HOME;for(uint8_t i=0;i<4;i++){g_home_positions[i]=home[i];g_motion_target[i]=home[i];}MasterController_RequestAction(MASTER_CTRL_ACTION_HOME);BT_SendPositionsCommand(BT_CMD_HOME_POS,home);g_home_retry_due_ms=HAL_GetTick()+250U;++g_lcd_event_sequence;g_prev_mode=(SystemMode_t)0xFF;}
void Process_Key_Event(uint8_t key)
{
  /* Startup safety interlock: only Home can start motion before a verified
   * 512-position return.  Keep BTN15 available only to release a real E-stop. */
  if (!g_home_ready && !(g_emergency_stop && (key == 15U)))
  {
    if ((key == 16U) && (g_homing_status != 1U))
    {
      Robot_MoveToHome();
    }
    return;
  }

  /* Preset buttons are valid only after entering Teaching (BTN13) or
   * Auto (BTN14).  In Admin JOG and during Home they must not redraw the
   * display, change a preset, or cancel the running Home command. */
  if ((key >= 1U) && (key <= TEACHING_PRESET_COUNT) &&
      ((g_homing_status != 0U) ||
       ((g_system_mode != MODE_TEACHING) && (g_system_mode != MODE_AUTO))))
  {
    return;
  }

  /* BTN16 is the only key that may retain/restart a home sequence. */
  if (key != 16U) CancelHomeMotion();

  if (key == 15U)
  {
    /* BTN15 is the only command accepted during E-STOP: release and JOG. */
    if (g_emergency_stop && !g_estop_jog_ready)
    {
      ++g_lcd_event_sequence;
      return;
    }
    g_emergency_stop = false;
    g_estop_waiting_slave_pose = false;
    g_estop_sync_active = false;
    SetSystemMode(MODE_ADMIN_JOG);
    g_run_state = RUN_STATE_STOPPED;
    g_motion_type = MOTION_NONE;
    g_auto_step_delay_active = false;
    g_auto_sequence_last_sent = false;
    g_teach_save_status = 0U;
    g_teach_capture_step = 0U;
    MasterController_RequestAction(MASTER_CTRL_ACTION_MANUAL);
    /* Do not re-enable the previous AUTO goal.  The slave first captures its
     * present positions as hold goals, then enables torque for safe JOG. */
    BT_SendPacket(BT_CMD_HOLD_CURRENT, NULL, 0U);
    printf("ADMIN JOG enabled (BTN15)\r\n");
    g_prev_mode = (SystemMode_t)0xFF;
    return;
  }

  if (g_emergency_stop) return;

  /* Any key other than BTN15 leaves administrator JOG and stops its stream. */
  g_admin_jog_enabled = false;

  if (key == 13U)
  {
    /* Never let a stale homing screen override the teaching screen. */
    g_homing_status = 0U;
    SetSystemMode(MODE_TEACHING);
    g_run_state = RUN_STATE_STOPPED;
    g_motion_type = MOTION_NONE;
    g_auto_step_delay_active = false;
    g_auto_sequence_last_sent = false;
    /* Teaching records the live master pose, so it must stream just like
     * Admin JOG while the operator moves the controller. */
    g_teach_save_status = 0U;
    MasterController_RequestAction(MASTER_CTRL_ACTION_MANUAL);
    Robot_SetTorque(1U);
    g_prev_mode = (SystemMode_t)0xFF;
    return;
  }

  if (key == 14U)
  {
    g_admin_jog_enabled = false;
    g_teach_save_status = 0U;
    /* First BTN14 enters Auto mode only.  A second BTN14, while AUTO is
     * READY, starts the selected preset.  This prevents an accidental
     * sequence start when BTN14 is pressed from Teaching/Admin mode. */
    if (g_system_mode != MODE_AUTO)
    {
      SetSystemMode(MODE_AUTO);
      g_run_state = RUN_STATE_STOPPED;
      g_motion_type = MOTION_NONE;
      g_auto_step_delay_active = false;
      g_auto_sequence_last_sent = false;
      g_auto_motion_released = false;
    }
    else if ((g_run_state == RUN_STATE_STOPPED) &&
             (g_selected_preset >= 1U) &&
             (g_selected_preset <= TEACHING_PRESET_COUNT) &&
             (g_teach_memory[g_selected_preset].saved_mask != 0U))
    {
      g_auto_sequence_step = 0U;
      while ((g_auto_sequence_step < TEACHING_SEQUENCE_STEPS) &&
             ((g_teach_memory[g_selected_preset].saved_mask & (1U << g_auto_sequence_step)) == 0U)) ++g_auto_sequence_step;
      for (uint8_t i = 0U; i < 4U; ++i) g_motion_target[i] = g_teach_memory[g_selected_preset].step[g_auto_sequence_step].axis[i];
      g_motion_type = MOTION_AUTO;
      g_run_state = RUN_STATE_RUNNING;
      /* First sequence point starts only after the slave Sharp sensor detects
       * an object.  Subsequent points use RUN_AUTO after this release. */
      g_auto_motion_released = false;
      g_auto_sequence_last_sent = false;
      BT_SendAutoStartPositions(g_motion_target);
    }
    g_prev_mode = (SystemMode_t)0xFF;
    return;
  }

  if (key == 16U)
  {
    Robot_MoveToHome();
    return;
  }

  if ((key == 12U) && (g_system_mode == MODE_TEACHING))
  {
    TeachingPoint_t *slot=&g_teach_memory[g_selected_preset].step[g_teach_capture_step];
    bool changed=false;
    for(uint8_t i=0U;i<4U;i++){uint16_t diff=(slot->axis[i]>g_robot_axis[i])?(slot->axis[i]-g_robot_axis[i]):(g_robot_axis[i]-slot->axis[i]);changed=changed||(diff>TEACHING_SAVE_DEADBAND);}
    if(changed || ((g_teach_memory[g_selected_preset].saved_mask & (1U<<g_teach_capture_step))==0U)){
      for(uint8_t i=0U;i<4U;i++)slot->axis[i]=g_robot_axis[i];
      g_teach_memory[g_selected_preset].saved_mask|=(uint32_t)(1U<<g_teach_capture_step);
      g_teach_last_saved_step=g_teach_capture_step;
      g_teach_save_status=TeachingStorage_Save(g_teach_memory)?1U:2U;
      if(g_teach_save_status==1U){++g_teach_save_event_sequence;g_teach_capture_step=(uint8_t)((g_teach_capture_step+1U)%TEACHING_SEQUENCE_STEPS);}
    }else g_teach_save_status=3U;
    g_admin_jog_enabled=true;
    ++g_lcd_event_sequence;
    g_prev_mode=(SystemMode_t)0xFF;
    return;
  }

  if ((key >= 1U) && (key <= TEACHING_PRESET_COUNT))
  {
    g_selected_preset = key;
    if (g_system_mode == MODE_TEACHING)
    {
      g_teach_capture_step = 0U;
      g_teach_save_status = 0U;
      g_admin_jog_enabled = true;
      ++g_lcd_event_sequence;
    }
    else if (g_system_mode == MODE_AUTO)
    {
      g_run_state = RUN_STATE_STOPPED;
      g_motion_type = MOTION_NONE;
      g_auto_step_delay_active = false;
      g_auto_sequence_last_sent = false;
    }
    g_prev_mode = (SystemMode_t)0xFF;
  }
}
uint8_t Keypad_Scan(void)
{
  for (int r = 0; r < 4; r++) {
    for (int i = 0; i < 4; i++) {
      HAL_GPIO_WritePin(ROW_PORTS[i], ROW_PINS[i], GPIO_PIN_RESET);
    }
    HAL_GPIO_WritePin(ROW_PORTS[r], ROW_PINS[r], GPIO_PIN_SET);
    
    // 최적화: 기존 복잡한 소프트웨어 루프 딜레이를 초단축하여 반응 속도 대폭 개선
    for(volatile int d=0; d<15; d++);

    for (int c = 0; c < 4; c++) {
      if (HAL_GPIO_ReadPin(COL_PORTS[c], COL_PINS[c]) == GPIO_PIN_SET) {
        return KEY_MAP[r][c];
      }
    }
  }
  return 0;
}

void Update_LCD1_Clean(void)
{
  bool changed[4] = {false, false, false, false};
  bool any_change = false;
  static bool header_drawn = false;
  static bool estop_screen_drawn = false;
  static bool previous_estop_sync = false;
  char buf[20];

  if (g_emergency_stop)
  {
    if (estop_screen_drawn && (previous_estop_sync == g_estop_sync_active)) return;
    if (osMutexWait(lcdSpiMutexHandle, 20U) != osOK) return;
    LCD2_CS_HIGH();
    LCD1_CS_LOW();
    ILI9341_FillScreen(ILI9341_RED);
    LCD_PutString(28U, 72U, "EMERGENCY STOP", ILI9341_WHITE, ILI9341_RED, 2U);
    /* Leave two blank lines beneath the title, then show the sole action. */
    LCD_PutString(20U, 168U, "BTN15 : JOG ENABLE", ILI9341_YELLOW, ILI9341_RED, 2U);
    LCD1_CS_HIGH();
    osMutexRelease(lcdSpiMutexHandle);
    estop_screen_drawn = true;
    previous_estop_sync = g_estop_sync_active;
    header_drawn = false;
    return;
  }

  if (estop_screen_drawn)
  {
    if (osMutexWait(lcdSpiMutexHandle, 20U) != osOK) return;
    LCD2_CS_HIGH();
    LCD1_CS_LOW();
    ILI9341_FillScreen(ILI9341_BLACK);
    LCD1_CS_HIGH();
    osMutexRelease(lcdSpiMutexHandle);
    estop_screen_drawn = false;
    previous_estop_sync = false;
    header_drawn = false;
  }

  for (uint8_t i = 0U; i < 4U; ++i)
  {
    if (g_robot_axis[i] != g_prev_axis[i])
    {
      changed[i] = true;
      any_change = true;
    }
  }
  if (!any_change && header_drawn) return;
  bool first_draw = !header_drawn;

  if (osMutexWait(lcdSpiMutexHandle, 5U) != osOK) return;
  LCD2_CS_HIGH();
  LCD1_CS_LOW();

  if (!header_drawn)
  {
    LCD_PutString(LCD1_START_X, LCD1_START_Y, "[CURRENT POSITION]", ILI9341_YELLOW, ILI9341_BLACK, LCD1_TITLE_SCALE);
    header_drawn = true;
  }

  for (uint8_t i = 0U; i < 4U; ++i)
  {
    if (changed[i] || first_draw)
    {
      uint16_t y = (uint16_t)(LCD1_START_Y + LCD1_LINE_HEIGHT * (i + 1U));
      snprintf(buf, sizeof(buf), "Axis%u : %-4u", (unsigned)(i + 1U), (unsigned)g_robot_axis[i]);
      /* Every glyph, including spaces, paints its black background: no digit ghosting. */
      LCD_PutString(LCD1_START_X, y, buf, ILI9341_GREEN, ILI9341_BLACK, LCD1_BODY_SCALE);
      g_prev_axis[i] = g_robot_axis[i];
    }
  }

  LCD1_CS_HIGH();
  osMutexRelease(lcdSpiMutexHandle);
}

void Update_LCD2_Clean(void)
{
  static uint8_t prev_preset = 0xFF;
  static uint8_t prev_homing_status = 0xFF;
  static uint32_t prev_slave_status_sequence = 0xFFFFFFFFUL;
  static uint32_t prev_teach_save_event_sequence = 0xFFFFFFFFUL;
  static uint32_t prev_lcd_event_sequence = 0xFFFFFFFFUL;
  static bool prev_home_ready = false;
  bool preset_changed = (g_selected_preset != prev_preset);
  bool homing_changed = (g_homing_status != prev_homing_status);

  if (g_system_mode == g_prev_mode && 
      g_run_state == g_prev_run_state && 
      g_emergency_stop == g_prev_estop &&
      !preset_changed && !homing_changed &&
      (g_teach_save_event_sequence == prev_teach_save_event_sequence) &&
      (g_lcd_event_sequence == prev_lcd_event_sequence) &&
      (g_home_ready == prev_home_ready) &&
      (((g_system_mode != MODE_ADMIN_JOG) && (g_homing_status == 0U)) ||
       (g_slave_status_sequence == prev_slave_status_sequence))) {
    return;
  }

  /* A mode transition needs a full refresh. */
  bool layout_changed = (g_system_mode != g_prev_mode) ||
                        (g_emergency_stop != g_prev_estop) || homing_changed ||
                        (g_home_ready != prev_home_ready);
  char buf[30];
  
  // 최적화: 뮤텍스 획득 타임아웃 단축
  if (osMutexWait(lcdSpiMutexHandle, 20) == osOK) {
    LCD1_CS_HIGH();
    LCD2_CS_LOW();

    /* Full clear only when the screen layout changes; status updates redraw text only. */
    if (layout_changed) ILI9341_FillScreen(ILI9341_BLACK);

    uint16_t y1 = LCD2_START_Y;
    uint16_t y2 = LCD2_START_Y + LCD2_LINE_HEIGHT;
    uint16_t y3 = LCD2_START_Y + (LCD2_LINE_HEIGHT * 2);
    uint16_t y4 = LCD2_START_Y + (LCD2_LINE_HEIGHT * 3);
    uint16_t y5 = LCD2_START_Y + (LCD2_LINE_HEIGHT * 4);
    uint16_t y6 = LCD2_START_Y + (LCD2_LINE_HEIGHT * 5);

    if (g_emergency_stop)
    {
      LCD_PutString(LCD2_START_X, y1, "EMERGENCY STOP", ILI9341_RED, ILI9341_BLACK, LCD2_TITLE_SCALE);
      LCD_PutString(LCD2_START_X, y2, "                    ", ILI9341_BLACK, ILI9341_BLACK, LCD2_BODY_SCALE);
      LCD_PutString(LCD2_START_X, y3, "                    ", ILI9341_BLACK, ILI9341_BLACK, LCD2_BODY_SCALE);
      LCD_PutString(LCD2_START_X, y4, "                    ", ILI9341_BLACK, ILI9341_BLACK, LCD2_BODY_SCALE);
      LCD_PutString(LCD2_START_X, y5, "                    ", ILI9341_BLACK, ILI9341_BLACK, LCD2_BODY_SCALE);
      LCD_PutString(LCD2_START_X, y6, "                    ", ILI9341_BLACK, ILI9341_BLACK, LCD2_BODY_SCALE);
    }
    else if (!g_home_ready && (g_homing_status == 0U))
    {
      /* The LCD font is ASCII-only, so use a reliable English industrial
       * label instead of Korean glyphs that would render as blanks. */
      /* 8-pixel fixed-width font: x positions are calculated for a 320-pixel
       * screen, so the boot title stays exactly centred. */
      LCD_PutString(48U, 35U, "PROJECT ARMIGO", ILI9341_CYAN, ILI9341_BLACK, 2U);
      LCD_PutString(88U, 100U, "4-AXIS", ILI9341_WHITE, ILI9341_BLACK, 3U);
      LCD_PutString(32U, 170U, "INDUSTRIAL ROBOT", ILI9341_WHITE, ILI9341_BLACK, 2U);
    }
    else if (g_homing_status != 0U)
    {
      LCD_PutString(LCD2_START_X, y1, "[HOME POSITION]     ", ILI9341_CYAN,   ILI9341_BLACK, LCD2_TITLE_SCALE);
      LCD_PutString(LCD2_START_X, y2, (g_homing_status == 1U) ? "STATUS: MOVING      " : "STATUS: COMPLETED   ", (g_homing_status == 1U) ? ILI9341_YELLOW : ILI9341_GREEN, ILI9341_BLACK, LCD2_BODY_SCALE);
      sprintf(buf, "A1:%04u A2:%04u", (unsigned)g_slave_axis[0], (unsigned)g_slave_axis[1]);
      LCD_PutString(LCD2_START_X, y3, buf, ILI9341_WHITE, ILI9341_BLACK, LCD2_BODY_SCALE);
      sprintf(buf, "A3:%04u A4:%04u", (unsigned)g_slave_axis[2], (unsigned)g_slave_axis[3]);
      LCD_PutString(LCD2_START_X, y4, buf, ILI9341_WHITE, ILI9341_BLACK, LCD2_BODY_SCALE);
      LCD_PutString(LCD2_START_X, y5, (g_homing_status == 1U) ? "ALL AXIS -> 512     " : "INPUT ENABLED       ", ILI9341_WHITE, ILI9341_BLACK, LCD2_BODY_SCALE);
      LCD_PutString(LCD2_START_X, y6, "                    ", ILI9341_BLACK, ILI9341_BLACK, LCD2_BODY_SCALE);
    }
    else if (g_system_mode == MODE_ADMIN_JOG)
    {
      LCD_PutString(LCD2_START_X, y1, "[ADMIN JOG MODE]", ILI9341_CYAN, ILI9341_BLACK, LCD2_TITLE_SCALE);
      LCD_PutString(LCD2_START_X, y2, "JOG RUNNING", ILI9341_GREEN, ILI9341_BLACK, LCD2_BODY_SCALE);
    }
    else if (g_system_mode == MODE_TEACHING)
    {
      LCD_PutString(LCD2_START_X, y1, "[TEACHING MODE]     ", ILI9341_MAGENTA, ILI9341_BLACK, LCD2_TITLE_SCALE);
      
      if (g_teach_save_status == 1U) {
        sprintf(buf, "P%02u STEP%u SAVED", (unsigned)g_selected_preset, (unsigned)(g_teach_last_saved_step+1U));
        LCD_PutString(LCD2_START_X, y2, buf, ILI9341_GREEN, ILI9341_BLACK, LCD2_BODY_SCALE);
      } else if (g_teach_save_status == 2U) {
        LCD_PutString(LCD2_START_X, y2, "FLASH SAVE ERROR!   ", ILI9341_RED, ILI9341_BLACK, LCD2_BODY_SCALE);
      } else if (g_teach_save_status == 3U) {
        LCD_PutString(LCD2_START_X, y2, "STEP UNCHANGED      ", ILI9341_WHITE, ILI9341_BLACK, LCD2_BODY_SCALE);
      } else {
        if (g_selected_preset >= 1U)
        {
          sprintf(buf, "PRESET %02u SELECTED", (unsigned)g_selected_preset);
          LCD_PutString(LCD2_START_X, y2, buf, ILI9341_WHITE, ILI9341_BLACK, LCD2_BODY_SCALE);
        }
        else
        {
          LCD_PutString(LCD2_START_X, y2, "SELECT PRESET 1~11 ", ILI9341_WHITE, ILI9341_BLACK, LCD2_BODY_SCALE);
        }
      }
      
      LCD_PutString(LCD2_START_X, y3, "                    ", ILI9341_BLACK, ILI9341_BLACK, LCD2_BODY_SCALE);
      LCD_PutString(LCD2_START_X, y4, "                    ", ILI9341_BLACK, ILI9341_BLACK, LCD2_BODY_SCALE);
    }
    else if (g_system_mode == MODE_AUTO)
    {
      uint16_t auto_y2 = (uint16_t)(y2 + 10U);
      uint16_t auto_y3 = (uint16_t)(y3 + 20U);
      uint16_t auto_y4 = (uint16_t)(y4 + 30U);
      uint16_t auto_y5 = (uint16_t)(y5 + 40U);

      LCD_PutString(LCD2_START_X, y1, "[AUTO MOVE MODE]    ", ILI9341_YELLOW, ILI9341_BLACK, LCD2_TITLE_SCALE);

      /* Fixed-line AUTO layout.  Each line paints its own background, so a
       * step update never clears or overlaps the entire LCD. */
      sprintf(buf, "PRESET: %02u        ", (unsigned)g_selected_preset);
      LCD_PutString(LCD2_START_X, auto_y2, buf, ILI9341_WHITE, ILI9341_BLACK, LCD2_BODY_SCALE);
      sprintf(buf, "STEP: %u/%u          ", (unsigned)TeachingSequence_Rank(g_selected_preset,g_auto_sequence_step), (unsigned)TeachingSequence_CountSaved(g_selected_preset));
      LCD_PutString(LCD2_START_X, auto_y3, buf, ILI9341_WHITE, ILI9341_BLACK, LCD2_BODY_SCALE);
      LCD_PutString(LCD2_START_X, auto_y4, "BTN14               ", ILI9341_YELLOW, ILI9341_BLACK, LCD2_BODY_SCALE);
      LCD_PutString(LCD2_START_X, auto_y5, "ENTER / START       ", ILI9341_YELLOW, ILI9341_BLACK, LCD2_BODY_SCALE);
    }

    LCD2_CS_HIGH();
    osMutexRelease(lcdSpiMutexHandle);

    /* Commit display state only after a complete LCD transaction. */
    g_prev_mode = g_system_mode;
    g_prev_run_state = g_run_state;
    g_prev_estop = g_emergency_stop;
    prev_preset = g_selected_preset;
    prev_homing_status = g_homing_status;
    prev_slave_status_sequence = g_slave_status_sequence;
    prev_teach_save_event_sequence = g_teach_save_event_sequence;
    prev_lcd_event_sequence = g_lcd_event_sequence;
    prev_home_ready = g_home_ready;
  }
}
/* USER CODE END Application */

