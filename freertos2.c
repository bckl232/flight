/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
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

// ---- System / CMSIS ---- //
#include "sys.h"
#include "stdio.h"
#include "string.h"

// ---- STM32 HAL Peripherals ---- //
#include "usart.h"
#include "tim.h"
#include "rtc.h"
#include "stm32f4xx_it.h"
#include "delay.h"

// ---- BSP Drivers ---- //
#include "key.h"
#include "lcd.h"
#include "lcd_init.h"
#include "CST816.h"
#include "DataSave.h"
#include "WDOG.h"
#include "power.h"
#include "KT6328.h"
#include "AHT21.h"
#include "LSM303.h"
#include "SPL06_001.h"
#include "em70x8.h"
#include "HrAlgorythm.h"
#include "MPU6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

// ---- Hardware Abstraction ---- //
#include "HWDataAccess.h"
#include "version.h"

// ---- LVGL ---- //
#include "lvgl.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"

// ---- UI ---- //
#include "ui.h"
#include "ui_HomePage.h"
#include "ui_MenuPage.h"
#include "ui_SetPage.h"
#include "ui_HRPage.h"
#include "ui_SPO2Page.h"
#include "ui_EnvPage.h"
#include "ui_CompassPage.h"
#include "ui_ChargPage.h"
#include "ui_OffTimePage.h"
#include "ui_DateTimeSetPage.h"
#include "ui_TimerPage.h"

// ---- Page Manager ---- //
#include "PageManager.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SCRRENEW_DEPTH  5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* ---------------------------------------------------------------------------*/
/*                         Extern Declarations                                 */
/* ---------------------------------------------------------------------------*/
/* USER CODE BEGIN Externs */

// HAL handles (defined in respective HAL peripheral files)
extern RTC_HandleTypeDef    hrtc;
extern UART_HandleTypeDef   huart1;
extern TIM_HandleTypeDef    htim3;

// Interrupt flags & buffers (stm32f4xx_it.h)
extern uint8_t HardInt_receive_str[25];
extern uint8_t HardInt_uart_flag;
extern uint8_t HardInt_Charg_flag;

// Hardware abstraction layer (HWDataAccess.h)
extern HW_InterfaceTypeDef HWInterface;

// UI globals — light / idle timeout values
extern uint8_t ui_LightSliderValue;
extern uint8_t ui_LTimeValue;
extern uint8_t ui_TTimeValue;

// UI globals — App sync enable flag
extern uint8_t ui_APPSy_EN;

// UI globals — timer page tick counters
extern uint8_t ui_TimerPageFlag;
extern uint8_t ui_TimerPage_min;
extern uint8_t ui_TimerPage_sec;
extern uint8_t ui_TimerPage_10ms;
extern uint8_t ui_TimerPage_ms;

// UI globals — page objects
extern lv_obj_t *ui_HomePage;
extern lv_obj_t *ui_ChargPage;

// Page Manager
extern Page_t Page_Charg;

// System clock reconfig (defined in main.c)
extern void SystemClock_Config(void);

// EM7028 HR algorithm library externs
extern uint8_t GET_BP_MAX(void);
extern uint8_t GET_BP_MIN(void);
extern void    Blood_Process(void);
extern void    Blood_50ms_process(void);
extern void    Blood_500ms_process(void);
extern int     em70xx_bpm_dynamic(int RECEIVED_BYTE, int g_sensor_x, int g_sensor_y, int g_sensor_z);
extern int     em70xx_reset(int ref);

/* USER CODE END Externs */

/* ---------------------------------------------------------------------------*/
/*                    Kernel Object Variables                                  */
/* ---------------------------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* ---------- Global variables ---------- */
uint16_t IdleTimerCount = 0;
uint32_t user_HR_timecount = 0;

/* ---------- Timer handles ---------- */
osTimerId_t IdleTimerHandle;

/* ---------- Thread handles & attributes ---------- */

// Hardware initialization (runs once then deletes itself)
osThreadId_t HardwareInitTaskHandle;
const osThreadAttr_t HardwareInitTask_attributes = {
  .name       = "HardwareInitTask",
  .stack_size = 128 * 10,
  .priority   = (osPriority_t)osPriorityHigh3,
};

// LVGL handler
osThreadId_t LvHandlerTaskHandle;
const osThreadAttr_t LvHandlerTask_attributes = {
  .name       = "LvHandlerTask",
  .stack_size = 128 * 24,
  .priority   = (osPriority_t)osPriorityLow,
};

// Watchdog feed
osThreadId_t WDOGFeedTaskHandle;
const osThreadAttr_t WDOGFeedTask_attributes = {
  .name       = "WDOGFeedTask",
  .stack_size = 128 * 1,
  .priority   = (osPriority_t)osPriorityHigh2,
};

// Idle / screen-dim entry
osThreadId_t IdleEnterTaskHandle;
const osThreadAttr_t IdleEnterTask_attributes = {
  .name       = "IdleEnterTask",
  .stack_size = 128 * 1,
  .priority   = (osPriority_t)osPriorityHigh,
};

// Stop-mode entry & resume
osThreadId_t StopEnterTaskHandle;
const osThreadAttr_t StopEnterTask_attributes = {
  .name       = "StopEnterTask",
  .stack_size = 128 * 16,
  .priority   = (osPriority_t)osPriorityHigh1,
};

// Key scanning
osThreadId_t KeyTaskHandle;
const osThreadAttr_t KeyTask_attributes = {
  .name       = "KeyTask",
  .stack_size = 128 * 1,
  .priority   = (osPriority_t)osPriorityNormal,
};

// Screen / page navigation
osThreadId_t ScrRenewTaskHandle;
const osThreadAttr_t ScrRenewTask_attributes = {
  .name       = "ScrRenewTask",
  .stack_size = 128 * 10,
  .priority   = (osPriority_t)osPriorityLow1,
};

// Sensor data refresh (temperature, humidity, compass, steps, etc.)
osThreadId_t SensorDataTaskHandle;
const osThreadAttr_t SensorDataTask_attributes = {
  .name       = "SensorDataTask",
  .stack_size = 128 * 5,
  .priority   = (osPriority_t)osPriorityLow1,
};

// Heart-rate data refresh
osThreadId_t HRDataTaskHandle;
const osThreadAttr_t HRDataTask_attributes = {
  .name       = "HRDataTask",
  .stack_size = 128 * 5,
  .priority   = (osPriority_t)osPriorityLow1,
};

// Charging-page auto entry / exit
osThreadId_t ChargPageEnterTaskHandle;
const osThreadAttr_t ChargPageEnterTask_attributes = {
  .name       = "ChargPageEnterTask",
  .stack_size = 128 * 10,
  .priority   = (osPriority_t)osPriorityLow1,
};

// BLE message send via UART
osThreadId_t MessageSendTaskHandle;
const osThreadAttr_t MessageSendTask_attributes = {
  .name       = "MessageSendTask",
  .stack_size = 128 * 5,
  .priority   = (osPriority_t)osPriorityLow1,
};

// MPU6050 wrist-orientation check
osThreadId_t MPUCheckTaskHandle;
const osThreadAttr_t MPUCheckTask_attributes = {
  .name       = "MPUCheckTask",
  .stack_size = 128 * 3,
  .priority   = (osPriority_t)osPriorityLow2,
};

// Data persistence to EEPROM
osThreadId_t DataSaveTaskHandle;
const osThreadAttr_t DataSaveTask_attributes = {
  .name       = "DataSaveTask",
  .stack_size = 128 * 5,
  .priority   = (osPriority_t)osPriorityLow2,
};

// Default task (blink LED)
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name       = "defaultTask",
  .stack_size = 128 * 4,
  .priority   = (osPriority_t)osPriorityNormal,
};

/* ---------- Message queue handles ---------- */
osMessageQueueId_t Key_MessageQueue;
osMessageQueueId_t Idle_MessageQueue;
osMessageQueueId_t Stop_MessageQueue;
osMessageQueueId_t IdleBreak_MessageQueue;
osMessageQueueId_t HomeUpdata_MessageQueue;
osMessageQueueId_t DataSave_MessageQueue;

/* USER CODE END Variables */

/* ---------------------------------------------------------------------------*/
/*                    Private Function Prototypes                              */
/* ---------------------------------------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

// Tasks
void StartDefaultTask(void *argument);
void HardwareInitTask(void *argument);
void LvHandlerTask(void *argument);
void WDOGFeedTask(void *argument);
void IdleEnterTask(void *argument);
void StopEnterTask(void *argument);
void KeyTask(void *argument);
void ScrRenewTask(void *argument);
void SensorDataUpdateTask(void *argument);
void HRDataUpdateTask(void *argument);
void ChargPageEnterTask(void *argument);
void MessageSendTask(void *argument);
void MPUCheckTask(void *argument);
void DataSaveTask(void *argument);

// Helpers
void IdleTimerCallback(void *argument);
void TaskTickHook(void);

// KV-parsing helpers for BLE messages
static void StrCMD_Get(uint8_t *str, uint8_t *cmd);
static void TimeFormat_Get(uint8_t *str);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationTickHook(void);

/* USER CODE END FunctionPrototypes */

/* ---------------------------------------------------------------------------*/
/*                       Kernel Initialization                                 */
/* ---------------------------------------------------------------------------*/

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* ---------- Create timers ---------- */
  IdleTimerHandle = osTimerNew(IdleTimerCallback, osTimerPeriodic, NULL, NULL);
  osTimerStart(IdleTimerHandle, 100);   // 100 ms period

  /* ---------- Create message queues ---------- */
  Key_MessageQueue         = osMessageQueueNew(1, 1, NULL);
  Idle_MessageQueue        = osMessageQueueNew(1, 1, NULL);
  Stop_MessageQueue        = osMessageQueueNew(1, 1, NULL);
  IdleBreak_MessageQueue   = osMessageQueueNew(1, 1, NULL);
  HomeUpdata_MessageQueue  = osMessageQueueNew(1, 1, NULL);
  DataSave_MessageQueue    = osMessageQueueNew(2, 1, NULL);

  /* ---------- Create threads ---------- */
  HardwareInitTaskHandle   = osThreadNew(HardwareInitTask,      NULL, &HardwareInitTask_attributes);
  LvHandlerTaskHandle      = osThreadNew(LvHandlerTask,         NULL, &LvHandlerTask_attributes);
  WDOGFeedTaskHandle       = osThreadNew(WDOGFeedTask,          NULL, &WDOGFeedTask_attributes);
  IdleEnterTaskHandle      = osThreadNew(IdleEnterTask,         NULL, &IdleEnterTask_attributes);
  StopEnterTaskHandle      = osThreadNew(StopEnterTask,         NULL, &StopEnterTask_attributes);
  KeyTaskHandle            = osThreadNew(KeyTask,               NULL, &KeyTask_attributes);
  ScrRenewTaskHandle       = osThreadNew(ScrRenewTask,          NULL, &ScrRenewTask_attributes);
  SensorDataTaskHandle     = osThreadNew(SensorDataUpdateTask,  NULL, &SensorDataTask_attributes);
  HRDataTaskHandle         = osThreadNew(HRDataUpdateTask,      NULL, &HRDataTask_attributes);
  ChargPageEnterTaskHandle = osThreadNew(ChargPageEnterTask,    NULL, &ChargPageEnterTask_attributes);
  MessageSendTaskHandle    = osThreadNew(MessageSendTask,       NULL, &MessageSendTask_attributes);
  MPUCheckTaskHandle       = osThreadNew(MPUCheckTask,          NULL, &MPUCheckTask_attributes);
  DataSaveTaskHandle       = osThreadNew(DataSaveTask,          NULL, &DataSaveTask_attributes);

  // Default task (LED blink)
  defaultTaskHandle        = osThreadNew(StartDefaultTask,      NULL, &defaultTask_attributes);

  /* ---------- Trigger initial Home screen update ---------- */
  uint8_t HomeUpdataStr = 0;
  osMessageQueuePut(HomeUpdata_MessageQueue, &HomeUpdataStr, 0, 1);

  /* USER CODE END Init */

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

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* ---------------------------------------------------------------------------*/
/*                         FreeRTOS Hooks                                     */
/* ---------------------------------------------------------------------------*/

/* USER CODE BEGIN 2 */
void vApplicationIdleHook(void)
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
void vApplicationTickHook(void)
{
   /* This function will be called by each tick interrupt if
   configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
   added here, but the tick hook is called from an interrupt context, so
   code must not attempt to block, and only the interrupt safe FreeRTOS API
   functions can be used (those that end in FromISR()). */
    TaskTickHook();
}
/* USER CODE END 3 */

/**
  * @brief  FreeRTOS Tick Hook — increments LVGL tick and timer-page counters
  * @param  None
  * @retval None
  */
void TaskTickHook(void)
{
    // Increment the LVGL tick
    lv_tick_inc(1);

    // Timer-page real-time counter
    if (ui_TimerPageFlag)
    {
        ui_TimerPage_ms += 1;
        if (ui_TimerPage_ms >= 10)
        {
            ui_TimerPage_ms = 0;
            ui_TimerPage_10ms += 1;
        }
        if (ui_TimerPage_10ms >= 100)
        {
            ui_TimerPage_10ms = 0;
            ui_TimerPage_sec += 1;
            uint8_t IdleBreakstr = 0;
            osMessageQueuePut(IdleBreak_MessageQueue, &IdleBreakstr, 0, 0);
        }
        if (ui_TimerPage_sec >= 60)
        {
            ui_TimerPage_sec = 0;
            ui_TimerPage_min += 1;
        }
        if (ui_TimerPage_min >= 60)
        {
            ui_TimerPage_min = 0;
        }
    }

    // HR sensor time count
    user_HR_timecount += 1;
}

/* ---------------------------------------------------------------------------*/
/*                        Idle Timer Callback                                 */
/* ---------------------------------------------------------------------------*/

/**
  * @brief  Software timer callback — manages idle → screen-off → stop transitions
  * @param  argument: Not used
  * @retval None
  */
void IdleTimerCallback(void *argument)
{
    IdleTimerCount += 1;

    // Dim the screen after ui_LTimeValue seconds
    if (IdleTimerCount == (ui_LTimeValue * 10))
    {
        uint8_t Idlestr = 0;
        osMessageQueuePut(Idle_MessageQueue, &Idlestr, 0, 1);
    }

    // Enter stop mode after ui_TTimeValue seconds
    if (IdleTimerCount == (ui_TTimeValue * 10))
    {
        uint8_t Stopstr = 1;
        IdleTimerCount  = 0;
        osMessageQueuePut(Stop_MessageQueue, &Stopstr, 0, 1);
    }
}

/* ---------------------------------------------------------------------------*/
/*                          Task Implementations                               */
/* ---------------------------------------------------------------------------*/

/* ====================== defaultTask (LED heartbeat) ======================= */

/**
  * @brief  Default task — blinks PC13 LED every 500 ms
  * @param  argument: Not used
  * @retval None
  */
void StartDefaultTask(void *argument)
{
  for (;;)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    osDelay(500);
  }
}

/* ====================== HardwareInitTask ================================== */

/**
  * @brief  Hardware initialization — runs once at boot, then deletes itself
  * @param  argument: Not used
  * @retval None
  */
void HardwareInitTask(void *argument)
{
    while (1)
    {
        vTaskSuspendAll();

        // ---- RTC Wake-up timer ---- //
        if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 2000, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
        {
            Error_Handler();
        }

        // ---- UART (BLE) start ---- //
        HAL_UART_Receive_DMA(&huart1, (uint8_t *)HardInt_receive_str, 25);
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

        // ---- PWM start (LCD backlight) ---- //
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

        // ---- SysTick delay ---- //
        delay_init();

        // ---- Power / battery ---- //
        HWInterface.Power.Init();

        // ---- Key GPIO ---- //
        Key_Port_Init();

        // ---- AHT21 temperature & humidity (3 retries) ---- //
        {
            uint8_t num = 3;
            while (num && HWInterface.AHT21.ConnectionError)
            {
                num--;
                HWInterface.AHT21.ConnectionError = HWInterface.AHT21.Init();
            }
        }

        // ---- E-compass LSM303 (3 retries) ---- //
        {
            uint8_t num = 3;
            while (num && HWInterface.Ecompass.ConnectionError)
            {
                num--;
                HWInterface.Ecompass.ConnectionError = HWInterface.Ecompass.Init();
            }
        }
        if (!HWInterface.Ecompass.ConnectionError)
            HWInterface.Ecompass.Sleep();

        // ---- Barometer SPL06 (3 retries) ---- //
        {
            uint8_t num = 3;
            while (num && HWInterface.Barometer.ConnectionError)
            {
                num--;
                HWInterface.Barometer.ConnectionError = HWInterface.Barometer.Init();
            }
        }

        // ---- IMU MPU6050 (3 retries) ---- //
        {
            uint8_t num = 3;
            while (num && HWInterface.IMU.ConnectionError)
            {
                num--;
                HWInterface.IMU.ConnectionError = HWInterface.IMU.Init();
            }
        }

        // ---- Heart-rate EM7028 (3 retries) ---- //
        {
            uint8_t num = 3;
            while (num && HWInterface.HR_meter.ConnectionError)
            {
                num--;
                HWInterface.HR_meter.ConnectionError = HWInterface.HR_meter.Init();
            }
        }
        if (!HWInterface.HR_meter.ConnectionError)
            HWInterface.HR_meter.Sleep();

        // ---- EEPROM / persistent settings ---- //
        EEPROM_Init();
        if (!EEPROM_Check())
        {
            uint8_t recbuf[3];
            SettingGet(recbuf, 0x10, 2);
            if ((recbuf[0] != 0 && recbuf[0] != 1) || (recbuf[1] != 0 && recbuf[1] != 1))
            {
                HWInterface.IMU.wrist_is_enabled = 0;
                ui_APPSy_EN = 0;
            }
            else
            {
                HWInterface.IMU.wrist_is_enabled = recbuf[0];
                ui_APPSy_EN = recbuf[1];
            }

            RTC_DateTypeDef nowdate;
            HAL_RTC_GetDate(&hrtc, &nowdate, RTC_FORMAT_BIN);

            SettingGet(recbuf, 0x20, 3);
            if (recbuf[0] == nowdate.Date)
            {
                uint16_t steps = 0;
                steps  = recbuf[1] & 0x00ff;
                steps  = steps << 8 | recbuf[2];
                if (!HWInterface.IMU.ConnectionError)
                    dmp_set_pedometer_step_count((unsigned long)steps);
            }
        }

        // ---- BLE KT6328 ---- //
        KT6328_GPIO_Init();
        KT6328_Disable();

        // ---- Touch CST816 ---- //
        CST816_GPIO_Init();
        CST816_RESET();

        // ---- LCD ---- //
        LCD_Init();
        LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
        delay_ms(10);
        LCD_Set_Light(50);
        LCD_ShowString(72, LCD_H / 2, (uint8_t *)"Welcome!", WHITE, BLACK, 24, 0);
        {
            uint8_t lcd_buf_str[17];
            sprintf((char *)lcd_buf_str, "OV-Watch V%d.%d.%d",
                    watch_version_major(), watch_version_minor(), watch_version_patch());
            LCD_ShowString(34, LCD_H / 2 + 48, (uint8_t *)lcd_buf_str, WHITE, BLACK, 24, 0);
        }
        delay_ms(1000);
        LCD_Fill(0, LCD_H / 2 - 24, LCD_W, LCD_H / 2 + 49, BLACK);

        // ---- LVGL and UI ---- //
        lv_init();
        lv_port_disp_init();
        lv_port_indev_init();
        ui_init();

        xTaskResumeAll();

        // One-shot task — delete itself after initialization
        osThreadExit();
    }
}

/* ====================== LvHandlerTask (LVGL tick driver) ================== */

/**
  * @brief  LVGL handler — calls lv_task_handler() every ms, breaks idle on activity
  * @param  argument: Not used
  * @retval None
  */
void LvHandlerTask(void *argument)
{
    uint8_t IdleBreakstr = 0;
    while (1)
    {
        if (lv_disp_get_inactive_time(NULL) < 1000)
        {
            // User interaction detected — reset idle timer
            osMessageQueuePut(IdleBreak_MessageQueue, &IdleBreakstr, 0, 0);
        }
        lv_task_handler();
        osDelay(1);
    }
}

/* ====================== WDOGFeedTask (watchdog feeder) ==================== */

/**
  * @brief  Watchdog feed — feeds the external WDOG every 100 ms
  * @param  argument: Not used
  * @retval None
  */
void WDOGFeedTask(void *argument)
{
    WDOG_Port_Init();
    while (1)
    {
        WDOG_Feed();
        WDOG_Enable();
        osDelay(100);
    }
}

/* ====================== IdleEnterTask (screen dimming) ==================== */

/**
  * @brief  Idle enter — dims the screen on idle, restores on break
  * @param  argument: Not used
  * @retval None
  */
void IdleEnterTask(void *argument)
{
    uint8_t Idlestr      = 0;
    uint8_t IdleBreakstr = 0;
    while (1)
    {
        // Dim screen
        if (osMessageQueueGet(Idle_MessageQueue, &Idlestr, NULL, 1) == osOK)
        {
            LCD_Set_Light(5);
        }
        // Restore brightness (key press, touch, etc.)
        if (osMessageQueueGet(IdleBreak_MessageQueue, &IdleBreakstr, NULL, 1) == osOK)
        {
            IdleTimerCount = 0;
            LCD_Set_Light(ui_LightSliderValue);
        }
        osDelay(10);
    }
}

/* ====================== StopEnterTask (stop-mode entry/resume) ============ */

/**
  * @brief  Stop enter — puts the MCU in STOP mode and resumes on wake event
  * @param  argument: Not used
  * @retval None
  */
void StopEnterTask(void *argument)
{
    uint8_t Stopstr;
    uint8_t HomeUpdataStr;
    uint8_t Wrist_Flag = 0;
    while (1)
    {
        if (osMessageQueueGet(Stop_MessageQueue, &Stopstr, NULL, 0) == osOK)
        {
sleep:
            IdleTimerCount = 0;

            // ---- Pre-sleep: deinit peripherals ---- //
            HAL_UART_MspDeInit(&huart1);
            LCD_RES_Clr();
            LCD_Close_Light();
            CST816_Sleep();

            // ---- Enter STOP ---- //
            vTaskSuspendAll();
            WDOG_Disnable();
            CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk);
            HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);

            // ---- Wake-up: restore clocks ---- //
            SET_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk);
            HAL_SYSTICK_Config(SystemCoreClock / (1000U / uwTickFreq));
            SystemClock_Config();
            WDOG_Feed();
            xTaskResumeAll();

            // ---- Post-wake: wrist-gesture check ---- //
            if (HWInterface.IMU.wrist_is_enabled)
            {
                uint8_t hor;
                hor = MPU_isHorizontal();
                if (hor && HWInterface.IMU.wrist_state == WRIST_DOWN)
                {
                    HWInterface.IMU.wrist_state = WRIST_UP;
                    Wrist_Flag = 1;
                }
                else if (!hor && HWInterface.IMU.wrist_state == WRIST_UP)
                {
                    HWInterface.IMU.wrist_state = WRIST_DOWN;
                    IdleTimerCount = 0;
                    goto sleep;
                }
            }

            // Check for valid wake source
            if (!KEY1 || KEY2 || HardInt_Charg_flag || Wrist_Flag)
            {
                Wrist_Flag = 0;
                // Valid wake — restore peripherals
            }
            else
            {
                IdleTimerCount = 0;
                goto sleep;
            }

            // ---- Restore peripherals ---- //
            HAL_UART_MspInit(&huart1);
            LCD_Init();
            LCD_Set_Light(ui_LightSliderValue);
            CST816_Wakeup();

            if (ChargeCheck())
            {
                HardInt_Charg_flag = 1;
            }

            // Notify Home screen to refresh
            osMessageQueuePut(HomeUpdata_MessageQueue, &HomeUpdataStr, 0, 1);
        }
        osDelay(100);
    }
}

/* ====================== KeyTask (button scanning) ========================= */

/**
  * @brief  Key task — scans hardware keys and posts messages
  * @param  argument: Not used
  * @retval None
  */
void KeyTask(void *argument)
{
    uint8_t keystr       = 0;
    uint8_t Stopstr      = 0;
    uint8_t IdleBreakstr = 0;
    while (1)
    {
        switch (KeyScan(0))
        {
            case 1:     // KEY1 short press → "back"
                keystr = 1;
                osMessageQueuePut(Key_MessageQueue,       &keystr,       0, 1);
                osMessageQueuePut(IdleBreak_MessageQueue, &IdleBreakstr, 0, 1);
                break;

            case 2:     // KEY2
                if (Page_Get_NowPage()->page_obj == &ui_HomePage)
                {
                    // On home screen → enter stop mode
                    osMessageQueuePut(Stop_MessageQueue, &Stopstr, 0, 1);
                }
                else
                {
                    // On any other screen → "back-to-bottom"
                    keystr = 2;
                    osMessageQueuePut(Key_MessageQueue,       &keystr,       0, 1);
                    osMessageQueuePut(IdleBreak_MessageQueue, &IdleBreakstr, 0, 1);
                }
                break;
        }
        osDelay(1);
    }
}

/* ====================== ScrRenewTask (page navigation) ==================== */

/**
  * @brief  Screen renew — processes key events for page navigation
  * @param  argument: Not used
  * @retval None
  */
void ScrRenewTask(void *argument)
{
    uint8_t keystr = 0;
    while (1)
    {
        if (osMessageQueueGet(Key_MessageQueue, &keystr, NULL, 0) == osOK)
        {
            if (keystr == 1)    // KEY1 → go back one page
            {
                Page_Back();
                if (Page_Get_NowPage()->page_obj == &ui_MenuPage)
                {
                    EM7028_hrs_DisEnable();     // HR sensor sleep
                    LSM303DLH_Sleep();          // e-compass sleep
                }
            }
            else if (keystr == 2)   // KEY2 → go back to bottom (home)
            {
                Page_Back_Bottom();
                EM7028_hrs_DisEnable();
                LSM303DLH_Sleep();
            }
        }
        osDelay(10);
    }
}

/* ====================== SensorDataUpdateTask ============================== */

/**
  * @brief  Sensor data update — refreshes all sensor readings for the active page
  * @param  argument: Not used
  * @retval None
  */
void SensorDataUpdateTask(void *argument)
{
    uint8_t IdleBreakstr = 0;
    while (1)
    {
        // ---- Home-screen periodic update ---- //
        uint8_t HomeUpdataStr;
        if (osMessageQueueGet(HomeUpdata_MessageQueue, &HomeUpdataStr, NULL, 0) == osOK)
        {
            // Battery
            HWInterface.Power.power_remain = HWInterface.Power.BatCalculate();
            if (!(HWInterface.Power.power_remain > 0 && HWInterface.Power.power_remain <= 100))
            {
                HWInterface.Power.power_remain = 0;
            }

            // Steps
            if (!(HWInterface.IMU.ConnectionError))
            {
                HWInterface.IMU.Steps = HWInterface.IMU.GetSteps();
            }

            // Temperature & humidity
            if (!(HWInterface.AHT21.ConnectionError))
            {
                float humi, temp;
                HWInterface.AHT21.GetHumiTemp(&humi, &temp);
                if (temp > -10 && temp < 50 && humi > 0 && humi < 100)
                {
                    HWInterface.AHT21.humidity    = (uint8_t)humi;
                    HWInterface.AHT21.temperature = (uint8_t)temp;
                }
            }

            // Kick data-save task
            uint8_t Datastr = 3;
            osMessageQueuePut(DataSave_MessageQueue, &Datastr, 0, 1);
        }

        // ---- Page-specific updates ---- //

        // SPO2 Page (placeholder)
        if (Page_Get_NowPage()->page_obj == &ui_SPO2Page)
        {
            osMessageQueuePut(IdleBreak_MessageQueue, &IdleBreakstr, 0, 1);
            // TODO: SPO2 measurement
        }
        // Environment Page
        else if (Page_Get_NowPage()->page_obj == &ui_EnvPage)
        {
            osMessageQueuePut(IdleBreak_MessageQueue, &IdleBreakstr, 0, 1);
            if (!HWInterface.AHT21.ConnectionError)
            {
                float humi, temp;
                HWInterface.AHT21.GetHumiTemp(&humi, &temp);
                if (temp > -10 && temp < 50 && humi > 0 && humi < 100)
                {
                    HWInterface.AHT21.temperature = (int8_t)temp;
                    HWInterface.AHT21.humidity    = (int8_t)humi;
                }
            }
        }
        // Compass Page
        else if (Page_Get_NowPage()->page_obj == &ui_CompassPage)
        {
            osMessageQueuePut(IdleBreak_MessageQueue, &IdleBreakstr, 0, 1);
            LSM303DLH_Wakeup();

            if (!HWInterface.Ecompass.ConnectionError)
            {
                int16_t Xa, Ya, Za, Xm, Ym, Zm;
                LSM303_ReadAcceleration(&Xa, &Ya, &Za);
                LSM303_ReadMagnetic(&Xm, &Ym, &Zm);
                float temp = Azimuth_Calculate(Xa, Ya, Za, Xm, Ym, Zm) + 0;   // +0 offset
                if (temp < 0) { temp += 360; }
                if (temp >= 0 && temp <= 360)
                {
                    HWInterface.Ecompass.direction = (uint16_t)temp;
                }
            }

            if (!HWInterface.Barometer.ConnectionError)
            {
                float alti = Altitude_Calculate();
                HWInterface.Barometer.altitude = (int16_t)alti;
            }
        }

        osDelay(500);
    }
}

/* ====================== HRDataUpdateTask ================================== */

/**
  * @brief  HR data update — reads HR sensor when on the HR page
  * @param  argument: Not used
  * @retval None
  */
void HRDataUpdateTask(void *argument)
{
    uint8_t  IdleBreakstr = 0;
    uint8_t  hr_temp      = 0;
    while (1)
    {
        if (Page_Get_NowPage()->page_obj == &ui_HRPage)
        {
            osMessageQueuePut(IdleBreak_MessageQueue, &IdleBreakstr, 0, 1);
            EM7028_hrs_Enable();

            if (!HWInterface.HR_meter.ConnectionError)
            {
                vTaskSuspendAll();
                hr_temp = HR_Calculate(EM7028_Get_HRS1(), user_HR_timecount);
                xTaskResumeAll();

                if (HWInterface.HR_meter.HrRate != hr_temp && hr_temp > 50 && hr_temp < 120)
                {
                    HWInterface.HR_meter.HrRate = hr_temp;
                }
            }
        }
        osDelay(50);
    }
}

/* ====================== ChargPageEnterTask ================================ */

/**
  * @brief  Charging page enter — auto-switches to/from charging UI on plug/unplug
  * @param  argument: Not used
  * @retval None
  */
void ChargPageEnterTask(void *argument)
{
    while (1)
    {
        if (HardInt_Charg_flag)
        {
            IdleTimerCount     = 0;
            HardInt_Charg_flag = 0;

            if ((ChargeCheck()) && (Page_Get_NowPage()->page_obj != &ui_ChargPage))
            {
                Page_Load(&Page_Charg);
            }
            else if ((!ChargeCheck()) && (Page_Get_NowPage()->page_obj == &ui_ChargPage))
            {
                Page_Back();
            }
        }
        osDelay(500);
    }
}

/* ====================== MessageSendTask (BLE UART) ======================== */

/* --- BLE command parser helpers --- */

/* Extract the command part before '='  (e.g. "OV+ST=..." → "OV+ST") */
static void StrCMD_Get(uint8_t *str, uint8_t *cmd)
{
    uint8_t i = 0;
    while (str[i] != '=')
    {
        cmd[i] = str[i];
        i++;
    }
}

/* Parse a time-set string: OV+ST=YYYYMMDDHHMMSS (20 chars total) */
static void TimeFormat_Get(uint8_t *str)
{
    RTC_DateTypeDef setdate = {0};
    RTC_TimeTypeDef settime = {0};

    setdate.Year    = (str[8]  - '0') * 10 + str[9]  - '0';
    setdate.Month   = (str[10] - '0') * 10 + str[11] - '0';
    setdate.Date    = (str[12] - '0') * 10 + str[13] - '0';
    settime.Hours   = (str[14] - '0') * 10 + str[15] - '0';
    settime.Minutes = (str[16] - '0') * 10 + str[17] - '0';
    settime.Seconds = (str[18] - '0') * 10 + str[19] - '0';

    if (setdate.Year > 0 && setdate.Year < 99
        && setdate.Month > 0 && setdate.Month <= 12
        && setdate.Date > 0 && setdate.Date <= 31
        && settime.Hours <= 23
        && settime.Minutes <= 59
        && settime.Seconds <= 59)
    {
        RTC_SetDate(setdate.Year, setdate.Month, setdate.Date);
        RTC_SetTime(settime.Hours, settime.Minutes, settime.Seconds);
        printf("TIMESETOK\r\n");
    }
}

/**
  * @brief  BLE message send — handles UART commands received over BLE
  * @param  argument: Not used
  * @retval None
  */
void MessageSendTask(void *argument)
{
    while (1)
    {
        if (HardInt_uart_flag)
        {
            HardInt_uart_flag = 0;
            uint8_t IdleBreakstr = 0;
            osMessageQueuePut(IdleBreak_MessageQueue, &IdleBreakstr, NULL, 1);
            printf("RecStr:%s\r\n", HardInt_receive_str);

            // ---- Command dispatch ---- //

            // Ping
            if (!strcmp((const char *)HardInt_receive_str, "OV"))
            {
                printf("OK\r\n");
            }
            // Firmware version
            else if (!strcmp((const char *)HardInt_receive_str, "OV+VERSION"))
            {
                printf("VERSION=V%d.%d.%d\r\n", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
            }
            // Full sensor data dump
            else if (!strcmp((const char *)HardInt_receive_str, "OV+SEND"))
            {
                RTC_DateTypeDef nowdate;
                RTC_TimeTypeDef nowtime;
                int8_t  humi;
                int8_t  temp;
                uint8_t HR;
                uint8_t SPO2;
                uint16_t stepNum;

                HAL_RTC_GetTime(&hrtc, &nowtime, RTC_FORMAT_BIN);
                HAL_RTC_GetDate(&hrtc, &nowdate, RTC_FORMAT_BIN);
                humi    = (int8_t)HWInterface.AHT21.humidity;
                temp    = (int8_t)HWInterface.AHT21.temperature;
                HR      = HWInterface.HR_meter.HrRate;
                SPO2    = HWInterface.HR_meter.SPO2;
                stepNum = HWInterface.IMU.Steps;

                printf("data:%2d-%02d\r\n",          nowdate.Month, nowdate.Date);
                printf("time:%02d:%02d:%02d\r\n",    nowtime.Hours, nowtime.Minutes, nowtime.Seconds);
                printf("humidity:%d%%\r\n",           humi);
                printf("temperature:%d\r\n",          temp);
                printf("Heart Rate:%d%%\r\n",         HR);
                printf("SPO2:%d%%\r\n",               SPO2);
                printf("Step today:%d\r\n",           stepNum);
            }
            // Set time (OV+ST=YYYYMMDDHHMMSS, length 20)
            else if (strlen((const char *)HardInt_receive_str) == 20)
            {
                uint8_t cmd[10];
                memset(cmd, 0, sizeof(cmd));
                StrCMD_Get(HardInt_receive_str, cmd);
                if (ui_APPSy_EN && !strcmp((const char *)cmd, "OV+ST"))
                {
                    TimeFormat_Get(HardInt_receive_str);
                }
            }

            memset(HardInt_receive_str, 0, sizeof(HardInt_receive_str));
        }
        osDelay(1000);
    }
}

/* ====================== MPUCheckTask (wrist orientation) ================== */

/**
  * @brief  MPU check — monitors wrist orientation for auto-sleep
  * @param  argument: Not used
  * @retval None
  */
void MPUCheckTask(void *argument)
{
    while (1)
    {
        if (HWInterface.IMU.wrist_is_enabled)
        {
            if (MPU_isHorizontal())
            {
                HWInterface.IMU.wrist_state = WRIST_UP;
            }
            else
            {
                if (WRIST_UP == HWInterface.IMU.wrist_state)
                {
                    HWInterface.IMU.wrist_state = WRIST_DOWN;
                    if (Page_Get_NowPage()->page_obj == &ui_HomePage ||
                        Page_Get_NowPage()->page_obj == &ui_MenuPage  ||
                        Page_Get_NowPage()->page_obj == &ui_SetPage)
                    {
                        uint8_t Stopstr = 0;
                        osMessageQueuePut(Stop_MessageQueue, &Stopstr, 0, 1);  // → sleep
                    }
                }
                HWInterface.IMU.wrist_state = WRIST_DOWN;
            }
        }
        osDelay(300);
    }
}

/* ====================== DataSaveTask (EEPROM persistence) ================= */

/**
  * @brief  Data save — persists settings and daily step count to EEPROM
  * @param  argument: Not used
  * @retval None
  *
  * EEPROM layout:
  *  [0x00]: 0x55  check byte
  *  [0x01]: 0xAA  check byte
  *  [0x10]: wrist_is_enabled
  *  [0x11]: ui_APPSy_EN
  *  [0x20]: Last-save day (0-31)
  *  [0x21]: Day steps low
  *  [0x22]: Day steps high
  */
void DataSaveTask(void *argument)
{
    while (1)
    {
        uint8_t Datastr = 0;
        if (osMessageQueueGet(DataSave_MessageQueue, &Datastr, NULL, 1) == osOK)
        {
            // ---- Save user settings ---- //
            uint8_t dat[3];
            dat[0] = HWInterface.IMU.wrist_is_enabled;
            dat[1] = ui_APPSy_EN;
            SettingSave(dat, 0x10, 2);

            // ---- Date-change / steps logic ---- //
            RTC_DateTypeDef nowdate;
            HAL_RTC_GetDate(&hrtc, &nowdate, RTC_FORMAT_BIN);

            SettingGet(dat, 0x20, 3);
            if (dat[0] != nowdate.Date)
            {
                // New day → reset step counter
                if (!HWInterface.IMU.ConnectionError)
                    HWInterface.IMU.SetSteps(0);

                dat[0] = nowdate.Date;
                dat[2] = 0;
                dat[1] = 0;
                SettingSave(dat, 0x20, 3);
            }
            else
            {
                // Same day → persist current step count
                uint16_t temp = HWInterface.IMU.GetSteps();
                dat[0] = nowdate.Date;
                dat[2] = temp & 0xff;
                dat[1] = (temp >> 8) & 0xff;
                SettingSave(dat, 0x20, 3);
            }
        }
        osDelay(100);
    }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
