/**
 * @file    freertos.c
 * @brief   所有 FreeRTOS 任务实现 (CMSIS OS v2 版本)
 *
 *          包含飞行控制系统的全部 4 个任务:
 *            - power_task   电源管理 (10s 周期)
 *            - flight_task  飞行控制 (6ms 周期, ~166Hz)
 *            - led_task     LED 状态指示 (100ms 周期)
 *            - com_task     无线通讯 (10ms 周期, 100Hz)
 *

 *          STM32F103C8T6 SRAM 20KB → 分配 12KB 给 RTOS 堆
 */

#include "freertos.h"
#include "Com_debug.h"
#include "Int_IP5305T.h"
#include "Int_motor.h"
#include "Int_SI24R1.h"
#include "Int_bat_ADC.h"
#include "App_receive_data.h"
#include "App_flight.h"

/* ================================================================
 *  Global variables - LED 结构体
 * ================================================================ */
LED_Struct left_top_led     = { .port = LED1_GPIO_Port, .pin = LED1_Pin };
LED_Struct right_top_led    = { .port = LED2_GPIO_Port, .pin = LED2_Pin };
LED_Struct right_bottom_led = { .port = LED3_GPIO_Port, .pin = LED3_Pin };
LED_Struct left_bottom_led  = { .port = LED4_GPIO_Port, .pin = LED4_Pin };

/* ================================================================
 *  Global variables - 系统状态
 * ================================================================ */

/* 指示当前的遥控连接状态 */
Remote_State remote_state = REMOTE_DISCONNECTED;

/* 指示当前的飞行状态 */
Flight_State flight_state = IDLE;

/* 遥控器接收到的遥控数据 (默认油门=0, 摇杆中位=500) */
Remote_Data remote_data = {
    .thr        = 0,
    .yaw        = 500,
    .pit        = 500,
    .rol        = 500,
    .fix_height = 0,
    .shutdown   = 0
};

/* 首次定高时记录的高度值 */
uint16_t fix_height = 0;

/* 回传数据缓冲区: 电池电压字符串 → 通过 SI24R1 发回遥控器 */
uint8_t back_buff[TX_PLOAD_WIDTH] = { 0 };

/* ================================================================
 *  Task handles (osThreadId_t = CMSIS OS v2 线程ID类型)
 * ================================================================ */
osThreadId_t power_task_handle  = NULL;
osThreadId_t flight_task_handle = NULL;
osThreadId_t led_task_handle    = NULL;
osThreadId_t com_task_handle    = NULL;

/* ================================================================
 *  Task function forward declarations
 * ================================================================ */
__NO_RETURN void power_task(void *args);
__NO_RETURN void flight_task(void *args);
__NO_RETURN void led_task(void *args);
__NO_RETURN void com_task(void *args);

/* ================================================================
 *  Task attribute definitions (osThreadAttr_t)
 *  stack_size 单位: bytes (FreeRTOS 用 words, 128 words * 4 = 512 bytes)
 * ================================================================ */
static const osThreadAttr_t power_task_attrs = {
    .name       = "power_task",
    .stack_size = POWER_TASK_STACK_SIZE,
    .priority   = POWER_TASK_PRIORITY,
};

static const osThreadAttr_t flight_task_attrs = {
    .name       = "flight_task",
    .stack_size = FLIGHT_TASK_STACK_SIZE,
    .priority   = FLIGHT_TASK_PRIORITY,
};

static const osThreadAttr_t led_task_attrs = {
    .name       = "led_task",
    .stack_size = LED_TASK_STACK_SIZE,
    .priority   = LED_TASK_PRIORITY,
};

static const osThreadAttr_t com_task_attrs = {
    .name       = "com_task",
    .stack_size = COM_TASK_STACK_SIZE,
    .priority   = COM_TASK_PRIORITY,
};

/* ================================================================
 *  App_freertos_start() - 初始化并启动 FreeRTOS 调度器
 *
 *  调用顺序:
 *    1. osKernelInitialize()    初始化内核
 *    2. osThreadNew() × 4       创建所有任务
 *    3. osKernelStart()         启动调度器 (永不返回)
 * ================================================================ */
void App_freertos_start(void)
{
    /* 1. 初始化 CMSIS-RTOS v2 内核 */
    osKernelInitialize();

    /* 2. 创建电源管理任务 */
    power_task_handle = osThreadNew(power_task, NULL, &power_task_attrs);
    if (power_task_handle == NULL) {
        Error_Handler();
    }

    /* 3. 创建飞行控制任务 */
    flight_task_handle = osThreadNew(flight_task, NULL, &flight_task_attrs);
    if (flight_task_handle == NULL) {
        Error_Handler();
    }

    /* 4. 创建 LED 指示任务 */
    led_task_handle = osThreadNew(led_task, NULL, &led_task_attrs);
    if (led_task_handle == NULL) {
        Error_Handler();
    }

    /* 5. 创建无线通讯任务 */
    com_task_handle = osThreadNew(com_task, NULL, &com_task_attrs);
    if (com_task_handle == NULL) {
        Error_Handler();
    }

    /* 6. 启动调度器 — 此函数永不返回 */
    osKernelStart();

    /* 调度器启动失败才会走到这里 */
    for (;;) {
        __NOP();
    }
}

/* ================================================================
 *  power_task() - 电源管理任务
 *
 *  功能:
 *    使用 osThreadFlagsWait() 阻塞等待关机通知,
 *    超时 10s 后自动执行电源保活 (Int_IP5305T_start);
 *    收到关机通知时执行关机操作 (Int_IP5305T_shutdown)
 *
 *  CMSIS OS v2 替代:
 *    ulTaskNotifyTake(pdTRUE, timeout) → osThreadFlagsWait(flag, osFlagsWaitAny, timeout)
 * ================================================================ */
__NO_RETURN void power_task(void *args)
{
    (void)args;

    for (;;) {
        /*
         * 阻塞等待 POWER_TASK_FLAG 标志位, 超时时间 10s
         * - 收到标志 (res > 0): 执行关机
         * - 超时 (res == 0): 执行电源保活
         */
        uint32_t res = osThreadFlagsWait(POWER_TASK_FLAG,
                                         osFlagsWaitAny,
                                         POWER_TASK_PERIOD);

        if (res & POWER_TASK_FLAG) {
            /* 收到关机通知 → 执行关机 */
            Int_IP5305T_shutdown();
        } else if (res == osFlagsErrorTimeout) {
            /* 超时 (10s) → 保持电源活跃, 防止自动关机 */
            Int_IP5305T_start();
        }
        /* 如果返回 osFlagsErrorResource 等错误, 忽略, 继续循环 */
    }
}

/* ================================================================
 *  flight_task() - 飞行控制任务
 *
 *  功能 (6ms 周期, ~166Hz):
 *    1. 读取 MPU6050 数据, 通过姿态解算获得欧拉角
 *    2. 执行级联 PID 控制 (外环角度 → 内环角速度)
 *    3. 定高模式下每 4 个周期 (24ms) 执行一次高度 PID
 *    4. 将 PID 输出混合后控制 4 个电机
 *
 *  CMSIS OS v2 替代:
 *    vTaskDelayUntil(&xLastWakeTime, period) → osDelayUntil(wakeup_time)
 *    xTaskGetTickCount()                     → osKernelGetTickCount()
 * ================================================================ */
__NO_RETURN void flight_task(void *args)
{
    (void)args;

    /* 获取当前基准时间 */
    uint32_t wakeup_time = osKernelGetTickCount();
    uint8_t  count       = 0;

    /* 必须先初始化 MPU6050, 电机, VL53L1X 后才能读取数据 */
    App_flight_init();

    for (;;) {
        /* 1. 读取 MPU6050 传感器数据, 姿态解算得到欧拉角 */
        App_flight_get_euler_angle();

        /* 2. 根据当前欧拉角和遥控器目标值执行 PID 解算 */
        App_flight_pid_process();

        /* 3. 定高模式下的高度 PID (每 4 周期执行一次, 24ms) */
        if (flight_state == FIX_HEIGHT) {
            count++;
            if (count >= 4) {
                App_flight_fix_height_pid_process();
                count = 0;
            }
        }

        /* 4. 根据 PID 计算结果控制电机转速 */
        App_flight_control_motor();

        /*
         * 5. 精确延时到下一个周期
         *    osDelayUntil() 基于绝对时间睡眠, 不受任务执行时间影响
         */
        wakeup_time += FLIGHT_TASK_PERIOD;
        osDelayUntil(wakeup_time);
    }
}

/* ================================================================
 *  led_task() - LED 状态指示任务
 *
 *  功能 (100ms 周期, 10Hz):
 *    前灯 (left_top, right_top): 指示遥控连接状态
 *      - REMOTE_CONNECTED    → 常亮
 *      - REMOTE_DISCONNECTED → 熄灭
 *    后灯 (left_bottom, right_bottom): 指示飞行状态
 *      - IDLE       → 慢闪 (500ms 亮 / 500ms 灭)
 *      - NORMAL     → 快闪 (200ms 亮 / 200ms 灭)
 *      - FIX_HEIGHT → 常亮
 *      - FAIL       → 熄灭
 * ================================================================ */
__NO_RETURN void led_task(void *args)
{
    (void)args;

    uint32_t wakeup_time = osKernelGetTickCount();
    uint8_t  count       = 0;

    for (;;) {
        count++;

        /* ========== 前灯: 遥控连接状态 ========== */
        if (remote_state == REMOTE_CONNECTED) {
            Int_led_turn_on(&left_top_led);
            Int_led_turn_on(&right_top_led);
        } else if (remote_state == REMOTE_DISCONNECTED) {
            Int_led_turn_off(&left_top_led);
            Int_led_turn_off(&right_top_led);
        }

        /* ========== 后灯: 飞行状态 ========== */
        if (flight_state == IDLE) {
            /* 慢闪: 5 个周期翻转一次 → 500ms 周期 */
            if (count % 5 == 0) {
                Int_led_toggle(&left_bottom_led);
                Int_led_toggle(&right_bottom_led);
            }
        } else if (flight_state == NORMAL) {
            /* 快闪: 2 个周期翻转一次 → 200ms 周期 */
            if (count % 2 == 0) {
                Int_led_toggle(&left_bottom_led);
                Int_led_toggle(&right_bottom_led);
            }
        } else if (flight_state == FIX_HEIGHT) {
            /* 常亮 */
            Int_led_turn_on(&left_bottom_led);
            Int_led_turn_on(&right_bottom_led);
        } else if (flight_state == FAIL) {
            /* 熄灭 */
            Int_led_turn_off(&left_bottom_led);
            Int_led_turn_off(&right_bottom_led);
        }

        /* count 循环: 10 个周期 = 1 秒 */
        if (count == 10) {
            count = 0;
        }

        /* 精确延时到下一个 100ms 基准点 */
        wakeup_time += LED_TASK_PERIOD;
        osDelayUntil(wakeup_time);
    }
}

/* ================================================================
 *  com_task() - 无线通讯任务
 *
 *  功能 (10ms 周期, 100Hz):
 *    1. 通过 SI24R1 接收遥控器数据
 *    2. 根据接收结果更新连接状态 (连接/断开重试)
 *    3. 检测关机指令 → 通过 Thread Flags 通知 power_task
 *    4. 处理飞行状态机 (IDLE → NORMAL → FIX_HEIGHT → FAIL → IDLE)
 *    5. 采集电池电压, 准备回传数据包
 *
 *  CMSIS OS v2 替代:
 *    vTaskDelay(ms)                  → osDelay(ms)
 *    xTaskNotifyGive(handle)         → osThreadFlagsSet(handle, flag)
 * ================================================================ */
__NO_RETURN void com_task(void *args)
{
    (void)args;

    /* 初始化电池 ADC */
    Int_bat_ADC_Init();

    for (;;) {
        /* 1. 接收遥控器数据 (含校验和帧头尾验证) */
        uint8_t res = App_receive_data();

        /* 2. 根据接收结果更新遥控连接状态 */
        App_process_connect_state(res);

        /* 3. 处理关机指令 → 通过 Thread Flag 通知 power_task */
        if (remote_data.shutdown == 1) {
            /*
             * 原 FreeRTOS: xTaskNotifyGive(power_task_handle)
             * CMSIS v2:   osThreadFlagsSet(thread_id, flags)
             */
            osThreadFlagsSet(power_task_handle, POWER_TASK_FLAG);
        }

        /* 4. 处理飞行状态机转换 */
        App_process_flight_state();

        /* 5. 准备回传电池电压数据 */
        float voltage = Int_bat_ADC_Read();
        sprintf((char *)back_buff, "%.2f", voltage);
        /* debug_printf("voltage:%.2f\r\n", voltage); */

        /*
         * 6. 10ms 相对延时
         *    (注意: 原代码使用 vTaskDelay 而非 vTaskDelayUntil,
         *     因此通讯周期 = 10ms + 任务执行时间)
         */
        osDelay(COM_TASK_PERIOD);
    }
}
