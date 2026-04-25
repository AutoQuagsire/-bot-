#include "app_foc.h"
#include "FOC.h"
#include "Filter.h"
#include "main.h"
#include "AS5047P_RW.h"
#include "sensor.h"
#include "driver.h"
#include "BLDCMotor.h"
#include "stm32g4xx_hal.h"
#include "sys.h"
#include <math.h>
#include <stdint.h>
#include "PID.h"

/* 这些外设句柄由 CubeMX 生成并在别处定义
 * 这里用 extern 引用，供应用层初始化时使用 */
extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim1;

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim4;
/* =========================
 * 应用层 FOC 对象
 * =========================
 * 这一层的对象不属于某个单独模块，而是“把各模块组起来”
 * 所以统一放在 app_foc.c 内部静态保存
 */
static AS5047P_Handle_t g_enc1;     // AS5047P 底层驱动句柄
static AS5047P_Handle_t g_enc2;     // AS5047P 底层驱动句柄

static Sensor_t         g_sensor1;  // 传感器公共层对象
static Sensor_t         g_sensor2;  // 传感器公共层对象


static Motor_t          g_motor1;   // 电机控制对象
static PID_t            g_speed_pid1;
static Driver_t        *g_driver1 = NULL; // 三相驱动对象（由 Driver 模块提供实例）
LowPassFilter_t         g_speed_lpf1;


static Motor_t          g_motor2;   // 电机控制对象
static Driver_t        *g_driver2 = NULL; // 三相驱动对象（由 Driver 模块提供实例）
static PID_t            g_speed_pid2;
LowPassFilter_t         g_speed_lpf2;


static uint32_t         g_last_loop_tick_ms = 0U;
static uint32_t         g_last_print_tick_ms = 0U;



PID_t Left_Velocity_FOC_PID;
float Left_Target = 20.0f;

static uint8_t          g_speed_fault1 = 0U;
static uint8_t          g_speed_fault2 = 0U;

#define APP_LOOP_TEST_UQ_V        (1.0f)
#define APP_LOOP_PRINT_PERIOD_MS  (100U)

#define APP_SPEED_LOOP_ENABLE      (1U)
#define APP_SPEED_TARGET_RAD_S     (20.0f)
#define APP_SPEED_KP               (0.06f)
#define APP_SPEED_KI               (0.00035f)
#define APP_SPEED_KD               (0.0f)
#define APP_SPEED_UQ_LIMIT         (8.0f)
#define APP_SPEED_I_LIMIT          (4.0f)
#define APP_SPEED_I_ERR_MIN        (0.05f)
#define APP_SPEED_I_SEP_RATIO      (0.75f)
#define APP_SPEED_VEL_FAULT_ABS    (80.0f)

#define APP_MATRIX_ENABLE          (0U)
#define APP_MATRIX_LEVEL_COUNT     (3U)
#define APP_MATRIX_RUNS_PER_LEVEL  (3U)
#define APP_MATRIX_SETTLE_MS       (1000U)
#define APP_MATRIX_MEASURE_MS      (4000U)

typedef struct {
    float avg_abs_vel;
    float max_abs_vel;
    float avg_vel;
    uint32_t sample_count;
} AppMatrixResult_t;

static const float g_matrix_uq_levels[APP_MATRIX_LEVEL_COUNT] = {0.5f, 1.0f, 1.5f};
static AppMatrixResult_t g_matrix_results[APP_MATRIX_LEVEL_COUNT][APP_MATRIX_RUNS_PER_LEVEL];
static uint8_t g_matrix_started = 0U;
static uint8_t g_matrix_finished = 0U;
static uint8_t g_matrix_level_idx = 0U;
static uint8_t g_matrix_run_idx = 0U;
static uint8_t g_matrix_phase = 0U; /* 0:settle, 1:measure */
static uint32_t g_matrix_phase_start_ms = 0U;
static float g_matrix_sum_abs_vel = 0.0f;
static float g_matrix_max_abs_vel = 0.0f;
static float g_matrix_sum_vel = 0.0f;
static uint32_t g_matrix_samples = 0U;

static void App_MatrixResetStats(void)
{
    g_matrix_sum_abs_vel = 0.0f;
    g_matrix_max_abs_vel = 0.0f;
    g_matrix_sum_vel = 0.0f;
    g_matrix_samples = 0U;
}

static void App_MatrixPrintFinalSummary(void)
{
    uint8_t lv = 0U;

    USB_Debug_Printf("[MATRIX] all runs finished\r\n");
    for (lv = 0U; lv < APP_MATRIX_LEVEL_COUNT; lv++) {
        float level_sum_avg_abs = 0.0f;
        float level_max_abs = 0.0f;
        float level_sum_avg_vel = 0.0f;
        uint8_t rn = 0U;

        for (rn = 0U; rn < APP_MATRIX_RUNS_PER_LEVEL; rn++) {
            AppMatrixResult_t *r = &g_matrix_results[lv][rn];
            level_sum_avg_abs += r->avg_abs_vel;
            level_sum_avg_vel += r->avg_vel;
            if (r->max_abs_vel > level_max_abs) {
                level_max_abs = r->max_abs_vel;
            }
        }

        USB_Debug_Printf("[MATRIX][L%u uq=%.2f] mean|vel|=%.3f peak|vel|=%.3f meanVel=%.3f\r\n",
                         (unsigned)(lv + 1U),
                         g_matrix_uq_levels[lv],
                         level_sum_avg_abs / (float)APP_MATRIX_RUNS_PER_LEVEL,
                         level_max_abs,
                         level_sum_avg_vel / (float)APP_MATRIX_RUNS_PER_LEVEL);
    }
}

static __attribute__((unused)) float App_MatrixStep(uint32_t now_ms, float vel)
{
    float uq = APP_LOOP_TEST_UQ_V;

    if (g_matrix_finished) {
        return 0.0f;
    }

    if (!g_matrix_started) {
        g_matrix_started = 1U;
        g_matrix_phase = 0U;
        g_matrix_phase_start_ms = now_ms;
        App_MatrixResetStats();
        USB_Debug_Printf("[MATRIX] start levels=%u runs=%u settle=%ums measure=%ums\r\n",
                         (unsigned)APP_MATRIX_LEVEL_COUNT,
                         (unsigned)APP_MATRIX_RUNS_PER_LEVEL,
                         (unsigned)APP_MATRIX_SETTLE_MS,
                         (unsigned)APP_MATRIX_MEASURE_MS);
    }

    uq = g_matrix_uq_levels[g_matrix_level_idx];

    if (g_matrix_phase == 0U) {
        if ((now_ms - g_matrix_phase_start_ms) >= APP_MATRIX_SETTLE_MS) {
            g_matrix_phase = 1U;
            g_matrix_phase_start_ms = now_ms;
            App_MatrixResetStats();
            USB_Debug_Printf("[MATRIX] measure begin level=%u run=%u uq=%.2f\r\n",
                             (unsigned)(g_matrix_level_idx + 1U),
                             (unsigned)(g_matrix_run_idx + 1U),
                             uq);
        }
        return uq;
    }

    if (g_matrix_phase == 1U) {
        float abs_vel = fabsf(vel);
        g_matrix_sum_abs_vel += abs_vel;
        g_matrix_sum_vel += vel;
        if (abs_vel > g_matrix_max_abs_vel) {
            g_matrix_max_abs_vel = abs_vel;
        }
        g_matrix_samples++;

        if ((now_ms - g_matrix_phase_start_ms) >= APP_MATRIX_MEASURE_MS) {
            AppMatrixResult_t *r = &g_matrix_results[g_matrix_level_idx][g_matrix_run_idx];
            const char *dir = ".";

            r->sample_count = g_matrix_samples;
            if (g_matrix_samples > 0U) {
                r->avg_abs_vel = g_matrix_sum_abs_vel / (float)g_matrix_samples;
                r->avg_vel = g_matrix_sum_vel / (float)g_matrix_samples;
                r->max_abs_vel = g_matrix_max_abs_vel;
            } else {
                r->avg_abs_vel = 0.0f;
                r->avg_vel = 0.0f;
                r->max_abs_vel = 0.0f;
            }

            if (r->avg_vel > 0.05f) {
                dir = "+";
            } else if (r->avg_vel < -0.05f) {
                dir = "-";
            }

            USB_Debug_Printf("[MATRIX][L%u R%u uq=%.2f] avg|vel|=%.3f max|vel|=%.3f avgVel=%.3f dir=%s n=%lu\r\n",
                             (unsigned)(g_matrix_level_idx + 1U),
                             (unsigned)(g_matrix_run_idx + 1U),
                             uq,
                             r->avg_abs_vel,
                             r->max_abs_vel,
                             r->avg_vel,
                             dir,
                             (unsigned long)r->sample_count);

            g_matrix_run_idx++;
            if (g_matrix_run_idx >= APP_MATRIX_RUNS_PER_LEVEL) {
                g_matrix_run_idx = 0U;
                g_matrix_level_idx++;
            }

            if (g_matrix_level_idx >= APP_MATRIX_LEVEL_COUNT) {
                g_matrix_finished = 1U;
                App_MatrixPrintFinalSummary();
                return 0.0f;
            }

            g_matrix_phase = 0U;
            g_matrix_phase_start_ms = now_ms;
            App_MatrixResetStats();
        }
    }

    return uq;
}

/**
 * @brief  FOC 应用层对象初始化
 * @retval 1: 初始化成功
 *         0: 初始化失败
 *
 * 初始化顺序说明：
 * 1. 先拿到 driver 实例，并初始化 PWM 输出
 * 2. 再初始化编码器底层驱动
 * 3. 然后把编码器挂到 Sensor 层，并初始化 Sensor
 * 4. 最后配置 Motor 参数，并把 Sensor / Driver 链接给 Motor
 *
 * 这个函数只负责“对象装配”，不负责校准动作
 */
uint8_t App_FOCStack_Init(void)
{
    /* 1) 获取左电机对应的 driver 实例 */
     g_driver1 = Driver_GetInstance(DRIVER_LEFT);

     if ( g_driver1 == NULL) {
        USB_Debug_Printf("Driver_GetInstance1 failed\r\n");
        return 0U;
    }    
    if (!Driver_Init(g_driver1,
                     &htim1,
                     TIM_CHANNEL_1,
                     TIM_CHANNEL_3,
                     TIM_CHANNEL_4,
                     19 * 0.577f)) {
        USB_Debug_Printf("Driver1_Init failed\r\n");
        return 0U;
    }

    if (!AS5047P_RW_Init(&g_enc1, &hspi3, EcdL_CS_GPIO_Port, EcdL_CS_Pin)) {
        USB_Debug_Printf("AS5047P_RW_Init1 failed\r\n");
        return 0U;
    }
    Sensor_LinkAS5047P(&g_enc1, &g_sensor1);    
    if (!Sensor_Init(&g_sensor1)) {
        USB_Debug_Printf("Sensor_Init1 failed\r\n");
        return 0U;
    }
    linkSensor(&g_sensor1, &g_motor1);
    linkDriver(g_driver1, &g_motor1);

    g_driver2 = Driver_GetInstance(DRIVER_RIGHT);    
    if ( g_driver2 == NULL) {
        USB_Debug_Printf("Driver_GetInstance2 failed\r\n");
        return 0U;
    }

    /* 2) 初始化三相驱动输出
     * 参数含义：
     * - htim1：用于输出 PWM 的定时器
     * - TIM_CHANNEL_1/3/4：三相对应的通道
     * - 19 * 0.577f：最大相电压限制（你当前的写法）
     */



    if (!Driver_Init(g_driver2,
                     &htim4,
                     TIM_CHANNEL_4,
                     TIM_CHANNEL_3,
                     TIM_CHANNEL_2,
                     19 * 0.577f)) {
        USB_Debug_Printf("Driver2_Init failed\r\n");
        return 0U;
    }

    /* 3) 初始化 AS5047P 底层驱动
     * 这里只做 SPI 与芯片基本通信确认 */
    if (!AS5047P_RW_Init(&g_enc2, &hspi1, EcdR_CS_GPIO_Port, EcdR_CS_Pin)) {
        USB_Debug_Printf("AS5047P_RW_Init2 failed\r\n");
        return 0U;
    }


    /* 4) 将 AS5047P 设备句柄挂到 Sensor 公共层 */

    /* 5) 初始化 Sensor 层
     * Sensor_Init 只做链路确认和初始角记录
     * 不在这里做真正的速度估计 */


    Sensor_LinkAS5047P(&g_enc2, &g_sensor2);
    if (!Sensor_Init(&g_sensor2)) {
        USB_Debug_Printf("Sensor_Init2 failed\r\n");
        return 0U;
    }

    /* 6) 初始化电机参数
     * 当前参数：
     * - 14.0f: 极对数
     * - 10.3f: 电源电压/母线电压（按你当前设定）
     * - 后面几个暂时为 0，后续再逐步补充
     */
    MotorParam_Init(&g_motor1, 14.0f, 10.3f, 0.0f, 0.0f, 0.0f);
    MotorParam_Init(&g_motor2, 14.0f, 10.3f, 0.0f, 0.0f, 0.0f);


    /* 7) 初始化电机当前状态
     * - 零位电角度先置 0，后面通过校准函数写入真实值
     * - 传感器方向先人工指定，后面可再做自动判定或验证
     */
    g_motor1.zero_electrical_angle = 0.0f;
    g_motor1.state.sensor_direction = sensor_direction_ccw;

    g_motor2.zero_electrical_angle = 0.0f;
    g_motor2.state.sensor_direction = sensor_direction_ccw;

    /* 8) 把 Sensor 和 Driver 链接给 Motor
     * 从这一步开始，Motor 才真正拥有“读角度”和“打PWM”的能力 */


    linkSensor(&g_sensor2, &g_motor2);
    linkDriver(g_driver2, &g_motor2);

#if APP_SPEED_LOOP_ENABLE
    g_speed_pid1.error_integral = 0.0f;
    g_speed_pid1.last_error = 0.0f;
    g_speed_pid1.output = 0.0f;
    g_speed_pid1.output_limit = APP_SPEED_UQ_LIMIT;
    g_speed_pid1.I_ERR_MIN = APP_SPEED_I_ERR_MIN;
    g_speed_pid1.I_SEP_RATIO = APP_SPEED_I_SEP_RATIO;
    PID_ParameterInitEx(&g_speed_pid1,
                        APP_SPEED_KP,
                        APP_SPEED_KI,
                        APP_SPEED_KD,
                        APP_SPEED_I_LIMIT,
                        APP_SPEED_UQ_LIMIT,
                        APP_SPEED_I_ERR_MIN,
                        APP_SPEED_I_SEP_RATIO);
    PID_ParameterInitEx(&g_speed_pid2,
                        APP_SPEED_KP,
                        APP_SPEED_KI,
                        APP_SPEED_KD,
                        APP_SPEED_I_LIMIT,
                        APP_SPEED_UQ_LIMIT,
                        APP_SPEED_I_ERR_MIN,
                        APP_SPEED_I_SEP_RATIO);


    Left_Velocity_FOC_PID = g_speed_pid1;
    Left_Target = APP_SPEED_TARGET_RAD_S;
    g_speed_fault1 = 0U;
    g_speed_fault2 = 0U;


    LowPassFilter_Init(&g_speed_lpf1, 100.0f, FOC_FREQUENCY);   
    LowPassFilter_Init(&g_speed_lpf2, 100.0f, FOC_FREQUENCY);   
#endif

    USB_Debug_Printf("FOC stack init ok\r\n");
    return 1U;
}

/**
 * @brief  上电后的启动校准流程
 * @retval 1: 校准成功
 *         0: 校准失败
 *
 * 当前这里只做“零位电角度校准”：
 * - 给一个固定电压矢量
 * - 让转子吸到指定电角位置
 * - 读取此时机械角
 * - 反算 zero_electrical_angle
 */
uint8_t App_StartupCalibrate(void)
{
    /* 参数说明：
     * - &g_motor1            : 要校准的电机对象
     * - 2.0f                 : 对齐电压
     * - 3.0f * PI / 2.0f     : 对齐目标电角度（3π/2）
     * - 300                  : 对齐稳定等待时间，单位 ms
     */
    if (!Motor_CalibrateZeroElectricalAngle(&g_motor1, -5.0f, PI / 2.0f, 300)) {
        USB_Debug_Printf("Startup calibrate1 failed\r\n");
        return 0U;
    }
    /* 打印校准结果，便于调试确认 */
    USB_Debug_Printf("zero_elec = %.6f\r\n", g_motor1.zero_electrical_angle);


    if (!Motor_CalibrateZeroElectricalAngle(&g_motor2, -5.0f, PI / 2.0f, 300)) {
        USB_Debug_Printf("Startup calibrate2 failed\r\n");
        return 0U;
    }

    USB_Debug_Printf("zero_elec = %.6f\r\n", g_motor2.zero_electrical_angle);
    return 1U;
}

/**
 * @brief 主循环中的应用层逻辑
 *
 * 当前先留空，后面可以逐步加入：
 * 1. Sensor_Update()
 * 2. 电角度更新
 * 3. 调试打印
 * 4. 开环测试
 * 5. 正式 FOC 闭环
 */
void App_Loop(void)
{
    uint32_t now_ms = HAL_GetTick();
    if (g_last_loop_tick_ms == 0U) {
        g_last_loop_tick_ms = now_ms;
        return;
    }

    uint32_t dt_ms = now_ms - g_last_loop_tick_ms;
    if (dt_ms == 0U) {
        return;
    }
    g_last_loop_tick_ms = now_ms;

    float dt = (float)dt_ms * 0.001f;
    if (!Motor_UpdateSensor(&g_motor1, dt)) {
        return;
    }

    float elec_angle = g_motor1.electrical_angle;
    float vel = Sensor_GetVelocityRaw(&g_sensor1);
    float uq_cmd = APP_LOOP_TEST_UQ_V;
    float vel_target = Left_Target;
    float vel_error = vel_target - vel;

#if APP_MATRIX_ENABLE
    uq_cmd = App_MatrixStep(now_ms, vel);
#endif

#if APP_SPEED_LOOP_ENABLE
    if (fabsf(vel) > APP_SPEED_VEL_FAULT_ABS) {
        g_speed_fault1 = 1U;
    }

    if (!g_speed_fault1) {
        PID_Calculate(&g_speed_pid1, vel_target, vel, 0U);
        uq_cmd = g_speed_pid1.output;
    } else {
        uq_cmd = 0.0f;
    }
#endif

    float sin_e1 = 0.0f;
    float cos_e1 = 0.0f;
    Get_SinCos(elec_angle, &sin_e1, &cos_e1);
    Motor_SetPhaseVoltageQBySinCos(&g_motor1, uq_cmd, sin_e1, cos_e1);

    if ((now_ms - g_last_print_tick_ms) >= APP_LOOP_PRINT_PERIOD_MS) {
        g_last_print_tick_ms = now_ms;
#if APP_SPEED_LOOP_ENABLE
        USB_Debug_Printf("tgt=%.2f vel=%.3f err=%.3f uq=%.2f mech=%.4f elec=%.4f fault=%u\r\n",
                         vel_target,
                         vel,
                         vel_error,
                         uq_cmd,
                         Sensor_GetAngle(&g_sensor1),
                         g_motor1.electrical_angle,
                         (unsigned)g_speed_fault1);
#else
        USB_Debug_Printf("mech=%.4f elec=%.4f vel=%.3f uq=%.2f\r\n",
                         Sensor_GetAngle(&g_sensor1),
                         g_motor1.electrical_angle,
                         vel,
                         uq_cmd);
#endif
    }
}




void App_FOCControlIT_Enable(void)
{
  HAL_TIM_Base_Start_IT(&htim5);
}


float vel1 = 0;
float vel_windowed1 = 0;
float vel_windowed_f1 = 0;
float uq_cmd1 = 0.0f;

float vel2 = 0;
float vel_windowed2 = 0;
float vel_windowed_f2 = 0;
float uq_cmd2 = 0.0f;
//10Khz中断内部程序
void App_LoopForIT(void)
{
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

    if (!Motor_UpdateSensor(&g_motor1, 0.0001f )) {
        return;
    }

    float elec_angle1 = g_motor1.electrical_angle ;
    uq_cmd1 = APP_LOOP_TEST_UQ_V;
    float vel_target1 = Left_Target;
    vel1 = Sensor_GetVelocityRaw(&g_sensor1);
    vel_windowed1 = Sensor_GetVelocityWindowed(&g_sensor1);
    vel_windowed_f1 = LowPassFilter_Update(&g_speed_lpf1, vel1);


    if (!Motor_UpdateSensor(&g_motor2, 0.0001f )) {
        return;
    }
    float elec_angle2 = g_motor2.electrical_angle ;
    uq_cmd2 = APP_LOOP_TEST_UQ_V;
    float vel_target2 = Left_Target;
    vel2 = Sensor_GetVelocityRaw(&g_sensor2);
    vel_windowed2 = Sensor_GetVelocityWindowed(&g_sensor2);
    vel_windowed_f2 = LowPassFilter_Update(&g_speed_lpf2, vel2);


    #if APP_SPEED_LOOP_ENABLE



            PID_Calculate(&g_speed_pid1, vel_target1, vel_windowed_f1, 0U);
            uq_cmd1 = g_speed_pid1.output;

            PID_Calculate(&g_speed_pid2, vel_target2, vel_windowed_f2, 0U);
            uq_cmd2 = g_speed_pid2.output;




    #endif


    float sin_e1 = 0.0f;
    float cos_e1 = 0.0f;
    Get_SinCos(elec_angle1, &sin_e1, &cos_e1);

    float sin_e2 = 0.0f;
    float cos_e2 = 0.0f;
    Get_SinCos(elec_angle2, &sin_e2, &cos_e2);

    Motor_SetPhaseVoltageQBySinCos(&g_motor1, uq_cmd1, sin_e1, cos_e1);
    Motor_SetPhaseVoltageQBySinCos(&g_motor2, uq_cmd2, sin_e2, cos_e2);

    Left_Velocity_FOC_PID = g_speed_pid1;
    pid_csv_data.timestamp_ms = HAL_GetTick();
    pid_csv_data.setpoint = vel_target1;
    pid_csv_data.input = vel_windowed_f1;
    pid_csv_data.pwm = uq_cmd1;
    pid_csv_data.error = vel_target1 - vel_windowed_f1;
    pid_csv_data.p_term = g_speed_pid1.Kp * pid_csv_data.error;
    pid_csv_data.i_term = g_speed_pid1.Ki * g_speed_pid1.error_integral;
    pid_csv_data.d_term = g_speed_pid1.Kd * (pid_csv_data.error - g_speed_pid1.last_error);
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

}

void DebuginWhile(void)
{
    static uint32_t last_print_ms = 0U;
    uint32_t now_ms = HAL_GetTick();

    if ((now_ms - last_print_ms) < 50U) {
        return;
    }

    last_print_ms = now_ms;
    USB_Debug_Printf("vel,windowed,filtered: %.2f, %.2f, %.2f, %.2f\r\n", uq_cmd1, vel_windowed_f1,uq_cmd2,vel_windowed_f2);

}
