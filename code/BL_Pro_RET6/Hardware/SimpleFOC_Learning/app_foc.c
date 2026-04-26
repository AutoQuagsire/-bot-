#include "app_foc.h"
#include "FOC.h"
#include "Filter.h"
#include "current_sense.h"
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


static Motor_t          g_motor1;   // 电机控制对象
static Driver_t        *g_driver1 = NULL; // 三相驱动对象（由 Driver 模块提供实例）
static AS5047P_Handle_t g_enc1;     // AS5047P 底层驱动句柄
static Sensor_t         g_sensor1;  // 传感器公共层对象
static CurrentSense_t  g_current_sense1; // 电流采样对象
static PID_t            g_speed_pid1;
LowPassFilter_t         g_speed_lpf1;
static PID_t            g_current_pid1;
LowPassFilter_t         g_current_lpf1;


static Motor_t          g_motor2;   // 电机控制对象
static Driver_t        *g_driver2 = NULL; // 三相驱动对象（由 Driver 模块提供实例）
static AS5047P_Handle_t g_enc2;     // AS5047P 底层驱动句柄
static Sensor_t         g_sensor2;  // 传感器公共层对象
static CurrentSense_t  g_current_sense2; // 电流采样对象
static PID_t            g_speed_pid2;
LowPassFilter_t         g_speed_lpf2;
static PID_t            g_current_pid2;
LowPassFilter_t         g_current_lpf2;


static uint32_t         g_last_loop_tick_ms = 0U;
static uint32_t         g_last_print_tick_ms = 0U;



PID_t Left_Velocity_FOC_PID;
float Left_Target = 20.0f;




#define LEFT_MOTOR_ENABLE 0U
#define RIGHT_MOTOR_ENABLE 1U

#define APP_LOOP_TEST_UQ_V        (1.0f)
#define APP_LOOP_PRINT_PERIOD_MS  (100U)

#define APP_SPEED_LOOP_ENABLE      (0U)
#define APP_SPEED_TARGET_RAD_S     (20.0f)
#define APP_SPEED_KP               (0.06f)
#define APP_SPEED_KI               (0.00035f)
#define APP_SPEED_KD               (0.0f)
#define APP_SPEED_UQ_LIMIT         (8.0f)
#define APP_SPEED_I_LIMIT          (4.0f)
#define APP_SPEED_I_ERR_MIN        (0.05f)
#define APP_SPEED_I_SEP_RATIO      (0.75f)
#define APP_SPEED_VEL_FAULT_ABS    (80.0f)


#define APP_CURRENT_LOOP_ENABLE      (1U)
#define APP_CURRENT_TARGET_A       (1.0f)
#define APP_CURRENT_KP             (5.5)
#define APP_CURRENT_KI             (0.8f)
#define APP_CURRENT_KD             (0.0f)
#define APP_CURRENT_OUT_LIMIT       (8.0f)
#define APP_CURRENT_I_LIMIT          (5.5f)
#define APP_CURRENT_I_ERR_MIN        (0.05f)
#define APP_CURRENT_I_SEP_RATIO      (0.7f)

#define APP_CS_SIGN_TEST_UQ_V         (0.8f)
#define APP_CS_SIGN_TEST_SETTLE_MS    (80U)
#define APP_CS_SIGN_TEST_SAMPLE_CNT   (120U)
#define APP_CS_SIGN_TEST_SAMPLE_DT_MS (1U)
#define APP_CS_SIGN_TEST_DEADBAND_A   (0.03f)

#define APP_SENSOR_DIR_TEST_UQ_V         (4.0f)
#define APP_SENSOR_DIR_TEST_SETTLE_MS    (500U)
#define APP_SENSOR_DIR_TEST_ELEC_STEP    (PI * 0.5f)
#define APP_SENSOR_DIR_TEST_DEADBAND_RAD (0.02f)
#define APP_SENSOR_DIR_TEST_MAX_STEP_MS  (2500U)



#define APP_MOVE_DOWNSAMPLE        (1U)


#if (APP_MOVE_DOWNSAMPLE < 1U)
#error "APP_MOVE_DOWNSAMPLE must be >= 1"
#endif

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
    #if LEFT_MOTOR_ENABLE
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


    /*电流采样初始化相关*/
    CurrentSense_Init(&g_current_sense1);//必须放最前面，因为会给参数全置零
    CurrentSense_Config(&g_current_sense1, &hadc1, &htim3, TIM_CHANNEL_4);
    CurrentSenseParam_Init(&g_current_sense1
                        , FOC_SHUNT_RESISTOR_OHM
                        , FOC_AMP_GAIN
                        , 1
                        , 1);
    CurrentSense_CalibrateOffsets(&g_current_sense1);

    /*电流采样初始化相关*/
    linkSensor(&g_sensor1, &g_motor1);
    linkDriver(g_driver1, &g_motor1);
    linkCurrentSense(&g_current_sense1, &g_motor1);

    MotorParam_Init(&g_motor1, 14.0f, 10.3f, 0.0f, 0.0f, 0.0f);
    g_motor1.zero_electrical_angle = 0.0f;
    g_motor1.state.sensor_direction = sensor_direction_ccw;

    #endif

    #if RIGHT_MOTOR_ENABLE
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

    if (!AS5047P_RW_Init(&g_enc2, &hspi1, EcdR_CS_GPIO_Port, EcdR_CS_Pin)) {
        USB_Debug_Printf("AS5047P_RW_Init2 failed\r\n");
        return 0U;
    }



    Sensor_LinkAS5047P(&g_enc2, &g_sensor2);
    if (!Sensor_Init(&g_sensor2)) {
        USB_Debug_Printf("Sensor_Init2 failed\r\n");
        return 0U;
    }
    CurrentSense_Init(&g_current_sense2);//必须放最前面，因为会给参数全置零
    CurrentSense_Config(&g_current_sense2, &hadc2, &htim2, TIM_CHANNEL_2);
    CurrentSenseParam_Init(&g_current_sense2
                        , FOC_SHUNT_RESISTOR_OHM
                        , FOC_AMP_GAIN
                        , 1
                        , 1);
    CurrentSense_CalibrateOffsets(&g_current_sense2);

    linkSensor(&g_sensor2, &g_motor2);
    linkDriver(g_driver2, &g_motor2);
    linkCurrentSense(&g_current_sense2, &g_motor2);

    MotorParam_Init(&g_motor2, 14.0f, 10.3f, 0.0f, 0.0f, 0.0f);
    g_motor2.zero_electrical_angle = 0.0f;
    g_motor2.state.sensor_direction = sensor_direction_ccw;
    #endif


#if APP_SPEED_LOOP_ENABLE
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
#if APP_CURRENT_LOOP_ENABLE
    PID_ParameterInitEx(&g_current_pid1,
                        APP_CURRENT_KP,
                        APP_CURRENT_KI,
                        APP_CURRENT_KD,
                        APP_CURRENT_I_LIMIT,
                        APP_CURRENT_OUT_LIMIT,
                        APP_CURRENT_I_ERR_MIN,
                        APP_CURRENT_I_SEP_RATIO);
    PID_ParameterInitEx(&g_current_pid2,
                        APP_CURRENT_KP,
                        APP_CURRENT_KI,
                        APP_CURRENT_KD,
                        APP_CURRENT_I_LIMIT,
                        APP_CURRENT_OUT_LIMIT,
                        APP_CURRENT_I_ERR_MIN,
                        APP_CURRENT_I_SEP_RATIO);

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
    #if LEFT_MOTOR_ENABLE
    if (!Motor_CalibrateZeroElectricalAngle(&g_motor1, -5.0f, PI / 2.0f, 300)) {
        USB_Debug_Printf("Startup calibrate1 failed\r\n");
        return 0U;
    }
    /* 打印校准结果，便于调试确认 */
    USB_Debug_Printf("zero_elec1 = %.6f\r\n", g_motor1.zero_electrical_angle);
    #endif

    #if RIGHT_MOTOR_ENABLE
    if (!Motor_CalibrateZeroElectricalAngle(&g_motor2, -5.0f, PI / 2.0f, 300)) {
        USB_Debug_Printf("Startup calibrate2 failed\r\n");
        return 0U;
    }

    USB_Debug_Printf("zero_elec2 = %.6f\r\n", g_motor2.zero_electrical_angle);
    #endif
    
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
static int8_t app_sign_with_deadband(float v, float deadband)
{
    if (v > deadband) {
        return 1;
    }
    if (v < -deadband) {
        return -1;
    }
    return 0;
}

static PhaseCurrent_t app_measure_phase_current_avg(Motor_t *motor,
                                                    CurrentSense_t *cs,
                                                    float uq,
                                                    float elec_angle)
{
    uint32_t i;
    PhaseCurrent_t avg = {0.0f, 0.0f};

    if ((motor == NULL) || (cs == NULL)) {
        return avg;
    }

    Motor_SetPhaseVoltageQ(motor, uq, elec_angle);
    HAL_Delay(APP_CS_SIGN_TEST_SETTLE_MS);

    for (i = 0U; i < APP_CS_SIGN_TEST_SAMPLE_CNT; i++) {
        PhaseCurrent_t cur = CurrentSense_GetPhaseCurrent(cs);
        avg.ia += cur.ia;
        avg.ib += cur.ib;
        HAL_Delay(APP_CS_SIGN_TEST_SAMPLE_DT_MS);
    }

    avg.ia /= (float)APP_CS_SIGN_TEST_SAMPLE_CNT;
    avg.ib /= (float)APP_CS_SIGN_TEST_SAMPLE_CNT;
    return avg;
}

static void app_current_sense_sign_test_one(const char *tag,
                                            Motor_t *motor,
                                            CurrentSense_t *cs)
{
    uint8_t was_enabled;
    PhaseCurrent_t s_0;
    PhaseCurrent_t s_pi;
    PhaseCurrent_t s_pi_2;
    PhaseCurrent_t s_3pi_2;
    int8_t a_score = 0;
    int8_t b_score = 0;
    float uq = APP_CS_SIGN_TEST_UQ_V;

    if ((motor == NULL) || (cs == NULL) || (motor->driver == NULL) || (!motor->driver->initialized)) {
        USB_Debug_Printf("[CS-SIGN][%s] skip (motor/cs not ready)\r\n", tag);
        return;
    }

    was_enabled = (uint8_t)motor->state.enabled;
    if (!was_enabled) {
        FOCMotor_enable(motor);
        HAL_Delay(10);
    }

    if (!cs->enabled) {
        CurrentSense_Enable(cs);
        HAL_Delay(10);
    }

    if ((motor->driver->voltage_limit > 0.0f) && (uq > motor->driver->voltage_limit * 0.3f)) {
        uq = motor->driver->voltage_limit * 0.3f;
    }
    if (uq < 0.3f) {
        uq = 0.3f;
    }

    USB_Debug_Printf("[CS-SIGN][%s] test start A_SIGN=%d B_SIGN=%d uq=%.2fV\r\n",
                     tag, cs->CsParam.A_SIGN, cs->CsParam.B_SIGN, uq);

    s_0 = app_measure_phase_current_avg(motor, cs, uq, 0.0f);
    s_pi = app_measure_phase_current_avg(motor, cs, uq, PI);
    s_pi_2 = app_measure_phase_current_avg(motor, cs, uq, 0.5f * PI);
    s_3pi_2 = app_measure_phase_current_avg(motor, cs, uq, 1.5f * PI);

    Motor_SetPhaseVoltageQ(motor, 0.0f, 0.0f);

    a_score += (app_sign_with_deadband(s_3pi_2.ia, APP_CS_SIGN_TEST_DEADBAND_A) == 1) ? 1 : -1;
    a_score += (app_sign_with_deadband(s_pi_2.ia, APP_CS_SIGN_TEST_DEADBAND_A) == -1) ? 1 : -1;
    b_score += (app_sign_with_deadband(s_0.ib, APP_CS_SIGN_TEST_DEADBAND_A) == 1) ? 1 : -1;
    b_score += (app_sign_with_deadband(s_pi.ib, APP_CS_SIGN_TEST_DEADBAND_A) == -1) ? 1 : -1;

    USB_Debug_Printf("[CS-SIGN][%s] theta0    : ia=% .4f ib=% .4f (expect ib>0)\r\n", tag, s_0.ia, s_0.ib);
    USB_Debug_Printf("[CS-SIGN][%s] thetaPI   : ia=% .4f ib=% .4f (expect ib<0)\r\n", tag, s_pi.ia, s_pi.ib);
    USB_Debug_Printf("[CS-SIGN][%s] thetaPI/2 : ia=% .4f ib=% .4f (expect ia<0)\r\n", tag, s_pi_2.ia, s_pi_2.ib);
    USB_Debug_Printf("[CS-SIGN][%s] theta3PI/2: ia=% .4f ib=% .4f (expect ia>0)\r\n", tag, s_3pi_2.ia, s_3pi_2.ib);

    USB_Debug_Printf("[CS-SIGN][%s] recommendation: A_SIGN %s (%d -> %d), B_SIGN %s (%d -> %d)\r\n",
                     tag,
                     (a_score >= 0) ? "KEEP" : "FLIP",
                     cs->CsParam.A_SIGN,
                     (a_score >= 0) ? cs->CsParam.A_SIGN : -cs->CsParam.A_SIGN,
                     (b_score >= 0) ? "KEEP" : "FLIP",
                     cs->CsParam.B_SIGN,
                     (b_score >= 0) ? cs->CsParam.B_SIGN : -cs->CsParam.B_SIGN);

    if (!was_enabled) {
        FOCMotor_disable(motor);
    }
}

void App_CurrentSenseSignTest(void)
{
    USB_Debug_Printf("[CS-SIGN] test begin (run with motor unloaded and hold rotor still)\r\n");

#if LEFT_MOTOR_ENABLE
    app_current_sense_sign_test_one("L", &g_motor1, &g_current_sense1);
#endif

#if RIGHT_MOTOR_ENABLE
    app_current_sense_sign_test_one("R", &g_motor2, &g_current_sense2);
#endif

    USB_Debug_Printf("[CS-SIGN] test end\r\n");
}

static float app_angle_diff_signed(float from, float to)
{
    float d = to - from;
    while (d > PI) {
        d -= 2.0f * PI;
    }
    while (d < -PI) {
        d += 2.0f * PI;
    }
    return d;
}

static float app_sensor_angle_after_settle(Motor_t *motor,
                                           Sensor_t *sensor,
                                           float uq,
                                           float elec_angle,
                                           uint32_t settle_ms)
{
    uint32_t i;
    uint32_t step_ms;
    uint32_t t0;
    uint32_t t1;
    uint32_t ticks_per_ms;

    Motor_SetPhaseVoltageQ(motor, uq, elec_angle);

    ticks_per_ms = SystemCoreClock / 1000U;
    if (ticks_per_ms == 0U) {
        ticks_per_ms = 1U;
    }

    step_ms = settle_ms;
    if (step_ms > APP_SENSOR_DIR_TEST_MAX_STEP_MS) {
        step_ms = APP_SENSOR_DIR_TEST_MAX_STEP_MS;
    }

    for (i = 0U; i < step_ms; i++) {
        Sensor_Update(sensor, 0.001f);
        t0 = DWT_GetTicks();
        do {
            t1 = DWT_GetElapsedTicks(t0);
        } while (t1 < ticks_per_ms);
    }

    Sensor_Update(sensor, 0.001f);
    return Sensor_GetAngle(sensor);
}

static void app_sensor_direction_test_one(const char *tag, Motor_t *motor, Sensor_t *sensor)
{
    uint8_t was_enabled;
    float uq = APP_SENSOR_DIR_TEST_UQ_V;
    float a0;
    float ap;
    float an;
    float dp;
    float dn;
    int8_t score = 0;

    if ((motor == NULL) || (sensor == NULL) || (motor->driver == NULL) || (!motor->driver->initialized) || (!sensor->initialized)) {
        USB_Debug_Printf("[DIR-TEST][%s] skip (motor/sensor not ready)\r\n", tag);
        return;
    }

    was_enabled = (uint8_t)motor->state.enabled;
    if (!was_enabled) {
        FOCMotor_enable(motor);
        HAL_Delay(10);
    }

    if ((motor->driver->voltage_limit > 0.0f) && (uq > motor->driver->voltage_limit * 0.3f)) {
        uq = motor->driver->voltage_limit * 0.3f;
    }
    if (uq < 0.3f) {
        uq = 0.3f;
    }

    USB_Debug_Printf("[DIR-TEST][%s] start uq=%.2fV elec_step=%.3f\r\n", tag, uq, APP_SENSOR_DIR_TEST_ELEC_STEP);

    USB_Debug_Printf("[DIR-TEST][%s] step a0...\r\n", tag);
    a0 = app_sensor_angle_after_settle(motor, sensor, uq, 0.0f, APP_SENSOR_DIR_TEST_SETTLE_MS);
    USB_Debug_Printf("[DIR-TEST][%s] step +elec...\r\n", tag);
    ap = app_sensor_angle_after_settle(motor, sensor, uq, APP_SENSOR_DIR_TEST_ELEC_STEP, APP_SENSOR_DIR_TEST_SETTLE_MS);
    USB_Debug_Printf("[DIR-TEST][%s] step -elec...\r\n", tag);
    an = app_sensor_angle_after_settle(motor, sensor, uq, -APP_SENSOR_DIR_TEST_ELEC_STEP, APP_SENSOR_DIR_TEST_SETTLE_MS);
    USB_Debug_Printf("[DIR-TEST][%s] step done\r\n", tag);

    Motor_SetPhaseVoltageQ(motor, 0.0f, 0.0f);

    dp = app_angle_diff_signed(a0, ap);  /* +elec step */
    dn = app_angle_diff_signed(a0, an);  /* -elec step */

    if (dp > APP_SENSOR_DIR_TEST_DEADBAND_RAD) {
        score++;
    } else if (dp < -APP_SENSOR_DIR_TEST_DEADBAND_RAD) {
        score--;
    }

    if (dn < -APP_SENSOR_DIR_TEST_DEADBAND_RAD) {
        score++;
    } else if (dn > APP_SENSOR_DIR_TEST_DEADBAND_RAD) {
        score--;
    }

    USB_Debug_Printf("[DIR-TEST][%s] a0=%.4f ap=%.4f an=%.4f d_plus=%.4f d_minus=%.4f\r\n",
                     tag, a0, ap, an, dp, dn);

    if (score > 0) {
        USB_Debug_Printf("[DIR-TEST][%s] recommendation: sensor_direction_cw\r\n", tag);
    } else if (score < 0) {
        USB_Debug_Printf("[DIR-TEST][%s] recommendation: sensor_direction_ccw\r\n", tag);
    } else {
        USB_Debug_Printf("[DIR-TEST][%s] recommendation: inconclusive (increase uq/settle time and retest)\r\n", tag);
    }

    if (!was_enabled) {
        FOCMotor_disable(motor);
    }
}

void App_SensorDirectionTest(void)
{
    USB_Debug_Printf("[DIR-TEST] begin (motor should be free to move, not held)\r\n");

#if LEFT_MOTOR_ENABLE
    app_sensor_direction_test_one("L", &g_motor1, &g_sensor1);
#endif

#if RIGHT_MOTOR_ENABLE
    app_sensor_direction_test_one("R", &g_motor2, &g_sensor2);
#endif

    USB_Debug_Printf("[DIR-TEST] end\r\n");
}

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
float uq_cmd1 = APP_LOOP_TEST_UQ_V;

float vel2 = 0;
float vel_windowed2 = 0;
float vel_windowed_f2 = 0;
float uq_cmd2 = APP_LOOP_TEST_UQ_V;
static uint16_t g_move_downsample_cnt = 0U;

static uint8_t loopFOC(void)
{
    if (!Motor_UpdateSensor(&g_motor1, FOC_PERIOD_S)) {
        return 0U;
    }

    if (!Motor_UpdateSensor(&g_motor2, FOC_PERIOD_S)) {
        return 0U;
    }

    {
        float sin_e1 = 0.0f;
        float cos_e1 = 0.0f;
        float sin_e2 = 0.0f;
        float cos_e2 = 0.0f;

        Get_SinCos(g_motor1.electrical_angle, &sin_e1, &cos_e1);
        Get_SinCos(g_motor2.electrical_angle, &sin_e2, &cos_e2);

        Motor_SetPhaseVoltageQBySinCos(&g_motor1, uq_cmd1, sin_e1, cos_e1);
        Motor_SetPhaseVoltageQBySinCos(&g_motor2, uq_cmd2, sin_e2, cos_e2);
    }

    return 1U;
}

static void move(void)
{
    float vel_target1 = Left_Target;
    float vel_target2 = Left_Target;

    vel1 = Sensor_GetVelocityRaw(&g_sensor1);
    vel_windowed1 = Sensor_GetVelocityWindowed(&g_sensor1);
    vel_windowed_f1 = LowPassFilter_Update(&g_speed_lpf1, vel1);

    vel2 = Sensor_GetVelocityRaw(&g_sensor2);
    vel_windowed2 = Sensor_GetVelocityWindowed(&g_sensor2);
    vel_windowed_f2 = LowPassFilter_Update(&g_speed_lpf2, vel2);

#if APP_SPEED_LOOP_ENABLE
    PID_Calculate(&g_speed_pid1, vel_target1, vel_windowed_f1, 0U);
    uq_cmd1 = g_speed_pid1.output;

    PID_Calculate(&g_speed_pid2, vel_target2, vel_windowed_f2, 0U);
    uq_cmd2 = g_speed_pid2.output;
#else
    uq_cmd1 = APP_LOOP_TEST_UQ_V;
    uq_cmd2 = APP_LOOP_TEST_UQ_V;
#endif

    Left_Velocity_FOC_PID = g_speed_pid1;
    pid_csv_data.timestamp_ms = HAL_GetTick();
    pid_csv_data.setpoint = vel_target1;
    pid_csv_data.input = vel_windowed_f1;
    pid_csv_data.pwm = uq_cmd1;
    pid_csv_data.error = vel_target1 - vel_windowed_f1;
    pid_csv_data.p_term = g_speed_pid1.Kp * pid_csv_data.error;
    pid_csv_data.i_term = g_speed_pid1.Ki * g_speed_pid1.error_integral;
    pid_csv_data.d_term = g_speed_pid1.Kd * (pid_csv_data.error - g_speed_pid1.last_error);
}
//10Khz中断内部程序
void App_LoopForIT(void)
{
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

    if (!loopFOC()) {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        return;
    }

    g_move_downsample_cnt++;
    if (g_move_downsample_cnt >= APP_MOVE_DOWNSAMPLE) {
        g_move_downsample_cnt = 0U;
        move();
    }

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
