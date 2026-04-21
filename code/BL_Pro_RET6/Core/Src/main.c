/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : PWM 裸测 —— 仅 TIM1 + TIM4，屏蔽所有其他功能
  ******************************************************************************
  * Copyright (c) 2026 STMicroelectronics. All rights reserved.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FOC.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_debug.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sys.h"   /* 保留：INT.c 中有 extern AS5047P_Handle encoder_right */
#include "pid_autotune.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* INT.c 中 extern 了这个变量，必须保留定义，否则链接报错 */
/* 左路编码器句柄保留（停用） */
AS5047P_Handle encoder_left;
/* 右路编码器句柄（当前启用） */
AS5047P_Handle encoder_right;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

   HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_SPI3_Init();
  MX_USB_Device_Init();
  MX_TIM8_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  /* 初始化时先关闭电机驱动，防止初始化过程中意外旋转 */
  HAL_GPIO_WritePin(Motor_EN_GPIO_Port, Motor_EN_Pin, GPIO_PIN_SET);


  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  /* Ensure TIM1 main output enabled (BDTR.MOE) for advanced timer outputs */


  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);


   /* 初始化滤波器（INT.c 全局定义） */
  extern LowPassFilter_t left_velocity_filter;
  extern LowPassFilter_t left_current_filter;
  extern LowPassFilter_t test_filter;
  LowPassFilter_Init(&left_velocity_filter, 100.0f, FOC_FREQUENCY*0.1f);   
  LowPassFilter_Init(&test_filter, 30.0f, FOC_FREQUENCY*0.1f);   
 
  
  LowPassFilter_Init(&left_current_filter, 800.0f, FOC_FREQUENCY);
  extern LowPassFilter_t right_velocity_filter;
  extern LowPassFilter_t right_current_filter;
  /* 右路电流环启用：速度滤波器用于角速度观测 */
  LowPassFilter_Init(&right_velocity_filter, 100.0f, FOC_FREQUENCY*0.1f);
  LowPassFilter_Init(&right_current_filter, 600.0f, FOC_FREQUENCY);



  /* 初始化 PID 控制器（INT.c 全局定义） */

  extern PID_t Left_Velocity_FOC_PID;
  extern PID_t Left_Current_FOC_PID;
  PID_ParameterInitEx(&Left_Velocity_FOC_PID, 0.00f, 0.00f, 0.00f, 0.2f,
                      1.5f, 0.05f, 0.75f);
  PID_ParameterInitEx(&Left_Current_FOC_PID, 5.5f, 0.8f, 0.0f, 5.5f,
                      Uq_max, 0.05f, 0.6f);
  extern PID_t Right_Velocity_FOC_PID;
  extern PID_t Right_Current_FOC_PID;
  PID_ParameterInitEx(&Right_Velocity_FOC_PID, 0.05f, 0.005f, 0.0005f, 3.0f,
                      Uq_max, 0.5f, 0.5f);
  PID_ParameterInitEx(&Right_Current_FOC_PID, 7.2, 0.61f, 0.0f, 10.0f,
                      Uq_max, 0.05f, 0.55f);	 /* 增加 Kp 和 Ki 以提高电流追踪能力 */
  
  /* 电流采样验证模式：执行右路电流采样校准（不输出SVPWM） */
  HAL_Delay(2000);
  CurrentSense_Init();

  /* 初始化磁编码器（AS5047P） */
  /* 使能 DWT 周期计数器（spi_transfer 超时依赖，不受 SysTick 优先级影响） */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT       = 0U;
  DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;

  AS5047P_Init(&encoder_left, &hspi3, EcdL_CS_GPIO_Port, EcdL_CS_Pin);
  AS5047P_Init(&encoder_right, &hspi1, EcdR_CS_GPIO_Port, EcdR_CS_Pin);
  HAL_Delay(10);




  HAL_GPIO_WritePin(Motor_EN_GPIO_Port, Motor_EN_Pin, GPIO_PIN_RESET);

float stInit = 0.0f;
float ctInit = 0.0f;
float theta_align =  PI*1.5f;  /* 预设的电角度零位，需根据实际情况调整（例如机械安装误差） */
float theta_field = 0.0f;   // 因为 FOC_SetSVPWM() 实际注入角 = theta_align + PI/2
float angle_sum_sin = 0.0f;
float angle_sum_cos = 0.0f;
float angle_avg = 0.0f;
float raw_elec_mean = 0.0f;
float elec_check = 0.0f;
float elec_err = 0.0f;
uint16_t ok_count = 0U;
uint16_t try_count = 0U;
extern float zero_elec_angle;

Get_SinCos(theta_align, &stInit, &ctInit);

/* 1. 给固定空间矢量，让转子吸住 */
FOC_SetSVPWM(&L_Motor, 4.0f, stInit, ctInit, 0.0f);
HAL_Delay(1000);

/* 2. 采样求圆均值 */
for (try_count = 0U; try_count < 96U && ok_count < 32U; try_count++)
{
    if (AS5047P_GetAngleWithoutTrack(&encoder_left) == AS5047P_OK)
    {
        angle_sum_sin += sinf(encoder_left.angle_rad);
        angle_sum_cos += cosf(encoder_left.angle_rad);
        ok_count++;
    }
    HAL_Delay(1);
}

if (ok_count > 0U)
{
    angle_avg = atan2f(angle_sum_sin / (float)ok_count,
                       angle_sum_cos / (float)ok_count);
    if (angle_avg < 0.0f) angle_avg += 2.0f * PI;

    raw_elec_mean = normalizeAngle((float)(DIR * PolePair) * angle_avg);

    /* 对于 theta_align = 3PI/2，这里实际应直接等于 raw_elec_mean */
    zero_elec_angle = raw_elec_mean;

    /* 3. 保持锁定时做校验 */
    if (AS5047P_GetAngleWithoutTrack(&encoder_left) == AS5047P_OK)
    {
        elec_check = normalizeAngle((float)(DIR * PolePair) * encoder_left.angle_rad - zero_elec_angle);
        elec_err = elec_check - theta_field;
        if (elec_err > PI) elec_err -= 2.0f * PI;
        else if (elec_err < -PI) elec_err += 2.0f * PI;
    }

    USB_Debug_Printf("zero=%.6f, mech=%.6f, raw_elec=%.6f, err=%.4f\r\n",
                     zero_elec_angle, angle_avg, raw_elec_mean, elec_err);
}
else
{
    zero_elec_angle = 0.0f;
    USB_Debug_Printf("零位标定失败\r\n");
}

/* 4. 最后再关输出 */
FOC_SetSVPWM(&L_Motor, 0.0f, stInit, ctInit, 0.0f);
HAL_Delay(200);



  /* 启动 FOC 定时器中断 */
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
  /* 右路电流环测试：启用驱动 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* ========== 过流保护检查 ========== */
    extern void OvercurrentProtection_Check(void);
    extern uint8_t OvercurrentProtection_GetStatus(void);
    OvercurrentProtection_Check();
    
    /* 处理 USB 接收的命令（非阻塞） */
    extern void Process_USB_Command(void);
    Process_USB_Command();

    /* 自动整定状态机（非阻塞） */
   PID_AutoTune_Update();
    

    /* 调试数据采集与输出 - CSV格式用于AI调参 */
    static uint32_t last_print = 0;
    static uint8_t exp1_header_printed = 0U;
#if PID_FAST_LOG_ENABLE
    static uint8_t fast_log_dump_active = 0U;
    static uint16_t fast_log_dump_idx = 0U;
#endif
    uint32_t now = HAL_GetTick();

#if PID_FAST_LOG_ENABLE
    if (pid_fast_log_full && !fast_log_dump_active)
    {
      fast_log_dump_active = 1U;
      fast_log_dump_idx = 0U;
      USB_Debug_Printf("# FASTLOG BEGIN,count=%u\r\n", (unsigned int)pid_fast_log_count);
      USB_Debug_Printf("# FASTLOG COLS: idx,iq_target,iq_meas,iq_raw,uq_final\r\n");
    }

    if (fast_log_dump_active)
    {
      uint8_t n;
      for (n = 0U; n < 4U && fast_log_dump_idx < pid_fast_log_count; n++)
      {
        pid_fast_log_sample_t s = pid_fast_log[fast_log_dump_idx];
        USB_Debug_Printf("%lu,%.4f,%.4f,%.4f,%.4f\r\n",
                         (unsigned long)fast_log_dump_idx,
                         s.setpoint,
                         s.filtered_iq,
                         s.raw_iq,
                         s.uq_final);
        fast_log_dump_idx++;
      }

      if (fast_log_dump_idx >= pid_fast_log_count)
      {
        USB_Debug_Printf("# FASTLOG END,count=%u\r\n", (unsigned int)pid_fast_log_count);
        USB_Debug_Printf("# FASTLOG capture paused (send fastlog:arm to arm again)\r\n");
        __disable_irq();
        pid_fast_log_count = 0U;
        pid_fast_log_full = 0U;
        __enable_irq();
        fast_log_dump_active = 0U;
        fast_log_dump_idx = 0U;
      }
    }
#endif

    if (exp1_header_printed == 0U)
    {
      USB_Debug_Printf("# EXP1: ts_ms,turn_idx,theta_mod_2pi,theta_abs,vel_raw,vel_filt,iq\r\n");
      exp1_header_printed = 1U;
    }

  #if PID_FAST_LOG_ENABLE
    if (!fast_log_dump_active && (now - last_print) >= 10U)
  #else
    if ((now - last_print) >= 10U)
  #endif
    {
      last_print = now;
      extern float Left_Target;
      extern float Left_Filtered_Iq;
      extern float left_filtered_velocity;
      extern CurrentDetect_t Left_Current_Detect;
      USB_Debug_Printf("%.2f,%.3f,%.3f,%.2f,%.3f,%.2f\r\n",
                        Left_Target,
                        Left_Current_Detect.I_a,
                        Left_Current_Detect.I_b,
                        encoder_left.angle_rad,
                        Left_Filtered_Iq,
                        left_filtered_velocity
                      );
    }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
