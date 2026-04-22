# Driver 配置指南

## 概述
本指南说明如何在 STM32G474 项目中完整配置 `Driver_t` 驱动模块，并将其与电机控制流程集成。

---

## 1. Driver 架构

### 数据结构
```c
typedef struct {
    uint8_t initialized;        // 初始化标志
    uint8_t enabled;            // 使能标志
    TIM_HandleTypeDef *htim;    // 指向 TIM 定时器句柄（控制 PWM）
    uint32_t chA;               // A 相 PWM 通道
    uint32_t chB;               // B 相 PWM 通道
    uint32_t chC;               // C 相 PWM 通道
    float voltage_limit;        // 最大输出电压限制
} Driver_t;
```

### 公开接口

| 函数 | 用途 |
|------|------|
| `Driver_Init(driver, htim, chA, chB, chC, voltage_limit)` | 初始化 driver 硬件参数 |
| `Driver_GetInstance()` | 获取全局 driver 实例 |
| `Driver_SetPwm(driver, ua, ub, uc)` | 设置三相 PWM 输出 |
| `Driver_Disable(driver)` | 禁用驱动（清零 PWM + 关闭使能） |

---

## 2. 配置流程（在 main.c 中）

### 第一步：包含头文件
```c
#include "BLDCMotor.h"
#include "driver.h"
#include "current_sense.h"
```

### 第二步：在 main 函数中初始化 driver

```c
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_TIM1_Init();  // 左路 PWM 定时器
    MX_TIM4_Init();  // 右路 PWM 定时器（可选）
    MX_USART1_UART_Init();
    MX_USB_DEVICE_Init();

    // ===== 驱动配置 =====
    // 获取全局 driver 实例
    Driver_t *left_driver = Driver_GetInstance();
    
    // 初始化左路驱动参数
    // htim1 是由 CubeMX 生成的 TIM1 句柄
    // 通道编号：TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3
    // voltage_limit：电源电压的 SVPWM 理论最大值（如 19V -> ~11V）
    Driver_Init(left_driver, 
                &htim1,                    // TIM1 用于左路 PWM
                TIM_CHANNEL_1,             // A 相
                TIM_CHANNEL_2,             // B 相
                TIM_CHANNEL_3,             // C 相
                11.0f);                    // 最大输出电压（V）
    
    // ===== 电机配置 =====
    Motor_t left_motor = {0};
    
    left_motor.param.pole_pairs = 14;
    left_motor.param.phase_resistance = 0.5f;
    left_motor.param.kv = 100.0f;
    left_motor.param.Lq = 0.001f;
    left_motor.param.Ld = 0.001f;
    
    left_motor.config.control_mode = motor_control_openloop_velocity;
    left_motor.config.voltage_limit = 11.0f;
    left_motor.config.voltage_sensor_align = 5.5f;
    
    left_motor.state.has_sensor = 0;  // 无传感器
    left_motor.state.sensor_direction = sensor_direction_unknown;
    
    // ===== 链接 driver 到电机 =====
    linkDriver(left_driver, &left_motor);
    
    // ===== 初始化电机 =====
    if (FOCMotor_init(&left_motor)) {
        // 初始化成功
        printf("Left motor init OK\n");
    } else {
        // 初始化失败
        printf("Left motor init FAILED\n");
        while(1);
    }
    
    // 主循环
    while(1)
    {
        // ... 电机控制代码
    }
    
    return 0;
}
```

---

## 3. 步骤说明

### Driver_Init 做了什么？
```c
void Driver_Init(Driver_t *driver, TIM_HandleTypeDef *htim,
                 uint32_t chA, uint32_t chB, uint32_t chC,
                 float voltage_limit)
{
    driver->htim = htim;               // 绑定定时器
    driver->chA = chA;                 // 绑定 A 相通道
    driver->chB = chB;                 // 绑定 B 相通道
    driver->chC = chC;                 // 绑定 C 相通道
    driver->voltage_limit = voltage_limit;  // 设置电压限制
    driver->enabled = 0;               // 默认禁用
    driver->initialized = 1;           // 标记已初始化
}
```

### linkDriver 做了什么？
```c
void linkDriver(Driver_t *driver, Motor_t *motor)
{
    motor->driver = driver;            // 把 driver 指针存到 motor
    driver->initialized = 1;           // 确保 driver 已初始化
}
```

### FOCMotor_init 做了什么？
```c
uint8_t FOCMotor_init(Motor_t *FOC_Motor)
{
    // 1. 检查 driver 连接与初始化
    if (!FOC_Motor->driver || !FOC_Motor->driver->initialized) {
        FOC_Motor->state.motor_status = motor_init_failed;
        return 0;
    }
    
    // 2. 进入初始化中状态
    FOC_Motor->state.motor_status = motor_initializing;
    
    // 3. 电压限制安全检查
    // 4. 整理电机参数
    // 5. 设置默认方向（开环无传感器）
    // 6. 调用使能、延时、置状态为未校准
    
    FOCMotor_enable(FOC_Motor);
    FOC_Motor->state.motor_status = motor_uncalibrated;
    
    return 1;
}
```

---

## 4. 工作流总结

```
┌─────────────────────────────────────┐
│ 1. Driver_Init                      │  硬件初始化
│    (绑定 TIM + 通道 + 电压限制)      │
└────────────┬────────────────────────┘
             │
             ▼
┌─────────────────────────────────────┐
│ 2. linkDriver                       │  连接层
│    (motor->driver = driver)         │
└────────────┬────────────────────────┘
             │
             ▼
┌─────────────────────────────────────┐
│ 3. FOCMotor_init                    │  业务层初始化
│    - 检查 driver                    │
│    - 电压钳位                       │
│    - 参数整理                       │
│    - 调用 enable                    │
│    - 设置状态为 uncalibrated        │
└────────────┬────────────────────────┘
             │
             ▼
       ┌──────────┐
       │ 可使用   │
       └──────────┘
```

---

## 5. PWM 通道编号参考（STM32G474）

对于 TIM1，常用的通道映射：
- `TIM_CHANNEL_1`: PA8
- `TIM_CHANNEL_2`: PA9
- `TIM_CHANNEL_3`: PA10

> 具体通道与 GPIO 的映射需根据 CubeMX 生成的配置确认。

---

## 6. 常见问题

**Q: 为什么 driver 初始化后仍需要调用 linkDriver？**

A: 这是分层设计的结果。
- `Driver_Init` 负责硬件接口配置
- `linkDriver` 负责在业务逻辑层连接驱动与电机
- 这样设计便于多电机共用一个驱动器的场景

**Q: voltage_limit 应该设置为多少？**

A: 通常设置为：`供电电压 × 0.577`（这是 SVPWM 理论最大值）
- 如果供电 19V，则 `voltage_limit = 19 × 0.577 ≈ 11V`

**Q: 能否使用多个 driver？**

A: 可以。目前 `Driver_GetInstance()` 返回单个全局实例，但架构支持改为创建多个 driver 对象，只需修改全局管理逻辑。

