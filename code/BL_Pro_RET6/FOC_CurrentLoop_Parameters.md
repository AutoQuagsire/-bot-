# FOC 电流环参数导出

本文档从工程源码中提取用于电流环整定的硬件与软件参数，并计算了电流采样系数与初始建议值。

**已在代码中明确的参数**

- **采样电阻**: 0.02 Ω  
  来源: [Hardware/FOC.h](Hardware/FOC.h#L30)

- **电流采样放大倍数 (amp_gain)**: 20  
  来源: [Hardware/FOC.h](Hardware/FOC.h#L31)

- **ADC 基准与分辨率**: 3.3 V, 12-bit (4096 counts)  
  来源: [Hardware/FOC.h](Hardware/FOC.h#L24), [Core/Src/adc.c](Core/Src/adc.c#L51)

- **ADC 触发与通道**: ADC1 使用 TIM2_CC2 触发 (下降沿)，两路转换  
  来源: [Core/Src/adc.c](Core/Src/adc.c#L60-L61), [Core/Src/adc.c](Core/Src/adc.c#L80-L93)

- **PWM / 定时器设置 (用于计算 PWM 频率)**
  - TIM1: 中心对齐, ARR = 4249, PSC = 0  
    来源: [Core/Src/tim.c](Core/Src/tim.c#L50-L52)
  - TIM2: ARR = 8499 (用于 ADC 触发)  
    来源: [Core/Src/tim.c](Core/Src/tim.c#L134-L136)

- **系统时钟 / VDD**: HSE=8MHz, PLL 配置见 main.c，VDD = 3.3V  
  来源: [Core/Inc/stm32g4xx_hal_conf.h](Core/Inc/stm32g4xx_hal_conf.h#L118), [Core/Src/main.c](Core/Src/main.c#L241-L246), [Core/Inc/stm32g4xx_hal_conf.h](Core/Inc/stm32g4xx_hal_conf.h#L183)


 - **电流环执行频率 (控制中断)**: FOC_PERIOD = 0.0001 s → 10 kHz  
  来源: [Hardware/FOC.h](Hardware/FOC.h#L7-L9), [Core/Src/tim.c](Core/Src/tim.c#L187-L189), 中断回调: [System/INT.c](System/INT.c#L43)



**由代码直接可算出的关键系数**

- 电压→电流（放大后）系数:  
  K_v→i = 1 / (R_shunt * amp_gain) = 1 / (0.02 * 20) = 2.5 A/V  
  实现位置: [Hardware/FOC.c](Hardware/FOC.c#L146)

- ADC 计数→电流系数:  
  K_count→i = _ADC_CONV * K_v→i = 0.00080586 * 2.5 = 0.00201465 A/count
  （代码中以 _ADC_CONV = 3.3/4096 给出）  
  实现位置: [Hardware/FOC.h](Hardware/FOC.h#L24)，[Hardware/FOC.c](Hardware/FOC.c#L171)

- PWM 频率（TIM1, 中心对齐）: 约 20 kHz （由 TIM1 ARR=4249, PSC=0 与系统定时器频率计算）  
  实现位置: [Core/Src/tim.c](Core/Src/tim.c#L50-L52)，主程序注释: [Core/Src/main.c](Core/Src/main.c#L100-L109)

- 电流环中断频率: 10 kHz（见 FOC_PERIOD_S）  
  实现位置: [Hardware/FOC.h](Hardware/FOC.h#L7-L9)，执行在 TIM3 中断: [System/INT.c](System/INT.c#L43-L60)


**代码中未找到 / 需要补充的参数（必须由实测或电机手册提供）**

- 电机相电阻 Rs（相电阻）:10.5欧姆
- 电机相电感 Ld / Lq 或等效相电感 Ls:3.3mH

这两个参数是基于电机电气模型推导 PI/PID 理论值所必需的；代码中未包含这些常数。


**电流采样低通滤波（代码实现与实际参数）**

- **滤波器实现文件**: [Hardware/Filter.h](Hardware/Filter.h#L1) 与 [Hardware/Filter.c](Hardware/Filter.c#L1)
- **滤波器类型**: 一阶离散低通（实现为 y = alpha * y_prev + (1-alpha) * x），首次调用直接用输入初始化。
- **初始化调用位置与参数**: 在 `main.c` 中调用：
  - `LowPassFilter_Init(&current_filter, 100.0f, FOC_FREQUENCY);` — 代码中传入的截止频率为 **100 Hz**，采样率为 `FOC_FREQUENCY`（2 kHz）。  来源: [Core/Src/main.c](Core/Src/main.c#L125-L127)
  - 注释不完全一致（代码注释写到“电流滤波：10 Hz 截止，强滤波观察效果”但实际传入参数是 100.0f）。以代码为准。

- **参数含义与公式（与源码一致）**:
  - 采样周期: Ts = 1 / sample_rate = 1 / 2000 = 0.0005 s
  - 时间常数: Tf = 1 / (2 * PI * fc) （fc 为截止频率）
  - alpha = Tf / (Tf + Ts)
  - 滤波更新: y[n] = alpha * y[n-1] + (1 - alpha) * x[n]

- **数值示例（当前代码参数）**:
  - fc = 100 Hz → Tf = 1 / (2*pi*100) ≈ 0.00159155 s
  - Ts = 0.0001 s (采样率 10 kHz)
  - alpha ≈ 0.00159155 / (0.00159155 + 0.0001) ≈ 0.9410
  - 因此滤波器为: y = 0.9410 * y_prev + 0.0590 * x

- **行为细节**:
  - 首次调用 `LowPassFilter_Update` 时，滤波器直接返回输入（并设置内部状态），避免初始跳变计算问题。
  - `LowPassFilter_Reset` 可用于在需要时重置滤波器状态（在 `Hardware/Filter.h` 中声明）。



