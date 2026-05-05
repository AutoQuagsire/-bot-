#include "sys.h"

// 初始化低通滤波器
void LowPassFilter_Init(LowPassFilter_t *lpf, float cutoff_freq, float sample_rate) {
    // 计算滤波系数 alpha = Tf / (Tf + Ts)
    // 其中 Tf = 1 / (2*pi*fc) 是时间常数，Ts = 1/sample_rate 是采样周期
    // 简化为：alpha = 1 / (1 + 2*pi*fc*Ts)

    float Ts = 1.0f / sample_rate;  // 采样周期
    float Tf = 1.0f / (2.0f * PI * cutoff_freq);  // 时间常数

    lpf->alpha = Tf / (Tf + Ts);  // 预计算滤波系数，避免运行时除法
    lpf->prev_y = 0.0f;
    lpf->initialized = 0;
}

// 低通滤波器更新
__attribute__((optimize("O2,fast-math")))
float LowPassFilter_Update(LowPassFilter_t *lpf, float x) {
    // 首次调用，直接使用输入值初始化
    if (!lpf->initialized) {
        lpf->prev_y = x;
        lpf->initialized = 1;
        return x;
    }

    // 一阶低通滤波：y = alpha * y_prev + (1 - alpha) * x
    // 只需 2 次乘法和 2 次加减法，无除法！
    lpf->prev_y = lpf->alpha * lpf->prev_y + (1.0f - lpf->alpha) * x;

    return lpf->prev_y;
}

// 重置滤波器
void LowPassFilter_Reset(LowPassFilter_t *lpf) {
    lpf->prev_y = 0.0f;
    lpf->initialized = 0;
}
