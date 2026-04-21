#ifndef __FILTER_H
#define __FILTER_H


#include "stdint.h"

// 高性能低通滤波器（固定采样率版本，无除法）
typedef struct {
    float prev_y;        // 上次滤波输出
    float alpha;         // 滤波系数（预计算，避免运行时除法）
    uint8_t initialized; // 是否已初始化
} LowPassFilter_t;

// 初始化低通滤波器
// lpf: 滤波器指针
// cutoff_freq: 截止频率 (Hz)，建议 5-20Hz 用于速度滤波
// sample_rate: 采样率 (Hz)，与定时器中断频率一致
void LowPassFilter_Init(LowPassFilter_t *lpf, float cutoff_freq, float sample_rate);

// 低通滤波器更新（高性能版本，无除法）
// lpf: 滤波器指针
// x: 输入值（原始测量值）
// 返回: 滤波后的值
float LowPassFilter_Update(LowPassFilter_t *lpf, float x);

// 重置滤波器（可选）
void LowPassFilter_Reset(LowPassFilter_t *lpf);

#endif
