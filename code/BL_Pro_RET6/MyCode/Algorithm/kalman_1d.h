#ifndef __KALMAN_1D_H
#define __KALMAN_1D_H

#include <stdint.h>


typedef struct
{
    float angle_rad;//估计角度
    float bias_radps; // 估计陀螺仪零偏 rad/s
    float rate_radps; // 去零偏后的角速度 rad/s

    float Pt[2][2]; // 后验误差协方差矩阵

    float Q_angle; // 角度过程噪声 
    float Q_bias; // gyro bias 过程噪声
    float R_measure; // 加速度计角度测量噪声

    float K0; // 调试用 
    float K1; // 调试用 
    float residual; // 残差 y
}Kalman1D_t;


void Kalman1D_Init(Kalman1D_t *kf, float init_angle_rad, float init_bias_radps);


float Kalman1D_Update(Kalman1D_t *kf, float measured_angle_rad, float gyro_rate_radps, float dt);





#endif