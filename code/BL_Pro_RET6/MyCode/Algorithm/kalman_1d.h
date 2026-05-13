#ifndef __KALMAN_1D_H
#define __KALMAN_1D_H

#include <stdint.h>
#include "imu_types.h"

typedef struct
{
    float angle_rad;//估计角度
    float angle_degree; // 角度的度数表示，仅供调试用
    float bias_radps; // 估计陀螺仪零偏 rad/s
    float rate_radps; // 去零偏后的角速度 rad/s
    float accel_pitch_rad; // 加速度计测量的俯仰角 rad
    float acc_norm ;//加速度计数据模长

    float Pt[2][2]; // 后验误差协方差矩阵

    float Q_angle; // 角度过程噪声 
    float Q_bias; // gyro bias 过程噪声
    float R_measure; // 加速度计角度测量噪声

    float K0; // 调试用 
    float K1; // 调试用 
    float residual; // 残差 y

    uint8_t initialized;
}Kalman1D_t;


void Kalman1D_Init(Kalman1D_t *kf, float init_angle_rad, float init_bias_radps);


float Kalman1D_Update(Kalman1D_t *kf, IMU_PhysData_t IMU_PhysData, float dt);




#endif
