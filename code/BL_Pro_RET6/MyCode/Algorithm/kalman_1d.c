#include "kalman_1d.h"

void Kalman1D_Init(Kalman1D_t *kf, float init_angle_rad, float init_bias_radps)
{
    if (kf == 0) {
        return;
    }
    
    //1.初始化状态和输入
    kf->angle_rad = init_angle_rad;
    kf->bias_radps = init_bias_radps;
    kf->rate_radps = 0.0f;              //去零偏后的角速度

    //2.初始化后验误差协方差矩阵
    kf->Pt[0][0] = 0.01f; // 角度估计初始误差较大
    kf->Pt[0][1] = 0.0f;
    kf->Pt[1][0] = 0.0f;
    kf->Pt[1][1] = 0.01f; 

    //3.初始化Q、R参数
    kf->Q_angle = 1e-5f;    //角度预测过程噪声   角速度瞬时测量噪声在这个模型里主要折算进 Q_angle。
    kf->Q_bias = 1e-7f;     //陀螺仪偏移噪声
    kf->R_measure = 3e-3f;  //加速度计测量噪声

    //4.初始化调试量
    kf->K0 = 0.0f;
    kf->K1 = 0.0f;
    kf->residual = 0.0f;

}





float Kalman1D_Update(Kalman1D_t *kf, float measured_angle_rad, float gyro_rate_radps, float dt)
{
    if (kf == 0 || dt <= 0.0f) {
    return 0.0f;
    }

    //1.建立状态方程
    kf->rate_radps = gyro_rate_radps - kf->bias_radps; // 去零偏后的角速度
    kf->angle_rad += kf->rate_radps;                   //更新先验预测角度


    // Q_angle / Q_bias are process noise intensity per second.
    // Q_angle_d / Q_bias_d are discrete process noise covariance per update.
    float Q_angle_d = kf->Q_angle * dt;
    float Q_bias_d  = kf->Q_bias  * dt;

    //2.先验协方差预测
    kf->Pt[0][0] = kf->Pt[0][0] - kf->Pt[0][1] * dt - kf->Pt[0][1] * dt + kf->Pt[1][1] * dt * dt +  Q_angle_d;
    kf->Pt[0][1] = kf->Pt[0][1] - kf->Pt[1][1] * dt;
    kf->Pt[1][0] = kf->Pt[1][0] - kf->Pt[1][1] * dt;
    kf->Pt[1][1] = kf->Pt[1][1] + Q_bias_d;

    //3.残差公式
    //残差的维度由测量值 z 决定，不由状态量 x 决定,所以这里没有陀螺仪bias相关的残差
    float yt = measured_angle_rad - 1 * kf->angle_rad;
    kf->residual = yt;

    //4.写Kalman增益Kt
    //分母为H * Pt * HT + R
    float S = kf->Pt[0][0] + kf->R_measure;
    if (S <= 1.0e-12f) {
        return kf->angle_rad;
    }//防止分母过小、增益过大引起数值不稳定

    //分子为Pt * HT, 这里H是[1 0], 所以分子就是Pt的第一列
    //直接写增益
    float Kt0 = kf->Pt[0][0] / S;
    float Kt1 = kf->Pt[1][0] / S;

    //保存调试量
    kf->K0 = Kt0;
    kf->K1 = Kt1;

    //5.更新最优估计值
    kf->angle_rad  += Kt0 * yt;
    kf->bias_radps += Kt1 * yt;
    //注意这里 bias 也被残差修正。虽然测量值只测角度，但因为 P[1][0] 反映了 bias 和 angle 误差的相关性，所以 K1 可以修正 bias。

    //6.更新后验协方差矩阵
    //这里一定要先保存旧的 P00 和 P01
    float P00_temp = kf->Pt[0][0];
    float P01_temp = kf->Pt[0][1];

    //这里最容易犯的错是：不保存临时变量，导致前面改了 P[0][0]，后面又用到已经被改过的值。
    kf->Pt[0][0] -= Kt0 * P00_temp;
    kf->Pt[0][1] -= Kt0 * P01_temp;
    kf->Pt[1][0] -= Kt1 * P00_temp;
    kf->Pt[1][1] -= Kt1 * P01_temp;

    return kf->angle_rad;
}
