/**
 ******************************************************************************
 * @file    as5047p.h
 * @brief   AS5047P 14-bit 磁编码器驱动库头文件（精简版）
 *
 * 硬件连接：
 *   SPI Mode 1 (CPOL=0, CPHA=1)，MSB First，16-bit，最大 10 MHz
 *   CSn 由软件控制，低有效
 *
 * 时序满足：
 *   tCSH/tL ≥ 350 ns（spi_transfer 入口 tCSH_delay 保证）
 *   SPI 超时：DWT 硬件周期计数器（不依赖 SysTick，ISR-safe）
 ******************************************************************************
 */

#ifndef __AS5047P_H
#define __AS5047P_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ── 寄存器地址 ─────────────────────────────────────────────────── */
#define AS5047P_REG_NOP       0x0000U
#define AS5047P_REG_ERRFL     0x0001U
#define AS5047P_REG_ANGLECOM  0x3FFFU

/* ── 预计算命令字 ────────────────────────────────────────────────── */
/* 读 ANGLECOM: addr=0x3FFF, R/W=1, bit[14:0]=0x7FFF(15个1,奇) → PARC=1 → 0xFFFF */
#define AS5047P_CMD_READ_ANGLECOM  0xFFFFU
/* 读 NOP:      addr=0x0000, R/W=1, bit[14:0]=0x4000(1个1,奇)  → PARC=1 → 0xC000 */
#define AS5047P_CMD_READ_NOP       0xC000U
/* 读 ERRFL:    addr=0x0001, R/W=1, bit[14:0]=0x4001(2个1,偶)  → PARC=0 → 0x4001 */
#define AS5047P_CMD_READ_ERRFL     0x4001U

/* ── 帧掩码 ─────────────────────────────────────────────────────── */
#define AS5047P_DATA_MASK   0x3FFFU  /* 有效角度位 [13:0]           */
#define AS5047P_EF_MASK     0x4000U  /* 错误标志位 [14]             */
#define AS5047P_RESOLUTION  16384U   /* 14 bit → 0~16383            */

/* ── ERRFL 错误位（读后自动清零）───────────────────────────────── */
#define AS5047P_ERRFL_FRERR_MASK    0x0001U
#define AS5047P_ERRFL_INVCOMM_MASK  0x0002U
#define AS5047P_ERRFL_PARERR_MASK   0x0004U

/* ── 返回状态 ───────────────────────────────────────────────────── */
typedef enum {
    AS5047P_OK    = 0,  /* 角度已正常更新              */
    AS5047P_ERROR = 1,  /* SPI 超时或校验失败          */
    AS5047P_STALE = 2   /* 保留，行为等同 ERROR        */
} AS5047P_Status;


#define AS5047P_VEL_WINDOW_N      5U
#define AS5047P_VEL_WINDOW_SIZE   (AS5047P_VEL_WINDOW_N + 1U)

/* ── 句柄结构体 ─────────────────────────────────────────────────── */
typedef struct {
    SPI_HandleTypeDef *hspi;               /* SPI 句柄                          */
    GPIO_TypeDef      *cs_port;            /* CSn GPIO 端口                     */
    uint16_t           cs_pin;             /* CSn GPIO 引脚                     */
    uint16_t           raw_angle;          /* 最近一次原始角度值 [0, 16383]     */
    float              angle_deg;          /* 最近一次角度（度）[0, 360)        */
    float              angle_rad;          /* 最近一次角度（弧度）[0, 2π)      */
    float              prev_angle_rad;     /* 上一次角度（弧度）                */
    float              angle_rad_with_track; /* 带圈数的绝对角度（弧度）        */
    int                full_rotations;     /* 完整圈数                          */
    float              velocity_rad;       /* 角速度（rad/s）                   */
    float              velocity_rad_test;       /* 角速度（rad/s）  */  
    uint8_t            vel_initialized;

    /* 新增：固定 N 重叠窗口差分测速用 */
    float              vel_angle_hist[AS5047P_VEL_WINDOW_SIZE];
    uint16_t           vel_hist_head;
    uint16_t           vel_hist_count;
    
    /* 3 点中值滤波 */
    float              vel_med3_hist[3];
    uint8_t            vel_med3_idx;
    uint8_t            vel_med3_count;

} AS5047P_Handle;

/* ── 公开函数 ───────────────────────────────────────────────────── */
void           AS5047P_Init(AS5047P_Handle *dev,
                            SPI_HandleTypeDef *hspi,
                            GPIO_TypeDef *cs_port,
                            uint16_t cs_pin);

AS5047P_Status AS5047P_GetAngleWithoutTrack(AS5047P_Handle *dev);
AS5047P_Status AS5047P_ReadAngle(AS5047P_Handle *dev);
AS5047P_Status AS5047P_GetAngle(AS5047P_Handle *dev);
float          AS5047P_Get_ElecAngle(AS5047P_Handle *dev);
float          normalizeAngle(float angle);
float          AS5047_GetVelocity(AS5047P_Handle *dev, float dt);

void  AS5047P_ResetVelocityWindow(AS5047P_Handle *dev);
float AS5047P_GetVelocityWindowed(AS5047P_Handle *dev, float dt);

#ifdef __cplusplus
}
#endif

#endif /* __AS5047P_H */
