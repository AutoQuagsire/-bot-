/**
 ******************************************************************************
 * @file    as5047p.c
 * @brief   AS5047P 14-bit 磁编码器驱动库实现
 *
 * 设计原则
 * --------
 * ISR 快速路径（AS5047P_GetAngleWithoutTrack）：
 *   - 正常：2 次 SPI 事务（pipeline）≈ 2×(tCSH 2.5µs + SPI 2µs) ≈ 9 µs
 *   - 出错：+ ERRFL pipeline + 重试 pipeline ≈ 27 µs 上限
 *   - 无论如何不超过两次 pipeline + 一次 ERRFL 读；重初始化留给后台
 *   - 使用 HAL_SPI_TransmitReceive：HAL 内部轮询 TXE/RXNE/BSY 完成传输，
 *     不依赖 HAL_GetTick() 判断完成，在高优先级 ISR 内同样可靠
 *
 * 后台恢复（AS5047P_ServiceRecovery，主循环调用）：
 *   - 读 ERRFL 日志（若 ISR 标记了 errfl_needed）
 *   - 连续失败达阈值后重新 DeInit/Init SPI 外设
 *
 * tCSH 时序保证：
 *   - spi_transfer 入口处统一插入 tCSH 延时（≥ 350 ns），
 *     调用方无需手动维护 CSn 间隔。
 ******************************************************************************
 */

#include "as5047p.h"
#include "sys.h"

/* ── 常量 ────────────────────────────────────────────────────────── */
#define AS5047P_REINIT_FAIL_THRESHOLD 4U   /* 连续失败多少次后触发 SPI 重初始化    */

/* ── 运行时状态（每个设备实例独立） ─────────────────────────────── */
typedef struct {
    AS5047P_Handle  *dev;
    uint16_t         last_errfl;      /* 最近一次读到的 ERRFL 内容（仅供日志） */
    uint8_t          fail_streak;     /* 连续读取失败计数                      */
    volatile uint8_t reinit_pending;  /* ISR 设置，主循环清除；需 volatile     */
    uint8_t          errfl_needed;    /* 后台 ERRFL 读取请求（低优先级日志用） */
} AS5047P_Runtime;

static AS5047P_Runtime s_runtime[2] = {{0}};

/* ── 运行时查找/分配 ─────────────────────────────────────────────── */
static AS5047P_Runtime *as5047p_get_runtime(AS5047P_Handle *dev)
{
    for (uint8_t i = 0; i < 2U; i++)
    {
        if (s_runtime[i].dev == dev) return &s_runtime[i];
    }
    for (uint8_t i = 0; i < 2U; i++)
    {
        if (s_runtime[i].dev == NULL)
        {
            s_runtime[i] = (AS5047P_Runtime){ .dev = dev };
            return &s_runtime[i];
        }
    }
    return &s_runtime[0]; /* 超出容量时复用 [0]，不应发生 */
}

/* ── 内部工具 ────────────────────────────────────────────────────── */

static inline void cs_low(AS5047P_Handle *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static inline void cs_high(AS5047P_Handle *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

/**
 * tCSH ≥ 350 ns（手册 tCSH/tL 要求）。
 * @170 MHz：25 次循环 × ~6 周期（volatile LDRB/STRB + NOP + 分支）≈ 882 ns，满足 ≥ 350 ns。
 * 已内置于 spi_transfer 入口，调用方不需再手动调用。
 */
static inline void as5047p_tCSH_delay(void)
{
    for (volatile uint8_t d = 0U; d < 25U; d++) { __NOP(); }
}

/** 校验帧 bit15（偶校验覆盖 bit[14:0]） */
static uint8_t as5047p_frame_parity_ok(uint16_t frame)
{
    uint16_t v = frame & 0x7FFFU;
    uint8_t  p = 0U;
    while (v != 0U) { p ^= (uint8_t)(v & 1U); v >>= 1U; }
    return p == (uint8_t)((frame >> 15U) & 1U);
}

/*
 * 单次 16-bit SPI 全双工事务。
 *
 * tCSH 延时在 cs_low 之前插入，确保无论前一事务何时结束，
 * 本次 CS 下降沿与上次 CS 上升沿之间的间隔始终 ≥ 350 ns。
 *
 * 注意：HAL_SPI_TransmitReceive 内部轮询 TXE/RXNE/BSY，不依赖 HAL_GetTick() 判断
 * 传输完成（只有超时判断依赖 Tick）。在 ISR 内 SysTick 被阻塞时，1ms 超时永远不会
 * 触发，但这无关紧要——正常传输在 ~2 µs 内 RXNE 自然置位，循环正常退出。
 * 只有 SPI 硬件卡死才会死锁，这时直接寄存器版本同样死锁。
 */
static HAL_StatusTypeDef spi_transfer(AS5047P_Handle *dev,
                                      uint16_t tx, uint16_t *rx)
{
    if (dev == NULL || dev->hspi == NULL || rx == NULL) return HAL_ERROR;
    if (HAL_SPI_GetState(dev->hspi) != HAL_SPI_STATE_READY) return HAL_BUSY;

    as5047p_tCSH_delay();   /* 保证 tCSH ≥ 350 ns */
    cs_low(dev);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(
            dev->hspi, (uint8_t *)&tx, (uint8_t *)rx, 1U, 1U);
    cs_high(dev);
    return st;
}

/**
 * AS5047P 流水线读（手册 Figure 15）：
 *   事务1  MOSI: cmd      MISO: 上一命令残留（忽略）
 *   事务2  MOSI: NOP      MISO: cmd 对应的数据帧
 */
static HAL_StatusTypeDef as5047p_read_pipeline(AS5047P_Handle *dev,
                                               uint16_t cmd, uint16_t *frame)
{
    uint16_t dummy = 0U;
    HAL_StatusTypeDef st = spi_transfer(dev, cmd, &dummy);
    if (st != HAL_OK) return st;
    return spi_transfer(dev, AS5047P_CMD_READ_NOP, frame);
}

/** 读 ERRFL 寄存器（读后自动清零） */
static HAL_StatusTypeDef as5047p_read_errfl(AS5047P_Handle *dev, uint16_t *out)
{
    uint16_t frame = 0U;
    if (out == NULL) return HAL_ERROR;
    HAL_StatusTypeDef st = as5047p_read_pipeline(dev, AS5047P_CMD_READ_ERRFL, &frame);
    if (st == HAL_OK) *out = frame & AS5047P_DATA_MASK;
    return st;
}

static void as5047p_mark_success(AS5047P_Handle *dev)
{
    as5047p_get_runtime(dev)->fail_streak = 0U;
}

static void as5047p_mark_failure(AS5047P_Handle *dev)
{
    AS5047P_Runtime *rt = as5047p_get_runtime(dev);
    if (rt->fail_streak < 0xFFU) rt->fail_streak++;
    if (rt->fail_streak >= AS5047P_REINIT_FAIL_THRESHOLD)
    {
        rt->reinit_pending = 1U;
        rt->fail_streak    = 0U;
    }
}

/* ── 公开函数实现 ────────────────────────────────────────────────── */

void AS5047P_Init(AS5047P_Handle *dev,
                  SPI_HandleTypeDef *hspi,
                  GPIO_TypeDef *cs_port,
                  uint16_t cs_pin)
{
    uint16_t errfl = 0U;

    dev->hspi             = hspi;
    dev->cs_port          = cs_port;
    dev->cs_pin           = cs_pin;
    dev->raw_angle        = 0U;
    dev->angle_deg        = 0.0f;
    dev->angle_rad        = 0.0f;
    dev->prev_angle_rad   = 0.0f;
    dev->full_rotations   = 0;
    dev->velocity_rad     = 0.0f;

    AS5047P_Runtime *rt = as5047p_get_runtime(dev);
    *rt = (AS5047P_Runtime){ .dev = dev };

    cs_high(dev);   /* 确保 CSn 初始为高 */

    /*
     * 上电时 GPIO 默认 RESET → AS5047P 检测到异常帧 → ERRFL 写入粘性错误位。
     * 必须通过一次读取使 ERRFL 自动清零，否则后续所有响应帧 EF=1。
     */
    (void)as5047p_read_errfl(dev, &errfl);
    rt->last_errfl = errfl;
}

/**
 * @brief  读取当前角度（ISR-safe 快速路径）
 *
 * 时序成本：
 *   正常路径   ≈ 5 µs（2 × SPI 事务 + 2 × tCSH）
 *   首次出错   ≈ 17 µs（额外 ERRFL 读 + 重试 pipeline）
 *
 * 返回值：
 *   AS5047P_OK    — 角度已更新
 *   AS5047P_STALE — 本次读取失败但未达重试阈值，dev->angle_rad 保持上次有效值
 *   AS5047P_ERROR — 连续失败达阈值，已置 reinit_pending，请尽快调用 ServiceRecovery
 */
AS5047P_Status AS5047P_GetAngleWithoutTrack(AS5047P_Handle *dev)
{
    uint16_t rx = 0U, errfl = 0U;
    HAL_StatusTypeDef st;
    AS5047P_Runtime *rt;

    if (dev == NULL) return AS5047P_ERROR;

    rt = as5047p_get_runtime(dev);

    /* 已置重初始化标志 → 不再尝试 SPI，等后台恢复 */
    if (rt->reinit_pending) return AS5047P_ERROR;

    /* ── 第一次尝试 ─────────────────────────────────────────────── */
    st = as5047p_read_pipeline(dev, AS5047P_CMD_READ_ANGLECOM, &rx);

    if (st == HAL_OK && as5047p_frame_parity_ok(rx))
    {
        /* 奇偶校验通过 → 数据有效。若 EF=1 则读 ERRFL 清粘性位 */
        if (rx & AS5047P_EF_MASK)
        {
            (void)as5047p_read_errfl(dev, &errfl);
            rt->last_errfl = errfl;
        }
        goto accept_angle;
    }

    /* ── 第一次失败：读 ERRFL（清粘性位），然后重试 ─────────────── */
    /*
     * 必须先读 ERRFL，否则第二次 pipeline 响应帧里 EF 仍为 1，
     * 即使数据本身正确也会被误判为失败。
     *
     * 要求：出错后立刻读 ERRFL（检到 PARERR/FRERR 后丢弃并重试）
     */
    (void)as5047p_read_errfl(dev, &errfl);
    rt->last_errfl = errfl;

    /*
     * CS 复位间隙：显式拉高 CSn + 延时，满足"连续失败重新拉高 CSn → 延时 → 重发"。
     * （spi_transfer 内部已有 tCSH，此处为语义可见性保留显式步骤）
     */
    cs_high(dev);
    as5047p_tCSH_delay();

    /* ── 第二次尝试（最终一次）────────────────────────────────────── */
    st = as5047p_read_pipeline(dev, AS5047P_CMD_READ_ANGLECOM, &rx);

    if (st != HAL_OK || !as5047p_frame_parity_ok(rx))
    {
        /* 两次均失败 → 累计 fail_streak，可能触发 reinit_pending */
        as5047p_mark_failure(dev);
        return (rt->reinit_pending) ? AS5047P_ERROR : AS5047P_STALE;
    }

    if (rx & AS5047P_EF_MASK)
    {
        (void)as5047p_read_errfl(dev, &errfl);
        rt->last_errfl = errfl;
    }

accept_angle:
    dev->raw_angle      = rx & AS5047P_DATA_MASK;
    dev->angle_deg      = (float)dev->raw_angle * (360.0f     / (float)AS5047P_RESOLUTION);
    dev->prev_angle_rad = dev->angle_rad;
    dev->angle_rad      = (float)dev->raw_angle * (6.28318530f / (float)AS5047P_RESOLUTION);
    as5047p_mark_success(dev);
    return AS5047P_OK;
}

/**
 * @brief  后台故障恢复服务（在主循环中调用，不可在 ISR 中调用）
 *
 * 职责：
 *   1. 若 errfl_needed：读 ERRFL 记录错误类型（低优先级日志）
 *   2. 若 reinit_pending：DeInit → Init SPI 外设，清 ERRFL，重置状态
 *
 * 满足"再不行就重新初始化 SPI 外设"要求。
 */
void AS5047P_ServiceRecovery(AS5047P_Handle *dev)
{
    AS5047P_Runtime *rt;
    uint16_t errfl = 0U;

    if (dev == NULL || dev->hspi == NULL) return;

    rt = as5047p_get_runtime(dev);

    /* 后台 ERRFL 读取（仅用于日志，无需在 ISR 路径上做） */
    if (rt->errfl_needed)
    {
        if (as5047p_read_errfl(dev, &errfl) == HAL_OK)
            rt->last_errfl = errfl;
        rt->errfl_needed = 0U;
    }

    if (!rt->reinit_pending) return;

    /* ── SPI 外设重初始化 ──────────────────────────────────────── */
    cs_high(dev);
    (void)HAL_SPI_DeInit(dev->hspi);
    (void)HAL_SPI_Init(dev->hspi);

    /* 清 ERRFL（重初始化后 CS 短暂置低可能产生新帧错误） */
    if (as5047p_read_errfl(dev, &errfl) == HAL_OK)
        rt->last_errfl = errfl;

    rt->reinit_pending = 0U;
    rt->fail_streak    = 0U;
    rt->errfl_needed   = 0U;
}

/* ── 兼容/扩展接口 ───────────────────────────────────────────────── */

AS5047P_Status AS5047P_ReadAngle(AS5047P_Handle *dev)
{
    return AS5047P_GetAngleWithoutTrack(dev);
}

AS5047P_Status AS5047P_GetAngle(AS5047P_Handle *dev)
{
    AS5047P_Status sta = AS5047P_GetAngleWithoutTrack(dev);
    if (sta == AS5047P_ERROR) return AS5047P_ERROR;

    float d_rad = dev->angle_rad - dev->prev_angle_rad;
    if (fabsf(d_rad) > (0.8f * 2.0f * PI))
    {
        dev->full_rotations += (d_rad > 0.0f) ? -1 : 1;
    }
    dev->angle_rad_with_track = (float)dev->full_rotations * 2.0f * PI + dev->angle_rad;
    return sta;
}

float AS5047P_Get_ElecAngle(AS5047P_Handle *dev)
{
    AS5047P_GetAngleWithoutTrack(dev);
    return normalizeAngle((float)(DIR * PolePair) * dev->angle_rad - zero_elec_angle);
}

float normalizeAngle(float angle)
{
    float a = fmod(angle, 2.0f * PI);
    return (a >= 0.0f) ? a : (a + 2.0f * PI);
}

float AS5047_GetVelocity(AS5047P_Handle *dev, float dt)
{
    float d_angle = dev->angle_rad - dev->prev_angle_rad;
    if      (d_angle >  PI) d_angle -= 2.0f * PI;
    else if (d_angle < -PI) d_angle += 2.0f * PI;
    dev->velocity_rad = d_angle / dt;
    return dev->velocity_rad;
}
