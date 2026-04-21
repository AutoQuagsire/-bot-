/**
 ******************************************************************************
 * @file    usb_debug.h
 * @brief   USB 虚拟串口调试发送封装
 *
 * 依赖：CubeMX 生成的 USB_Device/App/usbd_cdc_if.c（含 CDC_Transmit_FS）
 *
 * 典型用法：
 *   USB_Debug_Printf("angle=%.2f  vel=%.2f\r\n", angle_deg, velocity);
 ******************************************************************************
 */

#ifndef __USB_DEBUG_H
#define __USB_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ── 发送缓冲区大小（字节），按需调整，不超过 APP_TX_DATA_SIZE=2048） ── */
#define USB_DEBUG_BUF_SIZE   256U

/* ── 发送忙等待超时（ms）──────────────────────────────────────────── */
#define USB_DEBUG_TIMEOUT_MS  10U

/**
 * @brief  发送原始字节数组
 * @param  buf  数据指针
 * @param  len  字节数
 */
void USB_Debug_Send(const uint8_t *buf, uint16_t len);

/**
 * @brief  发送以 '\0' 结尾的字符串
 * @param  str  字符串指针
 */
void USB_Debug_SendStr(const char *str);

/**
 * @brief  printf 风格格式化后通过虚拟串口发送
 * @param  fmt  格式字符串（与 printf 一致）
 * @param  ...  可变参数
 *
 * 示例：
 *   USB_Debug_Printf("angle=%.2f  raw=%u\r\n", encoder.angle_deg, encoder.raw_angle);
 */
void USB_Debug_Printf(const char *fmt, ...);

/**
 * @brief  返回 CDC_Transmit_FS 返回 USBD_BUSY 的累积次数（用于诊断）
 * @retval busy 次数
 */
uint32_t USB_Debug_GetBusyCount(void);

/**
 * @brief  USB 接收回调（在 CDC_Receive_FS 中调用，仅记录数据）
 * @param  buf  接收缓冲区
 * @param  len  数据长度
 */
void USB_RxCallback(uint8_t* buf, uint32_t len);

/**
 * @brief  处理 USB 接收的命令（在主循环中调用，非阻塞）
 *         格式：tgt:数值\n (例如：tgt:30.5\n)
 * 
 * @note   更新 Target 变量供 FOC 控制使用
 */
void Process_USB_Command(void);

#ifdef __cplusplus
}
#endif

#endif /* __USB_DEBUG_H */
