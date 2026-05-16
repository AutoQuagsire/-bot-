# DebugLink 当前进度总结

## 目标

在 `BL_Pro_RET6` 项目中建立一条专用的上位机调试链路，用于后续平衡调试阶段的：

- 实时状态监视
- 参数读写
- 波形/抓包
- 故障与事件观察

当前方案采用：

- `USART1`
- `921600`
- 二进制协议
- PC 端 Python CLI 验证

保留现有 `USB CDC` 作为文本调试控制台，不再让它承担主调试链路职责。

## 当前结论

截至目前，**DebugLink V1 的基础通信链路已经打通并完成初步验证**。

已经确认可用的能力：

- `PING_REQ / ACK`
- `GET_DEVICE_INFO_REQ / DEVICE_INFO_RSP`
- `STREAM_CONTROL_REQ`
- `STATUS_STREAM`
- 状态流启停

目前这条链路已经具备作为后续专属上位机基础通信层的条件。

## 协议方案

协议文件：

- [DebugLink_Protocol_V1.md](/C:/Users/Lenovo/Documents/GitHub/-bot-/code/BL_Pro_RET6/DebugLink_Protocol_V1.md)

当前 V1 帧格式：

`SOF0(0x5A) + SOF1(0xA5) + VER + MSG + SEQ + LEN(LE) + PAYLOAD + CRC16(LE)`

协议特性：

- 小端
- 固定帧头
- 显式长度
- CRC16 校验
- 支持同步命令和异步状态流并存

当前已实际用到的消息 ID：

- `0x01` `PING_REQ`
- `0x02` `GET_DEVICE_INFO_REQ`
- `0x10` `STREAM_CONTROL_REQ`
- `0x80` `ACK`
- `0x81` `NACK`
- `0x82` `DEVICE_INFO_RSP`
- `0x90` `STATUS_STREAM`

## 固件侧进展

### 已新增模块

- [debug_link.h](/C:/Users/Lenovo/Documents/GitHub/-bot-/code/BL_Pro_RET6/MyCode/System/debug_link.h)
- [debug_link.c](/C:/Users/Lenovo/Documents/GitHub/-bot-/code/BL_Pro_RET6/MyCode/System/debug_link.c)
- [debug_link_protocol.h](/C:/Users/Lenovo/Documents/GitHub/-bot-/code/BL_Pro_RET6/MyCode/System/debug_link_protocol.h)
- [debug_link_protocol.c](/C:/Users/Lenovo/Documents/GitHub/-bot-/code/BL_Pro_RET6/MyCode/System/debug_link_protocol.c)

### 已完成内容

- 协议帧解析器
- 协议帧构造器
- CRC16 计算
- `USART1` 中断接收
- 主循环命令处理
- `ACK/NACK` 回复
- `DEVICE_INFO` 回复
- `STREAM_CONTROL` 启停控制
- `STATUS_STREAM` 周期发送

### 当前固件使用方式

- `main.c` 中完成 `DebugLink_Init()`
- `while(1)` 中周期调用 `DebugLink_Process()`
- 主循环中构造 `DebugLink_StatusSnapshot_t`
- `USART1_IRQHandler` 通过 HAL 进入接收回调

相关文件：

- [main.c](/C:/Users/Lenovo/Documents/GitHub/-bot-/code/BL_Pro_RET6/Core/Src/main.c)
- [stm32g4xx_it.c](/C:/Users/Lenovo/Documents/GitHub/-bot-/code/BL_Pro_RET6/Core/Src/stm32g4xx_it.c)
- [usart.c](/C:/Users/Lenovo/Documents/GitHub/-bot-/code/BL_Pro_RET6/Core/Src/usart.c)

### 近期修正

- 修复了早期命令回复偶发卡死的问题
- 修复了流数据和命令响应混发时 CLI 误判的问题
- 增强了串口错误恢复
- 关闭了姿态模块里默认的 USB 文本调试输出开关入口
- 将状态流调度从“`last = now`”改成“按周期累加”，减小节拍漂移

## PC 端工具进展

当前 CLI 工具：

- [debuglink_cli.py](/C:/Users/Lenovo/Documents/GitHub/-bot-/code/BL_Pro_RET6/tools/debuglink_cli.py)

已具备能力：

- `ping`
- `info`
- `stream`
- 串口打开重试
- 命令重试
- 串口稳定等待
- 忽略异步 `STATUS_STREAM` 干扰

最初出现过的问题：

- Windows 下 pyserial 在收包循环中反复修改 `timeout`，导致驱动偶发报错
- 状态流开启后，后续 `ping/info` 先收到 `0x90`，CLI 误判为异常回复

当前都已修正。

## 已验证结果

### 1. Ping

已验证：

```text
[LINK] open COM33 921600 (attempt 1/3)
[TX] PING_REQ seq=1
[RX] ACK req=0x01 status=0
```

说明：

- 串口打开正常
- 下位机收包正常
- 回复路径正常

### 2. Device Info

已验证：

```text
[LINK] open COM33 921600 (attempt 1/3)
[TX] GET_DEVICE_INFO_REQ seq=2
[RX] DEVICE_INFO_RSP
  device_type   = 0x01
  proto_version = 1
  fw            = 0.1.0
  cap_flags     = 0x0001
  max_payload   = 240
```

说明：

- 固件协议版本响应正常
- 设备信息读取正常
- 当前能力标志只声明了 `STATUS_STREAM`

### 3. Stream

最新一次实测结果：

```text
[LINK] open COM33 921600 (attempt 1/3)
[TX] STREAM_CONTROL_REQ enable=1 rate=100Hz
[STREAM] started rate=100Hz
...
[STREAM] stopped. 49 frames in 0.5s (103.5 Hz)
[TX] STREAM_CONTROL_REQ enable=0 -> ACK
```

说明：

- 状态流可以正常开启
- 状态流可以正常停止
- 停流 ACK 正常返回
- 实测链路频率已经接近目标 `100Hz`

## 当前状态流字段

当前 `STATUS_STREAM` 使用的字段包括：

- `tick_ms`
- `pitch_deg_x100`
- `pitch_rate_dps_x100`
- `wheel_vel_l_x1000`
- `wheel_vel_r_x1000`
- `iq_l_x1000`
- `iq_r_x1000`
- `uq_l_mv`
- `uq_r_mv`
- `bus_mv`
- `fault_flags`

对应固件发送位置：

- [debug_link.c](/C:/Users/Lenovo/Documents/GitHub/-bot-/code/BL_Pro_RET6/MyCode/System/debug_link.c)
- [main.c](/C:/Users/Lenovo/Documents/GitHub/-bot-/code/BL_Pro_RET6/Core/Src/main.c)

## 当前仍存在的问题

通信链路本身已通，但**业务数据语义还没有完全校准**。

当前日志中仍可见：

- `bus=0.00V`
- `iq_l=0 / iq_r=0`
- `vl=0 / vr=0`
- `uq_l=1.00V / uq_r=1.00V`
- `pitch` 基本固定在 `64.4 deg` 左右

这说明当前问题已经从“通信是否正常”切换为“上报变量是否真实有效”。

大概率原因包括：

- FOC 主流程尚未完全进入真实运行状态
- `main.c` 中某些初始化/控制逻辑仍处于注释或临时测试状态
- `STATUS_STREAM` 中某些字段仍是调试期占位来源，而不是最终观测量

## 对当前阶段的判断

目前项目在“专属上位机前置工作”这一块，已经完成了最关键的第一步：

- 协议确定
- 接口确定
- 波特率确定
- 基础链路跑通

也就是说，**DebugLink 已经从“方案讨论阶段”进入“可用原型阶段”**。

## 下一步建议

建议按下面顺序推进：

1. 校准 `STATUS_STREAM` 字段来源
   目标：让 `bus / iq / vel / uq / fault` 都对应真实可调试量

2. 接入参数读写
   优先做：
   - `GET_PARAM_REQ`
   - `SET_PARAM_REQ`
   - 少量关键控制参数

3. 增加 fast capture 机制
   目标：支撑之后平衡调试阶段的短窗口高速抓包

4. 再决定是否升级为 `USART1 RX DMA + TX DMA + ring buffer`
   当前 100Hz 状态流已经基本够用，DMA 可以放到下一阶段

## 建议的下一阶段任务

最推荐的直接下一步：

**逐项核对并修正 `STATUS_STREAM` 的数据来源。**

原因：

- 现在通信层已经稳定
- 上位机工具已经能用了
- 最应该提升的是“数据是否可信”

只要这一步做好，后面的平衡参数调试就能真正开始形成闭环。
