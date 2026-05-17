# 上位机工具功能汇总（DebugLink）

## 范围与目录
- 适用目录：`code/BL_Pro_RET6/tools` 下的全部上位机 Python 工具与脚本。
- 目标：通过 DebugLink 串口协议实现设备发现、状态流、FastRing 快速数据导出、以及 GUI/CLI 交互。

## 运行依赖
- Python 3
- `pyserial`（CLI 与传输层依赖）
- `PySide6`（GUI 依赖）

## 命令行工具：debuglink_cli.py
### 连接与通用参数
- `--port`：串口号（必填，例如 COM33）。
- `--baud`：波特率，默认 921600。
- `--open-retries`：串口打开重试次数。
- `--retries`：请求/响应命令重试次数。
- `--settle-ms`：打开串口后的静置时间，避免刚打开就发送导致失败。
- `--retry-delay-ms`：命令重试之间的延迟。

### 基础命令
- `ping`：发送 PING_REQ 并等待 ACK。
- `info`：获取设备信息（device type、协议版本、固件版本、cap flags、max payload）。

### 控制命令
- `driver on|off`：打开或关闭功率级（Power Stage）。
- `balance on|off`：开启或关闭姿态/速度控制环。

### 状态流（STATUS_STREAM）
- `stream --rate <hz>`：启动实时状态流，持续打印遥测数据，Ctrl-C 终止。
- 输出字段包含：
  - `tick_ms`、姿态目标/测量值、速度环 P/I 项、速度目标/测量值、Iq 命令与限幅、左右电流与电压、母线电压、故障标志与标签。
- 停止后会发送停止帧，并打印实际统计（帧数与平均频率）。

### FastRing 数据导出
- `fastring status`：读取 FastRing 状态（total_count、capacity、head、write_seq）。
- `fastring dump --out <file>`：读取最新 FastRing 窗口并写入单个“左右合并”CSV。
- `fastring side --side L|R --out <file>`：读取快照后只导出单侧数据。
- `fastring split --left-out <file> --right-out <file>`：一次快照同时导出左右 CSV。
- FastRing 导出流程：先 `status` 检查，再 `snapshot` 冻结，再循环 `read_chunk` 拉取所有样本。

### CSV 输出格式
- 双侧合并 CSV 列：
  - `idx,target_iq_l,iq_ref_l,filtered_iq_l,raw_iq_l,uq_final_l,`
    `target_iq_r,iq_ref_r,filtered_iq_r,raw_iq_r,uq_final_r,`
    `bus_v,sample_idx,status_flags`
- 单侧 CSV 列：
  - `idx,target_iq,iq_ref,filtered_iq,raw_iq,uq_final,source,capture_id`

## 图形界面：debuglink_gui.py
### Tab 结构
- Live：连接/断开、开始/停止流、暂停/恢复视图、实时状态展示。
- FastRing：状态查询、Dump L/R/Both/DUAL 导出。
- Control：Ping + Driver/Balance 按钮（目前仅日志提示“预留”）。

### Live 页行为
- Connect/Disconnect：打开或关闭串口连接。
- Start/Stop Stream：启停 STATUS_STREAM；Live 页同步显示 stream 状态。
- Pause View：仅暂停界面刷新，不影响设备端 stream。
- Live 字段包含：姿态目标/测量、速度环 P/I 项、速度目标/测量、Iq 命令与限幅、左右 Iq 与 Uq、母线电压、故障标志与标签等。

### FastRing 页行为
- `FastRing Status`：读取并显示总样本数、容量、head、write_seq。
- `Dump L` / `Dump R`：导出单侧 CSV。
- `Dump Both`：一次快照导出左右两份 CSV。
- `Dump Dual`：导出双侧合并 CSV。
- 导出时会锁定界面防止重复操作；完成后在日志区提示结果。

### 输出路径与默认文件名
- FastRing 导出默认写入 `current_loop_data/`。
- GUI 默认文件名示例：`left.csv`、`right.csv`、`fastring_dual.csv`。

### Mock 模式
- `--mock`：不连接串口，使用 MockFeeder 模拟 LiveFrame 与 FastRing 数据。
- 适合 UI 联调与演示。

## 传输层：debuglink_transport.py
### 连接与线程模型
- 串口连接采用互斥锁统一管理，避免命令与流读线程同时读写。
- `connect()` 会清理输入输出缓冲并重置内部序列号。
- `disconnect()` 会优先停止流线程。

### 基础 API
- `ping()`：发送 PING_REQ 并判断 ACK 状态。
- `get_info()`：获取设备信息并返回字典。

### Status Stream
- `stream_start(rate_hz)`：发送启用指令并启动后台读线程。
- `stream_stop()`：停止线程并发送停止指令。
- `set_stream_callback(cb)`：注册回调，收到 STATUS_STREAM 后解析为 `LiveFrame`。

### FastRing 访问
- `fastring_status()`：获取实时 FastRing 元数据。
- `fastring_snapshot()`：冻结快照并返回元数据。
- `fastring_read_chunk()`：按 `snapshot_write_seq + start_idx + max_samples` 拉取数据块。

### 异常类型
- `TransportError`：串口错误、协议错误等。
- `TransportTimeout`：请求超时。

## 解析与数据模型
### debuglink_models.py
- `LiveFrame`：STATUS_STREAM 解码后的实时遥测。
- `FastRingMeta`：FastRing 元信息（op_echo、total_count、capacity、head、write_seq）。
- `FastRingSample`：单条双侧采样（左右 target/iq_ref/filtered/raw/uq、bus_v、sample_idx、status_flags）。

### debuglink_parser.py
- `parse_status_stream()`：将 42 字节流帧解析为 `LiveFrame`。
- `parse_fastring_status()`：解析 FastRing 状态（固定 11 字节）。
- `parse_fastring_chunk()`：解析 FastRing 数据块（header 12 字节 + N * 26 字节样本）。
- 故障标志解码：输出 `spdfltL/spdfltR/stack/it/bus/iloop/sloop/cloop/drv_off/bal` 等标签。

## 附属脚本与工具
- debuglink_transport_demo.py：演示连接、ping、get_info、stream，并打印第一帧。
- ping_test.py：最小化 PING/ACK 测试脚本。
- tmp_transport_idempotent_test.py：验证 `stream_start` 重复调用与频率切换。
- fastring_csv_inspect.py：双侧 CSV 统计、差分统计、Pearson 一致性检查。
- fastcap_csv_inspect.py：单侧 CSV 统计与双文件对比。
- fastring_plot_model.py：曲线系列定义、缓冲区与降采样逻辑。
