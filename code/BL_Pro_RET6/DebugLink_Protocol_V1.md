# Debug Link Protocol V1

## Goal

This protocol is the dedicated host-link for the BL_Pro_RET6 project.
It is intended for:

- parameter tuning
- state streaming
- short-window high-speed capture
- fault/event reporting

It is not intended to replace the existing USB text console.
During bring-up and fault analysis:

- `USB CDC` remains the human-readable console
- `USART1 + DMA` becomes the machine-oriented host link

## Physical Link

- Interface: `USART1`
- Pins: `PA9 TX`, `PA10 RX`
- UART mode: `8N1`
- Baud rate: `921600`
- RX mode: `ReceiveToIdle DMA`
- TX mode: `DMA + software ring buffer`
- Byte order: `little-endian`

Recommended responsibilities:

- control loop ISR only writes samples into memory buffers
- packet assembly and UART TX run outside the fast control ISR

## Frame Format

Each frame uses the following binary layout:

| Field | Size | Description |
| --- | ---: | --- |
| `SOF0` | 1 | fixed `0x5A` |
| `SOF1` | 1 | fixed `0xA5` |
| `VER` | 1 | protocol version, V1 is `0x01` |
| `MSG` | 1 | message type |
| `SEQ` | 1 | sequence number, wraps naturally |
| `LEN` | 2 | payload length, little-endian |
| `PAYLOAD` | N | payload bytes |
| `CRC16` | 2 | CRC-CCITT-FALSE of `VER..PAYLOAD`, little-endian |

Notes:

- `SOF0/SOF1` are not included in CRC
- `LEN` may be `0`
- maximum payload for V1 is `240` bytes

## CRC

- name: `CRC-CCITT-FALSE`
- polynomial: `0x1021`
- init: `0xFFFF`
- xorout: `0x0000`

## Message Rules

Messages are divided into two groups:

- request/response messages
- autonomous push messages

Messages that change runtime behavior must be acknowledged.

ACK-required messages in V1:

- `SET_PARAM_REQ`
- `SAVE_PARAMS_REQ`
- `STREAM_CONTROL_REQ`
- `FAST_CAPTURE_REQ`

No ACK is required for periodic stream frames.

## Message IDs

### Host -> Device

| Name | ID | Purpose |
| --- | ---: | --- |
| `PING_REQ` | `0x01` | link keepalive |
| `GET_DEVICE_INFO_REQ` | `0x02` | read board/fw info |
| `STREAM_CONTROL_REQ` | `0x10` | start/stop state stream |
| `GET_PARAM_REQ` | `0x11` | read one parameter |
| `SET_PARAM_REQ` | `0x12` | write one parameter |
| `SAVE_PARAMS_REQ` | `0x13` | request persistent save |
| `FAST_CAPTURE_REQ` | `0x14` | arm/stop high-speed capture |

### Device -> Host

| Name | ID | Purpose |
| --- | ---: | --- |
| `ACK` | `0x80` | successful handling of a request |
| `NACK` | `0x81` | request rejected or invalid |
| `DEVICE_INFO_RSP` | `0x82` | board/fw capability report |
| `PARAM_VALUE_RSP` | `0x83` | parameter readback |
| `STATUS_STREAM` | `0x90` | periodic low-rate status stream |
| `FAST_CAPTURE_DATA` | `0x91` | buffered high-speed capture data |
| `EVENT` | `0x92` | fault/event report |

## Common Payloads

### `PING_REQ`

Payload:

| Field | Type |
| --- | --- |
| `host_time_ms` | `uint32` |

Reply:

- device sends `ACK`

### `ACK`

Payload:

| Field | Type | Description |
| --- | --- | --- |
| `req_msg` | `uint8` | acknowledged request ID |
| `status` | `uint8` | `0` means success |

### `NACK`

Payload:

| Field | Type | Description |
| --- | --- | --- |
| `req_msg` | `uint8` | rejected request ID |
| `reason` | `uint8` | see error codes |

Recommended reason codes:

- `1`: bad length
- `2`: bad parameter id
- `3`: bad parameter value
- `4`: busy
- `5`: unsupported
- `6`: crc or format error

### `GET_DEVICE_INFO_REQ`

Payload length: `0`

### `DEVICE_INFO_RSP`

Payload:

| Field | Type | Description |
| --- | --- | --- |
| `device_type` | `uint8` | project-defined type |
| `proto_version` | `uint8` | current protocol version |
| `fw_major` | `uint8` | firmware major |
| `fw_minor` | `uint8` | firmware minor |
| `fw_patch` | `uint8` | firmware patch |
| `cap_flags` | `uint16` | capability bitmask |
| `max_payload` | `uint16` | max supported payload |

Initial `cap_flags` proposal:

- bit0: status stream supported
- bit1: fast capture supported
- bit2: parameter read supported
- bit3: parameter write supported
- bit4: parameter save supported

## Stream Control

### `STREAM_CONTROL_REQ`

Payload:

| Field | Type | Description |
| --- | --- | --- |
| `enable` | `uint8` | `0` stop, `1` start |
| `stream_id` | `uint8` | V1 only supports `0` |
| `rate_hz` | `uint16` | requested output rate |
| `field_mask` | `uint32` | selected fields |

V1 recommendation:

- `stream_id = 0`
- default stream rate `100 Hz`

### `STATUS_STREAM`

This is the normal low-rate telemetry frame for tuning and plotting.

Payload:

| Field | Type | Scale | Description |
| --- | --- | --- | --- |
| `tick_ms` | `uint32` | 1 | system tick |
| `pitch_deg_x100` | `int16` | 0.01 deg | estimated pitch |
| `pitch_rate_dps_x100` | `int16` | 0.01 dps | pitch rate |
| `wheel_vel_l_x1000` | `int16` | 0.001 unit | left wheel velocity |
| `wheel_vel_r_x1000` | `int16` | 0.001 unit | right wheel velocity |
| `iq_l_x1000` | `int16` | 0.001 A | left q-axis current |
| `iq_r_x1000` | `int16` | 0.001 A | right q-axis current |
| `uq_l_mv` | `int16` | 1 mV | left voltage command |
| `uq_r_mv` | `int16` | 1 mV | right voltage command |
| `bus_mv` | `uint16` | 1 mV | bus voltage |
| `fault_flags` | `uint16` | bitmask | current fault state |

## Fast Capture

### `FAST_CAPTURE_REQ`

Payload:

| Field | Type | Description |
| --- | --- | --- |
| `cmd` | `uint8` | `0` stop, `1` arm, `2` export |
| `channel_set` | `uint8` | project-defined capture layout |
| `sample_count` | `uint16` | requested samples |
| `decimation` | `uint16` | export every Nth control sample |

Recommended V1 mode:

- short capture windows only
- typical sample count `128` to `512`
- no continuous 1 kHz long-duration streaming over UART

### `FAST_CAPTURE_DATA`

Payload:

| Field | Type | Description |
| --- | --- | --- |
| `capture_id` | `uint16` | capture sequence |
| `chunk_index` | `uint16` | current chunk number |
| `chunk_count` | `uint16` | total chunk count |
| `sample_index0` | `uint16` | first sample index in this chunk |
| `sample_count` | `uint16` | number of samples in this chunk |
| `sample_bytes` | `uint8[]` | packed sample bytes |

V1 principle:

- capture data is chunked
- each chunk must fit within the max payload
- sample layout is fixed by `channel_set`

## Parameter Access

All parameters use a uniform envelope:

- parameter ID is `uint16`
- parameter value is `int32`
- scale is defined per parameter

This keeps protocol logic simple while preserving enough range.

### `GET_PARAM_REQ`

Payload:

| Field | Type |
| --- | --- |
| `param_id` | `uint16` |

### `SET_PARAM_REQ`

Payload:

| Field | Type |
| --- | --- |
| `param_id` | `uint16` |
| `param_value` | `int32` |

### `PARAM_VALUE_RSP`

Payload:

| Field | Type |
| --- | --- |
| `param_id` | `uint16` |
| `param_value` | `int32` |

### `SAVE_PARAMS_REQ`

Payload length: `0`

## Initial Parameter Table

The exact mapping can grow later, but V1 reserves these IDs first.

| Param ID | Name | Scale |
| --- | --- | --- |
| `0x1000` | `ATT_KALMAN_Q` | value / `1e6` |
| `0x1001` | `ATT_KALMAN_R` | value / `1e6` |
| `0x2000` | `CUR_PID_KP_L` | value / `1e4` |
| `0x2001` | `CUR_PID_KI_L` | value / `1e6` |
| `0x2002` | `CUR_PID_KD_L` | value / `1e6` |
| `0x2003` | `CUR_PID_ILIM_L` | value / `1e3` A |
| `0x2010` | `CUR_PID_KP_R` | value / `1e4` |
| `0x2011` | `CUR_PID_KI_R` | value / `1e6` |
| `0x2012` | `CUR_PID_KD_R` | value / `1e6` |
| `0x2013` | `CUR_PID_ILIM_R` | value / `1e3` A |
| `0x3000` | `BAL_TARGET_PITCH` | value / `1e2` deg |
| `0x3001` | `BAL_TARGET_RATE` | value / `1e2` dps |
| `0x4000` | `STREAM_RATE_DEFAULT` | Hz |

Reserved ranges:

- `0x1000` to `0x1FFF`: attitude estimator
- `0x2000` to `0x2FFF`: motor current loop
- `0x3000` to `0x3FFF`: balance controller
- `0x4000` to `0x4FFF`: telemetry and debug
- `0xF000` to `0xFFFF`: manufacturing and internal use

## Event Frame

### `EVENT`

Payload:

| Field | Type | Description |
| --- | --- | --- |
| `event_id` | `uint16` | event type |
| `event_level` | `uint8` | `0` info, `1` warn, `2` error |
| `event_arg0` | `int32` | optional argument |

Initial event examples:

- over-current
- over-voltage
- motor enable holdoff entered
- sensor not ready
- capture complete

## Why This V1 Was Chosen

This V1 intentionally keeps the good parts of ANO-style binary framing:

- small parser
- fixed header
- explicit length
- reliable parameter transactions

But it does not inherit the full ANO ecosystem:

- no external address model
- no borrowed full message catalog
- no protocol coupling to someone else's host software

This keeps the firmware and the future custom host application aligned with the actual needs of this project.
