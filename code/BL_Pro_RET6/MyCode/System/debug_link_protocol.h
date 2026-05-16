#ifndef DEBUG_LINK_PROTOCOL_H
#define DEBUG_LINK_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define DL_PROTO_SOF0                0x5AU
#define DL_PROTO_SOF1                0xA5U
#define DL_PROTO_VERSION             0x01U
#define DL_PROTO_MAX_PAYLOAD         240U
#define DL_PROTO_HEADER_SIZE         7U
#define DL_PROTO_CRC_SIZE            2U
#define DL_PROTO_MAX_FRAME_SIZE      (DL_PROTO_HEADER_SIZE + DL_PROTO_MAX_PAYLOAD + DL_PROTO_CRC_SIZE)

typedef enum
{
    DL_MSG_PING_REQ             = 0x01,
    DL_MSG_GET_DEVICE_INFO_REQ  = 0x02,
    DL_MSG_STREAM_CONTROL_REQ   = 0x10,
    DL_MSG_GET_PARAM_REQ        = 0x11,
    DL_MSG_SET_PARAM_REQ        = 0x12,
    DL_MSG_SAVE_PARAMS_REQ      = 0x13,
    DL_MSG_FAST_CAPTURE_REQ     = 0x14,
    DL_MSG_POWER_STAGE_REQ      = 0x15,
    DL_MSG_ATTITUDE_CONTROL_REQ = 0x16,

    DL_MSG_ACK                  = 0x80,
    DL_MSG_NACK                 = 0x81,
    DL_MSG_DEVICE_INFO_RSP      = 0x82,
    DL_MSG_PARAM_VALUE_RSP      = 0x83,
    DL_MSG_STATUS_STREAM        = 0x90,
    DL_MSG_FAST_CAPTURE_DATA    = 0x91,
    DL_MSG_EVENT                = 0x92
} DL_MessageType_t;

typedef enum
{
    DL_ACK_STATUS_OK = 0U
} DL_AckStatus_t;

typedef enum
{
    DL_NACK_BAD_LENGTH = 1U,
    DL_NACK_BAD_PARAM_ID = 2U,
    DL_NACK_BAD_PARAM_VALUE = 3U,
    DL_NACK_BUSY = 4U,
    DL_NACK_UNSUPPORTED = 5U,
    DL_NACK_BAD_FRAME = 6U
} DL_NackReason_t;

typedef enum
{
    DL_PARSE_NONE = 0,
    DL_PARSE_FRAME_OK,
    DL_PARSE_FRAME_ERROR
} DL_ParseResult_t;

typedef struct
{
    uint8_t version;
    uint8_t msg_type;
    uint8_t seq;
    uint16_t payload_len;
    uint8_t payload[DL_PROTO_MAX_PAYLOAD];
} DL_Frame_t;

typedef struct
{
    uint8_t state;
    uint8_t header[DL_PROTO_HEADER_SIZE];
    uint16_t payload_index;
    uint16_t expected_payload_len;
    uint8_t payload[DL_PROTO_MAX_PAYLOAD];
    uint8_t crc_bytes[DL_PROTO_CRC_SIZE];
    uint8_t crc_index;
} DL_ProtocolParser_t;

void DL_Protocol_InitParser(DL_ProtocolParser_t *parser);
DL_ParseResult_t DL_Protocol_InputByte(DL_ProtocolParser_t *parser, uint8_t byte, DL_Frame_t *out_frame);

uint16_t DL_Protocol_CalcCrc16(const uint8_t *data, uint16_t len);
uint16_t DL_Protocol_BuildFrame(uint8_t msg_type,
                                uint8_t seq,
                                const uint8_t *payload,
                                uint16_t payload_len,
                                uint8_t *out_buf,
                                uint16_t out_buf_size);

uint16_t DL_ReadU16LE(const uint8_t *buf);
uint32_t DL_ReadU32LE(const uint8_t *buf);
int32_t DL_ReadS32LE(const uint8_t *buf);
void DL_WriteU16LE(uint8_t *buf, uint16_t value);
void DL_WriteU32LE(uint8_t *buf, uint32_t value);
void DL_WriteS32LE(uint8_t *buf, int32_t value);

#ifdef __cplusplus
}
#endif

#endif
