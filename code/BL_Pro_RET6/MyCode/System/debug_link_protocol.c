#include "debug_link_protocol.h"

#include <stddef.h>
#include <string.h>

enum
{
    DL_PARSE_STATE_WAIT_SOF0 = 0,
    DL_PARSE_STATE_WAIT_SOF1,
    DL_PARSE_STATE_READ_FIXED_HEADER,
    DL_PARSE_STATE_READ_PAYLOAD,
    DL_PARSE_STATE_READ_CRC
};

static void DL_Protocol_ResetParser(DL_ProtocolParser_t *parser)
{
    if (parser == NULL) {
        return;
    }

    parser->state = DL_PARSE_STATE_WAIT_SOF0;
    memset(parser->header, 0, sizeof(parser->header));
    parser->payload_index = 0U;
    parser->expected_payload_len = 0U;
    memset(parser->payload, 0, sizeof(parser->payload));
    memset(parser->crc_bytes, 0, sizeof(parser->crc_bytes));
    parser->crc_index = 0U;
}

void DL_Protocol_InitParser(DL_ProtocolParser_t *parser)
{
    DL_Protocol_ResetParser(parser);
}

uint16_t DL_Protocol_CalcCrc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;

    if (data == NULL) {
        return 0U;
    }

    for (i = 0U; i < len; i++) {
        uint8_t bit;
        crc ^= (uint16_t)data[i] << 8;
        for (bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            } else {
                crc <<= 1U;
            }
        }
    }

    return crc;
}

uint16_t DL_ReadU16LE(const uint8_t *buf)
{
    return (uint16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
}

uint32_t DL_ReadU32LE(const uint8_t *buf)
{
    return (uint32_t)buf[0]
         | ((uint32_t)buf[1] << 8)
         | ((uint32_t)buf[2] << 16)
         | ((uint32_t)buf[3] << 24);
}

int32_t DL_ReadS32LE(const uint8_t *buf)
{
    return (int32_t)DL_ReadU32LE(buf);
}

void DL_WriteU16LE(uint8_t *buf, uint16_t value)
{
    buf[0] = (uint8_t)(value & 0xFFU);
    buf[1] = (uint8_t)((value >> 8) & 0xFFU);
}

void DL_WriteU32LE(uint8_t *buf, uint32_t value)
{
    buf[0] = (uint8_t)(value & 0xFFU);
    buf[1] = (uint8_t)((value >> 8) & 0xFFU);
    buf[2] = (uint8_t)((value >> 16) & 0xFFU);
    buf[3] = (uint8_t)((value >> 24) & 0xFFU);
}

void DL_WriteS32LE(uint8_t *buf, int32_t value)
{
    DL_WriteU32LE(buf, (uint32_t)value);
}

uint16_t DL_Protocol_BuildFrame(uint8_t msg_type,
                                uint8_t seq,
                                const uint8_t *payload,
                                uint16_t payload_len,
                                uint8_t *out_buf,
                                uint16_t out_buf_size)
{
    uint16_t frame_len;
    uint16_t crc;

    if (out_buf == NULL) {
        return 0U;
    }

    if (payload_len > DL_PROTO_MAX_PAYLOAD) {
        return 0U;
    }

    if ((payload_len > 0U) && (payload == NULL)) {
        return 0U;
    }

    frame_len = (uint16_t)(DL_PROTO_HEADER_SIZE + payload_len + DL_PROTO_CRC_SIZE);
    if (out_buf_size < frame_len) {
        return 0U;
    }

    out_buf[0] = DL_PROTO_SOF0;
    out_buf[1] = DL_PROTO_SOF1;
    out_buf[2] = DL_PROTO_VERSION;
    out_buf[3] = msg_type;
    out_buf[4] = seq;
    DL_WriteU16LE(&out_buf[5], payload_len);

    if (payload_len > 0U) {
        memcpy(&out_buf[7], payload, payload_len);
    }

    crc = DL_Protocol_CalcCrc16(&out_buf[2], (uint16_t)(5U + payload_len));
    DL_WriteU16LE(&out_buf[7 + payload_len], crc);
    return frame_len;
}

DL_ParseResult_t DL_Protocol_InputByte(DL_ProtocolParser_t *parser, uint8_t byte, DL_Frame_t *out_frame)
{
    uint16_t crc_calc;
    uint16_t crc_recv;

    if ((parser == NULL) || (out_frame == NULL)) {
        return DL_PARSE_FRAME_ERROR;
    }

    switch (parser->state)
    {
    case DL_PARSE_STATE_WAIT_SOF0:
        if (byte == DL_PROTO_SOF0) {
            DL_Protocol_ResetParser(parser);
            parser->state = DL_PARSE_STATE_WAIT_SOF1;
        }
        break;

    case DL_PARSE_STATE_WAIT_SOF1:
        if (byte == DL_PROTO_SOF1) {
            parser->state = DL_PARSE_STATE_READ_FIXED_HEADER;
            parser->payload_index = 0U;
        } else if (byte == DL_PROTO_SOF0) {
            parser->state = DL_PARSE_STATE_WAIT_SOF1;
        } else {
            DL_Protocol_ResetParser(parser);
        }
        break;

    case DL_PARSE_STATE_READ_FIXED_HEADER:
        parser->header[parser->payload_index++] = byte;
        if (parser->payload_index >= 5U) {
            parser->expected_payload_len = DL_ReadU16LE(&parser->header[3]);
            parser->payload_index = 0U;

            if ((parser->header[0] != DL_PROTO_VERSION) ||
                (parser->expected_payload_len > DL_PROTO_MAX_PAYLOAD)) {
                DL_Protocol_ResetParser(parser);
                return DL_PARSE_FRAME_ERROR;
            }

            parser->state = (parser->expected_payload_len == 0U)
                          ? DL_PARSE_STATE_READ_CRC
                          : DL_PARSE_STATE_READ_PAYLOAD;
        }
        break;

    case DL_PARSE_STATE_READ_PAYLOAD:
        parser->payload[parser->payload_index++] = byte;
        if (parser->payload_index >= parser->expected_payload_len) {
            parser->payload_index = 0U;
            parser->crc_index = 0U;
            parser->state = DL_PARSE_STATE_READ_CRC;
        }
        break;

    case DL_PARSE_STATE_READ_CRC:
        parser->crc_bytes[parser->crc_index++] = byte;
        if (parser->crc_index >= DL_PROTO_CRC_SIZE) {
            uint8_t crc_buf[5U + DL_PROTO_MAX_PAYLOAD];

            memcpy(crc_buf, parser->header, 5U);
            if (parser->expected_payload_len > 0U) {
                memcpy(&crc_buf[5], parser->payload, parser->expected_payload_len);
            }

            crc_calc = DL_Protocol_CalcCrc16(crc_buf, (uint16_t)(5U + parser->expected_payload_len));
            crc_recv = DL_ReadU16LE(parser->crc_bytes);

            if (crc_calc != crc_recv) {
                DL_Protocol_ResetParser(parser);
                return DL_PARSE_FRAME_ERROR;
            }

            out_frame->version = parser->header[0];
            out_frame->msg_type = parser->header[1];
            out_frame->seq = parser->header[2];
            out_frame->payload_len = parser->expected_payload_len;
            if (parser->expected_payload_len > 0U) {
                memcpy(out_frame->payload, parser->payload, parser->expected_payload_len);
            }

            DL_Protocol_ResetParser(parser);
            return DL_PARSE_FRAME_OK;
        }
        break;

    default:
        DL_Protocol_ResetParser(parser);
        return DL_PARSE_FRAME_ERROR;
    }

    return DL_PARSE_NONE;
}
