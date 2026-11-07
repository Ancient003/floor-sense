/*
 * floorsense_protocol.h - Floor Sense serial frames for the STM32 firmware.
 *
 * Frame (multi-byte fields little-endian):
 *     0xAA 0x55 | type u8 | seq u8 | len u8 | payload[len] | crc16 u16
 * CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type, seq, len and payload.
 *
 * This is the C twin of ros2/floor_sense/floor_sense/protocol.py. Change both together.
 * Header-only and HAL-free: copy it into Core/Inc of the CubeIDE project.
 *
 * Sending (from the task that owns USART2):
 *     static uint8_t tx[FS_MAX_FRAME];          // static: DMA reads it after we return
 *     fs_imu_t m = { .t_us = now_us, .ax = ax, .ay = ay, .az = az, .gx = gx, .gy = gy, .gz = gz };
 *     size_t n = fs_build(tx, sizeof tx, FS_MSG_IMU, seq++, &m, sizeof m);
 *     HAL_UART_Transmit_DMA(&huart2, tx, n);    // wait for the previous transfer first
 *
 * Receiving motor commands (one byte at a time, e.g. from a DMA ring buffer):
 *     static fs_parser_t rx;
 *     if (fs_parse_byte(&rx, byte) && rx.type == FS_MSG_MOTOR && rx.len >= sizeof(fs_motor_t)) {
 *         fs_motor_t cmd;
 *         memcpy(&cmd, rx.payload, sizeof cmd);
 *     }
 */
#ifndef FLOORSENSE_PROTOCOL_H
#define FLOORSENSE_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__)
#error "Payload structs are sent as raw memory and assume a little-endian CPU"
#endif

#define FS_SYNC0        0xAAu
#define FS_SYNC1        0x55u
#define FS_HEADER_LEN   5u
#define FS_CRC_LEN      2u
#define FS_MAX_PAYLOAD  64u
#define FS_MAX_FRAME    (FS_HEADER_LEN + FS_MAX_PAYLOAD + FS_CRC_LEN)

/* Message types */
#define FS_MSG_IMU        0x01u   /* STM32 -> Pi */
#define FS_MSG_TERRAIN    0x02u   /* STM32 -> Pi */
#define FS_MSG_EVENT      0x03u   /* STM32 -> Pi */
#define FS_MSG_HEARTBEAT  0x04u   /* STM32 -> Pi */
#define FS_MSG_MOTOR      0x10u   /* Pi -> STM32 */

/* Terrain classes (NanoEdge class index + 1; 0 means not classified yet) */
#define FS_TERRAIN_UNKNOWN 0u
#define FS_TERRAIN_TILE    1u
#define FS_TERRAIN_CARPET  2u
#define FS_TERRAIN_GRASS   3u

/* Events */
#define FS_EVENT_NONE      0u
#define FS_EVENT_COLLISION 1u
#define FS_EVENT_SLIP      2u
#define FS_EVENT_STUCK     3u
#define FS_EVENT_LIFTED    4u
#define FS_EVENT_ANOMALY   5u

/* Heartbeat flags */
#define FS_FLAG_IMU_OK        0x0001u  /* sensor answering, samples on time */
#define FS_FLAG_MODEL_OK      0x0002u  /* NanoEdge library initialised */
#define FS_FLAG_MOTORS_ARMED  0x0004u  /* fresh command and no safety stop */
#define FS_FLAG_SAFETY_STOP   0x0008u  /* stopped after collision/stuck/lifted; MOTOR 0,0 clears */
#define FS_FLAG_CMD_TIMEOUT   0x0010u  /* no MOTOR frame for FS_CMD_TIMEOUT_MS, wheels stopped */

#define FS_CMD_TIMEOUT_MS     500u
#define FS_MOTOR_MAX          1000     /* per-mille duty, -1000..1000 */
#define FS_ACCEL_LSB_PER_G    8192     /* MPU9250 at +-4 g */
#define FS_GYRO_LSB_PER_DPS   65.5f    /* MPU9250 at +-500 dps */

typedef struct __attribute__((packed)) {
    uint32_t t_us;                     /* microseconds since boot, wraps after ~71 min */
    int16_t ax, ay, az;                /* raw accelerometer */
    int16_t gx, gy, gz;                /* raw gyro */
} fs_imu_t;

typedef struct __attribute__((packed)) {
    uint32_t t_us;
    uint8_t terrain;                   /* FS_TERRAIN_* for this window */
    uint8_t confidence;                /* 0..100 % */
    uint16_t vib_rms_mg;               /* vertical vibration RMS over the window, milli-g */
} fs_terrain_t;

typedef struct __attribute__((packed)) {
    uint32_t t_us;
    uint8_t event;                     /* FS_EVENT_* */
} fs_event_t;

typedef struct __attribute__((packed)) {
    uint32_t t_us;
    uint16_t flags;                    /* FS_FLAG_* */
} fs_heartbeat_t;

typedef struct __attribute__((packed)) {
    int16_t left;                      /* -FS_MOTOR_MAX..FS_MOTOR_MAX */
    int16_t right;
} fs_motor_t;

_Static_assert(sizeof(fs_imu_t) == 16, "fs_imu_t must match '<I6h'");
_Static_assert(sizeof(fs_terrain_t) == 8, "fs_terrain_t must match '<IBBH'");
_Static_assert(sizeof(fs_event_t) == 5, "fs_event_t must match '<IB'");
_Static_assert(sizeof(fs_heartbeat_t) == 6, "fs_heartbeat_t must match '<IH'");
_Static_assert(sizeof(fs_motor_t) == 4, "fs_motor_t must match '<hh'");

static inline uint16_t fs_crc16_update(uint16_t crc, const uint8_t *data, size_t len)
{
    uint32_t c = crc;
    for (size_t i = 0; i < len; i++) {
        c ^= (uint32_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            c = (c & 0x8000u) ? ((c << 1) ^ 0x1021u) : (c << 1);
        c &= 0xFFFFu;
    }
    return (uint16_t)c;
}

/* Write one frame into out. Returns the frame length, or 0 if it does not fit. */
static inline size_t fs_build(uint8_t *out, size_t out_size, uint8_t type, uint8_t seq,
                              const void *payload, size_t len)
{
    size_t total = FS_HEADER_LEN + len + FS_CRC_LEN;
    if (len > FS_MAX_PAYLOAD || out_size < total)
        return 0;
    const uint8_t *p = (const uint8_t *)payload;
    out[0] = FS_SYNC0;
    out[1] = FS_SYNC1;
    out[2] = type;
    out[3] = seq;
    out[4] = (uint8_t)len;
    for (size_t i = 0; i < len; i++)
        out[FS_HEADER_LEN + i] = p[i];
    uint16_t crc = fs_crc16_update(0xFFFFu, &out[2], 3u + len);
    out[FS_HEADER_LEN + len] = (uint8_t)(crc & 0xFFu);
    out[FS_HEADER_LEN + len + 1] = (uint8_t)(crc >> 8);
    return total;
}

/*
 * Byte-at-a-time receiver. Zero-initialise it (static or = {0}) before use.
 * After a CRC error it restarts at the next byte, so at most one frame is lost.
 */
typedef struct {
    uint8_t state;
    uint8_t type, seq, len, pos;
    uint16_t crc_rx;
    uint8_t payload[FS_MAX_PAYLOAD];
    uint32_t frames, crc_errors, bad_length;
} fs_parser_t;

enum { FS_ST_SYNC0 = 0, FS_ST_SYNC1, FS_ST_TYPE, FS_ST_SEQ, FS_ST_LEN,
       FS_ST_PAYLOAD, FS_ST_CRC0, FS_ST_CRC1 };

/* Feed one byte. Returns 1 when p->type, p->seq, p->len and p->payload hold a valid frame. */
static inline int fs_parse_byte(fs_parser_t *p, uint8_t b)
{
    switch (p->state) {
    case FS_ST_SYNC0:
        if (b == FS_SYNC0) p->state = FS_ST_SYNC1;
        return 0;
    case FS_ST_SYNC1:
        if (b == FS_SYNC1) p->state = FS_ST_TYPE;
        else if (b != FS_SYNC0) p->state = FS_ST_SYNC0;
        return 0;
    case FS_ST_TYPE:
        p->type = b;
        p->state = FS_ST_SEQ;
        return 0;
    case FS_ST_SEQ:
        p->seq = b;
        p->state = FS_ST_LEN;
        return 0;
    case FS_ST_LEN:
        if (b > FS_MAX_PAYLOAD) {
            p->bad_length++;
            p->state = (b == FS_SYNC0) ? FS_ST_SYNC1 : FS_ST_SYNC0;
            return 0;
        }
        p->len = b;
        p->pos = 0;
        p->state = b ? FS_ST_PAYLOAD : FS_ST_CRC0;
        return 0;
    case FS_ST_PAYLOAD:
        p->payload[p->pos++] = b;
        if (p->pos == p->len) p->state = FS_ST_CRC0;
        return 0;
    case FS_ST_CRC0:
        p->crc_rx = b;
        p->state = FS_ST_CRC1;
        return 0;
    case FS_ST_CRC1: {
        p->crc_rx |= (uint16_t)((uint16_t)b << 8);
        p->state = FS_ST_SYNC0;
        uint8_t hdr[3] = { p->type, p->seq, p->len };
        uint16_t crc = fs_crc16_update(0xFFFFu, hdr, 3u);
        crc = fs_crc16_update(crc, p->payload, p->len);
        if (crc != p->crc_rx) {
            p->crc_errors++;
            return 0;
        }
        p->frames++;
        return 1;
    }
    default:
        p->state = FS_ST_SYNC0;
        return 0;
    }
}

#endif /* FLOORSENSE_PROTOCOL_H */
