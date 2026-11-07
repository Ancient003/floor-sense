"""
Floor Sense serial protocol: framing, CRC and message layouts.

Every frame, in both directions, looks like this (multi-byte fields are little-endian):

    0xAA 0x55 | type u8 | seq u8 | len u8 | payload (len bytes) | crc16 u16

The CRC is CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type, seq, len and payload.
firmware/floorsense_protocol.h is the C side of this file. Change both together.
"""

from collections import namedtuple
import math
import struct

SYNC = b'\xaa\x55'
HEADER_LEN = 5          # sync (2) + type + seq + len
CRC_LEN = 2
MAX_PAYLOAD = 64

# STM32 -> Pi
MSG_IMU = 0x01
MSG_TERRAIN = 0x02
MSG_EVENT = 0x03
MSG_HEARTBEAT = 0x04
# Pi -> STM32
MSG_MOTOR = 0x10

Frame = namedtuple('Frame', 'type seq payload')
Imu = namedtuple('Imu', 't_us ax ay az gx gy gz')
Terrain = namedtuple('Terrain', 't_us terrain confidence vib_rms_mg')
Event = namedtuple('Event', 't_us event')
Heartbeat = namedtuple('Heartbeat', 't_us flags')
Motor = namedtuple('Motor', 'left right')

LAYOUTS = {
    MSG_IMU: (struct.Struct('<I6h'), Imu),
    MSG_TERRAIN: (struct.Struct('<IBBH'), Terrain),
    MSG_EVENT: (struct.Struct('<IB'), Event),
    MSG_HEARTBEAT: (struct.Struct('<IH'), Heartbeat),
    MSG_MOTOR: (struct.Struct('<hh'), Motor),
}
MSG_NAMES = {MSG_IMU: 'imu', MSG_TERRAIN: 'terrain', MSG_EVENT: 'event',
             MSG_HEARTBEAT: 'heartbeat', MSG_MOTOR: 'motor'}

TERRAIN_NAMES = {0: 'unknown', 1: 'tile', 2: 'carpet', 3: 'grass'}
EVENT_NAMES = {0: 'none', 1: 'collision', 2: 'slip', 3: 'stuck', 4: 'lifted', 5: 'anomaly'}

# Heartbeat flag bits
FLAG_IMU_OK = 0x01        # sensor answering and samples arriving on time
FLAG_MODEL_OK = 0x02      # NanoEdge library initialised
FLAG_MOTORS_ARMED = 0x04  # fresh motor command and no safety stop
FLAG_SAFETY_STOP = 0x08   # wheels stopped after collision/stuck/lifted; send MOTOR 0,0 to clear
FLAG_CMD_TIMEOUT = 0x10   # no MOTOR frame for 500 ms, wheels stopped
FLAG_NAMES = {FLAG_IMU_OK: 'imu_ok', FLAG_MODEL_OK: 'model_ok',
              FLAG_MOTORS_ARMED: 'motors_armed', FLAG_SAFETY_STOP: 'safety_stop',
              FLAG_CMD_TIMEOUT: 'cmd_timeout'}

# MPU9250 raw scaling as configured by the firmware
ACCEL_LSB_PER_G = 8192.0    # +-4 g range
GYRO_LSB_PER_DPS = 65.5     # +-500 dps range
STANDARD_GRAVITY = 9.80665
MOTOR_MAX = 1000            # motor command is per-mille duty, -1000..1000


def _crc_table():
    table = []
    for i in range(256):
        c = i << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) if c & 0x8000 else (c << 1)
        table.append(c & 0xFFFF)
    return tuple(table)


_CRC_TABLE = _crc_table()


def crc16(data, crc=0xFFFF):
    """Return the CRC-16/CCITT-FALSE of data (check value for b'123456789' is 0x29B1)."""
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ _CRC_TABLE[((crc >> 8) ^ b) & 0xFF]
    return crc


def encode(msg_type, seq, payload=b''):
    """Wrap a payload in a complete frame."""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f'payload is {len(payload)} bytes, limit is {MAX_PAYLOAD}')
    body = bytes((msg_type & 0xFF, seq & 0xFF, len(payload))) + bytes(payload)
    return SYNC + body + struct.pack('<H', crc16(body))


def pack(msg_type, seq, *fields):
    """Build a frame for a known message type from its fields."""
    layout, _ = LAYOUTS[msg_type]
    return encode(msg_type, seq, layout.pack(*fields))


def decode(frame):
    """
    Turn a Frame into a named tuple, or None for an unknown type.

    A payload longer than expected is accepted and the extra bytes ignored, so new
    fields can be appended later without breaking an older reader.
    """
    entry = LAYOUTS.get(frame.type)
    if entry is None:
        return None
    layout, kind = entry
    if len(frame.payload) < layout.size:
        raise ValueError(f'{MSG_NAMES[frame.type]} payload is {len(frame.payload)} bytes, '
                         f'expected {layout.size}')
    return kind(*layout.unpack_from(frame.payload))


def accel_to_ms2(raw):
    """Convert a raw accelerometer reading to m/s^2."""
    return raw / ACCEL_LSB_PER_G * STANDARD_GRAVITY


def gyro_to_rads(raw):
    """Convert a raw gyro reading to rad/s."""
    return math.radians(raw / GYRO_LSB_PER_DPS)


def flag_names(flags):
    """List the names of the bits set in a heartbeat flags word."""
    return [name for bit, name in FLAG_NAMES.items() if flags & bit]


class FrameParser:
    """
    Pull complete, CRC-checked frames out of a byte stream.

    Feed it whatever the serial port returns; partial frames are kept until the rest
    arrives. After a bad frame it drops one byte and searches again, so a real frame
    hidden inside a corrupted one is still found.
    """

    def __init__(self, max_payload=MAX_PAYLOAD):
        self.max_payload = max_payload
        self._buf = bytearray()
        self.frames = 0
        self.crc_errors = 0
        self.bad_length = 0
        self.discarded = 0      # bytes thrown away while hunting for a frame start

    def feed(self, data):
        """Add bytes and return the list of frames completed by them."""
        buf = self._buf
        buf += data
        out = []
        while True:
            start = buf.find(SYNC)
            if start < 0:
                keep = 1 if buf and buf[-1] == SYNC[0] else 0   # may be half a sync
                self.discarded += len(buf) - keep
                del buf[:len(buf) - keep]
                return out
            if start:
                self.discarded += start
                del buf[:start]
            if len(buf) < HEADER_LEN:
                return out
            n = buf[4]
            if n > self.max_payload:
                self.bad_length += 1
                self.discarded += 1
                del buf[:1]
                continue
            total = HEADER_LEN + n + CRC_LEN
            if len(buf) < total:
                return out
            body = bytes(buf[2:HEADER_LEN + n])
            (rx_crc,) = struct.unpack_from('<H', buf, HEADER_LEN + n)
            if crc16(body) != rx_crc:
                self.crc_errors += 1
                self.discarded += 1
                del buf[:1]
                continue
            out.append(Frame(body[0], body[1], body[3:]))
            self.frames += 1
            del buf[:total]


class SeqTracker:
    """Count frames lost on the link from gaps in the 8-bit sequence number."""

    def __init__(self):
        self.last = None
        self.lost = 0
        self.jumps = 0          # big jumps, usually the Nucleo restarting

    def update(self, seq):
        """Record a sequence number and return how many frames were skipped before it."""
        gap = 0 if self.last is None else (seq - self.last - 1) & 0xFF
        self.last = seq
        if gap >= 128:
            self.jumps += 1
            return 0
        self.lost += gap
        return gap
