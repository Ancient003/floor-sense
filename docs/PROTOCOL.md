# Floor Sense serial protocol

The link between the Nucleo (USART2, through the ST-LINK USB port) and the Pi (`/dev/ttyACM0`).
115200 baud, 8N1. Python side: `ros2/floor_sense/floor_sense/protocol.py`.
C side: `firmware/floorsense_protocol.h`. Change both together, then run the tests.

## Frame

| Bytes | Field   | Notes                                              |
|-------|---------|----------------------------------------------------|
| 2     | sync    | `0xAA 0x55`                                         |
| 1     | type    | see below                                          |
| 1     | seq     | +1 per frame sent, wraps at 255; each side has its own |
| 1     | len     | payload length, 0 to 64                            |
| len   | payload | little-endian, packed                              |
| 2     | crc     | CRC-16/CCITT-FALSE over type, seq, len, payload; low byte first |

CRC check value: `"123456789"` gives `0x29B1`. Known frame for testing:
MOTOR seq 7, left 300, right -300 is `aa 55 10 07 04 2c 01 d4 fe 8c 1f`.

## Messages

| Type | Name      | Direction  | Payload (struct)  | Size | Rate         |
|------|-----------|------------|-------------------|------|--------------|
| 0x01 | IMU       | STM32 → Pi | `<I6h` t_us, ax, ay, az, gx, gy, gz (raw) | 16 | 100 Hz |
| 0x02 | TERRAIN   | STM32 → Pi | `<IBBH` t_us, terrain, confidence %, vib_rms_mg | 8 | each window (7.8 Hz) |
| 0x03 | EVENT     | STM32 → Pi | `<IB` t_us, event | 5 | when it happens |
| 0x04 | HEARTBEAT | STM32 → Pi | `<IH` t_us, flags | 6 | 1 Hz |
| 0x10 | MOTOR     | Pi → STM32 | `<hh` left, right, -1000..1000 (per-mille duty) | 4 | 10 Hz |

Raw IMU scaling: accelerometer ±4 g = 8192 LSB/g, gyro ±500 dps = 65.5 LSB/dps.

Terrain: 0 unknown, 1 tile, 2 carpet, 3 grass. One frame per classifier window, not smoothed;
voting across windows is the Pi's job.

Events: 1 collision, 2 slip, 3 stuck, 4 lifted, 5 anomaly.

Heartbeat flags: bit 0 imu_ok, bit 1 model_ok, bit 2 motors_armed, bit 3 safety_stop,
bit 4 cmd_timeout.

## Rules the firmware must follow

- **Command timeout.** No MOTOR frame for 500 ms: stop the wheels, set `cmd_timeout`.
- **Safety stop.** Collision, stuck or lifted: stop the wheels inside the STM32, send the EVENT,
  set `safety_stop`. Stay stopped until a MOTOR frame with 0, 0 arrives.
- **Unknown types** are ignored, not treated as errors.
- **Adding fields:** append to the end of a payload only. Readers accept longer payloads and
  ignore the extra bytes, so old and new code keep working together.

## Bandwidth

At 115200 baud the link carries about 11.5 kB/s. IMU frames are 23 bytes, so 100 Hz uses
2.3 kB/s (20%). Streaming all 1000 samples a second would need 23 kB/s: raise the baud
(460800 or more) on both sides first and check that the ST-LINK port keeps up.
