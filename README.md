# Floor Sense

A proprioception module for a small wheeled robot. An MPU9250 on the chassis feeds an
STM32 Nucleo-L476RG, which classifies the floor (tile, carpet, grass) with a NanoEdge AI
library and detects collisions, stalls and lifts. A Raspberry Pi 4 running ROS 2 Jazzy
receives the results over USB serial and sends wheel commands back.

```
MPU9250 --I2C 1 kHz--> STM32 (sample, classify, safety stops) --USB serial--> Pi 4 (ROS 2)
                        L298N <-- PWM                               <-- cmd_vel
```

## Layout

| Path | What |
|------|------|
| `firmware/floorsense_protocol.h` | Frame format for the STM32 code (copy into the CubeIDE project) |
| `ros2/floor_sense/` | ROS 2 package: serial bridge node and a fake Nucleo |
| `docs/PROTOCOL.md` | The serial protocol, byte by byte |

## Build on the Pi

```bash
sudo apt install -y python3-serial
cd ~/floor_ws && colcon build --symlink-install
source install/setup.bash
```

## Try it without hardware

Terminal 1, a pretend Nucleo on a virtual serial port:
```bash
ros2 run floor_sense fake_nucleo
```
Terminal 2, the bridge pointed at it:
```bash
ros2 run floor_sense bridge --ros-args -p port:=/tmp/floorsense_fake
```
Terminal 3, look at what comes out:
```bash
ros2 topic echo /proprio/terrain
ros2 topic hz /imu/data_raw
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

The fake's data is synthetic. It tests the plumbing, not the classifier.

## With the Nucleo

```bash
ros2 run floor_sense bridge          # default port /dev/ttyACM0
```

## Bridge node

| Topic | Type | Meaning |
|-------|------|---------|
| `imu/data_raw` | sensor_msgs/Imu | accel (m/s²) and gyro (rad/s), no orientation; 100 Hz |
| `proprio/terrain` | std_msgs/String | tile, carpet, grass or unknown, one per classifier window |
| `proprio/confidence` | std_msgs/Float32 | 0 to 1, for that window |
| `proprio/vibration` | std_msgs/Float32 | vertical vibration RMS in g |
| `proprio/event` | std_msgs/String | collision, slip, stuck, lifted, anomaly |
| `cmd_vel` (in) | geometry_msgs/Twist | turned into wheel duty, open loop (no encoders) |

| Parameter | Default | |
|-----------|---------|--|
| `port` | `/dev/ttyACM0` | |
| `baud` | `115200` | |
| `frame_id` | `imu_link` | |
| `wheel_base` | `0.15` | metres between wheel centres; measure yours |
| `max_speed` | `0.20` | m/s at full duty; measure yours |
| `cmd_timeout` | `0.5` | seconds before a quiet cmd_vel means stop |
| `stats_period` | `5.0` | seconds between link reports, 0 to turn off |

Float parameters need a decimal point on the command line: `-p wheel_base:=0.2`, not `0`.

After a collision, stuck or lifted event the STM32 stops the wheels and keeps them stopped
until the commanded speed goes to zero (stop publishing cmd_vel for half a second, or send
zeros). This is on purpose: the robot never drives off again on its own.

## Tests

```bash
cd ~/floor-sense/ros2/floor_sense && python3 -m pytest test/test_protocol.py
```
