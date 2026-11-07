"""
ROS 2 bridge between the Floor Sense Nucleo and the Pi.

Serial in  -> imu/data_raw, proprio/terrain, proprio/confidence, proprio/vibration,
              proprio/event
cmd_vel in -> MOTOR frames out, 10 times a second (zeros when cmd_vel goes quiet)

    ros2 run floor_sense bridge
    ros2 run floor_sense bridge --ros-args -p port:=/tmp/floorsense_fake
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import serial

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String

from floor_sense import protocol as fs

SILENCE_TIMEOUT = 2.0      # seconds without bytes before the link counts as lost
WARN_EVENTS = {'collision', 'stuck', 'lifted', 'slip'}


class Bridge(Node):
    """Owns the serial port; translates frames to topics and cmd_vel to motor frames."""

    def __init__(self):
        super().__init__('floor_sense_bridge')
        self.port = self.declare_parameter('port', '/dev/ttyACM0').value
        self.baud = self.declare_parameter('baud', 115200).value
        self.frame_id = self.declare_parameter('frame_id', 'imu_link').value
        self.wheel_base = self.declare_parameter('wheel_base', 0.15).value
        self.max_speed = self.declare_parameter('max_speed', 0.20).value
        self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.5).value
        stats_period = self.declare_parameter('stats_period', 5.0).value

        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', qos_profile_sensor_data)
        self.terrain_pub = self.create_publisher(String, 'proprio/terrain', 10)
        self.conf_pub = self.create_publisher(Float32, 'proprio/confidence', 10)
        self.vib_pub = self.create_publisher(Float32, 'proprio/vibration', 10)
        self.event_pub = self.create_publisher(String, 'proprio/event', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)

        self.ser = None
        self.receiving = False     # these three only decide which log lines appear
        self.waiting_logged = False
        self.open_failed = False
        self.last_rx = None
        self.parser = fs.FrameParser()
        self.seq = fs.SeqTracker()
        self.tx_seq = 0
        self.tx_dropped = 0
        self.counts = {}
        self.prev_counts = {}
        self.bad_payload = 0
        self.unknown = 0
        self.flags = None
        self.last_terrain = None
        self.cmd = (0, 0)
        self.cmd_time = None

        self.create_timer(0.005, self.poll)
        self.create_timer(0.1, self.send_motor)
        self.create_timer(1.0, self.check_link)
        if stats_period > 0:
            self.stats_period = stats_period
            self.create_timer(stats_period, self.report)
        self.check_link()

    # ---- link management ------------------------------------------------------

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def check_link(self):
        """Open the port if needed; drop it if the Nucleo has gone quiet."""
        if self.ser is not None:
            if self.now_s() - self.last_rx > SILENCE_TIMEOUT:
                self.drop_link(f'no data for {SILENCE_TIMEOUT:.0f} s')
            return
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0, write_timeout=0.05)
        except (serial.SerialException, OSError) as e:
            if not self.open_failed:
                self.get_logger().warn(f'Cannot open {self.port} ({e}). Retrying every second.')
                self.open_failed = True
            return
        self.ser.reset_input_buffer()
        self.parser = fs.FrameParser()
        self.seq = fs.SeqTracker()
        self.last_rx = self.now_s()
        if not self.waiting_logged:
            self.get_logger().info(f'Opened {self.port} at {self.baud} baud, waiting for data')
            self.waiting_logged = True
        self.open_failed = False

    def drop_link(self, reason):
        if self.receiving:
            self.get_logger().warn(f'Lost the Nucleo on {self.port}: {reason}. Reconnecting.')
            self.receiving = False
        self.close_port()

    def close_port(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except (serial.SerialException, OSError):
                pass
            self.ser = None

    # ---- receive --------------------------------------------------------------

    def poll(self):
        if self.ser is None:
            return
        try:
            n = self.ser.in_waiting
            data = self.ser.read(n) if n else b''
        except (serial.SerialException, OSError) as e:
            self.drop_link(str(e))
            return
        if not data:
            return
        self.last_rx = self.now_s()
        if not self.receiving:
            self.receiving = True
            self.waiting_logged = False
            self.get_logger().info(f'Receiving data from {self.port}')
        stamp = self.get_clock().now().to_msg()
        for frame in self.parser.feed(data):
            self.on_frame(frame, stamp)

    def on_frame(self, frame, stamp):
        self.counts[frame.type] = self.counts.get(frame.type, 0) + 1
        self.seq.update(frame.seq)
        try:
            msg = fs.decode(frame)
        except ValueError as e:
            self.bad_payload += 1
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)
            return
        if msg is None:
            self.unknown += 1
            return
        if frame.type == fs.MSG_IMU:
            self.publish_imu(msg, stamp)
        elif frame.type == fs.MSG_TERRAIN:
            name = fs.TERRAIN_NAMES.get(msg.terrain, f'class_{msg.terrain}')
            self.last_terrain = (name, msg.confidence)
            self.terrain_pub.publish(String(data=name))
            self.conf_pub.publish(Float32(data=msg.confidence / 100.0))
            self.vib_pub.publish(Float32(data=msg.vib_rms_mg / 1000.0))
        elif frame.type == fs.MSG_EVENT:
            name = fs.EVENT_NAMES.get(msg.event, f'event_{msg.event}')
            self.event_pub.publish(String(data=name))
            if name in WARN_EVENTS:
                self.get_logger().warn(f'Event from Nucleo: {name}')
            else:
                self.get_logger().info(f'Event from Nucleo: {name}')
        elif frame.type == fs.MSG_HEARTBEAT:
            self.on_flags(msg.flags)

    def publish_imu(self, d, stamp):
        m = Imu()
        m.header.stamp = stamp
        m.header.frame_id = self.frame_id
        m.orientation_covariance[0] = -1.0          # no orientation estimate in this message
        m.linear_acceleration.x = fs.accel_to_ms2(d.ax)
        m.linear_acceleration.y = fs.accel_to_ms2(d.ay)
        m.linear_acceleration.z = fs.accel_to_ms2(d.az)
        m.angular_velocity.x = fs.gyro_to_rads(d.gx)
        m.angular_velocity.y = fs.gyro_to_rads(d.gy)
        m.angular_velocity.z = fs.gyro_to_rads(d.gz)
        self.imu_pub.publish(m)

    def on_flags(self, flags):
        if flags == self.flags:
            return
        names = ', '.join(fs.flag_names(flags)) or 'no flags'
        bad = (flags & fs.FLAG_SAFETY_STOP) or not (flags & fs.FLAG_IMU_OK)
        if bad:
            self.get_logger().warn(f'Nucleo status: {names}')
        else:
            self.get_logger().info(f'Nucleo status: {names}')
        self.flags = flags

    # ---- transmit -------------------------------------------------------------

    def on_cmd_vel(self, msg):
        """Turn a Twist into per-mille wheel duty (open loop: there are no encoders)."""
        half = msg.angular.z * self.wheel_base / 2.0
        left, right = msg.linear.x - half, msg.linear.x + half
        biggest = max(abs(left), abs(right))
        if biggest > self.max_speed:               # scale both, keep the curve
            left *= self.max_speed / biggest
            right *= self.max_speed / biggest
        k = fs.MOTOR_MAX / self.max_speed
        self.cmd = (int(round(left * k)), int(round(right * k)))
        self.cmd_time = self.now_s()

    def send_motor(self):
        fresh = self.cmd_time is not None and self.now_s() - self.cmd_time <= self.cmd_timeout
        self.write_motor(*(self.cmd if fresh else (0, 0)))

    def write_motor(self, left, right):
        if self.ser is None:
            return
        frame = fs.pack(fs.MSG_MOTOR, self.tx_seq, left, right)
        self.tx_seq = (self.tx_seq + 1) & 0xFF
        try:
            self.ser.write(frame)
        except serial.SerialTimeoutException:
            self.tx_dropped += 1
        except (serial.SerialException, OSError) as e:
            self.drop_link(str(e))

    # ---- diagnostics ----------------------------------------------------------

    def report(self):
        if not self.receiving:
            return
        rates = []
        for t, name in fs.MSG_NAMES.items():
            n = self.counts.get(t, 0) - self.prev_counts.get(t, 0)
            if n:
                rates.append(f'{name} {n / self.stats_period:.1f}/s')
        self.prev_counts = dict(self.counts)
        p = self.parser
        terrain = f'{self.last_terrain[0]} {self.last_terrain[1]}%' if self.last_terrain else '-'
        self.get_logger().info(
            f'{", ".join(rates) or "no frames"} | last terrain {terrain} | '
            f'crc errors {p.crc_errors}, lost {self.seq.lost}, '
            f'skipped bytes {p.discarded}')

    def shutdown(self):
        """Send one stop command and release the port."""
        self.write_motor(0, 0)
        self.close_port()


def main(args=None):
    rclpy.init(args=args)
    node = Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
