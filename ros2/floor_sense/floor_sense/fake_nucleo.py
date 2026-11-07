"""
Pretend to be the Nucleo so the Pi side can be built and tested without hardware.

Creates a virtual serial port, links it at /tmp/floorsense_fake and streams the same
frames the real firmware will send. Point the bridge at that path instead of
/dev/ttyACM0. Every number it sends is synthetic: it exercises the protocol and the
ROS side, it says nothing about how well the real classifier will do.

    ros2 run floor_sense fake_nucleo
    ros2 run floor_sense fake_nucleo --accuracy 0.7 --collision-period 10
"""

import argparse
import math
import os
import random
import select
import signal
import sys
import time
import tty

from floor_sense import protocol as fs

SURFACES = (1, 2, 3)                          # tile, carpet, grass
VIBRATION_G = {1: 0.05, 2: 0.03, 3: 0.25}     # made-up RMS vibration per surface
TERRAIN_PERIOD = 0.128                        # one classifier window per 128 samples at 1 kHz
HEARTBEAT_PERIOD = 1.0
CMD_TIMEOUT = 0.5
MAX_TURN_DPS = 90.0                           # yaw rate at full opposite wheel commands
STATUS_PERIOD = 5.0


class FakeNucleo:
    """Generates frames on a schedule and answers motor commands like the firmware will."""

    def __init__(self, args):
        self.args = args
        self.rng = random.Random(args.seed)
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)                # no echo, no newline translation, no ^C
        os.set_blocking(self.master, False)
        self.port = os.ttyname(self.slave)
        self.out = bytearray()
        self.dropped = 0
        self.seq = 0
        self.parser = fs.FrameParser()
        self.sent = {t: 0 for t in fs.MSG_NAMES}
        self.motor_frames = 0
        self.cmd = (0, 0)
        self.cmd_time = None
        self.safety_stop = False
        self.timed_out = True
        self.window = []                      # recent vertical vibration, in g
        self.spike = 0
        self.t0 = time.monotonic()

    # ---- output ---------------------------------------------------------------

    def send(self, msg_type, *fields):
        frame = fs.pack(msg_type, self.seq, *fields)
        self.seq = (self.seq + 1) & 0xFF
        if len(self.out) + len(frame) > 8192:  # nobody reading: drop whole frames
            self.dropped += 1
            return
        self.out += frame
        self.sent[msg_type] += 1

    def flush(self):
        while self.out:
            try:
                n = os.write(self.master, self.out)
            except BlockingIOError:
                return
            del self.out[:n]

    # ---- input ----------------------------------------------------------------

    def read_commands(self, now):
        try:
            data = os.read(self.master, 1024)
        except (BlockingIOError, OSError):
            return
        for frame in self.parser.feed(data):
            msg = fs.decode(frame)
            if frame.type != fs.MSG_MOTOR or msg is None:
                print(f'ignored frame type 0x{frame.type:02x}')
                continue
            self.motor_frames += 1
            cmd = (max(-fs.MOTOR_MAX, min(fs.MOTOR_MAX, msg.left)),
                   max(-fs.MOTOR_MAX, min(fs.MOTOR_MAX, msg.right)))
            self.cmd_time = now
            if self.timed_out:
                self.timed_out = False
                print('motor commands arriving')
            if self.safety_stop and cmd == (0, 0):
                self.safety_stop = False
                print('safety stop cleared by a zero command')
            if cmd != self.cmd:
                print(f'motor command  left {cmd[0]:+5d}  right {cmd[1]:+5d}')
                self.cmd = cmd

    def wheels(self, now):
        """Return the duty the firmware would actually apply right now."""
        if self.cmd_time is None or now - self.cmd_time > CMD_TIMEOUT:
            if not self.timed_out:
                self.timed_out = True
                print('no motor command for 500 ms: wheels stopped')
            return (0, 0)
        return (0, 0) if self.safety_stop else self.cmd

    # ---- synthetic sensor -----------------------------------------------------

    def surface(self, t):
        return SURFACES[int(t // self.args.surface_period) % len(SURFACES)]

    def imu_sample(self, t, now):
        rng = self.rng
        sigma = VIBRATION_G[self.surface(t)]
        left, right = self.wheels(now)
        vib = rng.gauss(0.0, sigma)
        self.window.append(vib)
        del self.window[:-64]
        ax = rng.gauss(0.0, sigma * 0.5)
        if self.spike:
            ax -= 2.5                         # sharp deceleration of a collision
            self.spike -= 1
        g = fs.ACCEL_LSB_PER_G
        d = fs.GYRO_LSB_PER_DPS
        yaw = (right - left) / (2 * fs.MOTOR_MAX) * MAX_TURN_DPS
        raw = (ax * g, rng.gauss(0.0, sigma * 0.5) * g, (1.0 + vib) * g,
               rng.gauss(0.0, 1.5) * d, rng.gauss(0.0, 1.5) * d,
               (yaw + rng.gauss(0.0, 0.8)) * d)
        clamp = [max(-32768, min(32767, int(round(v)))) for v in raw]
        self.send(fs.MSG_IMU, self.t_us(t), *clamp)

    def terrain(self, t):
        true = self.surface(t)
        if self.rng.random() < self.args.accuracy:
            guess, conf = true, self.rng.randint(75, 99)
        else:
            guess = self.rng.choice([s for s in SURFACES if s != true])
            conf = self.rng.randint(40, 70)
        w = self.window or [0.0]
        rms_mg = int(1000 * math.sqrt(sum(v * v for v in w) / len(w)))
        self.send(fs.MSG_TERRAIN, self.t_us(t), guess, conf, min(rms_mg, 0xFFFF))

    def heartbeat(self, t, now):
        flags = fs.FLAG_IMU_OK | fs.FLAG_MODEL_OK
        self.wheels(now)
        if self.timed_out:
            flags |= fs.FLAG_CMD_TIMEOUT
        if self.safety_stop:
            flags |= fs.FLAG_SAFETY_STOP
        if not self.timed_out and not self.safety_stop:
            flags |= fs.FLAG_MOTORS_ARMED
        self.send(fs.MSG_HEARTBEAT, self.t_us(t), flags)

    def collision(self, t):
        self.spike = 3
        self.safety_stop = True
        self.send(fs.MSG_EVENT, self.t_us(t), 1)
        print('collision sent: wheels stopped until a zero command arrives')

    def t_us(self, t):
        return int(t * 1e6) & 0xFFFFFFFF

    # ---- main loop ------------------------------------------------------------

    def run(self):
        a = self.args
        imu_dt = 1.0 / a.imu_rate
        due = {'imu': 0.0, 'terrain': TERRAIN_PERIOD, 'hb': 0.0, 'status': STATUS_PERIOD,
               'hit': a.collision_period if a.collision_period > 0 else math.inf}
        last_surface = None
        while True:
            now = time.monotonic()
            t = now - self.t0
            self.read_commands(now)
            if self.surface(t) != last_surface:
                last_surface = self.surface(t)
                print(f'surface is now {fs.TERRAIN_NAMES[last_surface]}')
            while due['imu'] <= t:
                self.imu_sample(due['imu'], now)
                due['imu'] += imu_dt
            if due['terrain'] <= t:
                self.terrain(t)
                due['terrain'] += TERRAIN_PERIOD
            if due['hb'] <= t:
                self.heartbeat(t, now)
                due['hb'] += HEARTBEAT_PERIOD
            if due['hit'] <= t:
                self.collision(t)
                due['hit'] += a.collision_period
            if due['status'] <= t:
                s = self.sent
                print(f'sent imu {s[fs.MSG_IMU]}, terrain {s[fs.MSG_TERRAIN]}, '
                      f'heartbeat {s[fs.MSG_HEARTBEAT]}, event {s[fs.MSG_EVENT]} | '
                      f'motor frames received {self.motor_frames} | '
                      f'dropped (nobody reading) {self.dropped}')
                due['status'] += STATUS_PERIOD
            self.flush()
            wait = max(0.0, min(due.values()) - (time.monotonic() - self.t0))
            select.select([self.master], [], [], min(wait, 0.01))

    def close(self):
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass


def main(argv=None):
    p = argparse.ArgumentParser(description='Fake Floor Sense Nucleo on a virtual serial port.')
    p.add_argument('--link', default='/tmp/floorsense_fake',
                   help='symlink to create for the port (default %(default)s)')
    p.add_argument('--imu-rate', type=float, default=100.0, help='IMU frames per second')
    p.add_argument('--accuracy', type=float, default=0.9,
                   help='share of classifier windows that are right (default %(default)s)')
    p.add_argument('--surface-period', type=float, default=10.0,
                   help='seconds on each surface before moving to the next')
    p.add_argument('--collision-period', type=float, default=25.0,
                   help='seconds between fake collisions, 0 for none')
    p.add_argument('--seed', type=int, default=None, help='random seed for repeatable runs')
    args = p.parse_args(argv)

    fake = FakeNucleo(args)
    if os.path.islink(args.link):
        os.remove(args.link)
    elif os.path.exists(args.link):
        sys.exit(f'{args.link} exists and is not a symlink; remove it or pass --link')
    os.symlink(fake.port, args.link)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    signal.signal(signal.SIGINT, signal.default_int_handler)   # even if started in background
    print(f'fake Nucleo on {fake.port}, linked at {args.link}. Ctrl+C to stop.', flush=True)
    try:
        fake.run()
    except KeyboardInterrupt:
        pass
    finally:
        if os.path.islink(args.link) and os.readlink(args.link) == fake.port:
            os.remove(args.link)
        fake.close()
        print('\nfake Nucleo stopped')


if __name__ == '__main__':
    main()
