"""
Floor Sense dashboard: one web page to watch, drive and record the robot.

Serves http://floorpi.local:8080 from the Pi. The page receives live data over
Server-Sent Events and sends drive, stop and record requests as small JSON POSTs.
Web threads only change shared state; the ROS timer is the only thing that
publishes cmd_vel and the only thing that opens or closes recording files.

    ros2 run floor_sense dashboard
    ros2 launch floor_sense robot.launch.py          (bridge + dashboard together)

Recordings go to ~/floor-sense-data as <surface>_<power>pct_<time>.csv plus a .json
with the details. Columns: t_s, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps.
/api/export cuts the finished recordings into fixed-length windows, one CSV per
surface, in the layout NanoEdge AI Studio imports (one window per line).
"""

import collections
import csv
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import math
import os
from pathlib import Path
import re
import shutil
import socket
import threading
import time
from urllib.parse import parse_qs, unquote
import zipfile

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String

G = 9.80665
WEB_DIR = Path(__file__).resolve().parent / 'web'
STATUS_TIMEOUT = 2.5      # s without a heartbeat before the Nucleo counts as disconnected
DRIVE_HOLD = 0.3          # s a drive request from the page stays valid
SETTLE_TIME = 1.0         # s of driving before a recording starts writing samples
SENSOR_GAP = 0.5          # s without IMU samples that ends a recording
STOP_EVENTS = {'collision', 'stuck', 'lifted'}
IMU_KEEP = 1500           # samples kept for the page's charts (15 s at 100 Hz)
EVENT_KEEP = 50
FILE_NAME = re.compile(r'^[A-Za-z0-9_.-]+\.(csv|json)$')
FONT_NAME = re.compile(r'^[a-z0-9-]+\.woff2$')
EXPORT_AXES = {'acc': ['ax_g', 'ay_g', 'az_g'], 'accz': ['az_g'],
               'all': ['ax_g', 'ay_g', 'az_g', 'gx_dps', 'gy_dps', 'gz_dps']}
EXPORT_WINDOWS = (64, 128, 256, 512, 1024)


def pi_health(folder):
    """Return CPU temperature, load, memory and free disk, each None if unreadable."""
    h = {'temp_c': None, 'load': None, 'cores': os.cpu_count(), 'mem_pct': None,
         'disk_free_gb': None}
    try:
        h['temp_c'] = round(int(Path('/sys/class/thermal/thermal_zone0/temp')
                                .read_text()) / 1000.0, 1)
    except (OSError, ValueError):
        pass
    try:
        h['load'] = round(os.getloadavg()[0], 2)
    except OSError:
        pass
    try:
        info = dict(line.split(':', 1) for line in
                    Path('/proc/meminfo').read_text().splitlines() if ':' in line)
        total = int(info['MemTotal'].split()[0])
        avail = int(info['MemAvailable'].split()[0])
        h['mem_pct'] = round(100.0 * (total - avail) / total, 1)
    except (OSError, KeyError, ValueError):
        pass
    try:
        target = folder if folder.is_dir() else Path.home()
        h['disk_free_gb'] = round(shutil.disk_usage(target).free / 1e9, 1)
    except OSError:
        pass
    return h


def local_ip():
    """Return this machine's address on the local network (no packets are sent)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))
        return s.getsockname()[0]
    except OSError:
        return '127.0.0.1'
    finally:
        s.close()


class Dashboard(Node):
    """Collects robot state for the page, drives on request, records labelled runs."""

    def __init__(self):
        super().__init__('floor_sense_dashboard')
        self.http_port = self.declare_parameter('http_port', 8080).value
        self.http_host = self.declare_parameter('http_host', '0.0.0.0').value
        self.data_dir = Path(self.declare_parameter(
            'data_dir', '~/floor-sense-data').value).expanduser()
        self.max_speed = self.declare_parameter('max_speed', 0.20).value
        self.wheel_base = self.declare_parameter('wheel_base', 0.15).value

        self.lock = threading.Lock()
        self.closing = False
        self.imu = collections.deque(maxlen=IMU_KEEP)      # (seq, t, ax..az in g, gx..gz dps)
        self.imu_seq = 0
        self.imu_times = collections.deque(maxlen=400)
        self.flags = []
        self.status_time = None
        self.terrain = None
        self.confidence = None
        self.vibration = None
        self.votes = collections.deque(maxlen=5)
        self.events = collections.deque(maxlen=EVENT_KEEP)  # (seq, 'HH:MM:SS', name)
        self.event_seq = 0
        self.link = {}
        self.drive = None             # (lin, ang, speed) from the page, each -1..1 / 0..1
        self.drive_time = 0.0
        self.stop_pending = False
        self.sent_motion = False
        self.rec = None
        self.rec_file = None
        self.last_rec = None
        self.files_version = 0
        self.health = {}
        self.health_time = 0.0

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Imu, 'imu/data_raw', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(String, 'proprio/terrain', self.on_terrain, 10)
        self.create_subscription(Float32, 'proprio/confidence', self.on_confidence, 10)
        self.create_subscription(Float32, 'proprio/vibration', self.on_vibration, 10)
        self.create_subscription(String, 'proprio/event', self.on_event, 10)
        self.create_subscription(String, 'proprio/status', self.on_status, 10)
        self.create_subscription(String, 'proprio/link', self.on_link, 10)
        self.create_timer(0.05, self.tick)

        handler = type('BoundHandler', (Handler,), {'node': self})
        try:
            self.server = ThreadingHTTPServer((self.http_host, self.http_port), handler)
        except OSError as e:
            self.get_logger().error(f'Cannot start the web page on port {self.http_port}: {e}. '
                                    'Is another dashboard already running?')
            raise
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        host = socket.gethostname()
        self.get_logger().info(
            f'Dashboard ready: http://{host}.local:{self.http_port}  '
            f'(phone: http://{local_ip()}:{self.http_port})')

    # ---- ROS callbacks (executor thread) ------------------------------------

    def on_imu(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        a, w = m.linear_acceleration, m.angular_velocity
        row = (round(t, 4), round(a.x / G, 4), round(a.y / G, 4), round(a.z / G, 4),
               round(math.degrees(w.x), 2), round(math.degrees(w.y), 2),
               round(math.degrees(w.z), 2))
        with self.lock:
            self.imu_seq += 1
            self.imu.append((self.imu_seq,) + row)
            self.imu_times.append(time.monotonic())
            rec = self.rec
            if rec and rec['phase'] == 'log' and self.rec_file:
                if rec['t0'] is None:
                    rec['t0'] = t
                rel = t - rec['t0']
                self.rec_file.write(f'{rel:.4f},' + ','.join(str(v) for v in row[1:]) + '\n')
                rec['samples'] += 1
                rec['t_last'] = rel

    def on_terrain(self, m):
        with self.lock:
            self.terrain = m.data
            self.votes.append(m.data)

    def on_confidence(self, m):
        with self.lock:
            self.confidence = m.data

    def on_vibration(self, m):
        with self.lock:
            self.vibration = m.data

    def on_event(self, m):
        with self.lock:
            self.event_seq += 1
            self.events.append((self.event_seq, datetime.now().strftime('%H:%M:%S'), m.data))
            if self.rec and m.data in STOP_EVENTS:
                self.rec['stop_reason'] = m.data
                self.stop_pending = True

    def on_status(self, m):
        with self.lock:
            self.flags = [f for f in m.data.split(',') if f]
            self.status_time = time.monotonic()
            if self.rec and 'safety_stop' in self.flags:
                self.rec['stop_reason'] = 'safety stop'

    def on_link(self, m):
        try:
            link = json.loads(m.data)
        except ValueError:
            return
        with self.lock:
            self.link = link

    # ---- timer: the only place that publishes cmd_vel and touches files -----

    def tick(self):
        now = time.monotonic()
        with self.lock:
            if self.rec:
                self.advance_recording(now)
            if self.stop_pending:
                self.stop_pending = False
                self.sent_motion = False
                self.drive = None
                self.publish(0.0, 0.0)
                return
            rec = self.rec
            if rec and rec['drive']:
                self.publish(rec['power'] / 100.0 * self.max_speed, 0.0)
                self.sent_motion = True
            elif self.drive and now - self.drive_time < DRIVE_HOLD:
                lin, ang, speed = self.drive
                self.publish(lin * speed * self.max_speed,
                             ang * speed * self.max_speed / self.wheel_base)
                self.sent_motion = True
            elif self.sent_motion:        # page went quiet mid-drive: stop now, don't wait
                self.sent_motion = False
                self.drive = None
                self.publish(0.0, 0.0)

    def publish(self, lin, ang):
        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        self.cmd_pub.publish(t)

    def advance_recording(self, now):
        rec = self.rec
        if rec['stop_reason']:
            self.finish_recording(rec['stop_reason'])
        elif rec['phase'] == 'settle':
            if now - rec['start'] >= (SETTLE_TIME if rec['drive'] else 0.0):
                self.open_recording(now)
        elif rec['phase'] == 'log':
            last = self.imu_times[-1] if self.imu_times else 0.0
            if now - rec['log_start'] >= rec['duration']:
                self.finish_recording(None)
            elif now - last > SENSOR_GAP:
                self.finish_recording('sensor data stopped')
            elif self.status_time is None or now - self.status_time > STATUS_TIMEOUT:
                self.finish_recording('Nucleo disconnected')

    def open_recording(self, now):
        rec = self.rec
        self.data_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
        rec['name'] = f"{rec['surface']}_{rec['power']}pct_{stamp}"
        self.rec_file = open(self.data_dir / f"{rec['name']}.csv", 'w', encoding='utf-8')
        self.rec_file.write('t_s,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps\n')
        rec['phase'] = 'log'
        rec['log_start'] = now
        rec['started'] = datetime.now().isoformat(timespec='seconds')

    def finish_recording(self, reason):
        rec, self.rec = self.rec, None
        if rec['drive']:
            self.stop_pending = True
        if self.rec_file is None:          # stopped while still settling: nothing written
            self.last_rec = {'name': None, 'complete': False, 'reason': reason or 'stopped',
                             'samples': 0}
            return
        self.rec_file.close()
        self.rec_file = None
        csv_path = self.data_dir / f"{rec['name']}.csv"
        if rec['samples'] == 0:
            csv_path.unlink(missing_ok=True)
            self.last_rec = {'name': None, 'complete': False,
                             'reason': reason or 'no samples arrived', 'samples': 0}
            return
        dur = rec['t_last'] or 0.0
        info = {
            'file': csv_path.name, 'surface': rec['surface'], 'power_pct': rec['power'],
            'drive': rec['drive'], 'duration_target_s': rec['duration'],
            'duration_s': round(dur, 2), 'samples': rec['samples'],
            'rate_hz': round(rec['samples'] / dur, 1) if dur > 0 else None,
            'complete': reason is None, 'reason': reason, 'started': rec['started'],
            'columns': ['t_s', 'ax_g', 'ay_g', 'az_g', 'gx_dps', 'gy_dps', 'gz_dps'],
        }
        (self.data_dir / f"{rec['name']}.json").write_text(json.dumps(info, indent=2))
        self.last_rec = {'name': csv_path.name, 'complete': reason is None,
                         'reason': reason, 'samples': rec['samples']}
        self.files_version += 1
        self.get_logger().info(f"Recorded {csv_path.name}: {rec['samples']} samples"
                               + (f' (stopped early: {reason})' if reason else ''))

    # ---- requests from the page (web threads) -------------------------------

    def request_drive(self, body):
        try:
            lin = max(-1.0, min(1.0, float(body.get('lin', 0))))
            ang = max(-1.0, min(1.0, float(body.get('ang', 0))))
            speed = max(0.0, min(1.0, float(body.get('speed', 0.5))))
        except (TypeError, ValueError):
            return {'error': 'lin, ang and speed must be numbers'}
        with self.lock:
            if self.rec and self.rec['drive']:
                return {'error': 'A recording is driving the robot. Stop it first.'}
            self.drive = (lin, ang, speed)
            self.drive_time = time.monotonic()
        return {'ok': True}

    def request_stop(self, _body):
        with self.lock:
            self.drive = None
            self.stop_pending = True
            if self.rec:
                self.rec['stop_reason'] = 'stopped by you'
        return {'ok': True}

    def request_record(self, body):
        surface = re.sub(r'[^a-z0-9-]+', '-', str(body.get('surface', '')).lower())
        surface = surface.strip('-')[:24]
        try:
            power = int(body.get('power', 0))
            duration = float(body.get('duration', 0))
        except (TypeError, ValueError):
            return {'error': 'Power and duration must be numbers.'}
        drive = bool(body.get('drive', True))
        if not surface:
            return {'error': 'Choose or type a surface name.'}
        if not 10 <= power <= 100:
            return {'error': 'Motor power must be between 10 and 100 %.'}
        if not 2 <= duration <= 60:
            return {'error': 'Duration must be between 2 and 60 seconds.'}
        now = time.monotonic()
        with self.lock:
            if self.rec:
                return {'error': 'A recording is already running.'}
            if self.status_time is None or now - self.status_time > STATUS_TIMEOUT:
                return {'error': 'No heartbeat from the Nucleo. Check the bridge and the USB '
                                 'cable.'}
            if not self.imu_times or now - self.imu_times[-1] > SENSOR_GAP:
                return {'error': 'No sensor data. Wire the MPU9250 and flash the sensor '
                                 'firmware first.'}
            if 'safety_stop' in self.flags:
                return {'error': 'Safety stop is on. Press Stop to clear it.'}
            self.drive = None
            self.rec = {'surface': surface, 'power': power, 'duration': duration,
                        'drive': drive, 'phase': 'settle',
                        'start': now, 'log_start': None, 'samples': 0, 't0': None,
                        't_last': None, 'stop_reason': None, 'name': None,
                        'started': None}
            self.last_rec = None
        return {'ok': True}

    def request_record_stop(self, _body):
        with self.lock:
            if not self.rec:
                return {'error': 'Nothing is recording.'}
            self.rec['stop_reason'] = 'stopped by you'
        return {'ok': True}

    # ---- data for the page ----------------------------------------------------

    def snapshot(self, imu_after, event_after):
        """Return everything new since the given cursors, plus current state."""
        now = time.monotonic()
        with self.lock:
            imu = [list(r[1:]) for r in self.imu if r[0] > imu_after]
            events = [list(e) for e in self.events if e[0] > event_after]
            recent = [t for t in self.imu_times if now - t <= 2.0]
            rate = (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) > 2 else 0.0
            connected = self.status_time is not None and now - self.status_time <= STATUS_TIMEOUT
            rec = None
            if self.rec:
                r = self.rec
                elapsed = now - (r['log_start'] or r['start'])
                rec = {'surface': r['surface'], 'power': r['power'], 'drive': r['drive'],
                       'duration': r['duration'], 'phase': r['phase'],
                       'elapsed': round(elapsed, 1), 'samples': r['samples']}
            state = {
                'nucleo': connected, 'flags': self.flags if connected else [],
                'imu_rate': round(rate, 1), 'terrain': self.terrain,
                'confidence': self.confidence, 'vibration': self.vibration,
                'votes': list(self.votes), 'imu': imu, 'events': events,
                'link': self.link, 'rec': rec, 'last_rec': self.last_rec,
                'files_version': self.files_version,
                'driving': bool(self.sent_motion),
                'config': {'max_speed': self.max_speed, 'settle_s': SETTLE_TIME},
            }
            cursors = (self.imu_seq, self.event_seq)
            refresh = now - self.health_time > 2.0
            if refresh:
                self.health_time = now
        if refresh:
            self.health = pi_health(self.data_dir)
        state['pi'] = self.health
        return state, cursors

    def list_files(self):
        if not self.data_dir.is_dir():
            return {'files': [], 'folder': str(self.data_dir)}
        files = []
        for meta in sorted(self.data_dir.glob('*.json'), reverse=True)[:100]:
            try:
                info = json.loads(meta.read_text())
            except (OSError, ValueError):
                continue
            files.append({k: info.get(k) for k in (
                'file', 'surface', 'power_pct', 'drive', 'duration_s', 'samples', 'rate_hz',
                'complete', 'reason', 'started')})
        return {'files': files, 'folder': str(self.data_dir)}

    def request_delete(self, body):
        name = str(body.get('file', ''))
        if not FILE_NAME.match(name) or not name.endswith('.csv'):
            return {'error': 'Not a recording file name.'}
        with self.lock:
            if self.rec and self.rec.get('name') and name.startswith(self.rec['name']):
                return {'error': 'That recording is still running.'}
            removed = 0
            for path in (self.data_dir / name, self.data_dir / (name[:-4] + '.json')):
                if path.is_file():
                    path.unlink()
                    removed += 1
            if not removed:
                return {'error': 'No such recording.'}
            self.files_version += 1
        self.get_logger().info(f'Deleted {name}')
        return {'ok': True}

    def build_export(self, query):
        """Cut finished recordings into NanoEdge windows; return (summary, zip bytes)."""
        try:
            window = int(query.get('window', ['256'])[0])
            hop = int(query.get('hop', [str(window)])[0])
        except ValueError:
            return {'error': 'window and hop must be whole numbers'}, None
        axes = query.get('axes', ['acc'])[0]
        partial = query.get('partial', ['0'])[0] == '1'
        if window not in EXPORT_WINDOWS or not 1 <= hop <= window or axes not in EXPORT_AXES:
            return {'error': 'Unsupported window, overlap or axes.'}, None
        cols = EXPORT_AXES[axes]
        lines = collections.defaultdict(list)
        rates = []
        skipped = []
        if self.data_dir.is_dir():
            for meta in sorted(self.data_dir.glob('*.json')):
                try:
                    info = json.loads(meta.read_text())
                    if not info.get('complete') and not partial:
                        continue
                    with open(self.data_dir / info['file'], newline='') as f:
                        rows = [[float(r[c]) for c in cols] for r in csv.DictReader(f)]
                except (OSError, ValueError, KeyError):
                    skipped.append(meta.stem)
                    continue
                if info.get('rate_hz'):
                    rates.append(info['rate_hz'])
                if len(rows) < window:
                    skipped.append(meta.stem)
                    continue
                for start in range(0, len(rows) - window + 1, hop):
                    flat = (v for row in rows[start:start + window] for v in row)
                    lines[info['surface']].append(','.join(f'{v:.5g}' for v in flat))
        summary = {'window': window, 'hop': hop, 'axes': cols,
                   'classes': {k: len(v) for k, v in sorted(lines.items())},
                   'skipped': skipped,
                   'rate_hz': round(sum(rates) / len(rates), 1) if rates else None}
        if not lines:
            summary['error'] = (f'No recording has {window} samples yet. Record longer runs '
                                'or pick a shorter window.')
            return summary, None
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, 'w', zipfile.ZIP_DEFLATED) as z:
            for surface, rows in lines.items():
                z.writestr(f'{surface}.csv', '\n'.join(rows) + '\n')
            z.writestr('README.txt', '\n'.join([
                'Floor Sense export for NanoEdge AI Studio (N-class classification).',
                f'Each line is one window of {window} samples; each sample is '
                f'{", ".join(cols)} in that order.',
                f'Windows start every {hop} samples. Sample rate about '
                f'{summary["rate_hz"]} Hz.',
                'Import one file per class. Train only on data recorded at the rate the',
                'classifier will run at on the Nucleo.',
                '', 'Windows per class:'] +
                [f'  {k}: {n}' for k, n in summary['classes'].items()]) + '\n')
        return summary, buf.getvalue()

    def close(self):
        """Stop the web server, finish any recording and send one stop."""
        with self.lock:
            self.closing = True
            if self.rec:
                self.rec['stop_reason'] = 'dashboard closed'
                self.advance_recording(time.monotonic())
            self.publish(0.0, 0.0)
        self.server.shutdown()
        self.server.server_close()


class Handler(BaseHTTPRequestHandler):
    """HTTP endpoints for the page. `node` is set on a subclass by Dashboard."""

    node = None
    server_version = 'FloorSense/1'

    def log_message(self, fmt, *args):
        pass                                   # keep the ROS log readable

    def do_GET(self):
        path = self.path.split('?', 1)[0]
        if path in ('/', '/index.html'):
            self.send_file(WEB_DIR / 'index.html', 'text/html; charset=utf-8')
        elif path == '/api/stream':
            self.stream()
        elif path == '/api/files':
            self.send_json(self.node.list_files())
        elif path == '/api/export':
            self.export()
        elif path.startswith('/fonts/') and FONT_NAME.match(path[len('/fonts/'):]):
            self.send_file(WEB_DIR / 'fonts' / path[len('/fonts/'):], 'font/woff2', cache=True)
        elif path.startswith('/data/'):
            self.download(unquote(path[len('/data/'):]))
        else:
            self.send_json({'error': 'not found'}, 404)

    def do_POST(self):
        routes = {'/api/drive': self.node.request_drive, '/api/stop': self.node.request_stop,
                  '/api/record/start': self.node.request_record,
                  '/api/record/stop': self.node.request_record_stop,
                  '/api/delete': self.node.request_delete}
        action = routes.get(self.path.split('?', 1)[0])
        if action is None:
            self.send_json({'error': 'not found'}, 404)
            return
        try:
            length = min(int(self.headers.get('Content-Length') or 0), 10000)
            body = json.loads(self.rfile.read(length) or b'{}')
            if not isinstance(body, dict):
                raise ValueError
        except ValueError:
            self.send_json({'error': 'Request body must be a JSON object.'}, 400)
            return
        result = action(body)
        self.send_json(result, 400 if 'error' in result else 200)

    def send_json(self, obj, status=200):
        data = json.dumps(obj).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def send_file(self, path, ctype, cache=False):
        try:
            data = path.read_bytes()
        except OSError:
            self.send_json({'error': f'{path.name} is missing from the package'}, 500)
            return
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'max-age=86400' if cache else 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def download(self, name):
        path = self.node.data_dir / name
        if not FILE_NAME.match(name) or not path.is_file():
            self.send_json({'error': 'no such recording'}, 404)
            return
        data = path.read_bytes()
        self.send_response(200)
        self.send_header('Content-Type', 'text/csv' if name.endswith('.csv')
                         else 'application/json')
        self.send_header('Content-Disposition', f'attachment; filename="{name}"')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def export(self):
        query = parse_qs(self.path.split('?', 1)[1] if '?' in self.path else '')
        summary, data = self.node.build_export(query)
        if data is None or query.get('dry', ['0'])[0] == '1':
            self.send_json(summary, 400 if data is None and 'error' in summary else 200)
            return
        stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
        self.send_response(200)
        self.send_header('Content-Type', 'application/zip')
        self.send_header('Content-Disposition',
                         f'attachment; filename="floorsense_nanoedge_{stamp}.zip"')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def stream(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Accel-Buffering', 'no')
        self.end_headers()
        imu_after = event_after = 0
        try:
            while not self.node.closing:
                state, (imu_after, event_after) = self.node.snapshot(imu_after, event_after)
                payload = json.dumps(state, separators=(',', ':'))
                self.wfile.write(f'data: {payload}\n\n'.encode())
                self.wfile.flush()
                time.sleep(0.1)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass


def main(args=None):
    rclpy.init(args=args)
    node = Dashboard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
