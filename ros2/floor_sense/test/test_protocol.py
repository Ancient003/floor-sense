"""Tests for the Floor Sense serial protocol. Run: python3 -m pytest test/test_protocol.py."""

import random

from floor_sense import protocol as fs
import pytest


def test_crc_check_value():
    assert fs.crc16(b'123456789') == 0x29B1


def test_frame_sizes():
    sizes = {t: len(fs.pack(t, 0, *([0] * len(kind._fields))))
             for t, (_, kind) in fs.LAYOUTS.items()}
    assert sizes == {fs.MSG_IMU: 23, fs.MSG_TERRAIN: 15, fs.MSG_EVENT: 12,
                     fs.MSG_HEARTBEAT: 13, fs.MSG_MOTOR: 11}


def test_known_frame_bytes():
    # Pinned so the C header and this file can be checked against the same bytes.
    assert fs.pack(fs.MSG_MOTOR, 7, 300, -300).hex() == 'aa55100704' + '2c01d4fe' + '8c1f'


@pytest.mark.parametrize('msg_type,fields', [
    (fs.MSG_IMU, (123456, 1, -2, 8192, -32768, 32767, 655)),
    (fs.MSG_TERRAIN, (0xFFFFFFFF, 2, 87, 512)),
    (fs.MSG_EVENT, (42, 1)),
    (fs.MSG_HEARTBEAT, (1000000, fs.FLAG_IMU_OK | fs.FLAG_MODEL_OK)),
    (fs.MSG_MOTOR, (-1000, 1000)),
])
def test_round_trip(msg_type, fields):
    frames = fs.FrameParser().feed(fs.pack(msg_type, 200, *fields))
    assert len(frames) == 1
    assert frames[0].type == msg_type and frames[0].seq == 200
    assert tuple(fs.decode(frames[0])) == fields


def test_byte_by_byte_and_garbage():
    stream = (b'\x00\xaa\x13' + fs.pack(fs.MSG_EVENT, 1, 5, 1)
              + b'\xaa' + fs.pack(fs.MSG_EVENT, 2, 6, 2))
    p = fs.FrameParser()
    got = []
    for b in stream:
        got += p.feed(bytes([b]))
    assert [f.seq for f in got] == [1, 2]
    assert p.crc_errors == 0 and p.discarded == 4


def test_corrupt_frame_is_dropped_and_next_one_found():
    bad = bytearray(fs.pack(fs.MSG_IMU, 1, 0, 0, 0, 0, 0, 0, 0))
    bad[8] ^= 0x40
    good = fs.pack(fs.MSG_IMU, 2, 1, 2, 3, 4, 5, 6, 7)
    p = fs.FrameParser()
    got = p.feed(bytes(bad) + good)
    assert [f.seq for f in got] == [2]
    assert p.crc_errors == 1


def test_frame_hidden_after_truncated_frame():
    # A frame cut short by a glitch, immediately followed by a full one.
    cut = fs.pack(fs.MSG_IMU, 1, 0, 0, 0, 0, 0, 0, 0)[:9]
    good = fs.pack(fs.MSG_HEARTBEAT, 2, 99, 3)
    got = fs.FrameParser().feed(cut + good + fs.pack(fs.MSG_HEARTBEAT, 3, 100, 3))
    assert 3 in [f.seq for f in got]


def test_length_limit():
    p = fs.FrameParser()
    assert p.feed(b'\xaa\x55\x01\x00\xff' + b'\x00' * 10) == []
    assert p.bad_length == 1
    with pytest.raises(ValueError):
        fs.encode(fs.MSG_IMU, 0, b'\x00' * (fs.MAX_PAYLOAD + 1))


def test_random_noise_with_frames_inside():
    rng = random.Random(1)
    frames = [fs.pack(fs.MSG_TERRAIN, i, i, 1, 90, 30) for i in range(200)]
    stream = b''.join(bytes(rng.randrange(256) for _ in range(rng.randrange(20))) + f
                      for f in frames)
    p = fs.FrameParser()
    got = []
    i = 0
    while i < len(stream):
        n = rng.randrange(1, 40)
        got += p.feed(stream[i:i + n])
        i += n
    seqs = [f.seq for f in got if f.type == fs.MSG_TERRAIN and len(f.payload) == 8]
    assert set(range(200)) <= set(seqs)


def test_short_payload_rejected_long_payload_accepted():
    with pytest.raises(ValueError):
        fs.decode(fs.Frame(fs.MSG_IMU, 0, b'\x00' * 4))
    longer = fs.LAYOUTS[fs.MSG_EVENT][0].pack(5, 2) + b'\x01\x02'
    assert fs.decode(fs.Frame(fs.MSG_EVENT, 0, longer)) == fs.Event(5, 2)
    assert fs.decode(fs.Frame(0x7F, 0, b'')) is None


def test_seq_tracker_wraps_and_counts_loss():
    s = fs.SeqTracker()
    for seq in (254, 255, 0, 3, 4):
        s.update(seq)
    assert s.lost == 2 and s.jumps == 0
    s.update(200)
    assert s.jumps == 1 and s.lost == 2


def test_unit_conversion():
    assert fs.accel_to_ms2(8192) == pytest.approx(9.80665)
    assert fs.gyro_to_rads(655) == pytest.approx(0.17453, rel=1e-4)
    assert fs.flag_names(fs.FLAG_IMU_OK | fs.FLAG_SAFETY_STOP) == ['imu_ok', 'safety_stop']
