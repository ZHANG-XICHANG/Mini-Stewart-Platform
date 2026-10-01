import math
import struct
import threading
import time
import unittest

from pca9685_transport import PoseSender, pose_packet
from stewart_kinematics import StewartKinematics


class FakeSerial:
    in_waiting = 0

    def __init__(self, short_write=False):
        self.writes = []
        self.sent = threading.Event()
        self.short_write = short_write

    def write(self, data):
        self.writes.append((time.monotonic(), data))
        self.sent.set()
        return len(data) - 1 if self.short_write else len(data)


class TransportTests(unittest.TestCase):
    def test_packet_decodes_like_receiver(self):
        solver = StewartKinematics()
        for pose in ([0]*6, [2, -1, 1, 2, -2, 1]):
            angles = solver.calculate_servo_angles(pose[:3], [math.radians(v) for v in pose[3:]])[0]
            packet = pose_packet(solver, pose)
            self.assertEqual(len(packet), 12)
            for value, expected in zip(struct.unpack('>6H', packet), angles):
                decoded = (value - 2047) * 120 / 4095
                self.assertAlmostEqual(decoded, math.degrees(expected), delta=120/4095)

    def test_invalid_pose_rejected(self):
        for pose in ([0]*5, [0,0,10000,0,0,0], [math.nan]*6, [math.inf]*6):
            with self.subTest(pose=pose), self.assertRaises(ValueError):
                pose_packet(StewartKinematics(), pose)

    def test_repeats_latest_target_and_stops(self):
        port = FakeSerial()
        sender = PoseSender(port)
        try:
            self.assertEqual(port.writes, [])
            sender.set_pose([0]*6)
            self.assertTrue(port.sent.wait(1))
            # Updates replace targets, rather than queuing 100 serial packets.
            for _ in range(100):
                sender.set_pose([2,0,0,0,0,0])
            time.sleep(0.08)
            self.assertEqual(port.writes[-1][1], pose_packet(StewartKinematics(), [2,0,0,0,0,0]))
            times = [t for t, _ in port.writes]
            self.assertTrue(all(b-a >= 0.019 for a,b in zip(times,times[1:])))
            with self.assertRaises(ValueError):
                sender.set_pose([0,0,10000,0,0,0])
            count = len(port.writes)
            time.sleep(0.05)
            self.assertEqual(len(port.writes), count)
        finally:
            sender.close()
        count = len(port.writes)
        time.sleep(0.03)
        self.assertEqual(len(port.writes), count)

    def test_short_write_halts_sender(self):
        port = FakeSerial(short_write=True)
        sender = PoseSender(port)
        try:
            sender.set_pose([0]*6)
            self.assertTrue(port.sent.wait(1))
            sender._thread.join(1)
            self.assertIsNotNone(sender.error)
            with self.assertRaises(OSError):
                sender.set_pose([0]*6)
            self.assertEqual(len(port.writes), 1)
        finally:
            sender.close()


if __name__ == '__main__':
    unittest.main()
