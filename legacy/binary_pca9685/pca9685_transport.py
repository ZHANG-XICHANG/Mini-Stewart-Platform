"""PC-side pose conversion and latest-target transport for the 12-byte receiver."""
import math
import struct
import threading

from stewart_kinematics import StewartKinematics


def pose_packet(solver, pose):
    """Use the existing visualizer's axis convention: translation mm, rotation degrees."""
    if len(pose) != 6 or not all(math.isfinite(value) for value in pose):
        raise ValueError("姿態必須包含六個有限數值")
    result = solver.calculate_servo_angles(pose[:3], [math.radians(v) for v in pose[3:]])
    if result is None:
        raise ValueError("姿態超出目前幾何模型的可達範圍")
    angles = [math.degrees(v) for v in result[0]]
    if not all(math.isfinite(v) and -60 <= v <= 60 for v in angles):
        raise ValueError("馬達角度超出 ±60°")
    values = [max(0, min(4095, int(2047 + v * 4095 / 120))) for v in angles]
    return struct.pack('>6H', *values)


class PoseSender:
    """Repeat only the latest valid target at no more than 50 Hz for receiver smoothing.

    No command is sent until set_pose succeeds. Closing stops transmission, not motor power.
    """
    def __init__(self, port):
        self.port = port
        self.solver = StewartKinematics()
        self.error = None
        self._packet = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def set_pose(self, pose):
        with self._lock:
            if self.error:
                raise OSError(self.error)
            # On invalid targets, stop repeating the previous target.
            self._packet = None
            self._packet = pose_packet(self.solver, pose)

    def _run(self):
        while not self._stop.is_set():
            try:
                with self._lock:
                    if self._packet is not None:
                        count = self.port.write(self._packet)
                        if count != len(self._packet):
                            raise OSError("串列封包未完整送出，請重新連線並重啟接收端")
                    if self.port.in_waiting:
                        self.port.reset_input_buffer()
            except Exception as exc:
                self.error = str(exc)
                return
            self._stop.wait(0.02)

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2)
        if self._thread.is_alive():
            raise OSError("串列傳送執行緒尚未停止")
