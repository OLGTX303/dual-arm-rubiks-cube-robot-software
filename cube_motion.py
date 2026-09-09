#!/usr/bin/env python3
"""Direct EMM-V5 TTL motion backend for the dual-arm Rubik's-cube robot.

This file is a drop-in replacement for ``software/cube_motion.py``.

Connection
----------
PC / Linux -> USB-TTL adapter -> shared EMM-V5 TTL bus -> motor IDs 1..4

There is no intermediate controller protocol. Every motor command in this file
is a native EMM-V5 frame ending in 0x6B. Synchronized moves are queued with the
native 0xFD sync flag and started with the native broadcast::

    00 FF 66 6B

Physical motor mapping used by my_codev1.2 / ttl_build:
    1 = right arm
    2 = right finger
    3 = left arm
    4 = left finger

The public API used by the existing software is preserved:
    DEFAULT_SERIAL_PORT
    BAUD_RATE
    cmd_zero(ser)
    cmd_enable(ser, ids, enable)
    cmd_stat(ser, id)
    cmd_get_pos(ser, id)
    cmd_wait_motion(ser, id)
    MotionCtrl(ser, *cmd_zero(...))
    MotionCtrl.two_finger_init()
    MotionCtrl.two_finger_clamp()
    MotionCtrl.motions(...)

IMPORTANT
---------
``cmd_zero`` does not drive into an end stop. It records the current encoder
positions in the PC as the safe reference, matching the v1.2 workflow. Before
starting, mechanically place both arms at the known horizontal/reference pose
and the fingers at the normal clamped pose.

Clamp overflow protection is built in. After a clamp the arm encoders are
checked against the planned cube orientation. Excess grip-induced rotation is
recovered automatically with a softer adaptive clamp and closed-loop native
TTL arm/finger correction moves.
"""

from __future__ import annotations

import logging
import os
import sys
import threading
import time
from dataclasses import dataclass
from typing import Iterable, Sequence

import serial
from serial.tools import list_ports


# ---------------------------------------------------------------------------
# Bus / motor configuration
# ---------------------------------------------------------------------------

DEFAULT_SERIAL_PORT = os.environ.get("CUBE_ROBOT_PORT", "/dev/ttyUSB0")
BAUD_RATE = int(os.environ.get("CUBE_ROBOT_BAUD", "115200"))
CHECKSUM = 0x6B

RIGHT_ARM_ID = 1
RIGHT_FINGER_ID = 2
LEFT_ARM_ID = 3
LEFT_FINGER_ID = 4
MOTOR_IDS = (RIGHT_ARM_ID, RIGHT_FINGER_ID, LEFT_ARM_ID, LEFT_FINGER_ID)
FINGER_IDS = (RIGHT_FINGER_ID, LEFT_FINGER_ID)
ARM_IDS = (RIGHT_ARM_ID, LEFT_ARM_ID)

RIGHT = False
LEFT = True
CW = True
CCW = False

# The assembled right arm has the native motor direction reversed relative to
# the robot's logical encoder-positive direction. This is the same mapping as
# motor_config.h in ttl_build.
RIGHT_ARM_DIRECTION_INVERT = False
# The right drive needs inverted command direction, but its encoder already reports the robot logical sign.
RIGHT_ARM_FEEDBACK_INVERT = True
LEFT_ARM_DIRECTION_INVERT = False

# Native position-command calibration used by ttl_build / my_codev1.2.
ARM_90_PULSES = 1600
FINGER_LIMIT_PULSES = 1600
FINGER_CLAMP_ABS = 160
FINGER_RELEASE_ABS = 220
FINGER_NO_LOAD_ABS = 1400

# cmd_zero records the current normal clamp as host coordinate zero.
JAW_CLAMP = 0
JAW_RELEASE = FINGER_RELEASE_ABS - FINGER_CLAMP_ABS       # +60
JAW_NO_LOAD = FINGER_NO_LOAD_ABS - FINGER_CLAMP_ABS      # +1240
JAW_MAX_FROM_CLAMP = FINGER_LIMIT_PULSES - FINGER_CLAMP_ABS  # +1440

# Finger motor pose includes jaw opening + signed arm/2 belt compensation.
BELT_FINGER_MAX_PULSES = 6400
POSITION_TOLERANCE_PULSES = 90
ARM_V_COLLISION_GAP_PULSES = 160

# Clamp-overflow recovery. A hard jaw clamp can mechanically drag an arm a
# number of degrees away from the planned cube orientation. Detect that encoder
# drift after every clamp, soften the grip a little, then return the arm to its
# planned position with native synchronized TTL moves.
ARM_PULSES_PER_DEG = ARM_90_PULSES / 90.0
CLAMP_OVERFLOW_TRIGGER_DEG = float(os.environ.get("CUBE_CLAMP_OVERFLOW_DEG", "4.0"))
CLAMP_OVERFLOW_SETTLE_DEG = float(os.environ.get("CUBE_CLAMP_SETTLE_DEG", "1.5"))
CLAMP_OVERFLOW_MAX_DEG = float(os.environ.get("CUBE_CLAMP_OVERFLOW_MAX_DEG", "70.0"))
CLAMP_RECOVERY_STEP_DEG = float(os.environ.get("CUBE_CLAMP_RECOVERY_STEP_DEG", "30.0"))
CLAMP_RECOVERY_BACKOFF_PULSES = int(os.environ.get("CUBE_CLAMP_BACKOFF_PULSES", "25"))
CLAMP_SOFT_MAX_PULSES = int(os.environ.get("CUBE_CLAMP_SOFT_MAX_PULSES", "45"))
CLAMP_RECOVERY_RPM = int(os.environ.get("CUBE_CLAMP_RECOVERY_RPM", "220"))
CLAMP_RECOVERY_ACCEL = int(os.environ.get("CUBE_CLAMP_RECOVERY_ACCEL", "160"))
CLAMP_RECOVERY_ATTEMPTS = int(os.environ.get("CUBE_CLAMP_RECOVERY_ATTEMPTS", "4"))
CLAMP_SETTLE_SECONDS = float(os.environ.get("CUBE_CLAMP_SETTLE_SECONDS", "0.06"))

# Realtime encoder frame 0x36 uses 65536 counts/rev. The position command uses
# 3200 pulses/rev (200 full steps * 16 microsteps).
ENCODER_COUNTS_PER_REV = 65536.0
MOTOR_PULSES_PER_REV = 3200.0

# Native EMM-V5 position acceleration values used by the firmware. The finger
# ramp is half the arm ramp in time so the 2:1 belt relationship is preserved.
ARM_POSITION_ACCEL = 240
FINGER_POSITION_ACCEL = 224

# Safe production defaults. Native EMM-V5 accepts a larger range, but these
# values keep the robot in the tested v1.2 operating region.
V_TWIST = int(os.environ.get("CUBE_TWIST_RPM", "700"))
V_FLIP = int(os.environ.get("CUBE_FLIP_RPM", "400"))
V_NO_LOAD = int(os.environ.get("CUBE_NO_LOAD_RPM", "700"))
V_FINGER = int(os.environ.get("CUBE_FINGER_RPM", "400"))
MIN_RPM = 100
MAX_RPM = 1400

# Native EMM-V5 function bytes.
FN_ENABLE = 0xF3
FN_STOP = 0xFE
FN_POSITION = 0xFD
FN_READ_POS = 0x36
FN_FLAGS = 0x3A
FN_HEALTH = 0x3B
FN_CLEAR_STALL = 0x0E

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(name)-12s | %(levelname)-8s | %(filename)s:%(lineno)d | %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)


class MotionError(RuntimeError):
    pass


@dataclass(frozen=True)
class _Move:
    motor_id: int
    logical_delta: int
    rpm: int
    accel: int


# ---------------------------------------------------------------------------
# Native EMM-V5 TTL driver
# ---------------------------------------------------------------------------

class _NativeTTL:
    """Native EMM-V5 multi-drop TTL driver using an already-open Serial."""

    def __init__(self, ser):
        self.ser = ser
        self._lock = threading.RLock()

    @staticmethod
    def _validate_id(motor_id: int) -> int:
        motor_id = int(motor_id)
        if motor_id not in MOTOR_IDS:
            raise ValueError("motor_id must be 1..4")
        return motor_id

    @staticmethod
    def _validate_rpm(rpm: int) -> int:
        rpm = int(rpm)
        if not MIN_RPM <= rpm <= MAX_RPM:
            raise ValueError("rpm must be %d..%d" % (MIN_RPM, MAX_RPM))
        return rpm

    @staticmethod
    def _native_delta(motor_id: int, logical_delta: int) -> int:
        """Convert logical robot direction to this motor's native direction."""
        logical_delta = int(logical_delta)
        if motor_id == RIGHT_ARM_ID and RIGHT_ARM_DIRECTION_INVERT:
            return -logical_delta
        if motor_id == LEFT_ARM_ID and LEFT_ARM_DIRECTION_INVERT:
            return -logical_delta
        return logical_delta

    def _write(self, frame: Iterable[int]) -> bytes:
        raw = bytes(frame)
        if not raw or raw[-1] != CHECKSUM:
            raise ValueError("native EMM-V5 frame must end in 0x6B")
        self.ser.write(raw)
        self.ser.flush()
        return raw

    def _read_reply(self, motor_id: int, function: int, expected_len: int,
                    timeout: float = 1.0) -> bytes:
        marker = bytes((motor_id & 0xFF, function & 0xFF))
        data = bytearray()
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            try:
                waiting = int(getattr(self.ser, "in_waiting", 0))
            except (OSError, serial.SerialException):
                waiting = 0

            if waiting > 0:
                data.extend(self.ser.read(waiting))
                start = data.find(marker)
                while start >= 0:
                    end = start + expected_len
                    if len(data) >= end and data[end - 1] == CHECKSUM:
                        return bytes(data[start:end])
                    start = data.find(marker, start + 1)
            else:
                # Some pyserial backends do not update in_waiting promptly.
                chunk = self.ser.read(1)
                if chunk:
                    data.extend(chunk)
                else:
                    time.sleep(0.001)

        raise MotionError(
            "TTL timeout waiting for motor %d function 0x%02X; rx=%s"
            % (motor_id, function, data.hex(" "))
        )

    def request(self, frame: Iterable[int], expected_len: int,
                timeout: float = 1.0) -> bytes:
        raw = bytes(frame)
        if len(raw) < 3:
            raise ValueError("native motor request is too short")
        motor_id = self._validate_id(raw[0])
        function = raw[1]
        if raw[-1] != CHECKSUM:
            raise ValueError("native motor request must end in 0x6B")

        with self._lock:
            try:
                self.ser.reset_input_buffer()
            except (AttributeError, serial.SerialException):
                pass
            self._write(raw)
            return self._read_reply(motor_id, function, expected_len, timeout)

    @staticmethod
    def _check_ack(reply: bytes, motor_id: int, function: int):
        if (
            len(reply) != 4
            or reply[0] != motor_id
            or reply[1] != function
            or reply[3] != CHECKSUM
        ):
            raise MotionError("malformed native ACK: " + reply.hex(" "))
        # EMM-V5 returns 0x02 for a successful command acknowledgement.
        if reply[2] != 0x02:
            raise MotionError(
                "motor %d rejected function 0x%02X: %s"
                % (motor_id, function, reply.hex(" "))
            )

    def enable(self, motor_id: int, enabled: bool, sync: bool = False):
        motor_id = self._validate_id(motor_id)
        frame = (
            motor_id, FN_ENABLE, 0xAB,
            1 if enabled else 0,
            1 if sync else 0,
            CHECKSUM,
        )
        reply = self.request(frame, 4, 1.2)
        self._check_ack(reply, motor_id, FN_ENABLE)

    def enable_ids(self, ids: Iterable[int], enabled: bool):
        for motor_id in ids:
            self.enable(int(motor_id), enabled)

    def enable_all(self, enabled: bool):
        self.enable_ids(MOTOR_IDS, enabled)

    def stop(self, motor_id: int, sync: bool = False):
        motor_id = self._validate_id(motor_id)
        frame = (motor_id, FN_STOP, 0x98, 1 if sync else 0, CHECKSUM)
        reply = self.request(frame, 4, 1.2)
        self._check_ack(reply, motor_id, FN_STOP)

    def stop_all(self):
        errors = []
        for motor_id in MOTOR_IDS:
            try:
                self.stop(motor_id)
            except Exception as exc:
                errors.append((motor_id, exc))
        if errors:
            logger.warning("best-effort stop had errors: %s", errors)

    def clear_stall(self, motor_id: int):
        motor_id = self._validate_id(motor_id)
        reply = self.request((motor_id, FN_CLEAR_STALL, 0x52, CHECKSUM), 4, 1.2)
        self._check_ack(reply, motor_id, FN_CLEAR_STALL)

    def read_position_raw(self, motor_id: int) -> int:
        motor_id = self._validate_id(motor_id)
        reply = self.request((motor_id, FN_READ_POS, CHECKSUM), 8, 1.2)
        if reply[0] != motor_id or reply[1] != FN_READ_POS or reply[-1] != CHECKSUM:
            raise MotionError("bad encoder reply: " + reply.hex(" "))
        magnitude = int.from_bytes(reply[3:7], "big", signed=False)
        if magnitude > 0x7FFFFFFF:
            raise MotionError("encoder magnitude outside signed 32-bit range")
        return -magnitude if reply[2] == 1 else magnitude

    @staticmethod
    def raw_delta_to_pulses(raw_delta: int) -> float:
        return float(raw_delta) * MOTOR_PULSES_PER_REV / ENCODER_COUNTS_PER_REV

    def read_position_pulses(self, motor_id: int) -> float:
        return self.raw_delta_to_pulses(self.read_position_raw(motor_id))

    def read_flags(self, motor_id: int, function: int = FN_FLAGS) -> int:
        motor_id = self._validate_id(motor_id)
        if function not in (FN_FLAGS, FN_HEALTH):
            raise ValueError("flag function must be 0x3A or 0x3B")
        reply = self.request((motor_id, function, CHECKSUM), 4, 1.2)
        if reply[0] != motor_id or reply[1] != function or reply[-1] != CHECKSUM:
            raise MotionError("bad flags reply: " + reply.hex(" "))
        return reply[2]

    def assert_healthy(self, motor_id: int):
        flags = self.read_flags(motor_id, FN_HEALTH)
        if not (flags & 0x01):
            raise MotionError("motor %d encoder not ready: 0x%02X" % (motor_id, flags))
        if not (flags & 0x02):
            raise MotionError("motor %d calibration not ready: 0x%02X" % (motor_id, flags))
        if flags & 0x10:
            raise MotionError("motor %d over-temperature: 0x%02X" % (motor_id, flags))
        if flags & 0x20:
            raise MotionError("motor %d over-current: 0x%02X" % (motor_id, flags))

    def queue_position(self, move: _Move):
        motor_id = self._validate_id(move.motor_id)
        rpm = self._validate_rpm(move.rpm)
        accel = int(move.accel)
        if not 0 <= accel <= 255:
            raise ValueError("accel must be 0..255")

        native_delta = self._native_delta(motor_id, move.logical_delta)
        direction = 1 if native_delta < 0 else 0
        pulses = abs(int(native_delta))
        if pulses > 0xFFFFFFFF:
            raise ValueError("position delta is too large")

        # Native EMM-V5 relative position, queued for synchronized start:
        # ID FD DIR RPM_H RPM_L ACC POS[4] RELATIVE(0) SYNC(1) 6B
        frame = (
            motor_id, FN_POSITION, direction,
            (rpm >> 8) & 0xFF, rpm & 0xFF,
            accel,
            (pulses >> 24) & 0xFF,
            (pulses >> 16) & 0xFF,
            (pulses >> 8) & 0xFF,
            pulses & 0xFF,
            0x00,  # relative mode
            0x01,  # synchronized-start queue
            CHECKSUM,
        )
        reply = self.request(frame, 4, 1.5)
        self._check_ack(reply, motor_id, FN_POSITION)

    def sync_start(self):
        # Native EMM-V5 multi-axis broadcast. A direct TTL adapter can place
        # this frame on the motor bus without any intermediary translation.
        with self._lock:
            self._write((0x00, 0xFF, 0x66, CHECKSUM))

    def wait_complete(self, motor_ids: Iterable[int], start_positions: dict[int, float],
                      timeout: float = 6.0):
        pending = set(int(x) for x in motor_ids)
        deadline = time.monotonic() + timeout
        started = time.monotonic()
        moved = {motor_id: False for motor_id in pending}

        # The completion flag can be latched from a previous trajectory on
        # some drive revisions. Require actual movement or >=120 ms elapsed.
        time.sleep(0.03)
        while pending and time.monotonic() < deadline:
            for motor_id in tuple(pending):
                flags = self.read_flags(motor_id, FN_FLAGS)
                if flags & 0x0C:
                    raise MotionError(
                        "motor %d stall/protection flags=0x%02X" % (motor_id, flags)
                    )
                now = self.read_position_pulses(motor_id)
                if abs(now - start_positions[motor_id]) > 8:
                    moved[motor_id] = True
                if (flags & 0x02) and (moved[motor_id] or time.monotonic() - started >= 0.12):
                    pending.remove(motor_id)
            if pending:
                time.sleep(0.01)

        if pending:
            raise MotionError("motion timeout waiting for motors %s" % sorted(pending))


# ---------------------------------------------------------------------------
# Legacy diagnostic helpers
# ---------------------------------------------------------------------------

def list_serial_ports():
    ports = list_ports.comports()
    if not ports:
        print("No serial ports detected")
        return
    print("Available serial ports:")
    for port in ports:
        print("  %s - %s" % (port.device, port.description))


def cmd_enable(ser, id_list, en):
    ids = [int(x) for x in id_list]
    if not ids:
        return True
    _NativeTTL(ser).enable_ids(ids, bool(en))
    return True


def cmd_stat(ser, motor_id):
    """Return old-style [count, trap, temp, pos, voltage] diagnostics."""
    bus = _NativeTTL(ser)
    motor_id = int(motor_id)
    flags = bus.read_flags(motor_id, FN_FLAGS)
    pos = round(bus.read_position_pulses(motor_id))
    trap_status = 1 if (flags & 0x0C) else 0
    return [0, trap_status, 0, pos, 0]


def cmd_get_pos(ser, motor_id):
    return round(_NativeTTL(ser).read_position_pulses(int(motor_id)))


def cmd_wait_motion(ser, motor_id, timeout=6.0):
    bus = _NativeTTL(ser)
    motor_id = int(motor_id)
    deadline = time.monotonic() + float(timeout)
    while time.monotonic() < deadline:
        flags = bus.read_flags(motor_id, FN_FLAGS)
        if flags & 0x0C:
            raise MotionError("motor %d stall/protection flags=0x%02X" % (motor_id, flags))
        if flags & 0x02:
            return round(bus.read_position_pulses(motor_id))
        time.sleep(0.01)
    raise MotionError("motion timeout waiting for motor %d" % motor_id)


def cmd_zero(ser):
    """Record the current safe mechanical pose as the PC-side reference.

    Return order is native motor ID order (1,2,3,4). ``MotionCtrl`` accepts the
    tuple directly, so existing ``zero = cmd_zero(); MotionCtrl(ser,*zero)``
    code does not need to change.
    """
    bus = _NativeTTL(ser)
    try:
        # Stop residual motion before defining the reference.
        bus.stop_all()
        time.sleep(0.08)

        for motor_id in MOTOR_IDS:
            flags = bus.read_flags(motor_id, FN_FLAGS)
            if flags & 0x0C:
                # Clear a latched stall only while the robot is stationary.
                bus.clear_stall(motor_id)
                time.sleep(0.03)
            bus.assert_healthy(motor_id)

        refs = tuple(bus.read_position_raw(motor_id) for motor_id in MOTOR_IDS)
        bus.enable_all(True)
        logger.info(
            "Direct TTL reference recorded: id1=%d id2=%d id3=%d id4=%d",
            refs[0], refs[1], refs[2], refs[3],
        )
        return refs
    except Exception:
        try:
            bus.stop_all()
        except Exception:
            pass
        raise


# ---------------------------------------------------------------------------
# High-level cube motion controller
# ---------------------------------------------------------------------------

_FOUR_TOKEN = {
    ("L2", "L+N", "R-F", "L0"): ("R", -90),
    ("L2", "L+N", "R+F", "L0"): ("R", +90),
    ("L2", "L+N", "R*F", "L0"): ("R", -180),
    ("R2", "R+N", "L-F", "R0"): ("L", -90),
    ("R2", "R+N", "L+F", "R0"): ("L", +90),
    ("R2", "R+N", "L*F", "R0"): ("L", -180),
}

_ARM_BACK_THREE = {
    ("L2", "L+N", "L0"): "L",
    ("R2", "R+N", "R0"): "R",
}

_FLIP_THREE = {
    ("L1", "R+F", "L0"): ("R", +90),
    ("L1", "R-F", "L0"): ("R", -90),
    ("L1", "R*F", "L0"): ("R", -180),
    ("R1", "L+F", "R0"): ("L", +90),
    ("R1", "L-F", "R0"): ("L", -90),
    ("R1", "L*F", "R0"): ("L", -180),
}


class MotionCtrl:
    """Direct native-TTL Rubik's-cube motion controller."""

    def __init__(self, ser, ref1=0, ref2=0, ref3=0, ref4=0):
        self.ser = ser
        self.bus = _NativeTTL(ser)
        self.ref_raw = {
            RIGHT_ARM_ID: int(ref1),
            RIGHT_FINGER_ID: int(ref2),
            LEFT_ARM_ID: int(ref3),
            LEFT_FINGER_ID: int(ref4),
        }
        self.arm = {"R": 0, "L": 0}
        self.jaw = {"R": JAW_CLAMP, "L": JAW_CLAMP}
        # Per-side adaptive clamp position. If a hard clamp drags the cube/arm,
        # overflow recovery backs this target off for the rest of the run.
        self.clamp_target = {"R": JAW_CLAMP, "L": JAW_CLAMP}

    # ------------------------- state / geometry -------------------------

    @staticmethod
    def _arm_id(side: str) -> int:
        if side == "R":
            return RIGHT_ARM_ID
        if side == "L":
            return LEFT_ARM_ID
        raise MotionError("side must be 'L' or 'R'")

    @staticmethod
    def _finger_id(side: str) -> int:
        if side == "R":
            return RIGHT_FINGER_ID
        if side == "L":
            return LEFT_FINGER_ID
        raise MotionError("side must be 'L' or 'R'")

    @staticmethod
    def _side_from_bool(left: bool) -> str:
        return "L" if bool(left) else "R"

    def _actual(self, motor_id: int) -> float:
        raw = self.bus.read_position_raw(motor_id)
        pulses = self.bus.raw_delta_to_pulses(raw - self.ref_raw[motor_id])
        # Commands for an inverted arm are converted from logical to native
        # direction in queue_position(). Encoder feedback must be converted
        # back the same way or the host would see a correct right-arm move as
        # having the opposite sign.
        if motor_id == RIGHT_ARM_ID and RIGHT_ARM_FEEDBACK_INVERT:
            pulses = -pulses
        if motor_id == LEFT_ARM_ID and LEFT_ARM_DIRECTION_INVERT:
            pulses = -pulses
        return pulses

    def _expected_motor(self, motor_id: int) -> float:
        if motor_id == RIGHT_ARM_ID:
            return float(self.arm["R"])
        if motor_id == LEFT_ARM_ID:
            return float(self.arm["L"])
        if motor_id == RIGHT_FINGER_ID:
            return float(self.jaw["R"] + self.arm["R"] / 2.0)
        if motor_id == LEFT_FINGER_ID:
            return float(self.jaw["L"] + self.arm["L"] / 2.0)
        raise ValueError("motor_id must be 1..4")

    def _verify_motor(self, motor_id: int, expected: float | None = None):
        if expected is None:
            expected = self._expected_motor(motor_id)
        actual = self._actual(motor_id)
        error = actual - expected
        if abs(error) > POSITION_TOLERANCE_PULSES:
            raise MotionError(
                "motor %d position mismatch: actual=%.1f expected=%.1f error=%.1f"
                % (motor_id, actual, expected, error)
            )
        return actual

    def verify_all(self):
        return {motor_id: self._verify_motor(motor_id) for motor_id in MOTOR_IDS}

    def _reconcile_from_encoders(self):
        """Rebuild host arm/jaw state from the four referenced encoders."""
        right_arm = round(self._actual(RIGHT_ARM_ID))
        left_arm = round(self._actual(LEFT_ARM_ID))
        right_finger = round(self._actual(RIGHT_FINGER_ID))
        left_finger = round(self._actual(LEFT_FINGER_ID))
        right_jaw = round(right_finger - right_arm / 2.0)
        left_jaw = round(left_finger - left_arm / 2.0)

        if not -POSITION_TOLERANCE_PULSES <= right_jaw <= JAW_MAX_FROM_CLAMP + POSITION_TOLERANCE_PULSES:
            raise MotionError("right jaw encoder state is outside safe reference range")
        if not -POSITION_TOLERANCE_PULSES <= left_jaw <= JAW_MAX_FROM_CLAMP + POSITION_TOLERANCE_PULSES:
            raise MotionError("left jaw encoder state is outside safe reference range")

        self.arm["R"] = right_arm
        self.arm["L"] = left_arm
        self.jaw["R"] = max(JAW_CLAMP, min(JAW_MAX_FROM_CLAMP, right_jaw))
        self.jaw["L"] = max(JAW_CLAMP, min(JAW_MAX_FROM_CLAMP, left_jaw))

    def _check_finger_target(self, motor_id: int, target: float):
        if motor_id not in FINGER_IDS:
            return
        if target < -BELT_FINGER_MAX_PULSES or target > BELT_FINGER_MAX_PULSES:
            raise MotionError(
                "finger motor %d target %.1f exceeds belt safety envelope"
                % (motor_id, target)
            )

    def _single_arm_collision_allowed(self, side: str, delta: int) -> bool:
        other = "L" if side == "R" else "R"
        current_arm = self.arm[side]
        next_arm = current_arm + delta
        current_gap = current_arm - self.arm[other]
        next_gap = current_gap + delta
        current_abs = abs(current_gap)
        next_abs = abs(next_gap)

        if (
            current_abs < ARM_V_COLLISION_GAP_PULSES
            and next_abs < current_abs
            and abs(next_arm) > abs(current_arm)
        ):
            return False
        if (current_gap > 0 > next_gap) or (current_gap < 0 < next_gap):
            return False
        return True

    def _pair_collision_allowed(self, side_a: str, delta_a: int,
                                side_b: str, delta_b: int) -> bool:
        gap = self.arm[side_a] - self.arm[side_b]
        next_gap = gap + delta_a - delta_b
        if abs(gap) < ARM_V_COLLISION_GAP_PULSES and abs(next_gap) < abs(gap):
            return False
        return True

    # ------------------------- native synchronized motion -------------------------

    def _sync(self, moves: Sequence[_Move], timeout: float = 6.0):
        moves = [move for move in moves if int(move.logical_delta) != 0]
        if not moves:
            return
        ids = [move.motor_id for move in moves]
        if len(ids) != len(set(ids)):
            raise MotionError("duplicate motor ID in synchronized trajectory")

        starts = {}
        targets = {}
        for move in moves:
            self._verify_motor(move.motor_id)
            starts[move.motor_id] = self.bus.read_position_pulses(move.motor_id)
            target = self._expected_motor(move.motor_id) + move.logical_delta
            self._check_finger_target(move.motor_id, target)
            targets[move.motor_id] = target

        try:
            for move in moves:
                self.bus.queue_position(move)
            self.bus.sync_start()
            self.bus.wait_complete(ids, starts, timeout=timeout)

            for motor_id, target in targets.items():
                self._verify_motor(motor_id, target)
        except Exception:
            self.bus.stop_all()
            raise

    def _recovery_sync(self, moves: Sequence[_Move], timeout: float = 5.0):
        """Relative native-TTL move used only while recovering encoder drift.

        Normal _sync() intentionally refuses to move a motor whose encoder is
        already away from the planned host state. Overflow recovery is exactly
        the case where the arm is known to be displaced, so this helper verifies
        the *relative correction* instead of requiring the old absolute state.
        """
        moves = [move for move in moves if int(move.logical_delta) != 0]
        if not moves:
            return
        ids = [move.motor_id for move in moves]
        if len(ids) != len(set(ids)):
            raise MotionError("duplicate motor ID in recovery trajectory")

        native_starts = {}
        logical_starts = {}
        for move in moves:
            logical_starts[move.motor_id] = self._actual(move.motor_id)
            native_starts[move.motor_id] = self.bus.read_position_pulses(move.motor_id)
            if move.motor_id in FINGER_IDS:
                self._check_finger_target(
                    move.motor_id,
                    logical_starts[move.motor_id] + move.logical_delta,
                )

        try:
            for move in moves:
                self.bus.queue_position(move)
            self.bus.sync_start()
            self.bus.wait_complete(ids, native_starts, timeout=timeout)

            # Recovery is deliberately slower than normal motion. Verify that
            # each axis actually performed the requested relative correction.
            for move in moves:
                after = self._actual(move.motor_id)
                travelled = after - logical_starts[move.motor_id]
                error = travelled - move.logical_delta
                if abs(error) > POSITION_TOLERANCE_PULSES:
                    raise MotionError(
                        "recovery motor %d delta mismatch: moved=%.1f expected=%d error=%.1f"
                        % (move.motor_id, travelled, move.logical_delta, error)
                    )
        except Exception:
            self.bus.stop_all()
            raise

    def _arm_encoder_errors(self):
        return {
            "R": self._actual(RIGHT_ARM_ID) - self.arm["R"],
            "L": self._actual(LEFT_ARM_ID) - self.arm["L"],
        }

    @staticmethod
    def _recovery_pair_collision_allowed(actual_r: float, delta_r: int,
                                         actual_l: float, delta_l: int) -> bool:
        current_gap = actual_r - actual_l
        next_gap = (actual_r + delta_r) - (actual_l + delta_l)
        # Do not let a recovery cross the two arm orientations through each
        # other. When already close to the collision region, only allow a move
        # that increases their separation.
        if (current_gap > 0 > next_gap) or (current_gap < 0 < next_gap):
            return False
        if abs(current_gap) < ARM_V_COLLISION_GAP_PULSES and abs(next_gap) < abs(current_gap):
            return False
        return True

    def _refresh_jaw_from_encoder(self, side: str):
        arm_actual = self._actual(self._arm_id(side))
        finger_actual = self._actual(self._finger_id(side))
        jaw_actual = round(finger_actual - arm_actual / 2.0)
        if -POSITION_TOLERANCE_PULSES <= jaw_actual <= JAW_MAX_FROM_CLAMP + POSITION_TOLERANCE_PULSES:
            self.jaw[side] = max(JAW_CLAMP, min(JAW_MAX_FROM_CLAMP, jaw_actual))
        return jaw_actual

    def _recover_clamp_overflow(self, clamped_sides: Sequence[str]):
        """Detect and undo arm rotation caused by an over-tight cube clamp.

        The planned arm state is kept unchanged. If clamp force has physically
        dragged either arm away from that state, the routine:
          1. slightly opens any currently clamped jaw involved in holding cube,
          2. remembers the softer clamp target for later clamps,
          3. returns displaced arm(s) to the planned encoder positions while
             solving the finger targets for the softened jaw coordinates.
        """
        clamped_sides = tuple(side for side in clamped_sides if side in ("R", "L"))
        if not clamped_sides:
            return False

        time.sleep(max(0.0, CLAMP_SETTLE_SECONDS))
        trigger = CLAMP_OVERFLOW_TRIGGER_DEG * ARM_PULSES_PER_DEG
        settle = CLAMP_OVERFLOW_SETTLE_DEG * ARM_PULSES_PER_DEG
        max_error = CLAMP_OVERFLOW_MAX_DEG * ARM_PULSES_PER_DEG
        max_step = max(1, round(CLAMP_RECOVERY_STEP_DEG * ARM_PULSES_PER_DEG))

        errors = self._arm_encoder_errors()
        affected = [side for side in ("R", "L") if abs(errors[side]) > trigger]
        if not affected:
            return False

        for side in affected:
            if abs(errors[side]) > max_error:
                raise MotionError(
                    "%s arm clamp overflow %.1f deg exceeds %.1f deg safety limit"
                    % (side, errors[side] / ARM_PULSES_PER_DEG, CLAMP_OVERFLOW_MAX_DEG)
                )

        logger.warning(
            "clamp overflow detected: R=%+.1f deg L=%+.1f deg; starting recovery",
            errors["R"] / ARM_PULSES_PER_DEG,
            errors["L"] / ARM_PULSES_PER_DEG,
        )

        # Back off jaws that are actually in a clamp state. This prevents the
        # same excess grip force from immediately dragging the corrected arm
        # away again. The adjusted target is retained for subsequent clamps.
        relax_moves = []
        for side in ("R", "L"):
            if self.jaw[side] <= CLAMP_SOFT_MAX_PULSES:
                old = int(self.jaw[side])
                new = min(
                    JAW_RELEASE,
                    CLAMP_SOFT_MAX_PULSES,
                    max(self.clamp_target[side], old) + CLAMP_RECOVERY_BACKOFF_PULSES,
                )
                if new > old:
                    relax_moves.append(
                        _Move(self._finger_id(side), new - old,
                              CLAMP_RECOVERY_RPM, CLAMP_RECOVERY_ACCEL)
                    )
                    self.clamp_target[side] = new

        if relax_moves:
            self._recovery_sync(relax_moves, timeout=4.0)
            for side in ("R", "L"):
                self._refresh_jaw_from_encoder(side)
            logger.warning(
                "adaptive clamp softened: R=%d L=%d pulses",
                self.clamp_target["R"], self.clamp_target["L"],
            )

        for attempt in range(1, CLAMP_RECOVERY_ATTEMPTS + 1):
            actual_r = self._actual(RIGHT_ARM_ID)
            actual_l = self._actual(LEFT_ARM_ID)
            actual_rf = self._actual(RIGHT_FINGER_ID)
            actual_lf = self._actual(LEFT_FINGER_ID)
            err_r = actual_r - self.arm["R"]
            err_l = actual_l - self.arm["L"]

            # The final finger target is solved independently from the arm
            # correction. This is essential: finger=arm_delta/2 would preserve
            # the already over-tight jaw geometry that caused the overflow.
            desired_rf = self.clamp_target["R"] + self.arm["R"] / 2.0
            desired_lf = self.clamp_target["L"] + self.arm["L"] / 2.0
            jaw_err_r = actual_rf - desired_rf
            jaw_err_l = actual_lf - desired_lf

            if (
                abs(err_r) <= settle and abs(err_l) <= settle
                and abs(jaw_err_r) <= POSITION_TOLERANCE_PULSES
                and abs(jaw_err_l) <= POSITION_TOLERANCE_PULSES
            ):
                break
            if abs(err_r) > max_error or abs(err_l) > max_error:
                raise MotionError("clamp overflow grew outside recovery safety envelope")

            corr_r = 0 if abs(err_r) <= settle else int(round(-err_r))
            corr_l = 0 if abs(err_l) <= settle else int(round(-err_l))
            corr_r = max(-max_step, min(max_step, corr_r))
            corr_l = max(-max_step, min(max_step, corr_l))

            if not self._recovery_pair_collision_allowed(actual_r, corr_r, actual_l, corr_l):
                raise MotionError("collision guard rejected clamp-overflow recovery")

            # Move fingers toward the *final softened jaw coordinates* while
            # the arms return toward their planned angles. Limit finger travel
            # per pass so a bad encoder/reference cannot cause a violent jaw
            # opening/closing command.
            finger_step = max_step
            corr_rf = int(round(desired_rf - actual_rf))
            corr_lf = int(round(desired_lf - actual_lf))
            corr_rf = max(-finger_step, min(finger_step, corr_rf))
            corr_lf = max(-finger_step, min(finger_step, corr_lf))

            recovery_moves = []
            if corr_rf:
                recovery_moves.append(
                    _Move(RIGHT_FINGER_ID, corr_rf,
                          max(MIN_RPM, CLAMP_RECOVERY_RPM // 2), CLAMP_RECOVERY_ACCEL)
                )
            if corr_r:
                recovery_moves.append(
                    _Move(RIGHT_ARM_ID, corr_r,
                          CLAMP_RECOVERY_RPM, CLAMP_RECOVERY_ACCEL)
                )
            if corr_lf:
                recovery_moves.append(
                    _Move(LEFT_FINGER_ID, corr_lf,
                          max(MIN_RPM, CLAMP_RECOVERY_RPM // 2), CLAMP_RECOVERY_ACCEL)
                )
            if corr_l:
                recovery_moves.append(
                    _Move(LEFT_ARM_ID, corr_l,
                          CLAMP_RECOVERY_RPM, CLAMP_RECOVERY_ACCEL)
                )

            logger.warning(
                "clamp recovery pass %d: arm R=%+.1f deg L=%+.1f deg, finger R=%+d L=%+d",
                attempt,
                corr_r / ARM_PULSES_PER_DEG,
                corr_l / ARM_PULSES_PER_DEG,
                corr_rf,
                corr_lf,
            )
            self._recovery_sync(recovery_moves, timeout=6.0)

        errors = self._arm_encoder_errors()
        for side in ("R", "L"):
            self._refresh_jaw_from_encoder(side)
        if abs(errors["R"]) > settle or abs(errors["L"]) > settle:
            raise MotionError(
                "clamp overflow recovery incomplete: R=%+.1f deg L=%+.1f deg"
                % (
                    errors["R"] / ARM_PULSES_PER_DEG,
                    errors["L"] / ARM_PULSES_PER_DEG,
                )
            )

        logger.info(
            "clamp overflow recovered: R=%+.2f deg L=%+.2f deg",
            errors["R"] / ARM_PULSES_PER_DEG,
            errors["L"] / ARM_PULSES_PER_DEG,
        )
        return True

    def _move_jaw(self, side: str, target: int, rpm: int = V_FINGER):
        requested_target = int(target)
        if requested_target == JAW_CLAMP:
            target = int(self.clamp_target[side])
        else:
            target = requested_target
        if not JAW_CLAMP <= target <= JAW_MAX_FROM_CLAMP:
            raise MotionError("jaw target %d outside safe range" % target)
        delta = target - self.jaw[side]
        if delta:
            finger = self._finger_id(side)
            self._sync((
                _Move(finger, delta, rpm, FINGER_POSITION_ACCEL),
            ))
            self.jaw[side] = target
        if requested_target == JAW_CLAMP:
            self._recover_clamp_overflow((side,))

    def _move_both_jaws(self, target: int, rpm: int = V_FINGER):
        requested_target = int(target)
        if not JAW_CLAMP <= requested_target <= JAW_MAX_FROM_CLAMP:
            raise MotionError("jaw target %d outside safe range" % requested_target)
        moves = []
        targets = {}
        for side in ("R", "L"):
            side_target = self.clamp_target[side] if requested_target == JAW_CLAMP else requested_target
            side_target = int(side_target)
            targets[side] = side_target
            delta = side_target - self.jaw[side]
            if delta:
                moves.append(_Move(self._finger_id(side), delta, rpm, FINGER_POSITION_ACCEL))
        self._sync(moves)
        self.jaw["R"] = targets["R"]
        self.jaw["L"] = targets["L"]
        if requested_target == JAW_CLAMP:
            self._recover_clamp_overflow(("R", "L"))

    def _coupled_turn(self, side: str, angle: int, rpm: int):
        if angle not in (-180, -90, 90, 180):
            raise MotionError("turn angle must be +/-90 or +/-180")
        rpm = self.bus._validate_rpm(rpm)
        # Keep the finger RPM exactly half of the arm RPM.
        if rpm & 1:
            rpm -= 1
        if rpm < MIN_RPM:
            rpm = MIN_RPM if MIN_RPM % 2 == 0 else MIN_RPM + 1
        arm_delta = int(angle / 90) * ARM_90_PULSES
        if not self._single_arm_collision_allowed(side, arm_delta):
            raise MotionError("arm collision guard rejected %s %d-degree turn" % (side, angle))

        finger_delta = arm_delta // 2
        self._sync((
            _Move(self._finger_id(side), finger_delta, max(MIN_RPM, rpm // 2), FINGER_POSITION_ACCEL),
            _Move(self._arm_id(side), arm_delta, rpm, ARM_POSITION_ACCEL),
        ))
        self.arm[side] += arm_delta

    def _arm_back(self, side: str, close_after: bool = True):
        """Open for clearance, +90 with belt following, optionally re-clamp."""
        self._move_jaw(side, JAW_NO_LOAD, V_FINGER)
        self._coupled_turn(side, +90, V_NO_LOAD)
        if close_after:
            self._move_jaw(side, JAW_CLAMP, V_FINGER)

    def _flip_adjust(self, flip_side: str, angle: int):
        """Direct four-axis version of the v1.2 FLIP_ADJUST primitive."""
        if angle not in (-180, -90, 90):
            raise MotionError("FLIP_ADJUST angle must be -180, -90 or +90")

        # Re-read all encoders before the compound action so host state cannot
        # silently drift after a reset or interrupted previous trajectory.
        self._reconcile_from_encoders()

        no_load_side = "R" if flip_side == "L" else "L"
        self._move_jaw(no_load_side, JAW_NO_LOAD, V_FINGER)

        no_load_delta = ARM_90_PULSES
        flip_delta = (-2 if angle == -180 else int(angle / 90)) * ARM_90_PULSES
        if not self._pair_collision_allowed(no_load_side, no_load_delta, flip_side, flip_delta):
            raise MotionError("arm collision guard rejected FLIP_ADJUST")

        moves = (
            _Move(self._finger_id(no_load_side), no_load_delta // 2,
                  max(MIN_RPM, V_NO_LOAD // 2), FINGER_POSITION_ACCEL),
            _Move(self._arm_id(no_load_side), no_load_delta,
                  V_NO_LOAD, ARM_POSITION_ACCEL),
            _Move(self._finger_id(flip_side), flip_delta // 2,
                  max(MIN_RPM, V_FLIP // 2), FINGER_POSITION_ACCEL),
            _Move(self._arm_id(flip_side), flip_delta,
                  V_FLIP, ARM_POSITION_ACCEL),
        )
        self._sync(moves, timeout=8.0)
        self.arm[no_load_side] += no_load_delta
        self.arm[flip_side] += flip_delta
        self._move_jaw(no_load_side, JAW_CLAMP, V_FINGER)

    # ------------------------- drop-in public methods -------------------------

    def emergency_stop(self):
        try:
            self.bus.stop_all()
        finally:
            try:
                self.bus.enable_all(False)
            except Exception:
                pass

    def two_finger_init(self):
        return self._move_both_jaws(JAW_RELEASE, V_FINGER)

    def two_finger_clamp(self):
        return self._move_both_jaws(JAW_CLAMP, V_FINGER)

    def move_finger_init(self, left):
        return self._move_jaw(self._side_from_bool(left), JAW_RELEASE, V_FINGER)

    def move_finger_lock(self, left):
        return self._move_jaw(self._side_from_bool(left), JAW_CLAMP, V_FINGER)

    def move_finger_flip(self, left, wait=0):
        # The direct backend completes the checked move before returning. This
        # is intentionally more conservative than the old early-return timing.
        del wait
        return self._move_jaw(self._side_from_bool(left), JAW_RELEASE, V_FINGER)

    def arm_90_no_load(self, left, no_finger_return=False):
        return self._arm_back(self._side_from_bool(left), close_after=not no_finger_return)

    def move_arm(self, angle, left, finger_current=None, speed=V_TWIST, accel=None, wait=0):
        del finger_current, accel, wait
        return self._coupled_turn(self._side_from_bool(left), int(angle), int(speed))

    def move_arm_without_finger(self, angle, left, speed=V_NO_LOAD, accel=None):
        # On the synchronous-belt mechanism an arm-only electrical move would
        # change the jaw opening. Always preserve the mechanical jaw coordinate
        # by moving the paired finger at half travel.
        del accel
        return self._coupled_turn(self._side_from_bool(left), int(angle), int(speed))

    def recover_horizontal(self):
        """Best-effort direct-TTL recovery to the recorded horizontal arms."""
        self.bus.stop_all()
        for motor_id in MOTOR_IDS:
            try:
                self.bus.clear_stall(motor_id)
            except Exception:
                pass
        self.bus.enable_all(True)
        self._reconcile_from_encoders()

        for side in ("R", "L"):
            tries = 0
            while abs(self.arm[side]) > POSITION_TOLERANCE_PULSES and tries < 8:
                delta = -self.arm[side]
                delta = max(-ARM_90_PULSES, min(ARM_90_PULSES, delta))
                if delta & 1:
                    delta -= 1
                if delta == 0:
                    break
                angle_like = delta
                if not self._single_arm_collision_allowed(side, delta):
                    raise MotionError("collision guard rejected horizontal recovery")
                self._sync((
                    _Move(self._finger_id(side), delta // 2,
                          max(MIN_RPM, V_NO_LOAD // 2), FINGER_POSITION_ACCEL),
                    _Move(self._arm_id(side), delta, V_NO_LOAD, ARM_POSITION_ACCEL),
                ))
                self.arm[side] += angle_like
                tries += 1

        self._reconcile_from_encoders()
        if abs(self.arm["R"]) > POSITION_TOLERANCE_PULSES or abs(self.arm["L"]) > POSITION_TOLERANCE_PULSES:
            raise MotionError("unable to recover both arms to horizontal reference")
        return True

    # ------------------------- optimizer token execution -------------------------

    def _basic(self, token: str):
        if len(token) < 2 or token[0] not in ("L", "R"):
            raise MotionError("invalid motion token %r" % token)
        side = token[0]
        op = token[1:]

        if op == "0":
            self._move_jaw(side, JAW_CLAMP, V_FINGER)
        elif op == "1":
            self._move_jaw(side, JAW_RELEASE, V_FINGER)
        elif op == "2":
            raise MotionError("%s must be grouped with %s+N" % (token, side))
        elif op == "+N":
            raise MotionError("%s must be grouped with a preceding %s2" % (token, side))
        elif op in ("+T", "+F"):
            self._coupled_turn(side, +90, V_TWIST if op.endswith("T") else V_FLIP)
        elif op in ("-T", "-F"):
            self._coupled_turn(side, -90, V_TWIST if op.endswith("T") else V_FLIP)
        elif op in ("*T", "*F"):
            self._coupled_turn(side, -180, V_TWIST if op.endswith("T") else V_FLIP)
        else:
            raise MotionError("unsupported motion token %s" % token)

    def motions(self, actions: Sequence[str] | str):
        """Execute cube_optimizer tokens with longest-match compound parsing."""
        if isinstance(actions, str):
            tokens = actions.split()
        else:
            tokens = [str(x).strip() for x in actions if str(x).strip()]

        i = 0
        try:
            while i < len(tokens):
                started = time.monotonic()

                # Four-token FLIP_ADJUST must be recognized before shorter
                # matches. It is one synchronized four-axis native TTL move.
                if i + 4 <= len(tokens):
                    key4 = tuple(tokens[i:i + 4])
                    compound = _FOUR_TOKEN.get(key4)
                    if compound is not None:
                        flip_side, angle = compound
                        logger.info("TTL compound: %s -> FLIP_ADJUST", " ".join(key4))
                        self._flip_adjust(flip_side, angle)
                        i += 4
                        continue

                if i + 3 <= len(tokens):
                    key3 = tuple(tokens[i:i + 3])

                    arm_back_side = _ARM_BACK_THREE.get(key3)
                    if arm_back_side is not None:
                        logger.info("TTL compound: %s -> ARM_BACK", " ".join(key3))
                        self._arm_back(arm_back_side, close_after=True)
                        i += 3
                        continue

                    flip = _FLIP_THREE.get(key3)
                    if flip is not None:
                        turn_side, angle = flip
                        support_side = "R" if turn_side == "L" else "L"
                        logger.info("TTL compound: %s -> FLIP", " ".join(key3))
                        self._move_jaw(support_side, JAW_RELEASE, V_FINGER)
                        self._coupled_turn(turn_side, angle, V_FLIP)
                        self._move_jaw(support_side, JAW_CLAMP, V_FINGER)
                        i += 3
                        continue

                # R2 R+N / L2 L+N with no same-side trailing 0: execute the
                # no-load turn and intentionally leave that jaw open.
                if i + 2 <= len(tokens):
                    first, second = tokens[i], tokens[i + 1]
                    if (
                        len(first) >= 2
                        and first[0] in ("L", "R")
                        and first[1:] == "2"
                        and second == first[0] + "+N"
                    ):
                        logger.info("TTL compound: %s %s -> ARM_BACK(open)", first, second)
                        self._arm_back(first[0], close_after=False)
                        i += 2
                        continue

                token = tokens[i]
                self._basic(token)
                i += 1
                logger.info(
                    "TTL token %d/%d %s completed in %.1f ms",
                    i, len(tokens), token,
                    (time.monotonic() - started) * 1000.0,
                )

            return True

        except Exception:
            logger.exception("direct TTL motion sequence aborted")
            try:
                self.bus.stop_all()
            except Exception:
                logger.exception("emergency stop also failed")
            raise


# ---------------------------------------------------------------------------
# Direct hardware test
# ---------------------------------------------------------------------------

def _main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_SERIAL_PORT
    try:
        with serial.Serial(
            port,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.02,
        ) as ser:
            print("Direct EMM-V5 TTL bus connected:", port)
            print("Motor map: 1=right arm, 2=right finger, 3=left arm, 4=left finger")
            print("Place both arms horizontal/reference and fingers at normal clamp.")
            input("Press Enter to record the current encoder positions as zero... ")

            zero = cmd_zero(ser)
            mc = MotionCtrl(ser, *zero)

            while True:
                command = input("motion ('R+T R-T', open, clamp, verify, recover, q) > ").strip()
                if command.lower() in ("q", "quit", "exit"):
                    break
                if command.lower() == "open":
                    mc.two_finger_init()
                elif command.lower() == "clamp":
                    mc.two_finger_clamp()
                elif command.lower() == "verify":
                    print(mc.verify_all())
                elif command.lower() == "recover":
                    mc.recover_horizontal()
                elif command:
                    mc.motions(command)

            mc.emergency_stop()

    except serial.SerialException as exc:
        logger.error("cannot open TTL serial port %s: %s", port, exc)
        list_serial_ports()
        raise SystemExit(1)
    except KeyboardInterrupt:
        print()


if __name__ == "__main__":
    _main()
