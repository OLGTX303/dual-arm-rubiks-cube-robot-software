#!/usr/bin/env python3
"""
Old hemn1990 closed-loop-stepper-motor v1.0.1 protocol -> ZDT X42S Emm-firmware bridge.

Purpose
-------
Keep software that sends the old FF FF / CRC8 protocol and translate it to
ZDT X42S Emm-firmware commands using the default fixed 0x6B checksum.

Supported old commands
----------------------
0x00 STAT   -> synthesize the old 15-byte status reply from ZDT reads
0x01 ENABLE -> ZDT F3 AB enable command
0x02 TRAP   -> ZDT Emm FD position command + runtime Ma_Limit update
0x03 ZERO   -> emulated old "move until blocked" using Emm FD + software stall detect
0x04 RESET  -> ZDT 08 97 reboot

Important compatibility choices
-------------------------------
* Target firmware: ZDT Emm firmware.
* Keep ZDT checksum mode at the factory/default fixed 0x6B.
* Keep ZDT Response mode = Receive.
* For closest old ZERO behavior, set ZDT Clog_Pro = Disable. The bridge does
  its own software stall detection and sends FE 98 stop, so ZDT coordinates
  are not reset by the built-in homing command.
* The old controller has a FIFO and a terminal-velocity v1 field. This bridge
  queues old TRAP commands, but ZDT Emm FD has no exact v1 equivalent; v1 is
  kept for diagnostics and otherwise ignored. Queued segments therefore may
  decelerate at segment boundaries.
* Emm FD has no per-command current-limit field. The bridge maps the old current
  percentage to the non-persistent closed-loop Ma_Limit command (45 66). On Emm
  firmware this value is documented as the maximum stall current, so it is only
  an approximation of the old controller current parameter.

Two ways to use it
------------------
1) Transparent PTY bridge on Linux/macOS:
     python3 old_to_zdt_bridge.py --zdt-port /dev/ttyUSB0 --pty-link /tmp/oldmotor
   Then point the old cube_motion.py at /tmp/oldmotor. It may still request
   1,000,000 baud; a PTY ignores the physical baud. The ZDT bus runs at the
   configured --zdt-baud (default 115200).

2) Drop-in serial-like class inside Python:
     from old_to_zdt_bridge import OldProtocolCompatSerial
     with OldProtocolCompatSerial('/dev/ttyUSB0') as ser:
         ... old cmd_enable/cmd_trap/cmd_stat code can use ser ...

Windows can use --old-port with a virtual COM-port pair (for example COM10/
COM11): old program opens one end; this bridge opens the other end.

This is a protocol compatibility layer, not an electrical level converter.
Connect the host to the ZDT TTL/RS485 interface appropriate for your board.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import os
import select
import struct
import sys
import time
try:
    import tty
except ImportError:
    tty = None
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Deque, Dict, Iterable, List, Optional, Sequence, Tuple

try:
    import serial  # type: ignore
except ImportError:  # allows --self-test without pyserial
    serial = None

LOG = logging.getLogger("old2zdt")

OLD_COUNTS_PER_REV = 16384.0
DEG_PER_OLD_COUNT = 360.0 / OLD_COUNTS_PER_REV
OLD_COUNT_PER_DEG = OLD_COUNTS_PER_REV / 360.0
ZDT_CHECKSUM = 0x6B

# Old trapezoid.c uses:
#   a_internal = a_input / 1024 encoder-count/control-cycle^2
# with a 5 kHz control loop. Convert that to RPM/s.
OLD_ACCEL_TO_RPM_S = (5000.0 * 5000.0 * 60.0) / (1024.0 * OLD_COUNTS_PER_REV)


class ProtocolError(RuntimeError):
    pass


class ZDTError(RuntimeError):
    pass


def clamp_int(v: float, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(round(v))))


def old_crc8(data: bytes) -> int:
    """Exact CRC routine used by old v1.0.1 firmware (poly 0x07)."""
    crc = 0
    for value in data:
        current = value
        for _ in range(8):
            if ((crc >> 7) ^ (current & 0x01)) != 0:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
            current >>= 1
    return crc


def old_ack_frame() -> bytes:
    frame = bytearray((0xFF, 0xFF, 0x05, 0x00))
    frame.append(old_crc8(bytes(frame)))
    return bytes(frame)


def old_stat_frame(cmd_count: int, trap_status: int, temperature_c: int,
                   position_counts: int, voltage_mv: int) -> bytes:
    temp = max(-128, min(127, int(temperature_c)))
    pos = max(-(1 << 31), min((1 << 31) - 1, int(position_counts)))
    voltage = max(-32768, min(32767, int(voltage_mv)))

    frame = bytearray((0xFF, 0xFF, 15, 0x00))
    frame += struct.pack("<H", cmd_count & 0xFFFF)
    frame += bytes((1 if trap_status else 0,))
    frame += struct.pack("b", temp)
    frame += struct.pack("<i", pos)
    frame += struct.pack("<h", voltage)
    frame.append(old_crc8(bytes(frame)))
    return bytes(frame)


def u16be(v: int) -> bytes:
    return int(v).to_bytes(2, "big", signed=False)


def u32be(v: int) -> bytes:
    return int(v).to_bytes(4, "big", signed=False)


def old_i16le(data: bytes) -> int:
    return struct.unpack("<h", data)[0]


def old_i32le(data: bytes) -> int:
    return struct.unpack("<i", data)[0]


@dataclass
class OldBlock:
    old_id: int
    cmd: int
    data: bytes


class OldFrameParser:
    """Streaming parser for FF FF LEN ... CRC frames."""

    def __init__(self) -> None:
        self.buf = bytearray()

    def feed(self, data: bytes) -> List[bytes]:
        self.buf.extend(data)
        out: List[bytes] = []
        while True:
            pos = self.buf.find(b"\xFF\xFF")
            if pos < 0:
                if self.buf and self.buf[-1] == 0xFF:
                    self.buf[:] = b"\xFF"
                else:
                    self.buf.clear()
                break
            if pos:
                del self.buf[:pos]
            if len(self.buf) < 3:
                break
            length = self.buf[2]
            if length < 5:
                del self.buf[0]
                continue
            if len(self.buf) < length:
                break
            frame = bytes(self.buf[:length])
            del self.buf[:length]
            out.append(frame)
        return out


def decode_old_frame(frame: bytes) -> List[OldBlock]:
    if len(frame) < 5 or frame[:2] != b"\xFF\xFF":
        raise ProtocolError("bad old frame header")
    if frame[2] != len(frame):
        raise ProtocolError(f"bad old length byte {frame[2]} != {len(frame)}")
    if old_crc8(frame[:-1]) != frame[-1]:
        raise ProtocolError("old CRC8 mismatch")

    count = frame[3]
    if count < 1 or count > 8:
        raise ProtocolError(f"bad old motor count {count}")
    body_len = len(frame) - 5
    if body_len % count:
        raise ProtocolError("old command blocks do not have equal length")
    block_len = body_len // count
    if block_len < 2:
        raise ProtocolError("old command block too short")

    blocks: List[OldBlock] = []
    p = 4
    for _ in range(count):
        b = frame[p:p + block_len]
        blocks.append(OldBlock(b[0], b[1], b[2:]))
        p += block_len
    return blocks


@dataclass
class MotorConfig:
    zdt_id: int
    direction: int = 1
    command_direction: int = 1
    encoder_offset_counts: int = 0
    reach_tolerance_counts: int = 64
    zero_motion_epsilon_counts: int = 8
    zero_stall_ms: int = 80
    zero_min_run_ms: int = 80
    zero_use_zdt_stall_flag: bool = False
    emm_pulses_per_rev: int = 3200


@dataclass
class BridgeConfig:
    old_full_scale_current_ma: int = 2500
    telemetry_cache_s: float = 0.5
    zdt_timeout_s: float = 0.10
    motors: Dict[int, MotorConfig] = field(default_factory=dict)

    @classmethod
    def defaults(cls) -> "BridgeConfig":
        return cls(motors={i: MotorConfig(zdt_id=i) for i in (1, 2, 3, 4)})

    @classmethod
    def load(cls, path: Optional[str]) -> "BridgeConfig":
        cfg = cls.defaults()
        if not path:
            return cfg
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
        cfg.old_full_scale_current_ma = int(raw.get(
            "old_full_scale_current_ma", cfg.old_full_scale_current_ma))
        cfg.telemetry_cache_s = float(raw.get("telemetry_cache_s", cfg.telemetry_cache_s))
        cfg.zdt_timeout_s = float(raw.get("zdt_timeout_s", cfg.zdt_timeout_s))

        if "motors" in raw:
            cfg.motors = {}
            for key, item in raw["motors"].items():
                old_id = int(key)
                cfg.motors[old_id] = MotorConfig(
                    zdt_id=int(item.get("zdt_id", old_id)),
                    direction=1 if int(item.get("direction", 1)) >= 0 else -1,
                    command_direction=1 if int(item.get("command_direction", item.get("direction", 1))) >= 0 else -1,
                    encoder_offset_counts=int(item.get("encoder_offset_counts", 0)),
                    reach_tolerance_counts=int(item.get("reach_tolerance_counts", 64)),
                    zero_motion_epsilon_counts=int(item.get("zero_motion_epsilon_counts", 8)),
                    zero_stall_ms=int(item.get("zero_stall_ms", 80)),
                    zero_min_run_ms=int(item.get("zero_min_run_ms", 80)),
                    zero_use_zdt_stall_flag=bool(item.get("zero_use_zdt_stall_flag", False)),
                    emm_pulses_per_rev=int(item.get("emm_pulses_per_rev", 3200)),
                )
        return cfg


class ZDTDriver:
    """Minimal X42S Emm-firmware 0x6B protocol driver used by the translator."""

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.10):
        if serial is None:
            raise RuntimeError("pyserial is required: pip install pyserial")
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.005,
            write_timeout=timeout,
        )
        self.timeout = timeout
        self.rxbuf = bytearray()

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

    @staticmethod
    def frame_enable(zdt_id: int, enable: bool, sync: int = 0) -> bytes:
        return bytes((zdt_id, 0xF3, 0xAB, 1 if enable else 0, sync & 1, ZDT_CHECKSUM))

    @staticmethod
    def frame_fd_emm(zdt_id: int, direction: int, speed_rpm: int,
                     accel_emm: int, pulses: int, mode: int, sync: int) -> bytes:
        """Emm position mode: Addr FD DIR SPEED(u16 RPM) ACC(u8) PULSES(u32) MODE SYNC 6B."""
        speed = max(0, min(3000, int(speed_rpm)))
        acc = max(0, min(255, int(accel_emm)))
        pos = max(0, min(0xFFFFFFFF, int(pulses)))
        frame = bytearray((zdt_id, 0xFD, direction & 1))
        frame += u16be(speed)
        frame += bytes((acc,))
        frame += u32be(pos)
        frame += bytes((mode & 0xFF, sync & 1, ZDT_CHECKSUM))
        return bytes(frame)

    @staticmethod
    def frame_closed_loop_max_current(zdt_id: int, current_ma: int, store: int = 0) -> bytes:
        """5.6.13: Addr 45 66 STORE CURRENT(u16 mA) 6B. For Emm this limits stall current."""
        cur = max(0, min(5000, int(current_ma)))
        return bytes((zdt_id, 0x45, 0x66, store & 1)) + u16be(cur) + bytes((ZDT_CHECKSUM,))

    @staticmethod
    def frame_stop(zdt_id: int, sync: int = 0) -> bytes:
        return bytes((zdt_id, 0xFE, 0x98, sync & 1, ZDT_CHECKSUM))

    @staticmethod
    def frame_sync_start() -> bytes:
        return bytes((0x00, 0xFF, 0x66, ZDT_CHECKSUM))

    @staticmethod
    def frame_reboot(zdt_id: int) -> bytes:
        return bytes((zdt_id, 0x08, 0x97, ZDT_CHECKSUM))

    def _read_expected(self, zdt_id: int, code: int, length: int,
                       timeout: Optional[float] = None,
                       skip_completion_9f: bool = False) -> bytes:
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while time.monotonic() < deadline:
            # Scan any already buffered bytes.
            i = 0
            while i + 1 < len(self.rxbuf):
                if self.rxbuf[i] == zdt_id and self.rxbuf[i + 1] == code:
                    if len(self.rxbuf) - i < length:
                        break
                    candidate = bytes(self.rxbuf[i:i + length])
                    del self.rxbuf[:i + length]
                    if candidate[-1] != ZDT_CHECKSUM:
                        i = 0
                        continue
                    if skip_completion_9f and length == 4 and candidate[2] == 0x9F:
                        i = 0
                        continue
                    return candidate
                i += 1
            chunk = self.ser.read(max(1, self.ser.in_waiting))
            if chunk:
                self.rxbuf.extend(chunk)
                if len(self.rxbuf) > 512:
                    del self.rxbuf[:-256]
            else:
                time.sleep(0.0005)
        raise ZDTError(f"timeout waiting ZDT id={zdt_id} code=0x{code:02X} len={length}")

    def _write(self, frame: bytes) -> None:
        LOG.debug("ZDT TX: %s", frame.hex(" ").upper())
        self.ser.write(frame)
        self.ser.flush()

    def command_ack(self, frame: bytes, zdt_id: int, code: int,
                    timeout: Optional[float] = None) -> int:
        self._write(frame)
        reply = self._read_expected(zdt_id, code, 4, timeout, skip_completion_9f=True)
        LOG.debug("ZDT RX: %s", reply.hex(" ").upper())
        status = reply[2]
        if status not in (0x02, 0x12, 0x22):
            raise ZDTError(
                f"ZDT id={zdt_id} code=0x{code:02X} returned status 0x{status:02X}")
        return status

    def enable(self, zdt_id: int, enable: bool, sync: int = 0) -> None:
        self.command_ack(self.frame_enable(zdt_id, enable, sync), zdt_id, 0xF3)

    def move_fd_emm(self, zdt_id: int, direction: int, speed_rpm: int,
                    accel_emm: int, pulses: int, mode: int, sync: int) -> None:
        self.command_ack(
            self.frame_fd_emm(zdt_id, direction, speed_rpm, accel_emm,
                              pulses, mode, sync),
            zdt_id, 0xFD)

    def set_closed_loop_max_current(self, zdt_id: int, current_ma: int) -> None:
        self.command_ack(
            self.frame_closed_loop_max_current(zdt_id, current_ma, 0),
            zdt_id, 0x45)

    def stop(self, zdt_id: int, sync: int = 0) -> None:
        self.command_ack(self.frame_stop(zdt_id, sync), zdt_id, 0xFE)

    def sync_start(self) -> None:
        # Broadcast sync-start has an optional/non-success reply (often 0xE2).
        # The queued FD commands are already accepted; do not abort the move.
        self._write(self.frame_sync_start())
        deadline = time.monotonic() + min(self.timeout, 0.05)
        while time.monotonic() < deadline:
            chunk = self.ser.read(max(1, self.ser.in_waiting))
            if chunk:
                self.rxbuf.extend(chunk)
            else:
                time.sleep(0.0005)

    def reboot(self, zdt_id: int) -> None:
        self.command_ack(self.frame_reboot(zdt_id), zdt_id, 0x08)

    def _read_cmd(self, zdt_id: int, code: int, reply_len: int) -> bytes:
        self._write(bytes((zdt_id, code, ZDT_CHECKSUM)))
        reply = self._read_expected(zdt_id, code, reply_len)
        LOG.debug("ZDT RX: %s", reply.hex(" ").upper())
        return reply

    def read_encoder(self, zdt_id: int) -> int:
        # Addr 31 encoder_u16 6B
        r = self._read_cmd(zdt_id, 0x31, 5)
        return int.from_bytes(r[2:4], "big", signed=False)

    def read_position_deg(self, zdt_id: int) -> float:
        # Emm firmware: Addr 36 sign raw_u32 6B; 65536 counts = 360 degrees.
        r = self._read_cmd(zdt_id, 0x36, 8)
        sign = -1.0 if r[2] == 0x01 else 1.0
        raw = int.from_bytes(r[3:7], "big", signed=False)
        return sign * raw * 360.0 / 65536.0

    def read_state(self, zdt_id: int) -> int:
        r = self._read_cmd(zdt_id, 0x3A, 4)
        return r[2]

    def read_temperature_c(self, zdt_id: int) -> int:
        # Manual: temp sign 00=negative, 01=positive.
        r = self._read_cmd(zdt_id, 0x39, 5)
        magnitude = r[3]
        return magnitude if r[2] == 0x01 else -magnitude

    def read_vbus_mv(self, zdt_id: int) -> int:
        r = self._read_cmd(zdt_id, 0x24, 5)
        return int.from_bytes(r[2:4], "big", signed=False)


@dataclass
class MotionCommand:
    kind: str  # "trap" or "zero"
    target_counts: int
    v1_rpm: int
    vmax_rpm: int
    accel_old: int
    current_pct: int
    started_at: float = 0.0
    last_pos_counts: Optional[int] = None
    start_pos_counts: Optional[int] = None
    last_moving_at: float = 0.0


@dataclass
class MotorRuntime:
    cmd_count: int = 0
    active: Optional[MotionCommand] = None
    queue: Deque[MotionCommand] = field(default_factory=deque)
    coord_ready: bool = False
    old_origin_counts: float = 0.0
    zdt_origin_deg: float = 0.0
    cached_temp: int = 25
    cached_vbus: int = 24000
    telemetry_time: float = 0.0
    last_current_ma: Optional[int] = None


class OldToZDTTranslator:
    def __init__(self, zdt: ZDTDriver, config: Optional[BridgeConfig] = None):
        self.zdt = zdt
        self.config = config or BridgeConfig.defaults()
        self.parser = OldFrameParser()
        self.rt: Dict[int, MotorRuntime] = {
            old_id: MotorRuntime() for old_id in self.config.motors
        }
        self.warned_v1 = False

    def _mcfg(self, old_id: int) -> MotorConfig:
        if old_id not in self.config.motors:
            raise ProtocolError(f"old motor id {old_id} is not configured")
        return self.config.motors[old_id]

    def _runtime(self, old_id: int) -> MotorRuntime:
        self._mcfg(old_id)
        return self.rt.setdefault(old_id, MotorRuntime())

    def _ensure_coord(self, old_id: int) -> None:
        r = self._runtime(old_id)
        if r.coord_ready:
            return
        c = self._mcfg(old_id)
        enc = self.zdt.read_encoder(c.zdt_id)
        zpos = self.zdt.read_position_deg(c.zdt_id)
        # 0..65535 encoder -> 0..16383 old counts, plus a user calibration offset.
        old_single = int(round(enc * OLD_COUNTS_PER_REV / 65536.0))
        old_single = (old_single + c.encoder_offset_counts) % int(OLD_COUNTS_PER_REV)
        r.old_origin_counts = float(old_single)
        r.zdt_origin_deg = float(zpos)
        r.coord_ready = True
        LOG.info(
            "coord old_id=%d zdt_id=%d encoder=%d old_origin=%d zdt_origin=%.1fdeg dir=%+d",
            old_id, c.zdt_id, enc, old_single, zpos, c.direction)

    def _old_from_zdt_deg(self, old_id: int, zdt_deg: float) -> int:
        self._ensure_coord(old_id)
        r = self._runtime(old_id)
        c = self._mcfg(old_id)
        delta_deg = (zdt_deg - r.zdt_origin_deg) * c.direction
        return int(round(r.old_origin_counts + delta_deg * OLD_COUNT_PER_DEG))

    def _zdt_deg_from_old(self, old_id: int, old_counts: int) -> float:
        self._ensure_coord(old_id)
        r = self._runtime(old_id)
        c = self._mcfg(old_id)
        delta_old = float(old_counts) - r.old_origin_counts
        return r.zdt_origin_deg + c.direction * delta_old * DEG_PER_OLD_COUNT

    def _current_ma(self, pct: int) -> int:
        pct = max(0, min(100, int(pct)))
        return clamp_int(
            self.config.old_full_scale_current_ma * pct / 100.0, 0, 5000)

    @staticmethod
    def _accel_emm(old_accel: int) -> int:
        """Map old acceleration to Emm's 0..255 acceleration index.

        Old acceleration is first converted to RPM/s. Emm documents:
            dRPM/dt = 1 / ((256-acc)*50us) = 20000/(256-acc) RPM/s
        acc=0 is Emm's special immediate-start mode, so old_accel=0 maps to 0.
        """
        if old_accel == 0:
            return 0
        target = abs(old_accel) * OLD_ACCEL_TO_RPM_S
        if target <= 0:
            return 0
        acc = 256.0 - (20000.0 / target)
        return clamp_int(acc, 1, 255)

    def _build_motion_values(self, old_id: int, m: MotionCommand) -> Tuple[int, int, int, int, int]:
        # The old x1 is an absolute virtual encoder target. Emm FD's mode 02 is
        # relative to the motor's current realtime position, so translate the
        # absolute old target into a relative pulse move at dispatch time. This
        # avoids requiring ZDT's internal coordinate zero to match the old board.
        c = self._mcfg(old_id)
        zpos = self.zdt.read_position_deg(c.zdt_id)
        current_old = self._old_from_zdt_deg(old_id, zpos)
        delta_old = m.target_counts - current_old
        signed_zdt_delta = delta_old * c.command_direction
        direction = 0x00 if signed_zdt_delta >= 0 else 0x01
        pulses = clamp_int(
            abs(delta_old) * max(1, c.emm_pulses_per_rev) / OLD_COUNTS_PER_REV,
            0, 0xFFFFFFFF)
        speed_rpm = clamp_int(abs(m.vmax_rpm), 0, 3000)
        accel_emm = self._accel_emm(m.accel_old)
        current = self._current_ma(m.current_pct)
        return direction, accel_emm, speed_rpm, pulses, current

    def _send_motion(self, old_id: int, m: MotionCommand, sync: int) -> None:
        c = self._mcfg(old_id)
        r = self._runtime(old_id)
        current_old = self._old_from_zdt_deg(old_id, self.zdt.read_position_deg(c.zdt_id))
        direction, accel, speed, pulses, current = self._build_motion_values(old_id, m)
        if m.v1_rpm != 0 and not self.warned_v1:
            LOG.warning(
                "old terminal velocity v1 is non-zero; ZDT Emm FD has no exact v1 field. "
                "Commands are queued but may decelerate at segment boundaries.")
            self.warned_v1 = True

        # Emm FD has no per-command current field. 45 66 changes the closed-loop
        # maximum current without storing it. The manual states that on Emm this
        # is the maximum stall current, so this is the closest safe approximation.
        if r.last_current_ma != current:
            self.zdt.set_closed_loop_max_current(c.zdt_id, current)
            r.last_current_ma = current

        # Emm FD mode 02 = relative to current realtime position.
        self.zdt.move_fd_emm(c.zdt_id, direction, speed, accel, pulses, 0x00, sync)
        now = time.monotonic()
        m.started_at = now
        m.last_moving_at = now
        m.last_pos_counts = None
        m.start_pos_counts = current_old
        LOG.debug(
            "motion old_id=%d kind=%s target=%d v1=%d vmax=%d emm_acc=%d pulses=%d current=%dmA sync=%d",
            old_id, m.kind, m.target_counts, m.v1_rpm, m.vmax_rpm,
            accel, pulses, current, sync)

    def _enqueue_motion_batch(self, commands: Sequence[Tuple[int, MotionCommand]]) -> None:
        start_now: List[Tuple[int, MotionCommand]] = []
        for old_id, m in commands:
            r = self._runtime(old_id)
            if r.active is None and not r.queue:
                r.active = m
                start_now.append((old_id, m))
            else:
                r.queue.append(m)
                LOG.debug("queued old_id=%d kind=%s depth=%d", old_id, m.kind, len(r.queue))

        if not start_now:
            return
        sync = 1 if len(start_now) > 1 else 0
        try:
            for old_id, m in start_now:
                self._send_motion(old_id, m, sync)
            if sync:
                self.zdt.sync_start()
        except Exception:
            # Avoid claiming an active command that was never started successfully.
            for old_id, m in start_now:
                r = self._runtime(old_id)
                if r.active is m:
                    r.active = None
            raise

    def _start_next_if_any(self, old_id: int) -> None:
        r = self._runtime(old_id)
        if r.active is not None or not r.queue:
            return
        m = r.queue.popleft()
        r.active = m
        try:
            self._send_motion(old_id, m, 0)
        except Exception:
            r.active = None
            raise

    def _read_pos_and_state(self, old_id: int) -> Tuple[int, int]:
        c = self._mcfg(old_id)
        zpos = self.zdt.read_position_deg(c.zdt_id)
        old_pos = self._old_from_zdt_deg(old_id, zpos)
        state = self.zdt.read_state(c.zdt_id)
        return old_pos, state

    def _finish_active(self, old_id: int) -> None:
        r = self._runtime(old_id)
        r.active = None
        self._start_next_if_any(old_id)

    def _refresh_motion(self, old_id: int) -> Tuple[int, int, int]:
        """Return (old_position_counts, zdt_state, old_trap_status)."""
        r = self._runtime(old_id)
        c = self._mcfg(old_id)
        pos, state = self._read_pos_and_state(old_id)
        m = r.active
        if m is None:
            self._start_next_if_any(old_id)
            return pos, state, 1 if self._runtime(old_id).active is not None else 0

        reached = bool(state & 0x02) and abs(pos - m.target_counts) <= c.reach_tolerance_counts
        now = time.monotonic()

        if m.kind == "zero":
            if reached:
                LOG.info("old ZERO reached full target without a stall, old_id=%d", old_id)
                self._finish_active(old_id)
            else:
                if m.last_pos_counts is None:
                    m.last_pos_counts = pos
                    m.last_moving_at = now
                elif abs(pos - m.last_pos_counts) > c.zero_motion_epsilon_counts:
                    m.last_pos_counts = pos
                    m.last_moving_at = now

                elapsed_ms = (now - m.started_at) * 1000.0
                still_ms = (now - m.last_moving_at) * 1000.0
                zdt_stall = bool(state & 0x04)
                stall = (
                    elapsed_ms >= c.zero_min_run_ms and
                    (still_ms >= c.zero_stall_ms or
                     (c.zero_use_zdt_stall_flag and zdt_stall))
                )
                if stall:
                    LOG.info(
                        "old ZERO stall emulated old_id=%d pos=%d still=%.1fms; sending FE stop",
                        old_id, pos, still_ms)
                    self.zdt.stop(c.zdt_id, 0)
                    self._finish_active(old_id)
        else:
            # The EMM in-position bit can remain set from the previous stop.
            # Require encoder tolerance for real arm moves; only allow the bit
            # alone for small finger moves that quantize below encoder tolerance.
            small_move = (m.start_pos_counts is not None and
                          abs(m.target_counts - m.start_pos_counts) <= 512)
            if reached or (small_move and bool(state & 0x02)):
                self._finish_active(old_id)

        r = self._runtime(old_id)
        trap = 1 if (r.active is not None or r.queue) else 0
        return pos, state, trap

    def _telemetry(self, old_id: int) -> Tuple[int, int]:
        r = self._runtime(old_id)
        c = self._mcfg(old_id)
        now = time.monotonic()
        if now - r.telemetry_time >= self.config.telemetry_cache_s:
            try:
                r.cached_temp = self.zdt.read_temperature_c(c.zdt_id)
                r.cached_vbus = self.zdt.read_vbus_mv(c.zdt_id)
                r.telemetry_time = now
            except ZDTError:
                # Position/state are the important compatibility values. Retain cached
                # telemetry if one of these secondary reads momentarily times out.
                LOG.warning("telemetry read timeout old_id=%d; using cached values", old_id)
        return r.cached_temp, r.cached_vbus

    def _handle_stat(self, block: OldBlock) -> bytes:
        if block.data:
            raise ProtocolError("old STAT must have no data")
        old_id = block.old_id
        r = self._runtime(old_id)
        r.cmd_count = (r.cmd_count + 1) & 0xFFFF
        pos, _state, trap = self._refresh_motion(old_id)
        temp, vbus = self._telemetry(old_id)
        return old_stat_frame(r.cmd_count, trap, temp, pos, vbus)

    def _handle_enable_batch(self, blocks: Sequence[OldBlock]) -> None:
        parsed: List[Tuple[int, bool]] = []
        for b in blocks:
            if len(b.data) != 1:
                raise ProtocolError("old ENABLE data length must be 1")
            self._runtime(b.old_id).cmd_count = (self._runtime(b.old_id).cmd_count + 1) & 0xFFFF
            parsed.append((b.old_id, bool(b.data[0])))

        # EMM firmware may return 0xE2 for broadcast sync-start here and leave
        # the motors disabled. Enable/disable each address immediately.
        for old_id, en in parsed:
            c = self._mcfg(old_id)
            self.zdt.enable(c.zdt_id, en, 0)
            if not en:
                r = self._runtime(old_id)
                r.active = None
                r.queue.clear()

    def _decode_motion(self, b: OldBlock) -> MotionCommand:
        if len(b.data) != 11:
            raise ProtocolError("old TRAP/ZERO data length must be 11")
        x1 = old_i32le(b.data[0:4])
        v1 = old_i16le(b.data[4:6])
        vmax = old_i16le(b.data[6:8])
        accel = old_i16le(b.data[8:10])
        current = b.data[10]
        return MotionCommand(
            kind="zero" if b.cmd == 0x03 else "trap",
            target_counts=x1,
            v1_rpm=v1,
            vmax_rpm=vmax,
            accel_old=accel,
            current_pct=current,
        )

    def _handle_motion_batch(self, blocks: Sequence[OldBlock]) -> None:
        commands: List[Tuple[int, MotionCommand]] = []
        for b in blocks:
            if b.cmd not in (0x02, 0x03):
                raise ProtocolError("mixed old motion packet contains unsupported command")
            r = self._runtime(b.old_id)
            r.cmd_count = (r.cmd_count + 1) & 0xFFFF
            commands.append((b.old_id, self._decode_motion(b)))
        self._enqueue_motion_batch(commands)

    def _handle_reset_batch(self, blocks: Sequence[OldBlock]) -> None:
        for b in blocks:
            if b.data:
                raise ProtocolError("old RESET must have no data")
            old_id = b.old_id
            r = self._runtime(old_id)
            r.cmd_count = (r.cmd_count + 1) & 0xFFFF
            c = self._mcfg(old_id)
            self.zdt.reboot(c.zdt_id)
            r.active = None
            r.queue.clear()
            r.coord_ready = False
            r.last_current_ma = None
            # Give the controller a short reboot window before another command.
            time.sleep(0.05)

    def handle_frame(self, frame: bytes) -> Optional[bytes]:
        LOG.debug("OLD RX: %s", frame.hex(" ").upper())
        blocks = decode_old_frame(frame)

        # Old STAT only replies when exactly one motor block is present.
        if len(blocks) == 1 and blocks[0].cmd == 0x00:
            reply = self._handle_stat(blocks[0])
            LOG.debug("OLD TX: %s", reply.hex(" ").upper())
            return reply

        cmds = {b.cmd for b in blocks}
        if cmds == {0x01}:
            self._handle_enable_batch(blocks)
        elif cmds.issubset({0x02, 0x03}):
            self._handle_motion_batch(blocks)
        elif cmds == {0x04}:
            self._handle_reset_batch(blocks)
        else:
            raise ProtocolError(
                "unsupported/mixed old command packet: " + ",".join(f"0x{x:02X}" for x in sorted(cmds)))

        reply = old_ack_frame()
        LOG.debug("OLD TX: %s", reply.hex(" ").upper())
        return reply

    def feed(self, data: bytes) -> bytes:
        out = bytearray()
        for frame in self.parser.feed(data):
            try:
                reply = self.handle_frame(frame)
                if reply:
                    out += reply
            except (ProtocolError, ZDTError) as e:
                # Real old hardware normally gives no valid ACK on malformed/no-response
                # conditions. Preserve that behavior rather than inventing an old error code.
                LOG.error("bridge dropped frame: %s", e)
        return bytes(out)


class OldProtocolCompatSerial:
    """A small pyserial-like facade that accepts old protocol bytes directly."""

    def __init__(self, zdt_port: str, zdt_baud: int = 115200,
                 config: Optional[BridgeConfig] = None,
                 timeout: float = 1.0, **_ignored) -> None:
        self.config = config or BridgeConfig.defaults()
        self.zdt = ZDTDriver(zdt_port, zdt_baud, self.config.zdt_timeout_s)
        self.translator = OldToZDTTranslator(self.zdt, self.config)
        self._rx = bytearray()
        self.timeout = timeout
        self.port = zdt_port
        self.is_open = True

    @property
    def in_waiting(self) -> int:
        return len(self._rx)

    def write(self, data: bytes) -> int:
        if not self.is_open:
            raise OSError("serial port is closed")
        reply = self.translator.feed(bytes(data))
        if reply:
            self._rx += reply
        return len(data)

    def read(self, size: int = 1) -> bytes:
        deadline = time.monotonic() + (self.timeout if self.timeout is not None else 1e9)
        while not self._rx and time.monotonic() < deadline:
            time.sleep(0.0005)
        n = min(max(0, int(size)), len(self._rx))
        data = bytes(self._rx[:n])
        del self._rx[:n]
        return data

    def flush(self) -> None:
        return None

    def reset_input_buffer(self) -> None:
        self._rx.clear()

    def close(self) -> None:
        if self.is_open:
            self.zdt.close()
            self.is_open = False

    def __enter__(self) -> "OldProtocolCompatSerial":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


class StreamBridge:
    def __init__(self, translator: OldToZDTTranslator):
        self.translator = translator

    def run_pty(self, link_path: Optional[str] = None) -> None:
        if os.name == "nt":
            raise RuntimeError("PTY mode is Unix-only; use --old-port on Windows")
        master_fd, slave_fd = os.openpty()
        if tty is None:
            raise RuntimeError("PTY mode requires the Unix tty module")
        tty.setraw(slave_fd)
        slave_name = os.ttyname(slave_fd)

        display_name = slave_name
        if link_path:
            link = Path(link_path)
            try:
                if link.is_symlink() or link.exists():
                    link.unlink()
                link.symlink_to(slave_name)
                display_name = str(link)
            except OSError as e:
                LOG.warning("could not create PTY symlink %s: %s", link_path, e)

        print(display_name, flush=True)
        LOG.info("old-protocol PTY: %s -> ZDT bus", display_name)
        try:
            while True:
                readable, _, _ = select.select([master_fd], [], [], 0.25)
                if not readable:
                    continue
                data = os.read(master_fd, 4096)
                if not data:
                    time.sleep(0.01)
                    continue
                reply = self.translator.feed(data)
                if reply:
                    os.write(master_fd, reply)
        finally:
            try:
                os.close(master_fd)
            finally:
                os.close(slave_fd)

    def run_serial_upstream(self, old_port: str, old_baud: int = 1_000_000) -> None:
        if serial is None:
            raise RuntimeError("pyserial is required: pip install pyserial")
        upstream = serial.Serial(old_port, old_baud, timeout=0.02)
        LOG.info("old-protocol upstream: %s @ %d", old_port, old_baud)
        try:
            while True:
                data = upstream.read(max(1, upstream.in_waiting))
                if not data:
                    continue
                reply = self.translator.feed(data)
                if reply:
                    upstream.write(reply)
                    upstream.flush()
        finally:
            upstream.close()


def self_test() -> None:
    # Known frames copied from the v1.0.1 firmware comments/source.
    examples = {
        "stat": ("ff ff 07 01 01 00 4f", 0x4F),
        "enable": ("ff ff 08 01 01 01 01 dc", 0xDC),
        "multi_enable": ("ff ff 11 04 01 01 01 02 01 01 03 01 01 04 01 01 77", 0x77),
        "trap": ("ff ff 12 01 01 02 00 40 00 00 00 00 f4 01 64 00 14 76", 0x76),
    }
    for name, (hex_s, expected) in examples.items():
        frame = bytes.fromhex(hex_s)
        got = old_crc8(frame[:-1])
        assert got == expected == frame[-1], (name, got, expected, frame[-1])
        decode_old_frame(frame)

    assert old_ack_frame() == bytes.fromhex("ff ff 05 00 e2")
    assert ZDTDriver.frame_enable(1, True, 0) == bytes.fromhex("01 f3 ab 01 00 6b")

    # Old trap example: x1=16384 (1 rev), v1=0, vmax=500 RPM,
    # old acceleration=100, current=20% -> 500 mA at 2.5 A full scale.
    trap = decode_old_frame(bytes.fromhex(examples["trap"][0]))[0]
    m = MotionCommand(
        kind="trap",
        target_counts=old_i32le(trap.data[0:4]),
        v1_rpm=old_i16le(trap.data[4:6]),
        vmax_rpm=old_i16le(trap.data[6:8]),
        accel_old=old_i16le(trap.data[8:10]),
        current_pct=trap.data[10],
    )
    assert m.target_counts == 16384
    assert m.vmax_rpm == 500
    assert m.accel_old == 100
    assert m.current_pct == 20
    assert clamp_int(100 * OLD_ACCEL_TO_RPM_S, 1, 65535) == 8941

    # Emm mapping for one relative revolution at 500 RPM. With the default
    # 16 microsteps, one revolution is 3200 pulses. Old accel=100 maps to 254.
    assert OldToZDTTranslator._accel_emm(100) == 254
    ma = ZDTDriver.frame_closed_loop_max_current(1, 500, 0)
    assert ma == bytes.fromhex("01 45 66 00 01 f4 6b")
    fd = ZDTDriver.frame_fd_emm(1, 0, 500, 254, 3200, 2, 0)
    assert fd == bytes.fromhex("01 fd 00 01 f4 fe 00 00 0c 80 02 00 6b")

    print("self-test OK")
    print("old TRAP example -> Emm current-limit approximation + FD move:")
    print(ma.hex(" ").upper())
    print(fd.hex(" ").upper())


def write_example_config(path: str) -> None:
    data = {
        "old_full_scale_current_ma": 2500,
        "telemetry_cache_s": 0.5,
        "zdt_timeout_s": 0.10,
        "motors": {
            str(i): {
                "zdt_id": i,
                "direction": 1,
                "encoder_offset_counts": 0,
                "reach_tolerance_counts": 64,
                "zero_motion_epsilon_counts": 8,
                "zero_stall_ms": 80,
                "zero_min_run_ms": 80,
                "zero_use_zdt_stall_flag": False,
                "emm_pulses_per_rev": 3200,
            } for i in (1, 2, 3, 4)
        },
    }
    Path(path).write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Bridge old hemn v1.0.1 FF-FF protocol to ZDT X42S Emm firmware")
    p.add_argument("--zdt-port", help="physical serial port connected to ZDT TTL/RS485 bus")
    p.add_argument("--zdt-baud", type=int, default=115200,
                   help="ZDT UART baud (default: 115200)")
    p.add_argument("--config", help="JSON motor mapping/calibration configuration")
    p.add_argument("--old-port",
                   help="old-protocol upstream serial port; omit to create a Unix PTY")
    p.add_argument("--old-baud", type=int, default=1_000_000,
                   help="upstream old-protocol baud for --old-port")
    p.add_argument("--pty-link",
                   help="optional symlink for generated PTY, e.g. /tmp/oldmotor")
    p.add_argument("--self-test", action="store_true", help="run codec self-tests and exit")
    p.add_argument("--write-example-config", metavar="PATH",
                   help="write a sample JSON config and exit")
    p.add_argument("-v", "--verbose", action="count", default=0)
    return p.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=log_level, format="%(asctime)s %(levelname)s %(message)s")

    if args.self_test:
        self_test()
        return 0
    if args.write_example_config:
        write_example_config(args.write_example_config)
        print(args.write_example_config)
        return 0
    if not args.zdt_port:
        print("error: --zdt-port is required (unless --self-test is used)", file=sys.stderr)
        return 2

    cfg = BridgeConfig.load(args.config)
    zdt = ZDTDriver(args.zdt_port, args.zdt_baud, cfg.zdt_timeout_s)
    translator = OldToZDTTranslator(zdt, cfg)
    bridge = StreamBridge(translator)
    try:
        if args.old_port:
            bridge.run_serial_upstream(args.old_port, args.old_baud)
        else:
            bridge.run_pty(args.pty_link)
    except KeyboardInterrupt:
        LOG.info("stopped")
    finally:
        zdt.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
