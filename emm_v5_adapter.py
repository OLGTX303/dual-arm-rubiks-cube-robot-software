"""Emm_V5 Rev1.3 TTL/UART 0x6B adapter for the ORIGINAL cube_motion.py.

This module changes ONLY the low-level motor-control protocol.
The high-level Rubik's-cube motion implementation stays in software/cube_motion.py.

Hardware transport:
    UART TTL only (TX, RX, GND), 115200 baud, 8-N-1.
    No RS485 transceiver, DE/RE direction pin, RTS toggling, or half-duplex code.

Original logical motor IDs used by software/cube_motion.py:
    1 = right jaw/finger
    2 = right rotating arm
    3 = left rotating arm
    4 = left jaw/finger

Physical Emm_V5 TTL addresses requested for the new robot:
    1 = right rotating arm
    2 = right jaw/finger
    3 = left rotating arm
    4 = left jaw/finger

Therefore the mapping happens ONLY at this protocol boundary:
    logical 1 -> physical 2
    logical 2 -> physical 1
    logical 3 -> physical 4
    logical 4 -> physical 3

Protocol reference: Emm42_V5.0 Rev1.3 custom UART protocol.
Checksum mode: fixed 0x6B.
"""

import logging
import time

logger = logging.getLogger(__name__)

CHECKSUM = 0x6B
UART_BAUD = 115200

LOGICAL_TO_PHYSICAL = {
    1: 1,  # right finger
    2: 2,  # right arm
    3: 4,  # left finger -> physical 4
    4: 3,  # left arm -> physical 3
}

# Adjust only after low-speed physical direction verification.
PHYSICAL_DIR_INVERT = {
    # Mechanical rule: CW opens the finger, CCW closes it.
    # Both sides follow the same original direction rule:
    # CW opens/release, CCW closes/clamp.
    1: False,
    2: False,
    3: False,
    4: False,
}

# Original custom controller exposes 16384 counts/motor-revolution.
OLD_COUNTS_PER_REV = 16384.0

# Emm_V5 0x36 real-time position is 65536 units/motor-revolution.
EMM_POSITION_UNITS_PER_REV = 65536.0

# Emm_V5 0xFD pulse field uses configured microstep pulses.
# Rev1.3 example: 16 microsteps => 3200 pulses/revolution.
EMM_MICROSTEP = 16
EMM_FULL_STEPS_PER_REV = 200
EMM_COMMAND_PULSES_PER_REV = EMM_FULL_STEPS_PER_REV * EMM_MICROSTEP

# Track targets in ORIGINAL logical coordinates so MotionCtrl remains unchanged.
_last_target = {}
_command_pos = {}
_primed = set()


def reset_command_positions():
    _last_target.clear()
    _command_pos.clear()
    _primed.clear()



def _physical_id(logical_id):
    logical_id = int(logical_id)
    if logical_id not in LOGICAL_TO_PHYSICAL:
        raise ValueError(f"unsupported logical motor id {logical_id}")
    return LOGICAL_TO_PHYSICAL[logical_id]


def _u16be(value):
    value = max(0, min(int(value), 0xFFFF))
    return bytes([(value >> 8) & 0xFF, value & 0xFF])


def _u32be(value):
    value = max(0, min(int(value), 0xFFFFFFFF))
    return bytes([
        (value >> 24) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 8) & 0xFF,
        value & 0xFF,
    ])


def _old_count_to_emm_pulses(count):
    """Original 16384-count absolute coordinate -> Emm_V5 FD pulse coordinate."""
    return int(round(float(count) * EMM_COMMAND_PULSES_PER_REV / OLD_COUNTS_PER_REV))


def _emm_position_to_old_count(position_units):
    """Emm_V5 0x36 65536/rev position -> original 16384/rev coordinate."""
    return int(round(float(position_units) * OLD_COUNTS_PER_REV / EMM_POSITION_UNITS_PER_REV))


def _write(ser, frame, label="cmd"):
    if logger.isEnabledFor(logging.DEBUG):
        logger.debug("%s TX: %s", label, frame.hex(" "))
    ser.write(frame)
    try:
        ser.flush()
    except Exception:
        pass


def _drain(ser):
    try:
        ser.reset_input_buffer()
    except Exception:
        pass


def _read_until_quiet(ser, first_timeout=0.20, quiet_time=0.006, max_len=64):
    deadline = time.time() + first_timeout
    last_rx = None
    data = bytearray()
    while time.time() < deadline and len(data) < max_len:
        waiting = getattr(ser, "in_waiting", 0)
        if waiting:
            data.extend(ser.read(min(waiting, max_len - len(data))))
            last_rx = time.time()
        elif data and last_rx is not None and (time.time() - last_rx) >= quiet_time:
            break
        else:
            time.sleep(0.001)
    if logger.isEnabledFor(logging.DEBUG) and data:
        logger.debug("RX: %s", bytes(data).hex(" "))
    return bytes(data)


def _find_frame(resp, physical_id, function_code, frame_len):
    for i in range(0, max(0, len(resp) - frame_len + 1)):
        if resp[i] == physical_id and resp[i + 1] == function_code:
            frame = resp[i:i + frame_len]
            if len(frame) == frame_len and frame[-1] == CHECKSUM:
                return frame
    return None


# -----------------------------------------------------------------------------
# Exact Emm_V5 Rev1.3 TTL command builders
# -----------------------------------------------------------------------------
def build_enable(physical_id, enable=True, sync=False):
    # ID F3 AB EN SYNC 6B
    return bytes([
        physical_id,
        0xF3,
        0xAB,
        0x01 if enable else 0x00,
        0x01 if sync else 0x00,
        CHECKSUM,
    ])


def build_velocity(physical_id, direction, rpm, acceleration, sync=False):
    # ID F6 DIR SPEED_H SPEED_L ACC SYNC 6B
    direction = 0x01 if direction else 0x00
    if PHYSICAL_DIR_INVERT.get(physical_id, False):
        direction ^= 0x01
    return (
        bytes([physical_id, 0xF6, direction])
        + _u16be(abs(int(rpm)))
        + bytes([max(0, min(int(acceleration), 255))])
        + bytes([0x01 if sync else 0x00, CHECKSUM])
    )


def build_position(physical_id, target_old_count, rpm, acceleration,
                   absolute=True, sync=False):
    """Build the exact Rev1.3 0xFD frame.

    ID + FD + DIR + SPEED(2) + ACC(1) + PULSES(4) + ABS/REL + SYNC + 6B

    Total length = 13 bytes.
    """
    pulse_target = _old_count_to_emm_pulses(target_old_count)
    direction = 0x01 if pulse_target < 0 else 0x00  # manual: 00=CW, 01=CCW
    if PHYSICAL_DIR_INVERT.get(physical_id, False):
        direction ^= 0x01

    return (
        bytes([physical_id, 0xFD, direction])
        + _u16be(abs(int(rpm)))
        + bytes([max(0, min(int(acceleration), 255))])
        + _u32be(abs(pulse_target))
        + bytes([
            0x01 if absolute is True else (0x02 if absolute == 'current' else 0x00),
            0x01 if sync else 0x00,
            CHECKSUM,
        ])
    )


def build_stop(physical_id, sync=False):
    # ID FE 98 SYNC 6B
    return bytes([
        physical_id,
        0xFE,
        0x98,
        0x01 if sync else 0x00,
        CHECKSUM,
    ])


def build_sync_start(address=0x00):
    # Manual multi-motor example uses broadcast: 00 FF 66 6B
    return bytes([address, 0xFF, 0x66, CHECKSUM])


def build_read_position(physical_id):
    # ID 36 6B
    return bytes([physical_id, 0x36, CHECKSUM])


def build_read_flags(physical_id):
    # ID 3A 6B
    return bytes([physical_id, 0x3A, CHECKSUM])


def parse_ack(resp, physical_id, function_code):
    """Parse control acknowledgement: ID FUNC STATUS 6B.

    STATUS 0x02 = command accepted
    STATUS 0xE2 = condition not met (e.g. disabled/stall protection)
    malformed command may return ID 00 EE 6B.
    """
    frame = _find_frame(resp, physical_id, function_code, 4)
    if frame is not None:
        return frame[2]
    err = _find_frame(resp, physical_id, 0x00, 4)
    if err is not None and err[2] == 0xEE:
        return 0xEE
    return None


def _send_control_and_check(ser, frame, physical_id, function_code,
                            response_timeout=0.035):
    _write(ser, frame)
    resp = _read_until_quiet(ser, first_timeout=response_timeout)
    if not resp:
        # Response=None is valid if driver Response menu is configured to nNone.
        return True
    status = parse_ack(resp, physical_id, function_code)
    if status is None:
        # Do not reject unrelated/stale bytes; query commands will validate exactly.
        return True
    if status == 0x02:
        return True
    if status == 0xE2:
        logger.error("Emm_V5 ID %d function 0x%02X rejected: condition not met",
                     physical_id, function_code)
        return False
    if status == 0xEE:
        logger.error("Emm_V5 ID %d reported malformed command 0x%02X",
                     physical_id, function_code)
        return False
    logger.error("Emm_V5 ID %d function 0x%02X returned status 0x%02X",
                 physical_id, function_code, status)
    return False


def read_position_raw(ser, physical_id):
    """Return signed Emm_V5 real-time position in 65536-units/revolution."""
    _drain(ser)
    _write(ser, build_read_position(physical_id), "read-position")
    resp = _read_until_quiet(ser, first_timeout=0.12)
    frame = _find_frame(resp, physical_id, 0x36, 8)
    if frame is None:
        return None
    sign = frame[2]
    magnitude = int.from_bytes(frame[3:7], "big", signed=False)
    value = -magnitude if sign else magnitude
    if PHYSICAL_DIR_INVERT.get(physical_id, False):
        value = -value
    return value


def read_flags(ser, physical_id):
    """Return Emm_V5 0x3A status byte or None.

    bit0 enable, bit1 reached/in-position, bit2 stall, bit3 stall-protection.
    """
    _drain(ser)
    _write(ser, build_read_flags(physical_id), "read-flags")
    resp = _read_until_quiet(ser, first_timeout=0.12)
    frame = _find_frame(resp, physical_id, 0x3A, 4)
    return None if frame is None else frame[2]


# -----------------------------------------------------------------------------
# Drop-in API expected by the UNCHANGED original cube_motion.py
# -----------------------------------------------------------------------------
def cmd_enable(ser, id_list, en):
    for logical_id in id_list:
        physical_id = _physical_id(logical_id)
        ok = _send_control_and_check(
            ser,
            build_enable(physical_id, bool(en), sync=False),
            physical_id,
            0xF3,
        )
        if not ok:
            return False
        time.sleep(0.002)
    return True


def cmd_get_pos(ser, logical_id):
    physical_id = _physical_id(logical_id)
    raw = read_position_raw(ser, physical_id)
    if raw is None:
        logger.debug("no valid 0x36 response: logical=%d physical=%d",
                     logical_id, physical_id)
        return None
    value = _emm_position_to_old_count(raw)
    _command_pos.setdefault(int(logical_id), value)
    return value


def cmd_stat(ser, logical_id):
    """Return original status shape: [count, trap_status, temp, pos, voltage].

    The old motion code only depends on trap_status and pos.
    Emm_V5's real status is read with 0x3A instead of estimating motion from time.
    """
    physical_id = _physical_id(logical_id)
    pos = cmd_get_pos(ser, logical_id)
    if pos is None:
        return None

    flags = read_flags(ser, physical_id)
    if flags is None:
        # Fallback only when status query itself is unavailable.
        target = _last_target.get(logical_id)
        if target is None:
            trap_status = 0
        else:
            tolerance = max(8, int(round(OLD_COUNTS_PER_REV * 0.3 / 360.0)))
            trap_status = 0 if abs(pos - target) <= tolerance else 1
    else:
        reached = bool(flags & 0x02)
        stalled = bool(flags & 0x04)
        protected = bool(flags & 0x08)
        if stalled or protected:
            # A stopped/stalled jaw during the original mechanical-zero routine
            # must be allowed to return, just like the original controller.
            trap_status = 0
        else:
            trap_status = 0 if reached else 1

    return [0, trap_status, 0, pos, 0]


def cmd_wait_motion(ser, logical_id):
    """Same blocking contract used by the original high-level software."""
    logger.debug("等待%d号控制板完成运动控制", logical_id)
    start_time = time.time()
    newest = None
    # Original function has no timeout; keep behavior bounded only against a
    # disconnected TTL bus so a missing cable cannot hang the Linux process forever.
    comm_deadline = start_time + 5.0
    motion_deadline = start_time + 20.0
    while True:
        if time.time() > motion_deadline:
            raise TimeoutError(f"Emm_V5 motion timeout for logical motor {logical_id}")
        resp = cmd_stat(ser, logical_id)
        if resp is None:
            if time.time() > comm_deadline:
                raise TimeoutError(f"Emm_V5 TTL no response from logical motor {logical_id}")
            time.sleep(0.002)
            continue
        newest = resp[3]
        if resp[1] == 0:
            break
        time.sleep(0.002)
    logger.debug("耗时: %.2fms", 1000 * (time.time() - start_time))
    return newest


def cmd_trap(ser, id_list, zero, trap_list):
    """Translate the original trapezoid API to Emm_V5 0xFD without changing callers.

    Original item:
        [absolute_target_count, end_speed, vmax_rpm, accel_level, max_current]

    Emm_V5 Rev1.3 0xFD supports:
        direction, vmax RPM, acceleration level, pulse count,
        relative/absolute flag, synchronous-start flag.

    The original absolute target, vmax and acceleration values are preserved and
    converted only at the wire-format boundary. `end_speed` and per-command
    `max_current` do not exist in 0xFD and therefore cannot be encoded in this
    protocol frame; driver stall-current/protection must be configured in Emm_V5.
    """
    if len(id_list) != len(trap_list):
        raise ValueError("id_list and trap_list length mismatch")

    sync = len(id_list) > 1
    queued_physical_ids = []

    for logical_id, trap in zip(id_list, trap_list):
        target, _end_speed, vmax, accel, _max_current = trap
        physical_id = _physical_id(logical_id)
        _last_target[logical_id] = int(target)

        # Keep continuous original coordinates and use mode 00 (relative to
        # the previous target); mode 02 stops at the signed position boundary.
        current = _command_pos.get(int(logical_id))
        if current is None:
            current = cmd_get_pos(ser, logical_id)
        if current is None:
            return False
        delta = int(target) - int(current)
        # Establish the driver's mode-02 baseline once per motor/session.
        # Then use mode-00 so repeated turns continue from the previous target.
        if int(logical_id) not in _primed:
            baseline = build_position(
                physical_id=physical_id,
                target_old_count=0,
                rpm=vmax,
                acceleration=accel,
                absolute='current',
                sync=False,
            )
            if not _send_control_and_check(ser, baseline, physical_id, 0xFD):
                return False
            _primed.add(int(logical_id))

        frame = build_position(
            physical_id=physical_id,
            target_old_count=delta,
            rpm=vmax,
            acceleration=accel,
            absolute=False,
            sync=sync,
        )
        ok = _send_control_and_check(ser, frame, physical_id, 0xFD)
        if not ok:
            return False
        _command_pos[int(logical_id)] = int(target)
        queued_physical_ids.append(physical_id)
        time.sleep(0.002)

    if sync and queued_physical_ids:
        # Exact Rev1.3 multi-motor sequence: queue each FD with sync=01,
        # then broadcast 00 FF 66 6B.
        _write(ser, build_sync_start(0x00), "sync-start")
        # With broadcast address only ID1 may reply; do not require an ACK here.
        _read_until_quiet(ser, first_timeout=0.025)

    return True


def cmd_stop(ser, id_list):
    """TTL immediate stop helper using 0xFE."""
    for logical_id in id_list:
        physical_id = _physical_id(logical_id)
        if not _send_control_and_check(
            ser, build_stop(physical_id, sync=False), physical_id, 0xFE
        ):
            return False
    return True


def cmd_velocity(ser, logical_id, direction, rpm, acceleration, sync=False):
    """Complete TTL velocity-mode helper (0xF6); not required by MotionCtrl today."""
    physical_id = _physical_id(logical_id)
    return _send_control_and_check(
        ser,
        build_velocity(physical_id, direction, rpm, acceleration, sync=sync),
        physical_id,
        0xF6,
    )
