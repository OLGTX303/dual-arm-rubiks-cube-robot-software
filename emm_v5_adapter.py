"""Emm_V5 Rev1.3 TTL/UART 0x6B adapter for the ORIGINAL cube_motion.py.

This module changes ONLY the low-level motor-control protocol.
The high-level Rubik's-cube motion implementation stays in cube_motion.py.

Hardware transport:
    UART TTL only (TX, RX, GND), 115200 baud, 8-N-1.
    No RS485 transceiver, DE/RE direction pin, RTS toggling, or half-duplex code.

Original logical motor IDs used by cube_motion.py
(read off move_arm(): right -> [1, 2] = [finger, arm], left -> [3, 4]):
    1 = right jaw/finger
    2 = right rotating arm
    3 = left jaw/finger
    4 = left rotating arm

Physical Emm_V5 TTL addresses on this robot:
    1 = right rotating arm
    2 = right jaw/finger
    3 = left rotating arm
    4 = left jaw/finger

Therefore the mapping happens ONLY at this protocol boundary:
    logical 1 (right finger) -> physical 2
    logical 2 (right arm)    -> physical 1
    logical 3 (left finger)  -> physical 4
    logical 4 (left arm)     -> physical 3

Old-protocol features that Emm_V5 0xFD does not have natively and that are
therefore emulated here, because cube_motion.py depends on all of them:

    * absolute int32 multi-turn target coordinates  -> single-turn 0x31 origin
      plus cumulative 0x36 feedback, and every 0xFD is issued as a relative
      move recomputed from the measured position (no dead-reckoning drift).
    * command FIFO (back-to-back cmd_trap on one motor) -> software queue.
      Without this, arm_90_no_load()'s three chained finger segments overwrite
      each other and the jaw never opens before the arm spins unloaded.
    * cmd 0x03 "move until mechanically blocked" (cmd_zero) -> software stall
      detection + 0xFE 0x98 stop + 0x0E 0x52 stall-protection reset.
    * per-command max_current (15% homing, 40% cube clamp) -> non-persistent
      closed-loop current limit 0x45 0x66.

The old terminal-velocity field v1 has no 0xFD equivalent; queued segments
decelerate at segment boundaries instead of blending.

Protocol reference: Emm42_V5.0 Rev1.3 custom UART protocol.
Checksum mode: fixed 0x6B.
"""

import logging
import time

logger = logging.getLogger(__name__)

CHECKSUM = 0x6B
UART_BAUD = 115200

LOGICAL_TO_PHYSICAL = {
    1: 2,  # right jaw/finger   -> TTL ID 2
    2: 1,  # right rotating arm -> TTL ID 1
    3: 4,  # left jaw/finger    -> TTL ID 4
    4: 3,  # left rotating arm  -> TTL ID 3
}

# Keyed by PHYSICAL TTL id. True means the Emm_V5 rotation sense is opposite to
# the original controller's coordinate sense, so both the 0xFD direction bit and
# the 0x36 feedback are negated together (they can never disagree).
# These values mirror the field-calibrated old_to_zdt_emm_config.json, where
# old ids 1 and 2 (physical 2 and 1, the right side) have direction = -1.
# Verify at low speed before trusting them on new hardware.
PHYSICAL_DIR_INVERT = {
    1: True,   # right arm
    2: True,   # right finger
    3: False,  # left arm
    4: False,  # left finger
}

# Per-physical-motor calibration of the mechanical zero phase inside one
# revolution, in original 16384-counts units. Same meaning and units as
# old_to_zdt_emm_config.json "encoder_offset_counts".
ENCODER_OFFSET_COUNTS = {1: 0, 2: 0, 3: 0, 4: 0}

# Original custom controller exposes 16384 counts/motor-revolution.
OLD_COUNTS_PER_REV = 16384.0

# Emm_V5 0x36 real-time position is 65536 units/motor-revolution.
EMM_POSITION_UNITS_PER_REV = 65536.0

# Emm_V5 0xFD pulse field uses configured microstep pulses.
# Rev1.3 default: 200 full steps * 16 microsteps => 3200 pulses/revolution.
# Must match the microstep actually programmed into each driver.
EMM_PULSES_PER_REV = {1: 3200, 2: 3200, 3: 3200, 4: 3200}

# Emm_V5 0xFD speed field is RPM.
MAX_RPM = 3000

# Old max_current is a percentage. Full scale of the original driver.
OLD_FULL_SCALE_CURRENT_MA = 2500

# Motion-completion criteria.
#
# The old trap_status meant "the trajectory generator is still running", NOT
# "the encoder reached the target". cube_motion.py depends on that difference:
# two_finger_clamp() deliberately commands past the cube surface and grips by
# stalling against it, so the target is never reached and a position-only
# completion test would block forever. Emm_V5's 0x3A bit1 is the equivalent
# trajectory status, so it is authoritative once the axis has demonstrably
# stopped; the position window only exists to reject a bit reading left over
# from the PREVIOUS move.
REACH_TOLERANCE_COUNTS = 64    # 1.4 deg; check_arm_pos() allows 182 counts
POST_DISPATCH_BLIND_S = 0.004  # ignore a stale 0x3A in-position bit
SETTLE_S = 0.030               # standstill that proves the move really ended
MOTION_EPSILON_COUNTS = 8      # movement below this counts as standstill
# Standstill after which a trapezoid is declared finished even though the
# driver never raised its in-position bit. A closed-loop driver with Clog_Pro
# disabled pushes against an obstruction indefinitely and never sets that bit,
# so without this the intentional grip in two_finger_clamp() would block the
# caller forever. The old trajectory generator had always finished by then.
# 150 ms at the slowest speed cube_motion.py commands (30 RPM = 8192 counts/s)
# is ~1200 counts of travel, far above MOTION_EPSILON_COUNTS, so a slow but
# genuinely moving axis cannot trip it.
STALL_DONE_S = 0.150

# Emulated old cmd 0x03 (move until blocked).
ZERO_MIN_RUN_S = 0.080
ZERO_STALL_S = 0.080


class _Motion:
    """One queued/active old-protocol trapezoid command."""

    __slots__ = ("kind", "target", "vmax", "accel", "current_pct",
                 "start_pos", "dispatched_at", "last_pos", "last_moving_at")

    def __init__(self, kind, target, vmax, accel, current_pct):
        self.kind = kind              # "trap" or "zero"
        self.target = int(target)
        self.vmax = int(vmax)
        self.accel = int(accel)
        self.current_pct = int(current_pct)
        self.start_pos = None
        self.dispatched_at = 0.0
        self.last_pos = None
        self.last_moving_at = 0.0


class _MotorState:
    __slots__ = ("origin_old", "origin_emm", "coord_ready",
                 "active", "queue", "last_current_ma")

    def __init__(self):
        self.origin_old = 0.0
        self.origin_emm = 0
        self.coord_ready = False
        self.active = None
        self.queue = []
        self.last_current_ma = None


_state = {}


def _motor(logical_id):
    return _state.setdefault(int(logical_id), _MotorState())


def reset_command_positions():
    """Forget cached coordinates/queues, e.g. after a driver reboot."""
    _state.clear()


def _physical_id(logical_id):
    logical_id = int(logical_id)
    if logical_id not in LOGICAL_TO_PHYSICAL:
        raise ValueError(f"unsupported logical motor id {logical_id}")
    return LOGICAL_TO_PHYSICAL[logical_id]


def _dir_sign(physical_id):
    return -1 if PHYSICAL_DIR_INVERT.get(physical_id, False) else 1


def _pulses_per_rev(physical_id):
    return max(1, int(EMM_PULSES_PER_REV.get(physical_id, 3200)))


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


def _old_count_to_emm_pulses(count, physical_id):
    """Original 16384-count delta -> Emm_V5 0xFD pulse count."""
    return int(round(float(count) * _pulses_per_rev(physical_id) / OLD_COUNTS_PER_REV))


def _emm_units_to_old_count(position_units):
    """Emm_V5 0x36 65536/rev units -> original 16384/rev counts."""
    return float(position_units) * OLD_COUNTS_PER_REV / EMM_POSITION_UNITS_PER_REV


def _current_ma(pct):
    pct = max(0, min(100, int(pct)))
    return max(0, min(5000, int(round(OLD_FULL_SCALE_CURRENT_MA * pct / 100.0))))


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
        + _u16be(min(abs(int(rpm)), MAX_RPM))
        + bytes([max(0, min(int(acceleration), 255))])
        + bytes([0x01 if sync else 0x00, CHECKSUM])
    )


def build_position(physical_id, pulses_signed, rpm, acceleration,
                   absolute=False, sync=False):
    """Build the exact Rev1.3 0xFD frame.

    ID + FD + DIR + SPEED(2) + ACC(1) + PULSES(4) + ABS/REL + SYNC + 6B

    Total length = 13 bytes.

    pulses_signed is a SIGNED pulse count in Emm_V5 command units. The sign
    selects the direction byte; Rev1.3 only defines 0x00 = relative and
    0x01 = absolute for the ABS/REL byte.
    """
    pulses_signed = int(pulses_signed)
    direction = 0x01 if pulses_signed < 0 else 0x00  # manual: 00=CW, 01=CCW
    if PHYSICAL_DIR_INVERT.get(physical_id, False):
        direction ^= 0x01

    return (
        bytes([physical_id, 0xFD, direction])
        + _u16be(min(abs(int(rpm)), MAX_RPM))
        + bytes([max(0, min(int(acceleration), 255))])
        + _u32be(abs(pulses_signed))
        + bytes([
            0x01 if absolute else 0x00,
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


def build_reset_clog_protection(physical_id):
    # ID 0E 52 6B
    return bytes([physical_id, 0x0E, 0x52, CHECKSUM])


def build_set_max_current(physical_id, current_ma, store=False):
    # ID 45 66 STORE CURRENT(2) 6B - non-persistent closed-loop current limit
    return (
        bytes([physical_id, 0x45, 0x66, 0x01 if store else 0x00])
        + _u16be(max(0, min(int(current_ma), 5000)))
        + bytes([CHECKSUM])
    )


def build_read_position(physical_id):
    # ID 36 6B
    return bytes([physical_id, 0x36, CHECKSUM])


def build_read_encoder(physical_id):
    # ID 31 6B - linearised single-turn encoder, 0..65535
    return bytes([physical_id, 0x31, CHECKSUM])


def build_read_flags(physical_id):
    # ID 3A 6B
    return bytes([physical_id, 0x3A, CHECKSUM])


def parse_ack(resp, physical_id, function_code):
    """Parse control acknowledgement: ID FUNC STATUS 6B.

    STATUS 0x02 = command accepted
    STATUS 0xE2 = condition not met (e.g. disabled/stall protection)
    A malformed command is answered with the 3-byte frame 00 EE 6B.
    """
    frame = _find_frame(resp, physical_id, function_code, 4)
    if frame is not None:
        return frame[2]
    # Malformed-command frame carries address 0x00 and function 0xEE.
    if _find_frame(resp, 0x00, 0xEE, 3) is not None:
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
    if status in (0x02, 0x9F):
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
    """Return signed Emm_V5 cumulative position in 65536-units/revolution."""
    _drain(ser)
    _write(ser, build_read_position(physical_id), "read-position")
    resp = _read_until_quiet(ser, first_timeout=0.12)
    frame = _find_frame(resp, physical_id, 0x36, 8)
    if frame is None:
        return None
    sign = frame[2]
    magnitude = int.from_bytes(frame[3:7], "big", signed=False)
    return -magnitude if sign else magnitude


def read_encoder_raw(ser, physical_id):
    """Return the linearised single-turn encoder value, 0..65535."""
    _drain(ser)
    _write(ser, build_read_encoder(physical_id), "read-encoder")
    resp = _read_until_quiet(ser, first_timeout=0.12)
    frame = _find_frame(resp, physical_id, 0x31, 5)
    if frame is None:
        return None
    return int.from_bytes(frame[2:4], "big", signed=False)


def read_flags(ser, physical_id):
    """Return Emm_V5 0x3A status byte or None.

    bit0 enable, bit1 reached/in-position, bit2 stall, bit3 stall-protection.
    """
    _drain(ser)
    _write(ser, build_read_flags(physical_id), "read-flags")
    resp = _read_until_quiet(ser, first_timeout=0.12)
    frame = _find_frame(resp, physical_id, 0x3A, 4)
    return None if frame is None else frame[2]


def reset_clog_protection(ser, physical_id):
    return _send_control_and_check(
        ser, build_reset_clog_protection(physical_id), physical_id, 0x0E)


# -----------------------------------------------------------------------------
# Original absolute multi-turn coordinate emulation
# -----------------------------------------------------------------------------
def _ensure_coord(ser, logical_id):
    """Anchor the original coordinate system to the mechanical encoder phase.

    The original controller reported an absolute int32 position whose phase
    inside one revolution was mechanically meaningful; cube_motion.py relies on
    that in cmd_zero() via ARM2_ZERO / ARM4_ZERO. Emm_V5's 0x36 origin is
    arbitrary (power-on / last zero), so the single-turn linearised encoder
    0x31 supplies the phase and 0x36 supplies the multi-turn accumulation.
    """
    st = _motor(logical_id)
    if st.coord_ready:
        return True
    physical_id = _physical_id(logical_id)
    enc = read_encoder_raw(ser, physical_id)
    emm = read_position_raw(ser, physical_id)
    if enc is None or emm is None:
        return False
    single = int(round(enc * OLD_COUNTS_PER_REV / EMM_POSITION_UNITS_PER_REV))
    single = (single + ENCODER_OFFSET_COUNTS.get(physical_id, 0)) % int(OLD_COUNTS_PER_REV)
    st.origin_old = float(single)
    st.origin_emm = int(emm)
    st.coord_ready = True
    logger.info("coord logical=%d physical=%d encoder=%d origin_old=%d dir=%+d",
                logical_id, physical_id, enc, single, _dir_sign(physical_id))
    return True


# -----------------------------------------------------------------------------
# Motion state machine (emulates the original command FIFO and cmd 0x03)
# -----------------------------------------------------------------------------
def _apply_current(ser, logical_id, physical_id, pct):
    """Emulate the old per-command max_current field.

    0xFD has no current field. 0x45 0x66 changes the closed-loop current limit
    without storing it, which is the closest Emm_V5 equivalent and is required
    for the 15% homing push and the 40% cube clamp to behave like the original.
    """
    st = _motor(logical_id)
    want = _current_ma(pct)
    if st.last_current_ma == want:
        return True
    ok = _send_control_and_check(
        ser, build_set_max_current(physical_id, want, store=False),
        physical_id, 0x45)
    if ok:
        st.last_current_ma = want
    else:
        # Keep going at the driver's configured limit rather than aborting the
        # solve; the limit is an approximation of the old field anyway.
        st.last_current_ma = None
        logger.warning("could not set current limit on physical ID %d", physical_id)
    return True


def _dispatch(ser, logical_id, motion, sync):
    """Issue one 0xFD relative move recomputed from the measured position."""
    physical_id = _physical_id(logical_id)
    # Deliberately the raw read, not cmd_get_pos(): cmd_get_pos() drives the
    # state machine, which is what called us.
    current = _read_old_pos(ser, logical_id)
    if current is None:
        return False

    _apply_current(ser, logical_id, physical_id, motion.current_pct)

    # Signed delta stays in ORIGINAL coordinates here; build_position() is the
    # single place that maps a coordinate sign onto the wire direction bit
    # (applying PHYSICAL_DIR_INVERT). Applying the sign here too would cancel
    # the inversion out.
    delta_old = motion.target - int(current)
    pulses = _old_count_to_emm_pulses(delta_old, physical_id)
    frame = build_position(physical_id, pulses, motion.vmax, motion.accel,
                           absolute=False, sync=sync)
    if not _send_control_and_check(ser, frame, physical_id, 0xFD):
        # The original controller had no latching stall protection: it simply
        # pushed at the commanded current limit. Emm_V5 latches, disables the
        # motor and then answers every 0xFD with 0xE2, which would strand the
        # solve (e.g. after the intentional stall of the cube clamp). Clear the
        # latch, re-enable and retry once to restore the old behaviour.
        flags = read_flags(ser, physical_id)
        if flags is not None and not (flags & 0x0C) and (flags & 0x01):
            return False
        logger.warning("physical ID %d rejected 0xFD (flags=%s); clearing stall "
                       "protection and retrying",
                       physical_id, "None" if flags is None else f"0x{flags:02X}")
        reset_clog_protection(ser, physical_id)
        _send_control_and_check(ser, build_enable(physical_id, True, sync=False),
                                physical_id, 0xF3)
        if not _send_control_and_check(ser, frame, physical_id, 0xFD):
            return False

    now = time.time()
    motion.start_pos = int(current)
    motion.dispatched_at = now
    motion.last_pos = int(current)
    motion.last_moving_at = now
    logger.debug("dispatch logical=%d kind=%s target=%d delta=%d pulses=%d "
                 "vmax=%d acc=%d cur=%d%% sync=%d",
                 logical_id, motion.kind, motion.target, delta_old, pulses,
                 motion.vmax, motion.accel, motion.current_pct, int(sync))
    return True


def _start_next(ser, logical_id):
    st = _motor(logical_id)
    if st.active is not None or not st.queue:
        return True
    motion = st.queue.pop(0)
    st.active = motion
    if not _dispatch(ser, logical_id, motion, sync=False):
        st.active = None
        return False
    return True


def _finish(ser, logical_id):
    _motor(logical_id).active = None
    _start_next(ser, logical_id)


def _pump(ser, logical_id, pos, flags=None):
    """Advance the software FIFO / zero state machine. Returns trap_status."""
    st = _motor(logical_id)
    motion = st.active
    if motion is None:
        _start_next(ser, logical_id)
        st = _motor(logical_id)
        return 1 if (st.active is not None or st.queue) else 0

    now = time.time()
    if now - motion.dispatched_at < POST_DISPATCH_BLIND_S:
        # The 0x3A in-position bit can still describe the PREVIOUS move.
        return 1

    within = abs(pos - motion.target) <= REACH_TOLERANCE_COUNTS
    in_position = None if flags is None else bool(flags & 0x02)

    # Standstill tracking, shared by both command types.
    if motion.last_pos is None or \
            abs(pos - motion.last_pos) > MOTION_EPSILON_COUNTS:
        motion.last_pos = pos
        motion.last_moving_at = now
    still_for = now - motion.last_moving_at

    if motion.kind == "zero":
        # Old cmd 0x03: run toward the target until mechanically blocked.
        if in_position and within:
            logger.info("emulated ZERO reached full target without a stall, "
                        "logical=%d", logical_id)
            _finish(ser, logical_id)
        else:
            blocked = bool(flags & 0x0C) if flags is not None else False
            if (now - motion.dispatched_at) >= ZERO_MIN_RUN_S and \
                    (still_for >= ZERO_STALL_S or blocked):
                physical_id = _physical_id(logical_id)
                logger.info("emulated ZERO stall logical=%d pos=%d; stopping",
                            logical_id, pos)
                _send_control_and_check(ser, build_stop(physical_id, sync=False),
                                        physical_id, 0xFE)
                # Emm_V5 latches stall protection and stays disabled until it is
                # cleared, which would reject every later 0xFD with 0xE2.
                if flags is None or (flags & 0x0C):
                    reset_clog_protection(ser, physical_id)
                    _send_control_and_check(
                        ser, build_enable(physical_id, True, sync=False),
                        physical_id, 0xF3)
                _finish(ser, logical_id)
    else:
        if in_position is None:
            # 0x3A unavailable: fall back to the position window alone.
            if within or still_for >= SETTLE_S:
                _finish(ser, logical_id)
        elif (in_position and (within or still_for >= SETTLE_S)) or \
                still_for >= STALL_DONE_S:
            if not within:
                # Normal for the cube clamp; suspicious for an arm move, which
                # cube_motion.check_arm_pos() will catch and report.
                logger.debug("logical %d stopped %d counts short of %d "
                             "(in_position=%s)", logical_id,
                             motion.target - pos, motion.target, in_position)
            _finish(ser, logical_id)
        elif flags is not None and not (flags & 0x01):
            # The driver disabled itself mid-move (stall protection). The old
            # controller had no such state; do not block the caller forever.
            logger.error("logical %d disabled mid-move at %d (target %d)",
                         logical_id, pos, motion.target)
            _finish(ser, logical_id)

    st = _motor(logical_id)
    return 1 if (st.active is not None or st.queue) else 0


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
        if not en:
            st = _motor(logical_id)
            st.active = None
            st.queue = []
        time.sleep(0.002)
    return True


def _read_old_pos(ser, logical_id):
    """Measured position in original coordinates. Pure read, no state machine."""
    if not _ensure_coord(ser, logical_id):
        logger.debug("cannot anchor coordinates for logical motor %d", logical_id)
        return None
    physical_id = _physical_id(logical_id)
    raw = read_position_raw(ser, physical_id)
    if raw is None:
        logger.debug("no valid 0x36 response: logical=%d physical=%d",
                     logical_id, physical_id)
        return None
    st = _motor(logical_id)
    delta = _emm_units_to_old_count(raw - st.origin_emm) * _dir_sign(physical_id)
    return int(round(st.origin_old + delta))


def cmd_get_pos(ser, logical_id):
    """Measured position, and advance this motor's software FIFO.

    The FIFO must be advanced here and not only in cmd_stat(), because
    cube_motion.py issues two-motor traps and then waits on only one of them
    (move_two_finger_raw -> cmd_wait_motion(3)), polling the other through
    wait_motion_by_pos()/check_arm_pos(), which call cmd_get_pos() alone. If
    only cmd_stat() retired motions, the un-waited motor would keep a finished
    motion marked active forever and its next command would sit in the queue
    unsent, deadlocking those polls until 运动控制超时.

    No 0x3A read is added here: completion falls back to the position window
    plus standstill, which keeps this on the original single-round-trip cost.
    """
    pos = _read_old_pos(ser, logical_id)
    if pos is None:
        return None
    _pump(ser, logical_id, pos, None)
    return pos


def cmd_stat(ser, logical_id):
    """Return original status shape: [count, trap_status, temp, pos, voltage].

    The old motion code only depends on trap_status and pos. Emm_V5's real
    status is read with 0x3A instead of estimating motion from elapsed time.
    """
    physical_id = _physical_id(logical_id)
    pos = _read_old_pos(ser, logical_id)
    if pos is None:
        return None
    flags = read_flags(ser, physical_id)
    trap_status = _pump(ser, logical_id, pos, flags)
    return [0, trap_status, 0, pos, 0]


def cmd_wait_motion(ser, logical_id):
    """Same blocking contract used by the original high-level software."""
    logger.debug("等待%d号控制板完成运动控制", logical_id)
    start_time = time.time()
    newest = None
    # The original function has no timeout; keep behaviour bounded only against
    # a disconnected TTL bus so a missing cable cannot hang the process forever.
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

    `zero` selects the original command type: 0x02 trapezoid, 0x03 move-until-
    blocked. Both are supported here; 0x03 is emulated with software stall
    detection because Emm_V5's own homing commands would reset the driver
    coordinate system, which cube_motion.py's cmd_zero() does not expect.

    The original absolute target, vmax and acceleration values are preserved
    and converted only at the wire-format boundary. `end_speed` (v1) has no
    0xFD equivalent, so queued segments decelerate at segment boundaries.
    """
    if len(id_list) != len(trap_list):
        raise ValueError("id_list and trap_list length mismatch")

    kind = "zero" if zero else "trap"
    pending = []
    for logical_id, trap in zip(id_list, trap_list):
        target, _end_speed, vmax, accel, max_current = trap
        _physical_id(logical_id)  # validate early
        pending.append((int(logical_id),
                        _Motion(kind, target, vmax, accel, max_current)))

    # The original controller had a per-motor FIFO. If any motor in this batch
    # is still busy, queue the WHOLE batch so a gear-coupled finger/arm pair is
    # never split into an unsynchronised pair of moves.
    busy = any(_motor(lid).active is not None or _motor(lid).queue
               for lid, _ in pending)
    if busy:
        for logical_id, motion in pending:
            _motor(logical_id).queue.append(motion)
            logger.debug("queued logical=%d kind=%s depth=%d", logical_id, kind,
                         len(_motor(logical_id).queue))
        return True

    sync = len(pending) > 1
    for logical_id, motion in pending:
        _motor(logical_id).active = motion
    for logical_id, motion in pending:
        if not _dispatch(ser, logical_id, motion, sync):
            # Never leave a motion marked active that was not started.
            for lid, _ in pending:
                _motor(lid).active = None
            return False
        time.sleep(0.002)

    if sync:
        # Exact Rev1.3 multi-motor sequence: queue each 0xFD with sync=01,
        # then broadcast 00 FF 66 6B.
        _write(ser, build_sync_start(0x00), "sync-start")
        # With the broadcast address only ID1 may reply; do not require an ACK.
        _read_until_quiet(ser, first_timeout=0.025)

    return True


def cmd_stop(ser, id_list):
    """TTL immediate stop helper using 0xFE."""
    ok_all = True
    for logical_id in id_list:
        physical_id = _physical_id(logical_id)
        st = _motor(logical_id)
        st.active = None
        st.queue = []
        if not _send_control_and_check(
            ser, build_stop(physical_id, sync=False), physical_id, 0xFE
        ):
            ok_all = False
    return ok_all


def cmd_velocity(ser, logical_id, direction, rpm, acceleration, sync=False):
    """Complete TTL velocity-mode helper (0xF6); not required by MotionCtrl today."""
    physical_id = _physical_id(logical_id)
    return _send_control_and_check(
        ser,
        build_velocity(physical_id, direction, rpm, acceleration, sync=sync),
        physical_id,
        0xF6,
    )
