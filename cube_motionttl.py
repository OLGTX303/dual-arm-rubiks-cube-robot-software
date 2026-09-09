"""Emm_V5 0x6B compatibility entry point.

This file deliberately DOES NOT duplicate or modify the Rubik's-cube motion
algorithm. It loads the repository's original software/cube_motion.py unchanged
and replaces only its low-level motor command functions with the Emm_V5 adapter.

Physical motor mapping on the TTL bus:
    ID 1 = right rotating arm
    ID 2 = right jaw/finger
    ID 3 = left rotating arm
    ID 4 = left jaw/finger

The original software's internal/logical IDs remain exactly as written in
software/cube_motion.py. The mapping is performed only inside emm_v5_adapter.py.
"""

from pathlib import Path
import importlib.util
import sys

import emm_v5_adapter as _emm


_HERE = Path(__file__).resolve().parent
# cube_motion.py normally sits next to this file; the ../software/ layout is the
# historical location kept as a fallback.
_CANDIDATES = (_HERE / "cube_motion.py", _HERE.parent / "software" / "cube_motion.py")
_ORIGINAL_PATH = next((p for p in _CANDIDATES if p.is_file()), None)
if _ORIGINAL_PATH is None:
    raise ImportError(
        "Cannot find the original cube_motion.py; looked in: "
        + ", ".join(str(p) for p in _CANDIDATES))

_spec = importlib.util.spec_from_file_location("cube_motion_original_v11", _ORIGINAL_PATH)
if _spec is None or _spec.loader is None:
    raise ImportError(f"Cannot load original cube motion module: {_ORIGINAL_PATH}")

_original = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = _original
_spec.loader.exec_module(_original)

# -----------------------------------------------------------------------------
# ONLY these low-level control-command functions are replaced.
# Everything else, including MotionCtrl and cmd_zero(), is the exact implementation
# loaded from software/cube_motion.py.
# -----------------------------------------------------------------------------
_original.cmd_enable = _emm.cmd_enable
_original.cmd_stat = _emm.cmd_stat
_original.cmd_wait_motion = _emm.cmd_wait_motion
_original.cmd_get_pos = _emm.cmd_get_pos
_original.cmd_trap = _emm.cmd_trap
cmd_stop = _emm.cmd_stop
reset_command_positions = _emm.reset_command_positions

# Re-export the original module's public API so existing callers can use this file
# as cube_motion without any motion-algorithm changes.
for _name, _value in vars(_original).items():
    if not _name.startswith("__"):
        globals()[_name] = _value

# Make the active command functions explicit in this module too.
cmd_enable = _emm.cmd_enable
cmd_stat = _emm.cmd_stat
cmd_wait_motion = _emm.cmd_wait_motion
cmd_get_pos = _emm.cmd_get_pos
cmd_trap = _emm.cmd_trap


if __name__ == "__main__":
    print("softwarev1.1 compatibility layer loaded successfully.")
    print("Original motion logic:", _ORIGINAL_PATH)
    print("Only low-level motor commands are replaced by Emm_V5 0x6B protocol.")
    print("Physical TTL IDs: 1=right arm, 2=right jaw, 3=left arm, 4=left jaw")
    print("Use this module from the robot software/tests exactly as cube_motion.")
