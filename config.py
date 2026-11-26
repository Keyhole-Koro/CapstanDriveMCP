"""Tunable constants for the capstan joint. Every name carries its unit."""
from pathlib import Path

# ---- mechanics --------------------------------------------------------------
GEAR_RATIO = 8.0             # motor turns per drum turn (pinion 8:1)
MOTOR_DIR_UP = -1            # sign of ODrive motor turns when the drum moves UP
                             # (+motor = down, observed 2026-11-25; measure_range.py re-checks it)
TRAVEL_MOTOR_TURNS = 2.47    # bottom stop -> top stop, free travel. The stops are compliant: pushed hard
                             # by hand they gave 2.62 (2026-11-25), first contact was ~2.43-2.47.
MARGIN_MOTOR_TURNS = 0.12    # commanded targets stay this far from each stop
# check_top_here (drum held on the top stop by hand, motor off) judges the measured travel:
TOP_CHECK_MIN_CLEARANCE_DEG = 2.7  # the top stop must be at least this far above the allowed maximum
                                   # (half of the 5.4 deg margin)
TOP_CHECK_LONG_DEG = 9.0     # more than this beyond the configured travel is suspicious (a hard push
                             # gave +6.75 deg), e.g. the bottom reference is off or the drum is not on the stop

# ---- motion settings, applied at runtime only (never saved to the ODrive) ---
MOVE_VEL = 1.0               # trajectory top speed [motor turns/s]  (= 45 deg/s at the drum)
MOVE_ACCEL = 2.0             # [motor turns/s^2]
MOVE_DECEL = 2.0             # [motor turns/s^2]
CURRENT_LIMIT_A = 6.0        # motor current limit [A] (moving slowly needs ~1 A)
VEL_LIMIT = 3.0              # ODrive controller velocity limit [motor turns/s]
# Gains from tune_gains.py (2026-11-25). Max final error 0.164 deg with the ODrive defaults
# (20 / 0.1667 / 0.3333) -> 0.011 deg here. 40 / 0.7 / 3.5 was as accurate but at the noise limit.
POS_GAIN = 35.0              # position loop [(motor turns/s) / motor turn]
VEL_GAIN = 0.5               # velocity loop [Nm / (motor turns/s)]
VEL_INTEGRATOR_GAIN = 2.5    # velocity loop integrator [Nm / motor turn]

# ---- supervisor (runs while the motor is enabled) ---------------------------
SUPERVISOR_PERIOD_S = 0.02
WATCHDOG_TIMEOUT_S = 0.5     # ODrive disarms itself if this process stops feeding it
VEL_ABORT = 2.5              # disarm above this speed [motor turns/s]
WINDOW_SLACK_DEG = 3.0       # disarm if the drum leaves [0 - slack, travel + slack]
ENABLE_HOLD_TOL_TURNS = 0.03 # max drift allowed right after enabling [motor turns]
ENABLE_HOLD_S = 0.5
OVERLOAD_A = 3.5             # disarm when the median current over OVERLOAD_S reaches this: free moves
OVERLOAD_S = 0.3             # need <= 2.54 A (measure_load.py), so this means pushing on something

HOME_DEG = 6.0               # go_home parks here: just inside the allowed range above the bottom stop

# ---- move_to ----------------------------------------------------------------
ARRIVAL_TOL_DEG = 0.5        # must get this close, or the move fails and the motor is turned off [drum deg]
SETTLE_TIMEOUT_S = 2.0       # ... within the planned trajectory time plus this
PRECISION_TOL_DEG = 0.1      # then wait for the drum to settle within this [drum deg] ...
PRECISION_WAIT_S = 1.0       # ... for at most this long

# ---- referencing ---------------------------------------------------------------
# The bottom is set by hand (set_bottom_here). Motor homing into the compliant stops was tried and
# dropped on 2026-11-25: a friction spot was mistaken for the stop, and pushing on the stop with
# the tuned gains made the drum chatter. For automatic homing, fit a limit switch and use the
# ODrive's own homing instead.

# ---- refuse to run if the ODrive's limits are already above these -----------
MAX_SAVED_VEL_LIMIT = 8.0
MAX_SAVED_CURRENT_A = 10.0

REFERENCE_FILE = Path(__file__).with_name("reference.json")
TELEMETRY_DIR = Path(__file__).with_name("logs")  # JSON-lines motion log (plots, videos); None = off
