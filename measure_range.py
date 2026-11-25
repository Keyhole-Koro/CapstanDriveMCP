"""Measure the drum's direction and travel BY HAND. The motor is never driven.

Run with the motor off (IDLE):  .venv/bin/python measure_range.py
Then, by hand:
  1) push the drum slowly DOWN onto the bottom stop and hold it there ~2 s
  2) push it slowly UP onto the top stop and hold it there ~2 s
  3) put it back roughly in the middle and let go
The script stops by itself once that is done (or after --seconds), prints the
direction (+motor = up or down) and the travel, and, if both agree with
config.py, stores the bottom stop as the reference for this ODrive boot.
"""
import argparse
import csv
import sys
import time

import config as C
from joint import CapstanJoint

MIN_SPAN = 1.0      # [motor turns] both stops must be at least this far apart
STILL_TOL = 0.005   # [motor turns]
STILL_S = 2.0
EDGE_TOL = 0.02     # [motor turns] counts as "at the stop"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=240.0)
    ap.add_argument("--log", help="write the samples to this CSV file")
    args = ap.parse_args()

    joint = CapstanJoint()
    state, pos = joint.motor_position()
    if state != "IDLE":
        sys.exit(f"the motor is {state}; it must be off (IDLE) before moving the drum by hand")
    print(f"recording; start at {pos:+.4f} motor turns", flush=True)

    samples = []
    lo = hi = pos
    anchor, anchor_t = pos, 0.0
    t0 = time.monotonic()
    done = False
    while time.monotonic() - t0 < args.seconds:
        state, pos = joint.motor_position()
        if state != "IDLE":
            sys.exit(f"the motor left IDLE ({state}); stopped")
        t = time.monotonic() - t0
        samples.append((t, pos))
        lo, hi = min(lo, pos), max(hi, pos)
        if abs(pos - anchor) > STILL_TOL:
            anchor, anchor_t = pos, t
        span = hi - lo
        in_middle = lo + 0.25 * span < pos < hi - 0.25 * span
        if span > MIN_SPAN and in_middle and t - anchor_t > STILL_S:
            done = True
            break
        time.sleep(0.05)

    if args.log:
        with open(args.log, "w", newline="") as f:
            csv.writer(f).writerows([("t_s", "motor_turns"), *samples])

    span = hi - lo
    print(f"samples {len(samples)}, min {lo:+.4f}, max {hi:+.4f}, span {span:.4f} motor turns")
    if not done:
        sys.exit("not finished: both stops were not reached, or the drum did not come back to rest in the middle")

    t_lo = next(t for t, p in samples if p <= lo + EDGE_TOL)
    t_hi = next(t for t, p in samples if p >= hi - EDGE_TOL)
    if t_lo < t_hi:   # the bottom (reached first) is the minimum: down = -motor
        bottom, dir_up = lo, +1
    else:             # the bottom is the maximum: down = +motor
        bottom, dir_up = hi, -1
    print(f"first stop reached at t={min(t_lo, t_hi):.1f} s, second at t={max(t_lo, t_hi):.1f} s")
    print(f"=> moving UP is {'+' if dir_up > 0 else '-'}motor (MOTOR_DIR_UP = {dir_up:+d}); "
          f"bottom stop at {bottom:+.4f}; travel {span:.3f} motor turns = {span * 360 / C.GEAR_RATIO:.1f} drum deg")

    problems = []
    if dir_up != C.MOTOR_DIR_UP:
        problems.append(f"direction differs from config.py (MOTOR_DIR_UP = {C.MOTOR_DIR_UP:+d})")
    # The stops are compliant, so pushing into them reads longer than the free travel in config.py.
    # Only a SHORTER reading is dangerous: config.py would then allow targets beyond the real stops.
    if span < C.TRAVEL_MOTOR_TURNS - 0.05:
        problems.append(f"measured travel {span:.3f} is shorter than config.py "
                        f"(TRAVEL_MOTOR_TURNS = {C.TRAVEL_MOTOR_TURNS}): were both stops reached?")
    if problems:
        sys.exit("NOT saving a reference: " + "; ".join(problems))
    joint.save_bottom_reference(bottom)
    print(f"saved the bottom reference to {C.REFERENCE_FILE}")


if __name__ == "__main__":
    main()
