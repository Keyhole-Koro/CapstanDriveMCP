"""Map the current needed to move the drum slowly, in both directions.  THIS MOVES THE MOTOR.

Sweeps 55 -> LOW -> HIGH -> 55 deg at SWEEP_DEG_S through joint.py (all safety
layers active) and samples position / velocity / current at ~100 Hz. Per 5-deg
bin it prints the median current while moving down and while moving up, split into
  friction  the part that always opposes the motion
  load      the part that does not depend on the direction (+ = pushes the drum up)

Run only while somebody watches the joint with a hand near the 24 V switch:
  .venv/bin/python scripts/measure_load.py --i-am-watching [--csv samples.csv]
"""
import argparse
import csv
import signal
import statistics
import sys
import time

from capstan import config as C
from capstan.joint import DEG_PER_MOTOR_TURN, CapstanJoint, JointError, trajectory_time

SWEEP_DEG_S = 10.0
LOW, HIGH, END = 6.0, 100.0, 55.0
BIN_DEG = 5.0


def on_term(*_):
    raise KeyboardInterrupt


def sweep(j, target_deg, samples):
    ax = j._axis
    target = j._motor(target_deg)
    limit = time.monotonic() + trajectory_time(abs(target - ax.pos_estimate)) + 3.0
    j._moved_since_enable = True
    ax.controller.input_pos = target
    while True:
        if not j._enabled():
            raise JointError(f"supervisor stopped the motor: {j._fault}")
        pos, vel, iq = ax.pos_estimate, ax.vel_estimate, ax.motor.foc.Iq_measured
        angle = j._deg(pos)
        samples.append((time.monotonic(), angle, vel * C.MOTOR_DIR_UP * DEG_PER_MOTOR_TURN, iq))
        if ax.controller.trajectory_done and abs(angle - target_deg) < 0.3:
            return
        if time.monotonic() > limit:
            raise JointError(f"did not reach {target_deg} deg (at {angle:.1f})")
        time.sleep(0.01)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--i-am-watching", action="store_true", help="confirm that somebody watches the joint")
    ap.add_argument("--csv", help="write the raw samples here")
    args = ap.parse_args()
    if not args.i_am_watching:
        sys.exit("this moves the motor: run it only while somebody watches the joint, with --i-am-watching")
    signal.signal(signal.SIGTERM, on_term)

    j = CapstanJoint()
    if not j.status()["referenced"]:
        sys.exit("no bottom reference for this ODrive boot")
    C.MOVE_VEL = SWEEP_DEG_S / DEG_PER_MOTOR_TURN  # runtime only, for this sweep
    samples = []
    try:
        j.enable()
        time.sleep(0.5)
        for target in (LOW, HIGH, END):
            sweep(j, target, samples)
        print(f"sweep done: {len(samples)} samples")
    except (JointError, KeyboardInterrupt) as e:
        print("STOPPED:", type(e).__name__, e)
    finally:
        try:
            j.disable()
        except Exception as e:
            print("!!! disable failed:", e)

    if args.csv:
        with open(args.csv, "w", newline="") as f:
            csv.writer(f).writerows([("t", "angle_deg", "vel_up_deg_s", "iq_A"), *samples])

    moving = [s for s in samples if abs(s[2]) > 0.7 * SWEEP_DEG_S]  # steady motion only
    down = [s for s in moving if s[2] < 0]
    up = [s for s in moving if s[2] > 0]
    if not down or not up:
        sys.exit("not enough steady motion in both directions to analyse")
    # sign that turns Iq into "pushing down": friction makes the down-moving current lean that way
    sign = 1 if statistics.median(s[3] for s in down) > statistics.median(s[3] for s in up) else -1
    print(f"{'bin [deg]':>12} {'down: n  push[A]':>17} {'up: n  push[A]':>16} | {'friction[A]':>11} {'load[A]':>8}")
    b = LOW - LOW % BIN_DEG
    while b < HIGH:
        d = [sign * s[3] for s in down if b <= s[1] < b + BIN_DEG]
        u = [sign * s[3] for s in up if b <= s[1] < b + BIN_DEG]
        if d and u:
            pd, pu = statistics.median(d), statistics.median(u)
            print(f"{b:5.0f}-{b + BIN_DEG:<5.0f}  {len(d):5d} {pd:+8.2f}   {len(u):5d} {pu:+8.2f}   | "
                  f"{(pd - pu) / 2:11.2f} {(pd + pu) / 2:+8.2f}")
        b += BIN_DEG


if __name__ == "__main__":
    main()
