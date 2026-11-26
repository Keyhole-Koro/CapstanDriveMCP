"""Pick the position-loop gains from measured step responses.  THIS MOVES THE MOTOR.

For a ladder of gain sets (softest first) the drum makes small moves around its
current angle (+STEP, back, -STEP, back). For each set it reports:
  settle  time until the error stays within 0.1 deg
  final   error at the end of the observation window
  over    overshoot past the target
  noise   position std / velocity RMS / current RMS while holding still
It stops climbing at the first sign of oscillation, turns the motor off and
names the last clean set. Nothing is saved: copy the chosen gains into config.py.

Run only while somebody watches the joint with a hand near the 24 V switch:
  .venv/bin/python scripts/tune_gains.py --i-am-watching
"""
import argparse
import math
import signal
import statistics
import sys
import time

from capstan import config as C
from capstan.joint import ANGLE_MAX, ANGLE_MIN, CapstanJoint, JointError, trajectory_time

LADDER = [  # (pos_gain, vel_gain [Nm/(turn/s)], vel_integrator_gain [Nm/turn])
    (20.0, 0.1667, 0.3333),  # ODrive defaults
    (25.0, 0.25, 0.75),
    (30.0, 0.35, 1.4),
    (35.0, 0.5, 2.5),
    (40.0, 0.7, 3.5),
]
STEP_DEG = 5.0
SETTLE_TOL_DEG = 0.1
TAIL_S = 1.5             # observe this long after the trajectory ends
OSC_POS_STD_DEG = 0.1    # holding still: position std above this = oscillating
OSC_VEL_RMS = 0.05       # holding still: velocity RMS above this [motor turns/s] = oscillating / noisy
OSC_VEL_ABORT = 0.3      # after the trajectory: any speed above this [motor turns/s] aborts at once
MAX_OVERSHOOT_DEG = 1.0


def on_term(*_):
    raise KeyboardInterrupt


def step(j, target_deg):
    """Command one move, sample it at ~100 Hz and return its metrics."""
    ax = j._axis
    target = j._motor(target_deg)
    start = ax.pos_estimate
    direction = 1 if target_deg > j._deg(start) else -1
    traj_s = trajectory_time(abs(target - start))
    window = traj_s + TAIL_S
    j._moved_since_enable = True
    ax.controller.input_pos = target
    t0 = time.monotonic()
    samples = []
    while (t := time.monotonic() - t0) < window:
        if not j._enabled():
            raise JointError(f"supervisor stopped the motor: {j._fault}")
        err, vel, iq = j._deg(ax.pos_estimate) - target_deg, ax.vel_estimate, ax.motor.foc.Iq_measured
        samples.append((t, err, vel, iq))
        if t > traj_s + 0.3 and abs(vel) > OSC_VEL_ABORT:
            raise JointError(f"oscillation: {vel:+.2f} motor turns/s while it should be holding still")
        time.sleep(0.005)
    errs = [s[1] for s in samples]
    settle = next((s[0] for i, s in enumerate(samples)
                   if all(abs(e) <= SETTLE_TOL_DEG for e in errs[i:])), None)
    hold = [s for s in samples if s[0] >= window - 0.5]
    return {"settle": settle, "final": errs[-1],
            "overshoot": max(0.0, max(e * direction for e in errs)),
            "pos_std": statistics.pstdev([s[1] for s in hold]),
            "vel_rms": math.sqrt(sum(s[2] ** 2 for s in hold) / len(hold)),
            "iq_rms": math.sqrt(sum(s[3] ** 2 for s in hold) / len(hold))}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--i-am-watching", action="store_true", help="confirm that somebody watches the joint")
    args = ap.parse_args()
    if not args.i_am_watching:
        sys.exit("this moves the motor: run it only while somebody watches the joint, with --i-am-watching")
    signal.signal(signal.SIGTERM, on_term)

    j = CapstanJoint()
    s = j.status()
    if not s["referenced"]:
        sys.exit("no bottom reference for this ODrive boot: run home first")
    a0 = s["angle_deg"]
    if not ANGLE_MIN + STEP_DEG + 1 <= a0 <= ANGLE_MAX - STEP_DEG - 1:
        sys.exit(f"the drum is at {a0:.1f} deg; move it towards the middle of the range first")

    print(f"start at {a0:.2f} deg; moves: +{STEP_DEG}, back, -{STEP_DEG}, back")
    print(f"{'pos_gain':>8} {'vel_gain':>8} {'int_gain':>8} | {'settle_s (4 moves)':>23} | "
          f"{'max|final|':>10} {'max_over':>8} | {'pos_std':>7} {'vel_rms':>7} {'iq_rms':>6}")
    last_clean = None
    try:
        for gains in LADDER:
            C.POS_GAIN, C.VEL_GAIN, C.VEL_INTEGRATOR_GAIN = gains
            j.enable()
            time.sleep(0.6)
            rows = [step(j, t) for t in (a0 + STEP_DEG, a0, a0 - STEP_DEG, a0)]
            j.disable()
            settles = " ".join("  -  " if r["settle"] is None else f"{r['settle']:5.2f}" for r in rows)
            print(f"{gains[0]:8.1f} {gains[1]:8.3f} {gains[2]:8.3f} | {settles:>23} | "
                  f"{max(abs(r['final']) for r in rows):10.3f} {max(r['overshoot'] for r in rows):8.3f} | "
                  f"{max(r['pos_std'] for r in rows):7.3f} {max(r['vel_rms'] for r in rows):7.3f} "
                  f"{max(r['iq_rms'] for r in rows):6.2f}", flush=True)
            if any(r["pos_std"] > OSC_POS_STD_DEG or r["vel_rms"] > OSC_VEL_RMS
                   or r["overshoot"] > MAX_OVERSHOOT_DEG for r in rows):
                print("-> oscillation, noise or overshoot: not going higher")
                break
            last_clean = gains
    except (JointError, KeyboardInterrupt) as e:
        print("STOPPED:", type(e).__name__, e)
    finally:
        try:
            j.disable()
        except Exception as e:
            print("!!! disable failed:", e)
    if last_clean:
        print(f"last clean set: POS_GAIN = {last_clean[0]}, VEL_GAIN = {last_clean[1]}, "
              f"VEL_INTEGRATOR_GAIN = {last_clean[2]}  (copy into config.py)")


if __name__ == "__main__":
    main()
