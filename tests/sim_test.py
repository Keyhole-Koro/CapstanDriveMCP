"""Hardware-free test of joint.py against a toy ODrive simulation.

No USB access, nothing moves: odrive.find_sync is replaced by a simulated device
and the reference file goes to a temporary directory.
Run: .venv/bin/python tests/sim_test.py
"""
import math
import sys
import tempfile
import threading
import time
from pathlib import Path
from types import SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import config as C  # noqa: E402
import joint as J  # noqa: E402
from odrive.enums import AxisState, ControlMode, InputMode, ODriveError  # noqa: E402

CL, IDLE = AxisState.CLOSED_LOOP_CONTROL, AxisState.IDLE


class Ctl:
    def __init__(self):
        self.config = NS(control_mode=ControlMode.POSITION_CONTROL, input_mode=InputMode.POS_FILTER,
                         vel_limit=8.0, vel_ramp_rate=1.0, pos_gain=20.0, vel_gain=0.1667,
                         vel_integrator_gain=0.3333)
        self.input_pos = self.pos_setpoint = self.input_vel = self.input_torque = 0.0

    @property
    def trajectory_done(self):
        return abs(self.pos_setpoint - self.input_pos) < 1e-9


class Axis:
    def __init__(self, plant):
        self._plant = plant
        self.controller = Ctl()
        self.config = NS(enable_watchdog=False, watchdog_timeout=0.0,
                         motor=NS(current_soft_max=10.0, phase_resistance_valid=True, phase_inductance_valid=True))
        self.trap_traj = NS(config=NS(vel_limit=2.0, accel_limit=0.5, decel_limit=0.5))
        self.commutation_mapper = NS(config=NS(offset_valid=True))
        self.motor = NS(foc=NS(Iq_measured=0.0), fet_thermistor=NS(temperature=30.0))
        self.current_state, self.active_errors, self.disarm_reason = IDLE, 0, 0
        self.pos_estimate = self.vel_estimate = 0.0
        self.last_feed = time.monotonic()

    @property
    def requested_state(self):
        return 0

    @requested_state.setter
    def requested_state(self, state):
        if state == CL and not self.active_errors:  # like the firmware: setpoints jump to the position
            self.controller.pos_setpoint = self.controller.input_pos = self.pos_estimate
            self._plant.arm_at = time.monotonic() + 0.02
        elif state == IDLE:
            self.current_state, self._plant.arm_at = IDLE, None

    def watchdog_feed(self):
        self.last_feed = time.monotonic()


class Stats:
    def __init__(self):
        self.t0 = time.monotonic()

    @property
    def uptime(self):
        return int((time.monotonic() - self.t0) * 1000)


class Plant:
    """1 kHz toy: setpoint ramps at the trajectory speed, the drum follows with a stiction
    band that shrinks with stiffer gains, stops at BOTTOM (down = +motor) and TOP."""

    def __init__(self):
        self.bottom, self.top = 1.25, 1.25 - 2.55
        self.axis = self.axis0 = Axis(self)
        self.serial_number, self.vbus_voltage, self.system_stats = 123, 24.0, Stats()
        self.arm_at = None
        self.vel_cmd = 0.0
        self.force_vel = None  # test hook: the drum runs away
        self.invert = False    # test hook: motor turns the wrong way
        threading.Thread(target=self._run, daemon=True).start()

    def clear_errors(self):
        self.axis.active_errors = self.axis.disarm_reason = 0

    def _run(self):
        last = time.monotonic()
        while True:
            time.sleep(0.001)
            now = time.monotonic()
            self._step(now - last, now)
            last = now

    def _step(self, dt, now):
        ax = self.axis
        ctl, cfg = ax.controller, ax.controller.config
        if self.arm_at is not None and now >= self.arm_at:
            self.arm_at, ax.current_state = None, CL
        if ax.current_state != CL:
            ax.vel_estimate = self.vel_cmd = 0.0
            return
        if ax.config.enable_watchdog and now - ax.last_feed > ax.config.watchdog_timeout:
            ax.current_state = IDLE
            ax.active_errors |= ODriveError.WATCHDOG_TIMER_EXPIRED
            ax.disarm_reason = int(ODriveError.WATCHDOG_TIMER_EXPIRED)
            return
        lim, pos = ax.config.motor.current_soft_max, ax.pos_estimate
        if cfg.control_mode == ControlMode.VELOCITY_CONTROL:
            ramp = cfg.vel_ramp_rate * dt
            self.vel_cmd += max(-ramp, min(ramp, ctl.input_vel - self.vel_cmd))
            vel = self.vel_cmd
        else:
            d = ctl.input_pos - ctl.pos_setpoint
            ctl.pos_setpoint += math.copysign(min(abs(d), ax.trap_traj.config.vel_limit * dt), d)
            err = ctl.pos_setpoint - pos
            stiffness = cfg.pos_gain * cfg.vel_gain
            band = 0.05 / stiffness  # error needed to beat 0.05 Nm of stiction
            vel = err * 3 * cfg.vel_integrator_gain / stiffness  # slow integrator creep, always present
            if abs(err) > band:
                vel += (err - math.copysign(band, err)) / 0.03
            vel = max(-cfg.vel_limit, min(cfg.vel_limit, vel))
        if self.invert:
            vel = -vel
        if self.force_vel is not None:
            vel = self.force_vel
        iq = math.copysign(min(lim, 0.8), vel) if abs(vel) > 1e-4 else 0.1
        new = pos + vel * dt
        if new >= self.bottom:
            new, vel, iq = self.bottom, 0.0, lim
        elif new <= self.top:
            new, vel, iq = self.top, 0.0, -lim
        ax.pos_estimate, ax.vel_estimate, ax.motor.foc.Iq_measured = new, vel, iq


def expect_error(fn, text):
    try:
        fn()
    except J.JointError as e:
        assert text in str(e), f"expected '{text}' in '{e}'"
        return str(e)
    raise AssertionError(f"expected a JointError containing '{text}'")


def check_restored(ax, label):
    c = ax.controller.config
    assert ax.current_state == IDLE, label
    assert ax.config.enable_watchdog is False and ax.config.watchdog_timeout == 0.0, label
    assert (c.control_mode, c.input_mode, c.vel_limit, c.vel_ramp_rate) == \
        (ControlMode.POSITION_CONTROL, InputMode.POS_FILTER, 8.0, 1.0), (label, c)
    assert (c.pos_gain, c.vel_gain, c.vel_integrator_gain) == (20.0, 0.1667, 0.3333), (label, c)
    assert ax.config.motor.current_soft_max == 10.0, label
    assert ax.controller.input_vel == 0.0, label


def main():
    C.HOMING_ENABLED = True  # the simulation exercises the homing logic itself
    tmp = Path(tempfile.mkdtemp())
    C.REFERENCE_FILE = tmp / "reference.json"
    plant = Plant()
    J.odrive.find_sync = lambda timeout=None: plant
    J.close_device_manager = lambda: None
    j = J.CapstanJoint()
    ax = plant.axis

    s = j.status()
    assert s["state"] == "IDLE" and s["referenced"] is False
    expect_error(j.enable, "no bottom reference")
    print("ok  status / enable refused without reference")

    r = j.home()
    assert r["homed"] and abs(ax.pos_estimate - plant.bottom) < 1e-6
    assert abs(r["travelled_down_deg"] - plant.bottom * J.DEG_PER_MOTOR_TURN) < 0.5, r
    assert s != r and r["referenced"] and abs(r["angle_deg"]) < 0.01
    check_restored(ax, "after home")
    print(f"ok  home: found the stop after {r['travelled_down_deg']} deg in {r['time_s']} s, settings restored")

    j.enable()
    assert ax.current_state == CL and j._enabled()
    assert (ax.controller.config.pos_gain, ax.controller.config.vel_gain) == (C.POS_GAIN, C.VEL_GAIN)
    r = j.move_to(50.0)
    assert abs(r["error_deg"]) <= C.ARRIVAL_TOL_DEG, r
    print(f"ok  move_to 50: error {r['error_deg']} deg, within 0.1: {r['within_precision']}, {r['time_s']} s")
    r = j.move_by(-10.0)
    assert abs(r["target_deg"] - (50.0 + r["error_deg"] * 0 - 10.0)) < 1.0, r
    expect_error(lambda: j.move_to(110.0), "outside the allowed range")
    print("ok  move_by / range check")

    tuned = C.POS_GAIN, C.VEL_GAIN, C.VEL_INTEGRATOR_GAIN
    C.POS_GAIN, C.VEL_GAIN, C.VEL_INTEGRATOR_GAIN = 35.0, 0.5, 2.5  # stiffer: settles tighter
    j.disable()
    check_restored(ax, "after disable")
    j.enable()
    r = j.move_to(60.0)
    assert r["within_precision"], r
    print(f"ok  stiffer gains settle within 0.1 deg: error {r['error_deg']} deg in {r['time_s']} s")
    C.POS_GAIN, C.VEL_GAIN, C.VEL_INTEGRATOR_GAIN = tuned

    plant.force_vel = 3.0  # runaway
    time.sleep(0.2)
    plant.force_vel = None
    assert not j._enabled() and "over-speed" in (j._fault or ""), j._fault
    check_restored(ax, "after supervisor stop")
    expect_error(lambda: j.move_to(40.0), "not enabled")
    print(f"ok  supervisor stopped a runaway: {j._fault}")

    j.enable()
    j._sup_stop.set()  # frozen server: nobody feeds the watchdog any more
    j._sup_thread.join(1.0)
    time.sleep(0.7)
    assert ax.current_state == IDLE and ax.active_errors & ODriveError.WATCHDOG_TIMER_EXPIRED
    j._call(lambda: j._disarm(ax))
    expect_error(j.enable, "WATCHDOG_TIMER_EXPIRED")
    j.clear_errors()
    print("ok  watchdog turns the motor off when the server freezes; enable refused until clear_errors")

    plant.invert = True
    expect_error(j.home, "moved UP")
    plant.invert = False
    check_restored(ax, "after wrong-way homing")
    plant.bottom, C.HOMING_MAX_TRAVEL = 100.0, 0.3
    expect_error(j.home, "no bottom stop")
    check_restored(ax, "after homing without a stop")
    print("ok  homing aborts on wrong direction / missing stop, settings restored")

    assert j.release() == {"released": True} and j._odrv is None
    print("ok  release_usb")
    print("ALL SIM TESTS PASSED")


if __name__ == "__main__":
    main()
