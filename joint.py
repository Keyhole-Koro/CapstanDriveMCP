"""Safety-wrapped control of one capstan joint: ODrive S1 over USB.

Callers work in drum degrees above the bottom end stop (0 = bottom, up = +).
Nothing is ever saved to the ODrive's flash: motion settings are applied at
runtime, and the ODrive's saved limits (8 turns/s, 10 A) stay as a backstop.

While the motor is enabled:
  * targets outside [ANGLE_MIN, ANGLE_MAX] are rejected before anything moves
  * the ODrive enforces the runtime current / speed / trajectory limits (config.py)
  * a supervisor thread disarms on ODrive errors, leaving closed loop, leaving
    the travel window, over-speed, or drift right after enabling
  * the ODrive watchdog disarms the motor if this process stops feeding it
Homing needs no limit switch: the drum runs down slowly with a low current
limit until it stalls on the bottom stop; that position becomes the reference.
"""
from __future__ import annotations

import json
import logging
import math
import threading
import time

import odrive
from odrive.device_manager import close_device_manager
from odrive.enums import AxisState, ControlMode, InputMode, ODriveError

import config as C

log = logging.getLogger("capstan")

DEG_PER_MOTOR_TURN = 360.0 / C.GEAR_RATIO
TRAVEL_DEG = C.TRAVEL_MOTOR_TURNS * DEG_PER_MOTOR_TURN
ANGLE_MIN = C.MARGIN_MOTOR_TURNS * DEG_PER_MOTOR_TURN
ANGLE_MAX = TRAVEL_DEG - ANGLE_MIN
BOOT_MATCH_TOL_S = 10.0  # a reference belongs to one ODrive boot (position resets to 0 at boot)
# controller.config values changed at runtime and restored when the motor is turned off
_CTL_KEYS = ("control_mode", "input_mode", "vel_limit", "vel_ramp_rate",
             "pos_gain", "vel_gain", "vel_integrator_gain")


class JointError(RuntimeError):
    """An anticipated refusal or failure; the message is meant for the operator."""


def error_names(mask: int) -> list[str]:
    return [e.name for e in ODriveError if mask & e.value]


def trajectory_time(dist: float) -> float:
    """Duration [s] of a trapezoidal move of `dist` motor turns with the config limits."""
    v, a, d = C.MOVE_VEL, C.MOVE_ACCEL, C.MOVE_DECEL
    ramps = v * v / (2 * a) + v * v / (2 * d)
    if dist <= ramps:  # never reaches top speed: triangular profile
        peak = math.sqrt(2 * dist * a * d / (a + d))
        return peak / a + peak / d
    return v / a + v / d + (dist - ramps) / v


class CapstanJoint:
    def __init__(self):
        self._lock = threading.RLock()  # serialises commands; the supervisor never takes it
        self._odrv = None
        self._axis = None
        self._originals = None          # runtime values restored when disarming
        self._bottom = None             # motor turns at the bottom stop, this ODrive boot
        self._fault = None              # why the supervisor last disarmed
        self._moved_since_enable = False  # ends the post-enable drift check
        self._sup_thread = None
        self._sup_stop = threading.Event()

    # ------------------------------------------------------------ connection
    def _connect(self):
        if self._odrv is not None:
            return
        try:
            odrv = odrive.find_sync(timeout=10)
        except Exception as e:
            raise JointError("could not connect to the ODrive over USB (unplugged, or the Web GUI / "
                             f"another script is using it): {type(e).__name__}") from e
        ax = odrv.axis0
        mot, ctl, tt = ax.config.motor, ax.controller.config, ax.trap_traj.config
        if not (mot.phase_resistance_valid and mot.phase_inductance_valid
                and ax.commutation_mapper.config.offset_valid):
            raise JointError("the ODrive is not calibrated")
        if ctl.vel_limit > C.MAX_SAVED_VEL_LIMIT + 1e-6 or mot.current_soft_max > C.MAX_SAVED_CURRENT_A + 1e-6:
            raise JointError(f"the ODrive limits are higher than expected (vel_limit={ctl.vel_limit}, "
                             f"current_soft_max={mot.current_soft_max}); another program changed them. "
                             "Reboot the ODrive to get the saved limits back.")
        if ax.current_state == AxisState.IDLE:
            ax.config.enable_watchdog = False  # leftover from a session that crashed
        self._originals = {"controller": {k: getattr(ctl, k) for k in _CTL_KEYS},
                           "current_soft_max": mot.current_soft_max,
                           "watchdog_timeout": ax.config.watchdog_timeout,
                           "trap": (tt.vel_limit, tt.accel_limit, tt.decel_limit)}
        self._odrv, self._axis = odrv, ax
        self._load_reference()
        log.info("connected to ODrive %s (referenced: %s)", odrv.serial_number, self._bottom is not None)

    def _drop(self):
        self._sup_stop.set()
        self._odrv = self._axis = None
        self._bottom = None

    def _call(self, fn):
        """Run fn() connected and under the command lock; a transport failure drops the connection."""
        with self._lock:
            self._connect()
            try:
                return fn()
            except JointError:
                raise
            except Exception as e:
                log.exception("ODrive I/O failed")
                self._drop()
                raise JointError(f"lost the ODrive connection ({type(e).__name__}: {e}); if the motor "
                                 "was enabled, the ODrive watchdog turns it off") from e

    # ------------------------------------------------------------- reference
    def _boot_epoch(self) -> float:
        return time.time() - self._odrv.system_stats.uptime / 1000.0

    def _load_reference(self):
        self._bottom = None
        try:
            ref = json.loads(C.REFERENCE_FILE.read_text())
        except (OSError, ValueError):
            return
        same_odrive = ref.get("serial") == str(self._odrv.serial_number)
        same_boot = abs(ref.get("boot_epoch", 0.0) - self._boot_epoch()) <= BOOT_MATCH_TOL_S
        if same_odrive and same_boot:
            self._bottom = float(ref["bottom_motor_turns"])
        else:
            log.info("reference.json belongs to an earlier ODrive boot; ignoring it")

    def _save_reference(self, bottom: float):
        ref = {"bottom_motor_turns": bottom, "serial": str(self._odrv.serial_number),
               "boot_epoch": self._boot_epoch(), "saved_at": time.strftime("%Y-%m-%d %H:%M:%S")}
        C.REFERENCE_FILE.write_text(json.dumps(ref, indent=2) + "\n")
        self._bottom = bottom

    def _deg(self, motor_turns: float) -> float:
        return (motor_turns - self._bottom) * C.MOTOR_DIR_UP * DEG_PER_MOTOR_TURN

    def _motor(self, deg: float) -> float:
        return self._bottom + C.MOTOR_DIR_UP * deg / DEG_PER_MOTOR_TURN

    # ------------------------------------------------------- arming / safety
    def _enabled(self) -> bool:
        return self._sup_thread is not None and self._sup_thread.is_alive()

    def _require_idle(self, ax):
        if self._enabled():
            raise JointError("the motor is on; call disable first")
        if ax.current_state != AxisState.IDLE:
            raise JointError(f"the axis is in {AxisState(ax.current_state).name}, expected IDLE "
                             "(is another program controlling it?)")
        if ax.active_errors:
            raise JointError(f"ODrive errors {error_names(ax.active_errors)}: check the cause, then call clear_errors")

    def _arm(self, ax):
        """Watchdog on, then closed loop. Raises if the axis does not get there."""
        ax.config.watchdog_timeout = C.WATCHDOG_TIMEOUT_S
        ax.watchdog_feed()
        ax.config.enable_watchdog = True
        ax.watchdog_feed()
        ax.requested_state = AxisState.CLOSED_LOOP_CONTROL
        deadline = time.monotonic() + 1.5
        while ax.current_state != AxisState.CLOSED_LOOP_CONTROL:
            ax.watchdog_feed()
            if ax.active_errors:
                raise JointError(f"could not turn the motor on: ODrive errors {error_names(ax.active_errors)}")
            if time.monotonic() > deadline:
                raise JointError(f"could not turn the motor on within 1.5 s (state {AxisState(ax.current_state).name}, "
                                 f"disarm reason {error_names(ax.disarm_reason)})")
            time.sleep(0.01)
        ax.watchdog_feed()

    def _disarm(self, ax):
        """Motor off, then undo the runtime changes. The watchdog is switched off
        only once the axis really is IDLE; otherwise it stays on and trips."""
        ax.requested_state = AxisState.IDLE
        deadline = time.monotonic() + 1.0
        while ax.current_state != AxisState.IDLE:
            if time.monotonic() > deadline:
                raise JointError("the ODrive did not go IDLE within 1 s; its watchdog will turn it off")
            time.sleep(0.01)
        ax.config.enable_watchdog = False
        ax.controller.input_vel = 0.0
        ax.controller.input_torque = 0.0
        o = self._originals
        if o:
            ctl, tt = ax.controller.config, ax.trap_traj.config
            for key, value in o["controller"].items():
                setattr(ctl, key, value)
            ax.config.motor.current_soft_max = o["current_soft_max"]
            ax.config.watchdog_timeout = o["watchdog_timeout"]
            tt.vel_limit, tt.accel_limit, tt.decel_limit = o["trap"]

    def _supervise(self, ax, bottom: float, hold_pos: float, t_enable: float):
        try:
            while not self._sup_stop.is_set():
                ax.watchdog_feed()
                state, err = ax.current_state, ax.active_errors
                pos, vel = ax.pos_estimate, ax.vel_estimate
                if self._sup_stop.is_set():  # disable() is turning it off on purpose
                    break
                ang = (pos - bottom) * C.MOTOR_DIR_UP * DEG_PER_MOTOR_TURN
                reason = None
                if err:
                    reason = f"ODrive error {error_names(err)}"
                elif state != AxisState.CLOSED_LOOP_CONTROL:
                    reason = (f"ODrive left closed loop (state {AxisState(state).name}, "
                              f"disarm reason {error_names(ax.disarm_reason)})")
                elif not -C.WINDOW_SLACK_DEG <= ang <= TRAVEL_DEG + C.WINDOW_SLACK_DEG:
                    reason = f"drum left the travel window ({ang:.1f} deg)"
                elif abs(vel) > C.VEL_ABORT:
                    reason = f"over-speed ({vel:+.2f} motor turns/s)"
                elif (not self._moved_since_enable and time.monotonic() - t_enable < C.ENABLE_HOLD_S
                      and abs(pos - hold_pos) > C.ENABLE_HOLD_TOL_TURNS):
                    reason = f"moved {pos - hold_pos:+.3f} motor turns right after enabling"
                if reason:
                    self._fault = reason
                    log.warning("supervisor disarming: %s", reason)
                    self._disarm(ax)
                    return
                time.sleep(C.SUPERVISOR_PERIOD_S)
        except Exception as e:
            self._fault = f"supervisor lost the ODrive ({type(e).__name__}); the ODrive watchdog turns the motor off"
            log.exception("supervisor I/O failed")
            try:
                ax.requested_state = AxisState.IDLE
            except Exception:
                pass

    # ---------------------------------------------------------------- public
    def status(self) -> dict:
        def read():
            ax, o = self._axis, self._odrv
            pos = ax.pos_estimate
            ref = self._bottom is not None
            state = ax.current_state
            # Iq is only measured while the motor is driven; in IDLE it keeps a stale value.
            current = round(abs(ax.motor.foc.Iq_measured), 2) if state == AxisState.CLOSED_LOOP_CONTROL else 0.0
            return {
                "state": AxisState(state).name,
                "enabled": self._enabled(),
                "odrive_errors": error_names(ax.active_errors),
                "last_disarm_reason": error_names(ax.disarm_reason),
                "last_supervisor_stop": self._fault,
                "referenced": ref,
                "angle_deg": round(self._deg(pos), 2) if ref else None,
                "allowed_range_deg": [round(ANGLE_MIN, 1), round(ANGLE_MAX, 1)],
                "velocity_deg_s": round(ax.vel_estimate * C.MOTOR_DIR_UP * DEG_PER_MOTOR_TURN, 1),
                "motor_current_A": current,
                "motor_pos_turns": round(pos, 4),
                "bus_voltage_V": round(o.vbus_voltage, 2),
                "fet_temp_C": round(ax.motor.fet_thermistor.temperature, 1),
            }
        return self._call(read)

    def motor_position(self) -> tuple[str, float]:
        """(axis state name, raw motor position in turns). Read-only."""
        return self._call(lambda: (AxisState(self._axis.current_state).name, self._axis.pos_estimate))

    def save_bottom_reference(self, bottom_motor_turns: float):
        """Store a bottom-stop position measured during this ODrive boot (motor must be off)."""
        def do():
            self._require_idle(self._axis)
            self._save_reference(bottom_motor_turns)
        self._call(do)

    def set_bottom_here(self) -> dict:
        def do():
            ax = self._axis
            self._require_idle(ax)
            if abs(ax.vel_estimate) > 0.05:
                raise JointError("the drum is moving; let it rest on the bottom stop")
            self._save_reference(ax.pos_estimate)
        self._call(do)
        return self.status()

    def home(self) -> dict:
        """Find the bottom stop without a limit switch and store it as the reference.
        Runs down at HOMING_VEL with the current limited to HOMING_CURRENT_A until the drum
        stalls on the stop, then turns the motor off."""
        if not C.HOMING_ENABLED:
            raise JointError("homing is disabled (it mistook a high-friction spot for the bottom stop); "
                             "have the human put the drum on the bottom stop by hand and call set_bottom_here")

        def do():
            ax = self._axis
            ctl = ax.controller
            self._require_idle(ax)
            down = -C.MOTOR_DIR_UP  # sign of motor turns when the drum moves down
            start, previous = ax.pos_estimate, self._bottom
            t0 = time.monotonic()
            stall_since = None
            moving_peak_a = 0.0
            try:
                cfg = ctl.config
                cfg.control_mode = ControlMode.VELOCITY_CONTROL
                cfg.input_mode = InputMode.VEL_RAMP
                cfg.vel_ramp_rate = 1.0
                cfg.vel_limit = 1.0
                cfg.vel_gain, cfg.vel_integrator_gain = C.VEL_GAIN, C.VEL_INTEGRATOR_GAIN
                ax.config.motor.current_soft_max = C.HOMING_CURRENT_A
                ctl.input_torque = 0.0
                ctl.input_vel = 0.0
                self._fault = None
                self._arm(ax)
                ctl.input_vel = down * C.HOMING_VEL
                while True:
                    ax.watchdog_feed()
                    state, err = ax.current_state, ax.active_errors
                    pos, vel, iq = ax.pos_estimate, ax.vel_estimate, ax.motor.foc.Iq_measured
                    now = time.monotonic()
                    if err or state != AxisState.CLOSED_LOOP_CONTROL:
                        raise JointError(f"homing stopped by the ODrive: state {AxisState(state).name}, "
                                         f"errors {error_names(err)}, disarm reason {error_names(ax.disarm_reason)}")
                    travelled = (pos - start) * down  # + = downwards
                    if travelled < -0.1:
                        raise JointError("homing moved UP instead of down: check MOTOR_DIR_UP in config.py")
                    if travelled > C.HOMING_MAX_TRAVEL:
                        raise JointError(f"no bottom stop found within {C.HOMING_MAX_TRAVEL} motor turns "
                                         "(is the cable slipping?)")
                    if abs(vel) > 2 * C.HOMING_VEL:
                        raise JointError(f"homing over-speed ({vel:+.2f} motor turns/s)")
                    if now - t0 > C.HOMING_MAX_TRAVEL / C.HOMING_VEL + 5.0:
                        raise JointError("homing timed out")
                    if abs(vel) < C.HOMING_STALL_VEL and abs(iq) >= 0.9 * C.HOMING_CURRENT_A:
                        if stall_since is None:
                            stall_since = now
                        if now - stall_since >= C.HOMING_STALL_S:
                            break
                    else:
                        stall_since = None
                        if abs(vel) > 0.1:
                            moving_peak_a = max(moving_peak_a, abs(iq))
                    time.sleep(0.02)
            finally:
                self._disarm(ax)  # stop pushing: motor off, runtime settings restored
            self._save_reference(pos)
            change = None if previous is None else round((pos - previous) * C.MOTOR_DIR_UP * DEG_PER_MOTOR_TURN, 2)
            moving_s = (stall_since or now) - t0
            return {"homed": True, "travelled_down_deg": round(travelled * DEG_PER_MOTOR_TURN, 1),
                    "time_s": round(time.monotonic() - t0, 1),
                    "avg_speed_deg_s": round(travelled * DEG_PER_MOTOR_TURN / moving_s, 1) if moving_s > 0 else None,
                    "moving_peak_current_A": round(moving_peak_a, 2),
                    "change_from_previous_reference_deg": change}
        result = self._call(do)
        return {**result, **self.status()}

    def enable(self) -> dict:
        def do():
            ax = self._axis
            ctl = ax.controller
            if self._enabled():
                return
            if self._bottom is None:
                raise JointError("no bottom reference for this ODrive boot: call home, or have the human put "
                                 "the drum on the bottom stop by hand and call set_bottom_here")
            self._require_idle(ax)
            bottom, pos = self._bottom, ax.pos_estimate
            ang = self._deg(pos)
            if not -C.WINDOW_SLACK_DEG <= ang <= TRAVEL_DEG + C.WINDOW_SLACK_DEG:
                raise JointError(f"the drum reads {ang:.1f} deg, outside the travel window, so the reference is "
                                 "wrong: call home again")
            try:
                cfg, tt = ctl.config, ax.trap_traj.config
                cfg.control_mode = ControlMode.POSITION_CONTROL
                cfg.input_mode = InputMode.TRAP_TRAJ
                cfg.vel_limit = C.VEL_LIMIT
                cfg.pos_gain, cfg.vel_gain, cfg.vel_integrator_gain = C.POS_GAIN, C.VEL_GAIN, C.VEL_INTEGRATOR_GAIN
                ax.config.motor.current_soft_max = C.CURRENT_LIMIT_A
                tt.vel_limit, tt.accel_limit, tt.decel_limit = C.MOVE_VEL, C.MOVE_ACCEL, C.MOVE_DECEL
                ctl.input_vel = 0.0
                ctl.input_torque = 0.0
                ctl.input_pos = pos
                self._fault = None
                self._arm(ax)
                setpoint, now = ctl.pos_setpoint, ax.pos_estimate
                if abs(setpoint - now) > 0.02:
                    raise JointError(f"position setpoint {setpoint:.3f} != position {now:.3f} after enabling")
            except BaseException:
                self._disarm(ax)
                raise
            self._sup_stop.clear()
            self._moved_since_enable = False
            self._sup_thread = threading.Thread(target=self._supervise, args=(ax, bottom, pos, time.monotonic()),
                                                name="capstan-supervisor", daemon=True)
            self._sup_thread.start()
        self._call(do)
        return self.status()

    def move_to(self, angle_deg: float) -> dict:
        if not ANGLE_MIN <= angle_deg <= ANGLE_MAX:
            raise JointError(f"target {angle_deg:.1f} deg is outside the allowed range "
                             f"[{ANGLE_MIN:.1f}, {ANGLE_MAX:.1f}] deg")

        def do():
            if not self._enabled():
                raise JointError("the motor is not enabled; call enable first"
                                 + (f" (last stop: {self._fault})" if self._fault else ""))
            ax, ctl = self._axis, self._axis.controller
            target = self._motor(angle_deg)
            fail_at = time.monotonic() + trajectory_time(abs(target - ax.pos_estimate)) + C.SETTLE_TIMEOUT_S
            self._moved_since_enable = True  # set before commanding, so the drift check never sees the move
            ctl.input_pos = target
            t0 = time.monotonic()
            arrived_at = in_tol_since = None
            peak_a = peak_v = 0.0
            while True:
                time.sleep(0.02)
                if not self._enabled():
                    raise JointError(f"stopped during the move: {self._fault or 'disabled'}")
                pos, vel, iq = ax.pos_estimate, ax.vel_estimate, ax.motor.foc.Iq_measured
                now = time.monotonic()
                peak_a, peak_v = max(peak_a, abs(iq)), max(peak_v, abs(vel))
                err = self._deg(pos) - angle_deg
                if arrived_at is None:
                    if ctl.trajectory_done and abs(err) <= C.ARRIVAL_TOL_DEG:
                        arrived_at = now
                    elif now > fail_at:
                        self.disable()
                        raise JointError(f"did not reach {angle_deg:.1f} deg in time (stopped at "
                                         f"{angle_deg + err:.1f} deg); motor turned off")
                if arrived_at is not None:  # settle: wait for PRECISION_TOL_DEG, at most PRECISION_WAIT_S
                    if abs(err) <= C.PRECISION_TOL_DEG:
                        if in_tol_since is None:
                            in_tol_since = now
                        if now - in_tol_since >= 0.1:
                            break
                    else:
                        in_tol_since = None
                    if now - arrived_at >= C.PRECISION_WAIT_S:
                        break
            return {"target_deg": round(angle_deg, 2), "reached_deg": round(angle_deg + err, 2),
                    "error_deg": round(err, 2), "within_precision": abs(err) <= C.PRECISION_TOL_DEG,
                    "time_s": round(now - t0, 2), "peak_current_A": round(peak_a, 2),
                    "peak_speed_deg_s": round(peak_v * DEG_PER_MOTOR_TURN, 1)}
        return self._call(do)

    def move_by(self, delta_deg: float) -> dict:
        def current():
            if self._bottom is None:
                raise JointError("no bottom reference: call home first")
            return self._deg(self._axis.pos_estimate)
        return self.move_to(self._call(current) + delta_deg)

    def disable(self) -> dict:
        """Motor off now. Requests IDLE before taking the command lock, so it also
        interrupts a move that is in progress."""
        self._sup_stop.set()
        ax = self._axis
        if ax is not None:
            try:
                ax.requested_state = AxisState.IDLE
            except Exception:
                pass
        if self._sup_thread is not None:
            self._sup_thread.join(timeout=1.0)
        self._call(lambda: self._disarm(self._axis))
        return self.status()

    def clear_errors(self) -> dict:
        def do():
            if self._enabled():
                raise JointError("disable the motor first")
            self._odrv.clear_errors()
            self._fault = None
        self._call(do)
        return self.status()

    def release(self) -> dict:
        """Motor off, then let go of the USB device (for the Web GUI / odrivetool)."""
        with self._lock:
            if self._odrv is not None:
                self.disable()
            self._drop()
            close_device_manager()
        return {"released": True}
