"""MCP server (stdio) that lets an LLM operate the capstan joint.

Register with Claude Code:
  claude mcp add capstan -- /home/kiho/Workspaces/26K66/.venv/bin/python /home/kiho/Workspaces/26K66/server.py

The server connects to the ODrive on the first tool call and keeps the USB
connection until release_usb is called or the server exits.
"""
import functools
import logging
import sys

from mcp.server.mcpserver import MCPServer
from mcp.server.mcpserver.exceptions import ToolError

import config as C
from joint import ANGLE_MAX, ANGLE_MIN, DEG_PER_MOTOR_TURN, CapstanJoint, JointError

logging.basicConfig(stream=sys.stderr, level=logging.INFO,
                    format="%(asctime)s %(name)s %(levelname)s: %(message)s")
log = logging.getLogger("capstan.server")

RANGE = f"[{ANGLE_MIN:.1f}, {ANGLE_MAX:.1f}] deg"
TOP_SPEED = f"{C.MOVE_VEL * DEG_PER_MOTOR_TURN:.0f} deg/s"

INSTRUCTIONS = f"""Operates one real capstan-driven joint (ODrive S1 + brushless motor, 8:1 drum).
Positions are the drum angle in degrees above the bottom end stop: 0 = bottom, up = positive.
Move targets must be within {RANGE}; speed is capped at {TOP_SPEED}.
Typical flow: status -> (if not referenced: ask the human to hold the drum on the bottom stop by
hand, then set_bottom_here) -> enable -> move_to / move_by -> go_home (or disable) when finished.
metrics reports accuracy and other figures since the server started.
The bottom reference is lost whenever the ODrive restarts (it survives the 24 V supply being off
while USB stays connected). Never try to find the stops by driving the motor into them.
This moves real hardware: before the first enable in a conversation, confirm with the human that the
joint is clear and watched. If anything looks wrong, call disable (the drum does not fall when the
motor is off). The server turns the motor off by itself if it pushes on something."""

server = MCPServer("capstan", instructions=INSTRUCTIONS)
joint = CapstanJoint()


def _guard(fn):
    """Turn JointError into a ToolError whose text reaches the model."""
    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except JointError as e:
            raise ToolError(str(e)) from e
        except Exception as e:
            log.exception("unexpected error in %s", fn.__name__)
            raise ToolError(f"internal error: {type(e).__name__}: {e}") from e
    return wrapper


@server.tool()
@_guard
def status() -> dict:
    """Read the joint without moving it: motor state, errors, whether the bottom reference is set,
    drum angle (deg above the bottom stop), velocity (+ = up), motor current, bus voltage, temperature."""
    return joint.status()


@server.tool()
@_guard
def set_bottom_here() -> dict:
    """Store the drum's current position as the bottom end stop (angle 0). Call it only while the
    human holds the drum on the bottom stop by hand; the motor must be disabled. Reports the shift
    from the previous reference of the same ODrive boot: more than a few degrees means the cable
    slipped, so tell the human."""
    return joint.set_bottom_here()


@server.tool()
@_guard
def enable() -> dict:
    """Turn the motor on and hold the current position. Needs the bottom reference."""
    return joint.enable()


@server.tool(description=f"Move the drum to an absolute angle (deg above the bottom stop) within {RANGE} "
                         f"and wait until it arrives and settles (top speed {TOP_SPEED}); returns the final "
                         f"error. The motor must be enabled.")
@_guard
def move_to(angle_deg: float) -> dict:
    return joint.move_to(angle_deg)


@server.tool(description=f"Move the drum by delta_deg from where it is now (+ = up) and wait until it arrives. "
                         f"The resulting angle must stay within {RANGE}. The motor must be enabled.")
@_guard
def move_by(delta_deg: float) -> dict:
    return joint.move_by(delta_deg)


@server.tool(description=f"Move the drum to the home position ({C.HOME_DEG:.0f} deg, just above the bottom stop; "
                         "0 deg is the stop itself) and then turn the motor off; turn_off=false keeps it on. "
                         "The motor must be enabled.")
@_guard
def go_home(turn_off: bool = True) -> dict:
    return joint.go_home(turn_off)


@server.tool()
@_guard
def metrics() -> dict:
    """Performance since the server started, without touching the hardware: moves completed /
    failed / refused, supervisor stops, final position error (mean / p95 / max), share of moves
    within 0.1 deg, move times, peak current and speed, total travel."""
    return joint.metrics()


@server.tool()
@_guard
def disable() -> dict:
    """Turn the motor off immediately (IDLE). The drum stays where it is. Use this to stop."""
    return joint.disable()


@server.tool()
@_guard
def clear_errors() -> dict:
    """Clear the ODrive's error flags once the cause has been checked. The motor must be disabled."""
    return joint.clear_errors()


@server.tool()
@_guard
def release_usb() -> dict:
    """Turn the motor off and release the ODrive's USB connection so the ODrive Web GUI or odrivetool
    can use it. The next tool call reconnects."""
    return joint.release()


if __name__ == "__main__":
    server.run("stdio")
