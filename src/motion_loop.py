"""
motion_loop.py — Unified motion phases for both arms.

BLACK ARM (A → C):  DEFAULT_BLACK_PHASES
WHITE ARM (C → B):  DEFAULT_WHITE_PHASES
GENERIC  (SRC→DST): DEFAULT_AB_PHASES
"""

from dataclasses import dataclass
import time
from typing import Callable

from config import (
    ACB_RETRACT_DWELL_MS,
    BLACK_GRIPPER_CLOSED,
    BLACK_GRIPPER_OPEN,
    WHITE_GRIPPER_CLOSED,
    WHITE_GRIPPER_OPEN,
    PRESS_SYNC_TIMEOUT_S,
)
from servos import ServoController


class MotionTimeoutError(RuntimeError):
    """Raised when the firmware does not acknowledge a motion phase in time."""


@dataclass(frozen=True)
class MotionPhase:
    """One editable step in a motion loop."""

    name: str
    action: str
    station: str | None = None
    pose: str | None = None
    gripper: int | None = None
    servo_index: int | None = None
    servo_angle: int | None = None
    joint_index: int | None = None
    joint_delta: int = 0
    pause_ms: int = 0
    wait_min_s: float | None = None
    wait_extra_s: float | None = None
    count_cycle: bool = False
    press_signal: str | None = None


# ============================================================
# EDITABLE LOOP DELAYS
# ============================================================
AB_PICK_SETTLE_DELAY_MS = 0
AB_DROP_SETTLE_DELAY_MS = 0

ACB_C_ABOVE_WAIT_MS = 2000
ACB_C_DROP_SETTLE_MS = 700
ACB_C_PICK_SETTLE_MS = 700

ARM_MOVE_MIN_WAIT_S   = 0.5
ARM_MOVE_EXTRA_WAIT_S = 0.2
ARM_MOVE_ACK_SLACK_S  = 15.0

GRIPPER_MOVE_TRAVEL_WAIT_SCALE = 0.35
GRIPPER_MOVE_MIN_WAIT_S        = 0.0
GRIPPER_MOVE_EXTRA_WAIT_S      = 0.0
GRIPPER_MOVE_ACK_SLACK_S       = 5.0


# ============================================================
# GENERIC SOURCE → DESTINATION LOOP
# ============================================================
DEFAULT_AB_PHASES = (
    MotionPhase("Recovery open gripper",   action="gripper", gripper=BLACK_GRIPPER_OPEN),
    MotionPhase("Approach source",         action="move",    station="SRC", pose="above", gripper=BLACK_GRIPPER_OPEN),
    MotionPhase("Descend to source pick",  action="move",    station="SRC", pose="pick",  gripper=BLACK_GRIPPER_OPEN),
    MotionPhase("Close gripper at source", action="gripper", gripper=BLACK_GRIPPER_CLOSED),
    MotionPhase("Pause after source grip", action="pause",   pause_ms=AB_PICK_SETTLE_DELAY_MS),
    MotionPhase("Lift from source",        action="move",    station="SRC", pose="above", gripper=BLACK_GRIPPER_CLOSED),
    MotionPhase("Transit to target",       action="move",    station="DST", pose="above", gripper=BLACK_GRIPPER_CLOSED),
    MotionPhase("Descend to target drop",  action="move",    station="DST", pose="pick",  gripper=BLACK_GRIPPER_CLOSED),
    MotionPhase("Open gripper at target",  action="gripper", gripper=BLACK_GRIPPER_OPEN),
    MotionPhase("Pause after target release", action="pause", pause_ms=AB_DROP_SETTLE_DELAY_MS),
    MotionPhase("Lift from target",        action="move",    station="DST", pose="above", gripper=BLACK_GRIPPER_OPEN),
    MotionPhase("Return above source",     action="move",    station="SRC", pose="above", gripper=BLACK_GRIPPER_OPEN, count_cycle=True),
)

# Alias kept for legacy imports
DEFAULT_ACB_PHASES = DEFAULT_AB_PHASES


# ============================================================
# BLACK ARM: A → C  (pick from A, drop at C, trigger press)
# During press: arm returns to A pick position so next cycle
# starts immediately once press is done.
# ============================================================
DEFAULT_BLACK_PHASES = (
    MotionPhase("Recovery open gripper",  action="gripper", gripper=BLACK_GRIPPER_OPEN),

    # Pick from A
    MotionPhase("Approach A",             action="move",    station="A", pose="above", gripper=BLACK_GRIPPER_OPEN),
    MotionPhase("Descend to A pick",      action="move",    station="A", pose="pick",  gripper=BLACK_GRIPPER_OPEN),
    MotionPhase("Close gripper at A",     action="gripper", gripper=BLACK_GRIPPER_CLOSED),

    # Carry to C and drop
    MotionPhase("Approach C with carry",  action="move",    station="C", pose="above", gripper=BLACK_GRIPPER_CLOSED),
    MotionPhase("Descend to C drop",      action="move",    station="C", pose="pick",  gripper=BLACK_GRIPPER_CLOSED),
    MotionPhase("Settle at C drop",       action="pause",   pause_ms=ACB_C_DROP_SETTLE_MS),
    MotionPhase("Open gripper at C",      action="gripper", gripper=BLACK_GRIPPER_OPEN),
    MotionPhase("Rise to C above",        action="move",    station="C", pose="above", gripper=BLACK_GRIPPER_OPEN),

    # Fire press signal (NON-BLOCKING — arm moves away immediately)
    MotionPhase("Fire press signal",      action="press_fire"),

    # Return to A pick position WHILE the press is running
    MotionPhase("Return to A above",      action="move",    station="A", pose="above", gripper=BLACK_GRIPPER_OPEN),
    MotionPhase("Descend to A pick pos",  action="move",    station="A", pose="pick",  gripper=BLACK_GRIPPER_OPEN),

    # Now block until press is done (White ARM picks from C after this)
    MotionPhase("Wait for press done",    action="press_wait", count_cycle=True),
)


# ============================================================
# WHITE ARM: C → B  (wait for press signal, pick C, place B)
# ============================================================
DEFAULT_WHITE_PHASES = (
    # Recovery: ensure gripper is open before entering wait state
    MotionPhase("Recovery open gripper",  action="gripper", gripper=WHITE_GRIPPER_OPEN),

    # Go to B above (standby / home position while waiting)
    MotionPhase("Standby at B above",     action="move",    station="B", pose="above", gripper=WHITE_GRIPPER_OPEN),

    # Wait for Press ESP2 to signal PICK_READY
    MotionPhase("Wait for press signal",  action="wait_for_press"),

    # Pick from C
    MotionPhase("Approach C",             action="move",    station="C", pose="above", gripper=WHITE_GRIPPER_OPEN),
    MotionPhase("Descend to C pick",      action="move",    station="C", pose="pick",  gripper=WHITE_GRIPPER_OPEN),
    MotionPhase("Close gripper at C",     action="gripper", gripper=WHITE_GRIPPER_CLOSED),
    MotionPhase("Rise to C above",        action="move",    station="C", pose="above", gripper=WHITE_GRIPPER_CLOSED),

    # Carry to B and place
    MotionPhase("Transit to B above",     action="move",    station="B", pose="above", gripper=WHITE_GRIPPER_CLOSED),
    MotionPhase("Descend to B pick",      action="move",    station="B", pose="pick",  gripper=WHITE_GRIPPER_CLOSED),
    MotionPhase("Open gripper at B",      action="gripper", gripper=WHITE_GRIPPER_OPEN),
    MotionPhase("Settle at B drop",       action="pause",   pause_ms=ACB_C_DROP_SETTLE_MS),

    # Return to B above ready for next cycle (count cycle here)
    MotionPhase("Rise to B above",        action="move",    station="B", pose="above", gripper=WHITE_GRIPPER_OPEN, count_cycle=True),
)


def run_motion_loop(
    *,
    loop_name: str,
    phases: tuple | list,
    is_running: Callable[[], bool],
    get_speed: Callable[[], int],
    get_initial_pose: Callable[[], list[int]],
    get_stations: Callable[[], dict],
    send_raw: Callable[[str], None],
    apply_pose: Callable[[list[int]], None],
    log_info: Callable[[str], None],
    log_error: Callable[[str], None],
    on_cycle_complete: Callable[[], None],
    get_ack_token: Callable[[str], int],
    wait_for_ack: Callable[[str, int, float], bool],
    is_press_enabled: Callable[[], bool] | None = None,
    max_cycles: int | None = None,
    start_phase_index: int = 0,
    on_phase_change: Callable[[int], None] | None = None,
) -> None:
    """Execute the configured phases until is_running() becomes False."""

    def sleep_interruptible(seconds: float) -> bool:
        deadline = time.time() + max(0.0, seconds)
        while time.time() < deadline:
            if not is_running():
                return False
            time.sleep(min(0.05, deadline - time.time()))
        return is_running()

    def resolve_pose(phase: MotionPhase, stations: dict) -> list[int]:
        if not phase.station or not phase.pose:
            raise ValueError(f"Phase '{phase.name}' is missing station/pose")
        station = stations.get(phase.station)
        if not station:
            raise KeyError(phase.station)
        pose = list(station[phase.pose])
        if phase.action == "retract":
            if phase.joint_index is None:
                raise ValueError(f"Phase '{phase.name}' is missing joint_index")
            pose[phase.joint_index] = max(0, pose[phase.joint_index] + phase.joint_delta)
        return pose

    def required_station_names() -> list[str]:
        names = []
        for phase in phases:
            if phase.station and phase.station not in names:
                names.append(phase.station)
        return names

    def move_to(pose: list[int], prev: list[int], phase: MotionPhase) -> list[int]:
        if len(pose) < len(prev):
            pose = list(pose) + prev[len(pose):]
        elif len(pose) > len(prev):
            raise ValueError(f"Phase '{phase.name}': pose length mismatch")
        if phase.gripper is not None and phase.gripper != prev[5]:
            log_error(
                f"{loop_name}: Phase '{phase.name}' expects gripper {phase.gripper} "
                f"but current gripper is {prev[5]}."
            )
            raise MotionTimeoutError(phase.name)
        pose[5] = prev[5]
        if pose == prev:
            apply_pose(list(pose))
            return list(pose)
        speed = get_speed()
        ack_token = get_ack_token("MOVE_DONE")
        send_raw(ServoController.move_cmd(pose, speed))
        max_travel = max(abs(a - b) for a, b in zip(pose, prev))
        wait_min_s   = ARM_MOVE_MIN_WAIT_S   if phase.wait_min_s   is None else phase.wait_min_s
        wait_extra_s = ARM_MOVE_EXTRA_WAIT_S if phase.wait_extra_s is None else phase.wait_extra_s
        wait_s = max(wait_min_s, speed * max_travel / 1000.0 + wait_extra_s) + ARM_MOVE_ACK_SLACK_S
        if not wait_for_ack("MOVE_DONE", ack_token, wait_s):
            log_error(f"{loop_name}: timeout waiting for MOVE_DONE after '{phase.name}'")
            send_raw(ServoController.stop_cmd())
            raise MotionTimeoutError(phase.name)
        if not is_running():
            return prev
        apply_pose(list(pose))
        return list(pose)

    def move_gripper(angle: int, prev: list[int], phase: MotionPhase) -> list[int]:
        if angle == prev[5]:
            apply_pose(list(prev))
            return list(prev)
        speed = get_speed()
        ack_key   = "SERVO_DONE:5"
        ack_token = get_ack_token(ack_key)
        send_raw(ServoController.single_cmd(5, angle, speed))
        travel = abs(angle - prev[5])
        wait_min_s   = GRIPPER_MOVE_MIN_WAIT_S   if phase.wait_min_s   is None else phase.wait_min_s
        wait_extra_s = GRIPPER_MOVE_EXTRA_WAIT_S if phase.wait_extra_s is None else phase.wait_extra_s
        travel_wait_s = speed * travel / 1000.0 * GRIPPER_MOVE_TRAVEL_WAIT_SCALE
        wait_s = max(wait_min_s, travel_wait_s + wait_extra_s) + GRIPPER_MOVE_ACK_SLACK_S
        if not wait_for_ack(ack_key, ack_token, wait_s):
            log_error(f"{loop_name}: timeout waiting for SERVO_DONE after '{phase.name}'")
            send_raw(ServoController.stop_cmd())
            raise MotionTimeoutError(phase.name)
        if not is_running():
            return prev
        nxt = list(prev)
        nxt[5] = angle
        apply_pose(list(nxt))
        return nxt

    def move_single_servo(idx: int, angle: int, prev: list[int], phase: MotionPhase) -> list[int]:
        if idx < 0 or idx >= len(prev):
            raise ValueError(f"Phase '{phase.name}' has invalid servo_index {idx}")
        if angle == prev[idx]:
            apply_pose(list(prev))
            return list(prev)
        speed = get_speed()
        ack_key   = f"SERVO_DONE:{idx}"
        ack_token = get_ack_token(ack_key)
        send_raw(ServoController.single_cmd(idx, angle, speed))
        travel = abs(angle - prev[idx])
        wait_min_s   = GRIPPER_MOVE_MIN_WAIT_S   if phase.wait_min_s   is None else phase.wait_min_s
        wait_extra_s = GRIPPER_MOVE_EXTRA_WAIT_S if phase.wait_extra_s is None else phase.wait_extra_s
        travel_wait_s = speed * travel / 1000.0 * GRIPPER_MOVE_TRAVEL_WAIT_SCALE
        wait_s = max(wait_min_s, travel_wait_s + wait_extra_s) + GRIPPER_MOVE_ACK_SLACK_S
        if not wait_for_ack(ack_key, ack_token, wait_s):
            log_error(f"{loop_name}: timeout waiting for SERVO_DONE after '{phase.name}'")
            send_raw(ServoController.stop_cmd())
            raise MotionTimeoutError(phase.name)
        if not is_running():
            return prev
        nxt = list(prev)
        nxt[idx] = angle
        apply_pose(list(nxt))
        return nxt

    cycle            = 0
    prev             = list(get_initial_pose())
    _press_fire_token = 0   # set by press_fire phase; consumed by press_wait phase

    while is_running() and (max_cycles is None or cycle < max_cycles):
        cycle += 1
        log_info(f"{loop_name} cycle #{cycle}")
        stations = get_stations()
        missing  = [n for n in required_station_names() if not stations.get(n)]
        if missing:
            log_error(f"Station(s) missing: {', '.join(missing)} — stopping {loop_name}")
            break

        phase_list_with_indices = []
        if cycle > 1:
            for idx, p in enumerate(phases):
                if loop_name == "BLACK ARM Loop" and p.name in ("Approach A", "Descend to A pick"):
                    continue
                phase_list_with_indices.append((idx, p))
        else:
            for idx, p in enumerate(phases):
                if idx >= start_phase_index:
                    phase_list_with_indices.append((idx, p))

        for actual_index, phase in phase_list_with_indices:
            if on_phase_change:
                on_phase_change(actual_index)
            if not is_running():
                break
            try:
                if phase.action in ("move", "retract"):
                    prev = move_to(resolve_pose(phase, stations), prev, phase)
                elif phase.action == "gripper":
                    if phase.gripper is None:
                        raise ValueError(f"Phase '{phase.name}' is missing gripper")
                    prev = move_gripper(phase.gripper, prev, phase)
                elif phase.action == "servo":
                    if phase.servo_index is None or phase.servo_angle is None:
                        raise ValueError(f"Phase '{phase.name}' missing servo_index/servo_angle")
                    prev = move_single_servo(phase.servo_index, phase.servo_angle, prev, phase)
                elif phase.action == "pause":
                    if not sleep_interruptible(phase.pause_ms / 1000.0):
                        break
                elif phase.action == "press_sync":
                    if not is_running():
                        break
                    if is_press_enabled is not None and not is_press_enabled():
                        log_info(f"{loop_name}: {phase.name} — Press Sync DISABLED, skipping")
                        continue
                    log_info(f"{loop_name}: {phase.name} — waiting for ESP2 press...")
                    ack_token = get_ack_token("PRESS_DONE")
                    send_raw("PRESS_SYNC")
                    if not wait_for_ack("PRESS_DONE", ack_token, PRESS_SYNC_TIMEOUT_S):
                        log_error(f"{loop_name}: {phase.name} — press failed or timed out")
                        break
                    elif not is_running():
                        break
                    else:
                        log_info(f"{loop_name}: {phase.name} — press completed")
                elif phase.action == "press_fire":
                    # Non-blocking press trigger — send PRESS_SYNC and continue
                    # immediately. Use press_wait phase later to collect PRESS_DONE.
                    if not is_running():
                        break
                    if is_press_enabled is not None and not is_press_enabled():
                        log_info(f"{loop_name}: {phase.name} — Press Sync DISABLED, skipping fire")
                        continue
                    # Record the token BEFORE sending so we don’t miss the ACK
                    _press_fire_token = get_ack_token("PRESS_DONE")
                    send_raw("PRESS_SYNC")
                    log_info(f"{loop_name}: {phase.name} — press signal fired, arm moving back to A")
                elif phase.action == "press_wait":
                    # Blocking wait for the PRESS_DONE ACK fired by press_fire
                    if not is_running():
                        break
                    if is_press_enabled is not None and not is_press_enabled():
                        log_info(f"{loop_name}: {phase.name} — Press Sync DISABLED, skipping wait")
                        continue
                    log_info(f"{loop_name}: {phase.name} — waiting for PRESS_DONE...")
                    if not wait_for_ack("PRESS_DONE", _press_fire_token, PRESS_SYNC_TIMEOUT_S):
                        log_error(f"{loop_name}: {phase.name} — press timed out or failed")
                        break
                    elif not is_running():
                        break
                    else:
                        log_info(f"{loop_name}: {phase.name} — press confirmed done")
                elif phase.action == "wait_for_press":
                    if not is_running():
                        break
                    if is_press_enabled is not None and not is_press_enabled():
                        log_info(f"{loop_name}: {phase.name} — Press Sync DISABLED, skipping wait")
                        continue
                    log_info(f"{loop_name}: {phase.name} — waiting for PICK_READY from press...")
                    ack_token = get_ack_token("PICK_READY_FROM_PRESS")
                    send_raw("WAIT_PICK_READY")
                    if not wait_for_ack("PICK_READY_FROM_PRESS", ack_token, PRESS_SYNC_TIMEOUT_S):
                        log_error(f"{loop_name}: {phase.name} — timeout waiting for PICK_READY")
                        break
                    elif not is_running():
                        break
                    else:
                        log_info(f"{loop_name}: {phase.name} — PICK_READY received, picking from C")
                else:
                    raise ValueError(f"Unsupported phase action: {phase.action}")
            except MotionTimeoutError:
                return

            if phase.count_cycle and is_running():
                on_cycle_complete()
