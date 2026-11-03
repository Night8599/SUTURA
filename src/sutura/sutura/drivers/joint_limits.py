"""Joint limit definitions and command validation for robotic arm drivers."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class JointLimit:
    """Position limit of a single revolute joint, in radians."""

    name: str
    lower: float
    upper: float

    def contains(self, value: float, tolerance: float = 0.0) -> bool:
        return (self.lower - tolerance) <= value <= (self.upper + tolerance)


def _deg(value: float) -> float:
    return math.radians(value)


# Niryo Ned (Ned1) limits, taken from niryo_robot_description/urdf/ned/niryo_ned_param.urdf.xacro.
# A safety margin is applied on top of these by the driver (see `safety_margin_rad`).
NED1_JOINT_LIMITS: tuple[JointLimit, ...] = (
    JointLimit('joint_1', _deg(-170.00), _deg(170.00)),
    JointLimit('joint_2', _deg(-109.44), _deg(34.96)),
    JointLimit('joint_3', _deg(-76.78), _deg(89.96)),
    JointLimit('joint_4', _deg(-119.75), _deg(119.75)),
    JointLimit('joint_5', _deg(-110.01), _deg(60.17)),
    JointLimit('joint_6', _deg(-144.96), _deg(144.96)),
)


class JointCommandError(ValueError):
    """Raised when a joint command is rejected by validation."""


def validate_joint_command(
    target: list[float],
    limits: tuple[JointLimit, ...],
    safety_margin_rad: float = 0.0,
    current: list[float] | None = None,
    max_step_rad: float | None = None,
) -> None:
    """
    Validates a joint position command. Raises JointCommandError on the first violation.

    Out-of-range commands are rejected, never clamped: a silently clamped command would
    move the arm somewhere the planner did not ask for.

    Args:
        target: Desired joint positions in radians.
        limits: One JointLimit per joint.
        safety_margin_rad: Extra margin kept away from each hardware limit.
        current: Current joint positions, required to check `max_step_rad`.
        max_step_rad: Maximum allowed displacement of any joint in a single command.
    """
    if len(target) != len(limits):
        raise JointCommandError(f"Expected {len(limits)} joint values, received {len(target)}.")

    for value, limit in zip(target, limits):
        if not math.isfinite(value):
            raise JointCommandError(f"{limit.name}: non-finite value {value}.")
        if not limit.contains(value, tolerance=-safety_margin_rad):
            raise JointCommandError(
                f"{limit.name}: {value:.4f} rad outside "
                f"[{limit.lower + safety_margin_rad:.4f}, {limit.upper - safety_margin_rad:.4f}] rad."
            )

    if max_step_rad is not None and current is not None:
        for value, now, limit in zip(target, current, limits):
            step = abs(value - now)
            if step > max_step_rad:
                raise JointCommandError(
                    f"{limit.name}: step of {step:.4f} rad exceeds max_step_rad={max_step_rad:.4f}."
                )
