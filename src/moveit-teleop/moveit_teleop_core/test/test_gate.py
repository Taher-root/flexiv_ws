"""When the hardware gate is open.

The gate follows the command PATH rather than the mode, because the two kinds
of gate real drivers advertise want opposite things during a planned move. The
Piper's CAN gate enables motion, so it must be open for move_group's trajectory
to reach the arm. The Flexiv driver's set_teleop_mode selects a servo streaming
mode, and its own FollowJointTrajectory action rejects any goal while that mode
is active -- so for a planned move the same gate must be SHUT.

Getting this wrong is silent: on the Flexiv the trajectory goal is rejected,
the planner reads a rejected goal as PLANNING_FAILED, that is a fallback code,
and so every planned home quietly becomes the unchecked interpolated one.
"""

import pytest
from moveit_teleop_msgs.msg import TeleopStatus

from moveit_teleop_core.mode_manager import gate_open_for

MOTION_ENABLE = True       # the Piper's CAN gate
STREAMING_MODE = False     # the Flexiv driver's set_teleop_mode


@pytest.mark.parametrize('kind', [MOTION_ENABLE, STREAMING_MODE])
def test_teleoperation_always_opens_the_gate(kind):
    # Streaming is unambiguous: the command goes to the controller topic and
    # needs the gate open whatever kind of gate it is.
    assert gate_open_for(TeleopStatus.TELEOP, None, kind) is True


@pytest.mark.parametrize('kind', [MOTION_ENABLE, STREAMING_MODE])
@pytest.mark.parametrize('mode', [TeleopStatus.IDLE, TeleopStatus.ALIGNED,
                                 TeleopStatus.HALTED])
def test_nothing_else_opens_it(mode, kind):
    assert gate_open_for(mode, None, kind) is False


def test_a_planned_move_needs_a_motion_enable_open():
    assert gate_open_for(TeleopStatus.PLANNING, 'moveit', MOTION_ENABLE) is True


def test_a_planned_move_needs_a_streaming_mode_shut():
    # Otherwise the driver rejects the goal it was just sent.
    assert gate_open_for(TeleopStatus.PLANNING, 'moveit', STREAMING_MODE) is False


@pytest.mark.parametrize('kind', [MOTION_ENABLE, STREAMING_MODE])
def test_the_homing_FALLBACK_always_opens_the_gate(kind):
    # The fallback streams to the controller topic exactly as teleop does, so
    # it needs the gate open even on an arm whose planned path needed it shut.
    assert gate_open_for(TeleopStatus.PLANNING, 'direct', kind) is True


@pytest.mark.parametrize('kind', [MOTION_ENABLE, STREAMING_MODE])
def test_planning_entered_from_outside_is_treated_as_trajectory_execution(kind):
    # ~/set_mode can put the manager in PLANNING with no move of its own, so
    # move_group is executing something: the trajectory rule applies.
    assert gate_open_for(TeleopStatus.PLANNING, None, kind) is bool(kind)
