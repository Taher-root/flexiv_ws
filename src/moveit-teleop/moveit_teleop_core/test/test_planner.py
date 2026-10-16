"""Planning a named posture, and what happens when the planner says no.

The fallback policy is the whole point of these tests. Replacing a move that
was never collision-checked with one that can be REFUSED trades an unguarded
motion for an arm that sometimes will not start, and which verdicts justify
moving anyway is the decision that makes that trade acceptable.
"""

import pytest
from moveit_msgs.msg import MoveItErrorCodes as Codes

from moveit_teleop_core.planner import (
    NamedMoveClient, error_name, no_fallback_reason, should_fall_back,
)

TARGETS = {'joint2': 1.65, 'joint3': -0.8}


class _Future:
    def __init__(self, result=None, raises=None):
        self._result, self._raises, self._done = result, raises, []

    def add_done_callback(self, callback):
        self._done.append(callback)
        callback(self)

    def result(self):
        if self._raises is not None:
            raise self._raises
        return self._result


class _Handle:
    def __init__(self, accepted=True, result_future=None):
        self.accepted = accepted
        self._result_future = result_future
        self.cancelled = False

    def get_result_async(self):
        return self._result_future

    def cancel_goal_async(self):
        self.cancelled = True


def _result(code):
    class _Inner:
        class result:                                   # noqa: N801
            class error_code:                           # noqa: N801
                val = code
    return _Inner


class _Client:
    def __init__(self, ready=True, handle=None, goal_raises=None):
        self.ready, self.handle, self.goal_raises = ready, handle, goal_raises
        self.goals = []

    def server_is_ready(self):
        return self.ready

    def wait_for_server(self, timeout_sec=None):
        return self.ready

    def send_goal_async(self, goal):
        self.goals.append(goal)
        return _Future(result=self.handle, raises=self.goal_raises)


class _Logger:
    def __init__(self):
        self.warnings = []

    def warn(self, message, **_):
        self.warnings.append(message)


class _Node:
    def __init__(self):
        self._logger = _Logger()

    def get_logger(self):
        return self._logger


def client_for(code=Codes.SUCCESS, accepted=True, ready=True, goal_raises=None,
               result_raises=None):
    handle = _Handle(accepted=accepted,
                     result_future=_Future(result=_result(code),
                                           raises=result_raises))
    node = _Node()
    return NamedMoveClient(node, 'arm', client=_Client(
        ready=ready, handle=handle, goal_raises=goal_raises)), node, handle


# ---- the goal ------------------------------------------------------------

def test_the_goal_asks_for_the_joint_configuration_not_a_pose():
    # A pose goal would let the planner pick a different IK branch: same tool
    # pose, elbow the other way round, not the posture anyone named.
    mover, _, _ = client_for()
    goal = mover.goal_for(TARGETS)
    assert goal.request.group_name == 'arm'
    assert not goal.request.goal_constraints[0].position_constraints
    assert not goal.request.goal_constraints[0].orientation_constraints
    named = {c.joint_name: c.position
             for c in goal.request.goal_constraints[0].joint_constraints}
    assert named == pytest.approx(TARGETS)


def test_the_goal_plans_and_executes_rather_than_planning_only():
    goal = client_for()[0].goal_for(TARGETS)
    assert goal.planning_options.plan_only is False


def test_the_scene_diff_is_flagged_as_a_diff():
    """An unflagged empty PlanningScene IS a scene: a world with no table."""
    goal = client_for()[0].goal_for(TARGETS)
    assert goal.planning_options.planning_scene_diff.is_diff is True
    assert goal.planning_options.planning_scene_diff.robot_state.is_diff is True


def test_limits_are_scaled_for_a_posture_move():
    mover, _, _ = client_for()
    mover.scaling = 0.25
    goal = mover.goal_for(TARGETS)
    assert goal.request.max_velocity_scaling_factor == pytest.approx(0.25)
    assert goal.request.max_acceleration_scaling_factor == pytest.approx(0.25)


# ---- sending -------------------------------------------------------------

def test_a_successful_move_reports_success_once():
    mover, _, _ = client_for(Codes.SUCCESS)
    seen = []
    assert mover.send(TARGETS, seen.append) is True
    assert seen == [Codes.SUCCESS]
    assert not mover.in_flight


def test_no_server_refuses_to_send_so_the_caller_can_fall_back():
    mover, _, _ = client_for(ready=False)
    seen = []
    assert mover.send(TARGETS, seen.append) is False
    assert seen == []           # nothing promised, nothing called


def test_a_rejected_goal_is_reported_as_a_planning_failure():
    # Rejected means no planner ever looked at it, so nothing has been judged
    # unsafe -- which is why this is a fallback code.
    mover, _, _ = client_for(accepted=False)
    seen = []
    mover.send(TARGETS, seen.append)
    assert seen == [Codes.PLANNING_FAILED]
    assert should_fall_back(seen[0])


def test_a_throwing_goal_call_is_also_only_a_planning_failure():
    mover, node, _ = client_for(goal_raises=RuntimeError('no transport'))
    seen = []
    mover.send(TARGETS, seen.append)
    assert seen == [Codes.PLANNING_FAILED]
    assert node.get_logger().warnings


def test_a_throwing_result_does_NOT_fall_back():
    # The goal was accepted, so execution may have begun; the arm could be
    # part-way along a path and interpolating from there is not safe.
    mover, _, _ = client_for(result_raises=RuntimeError('lost'))
    seen = []
    mover.send(TARGETS, seen.append)
    assert seen == [Codes.FAILURE]
    assert not should_fall_back(seen[0])


def test_cancelling_reaches_the_goal_handle():
    mover, _, handle = client_for()
    mover._goal_handle = handle
    mover.cancel()
    assert handle.cancelled


# ---- the fallback policy -------------------------------------------------

@pytest.mark.parametrize('code', [
    Codes.PLANNING_FAILED, Codes.TIMED_OUT, Codes.START_STATE_IN_COLLISION,
    Codes.START_STATE_INVALID, Codes.START_STATE_VIOLATES_PATH_CONSTRAINTS,
    Codes.INVALID_MOTION_PLAN,
])
def test_a_refused_plan_may_fall_back(code):
    # All of these leave the arm where it started with nothing executed.
    assert should_fall_back(code)


@pytest.mark.parametrize('code', [
    Codes.SUCCESS, Codes.CONTROL_FAILED, Codes.GOAL_IN_COLLISION,
    Codes.GOAL_VIOLATES_PATH_CONSTRAINTS, Codes.INVALID_GROUP_NAME,
    Codes.PREEMPTED, Codes.FAILURE,
])
def test_these_verdicts_must_not_fall_back(code):
    assert not should_fall_back(code)


def test_an_unknown_verdict_does_not_fall_back():
    # A code nobody has reasoned about is not evidence that moving is safe.
    assert not should_fall_back(-424242)
    assert 'no reasoned fallback' in no_fallback_reason(-424242)


def test_every_refusal_can_explain_itself():
    for code in (Codes.CONTROL_FAILED, Codes.GOAL_IN_COLLISION,
                 Codes.INVALID_GROUP_NAME, Codes.PREEMPTED):
        assert len(no_fallback_reason(code)) > 20


def test_codes_are_named_for_the_log():
    assert error_name(Codes.START_STATE_IN_COLLISION) == 'START_STATE_IN_COLLISION'
    assert error_name(-424242) == 'code -424242'
