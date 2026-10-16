"""Plan and execute a named posture through move_group.

Everything else in this stack STREAMS joint commands, which is right for
teleoperation and wrong for the one motion the operator is not driving. A
streamed command carries no path: the controller interpolates from wherever the
arm is to wherever it was told, straight through anything in between. For a
90 Hz correction that is fine, because the next correction is 11 ms away. For
homing -- the largest motion the arm ever makes, across several joints, from an
arbitrary posture -- it means the gripper can sweep through the table we went to
the trouble of modelling.

So named-posture moves go through move_group's MoveGroup action instead, which
plans around the planning scene, time-parameterises against the robot's own
velocity and acceleration limits, and reports a real result. It reaches the arm
through FollowJointTrajectory, which is the one execution interface every vendor
driver implements.

JOINT goals, not pose goals. A named state is a configuration, and asking for
the pose it happens to produce invites the planner to solve a different IK
branch -- an elbow flipped the other way reaches the same pose and is not the
posture anyone asked for.

Nothing here blocks. The caller owns a state machine with a watchdog on it, and
``spin_until_future_complete`` inside that machine's timer would re-enter the
executor and deadlock the thing that is supposed to be watching for faults.
"""

from __future__ import annotations

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes

#: ``-10 -> 'START_STATE_IN_COLLISION'``, for log lines a human has to read.
ERROR_NAMES = {
    value: name for name, value in vars(MoveItErrorCodes).items()
    if name.isupper() and isinstance(value, int)
}

#: Planner verdicts that justify falling back to a direct interpolated move.
#:
#: These all mean "move_group could not produce a path", with the arm still
#: sitting where it started and nothing executed. The fallback is then exactly
#: what this stack did before it could plan at all: no worse, and it keeps a
#: conservative scene model from turning into an arm that will not start.
#:
#: START_STATE_IN_COLLISION is the one that matters in practice. A planner
#: refuses to plan out of a colliding start state, and a gripper resting on a
#: modelled table legitimately IS in collision -- so the most ordinary way to
#: leave an arm is also the way to make planning impossible.
FALLBACK_CODES = frozenset({
    MoveItErrorCodes.PLANNING_FAILED,
    MoveItErrorCodes.TIMED_OUT,
    MoveItErrorCodes.START_STATE_IN_COLLISION,
    MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS,
    MoveItErrorCodes.START_STATE_INVALID,
    MoveItErrorCodes.INVALID_MOTION_PLAN,
})

#: Verdicts that must NOT fall back, with the reason, because interpolating
#: blindly afterwards is worse than stopping and saying so.
NO_FALLBACK_REASONS = {
    MoveItErrorCodes.CONTROL_FAILED:
        'execution began and failed, so the arm is part-way along a path and '
        'its posture is not what either end of this move assumed',
    MoveItErrorCodes.GOAL_IN_COLLISION:
        'the destination posture is itself in collision -- driving there '
        'blindly is driving into known geometry, and the scene model or the '
        'named state is wrong',
    MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS:
        'the destination violates a path constraint',
    MoveItErrorCodes.INVALID_GROUP_NAME:
        'the planning group does not exist, which is a configuration error no '
        'amount of retrying fixes',
    MoveItErrorCodes.PREEMPTED:
        'the move was cancelled deliberately',
}


def error_name(code: int) -> str:
    return ERROR_NAMES.get(int(code), f'code {int(code)}')


def should_fall_back(code: int) -> bool:
    """Is a direct interpolated move an acceptable answer to this verdict?

    Unknown codes do NOT fall back. A verdict nobody has reasoned about is not
    evidence that moving anyway is safe.
    """
    return int(code) in FALLBACK_CODES


def no_fallback_reason(code: int) -> str:
    """Why this verdict is not recoverable by moving anyway."""
    return NO_FALLBACK_REASONS.get(
        int(code), 'this verdict has no reasoned fallback, so the arm holds')


class NamedMoveClient:
    """Non-blocking plan-and-execute for one joint-space goal at a time.

    ``send`` returns False when there is nothing to send to, which is the
    caller's cue to fall back rather than wait: an arm whose MoveIt stack has
    no action server is a legitimate configuration, not a fault.
    """

    def __init__(self, node, group: str, action_name: str = '/move_action',
                 planning_time: float = 5.0, attempts: int = 2,
                 scaling: float = 0.3, tolerance: float = 0.01, client=None):
        from rclpy.action import ActionClient
        self.node = node
        self.group = group
        self.action_name = action_name
        self.planning_time = float(planning_time)
        self.attempts = int(attempts)
        self.scaling = float(scaling)
        self.tolerance = float(tolerance)
        self._client = client if client is not None else ActionClient(
            node, MoveGroup, action_name)
        self._goal_handle = None
        self._in_flight = False

    # ---- state ---------------------------------------------------------

    @property
    def in_flight(self) -> bool:
        return self._in_flight

    def server_ready(self, timeout_sec: float = 0.0) -> bool:
        if timeout_sec > 0.0:
            return self._client.wait_for_server(timeout_sec=timeout_sec)
        return self._client.server_is_ready()

    # ---- sending -------------------------------------------------------

    def goal_for(self, targets: dict) -> MoveGroup.Goal:
        """A plan-and-execute goal for ``{joint_name: position}``."""
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = self.group
        request.num_planning_attempts = self.attempts
        request.allowed_planning_time = self.planning_time
        request.max_velocity_scaling_factor = self.scaling
        request.max_acceleration_scaling_factor = self.scaling

        constraints = Constraints(name=f'{self.group}_named_state')
        for name, position in targets.items():
            constraints.joint_constraints.append(JointConstraint(
                joint_name=name, position=float(position),
                tolerance_above=self.tolerance,
                tolerance_below=self.tolerance, weight=1.0))
        request.goal_constraints.append(constraints)

        # is_diff on BOTH, and this is load-bearing rather than tidy: an
        # unflagged empty PlanningScene in a goal is a scene, not an absence of
        # one, and move_group would plan against a world with no table in it.
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        return goal

    def send(self, targets: dict, on_result) -> bool:
        """Start a move. ``on_result(code)`` is called exactly once, later.

        Returns False, having called nothing, when there is no server to send
        to or a move is already in flight.
        """
        if self._in_flight or not self._client.server_is_ready():
            return False

        self._in_flight = True
        future = self._client.send_goal_async(self.goal_for(targets))
        future.add_done_callback(
            lambda f: self._on_goal_response(f, on_result))
        return True

    def cancel(self) -> None:
        """Stop a move in flight. The result callback still fires, PREEMPTED."""
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

    # ---- callbacks -----------------------------------------------------

    def _finish(self, code: int, on_result) -> None:
        self._in_flight = False
        self._goal_handle = None
        on_result(int(code))

    def _on_goal_response(self, future, on_result) -> None:
        try:
            handle = future.result()
        except Exception as exc:                        # noqa: BLE001
            # Nothing was accepted, so nothing executed: this is the same
            # situation as a refused plan, not an interrupted move.
            self.node.get_logger().warn(f'{self.action_name} goal failed: {exc}')
            self._finish(MoveItErrorCodes.PLANNING_FAILED, on_result)
            return
        if handle is None or not handle.accepted:
            # A rejected goal never reached a planner, so nothing has been
            # judged unsafe -- treat it like an absent server.
            self._finish(MoveItErrorCodes.PLANNING_FAILED, on_result)
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self._on_result(f, on_result))

    def _on_result(self, future, on_result) -> None:
        try:
            result = future.result()
        except Exception as exc:                        # noqa: BLE001
            # The goal WAS accepted, so execution may have begun and the arm
            # may be part-way along a path. FAILURE does not fall back.
            self.node.get_logger().warn(f'{self.action_name} result failed: {exc}')
            self._finish(MoveItErrorCodes.FAILURE, on_result)
            return
        code = MoveItErrorCodes.FAILURE
        if result is not None and getattr(result, 'result', None) is not None:
            code = result.result.error_code.val
        self._finish(code, on_result)
