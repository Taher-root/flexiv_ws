"""Full-IK teleoperation solver: pose target in, joint trajectory out.

A drop-in replacement for ``moveit_servo`` in the teleop pipeline. It consumes
the same ``geometry_msgs/PoseStamped`` the retargeter already publishes and
drives the same JointTrajectoryController, so nothing upstream or downstream
changes.

The behavioural difference is the point: servo inverts the Jacobian locally and
halts a joint that reaches its bound, leaving the arm jammed. This solves a
constrained problem every frame with the limits as constraints, so it
reconfigures instead.

Everything robot-specific is read from ``robot_description`` and
``robot_description_semantic`` at startup, so no link or joint name appears
here.
"""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.parameter import Parameter
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from moveit_teleop_core.controllers import (
    controller_for_joints, fetch_controller_map, fetch_joint_limits,
    fetch_velocity_scaling,
)
from moveit_teleop_core.robot_info import RobotInfo, fetch_descriptions
from moveit_teleop_core.scene_config import resolve_planes
from moveit_teleop_msgs.msg import IKStatus

from .chain import Mount, SerialChain
from .collision import POLICIES, StateValidityMonitor
from .ik_solver import IKSolver, IKWeights
from .smoothing import DEFAULT_WEIGHTS, JointRateLimiter, WeightedMovingAverage

#: Above this Jacobian condition number a nominal posture is a bad idea: the
#: regularisation term would bias every solve toward a singularity. The Piper's
#: zero pose scores ~29000 and its joint mid-range ~34000, so this is not
#: hypothetical.
POORLY_CONDITIONED = 500.0

#: Ask for the joint mid-range as the regularisation target instead of a named
#: SRDF state. Maximises limit headroom; conditioning depends on the arm.
MIDRANGE = 'midrange'


class IKNode(Node):

    def __init__(self):
        super().__init__('ik_node')

        self.declare_parameter('move_group', 'arm')
        self.declare_parameter('description_source', '/move_group')
        self.declare_parameter('nominal_state', 'ready_pose')
        self.declare_parameter('controller', '')          # '' -> derive
        self.declare_parameter('joint_states_topic', '/control/joint_states')
        self.declare_parameter('target_topic', '~/pose_target_cmds')
        self.declare_parameter('solve_rate', 90.0)
        self.declare_parameter('command_time', 0.0)       # 0 -> one period

        self.declare_parameter('weight_position', 50.0)
        self.declare_parameter('weight_orientation', 10.0)
        self.declare_parameter('weight_regularization', 0.005)
        self.declare_parameter('weight_smoothness', 0.1)
        self.declare_parameter('limit_margin', 0.01)
        self.declare_parameter('max_iter', 30)

        self.declare_parameter('smoothing_weights', list(DEFAULT_WEIGHTS))
        # Fraction of the joint's maximum velocity teleoperation may use.
        self.declare_parameter('rate_limit_scale', 0.3)
        self.declare_parameter('use_moveit_limits', True)
        # MoveIt's default_velocity_scaling_factor is a PLANNING default (0.1
        # on this robot), not a safety ceiling. Folding it in as well made the
        # arm crawl at 0.5 rad/s, so it is opt-in.
        self.declare_parameter('use_moveit_velocity_scaling', False)
        # A solution further than this from the CURRENT measured configuration
        # is treated as an IK branch flip and refused. Valid IK, unacceptable
        # motion: the real arm must not be asked to swing through it.
        self.declare_parameter('max_joint_jump', 0.35)
        # On a refused jump, the solve is RETRIED once with the smoothness
        # weight multiplied by this, seeded from the last accepted
        # solution. Refusing on its own leaves the target unchanged, so the
        # next solve rediscovers the branch that was just rejected and the
        # arm sits still -- which is what 'sluggish near a surface' was.
        # The retry costs one extra solve (~1.4 ms against an 11 ms budget)
        # and only on frames that would otherwise have been dropped.
        # 0 disables the retry and restores plain refusal.
        self.declare_parameter('jump_retry_smoothness', 50.0)
        self.declare_parameter('target_timeout', 0.5)
        # 'none' | 'stop'. Deliberately NOT spelled 'off': YAML parses
        # off/on/yes/no as BOOLEANS, so 'off' would arrive here as False and
        # fail a string parameter.
        self.declare_parameter('collision_policy', 'stop')
        self.declare_parameter('collision_check_rate', 20.0)
        self.declare_parameter('collision_service', '/check_state_validity')
        # Virtual walls: flat [nx, ny, nz, offset, ...] enforcing
        # n . p >= offset on the tool, in the planning frame. A table the tool
        # must stay above at z = 0.05 is [0, 0, 1, 0.05].
        # Declared by TYPE rather than value: an empty list gives rclpy nothing
        # to infer from, and a typed-but-empty default counts as uninitialized,
        # which raises on read. get_parameter_or below supplies the empty case.
        self.declare_parameter('planes', Parameter.Type.DOUBLE_ARRAY)
        # Declares the same obstacles the collision checker sees. A box's
        # wall is derived from that box's bounds, so the surface the tool
        # slides along cannot drift from the one the arm is stopped at.
        self.declare_parameter('scene_config', '')

        group = self.get_parameter('move_group').value
        source = self.get_parameter('description_source').value

        # ---- robot model, entirely from MoveIt's own artifacts ------------
        urdf, srdf = fetch_descriptions(self, source)
        robot = RobotInfo.from_xml(urdf, srdf)
        base_link, tip_link = robot.group_chain(group)
        self.planning_frame = robot.model_frame
        self.chain = SerialChain(urdf, base_link, tip_link)
        # Targets are in the planning frame, the chain's FK starts at
        # base_link. Ignoring the difference solves for a pose displaced by
        # the whole mount -- the arm reaches for it and straightens out.
        self.mount = Mount(urdf, self.planning_frame, base_link)

        controller = self.get_parameter('controller').value
        if not controller:
            controller = controller_for_joints(
                fetch_controller_map(self, source), self.chain.joint_names)
        self.controller = controller

        nominal = self._resolve_nominal(robot, group)

        weights = IKWeights(
            position=float(self.get_parameter('weight_position').value),
            orientation=float(self.get_parameter('weight_orientation').value),
            regularization=float(self.get_parameter('weight_regularization').value),
            smoothness=float(self.get_parameter('weight_smoothness').value))
        planes = self._parse_planes(
            self.get_parameter_or(
                'planes',
                Parameter('planes', Parameter.Type.DOUBLE_ARRAY, [])).value)
        # Walls from the scene config come AFTER any given directly through
        # the `planes` parameter, and that order is what active_planes indexes
        # into. Directly-given planes have no footprint and are always on.
        scene_walls = resolve_planes(
            self, self.get_parameter('scene_config').value,
            self.planning_frame)
        self._plane_footprints = [None] * len(planes) + list(scene_walls)
        planes += [(w.normal, w.offset) for w in scene_walls]
        self.solver = IKSolver(
            self.chain, nominal=nominal, weights=weights,
            limit_margin=float(self.get_parameter('limit_margin').value),
            max_iter=int(self.get_parameter('max_iter').value),
            planes=planes)
        for normal, offset in planes:
            self.get_logger().info(
                f'virtual wall: {normal.round(3)} . p >= {offset:.3f} '
                '(a constraint, so the tool slides along it)')

        self.filter = WeightedMovingAverage(
            self.get_parameter('smoothing_weights').value)

        velocity, scale = self._resolve_motion_limits(source)
        self.limiter = JointRateLimiter(velocity, scale=scale)
        self.max_joint_jump = float(self.get_parameter('max_joint_jump').value)
        self.jump_retry_smoothness = float(
            self.get_parameter('jump_retry_smoothness').value)

        self.policy = str(self.get_parameter('collision_policy').value)
        if self.policy not in POLICIES:
            raise RuntimeError(
                f'collision_policy must be one of {POLICIES}, '
                f'got {self.policy!r}')
        self.collision = None
        if self.policy == 'stop':
            self.collision = StateValidityMonitor(
                self, group, self.chain.joint_names,
                service=self.get_parameter('collision_service').value,
                rate=float(self.get_parameter('collision_check_rate').value))

        self.get_logger().info(
            f"group='{group}' chain '{base_link}' -> '{tip_link}' "
            f'({self.chain.nq} joints) controller=\'{controller}\' '
            f"planning_frame='{self.planning_frame}' "
            '(all resolved from the robot model)')
        if self.mount.joint_names:
            self.get_logger().info(
                f"'{base_link}' is mounted on '{self.planning_frame}' through "
                f'{self.mount.joint_names}; tracked live from joint states')

        # ---- warm up -----------------------------------------------------
        # The first CasADi/IPOPT solve costs ~124 ms against a steady state of
        # ~0.7 ms. Paying that here rather than on the operator's first frame.
        pose = self.chain.forward(nominal)
        self.solver.solve(pose[:3, 3], pose[:3, :3])
        self.solver.reset(nominal)
        self._announce_guards()
        self.get_logger().info('solver warmed up')

        # ---- io ----------------------------------------------------------
        self.paused = True          # nothing moves until the mode manager says so
        self._target = None
        self._target_time = 0.0
        self._joint_state = {}
        self._solve_times = []
        self._last_result = None
        self._rate_limited = False
        self._refused_jumps = 0
        self._retried_jumps = 0
        self._last_solution_q = None
        self.target_timeout = float(self.get_parameter('target_timeout').value)

        self.rate = float(self.get_parameter('solve_rate').value)
        self.period = 1.0 / self.rate
        self.command_time = float(self.get_parameter('command_time').value) \
            or self.period

        self.command_pub = self.create_publisher(
            JointTrajectory, f'{controller}/joint_trajectory', 10)
        self.status_pub = self.create_publisher(IKStatus, '~/status', 10)
        self.create_subscription(
            PoseStamped, self.get_parameter('target_topic').value,
            self._on_target, 10)
        self.create_subscription(
            JointState, self.get_parameter('joint_states_topic').value,
            self._on_joint_states, 10)
        self.create_service(SetBool, '~/pause', self._on_pause)

        self.create_timer(self.period, self._tick)
        self.create_timer(0.1, self._publish_status)
        self.get_logger().info(f'solving at {self.rate:.0f} Hz, paused until enabled')

    # ---- setup helpers ----------------------------------------------------

    def _resolve_motion_limits(self, source: str):
        """Velocity/acceleration limits and the speed scale.

        Prefers MoveIt's joint_limits.yaml over the URDF. They can differ by an
        order of magnitude -- on the Piper the URDF says 5.0 rad/s while MoveIt
        applies a 0.1 scaling factor on top -- and streaming commands must obey
        the same ceiling as planned motions, or teleoperation moves the real arm
        far faster than anything else ever asks it to.
        """
        names = self.chain.joint_names
        velocity = np.array(self.chain.velocity_limits, float)
        scale = float(self.get_parameter('rate_limit_scale').value)
        origin = 'URDF'

        if self.get_parameter('use_moveit_limits').value:
            limits = fetch_joint_limits(self, names, source)
            if limits:
                origin = 'MoveIt joint_limits.yaml'
                for i, joint in enumerate(names):
                    entry = limits.get(joint, {})
                    if 'max_velocity' in entry:
                        velocity[i] = entry['max_velocity']
            if self.get_parameter('use_moveit_velocity_scaling').value:
                moveit_scale = fetch_velocity_scaling(self, source)
                scale *= moveit_scale
                origin += f" x planning scale {moveit_scale:g}"

        scale = min(max(scale, 1e-6), 1.0)
        self.get_logger().info(
            f'speed from {origin} x rate_limit_scale '
            f'{self.get_parameter("rate_limit_scale").value:g} -> '
            f'{np.round(velocity * scale, 2)} rad/s')
        return velocity, scale

    @staticmethod
    def _parse_planes(flat) -> list:
        """Flat [nx, ny, nz, offset, ...] -> [(normal, offset), ...].

        Flat because ROS parameters have no nested-array type.
        """
        values = list(flat or [])
        if len(values) % 4:
            raise RuntimeError(
                f"'planes' needs groups of 4 (nx, ny, nz, offset); got "
                f'{len(values)} values')
        return [(np.array(values[i:i + 3], float), float(values[i + 3]))
                for i in range(0, len(values), 4)]

    def _resolve_nominal(self, robot: RobotInfo, group: str) -> np.ndarray:
        """Posture the solver is regularised toward.

        Prefers an SRDF named state. The mid-range fallback is deliberately
        checked rather than trusted: on the Piper it is a worse singularity
        than the zero pose, and regularising toward it would bias every solve
        into that singularity.
        """
        name = str(self.get_parameter('nominal_state').value)
        order = self.chain.joint_names

        if name == MIDRANGE:
            # Explicitly requested. Maximises distance from every joint limit,
            # at the cost of whatever conditioning that posture happens to have
            # -- which on some arms is poor. The check below reports which.
            nominal = self.chain.midrange()
            origin = 'joint mid-range (requested)'
        else:
            try:
                state = robot.named_state(group, name)
                nominal = np.array([state.get(j, 0.0) for j in order])
                origin = f"SRDF named state '{name}'"
            except Exception:  # noqa: BLE001 - a normal, reportable case
                nominal = self.chain.midrange()
                origin = 'joint mid-range (no named state found)'

        condition = self.chain.condition_number(nominal)
        headroom = float(np.min(np.minimum(nominal - self.chain.lower,
                                           self.chain.upper - nominal)))
        if condition > POORLY_CONDITIONED:
            self.get_logger().warn(
                f'nominal posture from {origin} is poorly conditioned '
                f'(Jacobian condition number {condition:.0f}, joint-limit '
                f'headroom {headroom:.2f} rad). The solver is biased toward a '
                'near-singular configuration; with a small regularisation '
                'weight the effect is mild, but expect worse behaviour near '
                'singularities.')
        else:
            self.get_logger().info(
                f'nominal posture from {origin}, condition number '
                f'{condition:.1f}, joint-limit headroom {headroom:.2f} rad')
        return nominal

    # ---- io ---------------------------------------------------------------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_target(self, msg: PoseStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.planning_frame:
            # Refusing beats silently solving for a pose in the wrong frame,
            # which looks like a control bug and is very hard to see.
            self.get_logger().warn(
                f"ignoring target in frame '{msg.header.frame_id}'; expected "
                f"'{self.planning_frame}'", throttle_duration_sec=5.0)
            return
        p = msg.pose.position
        o = msg.pose.orientation
        self._target = (np.array([p.x, p.y, p.z]),
                        Rotation.from_quat([o.x, o.y, o.z, o.w]).as_matrix())
        self._target_time = self._now()

    def _on_joint_states(self, msg: JointState) -> None:
        self._joint_state.update(zip(msg.name, msg.position))

    def _on_pause(self, request, response):
        self.paused = bool(request.data)
        # Reseed on BOTH edges, and the resume edge is the one that matters.
        #
        # Reseeding only on pause leaves the solver's warm start and smoothness
        # reference at wherever the arm was when it paused. The arm then moves
        # while paused -- homing is exactly that -- so the first solve on resume
        # is pulled one frame toward the OLD configuration before the position
        # cost wins. That is the flicker to the past pose and back.
        self.filter.reset()
        self.limiter.reset()
        self._reseed_from_measured()
        if self.paused:
            # CRITICAL: forget the pose target too.
            #
            # Without this, whatever was last commanded survives the pause. The
            # retargeter stops publishing while disengaged, so on resume this
            # node still holds the PRE-HOMING target and drives straight back to
            # it at full speed before the first new target arrives. That is a
            # sudden, large, unexpected movement with an operator's hand on the
            # controller -- the exact thing the clutch is supposed to prevent.
            self._target = None
        response.success = True
        response.message = 'paused' if self.paused else 'running'
        self.get_logger().info(f'ik_node {response.message}')
        return response

    def _measured(self):
        if not all(j in self._joint_state for j in self.chain.joint_names):
            return None
        return np.array([self._joint_state[j] for j in self.chain.joint_names])

    def _reseed_from_measured(self) -> None:
        self._last_solution_q = None
        q = self._measured()
        if q is not None:
            self.solver.reset(q)
            self.limiter.reset(q)

    # ---- the loop ---------------------------------------------------------

    def _tick(self) -> None:
        if self.paused or self._target is None:
            return
        if self._now() - self._target_time > self.target_timeout:
            # Stale target. Stop commanding rather than driving toward a pose
            # the operator set some time ago; the mode manager's watchdog owns
            # the actual halt.
            return
        if self._measured() is None:
            return
        missing = [j for j in self.mount.joint_names if j not in self._joint_state]
        if missing:
            self.get_logger().warn(
                f'no joint state for mount joints {missing}; not solving',
                throttle_duration_sec=5.0)
            return
        self.solver.set_mount(self.mount.transform(self._joint_state))

        position, rotation = self._target
        # A bounded wall applies only while the TARGET is over its
        # footprint. Judging by the target rather than by the current tool
        # position is what lets the arm leave: judged by where it already
        # is, a tool resting on a table would keep the wall on and could
        # never be commanded past the edge.
        active = [w is None or w.applies_at(position)
                  for w in self._plane_footprints]
        result = self.solver.solve(position, rotation, active_planes=active)
        if not np.all(np.isfinite(result.q)):
            self.get_logger().error('solver returned a non-finite configuration')
            return

        # Refuse an IK branch flip before it reaches the arm.
        #
        # Compared against the PREVIOUS SOLUTION, not against the measured
        # configuration. The arm is rate-limited, so it legitimately trails a
        # target that has moved; measuring against it would flag every large but
        # perfectly smooth repositioning as a flip -- which it did, blocking the
        # arm entirely whenever a constraint demanded real motion.
        #
        # A genuine branch flip is a discontinuity between CONSECUTIVE solves
        # for nearly identical targets. The rate limiter handles the lag case.
        if self._last_solution_q is not None:
            jump = float(np.max(np.abs(result.q - self._last_solution_q)))
            if jump > self.max_joint_jump:
                # Try once more, pulled hard toward the solution we already
                # accepted. A constraint binding or releasing moves the optimum
                # legitimately, and there is usually a nearby answer; only a
                # true branch flip has none.
                retry = self._retry_near(result, position, rotation, active)
                if retry is not None:
                    result = retry
                else:
                    self._refused_jumps += 1
                    self.get_logger().warn(
                        f'refusing a {np.degrees(jump):.0f} deg jump between '
                        f'consecutive solves (limit '
                        f'{np.degrees(self.max_joint_jump):.0f} deg), and no '
                        'nearby solution exists: the solver switched IK '
                        'branch. Holding position.',
                        throttle_duration_sec=2.0)
                    measured = self._measured()
                    if measured is not None:
                        # Re-seed from reality so the next solve does not
                        # simply rediscover the branch we just rejected.
                        self.solver.reset(measured)
                    self._last_solution_q = None
                    return
        self._last_solution_q = result.q.copy()

        q = self.filter(result.q)
        self._rate_limited = self.limiter.would_limit(q, self.period)
        q = self.limiter(q, self.period)
        q = self.chain.clamp(q)

        if self.collision is not None:
            self.collision.offer(q)
            if not self.collision.valid:
                # Hold. Publishing nothing leaves the controller at the last
                # command, which is the last configuration known to be clear.
                self.get_logger().warn(
                    'holding: commanded state is in collision '
                    f'({", ".join(self.collision.contacts[:2]) or "self"})',
                    throttle_duration_sec=2.0)
                return

        self._publish_command(q)
        self._last_result = result
        self._solve_times.append(result.solve_time)
        if len(self._solve_times) > 500:
            self._solve_times = self._solve_times[-500:]

    def _publish_command(self, q) -> None:
        traj = JointTrajectory()
        traj.joint_names = list(self.chain.joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in q]
        point.time_from_start.sec = int(self.command_time)
        point.time_from_start.nanosec = int((self.command_time % 1.0) * 1e9)
        traj.points = [point]
        self.command_pub.publish(traj)

    def _announce_guards(self) -> None:
        """State every guard and its setting once, at startup.

        A guard turned off in config is otherwise invisible: the stack comes up
        looking identical, and the first sign is the behaviour it existed to
        prevent. Anything disabled is logged at WARN so it stands out in a
        terminal the operator glances at before putting a headset on.
        """
        self.get_logger().info(
            'guards: joint limits from the model, jump limit '
            f'{np.degrees(self.max_joint_jump):.0f} deg '
            f'(retry x{self.jump_retry_smoothness:g}), collision policy '
            f'{self.policy!r}, {len(self.solver.planes)} virtual wall(s)')

        off = []
        if self.policy == 'none':
            off.append('collision checking is OFF (collision_policy: none)')
        if self.max_joint_jump <= 0.0:
            off.append('the jump guard is OFF (max_joint_jump: 0)')
        if self.jump_retry_smoothness <= 0.0:
            off.append('jump retry is OFF; a refused flip will hold the arm')
        scale = float(self.get_parameter('rate_limit_scale').value)
        if scale >= 1.0:
            off.append(f'rate_limit_scale is {scale:g}: the arm may be driven '
                       'at the full limit the model declares')
        for line in off:
            self.get_logger().warn(f'GUARD DISABLED: {line}')

    def _retry_near(self, rejected, position, rotation, active):
        """Re-solve pulled toward the last accepted solution.

        Returns the retry when it lands inside the jump limit, else None. The
        seed is the last ACCEPTED solution, not the rejected one, so the
        stronger smoothness term pulls away from the flip rather than toward
        it.
        """
        if self.jump_retry_smoothness <= 0.0:
            return None
        result = self.solver.solve(
            position, rotation, q_seed=self._last_solution_q,
            active_planes=active,
            smoothness=self.solver.weights.smoothness * self.jump_retry_smoothness)
        jump = float(np.max(np.abs(result.q - self._last_solution_q)))
        if jump > self.max_joint_jump:
            return None
        self._retried_jumps += 1
        self.get_logger().info(
            f'a {np.degrees(jump):.0f} deg solution replaced a rejected branch '
            'flip; holding the arm was not necessary',
            throttle_duration_sec=5.0)
        return result

    def _publish_status(self) -> None:
        msg = IKStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.active = not self.paused and self._target is not None
        msg.paused = self.paused
        msg.collision_blocked = bool(self.collision is not None
                                     and not self.collision.valid)
        msg.rate_limited = bool(self._rate_limited)
        msg.refused_jumps = self._refused_jumps
        msg.retried_jumps = self._retried_jumps

        if self._last_result is not None:
            r = self._last_result
            msg.solve_time = float(r.solve_time)
            msg.position_error = float(r.position_error)
            msg.orientation_error = float(r.orientation_error)
            msg.solver_status = r.status
            msg.limited_joints = list(r.limited_joints)
        if self._solve_times:
            msg.solve_time_p99 = float(np.percentile(self._solve_times, 99))
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = IKNode()
    except Exception as exc:  # noqa: BLE001
        rclpy.logging.get_logger('ik_node').fatal(str(exc))
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
