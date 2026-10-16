"""Publish simple collision geometry into the planning scene.

Gives the collision policy something to act on, and gives MoveIt's planner the
same picture. The obvious first object is a floor: without one, nothing stops a
teleoperated arm being driven into the table it is bolted to.

This is the counterpart to the IK solver's virtual walls, and the two behave
differently on purpose:

  * a **virtual wall** is a constraint inside the solve. Exact, no lag, and the
    tool slides along it.
  * a **collision object** is geometry the checker sees. It stops motion rather
    than redirecting it, and it also blocks the ARM, not just the tool point.

Use both: the wall for the surface you are working against, objects for
everything the arm as a whole must not hit. Both are declared in ONE file
(see scene_config), and a box's wall is derived from that box's bounds, so
the surface the tool slides on is the surface the arm is stopped at.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool

from .robot_info import RobotInfo, fetch_descriptions
from .scene_config import SceneConfig


class SceneObjectsNode(Node):

    def __init__(self):
        super().__init__('scene_objects')

        self.declare_parameter('description_source', '/move_group')
        # The whole robot-specific surface of this node. Everything about WHAT
        # geometry exists lives in that file, so adding an obstacle never means
        # editing code or a launch file.
        self.declare_parameter('scene_config', '')

        path = self.get_parameter('scene_config').value
        if not path:
            raise RuntimeError(
                'scene_config is required: this node is configured by a '
                'planning-scene YAML, not by node parameters')
        scene = SceneConfig.from_file(path)

        source = self.get_parameter('description_source').value
        urdf, srdf = fetch_descriptions(self, source)
        robot = RobotInfo.from_xml(urdf, srdf)
        self._planning_frame = robot.model_frame

        # Latched, because the one consumer that matters -- the mode manager,
        # which must not plan a homing path against a world with no table in
        # it -- may well subscribe after this node has finished its single
        # publish. TRANSIENT_LOCAL means a late subscriber still gets it.
        self._applied_pub = self.create_publisher(
            Bool, '~/applied',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self._client = self.create_client(ApplyPlanningScene,
                                          '/apply_planning_scene')
        if not self._client.wait_for_service(timeout_sec=20.0):
            raise RuntimeError(
                'no /apply_planning_scene service; is move_group running?')

        objects = [self._box(b) for b in scene.collision_boxes()]
        if objects:
            self._apply(objects)
            for plane in scene.all_planes():
                self.get_logger().info(
                    f"wall '{plane.name}': {plane.normal.round(3).tolist()} . p "
                    f'>= {plane.offset:.4f} in {plane.frame!r}')
        else:
            self.get_logger().info(
                f'{path} declares no collision objects')
        # Announced only after the scene is in, never before, and never if
        # _apply raised: this flag is what releases planned homing.
        self._applied_pub.publish(Bool(data=True))

    def _box(self, box) -> CollisionObject:
        """One axis-aligned box, placed by its bounds.

        The frame is whatever the config named -- a robot link is fine, and is
        the useful case: geometry declared against base_link stays put when the
        robot is re-mounted, where geometry in the planning frame does not.
        """
        obj = CollisionObject()
        obj.header.frame_id = box.frame or self._planning_frame
        obj.id = box.name
        obj.operation = CollisionObject.ADD

        solid = SolidPrimitive()
        solid.type = SolidPrimitive.BOX
        solid.dimensions = [float(v) for v in box.size]
        obj.primitives = [solid]

        # The offset goes on the OBJECT pose, with the primitive at the object
        # origin. Splitting it across both is legal and makes every consumer
        # add two poses to find the box, which is easy to get wrong.
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (
            float(v) for v in box.center)
        pose.orientation.w = 1.0
        obj.pose = pose
        obj.primitive_poses = [Pose(orientation=pose.orientation)]

        top = box.center[2] + box.size[2] / 2
        self.get_logger().info(
            f"box '{box.name}': {box.size[0]:.3f}x{box.size[1]:.3f}x"
            f"{box.size[2]:.3f} m in '{obj.header.frame_id}', "
            f'top face at z={top:.4f}'
            + (f", wall on {box.wall}" if box.wall else ''))
        return obj

    def _apply(self, objects) -> None:
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = objects

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError('move_group refused the planning scene update')
        self.get_logger().info(
            f'applied {len(objects)} collision object(s) to the planning scene')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SceneObjectsNode()
    except Exception as exc:  # noqa: BLE001
        rclpy.logging.get_logger('scene_objects').fatal(str(exc))
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
