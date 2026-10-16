"""Everything robot-specific, read from the artifacts MoveIt already owns.

This module is the genericity claim in code. If a link name, joint name or
frame ever needs to be written into a config file, the reason should be that
it genuinely is not recoverable from here -- not that nobody looked.

What is recoverable:

* the planning frame        -> the URDF root link
* the group's base/tip link -> the SRDF ``<chain>``
* the group's joint names   -> walking the URDF from base to tip
* the end effector link     -> the SRDF ``<end_effector parent_link=...>``
* named postures            -> SRDF ``<group_state>`` (ready_pose, gripper_open, ...)

What is not, and therefore has to stay in config:

* which ros2_control controller actuates the gripper
* the device's own coordinate convention (belongs to the device, not the robot)
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass

# ROS imports are deliberately deferred into fetch_descriptions() so that
# RobotInfo -- which is pure XML parsing -- can be imported and unit-tested
# without a ROS environment. The parsing is where the interesting logic is.

#: Parameters every MoveIt-enabled node carries. Read from a running node
#: rather than re-parsed from xacro so that we are guaranteed to be reasoning
#: about the same robot the planner and servo are using -- a subtle and
#: expensive class of bug when xacro args drift between launch files.
URDF_PARAM = 'robot_description'
SRDF_PARAM = 'robot_description_semantic'


class RobotInfoError(RuntimeError):
    """Raised when the robot model cannot answer a question we need answered."""


def fetch_descriptions(node, source_node: str = '/move_group',
                       timeout_sec: float = 10.0, attempts: int = 3) -> tuple:
    """Return ``(urdf_xml, srdf_xml)`` from a running node's parameters.

    RETRIED, because one unanswered request used to be fatal. The service
    existing does not mean it will answer promptly: the URDF and SRDF are large
    strings, every node in the stack asks for both within the same second of
    launch, and move_group serves them one at a time while also starting up. A
    request that goes unanswered for ``timeout_sec`` is a transient under that
    load, not a broken robot -- but these nodes fail fast, so treating it as
    broken killed the solver for the whole run, and the stack came up with the
    gripper working and the arm silently unable to move.
    """
    import rclpy
    from rcl_interfaces.srv import GetParameters

    client = node.create_client(GetParameters, f'{source_node}/get_parameters')
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RobotInfoError(
            f'no parameter service on {source_node}; is the MoveIt stack up?')

    request = GetParameters.Request()
    request.names = [URDF_PARAM, SRDF_PARAM]

    result = None
    for attempt in range(1, attempts + 1):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        result = future.result()
        if result is not None and len(result.values) == 2:
            break
        # Drop the stale request rather than leaving it pending: a late reply
        # arriving during a later spin would otherwise complete a future
        # nobody is reading any more.
        future.cancel()
        try:
            client.remove_pending_request(future)
        except AttributeError:          # older rclpy
            pass
        result = None
        if attempt < attempts:
            node.get_logger().warn(
                f'{source_node} did not answer a parameter request within '
                f'{timeout_sec:.0f}s (attempt {attempt} of {attempts}); '
                'retrying -- it is probably still serving the rest of the '
                'stack the same descriptions')
    if result is None:
        raise RobotInfoError(
            f'could not read descriptions from {source_node}: '
            f'{attempts} requests went unanswered in '
            f'{attempts * timeout_sec:.0f}s. The service exists, so '
            f'{source_node} is up but not answering. Relaunch the teleop '
            'stack once the robot bringup has settled.')

    urdf, srdf = result.values[0].string_value, result.values[1].string_value
    if not urdf or not srdf:
        raise RobotInfoError(
            f'{source_node} has empty {URDF_PARAM!r} or {SRDF_PARAM!r}')
    return urdf, srdf


@dataclass
class RobotInfo:
    """Parsed view of one robot's URDF and SRDF."""

    urdf: ET.Element
    srdf: ET.Element

    @classmethod
    def from_xml(cls, urdf_xml: str, srdf_xml: str) -> 'RobotInfo':
        return cls(ET.fromstring(urdf_xml), ET.fromstring(srdf_xml))

    # ---- frames ----------------------------------------------------------

    @property
    def model_frame(self) -> str:
        """The URDF root link, which is MoveIt's planning frame.

        Worth being explicit about: on the Piper this is ``world``, not
        ``base_link``, while RViz's default fixed frame is ``base_link``. Pose
        targets stamped with the wrong one of those still look plausible and
        move the arm to the wrong place, so this is always resolved rather
        than assumed.
        """
        children = {j.find('child').get('link')
                    for j in self.urdf.findall('joint')
                    if j.find('child') is not None}
        roots = [link.get('name') for link in self.urdf.findall('link')
                 if link.get('name') not in children]
        if len(roots) != 1:
            raise RobotInfoError(
                f'expected exactly one URDF root link, found {roots}')
        return roots[0]

    # ---- groups ----------------------------------------------------------

    def _group(self, group: str) -> ET.Element:
        for element in self.srdf.findall('group'):
            if element.get('name') == group:
                return element
        available = [g.get('name') for g in self.srdf.findall('group')]
        raise RobotInfoError(
            f'planning group {group!r} not in the SRDF; available: {available}')

    def group_chain(self, group: str) -> tuple:
        """Return ``(base_link, tip_link)`` for a chain-defined group."""
        chain = self._group(group).find('chain')
        if chain is None:
            raise RobotInfoError(
                f'group {group!r} is not defined as a <chain>; cannot infer '
                'its tip link, so ee_link must be given explicitly')
        return chain.get('base_link'), chain.get('tip_link')

    def group_tip_link(self, group: str) -> str:
        return self.group_chain(group)[1]

    def end_effector_link(self, group: str) -> str:
        """Best available answer for "which link do we command".

        Prefers an ``<end_effector>`` whose ``parent_group`` is this group,
        because that is what the SRDF author explicitly designated. Falls back
        to the chain tip. Refuses to guess when several end effectors are
        attached, since picking the wrong one silently commands the wrong link.
        """
        candidates = [e.get('parent_link') for e in self.srdf.findall('end_effector')
                      if e.get('parent_group') == group]
        unique = sorted(set(c for c in candidates if c))
        if len(unique) > 1:
            raise RobotInfoError(
                f'group {group!r} has multiple end effectors on {unique}; '
                'set ee_link explicitly to disambiguate')
        return unique[0] if unique else self.group_tip_link(group)

    def group_joints(self, group: str) -> list:
        """Actuated joint names from the group's base to its tip, in order.

        Walks the URDF rather than trusting declaration order, because the
        order is what a JointTrajectory's ``joint_names`` must agree with.
        """
        base, tip = self.group_chain(group)

        parent_of = {}
        for joint in self.urdf.findall('joint'):
            child = joint.find('child')
            parent = joint.find('parent')
            if child is not None and parent is not None:
                parent_of[child.get('link')] = (
                    joint.get('name'), joint.get('type'), parent.get('link'))

        chain, link = [], tip
        while link != base:
            if link not in parent_of:
                raise RobotInfoError(
                    f'no path from {tip!r} back to {base!r} in the URDF')
            name, jtype, parent = parent_of[link]
            if jtype not in ('fixed',):
                chain.append(name)
            link = parent
        return list(reversed(chain))

    def group_joint_names(self, group: str) -> list:
        """Joint names a group declares directly, for list-defined groups.

        Chain groups (an arm) describe themselves by base and tip links, so
        their joints come from walking the URDF. Groups defined as a list of
        links and joints (a gripper, a hand) name their joints outright, and
        for those :meth:`group_joints` has no chain to walk.
        """
        element = self._group(group)
        names = [j.get('name') for j in element.findall('joint')]
        if names:
            return names
        # Fall back to the chain form so callers need not know which kind of
        # group they were handed.
        return self.group_joints(group)

    def actuated_group_joints(self, group: str) -> list:
        """Group joints excluding fixed and mimic joints.

        Mimic joints matter here: the Piper gripper models two finger joints
        that mimic the driven one. They appear in the SRDF group and in TF, but
        commanding them is an error -- ros2_control does not expose them and
        the controller does not own them.
        """
        mimicked = {j.get('name') for j in self.urdf.findall('joint')
                    if j.find('mimic') is not None}
        types = {j.get('name'): j.get('type') for j in self.urdf.findall('joint')}
        return [j for j in self.group_joint_names(group)
                if j not in mimicked and types.get(j) != 'fixed']

    # ---- named states ----------------------------------------------------

    def named_state(self, group: str, name: str) -> dict:
        """Return ``{joint: value}`` for an SRDF ``<group_state>``."""
        for state in self.srdf.findall('group_state'):
            if state.get('name') == name and state.get('group') == group:
                return {j.get('name'): float(j.get('value'))
                        for j in state.findall('joint')}
        available = sorted(s.get('name') for s in self.srdf.findall('group_state')
                           if s.get('group') == group)
        raise RobotInfoError(
            f'no group_state {name!r} for group {group!r}; available: {available}')

    def named_states(self, group: str) -> list:
        return sorted(s.get('name') for s in self.srdf.findall('group_state')
                      if s.get('group') == group)

    def parent_joint_of(self, link: str) -> str:
        """Name of the joint whose child is ``link``.

        Looked up rather than guessed from the link name: the joint attaching
        ``tcp_link`` is called ``tcp_joint``, not ``tcp_link_joint``, and a
        naming assumption here fails silently.
        """
        for element in self.urdf.findall('joint'):
            child = element.find('child')
            if child is not None and child.get('link') == link:
                return element.get('name')
        raise RobotInfoError(f'no joint has {link!r} as its child')

    def joint_origin(self, joint: str) -> tuple:
        """Return a joint's ``(xyz, rpy)`` origin as floats.

        Used to compare two models that share every joint NAME but place them
        differently -- a tcp_offset mismatch being the case that matters here,
        since it moves the control point without changing the joint set.
        """
        for element in self.urdf.findall('joint'):
            if element.get('name') != joint:
                continue
            origin = element.find('origin')
            if origin is None:
                return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            xyz = tuple(float(v) for v in (origin.get('xyz') or '0 0 0').split())
            rpy = tuple(float(v) for v in (origin.get('rpy') or '0 0 0').split())
            return (xyz, rpy)
        raise RobotInfoError(f'joint {joint!r} not in the URDF')

    def joint_limits(self, joint: str) -> tuple:
        """Return ``(lower, upper)`` from the URDF, or ``(None, None)``."""
        for element in self.urdf.findall('joint'):
            if element.get('name') == joint:
                limit = element.find('limit')
                if limit is None:
                    return (None, None)
                lower, upper = limit.get('lower'), limit.get('upper')
                return (float(lower) if lower is not None else None,
                        float(upper) if upper is not None else None)
        raise RobotInfoError(f'joint {joint!r} not in the URDF')
