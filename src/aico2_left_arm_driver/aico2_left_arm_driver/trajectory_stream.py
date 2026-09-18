"""Sample sensor_msgs/JointTrajectory for NRT SendJointPosition streaming."""

from __future__ import annotations

from typing import List, Sequence, Tuple

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def _duration_sec(point: JointTrajectoryPoint) -> float:
    return point.time_from_start.sec + point.time_from_start.nanosec * 1e-9


def trajectory_duration(traj: JointTrajectory) -> float:
    if not traj.points:
        return 0.0
    return _duration_sec(traj.points[-1])


def reorder_to_driver(
    traj: JointTrajectory, driver_joint_names: Sequence[str]
) -> Tuple[List[int], JointTrajectory]:
    """Map trajectory joint order to driver joint order; raises ValueError on mismatch."""
    name_to_idx = {n: i for i, n in enumerate(traj.joint_names)}
    indices = []
    for name in driver_joint_names:
        if name not in name_to_idx:
            raise ValueError(f"trajectory missing joint '{name}'")
        indices.append(name_to_idx[name])

    out = JointTrajectory()
    out.header = traj.header
    out.joint_names = list(driver_joint_names)
    for pt in traj.points:
        new_pt = JointTrajectoryPoint()
        new_pt.time_from_start = pt.time_from_start
        if pt.positions:
            new_pt.positions = [pt.positions[i] for i in indices]
        if pt.velocities:
            new_pt.velocities = [pt.velocities[i] for i in indices]
        if pt.accelerations:
            new_pt.accelerations = [pt.accelerations[i] for i in indices]
        if pt.effort:
            new_pt.effort = [pt.effort[i] for i in indices]
        out.points.append(new_pt)
    return indices, out


def sample_trajectory(
    traj: JointTrajectory, t_sec: float, dof: int
) -> Tuple[List[float], List[float]]:
    """Linear interpolation of positions and velocities at time t."""
    if not traj.points:
        return [0.0] * dof, [0.0] * dof

    if t_sec <= 0.0:
        pt = traj.points[0]
        q = list(pt.positions) if pt.positions else [0.0] * dof
        dq = list(pt.velocities) if pt.velocities else [0.0] * dof
        return q, dq

    prev = traj.points[0]
    if _duration_sec(prev) >= t_sec:
        q = list(prev.positions) if prev.positions else [0.0] * dof
        dq = list(prev.velocities) if prev.velocities else [0.0] * dof
        return q, dq

    for nxt in traj.points[1:]:
        t_next = _duration_sec(nxt)
        t_prev = _duration_sec(prev)
        if t_sec <= t_next:
            if t_next <= t_prev:
                q = list(nxt.positions) if nxt.positions else [0.0] * dof
                dq = list(nxt.velocities) if nxt.velocities else [0.0] * dof
                return q, dq
            alpha = (t_sec - t_prev) / (t_next - t_prev)
            q_prev = list(prev.positions) if prev.positions else [0.0] * dof
            q_next = list(nxt.positions) if nxt.positions else q_prev
            q = [a + alpha * (b - a) for a, b in zip(q_prev, q_next)]
            if prev.velocities and nxt.velocities:
                dq_prev = list(prev.velocities)
                dq_next = list(nxt.velocities)
                dq = [a + alpha * (b - a) for a, b in zip(dq_prev, dq_next)]
            elif nxt.velocities:
                dq = list(nxt.velocities)
            else:
                dq = [0.0] * dof
            return q, dq
        prev = nxt

    last = traj.points[-1]
    q = list(last.positions) if last.positions else [0.0] * dof
    dq = list(last.velocities) if last.velocities else [0.0] * dof
    return q, dq
