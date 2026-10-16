"""Clutched relative-pose retargeting: the default strategy.

The operator's device is a 6-DoF mouse, not an absolute pointer. On engage we
anchor both the device pose and the current end-effector pose; while engaged we
apply the device's motion *since that anchor* to the anchored EE pose. On
disengage the operator can reposition their hand freely, and the next engage
re-anchors, which is what makes the workspace ratchetable.

The critical detail is where the anchor comes from on re-engage: the robot's
*actual* current EE pose, read from TF, not the last commanded target. Anchoring
to the last target would accumulate the tracking error between what we asked for
and what servo achieved, and each engage/disengage cycle would push the command
further from reality until the arm lurched.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from ..scene_config import project, violations
from ..transforms import (
    heading_rotation,
    PoseSmoother,
    convention_matrix,
    make_transform,
    scale_rotation,
    split_transform,
)
from .base import RetargetInput, RetargetOutput, Retargeter


class RelativePoseRetargeter(Retargeter):
    """Accumulates device motion since clutch engage onto the anchored EE pose.

    Config keys (all optional except where noted):

    ``translation_scale``   device metres -> robot metres (default 0.5)
    ``rotation_scale``      device radians -> robot radians (default 1.0)
    ``translation_ref``     ``base`` or ``tool`` (default ``base``)
    ``rotation_ref``        ``base`` or ``tool`` (default ``tool``)
    ``device_convention``   name or 3x3 mapping device axes to ROS (default ``ros``)
    ``smoothing.alpha``     EMA weight of the new sample (default 1.0 = off)
    ``max_translation_step`` metres per update the target may jump (default 0.05)
    ``external_motion_threshold`` metres of unexplained EE motion that forces a
                            re-anchor (default 0.05)
    """

    def __init__(self, config: dict, robot_info=None):
        super().__init__(config, robot_info)

        self.translation_scale = float(self.config.get('translation_scale', 0.5))
        self.rotation_scale = float(self.config.get('rotation_scale', 1.0))
        self.translation_ref = str(self.config.get('translation_ref', 'base')).lower()
        # Defaults to 'base' (world-referenced). 'tool' was the old default and
        # felt wrong: it composed a body-frame delta as though it were
        # extrinsic, so wrist twist rotated the tool about the wrong axis.
        self.rotation_ref = str(self.config.get('rotation_ref', 'base')).lower()

        for name, value in (('translation_ref', self.translation_ref),
                            ('rotation_ref', self.rotation_ref)):
            if value not in ('base', 'device', 'tool'):
                raise ValueError(
                    f'{name} must be "base", "device" or "tool", '
                    f'got {value!r}')

        self.convention = convention_matrix(
            self.config.get('device_convention', 'ros'))
        # Maps controller-BODY axes onto tool-BODY axes. Only used when a
        # 'tool' reference is selected; world-referenced motion needs no such
        # constant, which is a good reason to prefer it.
        self.alignment = convention_matrix(
            self.config.get('controller_alignment', 'ros'))

        smoothing = self.config.get('smoothing') or {}
        self.smoother = PoseSmoother(float(smoothing.get('alpha', 1.0)))

        # A safety net independent of servo: if a single update would move the
        # target further than this, the device stream is untrustworthy (a
        # teleport, a decoded garbage pose) and we decline rather than fling
        # the arm. Servo would also decelerate, but by then the command exists.
        self.max_translation_step = float(
            self.config.get('max_translation_step', 0.05))

        # If the end effector is somewhere we did not command it to be, then
        # something else moved the robot: a planned motion, a move to the ready
        # posture, a manual jog, or simply our anchor predating a startup
        # motion. Continuing to accumulate against a stale anchor would command
        # a pose far from reality, which servo answers with an emergency stop
        # (or, worse, a lurch). Re-anchoring is always the right response.
        # The leash. The commanded target may never sit further than this from
        # where the end effector ACTUALLY is.
        #
        # Without it, an arm stopped by a joint limit falls behind while the
        # operator keeps moving, so the target runs away: the arm stays put,
        # then snaps when it frees up, and the divergence trips the
        # external-motion guard into re-anchoring every few frames. That is the
        # "anchored / tracking / anchored" flapping.
        #
        # Clamping only the OUTPUT leaves the anchor intact, so the full
        # commanded pose is restored the moment the arm catches up.
        self.max_target_lead = float(self.config.get('max_target_lead', 0.05))
        self.max_target_lead_angle = float(
            self.config.get('max_target_lead_angle', 0.5))

        # How far a wall may move the command BEYOND what the operator's
        # hand drove, in one update, before the mapping is re-zeroed
        # instead. Ordinary motion contributes no excess at all however
        # fast it is, so this can be small. 0 disables it.
        self.max_command_step = float(
            self.config.get('max_command_step', 0.03))

        self.external_motion_threshold = float(
            self.config.get('external_motion_threshold', 0.15))
        if self.external_motion_threshold <= self.max_target_lead:
            # Otherwise normal leashed lag reads as a teleport and the
            # retargeter re-anchors continuously.
            raise ValueError(
                f'external_motion_threshold ({self.external_motion_threshold}) '
                f'must exceed max_target_lead ({self.max_target_lead}), or '
                'ordinary tracking lag is mistaken for external motion')

        # Walls, in the PLANNING frame, as scene_config.Plane objects --
        # which carry a footprint, so a table only constrains the tool
        # while the tool is actually over it.
        # Set by the node once TF can resolve them; empty means no walls.
        self.walls = []
        # Which walls the RAW target was behind on the last update. The
        # operator is pressing into a surface; a UI can beep or buzz.
        self.contacts = []
        # Metres past the deepest wall, so feedback can scale with how
        # hard the operator is pressing rather than being on/off.
        self.contact_depth = 0.0
        self._device_anchor = None   # 4x4, device frame
        self._heading = None         # 3x3, operator heading at engage
        self._ee_anchor = None       # 4x4, planning frame
        self._last_target = None     # 4x4, planning frame
        self._last_composed = None   # 4x4, before wall projection
        self._last_projected = None  # 4x4, after projection, before leash
        self._was_clutched = False

    def reset(self) -> None:
        self._device_anchor = None
        self._last_composed = None
        self._last_projected = None
        self._heading = None
        self._ee_anchor = None
        self._last_target = None
        self._was_clutched = False
        self.smoother.reset()

    # ---- main entry point -------------------------------------------------

    def update(self, sample: RetargetInput) -> RetargetOutput:
        if not sample.pose_valid:
            # Tracking loss must drop the anchor. Holding it would mean that
            # when tracking returns, the device's re-acquired pose is compared
            # against an anchor from before the gap and the arm jumps by
            # however far the operator's hand moved meanwhile.
            self._release()
            return self._inactive(sample, 'tracking lost')

        if not sample.clutch:
            self._release()
            return self._inactive(sample, 'declutched')

        position, orientation = self.smoother(
            sample.device_position, sample.device_orientation)
        device_now = make_transform(position, orientation)

        if self._external_motion(sample):
            # Drop the anchor so the branch below re-takes it from where the
            # robot actually is.
            self._device_anchor = None
            self._was_clutched = False

        if not self._was_clutched or self._device_anchor is None:
            # Rising edge: anchor to the device pose and, crucially, to the
            # robot's ACTUAL current pose.
            self.smoother.reset()
            position, orientation = self.smoother(
                sample.device_position, sample.device_orientation)
            device_now = make_transform(position, orientation)
            return self._reanchor(sample, device_now, 'anchored')

        # Project onto the walls BEFORE leashing, and the order is the whole
        # fix. The leash clamps the target onto a SPHERE around the end
        # effector: a target half a metre under the table becomes a point 0.3 m
        # away in a mostly-downward direction, so sliding your hand sideways
        # barely rotates it (the arm does not slide) and the accumulated
        # distance is covered at full rate the moment the target is reachable
        # again (the arm jumps). Projected first, the target sits on the table
        # directly under the hand -- close to the arm, so the leash never
        # bites, and free to move tangentially.
        composed = self._compose(device_now)

        # The jump guard measures the DEVICE-driven target, before any wall
        # projection. Its job is to catch an untrustworthy stream -- a
        # teleport, a decoded garbage pose -- and projection is neither: moving
        # the target onto a surface can legitimately shift it a long way in one
        # step, and measuring after would reject every update the moment the
        # operator's hand crossed a wall.
        step = np.linalg.norm(composed[:3, 3] - self._last_composed[:3, 3])
        if step > self.max_translation_step:
            return self._inactive(
                sample, f'rejected {step:.3f} m jump (limit '
                        f'{self.max_translation_step:.3f} m)')
        self._last_composed = composed

        projected = self._project(composed)

        # A wall changing under the target moves the COMMAND without the
        # operator's hand having moved: reach past the edge of a table and the
        # projection stops applying, so the command snaps from the table
        # surface to wherever the hand actually is. That is a teleport as far
        # as the arm is concerned, and it is the jump left over after
        # projection fixed the sliding.
        #
        # Re-anchor rather than command it. The arm holds still, the mapping is
        # re-zeroed here, and the operator guides it on from where it is --
        # exactly what releasing and re-taking the clutch would do, without
        # having to notice in time and do it.
        #
        # Rejecting instead (as the device guard above does) would be wrong:
        # that keeps the stale anchor, so the divergence survives and the arm
        # lurches the moment the target becomes commandable again.
        # Measured as the EXCESS over what the hand actually drove, not as the
        # raw command step. A fast hand legitimately moves the command a long
        # way in one update, and thresholding the raw step would re-anchor
        # constantly during ordinary motion. What no hand motion can explain is
        # the command moving FURTHER than the device did -- and only a wall
        # appearing or disappearing under the target does that.
        # Measured BEFORE the leash. The leash releasing as the arm catches up
        # also moves the command further than the hand did, and that is
        # ordinary tracking, not a discontinuity -- checking after it would
        # re-anchor every time the arm closed a gap.
        moved = np.linalg.norm(projected[:3, 3] - self._last_projected[:3, 3])
        excess = moved - step
        if self.max_command_step > 0.0 and excess > self.max_command_step:
            return self._reanchor(
                sample, device_now,
                f're-anchored: a wall moved the command {excess:.3f} m '
                f'further than the hand did (limit '
                f'{self.max_command_step:.3f} m)')

        self._last_projected = projected
        target = self._leash(projected, sample)
        self._last_target = target
        position, orientation = split_transform(target)
        return RetargetOutput(position=position, orientation=orientation,
                              gripper=sample.gripper, active=True, reason='tracking')

    # ---- helpers ----------------------------------------------------------

    def _compose(self, device_now: np.ndarray) -> np.ndarray:
        """Map the controller's motion since engage onto the anchored EE pose.

        Two genuinely different deltas exist and confusing them is the classic
        way to make teleoperation feel wrong:

        **World (extrinsic)** -- how the controller moved *in the room*::

            dp_world = p_now - p_anchor
            dR_world = R_now @ R_anchor.T

        **Body (intrinsic)** -- how it moved *relative to itself*::

            dR_body = R_anchor.T @ R_now
            dp_body = R_anchor.T @ dp_world

        World-referenced is what feels direct, and it is the default for both:
        however you move and turn your hand in the room, the tool does the same
        thing in the room. It also needs no controller-to-tool alignment
        constant, because both sides are expressed in the same shared frame.

        Body-referenced rotation ("tool") does need one: the WebXR grip frame
        and the robot tool frame point different ways, so twisting your wrist
        would otherwise rotate the tool about an unrelated axis. That is what
        ``controller_alignment`` is for.
        """
        anchor = self._ee_anchor
        R_a, p_a = self._device_anchor[:3, :3], self._device_anchor[:3, 3]
        R_n, p_n = device_now[:3, :3], device_now[:3, 3]
        C = self.convention

        dp_world = C @ (p_n - p_a)
        dR_world = C @ (R_n @ R_a.T) @ C.T

        target = np.eye(4)

        # 'device' re-expresses the room-frame delta in the operator's own
        # heading, so forward means "away from me" however I am facing.
        H = self._heading if self._heading is not None else np.eye(3)

        if self.rotation_ref == 'tool':
            A = self.alignment
            dR_body = A @ (R_a.T @ R_n) @ A.T
            target[:3, :3] = anchor[:3, :3] @ scale_rotation(
                dR_body, self.rotation_scale)
        else:
            dR = dR_world if self.rotation_ref == 'base' else H.T @ dR_world @ H
            # Rotate the tool the same way the hand turned -- in the room
            # ('base') or relative to the operator's heading ('device').
            target[:3, :3] = scale_rotation(
                dR, self.rotation_scale) @ anchor[:3, :3]

        if self.translation_ref == 'tool':
            dp_body = self.alignment @ (R_a.T @ (p_n - p_a))
            target[:3, 3] = anchor[:3, 3] + anchor[:3, :3] @ (
                dp_body * self.translation_scale)
        else:
            dp = dp_world if self.translation_ref == 'base' else H.T @ dp_world
            target[:3, 3] = anchor[:3, 3] + dp * self.translation_scale

        return target

    def _reanchor(self, sample, device_now: np.ndarray, reason: str):
        """Re-zero the mapping here, and hold the arm still.

        Used both on the rising edge of the clutch and whenever a wall moves
        the command without the operator having moved. One path for both so the
        two cannot drift apart: whatever "engaging" means, this is it.
        """
        self._device_anchor = device_now
        # Which way the operator is facing, frozen for the duration of this
        # clutch. It MUST be frozen: recomputing it every sample would let the
        # mapping rotate under the operator while they are mid-motion, so
        # pushing in a straight line would curve.
        self._heading = heading_rotation(
            self.convention @ device_now[:3, :3] @ self.convention.T)
        self._ee_anchor = make_transform(sample.ee_position,
                                         sample.ee_orientation)
        self._last_target = self._ee_anchor
        self._last_composed = self._ee_anchor
        self._last_projected = self._ee_anchor
        self._was_clutched = True
        # Commanding the CURRENT pose, so the arm does not move on this update.
        return RetargetOutput(
            position=np.asarray(sample.ee_position, float),
            orientation=np.asarray(sample.ee_orientation, float),
            gripper=sample.gripper, active=True, reason=reason)

    def _project(self, target: np.ndarray) -> np.ndarray:
        """Slide the target along any wall it has gone behind."""
        if not self.walls:
            self.contacts, self.contact_depth = [], 0.0
            return target
        behind = violations(target[:3, 3], self.walls, tolerance=1e-6)
        self.contacts = [self.walls[i].name for i, _ in behind]
        self.contact_depth = behind[0][1] if behind else 0.0
        if not behind:
            return target
        target = target.copy()
        target[:3, 3] = project(target[:3, 3], self.walls)
        return target

    def _leash(self, target: np.ndarray, sample: RetargetInput) -> np.ndarray:
        """Hold the target within reach of where the arm actually is."""
        ee_p = np.asarray(sample.ee_position, float)
        lead = target[:3, 3] - ee_p
        distance = np.linalg.norm(lead)
        if distance > self.max_target_lead:
            target = target.copy()
            target[:3, 3] = ee_p + lead * (self.max_target_lead / distance)

        # Same idea for orientation: a target the wrist cannot reach yet must
        # not keep winding further away from it.
        ee_R = Rotation.from_quat(np.asarray(sample.ee_orientation, float))
        rel = (Rotation.from_matrix(target[:3, :3]) * ee_R.inv()).as_rotvec()
        angle = np.linalg.norm(rel)
        if angle > self.max_target_lead_angle:
            clamped = Rotation.from_rotvec(rel * (self.max_target_lead_angle / angle))
            target = target.copy()
            target[:3, :3] = (clamped * ee_R).as_matrix()
        return target

    def _external_motion(self, sample: RetargetInput) -> bool:
        """Has the EE moved somewhere we did not command?"""
        if self._last_target is None or not self._was_clutched:
            return False
        drift = np.linalg.norm(
            np.asarray(sample.ee_position, float) - self._last_target[:3, 3])
        return bool(drift > self.external_motion_threshold)

    def _release(self) -> None:
        """Drop the anchor but remember nothing else; the next engage re-anchors."""
        if self._was_clutched:
            self._was_clutched = False
            self._device_anchor = None
            self.smoother.reset()

    def _inactive(self, sample: RetargetInput, reason: str) -> RetargetOutput:
        return RetargetOutput(
            position=np.asarray(sample.ee_position, float),
            orientation=np.asarray(sample.ee_orientation, float),
            gripper=sample.gripper, active=False, reason=reason)
