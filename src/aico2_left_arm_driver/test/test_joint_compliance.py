"""Unit tests for the impedance-mode compliance knobs (no rclpy, no hardware).

arm_driver_node imports rclpy, which is not importable in a plain pytest run,
so _apply_joint_stiffness is lifted out of the source by AST and executed
against a stub self. That runs the real code text rather than a paraphrase of
it, which is the whole point: this method is what decides the numbers the
controller receives, and getting them wrong faults the arm.
"""

import ast
import textwrap
from pathlib import Path

import pytest

_SRC = (Path(__file__).resolve().parents[1]
        / "aico2_left_arm_driver" / "arm_driver_node.py")

# Rizon 4 as mounted on AICO2: 2 waist axes then 7 arm axes. The waist is
# positionally rigid (+inf nominal stiffness) and its entries must pass
# through untouched; the arm torque limits are not uniform.
K_NOM = [float("inf"), float("inf"), 6000.0, 6000.0, 4200.0, 4200.0,
         1500.0, 1500.0, 1500.0]
TAU_MAX = [123.0, 123.0, 123.0, 123.0, 64.0, 64.0, 39.0, 39.0, 39.0]


def _load_apply_stiffness():
    text = _SRC.read_text()
    tree = ast.parse(text)
    cls = next(n for n in tree.body
               if isinstance(n, ast.ClassDef) and n.name == "ArmDriverNode")
    fn = next(n for n in cls.body
              if isinstance(n, ast.FunctionDef)
              and n.name == "_apply_joint_stiffness")
    ns = {"mode_to_str": lambda mode: mode}
    exec(textwrap.dedent(ast.get_source_segment(text, fn)), ns)
    return ns["_apply_joint_stiffness"]


_apply_stiffness = _load_apply_stiffness()


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(("info", msg))

    def warn(self, msg):
        self.lines.append(("warn", msg))

    def error(self, msg):
        self.lines.append(("error", msg))


class _Session:
    """Records what would reach the controller."""

    def __init__(self, mode="NRT_JOINT_IMPEDANCE"):
        self.K_q = None
        self.tau = None
        self._mode = mode
        self.robot = self

    def mode(self):
        return self._mode

    def joint_stiffness_nominal(self):
        return list(K_NOM)

    def joint_torque_max(self):
        return list(TAU_MAX)

    def set_joint_impedance(self, K_q):
        self.K_q = list(K_q)

    def set_max_contact_torque(self, tau):
        self.tau = list(tau)


class _Node:
    def __init__(self, ratio, torque, mode="impedance",
                 rdk_mode="NRT_JOINT_IMPEDANCE"):
        self._mock = False
        self._session = _Session(rdk_mode)
        self._joint_control_mode = mode
        self._arm_dof = 7
        self._joint_stiffness_ratio = ratio
        self._max_contact_torque = torque
        self._log = _Logger()

    def get_logger(self):
        return self._log


def _run(ratio, torque, **kwargs):
    node = _Node(ratio, torque, **kwargs)
    _apply_stiffness(node)
    assert all(level != "error" for level, _ in node._log.lines), node._log.lines
    return node


def _log_text(node):
    return " ".join(msg for _, msg in node._log.lines)


def test_stiffness_scales_arm_axes_only():
    node = _run(0.15, 10.0)
    assert node._session.K_q[:2] == [float("inf")] * 2
    assert node._session.K_q[2:] == [k * 0.15 for k in K_NOM[2:]]


def test_contact_torque_bounds_arm_axes_and_leaves_the_waist():
    node = _run(0.15, 10.0)
    # The waist keeps its own limit: bounding its contact torque would only let
    # the column sag under the arms.
    assert node._session.tau == TAU_MAX[:2] + [10.0] * 7
    assert "max contact torque 10 Nm" in _log_text(node)
    assert "clamped" not in _log_text(node)


def test_ceiling_above_tau_max_is_clamped_per_axis_not_rejected():
    # SetMaxContactTorque validates each entry against [0, tau_max], so a
    # uniform request has to be clamped or the call raises and nothing lands.
    node = _run(0.15, 100.0)
    assert node._session.tau == [123.0, 123.0, 100.0, 100.0,
                                 64.0, 64.0, 39.0, 39.0, 39.0]
    assert "clamped to tau_max" in _log_text(node)


def test_zero_leaves_the_ceiling_unset_and_says_so():
    node = _run(0.15, 0.0)
    assert node._session.tau is None
    assert "torque UNBOUNDED" in _log_text(node)


def test_position_mode_touches_neither_knob():
    node = _run(0.15, 10.0, mode="position")
    assert node._session.K_q is None
    assert node._session.tau is None


def test_wrong_rdk_mode_is_a_noop():
    # SetJointImpedance raises outside a joint impedance mode, so the method
    # must check before calling rather than relying on the except clause.
    node = _run(0.15, 10.0, rdk_mode="NRT_JOINT_POSITION")
    assert node._session.K_q is None
    assert node._session.tau is None


@pytest.mark.parametrize("ratio", [0.0, 0.15, 1.0])
def test_every_valid_ratio_keeps_stiffness_within_nominal(ratio):
    node = _run(ratio, 10.0)
    for value, nominal in zip(node._session.K_q, K_NOM):
        assert 0.0 <= value <= nominal
