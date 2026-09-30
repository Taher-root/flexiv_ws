"""Unit tests for waist commanding (no rclpy, no hardware).

arm_driver_node imports rclpy, so _expand_arm_to_rdk is lifted out by AST and
run against a stub session. The property that matters most is the first two
cases: with waist_joint_names empty the function must behave exactly as it did
before the waist work, because that is what every existing launch relies on.

Measured q is the real Rizon4-063352 reading from 2026-09-30.
"""
import ast, textwrap
from pathlib import Path
SRC = str(Path(__file__).resolve().parents[1]
          / "aico2_left_arm_driver" / "arm_driver_node.py")
text = open(SRC).read()
cls = next(n for n in ast.parse(text).body
           if isinstance(n, ast.ClassDef) and n.name == "ArmDriverNode")
fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef)
          and n.name == "_expand_arm_to_rdk")
ns = {"List": list, "Tuple": tuple}
exec(textwrap.dedent(ast.get_source_segment(text, fn)), ns)
expand = ns["_expand_arm_to_rdk"]

MEAS_Q  = [0.09, 5.85, -15.97, -55.98, 37.68, 30.62, 80.95, 57.59, -121.87]
MEAS_DQ = [0.0] * 9

class Session:
    def states(self):
        return type("S", (), {"q": list(MEAS_Q), "dq": list(MEAS_DQ)})()

class Node:
    def __init__(self, waist):
        self._session = Session(); self._rdk_dof = 9
        self._arm_dof = 7; self._dof = 7; self._ext_dof = 2
        self._waist_joint_names = waist
    def _rdk_arm_start_index(self): return 2


ARM = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
WAIST = ["AGV_Joint1", "AGV_Joint2"]


def test_waist_off_is_the_old_behaviour():
    q, dq = expand(Node([]), ARM, [0.0] * 7)
    assert q[:2] == MEAS_Q[:2]        # waist holds its measured position
    assert q[2:] == ARM
    assert len(q) == 9 and len(dq) == 9


def test_waist_on_commands_the_external_axes():
    q, _ = expand(Node(WAIST), ARM + [0.5, 0.7], [0.0] * 9)
    assert q[:2] == [0.5, 0.7]
    assert q[2:] == ARM
    assert len(q) == 9


def test_arm_only_vector_while_waist_enabled_holds_the_waist():
    # A client that sends only the 7 arm joints must not move the torso.
    q, _ = expand(Node(WAIST), ARM, [0.0] * 7)
    assert q[:2] == MEAS_Q[:2]


def test_never_writes_past_the_external_axes():
    q, _ = expand(Node(WAIST + ["BOGUS"]), ARM + [0.5, 0.7, 9.9], [0.0] * 10)
    assert q[:2] == [0.5, 0.7]
    assert 9.9 not in q
    assert len(q) == 9


# ---------------------------------------------------------------------------
# _cmd_names_for: the waist must be OPTIONAL in an incoming trajectory.
# Requiring it unconditionally rejected every 7-joint client (MoveIt's
# left_arm group, check_arm_ros.py, measure_tracking.py) as soon as
# control_waist was enabled -- reorder_to_driver raises on a missing joint.
# ---------------------------------------------------------------------------
import ast as _ast, textwrap as _tw

_text = open(SRC).read()
_cls = next(n for n in _ast.parse(_text).body
            if isinstance(n, _ast.ClassDef) and n.name == "ArmDriverNode")
_fn = next(n for n in _cls.body if isinstance(n, _ast.FunctionDef)
           and n.name == "_cmd_names_for")
_ns = {"List": list}
exec(_tw.dedent(_ast.get_source_segment(_text, _fn)), _ns)
cmd_names_for = _ns["_cmd_names_for"]

ARM_NAMES = [f"Left_joint{i}" for i in range(1, 8)]


class _Traj:
    def __init__(self, names):
        self.joint_names = names


def _node(waist):
    n = Node(waist)
    n._joint_names = ARM_NAMES
    return n


def test_moveit_left_arm_plan_is_accepted_while_waist_is_enabled():
    # 7-joint trajectory, waist owned: command the arm, leave the waist.
    assert cmd_names_for(_node(WAIST), _Traj(ARM_NAMES)) == ARM_NAMES


def test_nine_joint_goal_commands_the_waist_too():
    assert cmd_names_for(_node(WAIST), _Traj(ARM_NAMES + WAIST)) == \
        ARM_NAMES + WAIST


def test_one_waist_joint_is_enough():
    assert cmd_names_for(_node(WAIST), _Traj(ARM_NAMES + ["AGV_Joint2"])) == \
        ARM_NAMES + ["AGV_Joint2"]


def test_waist_ignored_when_the_driver_does_not_own_it():
    assert cmd_names_for(_node([]), _Traj(ARM_NAMES + WAIST)) == ARM_NAMES
