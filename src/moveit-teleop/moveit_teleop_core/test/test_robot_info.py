"""Reading the robot model off a running move_group.

The retry here is not defensive padding. Every node in the stack asks
move_group for the URDF and the SRDF within the same second of launch, and
these nodes fail fast: one unanswered request used to kill the solver for the
whole run, leaving a stack whose gripper worked and whose arm could not move.
"""

import sys
import types

import pytest

from moveit_teleop_core.robot_info import (
    RobotInfoError, SRDF_PARAM, URDF_PARAM, fetch_descriptions,
)


class _Value:
    def __init__(self, text):
        self.string_value = text


class _Result:
    def __init__(self, *texts):
        self.values = [_Value(t) for t in texts]


class _Future:
    def __init__(self, result):
        self._result = result
        self.cancelled = False

    def result(self):
        return self._result

    def cancel(self):
        self.cancelled = True


class _Client:
    """Answers with ``answers[i]`` on the i-th call; None means 'no reply'."""

    def __init__(self, answers):
        self.answers = list(answers)
        self.requests = []
        self.futures = []
        self.dropped = []

    def wait_for_service(self, timeout_sec=None):
        return True

    def call_async(self, request):
        self.requests.append(request)
        future = _Future(self.answers.pop(0) if self.answers else None)
        self.futures.append(future)
        return future

    def remove_pending_request(self, future):
        self.dropped.append(future)


class _Logger:
    def __init__(self):
        self.warnings = []

    def warn(self, message, **_):
        self.warnings.append(message)


class _Node:
    def __init__(self, client):
        self.client = client
        self.logger = _Logger()
        self.service_names = []

    def create_client(self, _type, name):
        self.service_names.append(name)
        return self.client

    def get_logger(self):
        return self.logger


@pytest.fixture(autouse=True)
def _no_spin(monkeypatch):
    """The stub futures are already resolved, so spinning is a no-op."""
    import rclpy
    monkeypatch.setattr(rclpy, 'spin_until_future_complete',
                        lambda *a, **k: None, raising=False)
    # fetch_descriptions imports rcl_interfaces lazily; keep that import cheap
    # and independent of a running ROS graph.
    if 'rcl_interfaces.srv' not in sys.modules:
        module = types.ModuleType('rcl_interfaces.srv')

        class GetParameters:
            class Request:
                def __init__(self):
                    self.names = []

        module.GetParameters = GetParameters
        sys.modules['rcl_interfaces.srv'] = module


def test_the_descriptions_come_back_on_the_first_answer():
    client = _Client([_Result('<robot/>', '<srdf/>')])
    node = _Node(client)
    assert fetch_descriptions(node, '/move_group') == ('<robot/>', '<srdf/>')
    assert node.service_names == ['/move_group/get_parameters']
    assert client.requests[0].names == [URDF_PARAM, SRDF_PARAM]
    assert node.logger.warnings == []


def test_an_unanswered_request_is_retried_not_fatal():
    # The real failure: move_group is up and the service exists, but the reply
    # does not arrive inside the timeout because it is serving the rest of the
    # stack the same two large strings.
    client = _Client([None, _Result('<robot/>', '<srdf/>')])
    node = _Node(client)
    assert fetch_descriptions(node, '/move_group') == ('<robot/>', '<srdf/>')
    assert len(client.requests) == 2
    assert len(node.logger.warnings) == 1
    assert 'retrying' in node.logger.warnings[0]


def test_a_stale_request_is_dropped_before_retrying():
    # A late reply must not complete a future nobody reads any more.
    client = _Client([None, _Result('<robot/>', '<srdf/>')])
    fetch_descriptions(_Node(client), '/move_group')
    assert client.futures[0].cancelled
    assert client.dropped == [client.futures[0]]


def test_giving_up_says_the_service_was_there_and_what_to_do():
    client = _Client([None, None, None])
    with pytest.raises(RobotInfoError) as exc:
        fetch_descriptions(_Node(client), '/move_group', timeout_sec=10.0)
    assert len(client.requests) == 3
    assert 'went unanswered in 30s' in str(exc.value)
    assert 'Relaunch' in str(exc.value)


def test_a_missing_service_is_a_different_error():
    class _Absent(_Client):
        def wait_for_service(self, timeout_sec=None):
            return False

    with pytest.raises(RobotInfoError, match='no parameter service'):
        fetch_descriptions(_Node(_Absent([])), '/move_group')


def test_an_empty_description_is_refused():
    client = _Client([_Result('', '<srdf/>')])
    with pytest.raises(RobotInfoError, match=URDF_PARAM):
        fetch_descriptions(_Node(client), '/move_group')
    assert SRDF_PARAM  # both names appear in the message
