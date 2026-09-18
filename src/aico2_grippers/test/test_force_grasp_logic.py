from aico2_grippers.force_grasp_logic import ForceGraspController, ForceGraspParams


def test_open_on_low_trigger():
    ctrl = ForceGraspController(ForceGraspParams())
    cmd = ctrl.update(0.0, 40.0)
    assert cmd == ("open", 0.0)
    assert ctrl.update(0.0, 40.0) is None


def test_grasp_force_scales_with_trigger():
    ctrl = ForceGraspController(
        ForceGraspParams(grasp_threshold=0.2, max_force_fraction=1.0)
    )
    cmd = ctrl.update(0.5, 40.0)
    assert cmd is not None
    kind, force = cmd
    assert kind == "grasp"
    assert force == -20.0


def test_hysteresis_between_thresholds():
    ctrl = ForceGraspController(ForceGraspParams())
    ctrl.update(0.0, 40.0)
    assert ctrl.update(0.1, 40.0) is None


def test_grasp_debounce_small_force_change():
    ctrl = ForceGraspController(
        ForceGraspParams(
            grasp_threshold=0.2,
            max_force_fraction=1.0,
            min_resend_force_delta=5.0,
        )
    )
    ctrl.update(0.5, 40.0)
    assert ctrl.update(0.51, 40.0) is None
    cmd = ctrl.update(0.8, 40.0)
    assert cmd is not None
    assert cmd[0] == "grasp"
