"""The config file must actually decide what runs.

These tests exist because the previous arrangement looked correct and did
nothing: piper.yaml was well-formed, well-commented, and read by no code.
"""

import ast
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'launch'))

from _teleop_config import (  # noqa: E402
    CONFIG_KEYS, FALLBACKS, FILE_FALLBACKS, FILE_KEYS, PACKAGE_CONFIG, UNSET,
    TeleopConfigError, load_config, resolve_config_path,
)

SAMPLES = Path(__file__).resolve().parents[1] / 'config' / 'samples'
# The samples are reference copies of the two live configs, which live in the
# ROBOTS' packages. Testing against them keeps this suite runnable without a
# built workspace while still covering a real, complete file.
PACKAGED_CONFIG = SAMPLES / 'piper_teleop.yaml'


def write(tmp_path, mapping):
    path = tmp_path / 'robot.yaml'
    path.write_text(yaml.safe_dump({'moveit_teleop': mapping}))
    return str(path)


def test_every_config_key_has_a_fallback():
    # Otherwise a config file omitting one key raises KeyError at launch.
    assert set(CONFIG_KEYS) == set(FALLBACKS)
    assert set(FILE_KEYS) == set(FILE_FALLBACKS)


def test_config_and_file_keys_do_not_overlap():
    # They are resolved differently -- file keys are made relative to the
    # config file -- so a name in both would silently take one path only.
    assert not set(CONFIG_KEYS) & set(FILE_KEYS)


def test_values_come_from_the_file(tmp_path):
    path = write(tmp_path, {'move_group': 'left_arm',
                            'retargeting': {'rotation_scale': 0.25}})
    resolved = load_config(path, 'ik')
    assert resolved['move_group'] == 'left_arm'
    assert resolved['rotation_scale'] == '0.25'


def test_per_solver_overrides_the_common_value(tmp_path):
    path = write(tmp_path, {'retargeting': {
        'translation_scale': 0.5,
        'per_solver': {'ik': {'translation_scale': 1.5}},
    }})
    assert load_config(path, 'ik')['translation_scale'] == '1.5'
    assert load_config(path, 'servo')['translation_scale'] == '0.5'


def test_absent_keys_are_simply_not_returned(tmp_path):
    resolved = load_config(write(tmp_path, {'move_group': 'arm'}), 'ik')
    assert 'rotation_scale' not in resolved   # resolve() applies the fallback


def test_booleans_do_not_become_python_capitalised(tmp_path):
    # 'True' is not a valid launch boolean; IfCondition rejects it.
    path = write(tmp_path, {'safety': {'hardware_gate': {'enabled': True}}})
    assert load_config(path, 'ik')['hardware_gate_enabled'] == 'true'


def test_null_becomes_the_empty_sentinel(tmp_path):
    # A null must reach launch as '', not as the string 'None'.
    path = write(tmp_path, {'gripper': {'group': None}})
    assert load_config(path, 'ik')['gripper_group'] == ''


def test_missing_file_is_an_error_not_a_silent_default():
    with pytest.raises(TeleopConfigError, match='not found'):
        load_config('/nonexistent/robot.yaml', 'ik')


def test_file_without_the_root_key_is_rejected(tmp_path):
    path = tmp_path / 'robot.yaml'
    path.write_text(yaml.safe_dump({'something_else': {}}))
    with pytest.raises(TeleopConfigError, match='moveit_teleop'):
        load_config(str(path), 'ik')


def test_unknown_per_solver_key_is_rejected(tmp_path):
    # A typo here would otherwise be silently ignored, which is the whole
    # class of bug this change removes.
    path = write(tmp_path, {'retargeting': {
        'per_solver': {'ik': {'translation_scail': 1.5}}}})
    with pytest.raises(TeleopConfigError, match='not a known setting'):
        load_config(path, 'ik')


def test_packaged_config_supplies_every_key_the_launch_needs():
    for solver in ('servo', 'ik'):
        resolved = load_config(str(PACKAGED_CONFIG), solver)
        missing = set(CONFIG_KEYS) - set(resolved)
        assert not missing, f'{solver}: the sample omits {sorted(missing)}'


def test_packaged_config_keeps_the_retargeter_invariant():
    # RelativePoseRetargeter raises unless external_motion_threshold exceeds
    # max_target_lead. A config file that violates it fails at RUNTIME, well
    # after launch appears to have succeeded.
    for solver in ('servo', 'ik'):
        resolved = load_config(str(PACKAGED_CONFIG), solver)
        assert (float(resolved['external_motion_threshold'])
                > float(resolved['max_target_lead'])), solver


def test_sentinel_is_not_a_plausible_real_value():
    assert UNSET not in FALLBACKS.values()


# ---- the replay panel follows the recorder --------------------------------
# recording.yaml owns output_dir; the replay player had its own episodes_dir.
# Two settings for one directory means changing the recorder's leaves the panel
# listing the old location: every episode you record is invisible, the stale
# ones still show, and nothing errors because both halves are individually
# fine. Found by recording an episode that went to Downloads while the panel
# listed ~/teleop_episodes.

from _teleop_config import _resolve_episodes_dir  # noqa: E402


def recording_yaml(tmp_path, output_dir):
    path = tmp_path / 'recording.yaml'
    path.write_text(yaml.safe_dump(
        {'recording': {'output_dir': output_dir,
                       'streams': [{'topic': '/js', 'role': 'observation'}]}}))
    return str(path)


def test_unset_follows_the_recording_config(tmp_path):
    cfg = {'episodes_dir': UNSET,
           'recording_config': recording_yaml(tmp_path, '/data/episodes')}
    _resolve_episodes_dir(cfg)
    assert cfg['episodes_dir'] == '/data/episodes'


def test_an_explicit_directory_still_wins(tmp_path):
    cfg = {'episodes_dir': '/archive/run7',
           'recording_config': recording_yaml(tmp_path, '/data/episodes')}
    _resolve_episodes_dir(cfg)
    assert cfg['episodes_dir'] == '/archive/run7'


def test_falls_back_when_there_is_no_recording_config():
    cfg = {'episodes_dir': UNSET, 'recording_config': ''}
    _resolve_episodes_dir(cfg)
    assert cfg['episodes_dir'] == '~/teleop_episodes'


def test_a_missing_recording_file_does_not_raise():
    cfg = {'episodes_dir': UNSET, 'recording_config': '/nope/recording.yaml'}
    _resolve_episodes_dir(cfg)
    assert cfg['episodes_dir'] == '~/teleop_episodes'


# ---- every config key must be a launch argument ---------------------------
# jump_retry_smoothness was added to CONFIG_KEYS, to piper.yaml and to
# ik.launch.py, but not declared in teleop.launch.py -- which then referenced
# it. Launch aborts with "launch configuration ... does not exist" before a
# single node starts, for EVERY robot, and no existing test noticed: the
# config half was complete and self-consistent, and nothing compared it
# against what the launch file actually declares.

def declared_launch_arguments(name):
    """Names passed to DeclareLaunchArgument, read statically."""
    tree = ast.parse((Path(__file__).resolve().parents[1]
                      / 'launch' / name).read_text())
    return {node.args[0].value
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and getattr(node.func, 'id', '') == 'DeclareLaunchArgument'
            and node.args and isinstance(node.args[0], ast.Constant)}


def test_teleop_launch_declares_every_config_key():
    owned = set(CONFIG_KEYS) | set(FILE_KEYS)
    missing = owned - declared_launch_arguments('teleop.launch.py')
    assert not missing, (
        f'teleop.launch.py never declares {sorted(missing)}, so launch aborts '
        'before any node starts')


# ---- the headset is the default input -------------------------------------
# This stack exists to be driven by a headset, so `device` defaults to 'xr' and
# the scripted device is opt-in. Pinned because the default is one word in one
# launch file, and flipping it back would be silent: every hardware-free test
# passes either way, since they all pass device:=fake explicitly.

def launch_argument_default(name, launch_file='teleop.launch.py'):
    """The default_value of one DeclareLaunchArgument, read statically."""
    tree = ast.parse((Path(__file__).resolve().parents[1]
                      / 'launch' / launch_file).read_text())
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and getattr(node.func, 'id', '') == 'DeclareLaunchArgument'
                and node.args and isinstance(node.args[0], ast.Constant)
                and node.args[0].value == name):
            for kw in node.keywords:
                if kw.arg == 'default_value' and isinstance(kw.value, ast.Constant):
                    return kw.value.value
    raise AssertionError(f'{launch_file} declares no {name!r} with a literal default')


def test_the_headset_is_the_default_device():
    assert launch_argument_default('device') == 'xr'


# ---- a launch file must declare what it reads ------------------------------
# Twice now a LaunchConfiguration was referenced that nothing declared, and
# both times launch aborted before any node started: jump_retry_smoothness in
# teleop.launch.py, and feedback_topic in xr_device.launch.py -- the latter
# only when run standalone, which is exactly what its own docstring tells you
# to do, because including it from teleop.launch.py supplies the name.

# Injected into the launch context by _teleop_config.resolve rather than
# declared, so it is legitimately absent from DeclareLaunchArgument.
INJECTED = {'topic_substitutions'}


def launch_configuration_names(path):
    """(declared, referenced) launch-configuration names in one launch file."""
    tree = ast.parse(path.read_text())
    declared, used = set(), set()
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and node.args
                and isinstance(node.args[0], ast.Constant)):
            fn = getattr(node.func, 'id', '')
            if fn == 'DeclareLaunchArgument':
                declared.add(node.args[0].value)
            elif fn == 'LaunchConfiguration':
                used.add(node.args[0].value)
    return declared, used


@pytest.mark.parametrize('launch_file', sorted(
    p.name for p in (Path(__file__).resolve().parents[1]
                     / 'launch').glob('*.launch.py')))
def test_launch_file_declares_every_configuration_it_reads(launch_file):
    path = Path(__file__).resolve().parents[1] / 'launch' / launch_file
    declared, used = launch_configuration_names(path)
    undeclared = sorted(used - declared - INJECTED)
    assert not undeclared, (
        f'{launch_file} reads {undeclared} but declares none of them; launch '
        'aborts before any node starts')


# ---- the config file lives with the ROBOT ---------------------------------
# It moved out of this package and into the arm's MoveIt config package, so
# that adding an arm is a file in that arm's package rather than an edit here.

def test_a_key_nothing_reads_is_rejected(tmp_path):
    # The failure this prevents is real: fr3_teleop.yaml set
    # retargeting.max_command_step for weeks while the resolver had no such
    # key, so the file documented a setting that was never applied.
    path = write(tmp_path, {'retargeting': {'rotation_scail': 0.5}})
    with pytest.raises(TeleopConfigError, match='which nothing reads'):
        load_config(path, 'ik')


def test_max_command_step_is_now_read(tmp_path):
    path = write(tmp_path, {'retargeting': {'max_command_step': 0.07}})
    assert load_config(path, 'ik')['max_command_step'] == '0.07'


def test_file_names_resolve_relative_to_the_config(tmp_path):
    (tmp_path / 'teleop_scene.yaml').write_text('planning_scene: {}')
    path = write(tmp_path, {'files': {'scene': 'teleop_scene.yaml'}})
    assert load_config(path, 'ik')['scene_config'] == str(
        tmp_path / 'teleop_scene.yaml')


def test_an_absolute_file_name_is_left_alone(tmp_path):
    path = write(tmp_path, {'files': {'scene': '/etc/elsewhere.yaml'}})
    assert load_config(path, 'ik')['scene_config'] == '/etc/elsewhere.yaml'


def test_an_explicit_robot_config_wins_over_the_package():
    cfg = {'robot_config': '/tmp/mine.yaml',
           'robot_config_package': 'agx_arm_moveit'}
    assert resolve_config_path(cfg) == '/tmp/mine.yaml'


def test_naming_neither_is_an_error_that_says_what_to_do():
    cfg = {'robot_config': UNSET, 'robot_config_package': ''}
    with pytest.raises(TeleopConfigError, match='robot_config_package'):
        resolve_config_path(cfg)


def test_an_unknown_package_is_named_in_the_error():
    cfg = {'robot_config': UNSET, 'robot_config_package': 'no_such_arm_config'}
    with pytest.raises(TeleopConfigError, match='no_such_arm_config'):
        resolve_config_path(cfg)


def test_the_package_convention_is_the_one_documented():
    # Named here so that changing it breaks a test rather than every robot.
    assert PACKAGE_CONFIG == 'config/teleop.yaml'


@pytest.mark.parametrize('sample', sorted(
    p.name for p in SAMPLES.glob('*_teleop.yaml')))
def test_every_teleop_sample_parses_and_uses_only_known_keys(sample):
    # A sample that does not load is worse than no sample: it is the file
    # someone copies to start a new arm.
    for solver in ('ik', 'servo'):
        load_config(str(SAMPLES / sample), solver)


@pytest.mark.parametrize('sample', sorted(
    p.name for p in SAMPLES.glob('*scene*.yaml')))
def test_every_scene_sample_parses(sample):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]
                           / 'moveit_teleop_core'))
    from moveit_teleop_core.scene_config import SceneConfig
    SceneConfig.from_file(SAMPLES / sample)


def test_the_template_names_files_the_template_ships(tmp_path):
    # The template's files: section points at template_scene.yaml by the name
    # a copied-and-renamed file would use, so a new arm that follows the
    # instructions literally gets a working pair.
    resolved = load_config(str(SAMPLES / 'template_teleop.yaml'), 'ik')
    assert resolved['scene_config'].endswith('teleop_scene.yaml')


# ---- the docs must point at files that exist ------------------------------
# A bringup guide that links to a renamed page is worse than no link: it sends
# someone to a 404 while looking like it was maintained.

DOC_DIR = Path(__file__).resolve().parents[1] / 'doc'
DOC_LINK = __import__('re').compile(r'\]\(([^)#][^)]*)\)')


@pytest.mark.parametrize('doc', sorted(
    [p.name for p in DOC_DIR.glob('*.md')] + ['../README.md']))
def test_internal_doc_links_resolve(doc):
    path = (DOC_DIR / doc).resolve()
    for target in DOC_LINK.findall(path.read_text()):
        if target.startswith(('http://', 'https://', 'mailto:')):
            continue
        target = target.split('#')[0]
        if not target:
            continue
        assert (path.parent / target).exists(), f'{doc} links to missing {target}'


def test_the_readme_lists_every_doc():
    listed = (DOC_DIR.parent / 'README.md').read_text()
    for page in sorted(DOC_DIR.glob('*.md')):
        assert page.name in listed, f'README does not mention doc/{page.name}'


# ---- the launch files must actually BUILD -----------------------------------
# Every test above this line reads the launch files as text or as an AST, which
# is why a DeclareLaunchArgument carrying both the UNSET sentinel and a
# choices= list passed the whole suite and then failed on the device at launch:
# choices are validated when the description is CONSTRUCTED, and nothing here
# ever constructed one.

import importlib.util  # noqa: E402

LAUNCH_DIR = Path(__file__).resolve().parents[1] / 'launch'
LAUNCH_FILES = sorted(p.name for p in LAUNCH_DIR.glob('*.launch.py'))


def load_launch_module(path):
    """Import a `*.launch.py`, whose name is not a valid module name."""
    spec = importlib.util.spec_from_file_location(
        path.name.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_there_are_launch_files_to_check():
    assert LAUNCH_FILES, 'the glob found nothing, so the test below proves nothing'


@pytest.mark.parametrize('name', LAUNCH_FILES)
def test_the_launch_description_builds(name):
    module = load_launch_module(LAUNCH_DIR / name)
    assert module.generate_launch_description() is not None


@pytest.mark.parametrize('name', LAUNCH_FILES)
def test_no_config_owned_argument_declares_choices(name):
    """UNSET plus choices= is always a launch-time abort.

    The sentinel is not one of the choices, so the declaration itself is
    refused -- and even if it were accepted, choices never see a value that
    came from the config file. Enumerations belong in _teleop_config.CHOICES,
    which is checked after resolution.
    """
    tree = ast.parse((LAUNCH_DIR / name).read_text())
    offenders = []
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id == 'DeclareLaunchArgument'):
            continue
        keywords = {k.arg: k.value for k in node.keywords}
        default = keywords.get('default_value')
        unset = (isinstance(default, ast.Name) and default.id == 'UNSET') or (
            isinstance(default, ast.Constant) and default.value == UNSET)
        if unset and 'choices' in keywords:
            argument = (node.args[0].value if node.args
                        else ast.unparse(keywords.get('name', node)))
            offenders.append(f'{name}:{node.lineno} {argument}')
    assert not offenders, (
        'declare these in _teleop_config.CHOICES instead: ' + ', '.join(offenders))
