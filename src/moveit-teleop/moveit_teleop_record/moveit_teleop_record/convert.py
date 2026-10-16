"""Recorded episodes -> a LeRobot v2.1 dataset for GR00T or pi0.5 fine-tuning.

    ros2 run moveit_teleop_record convert --robot-config-package agx_arm_moveit \
        ~/episodes ~/datasets/piper_move_iron
    ros2 run moveit_teleop_record convert --robot-config-package agx_arm_moveit \
        ~/more_episodes ~/datasets/piper_move_iron --append

The robot is named exactly as the teleop launch names it, and everything about
its joints comes from its MoveIt model -- see robot_model.py for what is derived
and what the robot's teleop.yaml must provide. Per-robot export choices live in
that file's `dataset:` section; a flag given here overrides it.

Two stages, one command:

  1. extract   (this process, ROS Python) -- read each bag on CAPTURE time,
               align the joints to the camera, trim to the teleop span, and
               write a bundle. See moveit_teleop_record.sync for the rules.
  2. write     (a separate LeRobot interpreter, with ROS stripped from its
               environment) -- build the dataset with openpi's pinned lerobot,
               add GR00T's meta/modality.json, then load it back and check it.

Stage 2 needs the environment made by scripts/setup_lerobot_env.sh. Point at it
with --lerobot-python or LEROBOT_PYTHON if it is not in the default place.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

from .extract import ExportSpec, extract, write_bundle
from .robot_model import RobotConfigError, load_robot_model
from .sync import SyncError, lag_verdict, pool_curves

DEFAULT_LEROBOT_PYTHON = Path.home() / 'miniconda3/envs/lerobot_v21/bin/python'
WRITER = Path(__file__).with_name('lerobot_writer.py')


def _episodes(paths) -> list:
    """Episode directories: each path is one episode, or a directory of them."""
    found = []
    for p in map(Path, paths):
        p = p.expanduser()
        if (p / 'episode.json').exists():
            found.append(p)
        elif p.is_dir():
            found += sorted(c for c in p.iterdir() if (c / 'episode.json').exists())
        else:
            raise SystemExit(f'{p}: not an episode or a directory of episodes')
    return found


def _lerobot_python(arg) -> Path:
    candidate = Path(arg or os.environ.get('LEROBOT_PYTHON') or DEFAULT_LEROBOT_PYTHON)
    if not candidate.exists():
        try:
            from ament_index_python.packages import get_package_share_directory
            setup = (Path(get_package_share_directory('moveit_teleop_record'))
                     / 'scripts' / 'setup_lerobot_env.sh')
        except Exception:  # noqa: BLE001 - only used to word the error
            setup = Path('moveit_teleop_record/scripts/setup_lerobot_env.sh')
        raise SystemExit(
            f'no LeRobot interpreter at {candidate}. Create the pinned environment '
            f'once with:\n    bash {setup}\nor pass --lerobot-python.')
    return candidate


def _clean_env() -> dict:
    """The writer's environment, with every trace of ROS removed.

    ROS's setup scripts put its Python packages on PYTHONPATH -- including its
    own numpy. Inherited, those shadow the LeRobot environment's pinned
    versions, and the failure shows up far from the cause.
    """
    keep = ('HOME', 'USER', 'LANG', 'LC_ALL', 'TERM', 'TMPDIR',
            'HF_HOME', 'HF_LEROBOT_HOME', 'CUDA_VISIBLE_DEVICES')
    env = {k: os.environ[k] for k in keep if k in os.environ}
    env['PATH'] = os.pathsep.join(
        d for d in os.environ.get('PATH', '/usr/bin:/bin').split(os.pathsep)
        if '/opt/ros' not in d and 'install' not in d.split(os.sep)[-3:])
    env['HF_HUB_OFFLINE'] = '1'          # a local dataset needs no network
    return env


def _print_report(name: str, report: dict) -> None:
    trim = report.get('trim')
    motion = report['motion_check']
    kept = f"{report['frames_kept']}/{report['frames_recorded']} frames"
    span = (f", teleop {trim['start_s']}->{trim['end_s']} s of {trim['recorded_s']} s"
            if trim else '')
    print(f'  {name}')
    print(f'     {kept} at {report["dataset_fps"]} fps{span}')
    print(f'     camera latency {report["camera_latency_ms"]} ms (corrected: aligned on '
          f'capture time), joints bracket every frame within '
          f'{report["max_joint_bracket_ms"]} ms')
    if motion['offset_ms'] is None:
        print(f'     image-motion vs joint-speed: {motion["verdict"]}')
    else:
        band = motion['band_ms']
        print(f'     image-motion vs joint-speed: best {motion["offset_ms"]:+.0f} ms, '
              f'r={motion["r"]}'
              + (f', consistent with {band[0]:+d}..{band[1]:+d} ms' if band else '')
              + f' -> {motion["verdict"]}')
    for note in report.get('notes', []):
        print(f'     note: {note}')


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('inputs', nargs='+',
                   help='episode directories, or directories of episodes, then '
                        'the output dataset directory LAST')
    robot = p.add_argument_group('robot (one required)')
    robot.add_argument('--robot-config-package', default=None,
                       help="the arm's MoveIt config package; reads config/teleop.yaml")
    robot.add_argument('--robot-config', default=None,
                       help='explicit path to a teleop.yaml; wins over the package')
    p.add_argument('--repo-id', default=None,
                   help='dataset id (default: local/<output directory name>)')
    p.add_argument('--task', default=None,
                   help='use this task string for every episode, replacing what '
                        'was recorded -- for fixing inconsistent labels')
    # Defaults of None so the robot config's dataset: section can supply them.
    p.add_argument('--image-size', default=None,
                   help="WxH or 'native' (config: dataset.image_size)")
    p.add_argument('--vcodec', default=None, choices=['libsvtav1', 'h264'],
                   help='(config: dataset.vcodec)')
    p.add_argument('--trim', default=None, choices=['teleop', 'none'],
                   help='(config: dataset.trim)')
    p.add_argument('--action-stream', default=None, help='(config: dataset.action_stream)')
    p.add_argument('--state-stream', default=None, help='(config: dataset.state_stream)')
    p.add_argument('--camera-offset-ms', type=float, default=0.0,
                   help='shift applied to camera capture times before sampling joints')
    mode = p.add_mutually_exclusive_group()
    mode.add_argument('--append', action='store_true',
                      help='add episodes to an existing dataset')
    mode.add_argument('--overwrite', action='store_true',
                      help='replace an existing dataset')
    p.add_argument('--strict', action='store_true',
                   help='fail if any episode is skipped or reads as misaligned')
    p.add_argument('--lerobot-python', default=None)
    p.add_argument('--keep-bundles', action='store_true')
    args = p.parse_args(argv)

    if len(args.inputs) < 2:
        p.error('give at least one episode source and an output directory')
    *sources, output = args.inputs
    output = Path(output).expanduser().resolve()
    repo_id = args.repo_id or f'local/{output.name}'

    if output.exists() and any(output.iterdir()):
        is_dataset = (output / 'meta' / 'info.json').exists()
        if not is_dataset:
            raise SystemExit(f'{output} exists and is not a LeRobot dataset; '
                             'refusing to write into it')
        if not (args.append or args.overwrite):
            raise SystemExit(f'{output} is already a dataset: pass --append to add '
                             'episodes or --overwrite to replace it')

    try:
        model = load_robot_model(args.robot_config_package, args.robot_config)
    except RobotConfigError as exc:
        raise SystemExit(f'robot config: {exc}')
    settings = model.dataset
    for flag, key in (('image_size', 'image_size'), ('vcodec', 'vcodec'),
                      ('trim', 'trim'), ('action_stream', 'action_stream'),
                      ('state_stream', 'state_stream')):
        if getattr(args, flag) is not None:
            settings[key] = getattr(args, flag)
    description = model.describe()

    python = _lerobot_python(args.lerobot_python)
    episodes = _episodes(sources)
    if not episodes:
        raise SystemExit('no episodes found')

    print(f"robot {model.name!r} from {model.config_path}")
    for g in description['groups']:
        print(f"  {g['name']:8s} [{g['start']}:{g['end']}]  SRDF '{g['srdf_group']}': "
              + ', '.join(f'{j} ({u})' for j, u in zip(g['joints'], g['units'])))
    if args.task:
        print(f'  task for every episode: {args.task!r}')

    spec = ExportSpec(groups=description['groups'], task=args.task,
                      trim=settings['trim'], state_stream=settings['state_stream'],
                      action_stream=settings['action_stream'],
                      camera_offset=args.camera_offset_ms / 1000.0)
    staging = Path(tempfile.mkdtemp(prefix='teleop_export_',
                                    dir=output.parent if output.parent.exists() else None))
    bundles, skipped, tasks, curves, periods = [], [], {}, [], []
    print(f'\naligning {len(episodes)} episode(s):')
    try:
        for ep in episodes:
            meta = json.loads((ep / 'episode.json').read_text())
            if meta.get('halted_during_episode'):
                skipped.append((ep.name, 'the watchdog halted during this episode'))
                continue
            try:
                bundle = extract(ep, spec)
            except SyncError as exc:
                skipped.append((ep.name, str(exc)))
                continue
            curves.append(bundle.report['motion_check'].pop('curve'))
            periods.append(1.0 / bundle.report['camera_rate_hz'])
            _print_report(ep.name, bundle.report)
            if bundle.report['motion_check']['verdict'] == 'MISALIGNED':
                msg = 'image motion and joint motion disagree by more than half a frame'
                if args.strict:
                    skipped.append((ep.name, msg))
                    continue
                print(f'     WARNING: {msg}; check the camera time_domain')
            tasks.setdefault(bundle.task, []).append(ep.name)
            bundles.append(write_bundle(bundle, staging / f'{len(bundles):04d}'))

        for name, why in skipped:
            print(f'  SKIP {name}: {why}')
        if len(curves) > 1:
            # Same sensors and clocks in every episode, so the evidence pools.
            pooled = lag_verdict(*pool_curves(curves), float(np.median(periods)))
            band = pooled['band_ms']
            print(f"\n  all episodes pooled ({pooled['intervals']} frame intervals): best "
                  f"{pooled['offset_ms']:+.0f} ms, r={pooled['r']}"
                  + (f', consistent with {band[0]:+d}..{band[1]:+d} ms' if band else '')
                  + f" -> {pooled['verdict']}")
            if pooled['verdict'] == 'MISALIGNED':
                print('     WARNING: the camera and joint clocks disagree across the whole '
                      'session; check the camera time_domain before training on this')
        if len(tasks) > 1:
            print('\n  NOTE: episodes carry different task strings, and each becomes a '
                  'separate instruction:')
            for task, names in tasks.items():
                print(f'     {task!r}: {len(names)} episode(s)')
            print("     if they are the same task, re-run with --task '...'")
        if not bundles:
            print('nothing to export', file=sys.stderr)
            return 1
        if skipped and args.strict:
            print(f'{len(skipped)} episode(s) skipped and --strict is set', file=sys.stderr)
            return 1

        export_config = staging / 'export.json'
        export_config.write_text(json.dumps({'robot': description, 'dataset': settings}, indent=2))
        cmd = [str(python), str(WRITER), '--root', str(output), '--repo-id', repo_id,
               '--image-size', settings['image_size'], '--vcodec', settings['vcodec'],
               '--export-config', str(export_config), '--bundles', *map(str, bundles)]
        if args.append:
            cmd.append('--append')
        if args.overwrite:
            cmd.append('--overwrite')
        print(f'\nwriting {len(bundles)} episode(s) with {python}', flush=True)
        result = subprocess.run(cmd, env=_clean_env())
        if result.returncode == 0:
            if skipped:
                print(f'\n{len(skipped)} episode(s) were skipped; see SKIP lines above.')
            print(f'\nview it:   ros2 run moveit_teleop_record view {output}')
        return result.returncode
    finally:
        if args.keep_bundles:
            print(f'bundles kept in {staging}')
        else:
            shutil.rmtree(staging, ignore_errors=True)


def view(argv=None) -> int:
    """Open a dataset in LeRobot's own HTML viewer: video synced with state and action.

        ros2 run moveit_teleop_record view ~/Downloads/piper_move_iron
    """
    p = argparse.ArgumentParser(description=view.__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('dataset')
    p.add_argument('--port', type=int, default=9090)
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--episodes', nargs='*', type=int, default=None)
    p.add_argument('--lerobot-python', default=None)
    args = p.parse_args(argv)

    root = Path(args.dataset).expanduser().resolve()
    if not (root / 'meta' / 'info.json').exists():
        raise SystemExit(f'{root} is not a LeRobot dataset')
    python = _lerobot_python(args.lerobot_python)
    cmd = [str(python), '-m', 'lerobot.scripts.visualize_dataset_html',
           '--repo-id', f'local/{root.name}', '--root', str(root),
           '--host', args.host, '--port', str(args.port)]
    if args.episodes:
        cmd += ['--episodes', *map(str, args.episodes)]
    print(f'open http://{args.host}:{args.port}  (Ctrl-C to stop)', flush=True)
    try:
        return subprocess.run(cmd, env=_clean_env()).returncode
    except KeyboardInterrupt:
        return 0


if __name__ == '__main__':
    raise SystemExit(main())
