"""Load a `convert`-made dataset with Isaac-GR00T's own data code, and check it.

    python verify_gr00t.py --gr00t /path/to/Isaac-GR00T --dataset /path/to/dataset

Run in an environment with GR00T's data-loading stack (numpy 1.26.4, pandas
2.2.3, pyarrow 23.0.1, scipy, torchcodec 0.8.0) -- a full GR00T install works,
and so does scripts/setup_gr00t_check_env.sh. It never imports ROS.

What it proves, against GR00T's real loader rather than its documentation:

* the dataset's generated modality config registers, and GR00T's own stats
  generator runs
  (meta/stats.json and relative_stats.json, exactly as fine-tuning makes them)
* GR00T slices state and action into the robot's groups. Its loader only
  slices a cell it reads as a numpy array and otherwise passes the WHOLE column
  through without a word, so this is checked, not assumed
* every video frame decodes, one per row, at the dataset resolution
* the task string reaches the language column

The dataset is copied first unless --in-place, so the stats files GR00T writes
do not land in it.
"""

from __future__ import annotations

import argparse
import importlib
import json
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--gr00t', required=True, type=Path, help='Isaac-GR00T checkout')
    p.add_argument('--dataset', required=True, type=Path)
    p.add_argument('--config', type=Path, default=None,
                   help='modality config (default: the one convert generated in '
                        'the dataset, <dataset>/gr00t_modality_config.py)')
    p.add_argument('--in-place', action='store_true')
    args = p.parse_args(argv)

    if args.config is None:
        args.config = args.dataset / 'gr00t_modality_config.py'
    sys.path.insert(0, str(args.gr00t.resolve()))
    # Exactly what launch_finetune.load_modality_config does.
    sys.path.append(str(args.config.resolve().parent))
    importlib.import_module(args.config.stem)

    from gr00t.configs.data.embodiment_configs import MODALITY_CONFIGS
    from gr00t.data.dataset.lerobot_episode_loader import LeRobotEpisodeLoader
    from gr00t.data.embodiment_tags import EmbodimentTag
    from gr00t.data.stats import generate_rel_stats, generate_stats

    tag = EmbodimentTag.NEW_EMBODIMENT
    config = MODALITY_CONFIGS[tag.value]

    dataset = args.dataset
    if not args.in_place:
        dataset = Path(tempfile.mkdtemp(prefix='gr00t_check_')) / args.dataset.name
        shutil.copytree(args.dataset, dataset)

    failures = []

    def check(ok, what):
        print(f"  {'ok  ' if ok else 'FAIL'} {what}")
        if not ok:
            failures.append(what)

    print(f'GR00T data code: {args.gr00t}')
    generate_stats(dataset)
    generate_rel_stats(dataset, tag)
    stats = json.loads((dataset / 'meta' / 'stats.json').read_text())
    rel = json.loads((dataset / 'meta' / 'relative_stats.json').read_text())
    check('observation.state' in stats and 'action' in stats,
          'GR00T generated meta/stats.json for observation.state and action')
    action = config['action']
    relative = [k for k, ac in zip(action.modality_keys, action.action_configs or [])
                if ac.rep.name == 'RELATIVE']
    check(all(k in rel for k in relative),
          f'GR00T generated relative action stats for {relative}')

    loader = LeRobotEpisodeLoader(dataset, config)
    modality = json.loads((dataset / 'meta' / 'modality.json').read_text())
    info = json.loads((dataset / 'meta' / 'info.json').read_text())
    tasks = {json.loads(l)['task_index']: json.loads(l)['task']
             for l in (dataset / 'meta' / 'tasks.jsonl').read_text().splitlines() if l}
    h, w = info['features'][f"observation.images.{config['video'].modality_keys[0]}"]['shape'][:2]
    check(len(loader) == info['total_episodes'],
          f'GR00T episode loader sees {len(loader)} episode(s)')

    import pandas as pd
    for i in range(len(loader)):
        df = loader[i]
        raw = pd.read_parquet(sorted((dataset / 'data').rglob('*.parquet'))[i])
        state_keys = config['state'].modality_keys
        check(isinstance(raw['observation.state'].iloc[0], np.ndarray),
              f'episode {i}: parquet state cells read as numpy arrays under '
              f'pandas {pd.__version__} (else GR00T would not slice them)')
        for part, column in (('state', 'observation.state'), ('action', 'action')):
            full = np.stack(raw[column].to_numpy())
            for group in state_keys:
                rng = modality[part][group]
                key = f'{part}.{group}'
                got = np.stack(df[key].to_numpy()) if key in df else None
                want = full[:, rng['start']:rng['end']]
                check(got is not None and got.shape == want.shape
                      and np.allclose(got, want),
                      f'episode {i}: {key} is {None if got is None else got.shape}, '
                      f"sliced [{rng['start']}:{rng['end']}] from {column}")
        for cam in config['video'].modality_keys:
            frames = df[f'video.{cam}'].to_list()
            first = np.asarray(frames[0])
            check(len(frames) == len(raw) and first.shape == (h, w, 3),
                  f'episode {i}: video.{cam} decoded {len(frames)} frames of '
                  f'{first.shape} for {len(raw)} rows')
        lang_cols = [c for c in df.columns if c.startswith('language.')]
        lang = df[lang_cols[0]].iloc[0] if lang_cols else None
        expected = tasks[int(raw['task_index'].iloc[0])]
        check(lang == expected or (isinstance(lang, (list, tuple)) and expected in lang),
              f'episode {i}: language is {lang!r}')

    if not args.in_place:
        shutil.rmtree(dataset.parent, ignore_errors=True)
    if failures:
        print(f'\n{len(failures)} GR00T check(s) FAILED')
        return 1
    print('\nGR00T loads this dataset as fine-tuning will.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
