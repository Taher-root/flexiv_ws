#!/usr/bin/env bash
# The environment `convert` writes datasets with: openpi's pinned LeRobot.
#
#   bash setup_lerobot_env.sh            # creates conda env lerobot_v21
#   LEROBOT_ENV=myname bash setup_lerobot_env.sh
#
# Why these exact versions: openpi (pi0 / pi0.5) loads datasets with lerobot at
# commit 0cf8648 (CODEBASE_VERSION v2.1), and its uv.lock fixes the rest. A
# plain `pip install` of that commit resolves huggingface-hub 1.x, numpy 2.x and
# friends -- far newer than that lerobot was written against. So everything
# that touches reading or writing a dataset is pinned to openpi's lock, and the
# dataset is written by the same stack that will read it.
#
# CPU is enough: nothing here trains. Needs conda (Miniconda is fine).
set -euo pipefail

ENV_NAME="${LEROBOT_ENV:-lerobot_v21}"
CONDA="${CONDA_EXE:-$HOME/miniconda3/bin/conda}"
LEROBOT_REV=0cf864870cf29f4738d3ade893e6fd13fbd7cdb5

if ! "$CONDA" env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
    # conda-forge only: the default channels now demand a terms-of-service
    # acceptance that fails non-interactively.
    "$CONDA" create -y -q -n "$ENV_NAME" --override-channels -c conda-forge python=3.11
fi
PY="$("$CONDA" run -n "$ENV_NAME" which python)"

# ROS's setup scripts put its Python packages on PYTHONPATH, which would shadow
# these pins. Install -- and later run -- with a clean environment.
clean() { env -i HOME="$HOME" PATH=/usr/bin:/bin LANG=C.UTF-8 "$@"; }

clean "$PY" -m pip install -q "lerobot @ git+https://github.com/huggingface/lerobot@${LEROBOT_REV}"
clean "$PY" -m pip install -q \
    huggingface-hub==0.32.3 datasets==3.6.0 pyarrow==20.0.0 av==17.0.0 \
    torch==2.7.1 torchvision==0.22.1 torchcodec==0.4.0 numpy==1.26.4 \
    pillow==11.2.1 opencv-python-headless==4.11.0.86 imageio==2.37.0 \
    imageio-ffmpeg==0.6.0 draccus==0.10.0 einops==0.8.1 fsspec==2025.3.0 \
    jsonlines==4.0.0 pandas==2.2.3 diffusers==0.33.1 rerun-sdk==0.23.1 zarr==3.0.8

clean "$PY" - <<'PYCHECK'
from lerobot.common.datasets.lerobot_dataset import CODEBASE_VERSION
from lerobot.common.datasets.video_utils import get_safe_default_codec
assert CODEBASE_VERSION == "v2.1", CODEBASE_VERSION
print(f"lerobot ready: codebase {CODEBASE_VERSION}, video decode via {get_safe_default_codec()}")
PYCHECK
echo "convert will find it at: $PY"
