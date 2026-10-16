#!/usr/bin/env bash
# A small environment for training/verify_gr00t.py: GR00T's DATA code only.
#
#   bash setup_gr00t_check_env.sh [checkout_dir]
#
# Full Isaac-GR00T needs flash-attn, CUDA and torch 2.9 to train. Loading a
# dataset -- modality config, stats, parquet slicing, video decoding -- needs
# none of that, and is the part worth checking before spending GPU hours. The
# pins are GR00T's uv.lock for that path; pyarrow in particular decides how list
# columns come out of parquet, which is exactly what GR00T's slicing depends on.
#
# If you already have a full GR00T environment, use it instead.
set -euo pipefail

CHECKOUT="${1:-$HOME/Isaac-GR00T}"
ENV_NAME="${GR00T_CHECK_ENV:-gr00t_dataload}"
CONDA="${CONDA_EXE:-$HOME/miniconda3/bin/conda}"

if [ ! -d "$CHECKOUT/gr00t" ]; then
    # Sparse and blob-less: the demo videos are most of the repository.
    git clone -q --depth 1 --filter=blob:none --sparse \
        https://github.com/NVIDIA/Isaac-GR00T.git "$CHECKOUT"
    git -C "$CHECKOUT" sparse-checkout set gr00t/data gr00t/utils gr00t/configs
fi

if ! "$CONDA" env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
    "$CONDA" create -y -q -n "$ENV_NAME" --override-channels -c conda-forge python=3.12
fi
PY="$("$CONDA" run -n "$ENV_NAME" which python)"
clean() { env -i HOME="$HOME" PATH=/usr/bin:/bin LANG=C.UTF-8 "$@"; }

clean "$PY" -m pip install -q --index-url https://download.pytorch.org/whl/cpu torch==2.9.0
clean "$PY" -m pip install -q torchcodec==0.8.0 numpy==1.26.4 pandas==2.2.3 \
    pyarrow==23.0.1 scipy==1.15.3 tqdm==4.67.3 opencv-python-headless==4.11.0.86

echo "check a dataset with:"
echo "  env -i HOME=\$HOME PATH=/usr/bin:/bin $PY \\"
echo "      \$(ros2 pkg prefix moveit_teleop_record)/share/moveit_teleop_record/training/verify_gr00t.py \\"
echo "      --gr00t $CHECKOUT --dataset /path/to/dataset"
