#!/usr/bin/env bash
set -e

source /opt/miniconda3/etc/profile.d/conda.sh
conda activate ffs_py310

exec /opt/miniconda3/envs/ffs_py310/bin/python \
  /workspace/code/Guangtong_ws/src/ffs_ros/scripts/ffs_topic_node.py "$@"
