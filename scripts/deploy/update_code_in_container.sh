#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${CONTAINER:-falcons_zed}"
WORKSPACE="${WORKSPACE:-/workspace/code/Guangtong_ws}"
REPO_URL="${REPO_URL:-https://github.com/zhh666-s/depth_completion.git}"
BRANCH="${BRANCH:-main}"
BUILD="${BUILD:-1}"

if ! command -v docker >/dev/null 2>&1; then
  echo "docker command not found. Run this script on the host machine, not inside the project container." >&2
  exit 1
fi

if ! docker container inspect "$CONTAINER" >/dev/null 2>&1; then
  echo "Container '$CONTAINER' does not exist. Run scripts/deploy/create_or_start_container.sh first." >&2
  exit 1
fi

if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER")" != "true" ]; then
  docker start "$CONTAINER" >/dev/null
fi

docker exec \
  -e WORKSPACE="$WORKSPACE" \
  -e REPO_URL="$REPO_URL" \
  -e BRANCH="$BRANCH" \
  -e BUILD="$BUILD" \
  "$CONTAINER" \
  bash -lc '
set -eo pipefail

if [ -f /opt/ros/noetic/setup.bash ]; then
  source /opt/ros/noetic/setup.bash
fi

if ! command -v git >/dev/null 2>&1; then
  echo "git command not found in container." >&2
  exit 1
fi

mkdir -p "$(dirname "$WORKSPACE")"

if [ ! -d "$WORKSPACE/.git" ]; then
  if [ -e "$WORKSPACE" ] && [ -n "$(find "$WORKSPACE" -mindepth 1 -maxdepth 1 2>/dev/null | head -n 1)" ]; then
    backup="${WORKSPACE}.backup.$(date +%Y%m%d_%H%M%S)"
    echo "Existing non-git workspace found. Moving it to $backup"
    mv "$WORKSPACE" "$backup"
  fi
  git clone --branch "$BRANCH" "$REPO_URL" "$WORKSPACE"
fi

cd "$WORKSPACE"
git remote set-url origin "$REPO_URL" 2>/dev/null || git remote add origin "$REPO_URL"
git fetch origin "$BRANCH"
git reset --hard
git clean -fd
git checkout -B "$BRANCH" "origin/$BRANCH"
git reset --hard "origin/$BRANCH"
git clean -fd

if [ "$BUILD" = "1" ]; then
  catkin_make
fi
'

echo "Container '$CONTAINER' is synced to $REPO_URL#$BRANCH."
if [ "$BUILD" = "1" ]; then
  echo "catkin_make completed in $WORKSPACE."
fi
