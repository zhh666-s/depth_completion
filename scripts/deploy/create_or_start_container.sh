#!/usr/bin/env bash
set -euo pipefail

IMAGE="${IMAGE:-zhenghaohan/depth_completion:20260529}"
CONTAINER="${CONTAINER:-falcons_zed}"
WORKSPACE="${WORKSPACE:-/workspace/code/Guangtong_ws}"
REPO_URL="${REPO_URL:-https://github.com/zhh666-s/depth_completion.git}"
BRANCH="${BRANCH:-main}"
BUILD="${BUILD:-1}"

if ! command -v docker >/dev/null 2>&1; then
  echo "docker command not found. Run this script on the deployment host machine." >&2
  exit 1
fi

if [ -n "${DISPLAY:-}" ] && command -v xhost >/dev/null 2>&1; then
  xhost +local:root >/dev/null 2>&1 || true
fi

docker pull "$IMAGE"

if docker container inspect "$CONTAINER" >/dev/null 2>&1; then
  if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER")" != "true" ]; then
    docker start "$CONTAINER" >/dev/null
  fi
else
  display_value="${DISPLAY:-:0}"
  x11_args=()
  usb_args=()

  if [ -d /tmp/.X11-unix ]; then
    x11_args+=(--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw")
  fi

  if [ -e /dev/bus/usb ]; then
    usb_args+=(--device=/dev/bus/usb)
  fi

  docker run -dit \
    --name "$CONTAINER" \
    --privileged \
    --network host \
    --ipc=host \
    --shm-size=8g \
    --env="DISPLAY=$display_value" \
    --env="QT_X11_NO_MITSHM=1" \
    "${x11_args[@]}" \
    --volume="/dev:/dev" \
    "${usb_args[@]}" \
    --group-add video \
    --restart unless-stopped \
    "$IMAGE" \
    bash -lc "sleep infinity" >/dev/null
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

echo "Container '$CONTAINER' is ready."
echo "Enter it with: docker exec -it $CONTAINER bash"
