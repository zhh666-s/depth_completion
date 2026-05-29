#!/usr/bin/env bash
set -euo pipefail

IMAGE="${IMAGE:-zhenghaohan/depth_completion:20260529}"
WORKSPACE="${WORKSPACE:-/workspace/code/Guangtong_ws}"
SOURCE_CONTAINER="${SOURCE_CONTAINER:-}"
EXPORT_DIR="${EXPORT_DIR:-.docker-export}"
PUSH="${PUSH:-1}"
KEEP_TAR="${KEEP_TAR:-0}"

if ! command -v docker >/dev/null 2>&1; then
  echo "docker command not found. Run this script on the Docker host that owns the source container." >&2
  exit 1
fi

if ! command -v tar >/dev/null 2>&1; then
  echo "tar command not found." >&2
  exit 1
fi

if [ -z "$SOURCE_CONTAINER" ] && [ -f /proc/1/cgroup ]; then
  current_container_id="$(sed -n 's#^.*/docker/\([0-9a-f]\{12,\}\).*#\1#p' /proc/1/cgroup | head -n 1)"
  if [ -n "$current_container_id" ] && docker container inspect "$current_container_id" >/dev/null 2>&1; then
    SOURCE_CONTAINER="$current_container_id"
  fi
fi

if [ -z "$SOURCE_CONTAINER" ]; then
  echo "SOURCE_CONTAINER is required." >&2
  echo "Example: SOURCE_CONTAINER=<container_name_or_id> IMAGE=$IMAGE $0" >&2
  exit 1
fi

docker container inspect "$SOURCE_CONTAINER" >/dev/null

tag_safe="$(printf '%s' "$IMAGE" | tr '/:' '__')"
mkdir -p "$EXPORT_DIR"
rootfs_tar="$EXPORT_DIR/${tag_safe}_rootfs.tar"
delete_list="$EXPORT_DIR/${tag_safe}_delete.txt"

echo "Exporting container '$SOURCE_CONTAINER' to $rootfs_tar ..."
docker export "$SOURCE_CONTAINER" -o "$rootfs_tar"

workspace_in_tar="${WORKSPACE#/}"
workspace_regex="$(printf '%s' "$workspace_in_tar" | sed 's/[][(){}.^$*+?|\\/]/\\&/g')"

exclude_regex="^((root|home/[^/]+)/\\.codex(/|$)|(root|home/[^/]+)/\\.vscode-server(/|$)|(root|home/[^/]+)/\\.vscode(/|$)|(root|home/[^/]+)/\\.cursor-server(/|$)|${workspace_regex}/camera_test_output(/|$)|${workspace_regex}/core(\\..*)?$|${workspace_regex}/src/innovusion_zed/data/EM4_calibration[^/]*(/|$)|${workspace_regex}/src/innovusion_zed/data/remap[^/]*(/|$)|${workspace_regex}/src/innovusion_zed/data/[^/]+/depth_denoised(/|$))"

echo "Finding local data and Codex/VSCode files to exclude ..."
tar -tf "$rootfs_tar" | grep -E "$exclude_regex" > "$delete_list" || true

if [ -s "$delete_list" ]; then
  echo "Removing $(wc -l < "$delete_list") paths from exported rootfs ..."
  tar --delete -f "$rootfs_tar" -T "$delete_list"
else
  echo "No excluded paths found in exported rootfs."
fi

echo "Importing sanitized image as $IMAGE ..."
docker import \
  --change "WORKDIR $WORKSPACE" \
  --change 'CMD ["/bin/bash"]' \
  "$rootfs_tar" \
  "$IMAGE"

if [ "$PUSH" = "1" ]; then
  echo "Pushing $IMAGE ..."
  docker push "$IMAGE"
else
  echo "PUSH=0, image was imported locally but not pushed."
fi

if [ "$KEEP_TAR" != "1" ]; then
  rm -f "$rootfs_tar" "$delete_list"
fi

echo "Done: $IMAGE"
