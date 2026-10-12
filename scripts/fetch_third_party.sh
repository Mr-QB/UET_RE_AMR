#!/bin/bash
# Fetch third-party source repos into ros2/third_party/ at the commits pinned in
# ros2/third_party.lock. Run once per machine (the setup modes call the
# needed parts automatically).   Usage: fetch_third_party.sh [name ...]
set -e
source "$(dirname "$0")/common.sh"
names=("$@")
[ ${#names[@]} -eq 0 ] && mapfile -t names < <(grep -vE '^\s*(#|$)' "$REPO_ROOT/ros2/third_party.lock" | awk '{print $1}' | sed 's/@.*//' | awk '!seen[$0]++')
for n in "${names[@]}"; do fetch_third_party "$n"; done
