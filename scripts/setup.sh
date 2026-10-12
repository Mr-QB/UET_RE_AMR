#!/bin/bash
# =============================================================================
# UET AMR — setup entry point. Pick how this machine runs the robot software;
# only what that mode needs is fetched or compiled.
#
#   scripts/setup.sh                 menu
#   scripts/setup.sh <mode> [args]
#
#   native-dev     bare metal, developer   ROS deps + build + PlatformIO
#   native-prod    bare metal, robot       ROS deps + build
#   docker-dev     Docker dev environment on NVIDIA's isaac_ros_common layering (x86 and Jetson):
#                    no argument = interactive shell; build | shell | run <cmd> | down = long-lived
#                    container for a fast `git pull` + incremental build loop (see docs)
#
# Layout:  setup.sh (this) -> modes/<mode>.sh -> common.sh -> lib/*.sh
# =============================================================================
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

modes=()
for f in "$HERE"/modes/*.sh; do modes+=("$(basename "$f" .sh)"); done

mode="${1:-}"
if [ -z "$mode" ]; then
  echo "How will this machine run the AMR software?"
  select mode in "${modes[@]}"; do [ -n "$mode" ] && break; done
else
  shift
fi

if [ ! -x "$HERE/modes/$mode.sh" ]; then
  echo "unknown mode: $mode (available: ${modes[*]})"
  exit 1
fi
exec "$HERE/modes/$mode.sh" "$@"
