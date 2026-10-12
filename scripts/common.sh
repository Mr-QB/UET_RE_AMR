# =============================================================================
# UET AMR -- shared library. Sourced by the scripts in this directory, not run.
#
#   lib/third_party.sh  fetch_third_party   (pins: ros2/third_party.lock)
#   lib/udev.sh         install_udev_rules  (rules: hardware/udev/)
#   lib/ros.sh          install_ros2_deps, build_workspace, setup_bashrc
#   lib/realsense.sh    setup_realsense_jetson (RSUSB librealsense over apt's)
#   lib/devenv.sh       dev_prepare, dev_build_image (docker dev env on NVIDIA's layering)
# =============================================================================

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPTS_DIR")"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Hardware detection
is_jetson() {
  [ -f /etc/nv_tegra_release ]
}

has_nvidia_gpu() {
  is_jetson && return 0
  command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1
}

for _lib in third_party udev ros realsense devenv; do
  source "$SCRIPTS_DIR/lib/$_lib.sh"
done
unset _lib
