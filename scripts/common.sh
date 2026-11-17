# =============================================================================
# UET AMR -- shared library. Sourced by the scripts in this directory, not run.
#
#   lib/third_party.sh  fetch_third_party   (pins: ros2/third_party.lock)
#   lib/udev.sh         install_udev_rules  (rules: hardware/udev/)
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

for _lib in third_party udev devenv; do
  source "$SCRIPTS_DIR/lib/$_lib.sh"
done
unset _lib
