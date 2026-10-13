# Dev environment on NVIDIA's isaac_ros_common layering (run_dev.sh / build_image_layers.sh).
# Our own layer, docker/Dockerfile.uet_amr, is stacked on top through NVIDIA's mechanism:
# CONFIG_DOCKER_SEARCH_DIRS adds our Dockerfile directory, and the image key selects the stack
#   x86:    x86_64.ros2_humble.uet_amr
#   Jetson: aarch64.ros2_humble.realsense.uet_amr   (adds NVIDIA's RSUSB librealsense layer)
# The prebuilt NVIDIA base is pulled when one exists (aarch64); otherwise it is built.

ISAAC_SCRIPTS="$REPO_ROOT/ros2/third_party/isaac_ros_common/scripts"
DEV_IMAGE="isaac_ros_dev-$(uname -m)"

dev_image_key() {
  local key=ros2_humble
  is_jetson && key=$key.realsense
  echo "$(uname -m).$key.uet_amr"
}

# Everything the NVIDIA scripts need, written where they read it (no files in $HOME except the
# dockerargs file, which run_dev.sh only looks for there).
dev_prepare() {
  fetch_third_party isaac_ros_common

  # package.xml-only tree = build context of our layer
  local ctx="$REPO_ROOT/docker/.pkgxml"
  rm -rf "$ctx"; mkdir -p "$ctx"
  (cd "$REPO_ROOT/ros2" && find src -name package.xml -not -path '*/test/*' -exec cp --parents {} "$ctx" \;)

  # Read by both run_dev.sh and build_image_layers.sh. The directory is gitignored.
  local key; key=$(dev_image_key | cut -d. -f2-)    # run_dev.sh prepends the architecture itself
  cat > "$ISAAC_SCRIPTS/.isaac_ros_common-config" <<CFG
CONFIG_IMAGE_KEY=$key
CONFIG_DOCKER_SEARCH_DIRS=("$REPO_ROOT/docker")
CFG

  # Extra `docker run` args for run_dev.sh. Drop any --device whose path isn't present here:
  # docker run fails outright on a missing device.
  local line
  while IFS= read -r line; do
    case "$line" in
      --device=*) [ -e "${line#--device=}" ] || { echo "  (${line#--device=} not present -- not passing it to the container)" >&2; continue; } ;;
    esac
    printf '%s\n' "$line"
  done < "$REPO_ROOT/docker/isaac_ros_dev-dockerargs" > "$HOME/.isaac_ros_dev-dockerargs"
}

# Build (or reuse the cached) dev image: NVIDIA layers + docker/Dockerfile.uet_amr.
dev_build_image() {
  dev_prepare
  "$ISAAC_SCRIPTS/build_image_layers.sh" --image_key "$(dev_image_key)" --image_name "$DEV_IMAGE"
}
