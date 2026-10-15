# Dev environment on NVIDIA's isaac_ros_common layering (run_dev.sh / build_image_layers.sh).
# Our own layer, docker/Dockerfile.uet_amr, is stacked on top through NVIDIA's mechanism:
# CONFIG_DOCKER_SEARCH_DIRS adds our Dockerfile directory, and the image key selects the stack
#   x86:    x86_64.ros2_humble.uet_amr
#   Jetson: aarch64.ros2_humble.realsense.uet_amr_realsense.uet_amr
#           (NVIDIA's RSUSB librealsense layer, then ours replacing it with 2.58.4; see dev_build_image)
# The prebuilt NVIDIA base is pulled when one exists (aarch64); otherwise it is built.

ISAAC_SCRIPTS="$REPO_ROOT/ros2/third_party/isaac_ros_common/scripts"
DEV_IMAGE="isaac_ros_dev-$(uname -m)"

dev_image_key() {
  local key=ros2_humble
  is_jetson && key=$key.realsense.uet_amr_realsense
  echo "$(uname -m).$key.uet_amr"
}

# Build context of our layer: one stub package.xml per package holding only its name and dependency
# tags. Version/description/maintainer edits then don't change the context, so they neither rebuild
# the image nor invalidate its rosdep layer. Deterministic: same dependencies -> same bytes.
gen_pkgxml_context() {
  local ctx="$REPO_ROOT/docker/.pkgxml" f out
  rm -rf "$ctx"; mkdir -p "$ctx"
  while IFS= read -r f; do
    out="$ctx/${f#src/}"; mkdir -p "$(dirname "$out")"
    {
      echo '<?xml version="1.0"?>'
      echo '<package format="3">'
      echo "  <name>$(sed -n 's:.*<name>\(.*\)</name>.*:\1:p' "$REPO_ROOT/ros2/$f" | head -1)</name>"
      echo '  <version>0.0.0</version><description>stub</description>'
      echo '  <maintainer email="stub@example.com">stub</maintainer><license>x</license>'
      grep -E '<(buildtool_depend|build_depend|build_export_depend|depend|exec_depend|test_depend)[ >]' "$REPO_ROOT/ros2/$f" | sed 's/^[[:space:]]*/  /' || true
      echo '</package>'
    } > "$out"
  done < <(cd "$REPO_ROOT/ros2" && find src -name package.xml -not -path '*/test/*' | LC_ALL=C sort)
}

# Everything the NVIDIA scripts need, written where they read it (no files in $HOME except the
# dockerargs file, which run_dev.sh only looks for there).
dev_prepare() {
  fetch_third_party isaac_ros_common

  gen_pkgxml_context

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

# Build (or reuse the cached) dev image: NVIDIA layers + our layers.
# On a Jetson the librealsense compile (Dockerfile.uet_amr_realsense, ~15 min) is tagged
# uet_amr_realsense:<hash of its Dockerfile> and reused while that tag exists. Unlike BuildKit's layer
# cache, a tagged image survives `docker builder prune`, so only a pin change recompiles it.
dev_build_image() {
  dev_prepare
  if ! is_jetson; then
    "$ISAAC_SCRIPTS/build_image_layers.sh" --image_key "$(dev_image_key)" --image_name "$DEV_IMAGE"
    return
  fi
  local rs="uet_amr_realsense:$(sha256sum "$REPO_ROOT/docker/Dockerfile.uet_amr_realsense" | cut -c1-12)"
  if ! docker image inspect "$rs" >/dev/null 2>&1; then
    "$ISAAC_SCRIPTS/build_image_layers.sh" --image_key "$(dev_image_key | sed 's/\.uet_amr$//')" --image_name "$rs"
  fi
  docker build --network host -f "$REPO_ROOT/docker/Dockerfile.uet_amr" --build-arg BASE_IMAGE="$rs" \
    -t "$DEV_IMAGE" "$REPO_ROOT/docker"
}
