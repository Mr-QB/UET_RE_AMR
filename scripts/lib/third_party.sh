# Third-party source repos (rplidar_ros, librealsense, isaac_ros_common) are NOT in git:
# they are cloned on first setup to ros2/third_party/ (gitignored; plain `colcon build`
# from ros2/ picks them up) at the commits pinned in ros2/third_party.lock. Idempotent.
THIRD_PARTY_DIR="$REPO_ROOT/ros2/third_party"

fetch_third_party() {
  local name="$1" url sha dir="$THIRD_PARTY_DIR/$1"
  # A "name@arch" row overrides "name" on that architecture.
  local lock="$REPO_ROOT/ros2/third_party.lock" row
  row=$(grep -E "^$name@$(uname -m)[[:space:]]" "$lock" || grep -E "^$name[[:space:]]" "$lock") \
    || { echo "unknown third-party repo: $name (see ros2/third_party.lock)"; return 1; }
  read -r _ url sha ref <<< "$row"

  if [ ! -d "$dir/.git" ]; then
    echo -e "${YELLOW}Cloning $name...${NC}"
    mkdir -p "$THIRD_PARTY_DIR"
    if [ -n "$ref" ]; then git clone --quiet --depth 1 --branch "$ref" "$url" "$dir"; else git clone --quiet "$url" "$dir"; fi
  fi
  if [ "$(git -C "$dir" rev-parse HEAD)" != "$sha" ]; then
    git -C "$dir" fetch --quiet origin "$sha" 2>/dev/null || git -C "$dir" fetch --quiet --tags origin
    git -C "$dir" checkout --quiet "$sha"
  fi

  # Not ROS workspace packages -- keep colcon from scanning them.
  case "$name" in librealsense) touch "$dir/COLCON_IGNORE" ;; esac

  # Isaac ROS repos ship binaries via Git LFS; without git-lfs the checkout holds pointer
  # files that fail later with a cryptic linker "syntax error".
  if [ "$name" = isaac_ros_common ]; then
    command -v git-lfs >/dev/null 2>&1 || { echo "git-lfs is required for $name (apt install git-lfs)"; return 1; }
    (cd "$dir" && git lfs install --local >/dev/null && git lfs pull)
    # Only its scripts/Dockerfiles are used; its ROS packages come from NVIDIA's apt repo.
    touch "$dir/COLCON_IGNORE"
  fi
}
