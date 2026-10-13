# Jetson only: the apt librealsense2 (ros-humble-librealsense2) uses the kernel V4L2 backend,
# which fails on Jetson kernels. Build librealsense once with the RSUSB (libusb) backend and
# put it over the apt library; apt's realsense2_camera links the same soname/version
# (librealsense2.so.2.58), so no realsense-ros source build is needed. Idempotent.
# No-op elsewhere: the plain apt packages work there.
setup_realsense_jetson() {
  if ! is_jetson; then
    echo "  (Not a Jetson -- apt realsense2_camera is used as-is)"
    return
  fi

  fetch_third_party librealsense
  local src="$THIRD_PARTY_DIR/librealsense" apt_lib=/opt/ros/humble/lib/aarch64-linux-gnu
  local built="$src/build/Release/librealsense2.so.2.58.4"

  [ -f "$built" ] || sudo apt-get install -y -q libusb-1.0-0-dev libssl-dev pkg-config cmake build-essential udev

  # CUDA speeds up librealsense's own align/pointcloud processing; use it when the machine
  # has a CUDA toolkit (JetPack ships nvcc). Reconfiguring is cheap and a no-op if unchanged.
  local cuda=OFF nvcc=""
  nvcc=$(command -v nvcc || ls /usr/local/cuda/bin/nvcc 2>/dev/null || true)
  [ -n "$nvcc" ] && cuda=ON
  echo -e "${YELLOW}Building librealsense (RSUSB backend, CUDA=$cuda)...${NC}"
  CUDACXX="$nvcc" cmake -S "$src" -B "$src/build" -DCMAKE_BUILD_TYPE=Release -DFORCE_RSUSB_BACKEND=ON \
    -DBUILD_WITH_CUDA=$cuda -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_UNIT_TESTS=OFF
  cmake --build "$src/build" -j"$(nproc)"
  [ -f "$built" ] || built=$(find "$src/build" -name 'librealsense2.so.2.58.4' | head -1)
  [ -f "$built" ] || { echo "librealsense build output not found"; return 1; }

  sudo cmake --install "$src/build"      # /usr/local: rs-enumerate-devices etc.
  sudo ldconfig
  if [ -d "$apt_lib" ] && ! cmp -s "$built" "$apt_lib/librealsense2.so.2.58.4"; then
    echo -e "${YELLOW}Replacing apt librealsense2 with the RSUSB build...${NC}"
    sudo find /usr/local -name 'librealsense2.so*' -exec cp -a --remove-destination {} "$apt_lib/" \;
    sudo apt-mark hold ros-humble-librealsense2 >/dev/null   # keep apt upgrades from undoing it
  fi
}
