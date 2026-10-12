# Host udev rules (hardware/udev/*.rules): /dev/rplidar and /dev/esp32 symlinks, RealSense USB access.
# Needed on every machine that talks to the hardware -- native or Docker -- and a
# container can't install them, so every setup mode calls this on the host.
install_udev_rules() {
  if [ -f /run/.containerenv ] || [ -f /.dockerenv ] || [ -n "$DISTROBOX_ENTERED" ]; then
    echo -e "${YELLOW}[Container detected] Install udev rules on the HOST:${NC}"
    echo "  sudo cp $REPO_ROOT/hardware/udev/*.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger"
    return
  fi

  local changed=0 rule dest
  # Old name from earlier setups; same rule as 99-rplidar.rules.
  [ -f /etc/udev/rules.d/rplidar.rules ] && sudo rm -f /etc/udev/rules.d/rplidar.rules && changed=1
  for rule in "$REPO_ROOT"/hardware/udev/*.rules; do
    dest="/etc/udev/rules.d/$(basename "$rule")"
    if ! cmp -s "$rule" "$dest"; then
      echo -e "${YELLOW}Installing $(basename "$rule")...${NC}"
      sudo cp "$rule" "$dest"
      changed=1
    fi
  done
  if [ "$changed" = 1 ]; then
    sudo udevadm control --reload && sudo udevadm trigger
    # Serial (/dev/ttyUSB*) access without sudo
    getent group dialout >/dev/null && ! id -nG "$USER" | grep -qw dialout && sudo usermod -aG dialout "$USER" \
      && echo "  Added $USER to 'dialout' -- log out and back in for it to apply."
    return 0
  else
    echo "  (udev rules already installed)"
  fi
}
