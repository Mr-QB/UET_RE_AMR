# Host udev rules (hardware/udev/*.rules): /dev/rplidar and /dev/esp32 symlinks, RealSense USB access.
# Needed on every machine that talks to the hardware -- and a
# container can't install them, so this runs on the host.
install_udev_rules() {
  local changed=0 rule dest
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
