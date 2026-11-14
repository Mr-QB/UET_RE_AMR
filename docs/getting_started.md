# Using the robot

Set the machine up first: [setup.md](setup.md). All commands below run inside the container
(`scripts/amr shell`, or prefix a command with `scripts/amr run`).

## Bringup

```bash
ros2 launch uet_amr_bringup amr_bringup.launch.py                  # mode:=slam (default)
ros2 launch uet_amr_bringup amr_bringup.launch.py mode:=nav map:=<path>/<map>.yaml
```

Hardware, localization (EKF) and sensors stay up while the mode changes. Useful arguments: `serial_port`, `use_rviz`,
`use_teleop`, `teleop_source`, `foxglove`, `use_detection`. Modes can be switched at runtime
(`/amr/set_mode`, `uet_amr_msgs/srv/SetMode`).

## Mapping and navigation

1. Start in `mode:=slam` and drive the robot around.
2. Save the map with the `/amr/save_map` service (`uet_amr_msgs/srv/SaveMap`). It writes the map image, the slam_toolbox
   posegraph and the cleaned `.pgm` into `ros2/src/uet_amr_maps/maps/<name>/`. `/amr/list_maps` and `/amr/get_map` list and download them.
3. Switch to `nav` with that map (`/amr/set_mode`, or restart bringup with `mode:=nav map:=...`), set the initial pose,
   and send goals (RViz "2D Goal Pose", Foxglove, or the dashboard).

## Teleop

Teleop starts with bringup but is off until enabled (L2 deadman for the joystick):

```bash
ros2 service call /amr/set_teleop uet_amr_msgs/srv/SetTeleop "{source: joy}"   # or keyboard, or none
```

## Firmware

The STM32 (`firmware/amr_hoverboard_controller`) and ESP32 (`firmware/amr_uart_bridge`) are PlatformIO projects flashed from
your own machine, not from the container; see their READMEs. Install PlatformIO with `pip install platformio`.

## Known firmware-driven quirks (ROS-side compensations)

The firmware is unchanged, so some ROS settings compensate for it. Don't "fix" them without changing the firmware:

- Command gains are split (`motor_command_scale_linear` 18.5, `angular` 39.5 in `uet_amr_hardware/urdf/uet_amr_real.ros2control`)
  because the firmware's steering coefficient halves turn gain.
- Wheel speed is truncated to int8 rpm in the STM32 feedback, which caps reliable speed at about 1.1 m/s.
- The e-stop bit in the ESP32 protocol is always false.
- `wheel_separation: 0.45` in `uet_amr_bringup/config/controllers.yaml` is a calibrated effective value, not the URDF geometry (about 0.304).
