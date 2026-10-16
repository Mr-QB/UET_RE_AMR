# uet_amr_teleop

Teleoperation for UET AMR: PS5-layout joystick and keyboard, both reaching
`/diff_drive_controller/cmd_vel_unstamped` through `teleop_gate`.

`amr_bringup.launch.py` includes `launch/teleop.launch.py`, which starts the gate, `joy_node`, `teleop_twist_joy` and the
keyboard node. The gate starts with source `none`, so nothing drives until you enable one with the service.
`teleop:=false` on bringup leaves teleop out completely; `teleop_source:=joy` starts with a source already enabled.
Without bringup (e.g. in simulation): `ros2 launch uet_amr_teleop teleop.launch.py initial_source:=keyboard`.

Both sources publish to `/teleop/<source>/cmd_vel_raw`; the gate forwards only the selected one. Switch at runtime with
`uet_amr_msgs/srv/SetTeleop` (the active source is also published, latched, on `/amr/teleop_source`):

```bash
ros2 service call /amr/set_teleop uet_amr_msgs/srv/SetTeleop "{source: joy}"       # or keyboard
ros2 service call /amr/set_teleop uet_amr_msgs/srv/SetTeleop "{source: none}"      # disable, robot stops
```

## Joystick

Hold **L2** (deadman) and use the left stick; releasing L2 stops the robot. Button/axis numbers are in
`config/joy_teleop.yaml` (PS5 layout as `joy_node` reports it; check with `ros2 topic echo /joy` if the pad changes).

## Keyboard

The keyboard node reads keys from the terminal the launch runs in.

```
   q    w    e
   a    s    d
   z    x    c

i/o : increase/decrease max speeds by 10%
k/l : increase/decrease only linear speed by 10%
,/. : increase/decrease only angular speed by 10%
```
