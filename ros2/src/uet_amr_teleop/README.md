# uet_amr_teleop

Teleoperation for UET AMR — keyboard and joystick control. Publishes to
`/diff_drive_controller/cmd_vel_unstamped`.

## Keyboard

```bash
ros2 launch uet_amr_teleop keyboard_teleop.launch.py
```

```
   q    w    e
   a    s    d
   z    x    c

i/o : increase/decrease max speeds by 10%
k/l : increase/decrease only linear speed by 10%
,/. : increase/decrease only angular speed by 10%
```

## Controller (joystick)

```bash
# default to ps,can be changed to controller:=xbox
ros2 launch uet_amr_teleop controller_teleop.launch.py controller:=ps
```