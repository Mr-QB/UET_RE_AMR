# uet_amr_navigation

Nav2-based navigation stack configuration for UET AMR, plus `slam_toolbox`
mapping. See `docs/getting_started.md` for the full SLAM -> save map -> Nav2
workflow.

## Saved maps

Maps are produced by driving the robot around with SLAM running and then
saving the resulting occupancy grid with:

```bash
ros2 run nav2_map_server map_saver_cli -f src/uet_amr_navigation/maps/warehouse
```

Run this from a second terminal *while* `slam.launch.py` (or
`amr_simulation.launch.py`/`amr_hardware.launch.py` with `mode:=slam`) is
still running -- it saves whatever `slam_toolbox` has published on `/map` at
that moment.

Save directly into `maps/` in the source tree (not the colcon `install/`
share/ copy) using the exact path above, then `colcon build --symlink-install`
so the installed copy picks it up. This produces `warehouse.pgm` +
`warehouse.yaml`, matching the default `map` argument already hardcoded in
`navigation.launch.py` and `amr_simulation.launch.py` -- no other file needs
to change once you save under this name.

If you build a different map (e.g. a different room), pass a different name
to `-f` and pass `map:=<path-to-your-map>.yaml` when launching `mode:=nav`.
