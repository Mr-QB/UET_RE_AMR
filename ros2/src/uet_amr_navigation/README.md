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
`simulation.launch.py`/`amr_bringup.launch.py` with `mode:=slam`) is
still running -- it saves whatever `slam_toolbox` has published on `/map` at
that moment.

Save directly into `maps/` in the source tree (not the colcon `install/`
share/ copy) using the exact path above, then `colcon build --symlink-install`
so the installed copy picks it up. This produces `warehouse.pgm` +
`warehouse.yaml`, matching the default `map` argument already hardcoded in
`navigation.launch.py` and `simulation.launch.py` -- no other file needs
to change once you save under this name.

If you build a different map (e.g. a different room), pass a different name
to `-f` and pass `map:=<path-to-your-map>.yaml` when launching `mode:=nav`.

## Zones (no-go / slow areas)

`zones_manager.py` (started by `navigation.launch.py`) applies the dashboard's saved-map zones, schema version 1, via
`/amr/apply_zones` (`uet_amr_msgs/srv/ApplyZones`, argument `zones_json` = the exported JSON). It:

- rejects the request unless the export's map fingerprint matches the active map (SHA-256 of the 8-byte big-endian
  YAML length, the YAML bytes, then the PGM bytes), the frame is `map`, the geometry is valid, and the map origin yaw
  is 0 (Nav2 Humble's StaticLayer and costmap filters ignore yaw);
- rasterizes `no_go` zones into `/keepout_filter_mask` and `slow` zones into `/speed_filter_mask` (conservatively:
  every cell whose centre is inside, or touched by the boundary). No-go wins, overlapping slow zones use the lowest
  limit, labels never restrict. Speed mask: absolute m/s (filter type 2), base 0, multiplier = fastest slow limit / 100,
  limits rounded down to 1-100; a collection with a limit under 1% of the fastest is rejected;
- persists the JSON as `zones.json` next to the map YAML and reinstalls it on start when the fingerprint still matches;
- publishes masks and `/keepout_costmap_filter_info`, `/speed_costmap_filter_info` as transient-local topics, and keeps
  the previous revision active on any failure. Older revisions of the same map are rejected.

`success` in the response means the masks were installed and published, not that the costmaps already consumed them.
`nav2.yaml` / `nav2_nvblox.yaml` enable `KeepoutFilter` in both costmaps and `SpeedFilter` in the local costmap only.
Tests: `python3 -m pytest test/test_zones_manager.py`.
