# uet_amr_maps

Saved maps and map tooling.

- `maps/<name>/`: `<name>.yaml` + `.pgm` (Nav2), `<name>.posegraph` + `.data` (slam_toolbox), optional `zones.json`.
  Saved by the `/amr/save_map` service of `amr_mode_manager` through `launch/save_map.launch.py`, which runs `clean_map.py`
  on the `.pgm` (removes lidar speckle). Maps live in the source tree (use `colcon build --symlink-install`).
  Default map for bringup/navigation: `maps/gd2/gd2.yaml`.
- `scripts/zones_manager.py`: dashboard zones, below.

## Zones (no-go / slow areas)

`zones_manager.py` (started by `uet_amr_navigation`'s `navigation.launch.py`) applies the dashboard's saved-map zones, schema version 1, via
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
`uet_amr_navigation/config/nav2.yaml` enables `KeepoutFilter` in both costmaps and `SpeedFilter` in the local costmap only.
Tests: `python3 -m pytest test/test_zones_manager.py`.
