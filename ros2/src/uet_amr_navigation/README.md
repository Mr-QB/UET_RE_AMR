# uet_amr_navigation

Nav2 and slam_toolbox configuration for UET AMR.

- `launch/slam.launch.py`: slam_toolbox online mapping (`config/slam.yaml`).
- `launch/navigation.launch.py`: AMCL localization on a saved map + Nav2 (`config/nav2.yaml`), the nvblox perception
  pipeline (`uet_amr_perception`), the zones manager (`uet_amr_maps`) and two Nav2 workarounds:
  `cmd_vel_relay.py` (behavior_server publishes `/cmd_vel`, which nothing consumed) and `amcl_nomotion_updater.py`
  (a stalled robot never gets an AMCL correction).
- `config/behavior_trees/nav_to_pose_backup_first.xml`: backs up before spinning, which the default tree can't do against a wall.
- Velocity smoother output is remapped to `/diff_drive_controller/cmd_vel_unstamped`.

Normally started by `amr_mode_manager` (`/amr/set_mode`) from `uet_amr_bringup`, not by hand. The local costmap's obstacle
sources are the RPLidar scan and nvblox (GPU); there is no CPU fallback. Mapping workflow: [docs/getting_started.md](../../../docs/getting_started.md).
