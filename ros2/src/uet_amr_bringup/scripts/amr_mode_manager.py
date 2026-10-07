#!/usr/bin/env python3
"""Switch the AMR between SLAM and Nav2 without restarting shared bringup."""

import os
import re
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node

from uet_amr_bringup.srv import SetMode


class AmrModeManager(Node):
    def __init__(self):
        super().__init__('amr_mode_manager')
        self.declare_parameter('initial_mode', 'slam')
        self.declare_parameter('initial_map', '')
        self.declare_parameter('map_directory', 'auto')
        self.declare_parameter('use_rviz', False)
        # NVBlox is only available inside the Isaac ROS Docker environment.
        # Do not infer it from Jetson hardware when this bringup runs natively.
        self.declare_parameter('use_nvblox', 'false')
        self.declare_parameter('startup_grace_sec', 3.0)
        self.declare_parameter('shutdown_timeout_sec', 10.0)
        self.declare_parameter('save_timeout_sec', 45.0)

        self._mode = 'stopped'
        self._process = None
        self._active_map = ''
        self._switch_lock = threading.Lock()
        configured_map_directory = str(self.get_parameter('map_directory').value)
        if configured_map_directory and configured_map_directory.lower() != 'auto':
            self._map_directory = os.path.abspath(os.path.expanduser(configured_map_directory))
        else:
            from ament_index_python.packages import get_package_share_directory
            navigation_share = get_package_share_directory('uet_amr_navigation')
            save_map_launch = os.path.realpath(os.path.join(
                navigation_share, 'launch', 'save_map.launch.py'))
            self._map_directory = os.path.abspath(os.path.join(
                os.path.dirname(save_map_launch), '..', 'maps'))
        self._use_rviz = bool(self.get_parameter('use_rviz').value)
        self._use_nvblox = str(self.get_parameter('use_nvblox').value).strip().lower()
        self._use_sim_time = bool(self.get_parameter('use_sim_time').value)
        self._startup_grace = float(self.get_parameter('startup_grace_sec').value)
        self._shutdown_timeout = float(self.get_parameter('shutdown_timeout_sec').value)
        self._save_timeout = float(self.get_parameter('save_timeout_sec').value)

        self._service = self.create_service(SetMode, '/amr/set_mode', self._set_mode)
        self._monitor = self.create_timer(1.0, self._monitor_process)

        initial_mode = str(self.get_parameter('initial_mode').value)
        initial_map = str(self.get_parameter('initial_map').value)
        if initial_mode == 'nav' and not initial_map:
            self.get_logger().error('initial_mode is nav but initial_map is empty; staying stopped')
        elif initial_mode in ('slam', 'nav'):
            try:
                self._start_mode(initial_mode, initial_map)
            except (OSError, RuntimeError) as exc:
                self.get_logger().error(f'Could not start initial {initial_mode} mode: {exc}')
        else:
            self.get_logger().error(f'Invalid initial_mode {initial_mode!r}; expected slam or nav')

        self.get_logger().info('Mode service ready: /amr/set_mode (uet_amr_bringup/srv/SetMode)')

    def _launch_command(self, mode, map_yaml=''):
        launch_file = 'slam.launch.py' if mode == 'slam' else 'navigation.launch.py'
        command = [
            'ros2', 'launch', 'uet_amr_navigation', launch_file,
            f'use_sim_time:={str(self._use_sim_time).lower()}',
            f'use_rviz:={str(self._use_rviz).lower()}',
        ]
        if mode == 'nav':
            command.extend([
                f'map:={map_yaml}',
            ])
            if self._use_nvblox in ('true', 'false'):
                command.append(f'use_nvblox:={self._use_nvblox}')
        return command

    def _start_mode(self, mode, map_yaml=''):
        if mode == 'nav' and self._ros_node_is_running('slam_toolbox'):
            raise RuntimeError(
                'Cannot start Nav2 while /slam_toolbox is still running; refusing to run two /map publishers')
        if mode == 'nav' and not os.path.isfile(map_yaml):
            raise RuntimeError(f'Map YAML does not exist: {map_yaml}')
        process = subprocess.Popen(
            self._launch_command(mode, map_yaml),
            start_new_session=True,
        )
        time.sleep(self._startup_grace)
        if process.poll() is not None:
            raise RuntimeError(f'{mode} launch exited during startup (exit code {process.returncode})')
        self._process = process
        self._mode = mode
        self._active_map = map_yaml if mode == 'nav' else ''
        self.get_logger().info(f'Started {mode} mode (launch PID {process.pid})')

    def _stop_mode(self):
        process = self._process
        if process is None:
            self._mode = 'stopped'
            return
        stopped_mode = self._mode
        process_group = process.pid
        if process.poll() is None:
            try:
                os.killpg(process_group, signal.SIGINT)
                process.wait(timeout=self._shutdown_timeout)
            except subprocess.TimeoutExpired:
                self.get_logger().warning('Launch did not stop after SIGINT; sending SIGTERM')
                try:
                    os.killpg(process_group, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    self.get_logger().error('Launch did not stop after SIGTERM; sending SIGKILL')
                    try:
                        os.killpg(process_group, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait(timeout=3.0)

        # ros2 launch can exit before all of its child nodes have disappeared
        # from the ROS graph. In particular, Nav2 and SLAM must never overlap
        # as publishers of /map.
        if stopped_mode == 'slam' and not self._wait_for_ros_node_exit('slam_toolbox', 3.0):
            self.get_logger().warning('/slam_toolbox is still visible after launch shutdown; sending SIGTERM to its process group')
            try:
                os.killpg(process_group, signal.SIGTERM)
            except ProcessLookupError:
                pass
            if not self._wait_for_ros_node_exit('slam_toolbox', 2.0):
                try:
                    os.killpg(process_group, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                if not self._wait_for_ros_node_exit('slam_toolbox', 2.0):
                    raise RuntimeError(
                        '/slam_toolbox is still running; refusing to start Nav2 alongside it')

        self._process = None
        self._mode = 'stopped'

    def _ros_node_is_running(self, node_name):
        return any(
            name == node_name and namespace == '/'
            for name, namespace in self.get_node_names_and_namespaces()
        )

    def _wait_for_ros_node_exit(self, node_name, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if not self._ros_node_is_running(node_name):
                return True
            time.sleep(0.2)
        return not self._ros_node_is_running(node_name)

    def _save_map(self, map_name):
        if not re.fullmatch(r'[A-Za-z0-9_-]+', map_name):
            raise RuntimeError('map_name may contain only letters, numbers, underscore, and hyphen')
        map_directory = os.path.join(self._map_directory, map_name)
        os.makedirs(map_directory, exist_ok=True)
        map_base = os.path.join(map_directory, map_name)
        output_files = [map_base + suffix for suffix in ('.yaml', '.pgm', '.posegraph', '.data')]
        previous_mtimes = {
            path: os.stat(path).st_mtime_ns if os.path.exists(path) else None
            for path in output_files
        }
        save_process = subprocess.run(
            ['ros2', 'launch', 'uet_amr_navigation', 'save_map.launch.py',
             f'map_filename:={map_name}', f'map_directory:={map_directory}'],
            timeout=self._save_timeout * 2,
            check=False,
        )
        if save_process.returncode != 0:
            raise RuntimeError(
                f'save_map.launch.py failed with exit code {save_process.returncode}')

        # The launch serializes the pose graph after map_saver_cli completes.
        # Verify that each expected output was created or updated by this call.
        stale_or_missing = [
            path for path in output_files
            if not os.path.isfile(path)
            or (previous_mtimes[path] is not None
                and os.stat(path).st_mtime_ns <= previous_mtimes[path])
        ]
        if stale_or_missing:
            raise RuntimeError(
                'save_map.launch.py did not create/update expected files: '
                + ', '.join(stale_or_missing))
        return map_base + '.yaml'

    def _set_mode(self, request, response):
        if not self._switch_lock.acquire(blocking=False):
            response.success = False
            response.message = 'A mode transition is already in progress'
            response.current_mode = self._mode
            return response

        try:
            target = request.mode.strip().lower()
            if target not in ('slam', 'nav'):
                raise RuntimeError('mode must be "slam" or "nav"')
            self._monitor_process()
            saving_new_map = self._mode == 'slam' and request.save_current_map
            if target == 'nav' and not request.map_yaml and not saving_new_map:
                raise RuntimeError('map_yaml is required when switching to nav mode')

            if target == self._mode and self._process is not None and self._process.poll() is None:
                if target == 'slam' and request.save_current_map:
                    if not request.map_name:
                        raise RuntimeError('map_name is required when save_current_map is true')
                    saved_yaml = self._save_map(request.map_name)
                    response.success = True
                    response.message = f'Map and pose graph saved; map YAML: {saved_yaml}'
                    response.current_mode = self._mode
                    return response
                response.success = True
                response.message = f'Already running {target} mode'
                response.current_mode = self._mode
                return response

            map_yaml = os.path.abspath(os.path.expanduser(request.map_yaml)) if request.map_yaml else ''
            old_mode = self._mode
            old_map = self._active_map
            if self._mode == 'slam' and target == 'nav' and request.save_current_map:
                if not request.map_name:
                    raise RuntimeError('map_name is required when save_current_map is true')
                map_yaml = self._save_map(request.map_name)

            self._stop_mode()
            try:
                self._start_mode(target, map_yaml)
            except (OSError, RuntimeError):
                # A failed target launch should not leave the robot without a
                # mode when the previous mode can be restarted.
                if old_mode in ('slam', 'nav'):
                    try:
                        self._start_mode(old_mode, old_map)
                    except (OSError, RuntimeError) as recovery_error:
                        self.get_logger().error(f'Could not restore {old_mode}: {recovery_error}')
                raise

            response.success = True
            response.message = f'Switched to {target} mode'
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
            response.success = False
            response.message = str(exc)
        finally:
            response.current_mode = self._mode
            self._switch_lock.release()
        return response

    def _monitor_process(self):
        if self._process is not None and self._process.poll() is not None:
            code = self._process.returncode
            self.get_logger().error(f'{self._mode} launch exited unexpectedly (exit code {code})')
            self._process = None
            self._mode = 'stopped'

    def destroy_node(self):
        self._stop_mode()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AmrModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
