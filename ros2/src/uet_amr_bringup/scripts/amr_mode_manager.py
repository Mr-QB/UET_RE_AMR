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

from uet_amr_msgs.srv import SaveMap, SetMode


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
        self.declare_parameter('slam_shutdown_timeout_sec', 5.0)
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
        self._slam_shutdown_timeout = float(
            self.get_parameter('slam_shutdown_timeout_sec').value)
        self._save_timeout = float(self.get_parameter('save_timeout_sec').value)

        self._set_mode_service = self.create_service(
            SetMode, '/amr/set_mode', self._set_mode)
        self._save_map_service = self.create_service(
            SaveMap, '/amr/save_map', self._save_map_callback)
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

        self.get_logger().info(
            'Services ready: /amr/set_mode (uet_amr_msgs/srv/SetMode), '
            '/amr/save_map (uet_amr_msgs/srv/SaveMap)')

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
        if mode == 'nav':
            slam_pids = self._slam_toolbox_process_pids()
            if slam_pids:
                raise RuntimeError(
                    'Cannot start Nav2 because SLAM has not fully stopped '
                    f'(process PID(s): {", ".join(map(str, slam_pids))})')
            if self._ros_node_is_running('slam_toolbox'):
                self.get_logger().warning(
                    'ROS graph still lists /slam_toolbox, but no slam_toolbox '
                    'process exists; treating this as stale DDS discovery')
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
            if not self._wait_for_slam_shutdown(self._slam_shutdown_timeout):
                raise RuntimeError(
                    f'slam_toolbox process is still running after '
                    f'{self._slam_shutdown_timeout:.1f} seconds; target mode was not started')
            self._mode = 'stopped'
            return
        stopped_mode = self._mode
        process_group = process.pid

        if stopped_mode == 'slam':
            deadline = time.monotonic() + self._slam_shutdown_timeout
            term_sent = False
            kill_sent = False
            try:
                os.killpg(process_group, signal.SIGINT)
            except ProcessLookupError:
                pass

            while True:
                if self._slam_is_stopped(process):
                    self._process = None
                    self._mode = 'stopped'
                    self._active_map = ''
                    return

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    # A graph entry can outlive its process. Mark the mode
                    # stopped when both the launch and slam_toolbox process
                    # have exited, even if DDS has not expired that entry.
                    if process.poll() is not None and not self._slam_toolbox_process_pids():
                        self._process = None
                        self._mode = 'stopped'
                        self._active_map = ''
                    raise RuntimeError(
                        f'SLAM did not stop completely within '
                        f'{self._slam_shutdown_timeout:.1f} seconds; Nav2 was not started')

                if remaining <= 1.0 and not kill_sent:
                    self.get_logger().warning(
                        'SLAM is still running near the shutdown deadline; sending SIGKILL')
                    try:
                        os.killpg(process_group, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    kill_sent = True
                elif remaining <= 2.0 and not term_sent:
                    self.get_logger().warning(
                        'SLAM has not stopped yet; sending SIGTERM')
                    try:
                        os.killpg(process_group, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    term_sent = True
                time.sleep(min(0.1, remaining))

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

        self._process = None
        self._mode = 'stopped'
        self._active_map = ''

    def _slam_is_stopped(self, process=None):
        process_exited = process is None or process.poll() is not None
        return process_exited and not self._slam_toolbox_process_pids()

    def _wait_for_slam_shutdown(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._slam_is_stopped():
                return True
            time.sleep(0.1)
        return self._slam_is_stopped()

    def _ros_node_is_running(self, node_name):
        return any(
            name == node_name and namespace == '/'
            for name, namespace in self.get_node_names_and_namespaces()
        )

    @staticmethod
    def _slam_toolbox_process_pids():
        """Return PIDs of standalone slam_toolbox processes on this host."""
        pids = []
        try:
            entries = os.scandir('/proc')
        except OSError:
            return pids

        with entries:
            for entry in entries:
                if not entry.name.isdigit():
                    continue
                try:
                    with open(os.path.join(entry.path, 'cmdline'), 'rb') as cmdline_file:
                        argv = [part.decode(errors='ignore') for part in
                                cmdline_file.read().split(b'\0') if part]
                except OSError:
                    continue
                if not argv:
                    continue
                executable = os.path.basename(argv[0])
                if (executable in ('async_slam_toolbox_node', 'sync_slam_toolbox_node')
                        or '__node:=slam_toolbox' in argv):
                    pids.append(int(entry.name))
        return pids

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

    def _save_map_callback(self, request, response):
        if not self._switch_lock.acquire(blocking=False):
            response.success = False
            response.message = 'A mode transition or map save is already in progress'
            response.map_yaml = ''
            return response

        try:
            self._monitor_process()
            if (self._mode != 'slam' or self._process is None
                    or self._process.poll() is not None):
                raise RuntimeError('Map can only be saved while SLAM mode is running')
            response.map_yaml = self._save_map(request.map_name)
            response.success = True
            response.message = f'Map saved to {response.map_yaml}'
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
            response.success = False
            response.message = str(exc)
            response.map_yaml = ''
        finally:
            self._switch_lock.release()
        return response

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
            if target == 'nav' and not request.map_yaml:
                raise RuntimeError('map_yaml is required when switching to nav mode')

            if target == self._mode and self._process is not None and self._process.poll() is None:
                response.success = True
                response.message = f'Already running {target} mode'
                response.current_mode = self._mode
                return response

            map_yaml = os.path.abspath(os.path.expanduser(request.map_yaml)) if request.map_yaml else ''
            old_mode = self._mode
            old_map = self._active_map

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
