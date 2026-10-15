"""
Odometry accuracy test runner for the UET AMR.

Drives a straight line or an in-place rotation, stopping on odometry progress,
and logs wheel odometry, EKF odometry and gyro-integrated yaw to CSV so that
odom_report can turn the runs into graphs for the report. Wheel slip and
scale errors only show up against an external reference, so after each run
the tool asks for the *measured* result (tape measure / protractor mark) and
stores it next to the logged data.

  ros2 run uet_amr_evaluation odom_test straight 10
  ros2 run uet_amr_evaluation odom_test rotate 360
  ros2 run uet_amr_evaluation odom_test rotate -720 --speed 0.6

Each run is written to <out>/<timestamp>_<kind>_<value>/{data.csv,meta.json}.
"""
import argparse
import csv
import json
import math
import os
import sys
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

CMD_VEL_TOPIC = '/diff_drive_controller/cmd_vel_unstamped'
WHEEL_ODOM_TOPIC = '/diff_drive_controller/odom'
EKF_ODOM_TOPIC = '/odometry/filtered'
IMU_TOPIC = '/camera/camera/imu'

RATE_HZ = 50.0
# Braking distance assumed when deciding when to start decelerating; matches
# the deceleration limits in uet_amr_description/config/controllers.yaml.
LIN_DECEL = 0.8
ANG_DECEL = 1.2


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Track:
    """Pose of one odometry source relative to where the test started."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.origin = None   # (x, y, yaw) captured at first sample after reset
        self.x = self.y = self.yaw = 0.0   # in start frame, yaw unwrapped
        self.v = self.w = 0.0
        self.seen = False
        self._last_yaw = None

    def update(self, x, y, yaw, v, w):
        self.seen = True
        self.v, self.w = v, w
        if self.origin is None:
            self.origin = (x, y, yaw)
            self._last_yaw = yaw
        self.yaw += wrap(yaw - self._last_yaw)
        self._last_yaw = yaw
        ox, oy, oyaw = self.origin
        dx, dy = x - ox, y - oy
        c, s = math.cos(oyaw), math.sin(oyaw)
        self.x = c * dx + s * dy
        self.y = -s * dx + c * dy


class OdomTest(Node):
    def __init__(self):
        super().__init__('odom_test')
        self.wheel = Track()
        self.ekf = Track()
        self.gyro_yaw = 0.0
        self.gyro_seen = False
        self._gyro_t = None
        self.pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.create_subscription(Odometry, WHEEL_ODOM_TOPIC,
                                 lambda m: self._odom(self.wheel, m), 10)
        self.create_subscription(Odometry, EKF_ODOM_TOPIC,
                                 lambda m: self._odom(self.ekf, m), 10)
        self.create_subscription(Imu, IMU_TOPIC, self._imu,
                                 qos_profile_sensor_data)

    def _odom(self, track, m):
        p = m.pose.pose
        track.update(p.position.x, p.position.y, yaw_of(p.orientation),
                     m.twist.twist.linear.x, m.twist.twist.angular.z)

    def _imu(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self._gyro_t is not None:
            dt = t - self._gyro_t
            if 0.0 < dt < 0.5:
                self.gyro_yaw += m.angular_velocity.z * dt
        self._gyro_t = t
        self.gyro_seen = True

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def send(self, lin, ang):
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        self.pub.publish(msg)

    def stop(self):
        for _ in range(5):
            self.send(0.0, 0.0)
            self.spin_for(0.02)

    def wait_for_topics(self, timeout=10.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not self.wheel.seen:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.wheel.seen:
            raise RuntimeError(f'no messages on {WHEEL_ODOM_TOPIC}; is the bringup running?')
        self.spin_for(0.5)
        if not self.ekf.seen:
            self.get_logger().warn(f'no {EKF_ODOM_TOPIC}; EKF columns will be empty')
        if not self.gyro_seen:
            self.get_logger().warn(f'no {IMU_TOPIC}; gyro columns will be empty')

    def run(self, kind, target, speed, accel, control, timeout, settle):
        """Execute one test, returns (rows, summary)."""
        for t in (self.wheel, self.ekf):
            t.reset()
        self.gyro_yaw = 0.0
        self.spin_for(0.3)  # let every track capture its origin

        ctl = self.ekf if control == 'ekf' and self.ekf.seen else self.wheel
        sign = 1.0 if target >= 0 else -1.0
        goal = abs(target)
        decel = LIN_DECEL if kind == 'straight' else ANG_DECEL

        def progress():
            return abs(ctl.x) if kind == 'straight' else abs(ctl.yaw)

        rows = []
        period = 1.0 / RATE_HZ
        t0 = time.monotonic()
        cmd = 0.0
        braking = False
        stopped_since = None
        reason = 'target'

        def log():
            rows.append([
                time.monotonic() - t0,
                self.wheel.x, self.wheel.y, self.wheel.yaw, self.wheel.v, self.wheel.w,
                self.ekf.x, self.ekf.y, self.ekf.yaw, self.ekf.v, self.ekf.w,
                self.gyro_yaw, cmd * sign,
            ])

        next_tick = t0
        while rclpy.ok():
            now = time.monotonic()
            if now - t0 > timeout:
                reason = 'timeout'
                break
            rclpy.spin_once(self, timeout_sec=max(0.0, next_tick - now))
            now = time.monotonic()
            if now < next_tick:
                continue
            next_tick += period

            remaining = goal - progress()
            if not braking and remaining <= cmd * cmd / (2.0 * decel):
                braking = True
            if braking:
                cmd = 0.0
            else:
                cmd = min(speed, cmd + accel * period)
            if kind == 'straight':
                self.send(sign * cmd, 0.0)
            else:
                self.send(0.0, sign * cmd)
            log()

            if braking:
                moving = abs(ctl.v) > 0.01 if kind == 'straight' else abs(ctl.w) > 0.02
                if moving:
                    stopped_since = None
                elif stopped_since is None:
                    stopped_since = now
                elif now - stopped_since >= settle:
                    break
        self.stop()
        # keep logging briefly so the tail of the settle is in the data
        for _ in range(int(0.5 * RATE_HZ)):
            self.spin_for(period)
            log()

        w, e = self.wheel, self.ekf
        if kind == 'straight':
            summary = {
                'wheel_reported': w.x, 'ekf_reported': e.x if e.seen else None,
                'wheel_lateral': w.y, 'wheel_heading_drift_deg': math.degrees(w.yaw),
                'gyro_heading_drift_deg': math.degrees(self.gyro_yaw) if self.gyro_seen else None,
            }
        else:
            summary = {
                'wheel_reported_deg': math.degrees(w.yaw),
                'ekf_reported_deg': math.degrees(e.yaw) if e.seen else None,
                'gyro_reported_deg': math.degrees(self.gyro_yaw) if self.gyro_seen else None,
                'wheel_position_drift': math.hypot(w.x, w.y),
            }
        summary['duration_s'] = rows[-1][0] if rows else 0.0
        summary['ended_by'] = reason
        return rows, summary


COLUMNS = ['t',
           'wheel_x', 'wheel_y', 'wheel_yaw', 'wheel_v', 'wheel_w',
           'ekf_x', 'ekf_y', 'ekf_yaw', 'ekf_v', 'ekf_w',
           'gyro_yaw', 'cmd']


def ask_measured(kind):
    unit = 'm' if kind == 'straight' else 'deg'
    what = 'distance travelled (tape measure)' if kind == 'straight' \
        else 'total rotation (marked heading, incl. full turns)'
    try:
        ans = input(f'Measured {what} in {unit} [Enter to skip]: ').strip()
    except EOFError:
        return None
    return float(ans) if ans else None


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('kind', choices=['straight', 'rotate'])
    p.add_argument('value', type=float,
                   help='metres for straight, degrees for rotate (negative = reverse / clockwise)')
    p.add_argument('--speed', type=float, help='m/s (default 0.3) or rad/s (default 0.5)')
    p.add_argument('--accel', type=float, help='command ramp, m/s^2 (default 0.5) or rad/s^2 (default 1.0)')
    p.add_argument('--control', choices=['wheel', 'ekf'], default='wheel',
                   help='odometry used to decide when to stop')
    p.add_argument('--out', default=os.path.expanduser('~/amr_odom_tests'))
    p.add_argument('--label', default='', help='free-text note stored in meta.json (floor, tyre pressure, ...)')
    p.add_argument('--measured', type=float, help='measured result, skips the prompt')
    p.add_argument('--timeout', type=float, default=300.0)
    p.add_argument('--settle', type=float, default=1.0, help='seconds standing still before ending')
    p.add_argument('-y', '--yes', action='store_true', help='do not wait for Enter before moving')
    args = p.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    straight = args.kind == 'straight'
    speed = args.speed or (0.3 if straight else 0.5)
    accel = args.accel or (0.5 if straight else 1.0)
    target = args.value if straight else math.radians(args.value)

    rclpy.init()
    node = OdomTest()
    try:
        node.wait_for_topics()
        if not args.yes:
            what = f'{args.value:g} m' if straight else f'{args.value:g} deg'
            input(f'Place the robot at its start mark. About to drive {args.kind} {what}. '
                  f'Clear the area, press Enter to go (Ctrl-C aborts)... ')
        rows, summary = node.run(args.kind, target, speed, accel, args.control,
                                 args.timeout, args.settle)
    except KeyboardInterrupt:
        node.stop()
        print('\naborted, nothing saved')
        return
    except RuntimeError as exc:
        print(exc)
        return
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

    measured = args.measured if args.measured is not None else ask_measured(args.kind)

    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    run_dir = os.path.join(args.out, f'{stamp}_{args.kind}_{args.value:g}')
    os.makedirs(run_dir, exist_ok=True)
    with open(os.path.join(run_dir, 'data.csv'), 'w', newline='') as f:
        wr = csv.writer(f)
        wr.writerow(COLUMNS)
        wr.writerows(rows)
    meta = {
        'kind': args.kind,
        'target': args.value,
        'unit': 'm' if straight else 'deg',
        'speed': speed, 'accel': accel, 'control': args.control,
        'label': args.label, 'measured': measured,
        'timestamp': datetime.now().isoformat(timespec='seconds'),
        'summary': summary,
    }
    with open(os.path.join(run_dir, 'meta.json'), 'w') as f:
        json.dump(meta, f, indent=2)
    print(json.dumps(summary, indent=2))
    print(f'saved {run_dir}')
    print('(edit "measured" in meta.json later if you did not have it yet)')


if __name__ == '__main__':
    main()
