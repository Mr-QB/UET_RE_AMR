# Copyright 2011 Brown University Robotics.
# Copyright 2017 Open Source Robotics Foundation, Inc.
# Copyright 2026 UET Robotics Club, University of Engineering and Technology,
#                Vietnam National University, Hanoi (VNU).
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import sys
import select
import termios
import tty

from geometry_msgs.msg import Twist
import rclpy

MSG = """
---------------------------------------------------
UET AMR - Teleop Twist Keyboard (Single Launch)
---------------------------------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c

i/o : increase/decrease max speeds by 10%
k/l : increase/decrease only linear speed by 10%
,/. : increase/decrease only angular speed by 10%

CTRL-C to quit
---------------------------------------------------
"""

MOVE_BINDINGS = {
    'w': (1, 0),
    'a': (1, 1),
    'e': (0, -1),
    'q': (0, 1),
    'd': (1, -1),
    's': (-1, 0),
    'c': (-1, -1),
    'z': (-1, 1),
    'x': (0, 0),
}

SPEED_BINDINGS = {
    'i': (1.1, 1.1),
    'o': (0.9, 0.9),
    'k': (1.1, 1.0),
    'l': (0.9, 1.0),
    ',': (1.0, 1.1),
    '.': (1.0, 0.9),
}


def get_key(settings, tty_file):
    tty.setraw(tty_file.fileno())
    ready, _, _ = select.select([tty_file], [], [], 0.1)
    key = tty_file.read(1) if ready else ''
    termios.tcsetattr(tty_file.fileno(), termios.TCSADRAIN, settings)
    return key


def main():
    try:
        tty_file = open('/dev/tty', 'r')
    except Exception:
        tty_file = sys.stdin

    settings = termios.tcgetattr(tty_file.fileno())

    rclpy.init()
    node = rclpy.create_node('uet_amr_teleop_keyboard')
    pub = node.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)

    speed = 0.5
    turn = 1.0
    x = 0.0
    th = 0.0

    try:
        print(MSG)
        print(f"currently:\tspeed {speed:.2f}\tturn {turn:.2f}")

        while rclpy.ok():
            key = get_key(settings, tty_file)

            if key in MOVE_BINDINGS.keys():
                x = MOVE_BINDINGS[key][0]
                th = MOVE_BINDINGS[key][1]
            elif key in SPEED_BINDINGS.keys():
                speed *= SPEED_BINDINGS[key][0]
                turn *= SPEED_BINDINGS[key][1]
                print(f"currently:\tspeed {speed:.2f}\tturn {turn:.2f}")
            else:
                x = 0.0
                th = 0.0
                if key == '\x03':
                    break

            twist = Twist()
            twist.linear.x = x * speed
            twist.angular.z = th * turn
            pub.publish(twist)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(tty_file.fileno(), termios.TCSADRAIN, settings)
        if tty_file is not sys.stdin:
            tty_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()