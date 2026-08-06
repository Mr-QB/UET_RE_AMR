import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from launch import LaunchDescription
from launch.actions import OpaqueFunction

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
    'a': (1, -1),
    'e': (0, 1),
    'q': (0, -1),
    'd': (1, 1),
    's': (-1, 0),
    'c': (-1, 1),
    'z': (-1, -1),
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

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def run_teleop_node(context, *args, **kwargs):
    settings = termios.tcgetattr(sys.stdin)

    if not rclpy.ok():
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
            key = get_key(settings)

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
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=run_teleop_node)
    ])