#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

# SDL button indices, standard D-pad ordering (matches confirmed L1=9/R1=10):
# 11=dpad_up, 12=dpad_down, 13=dpad_left, 14=dpad_right
BTN_LINEAR_UP = 11
BTN_LINEAR_DOWN = 12
BTN_ANGULAR_DOWN = 13
BTN_ANGULAR_UP = 14

TARGET_NODE = 'teleop_twist_joy'


class JoySpeedAdjust(Node):
    def __init__(self):
        super().__init__('joy_speed_adjust')
        self.linear = 0.5
        self.angular = -1.0
        self.prev_buttons = []

        self.cli = self.create_client(SetParameters, f'/{TARGET_NODE}/set_parameters')
        self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.get_logger().info(f'currently: linear {self.linear:.2f}\tangular {self.angular:.2f}')

    def set_param(self, name, value):
        if not self.cli.service_is_ready():
            return
        req = SetParameters.Request()
        param = Parameter()
        param.name = name
        param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
        req.parameters = [param]
        self.cli.call_async(req)

    def joy_cb(self, msg):
        if not self.prev_buttons:
            self.prev_buttons = list(msg.buttons)
            return

        def pressed(i):
            return msg.buttons[i] == 1 and self.prev_buttons[i] == 0

        changed = False
        if pressed(BTN_LINEAR_UP):
            self.linear *= 1.1
            changed = True
        if pressed(BTN_LINEAR_DOWN):
            self.linear *= 0.9
            changed = True
        if pressed(BTN_ANGULAR_UP):
            self.angular *= 1.1
            changed = True
        if pressed(BTN_ANGULAR_DOWN):
            self.angular *= 0.9
            changed = True

        if changed:
            self.set_param('scale_linear.x', self.linear)
            self.set_param('scale_angular.yaw', self.angular)
            self.get_logger().info(f'currently: linear {self.linear:.2f}\tangular {self.angular:.2f}')

        self.prev_buttons = list(msg.buttons)


def main():
    rclpy.init()
    node = JoySpeedAdjust()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
