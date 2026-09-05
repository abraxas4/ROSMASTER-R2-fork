#!/usr/bin/env python3
"""Keep joystick steer/speed inside R2 Ackermann limits during mapping."""
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParameters


class MappingSpeedGuard(Node):
    def __init__(self):
        super().__init__('mapping_speed_guard')
        self.declare_parameter('joy_node', 'joy_ctrl')
        # Hardware set_car_motion vy for R2 is about [-0.045, 0.045].
        self.declare_parameter('yspeed_limit', 0.045)
        self.declare_parameter('xspeed_limit', 0.35)
        self.timer = self.create_timer(2.0, self.apply)
        self._applied = False

    def apply(self):
        name = self.get_parameter('joy_node').get_parameter_value().string_value
        client = self.create_client(SetParameters, f'/{name}/set_parameters')
        if not client.wait_for_service(timeout_sec=0.2):
            return
        xs = self.get_parameter('xspeed_limit').value
        ys = self.get_parameter('yspeed_limit').value
        req = SetParameters.Request()
        for key, val in (('xspeed_limit', xs), ('yspeed_limit', ys)):
            p = Parameter()
            p.name = key
            p.value.type = ParameterType.PARAMETER_DOUBLE
            p.value.double_value = float(val)
            req.parameters.append(p)
        client.call_async(req)
        if not self._applied:
            self.get_logger().info(
                f'set {name} xspeed_limit={xs} yspeed_limit={ys} (R2 steer units)'
            )
            self._applied = True


def main():
    rclpy.init()
    node = MappingSpeedGuard()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
