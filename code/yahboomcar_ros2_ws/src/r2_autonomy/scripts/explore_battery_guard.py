#!/usr/bin/env python3
"""Pause frontier exploration and cancel Nav2 goals when battery is low.

Subscribes to /battery_voltage (std_msgs/Float32). Never publishes /cmd_vel.
"""
from action_msgs.srv import CancelGoal

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class ExploreBatteryGuard(Node):
    def __init__(self):
        super().__init__('explore_battery_guard')

        self.declare_parameter('battery_topic', '/battery_voltage')
        self.declare_parameter('stop_voltage', 10.5)
        self.declare_parameter('explore_resume_topic', '/explore/resume')
        self.declare_parameter('cancel_nav_goals', True)
        self.declare_parameter(
            'cancel_action_names',
            ['navigate_to_pose', 'navigate_through_poses'],
        )

        self.stop_voltage = float(self.get_parameter('stop_voltage').value)
        self.cancel_nav_goals = bool(self.get_parameter('cancel_nav_goals').value)
        resume_topic = self.get_parameter('explore_resume_topic').value
        battery_topic = self.get_parameter('battery_topic').value
        action_names = list(self.get_parameter('cancel_action_names').value)

        self._stopped = False
        self._resume_pub = self.create_publisher(Bool, resume_topic, 10)
        self._cancel_clients = [
            (name, self.create_client(CancelGoal, f'/{name}/_action/cancel_goal'))
            for name in action_names
        ]

        self.create_subscription(Float32, battery_topic, self._on_voltage, 10)
        self.get_logger().info(
            f'Explore battery guard: stop below {self.stop_voltage:.2f} V '
            f'(topic {battery_topic}). Does not publish /cmd_vel.'
        )

    def _on_voltage(self, msg):
        if self._stopped or msg.data >= self.stop_voltage:
            return
        self._stopped = True
        self.get_logger().warn(
            f'Battery {msg.data:.2f} V < {self.stop_voltage:.2f} V: '
            'pausing explore and cancelling Nav2 goals'
        )
        pause = Bool()
        pause.data = False
        self._resume_pub.publish(pause)
        if self.cancel_nav_goals:
            self._cancel_all_nav()

    def _cancel_all_nav(self):
        # Empty CancelGoal request (zero UUID, zero stamp) cancels all goals.
        request = CancelGoal.Request()
        for name, client in self._cancel_clients:
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f'{name} cancel service not available')
                continue
            client.call_async(request)
            self.get_logger().info(f'Cancel-all requested on {name}')


def main(args=None):
    rclpy.init(args=args)
    node = ExploreBatteryGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
