#!/usr/bin/env python3
"""
Battery monitor for R2 robot.
Reads battery voltage from /dev/shm/r2_battery_v and triggers return-to-home
when voltage drops below threshold (10.5V = ~20%).
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import os


class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        
        self.declare_parameter('voltage_file', '/dev/shm/r2_battery_v')
        self.declare_parameter('low_battery_threshold', 10.5)
        self.declare_parameter('check_interval', 5.0)
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('home_yaw', 0.0)
        
        self.voltage_file = self.get_parameter('voltage_file').value
        self.threshold = self.get_parameter('low_battery_threshold').value
        self.check_interval = self.get_parameter('check_interval').value
        self.home_x = self.get_parameter('home_x').value
        self.home_y = self.get_parameter('home_y').value
        self.home_yaw = self.get_parameter('home_yaw').value
        
        self.battery_pub = self.create_publisher(Float32, 'battery_voltage', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.returning_home = False
        self.low_battery_triggered = False
        
        self.timer = self.create_timer(self.check_interval, self.check_battery)
        
        self.get_logger().info(
            f'Battery monitor initialized. Threshold: {self.threshold}V, '
            f'Home: ({self.home_x}, {self.home_y})'
        )
        
    def read_voltage(self):
        try:
            if os.path.exists(self.voltage_file):
                with open(self.voltage_file, 'r') as f:
                    voltage_str = f.read().strip()
                    return float(voltage_str)
            else:
                self.get_logger().warn(
                    f'Battery voltage file not found: {self.voltage_file}',
                    throttle_duration_sec=30.0
                )
                return None
        except (ValueError, IOError) as e:
            self.get_logger().error(
                f'Failed to read battery voltage: {e}',
                throttle_duration_sec=30.0
            )
            return None
            
    def check_battery(self):
        voltage = self.read_voltage()
        
        if voltage is not None:
            msg = Float32()
            msg.data = voltage
            self.battery_pub.publish(msg)
            
            if voltage < self.threshold and not self.low_battery_triggered:
                self.low_battery_triggered = True
                self.get_logger().warn(
                    f'Low battery detected! Voltage: {voltage:.2f}V < {self.threshold}V'
                )
                self.return_to_home()
                
    def return_to_home(self):
        if self.returning_home:
            return
            
        self.returning_home = True
        self.get_logger().info('Initiating return to home...')
        
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation server not available')
            return
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.home_x
        goal_msg.pose.pose.position.y = self.home_y
        goal_msg.pose.pose.position.z = 0.0
        
        import math
        goal_msg.pose.pose.orientation.z = math.sin(self.home_yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(self.home_yaw / 2.0)
        
        self.get_logger().info(f'Sending home goal: ({self.home_x}, {self.home_y})')
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Return to home goal rejected')
            self.returning_home = False
            return
            
        self.get_logger().info('Return to home goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
        
    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Return to home completed with status: {result}')
        self.returning_home = False


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
