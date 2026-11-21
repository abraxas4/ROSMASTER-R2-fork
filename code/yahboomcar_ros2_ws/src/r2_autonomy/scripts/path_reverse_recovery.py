#!/usr/bin/env python3
"""
Path-tracking reverse recovery for R2 robot.
Records recent driven path and reverses along it when blocked.
Alternates steering direction (left/right) after each recovery attempt.
Pose source: odom topic (default /odom, EKF, odom frame) instead of a TF
listener (/tf ~75 Hz in rclpy cost ~20% CPU at rest). Short-range path
replay belongs in the continuous odom frame (REP-105).
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from r2_autonomy_msgs.action import PathReverseRecovery
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
import math
from collections import deque


class PathReverseRecoveryNode(Node):
    def __init__(self):
        super().__init__('path_reverse_recovery')
        
        self.declare_parameter('path_buffer_duration', 5.0)
        self.declare_parameter('reverse_distance', 0.50)
        self.declare_parameter('reverse_speed', 0.15)
        self.declare_parameter('forward_distance', 0.30)
        self.declare_parameter('forward_speed', 0.12)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('path_sample_distance', 0.05)
        self.declare_parameter('odom_topic', '/odom')
        
        self.path_buffer_duration = self.get_parameter('path_buffer_duration').value
        self.reverse_distance = self.get_parameter('reverse_distance').value
        self.reverse_speed = self.get_parameter('reverse_speed').value
        self.forward_distance = self.get_parameter('forward_distance').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.path_sample_distance = self.get_parameter('path_sample_distance').value
        
        self.cb_group = ReentrantCallbackGroup()
        self.cur = None  # (x, y, yaw) in odom frame
        self.odom_frame = 'odom'
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                 self.odom_cb, 10, callback_group=self.cb_group)
        
        self.recent_path = deque(maxlen=200)
        self.last_pose = None
        self.last_steering_direction = 0
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, 'recovery_path', 10)
        
        self.action_server = ActionServer(
            self,
            PathReverseRecovery,
            'path_reverse_recovery',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )
        
        self.timer = self.create_timer(0.1, self.record_path)
        
        self.get_logger().info('Path reverse recovery node initialized')
        
    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT
        
    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT
        
    def odom_cb(self, msg):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.cur = (p.position.x, p.position.y, yaw)
        if msg.header.frame_id:
            self.odom_frame = msg.header.frame_id

    def record_path(self):
        """Record recent poses for reverse tracking"""
        if self.cur is None:
            return
        current_x, current_y, current_yaw = self.cur
        current_time = self.get_clock().now()
        
        if self.last_pose is not None:
            dx = current_x - self.last_pose[0]
            dy = current_y - self.last_pose[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance >= self.path_sample_distance:
                cutoff_time = current_time - rclpy.duration.Duration(
                    seconds=self.path_buffer_duration
                )
                
                while self.recent_path and self.recent_path[0][3] < cutoff_time:
                    self.recent_path.popleft()
                    
                self.recent_path.append((current_x, current_y, current_yaw, current_time))
                self.last_pose = (current_x, current_y, current_yaw)
        else:
            self.recent_path.append((current_x, current_y, current_yaw, current_time))
            self.last_pose = (current_x, current_y, current_yaw)
            
    def execute_callback(self, goal_handle):
        """Execute reverse recovery along recorded path"""
        self.get_logger().info('Starting path reverse recovery')
        
        result = PathReverseRecovery.Result()
        feedback = PathReverseRecovery.Feedback()
        
        if len(self.recent_path) < 2:
            self.get_logger().warn('Not enough path history for reverse recovery')
            goal_handle.abort()
            result.success = False
            return result
            
        path_to_reverse = list(self.recent_path)
        path_to_reverse.reverse()
        
        path_msg = Path()
        path_msg.header.frame_id = self.odom_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw, _ in path_to_reverse:
            pose = PoseStamped()
            pose.header.frame_id = self.odom_frame
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)
        
        if self.cur is None:
            self.get_logger().error('No odom pose yet')
            goal_handle.abort()
            result.success = False
            return result
        start_x, start_y, _ = self.cur
        current_x, current_y = start_x, start_y
            
        self.get_logger().info(
            f'Reversing {self.reverse_distance}m along {len(path_to_reverse)} waypoints'
        )
        
        distance_traveled = 0.0
        period = 0.05  # 20 Hz
        
        while distance_traveled < self.reverse_distance:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.stop_robot()
                result.success = False
                return result
                
            current_x, current_y, current_yaw = self.cur
                
            dx = current_x - start_x
            dy = current_y - start_y
            distance_traveled = math.sqrt(dx*dx + dy*dy)
            
            target_pose = self.find_nearest_path_point(path_to_reverse, current_x, current_y)
            if target_pose is None:
                break
                
            cmd = Twist()
            cmd.linear.x = -self.reverse_speed
            
            angle_to_target = math.atan2(
                target_pose[1] - current_y,
                target_pose[0] - current_x
            )
            angle_error = angle_to_target - current_yaw
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
            
            cmd.angular.z = -1.0 * angle_error
            
            self.cmd_vel_pub.publish(cmd)
            
            feedback.distance_traveled = distance_traveled
            goal_handle.publish_feedback(feedback)
            
            time.sleep(period)
            
        self.stop_robot()
        self.get_logger().info(f'Reversed {distance_traveled:.2f}m')
        
        self.last_steering_direction = -self.last_steering_direction if self.last_steering_direction != 0 else 1
        
        self.get_logger().info(
            f'Now driving forward {self.forward_distance}m with steering bias: '
            f'{"LEFT" if self.last_steering_direction > 0 else "RIGHT"}'
        )
        
        start_x = current_x
        start_y = current_y
        distance_traveled = 0.0
        
        while distance_traveled < self.forward_distance:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.stop_robot()
                result.success = False
                return result
                
            current_x, current_y, current_yaw = self.cur
                
            dx = current_x - start_x
            dy = current_y - start_y
            distance_traveled = math.sqrt(dx*dx + dy*dy)
            
            cmd = Twist()
            cmd.linear.x = self.forward_speed
            cmd.angular.z = self.last_steering_direction * 0.3
            
            self.cmd_vel_pub.publish(cmd)
            
            feedback.distance_traveled = self.reverse_distance + distance_traveled
            goal_handle.publish_feedback(feedback)
            
            time.sleep(period)
            
        self.stop_robot()
        self.get_logger().info('Path reverse recovery completed')
        
        goal_handle.succeed()
        result.success = True
        return result
        
    def find_nearest_path_point(self, path, x, y):
        """Find nearest point on recorded path"""
        min_dist = float('inf')
        nearest = None
        for px, py, pyaw, _ in path:
            dist = math.sqrt((px - x)**2 + (py - y)**2)
            if dist < min_dist:
                min_dist = dist
                nearest = (px, py, pyaw)
        return nearest
        
    def stop_robot(self):
        """Send stop command"""
        cmd = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PathReverseRecoveryNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
