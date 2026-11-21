#!/usr/bin/env python3
"""
Patrol waypoint generator for R2 robot.
Generates patrol waypoints automatically from saved map by sampling free space.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
import random
import math


class PatrolWaypointGenerator(Node):
    def __init__(self):
        super().__init__('patrol_waypoint_generator')
        
        self.declare_parameter('num_waypoints', 10)
        self.declare_parameter('min_distance_between_waypoints', 1.5)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('free_threshold', 25)
        self.declare_parameter('safety_margin', 0.50)
        self.declare_parameter('loop_patrol', True)
        
        self.num_waypoints = self.get_parameter('num_waypoints').value
        self.min_distance = self.get_parameter('min_distance_between_waypoints').value
        self.occupied_threshold = self.get_parameter('occupied_threshold').value
        self.free_threshold = self.get_parameter('free_threshold').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.loop_patrol = self.get_parameter('loop_patrol').value
        
        self.map_data = None
        self.waypoints = []
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.waypoint_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        
        self.get_logger().info('Patrol waypoint generator initialized. Waiting for map...')
        
    def map_callback(self, msg):
        """Process map and generate waypoints"""
        if self.map_data is not None:
            return
            
        self.map_data = msg
        self.get_logger().info(
            f'Map received: {msg.info.width}x{msg.info.height}, resolution: {msg.info.resolution}'
        )
        
        self.waypoints = self.generate_waypoints()
        
        if self.waypoints:
            self.get_logger().info(f'Generated {len(self.waypoints)} patrol waypoints')
            self.start_patrol()
        else:
            self.get_logger().error('Failed to generate patrol waypoints')
            
    def is_free_space(self, x_idx, y_idx):
        """Check if map cell is free space with safety margin"""
        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        
        margin_cells = int(self.safety_margin / resolution)
        
        for dy in range(-margin_cells, margin_cells + 1):
            for dx in range(-margin_cells, margin_cells + 1):
                check_x = x_idx + dx
                check_y = y_idx + dy
                
                if check_x < 0 or check_x >= width or check_y < 0 or check_y >= height:
                    return False
                    
                index = check_y * width + check_x
                if index >= len(self.map_data.data):
                    return False
                    
                value = self.map_data.data[index]
                
                if value == -1 or value >= self.occupied_threshold:
                    return False
                    
        return True
        
    def generate_waypoints(self):
        """Generate patrol waypoints by sampling free space"""
        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        
        free_cells = []
        for y in range(0, height, 5):
            for x in range(0, width, 5):
                if self.is_free_space(x, y):
                    world_x = origin_x + (x + 0.5) * resolution
                    world_y = origin_y + (y + 0.5) * resolution
                    free_cells.append((world_x, world_y))
                    
        if len(free_cells) < self.num_waypoints:
            self.get_logger().warn(
                f'Only {len(free_cells)} free cells found, requested {self.num_waypoints}'
            )
            
        waypoints = []
        attempts = 0
        max_attempts = len(free_cells) * 10
        
        while len(waypoints) < min(self.num_waypoints, len(free_cells)) and attempts < max_attempts:
            attempts += 1
            candidate = random.choice(free_cells)
            
            too_close = False
            for wx, wy in waypoints:
                dist = math.sqrt((candidate[0] - wx)**2 + (candidate[1] - wy)**2)
                if dist < self.min_distance:
                    too_close = True
                    break
                    
            if not too_close:
                waypoints.append(candidate)
                
        poses = []
        for i, (x, y) in enumerate(waypoints):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            
            if i < len(waypoints) - 1:
                next_x, next_y = waypoints[i + 1]
            elif self.loop_patrol and len(waypoints) > 0:
                next_x, next_y = waypoints[0]
            else:
                next_x, next_y = x + 1.0, y
                
            yaw = math.atan2(next_y - y, next_x - x)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            poses.append(pose)
            
        return poses
        
    def start_patrol(self):
        """Start following generated waypoints"""
        if not self.waypoint_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Waypoint follower server not available')
            return
            
        goal = FollowWaypoints.Goal()
        goal.poses = self.waypoints
        
        self.get_logger().info(f'Sending {len(goal.poses)} waypoints to follow')
        
        send_goal_future = self.waypoint_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Waypoint goal rejected')
            return
            
        self.get_logger().info('Waypoint goal accepted, patrol starting')
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
        
    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Patrol completed. Missed waypoints: {result.missed_waypoints}')
        
        if self.loop_patrol:
            self.get_logger().info('Restarting patrol loop...')
            self.start_patrol()


def main(args=None):
    rclpy.init(args=args)
    node = PatrolWaypointGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
