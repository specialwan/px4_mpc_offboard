#!/usr/bin/env python3
"""
Smart Waypoint Publisher with Auto-Advance
Publishes waypoints untuk MPC Waypoint Follower

Features:
- Auto-advance to next waypoint when current waypoint reached
- Subscribe to drone position
- Smart waypoint sequencing
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np


class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        
        # QoS profile for PX4 topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publisher untuk waypoint
        self.waypoint_pub = self.create_publisher(
            PoseStamped,
            '/waypoint/target',
            10
        )
        
        # Subscriber untuk posisi drone (untuk auto-advance)
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.position_callback,
            qos_profile
        )
        
        # State variables
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.waypoint_reached = False
        self.last_log_time = self.get_clock().now()
        self.mission_complete = False  # Flag untuk mission complete
        
        # Trajectory smoothness parameters
        self.declare_parameter('continuous_mode', True)  # True = smooth continuous, False = stop at each waypoint
        self.declare_parameter('acceptance_radius', 0.8)  # smaller = smoother transitions
        self.declare_parameter('lookahead_distance', 2.0)  # advance to next WP when this close
        self.declare_parameter('loop_mission', False)  # Auto-loop trajectories
        
        self.continuous_mode = self.get_parameter('continuous_mode').get_parameter_value().bool_value
        self.acceptance_radius = self.get_parameter('acceptance_radius').get_parameter_value().double_value
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.loop_mission = self.get_parameter('loop_mission').get_parameter_value().bool_value
        
        # Definisi waypoint sequence (NED coordinates)
        # Format: [x (North), y (East), z (Down), yaw]
        # Default waypoints (can be overridden by trajectory modes)
        self.waypoints = [
            [0.0, 0.0, -5.0, 0.0],      # Waypoint 1: Origin, 5m altitude
            [50.0, 0.0, -5.0, 0.0],    # Waypoint 2: 10m North, 10m altitude
            [50.0, 50.0, -5.0, 0.0],   # Waypoint 3: 20m North, 10m East, 15m altitude
            [0.0, 50.0, -5.0, 0.0],   # Waypoint 4: 10m North, 20m East, 10m altitude 
            [0.0, 100.0, -5.0, 0.0],     # Waypoint 5: 10m East, 5m altitude
            [50.0, 100.0, -5.0, 0.0],      # Waypoint 6: Back to origin
        ]
        
        # ===== Trajectory Mode Selection =====
        # Supported modes: 'default', 'circle', 'square', 'helix'
        self.declare_parameter('waypoint_mode', 'default')
        self.waypoint_mode = self.get_parameter('waypoint_mode').get_parameter_value().string_value
        
        # Circle parameters
        self.declare_parameter('circle_center_n', 0.0)
        self.declare_parameter('circle_center_e', 0.0)
        self.declare_parameter('circle_radius', 25.0)
        self.declare_parameter('circle_altitude', -5.0)  # NED down
        self.declare_parameter('circle_points', 80)  # Increased for smooth trajectory
        
        # Square parameters
        self.declare_parameter('square_center_n', 0.0)
        self.declare_parameter('square_center_e', 0.0)
        self.declare_parameter('square_size', 40.0)
        self.declare_parameter('square_altitude', -5.0)
        self.declare_parameter('square_points_per_side', 10)  # Increased for smooth sides
        
        # Helix parameters
        self.declare_parameter('helix_center_n', 0.0)
        self.declare_parameter('helix_center_e', 0.0)
        self.declare_parameter('helix_radius', 20.0)
        self.declare_parameter('helix_start_altitude', -5.0)
        self.declare_parameter('helix_end_altitude', -25.0)
        self.declare_parameter('helix_turns', 3.0)  # number of full rotations
        self.declare_parameter('helix_points', 120)  # Increased for smooth spiral
        
        # Generate trajectory based on mode
        if self.waypoint_mode == 'circle':
            self.waypoints = self.generate_circle_waypoints(
                center_n=self.get_parameter('circle_center_n').get_parameter_value().double_value,
                center_e=self.get_parameter('circle_center_e').get_parameter_value().double_value,
                radius=self.get_parameter('circle_radius').get_parameter_value().double_value,
                altitude=self.get_parameter('circle_altitude').get_parameter_value().double_value,
                num_points=self.get_parameter('circle_points').get_parameter_value().integer_value
            )
        elif self.waypoint_mode == 'square':
            self.waypoints = self.generate_square_waypoints(
                center_n=self.get_parameter('square_center_n').get_parameter_value().double_value,
                center_e=self.get_parameter('square_center_e').get_parameter_value().double_value,
                size=self.get_parameter('square_size').get_parameter_value().double_value,
                altitude=self.get_parameter('square_altitude').get_parameter_value().double_value,
                points_per_side=self.get_parameter('square_points_per_side').get_parameter_value().integer_value
            )
        elif self.waypoint_mode == 'helix':
            self.waypoints = self.generate_helix_waypoints(
                center_n=self.get_parameter('helix_center_n').get_parameter_value().double_value,
                center_e=self.get_parameter('helix_center_e').get_parameter_value().double_value,
                radius=self.get_parameter('helix_radius').get_parameter_value().double_value,
                start_altitude=self.get_parameter('helix_start_altitude').get_parameter_value().double_value,
                end_altitude=self.get_parameter('helix_end_altitude').get_parameter_value().double_value,
                turns=self.get_parameter('helix_turns').get_parameter_value().double_value,
                num_points=self.get_parameter('helix_points').get_parameter_value().integer_value
            )
        
        self.current_waypoint_index = 0
        
        # Publisher timer (10 Hz - faster for responsive waypoint updates)
        self.timer = self.create_timer(0.1, self.publish_waypoint)
        
        self.get_logger().info('='*60)
        self.get_logger().info('Smart Waypoint Publisher with Auto-Advance')
        self.get_logger().info(f'Waypoint Mode: {self.waypoint_mode}')
        self.get_logger().info(f'Total waypoints: {len(self.waypoints)}')
        self.get_logger().info(f'Tracking Mode: {"CONTINUOUS (smooth)" if self.continuous_mode else "DISCRETE (stop at each WP)"}')
        self.get_logger().info(f'Acceptance radius: {self.acceptance_radius}m')
        self.get_logger().info(f'Lookahead distance: {self.lookahead_distance}m')
        self.get_logger().info(f'Loop mission: {"Yes" if self.loop_mission else "No (stop after last waypoint)"}')
        self.get_logger().info('Publishing waypoints to /waypoint/target')
        self.get_logger().info('='*60)
        
    def position_callback(self, msg):
        """Update current drone position and manage waypoint transitions"""
        self.current_position = np.array([msg.x, msg.y, msg.z])
        
        # Skip if mission already complete and not looping
        if self.mission_complete and not self.loop_mission:
            return
        
        if self.current_waypoint_index >= len(self.waypoints):
            return
        
        # Current waypoint
        wp = self.waypoints[self.current_waypoint_index]
        target_position = np.array([wp[0], wp[1], wp[2]])
        distance = np.linalg.norm(self.current_position - target_position)
        
        # Log distance every 2 seconds
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds / 1e9 > 2.0:
            self.get_logger().info(
                f'WP{self.current_waypoint_index+1}/{len(self.waypoints)}: {distance:.2f}m'
            )
            self.last_log_time = now
        
        # CONTINUOUS MODE: Advance based on lookahead distance (smooth transitions)
        if self.continuous_mode:
            # Switch to next waypoint when within lookahead distance
            if distance < self.lookahead_distance:
                if self.current_waypoint_index < len(self.waypoints) - 1:
                    self.current_waypoint_index += 1
                    self.get_logger().info(
                        f'→ Advancing to WP{self.current_waypoint_index+1} (continuous mode)'
                    )
                else:
                    # Last waypoint - check if should loop
                    if self.loop_mission:
                        self.get_logger().info('↻ Looping trajectory...')
                        self.current_waypoint_index = 0
                    else:
                        if not self.mission_complete:
                            self.get_logger().info('✓ Mission complete - hovering at final waypoint')
                            self.mission_complete = True
        
        # DISCRETE MODE: Stop at each waypoint (original behavior)
        else:
            if distance < self.acceptance_radius and not self.waypoint_reached:
                self.waypoint_reached = True
                self.get_logger().info(
                    f'✓ Waypoint {self.current_waypoint_index+1} REACHED! '
                    f'(distance: {distance:.2f}m)'
                )
                
                # Auto-advance to next waypoint
                if self.current_waypoint_index < len(self.waypoints) - 1:
                    self.current_waypoint_index += 1
                    self.waypoint_reached = False
                    self.get_logger().info(
                        f'➜ Moving to Waypoint {self.current_waypoint_index+1}/{len(self.waypoints)}'
                    )
                else:
                    # Last waypoint reached
                    self.get_logger().info('='*60)
                    self.get_logger().info('🎉 MISSION COMPLETE! All waypoints reached.')
                    
                    if self.loop_mission:
                        self.get_logger().info('Looping back to first waypoint...')
                        self.get_logger().info('='*60)
                        self.current_waypoint_index = 0
                        self.waypoint_reached = False
                    else:
                        self.get_logger().info('Mission will hover at final waypoint.')
                        self.get_logger().info('='*60)
                        self.mission_complete = True
        
    def publish_waypoint(self):
        """Publish current waypoint"""
        if self.current_waypoint_index < len(self.waypoints):
            wp = self.waypoints[self.current_waypoint_index]
            
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            
            # Position (NED)
            msg.pose.position.x = wp[0]
            msg.pose.position.y = wp[1]
            msg.pose.position.z = wp[2]
            
            # Orientation (yaw only, convert to quaternion)
            yaw = wp[3]
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = np.sin(yaw / 2.0)
            msg.pose.orientation.w = np.cos(yaw / 2.0)
            
            self.waypoint_pub.publish(msg)
    
    def generate_circle_waypoints(self, center_n=0.0, center_e=0.0, radius=25.0, altitude=-5.0, num_points=16):
        """Generate waypoints in a circle trajectory (NED frame).
        
        Args:
            center_n: Center North coordinate
            center_e: Center East coordinate
            radius: Circle radius in meters
            altitude: Fixed altitude (NED down, negative = up)
            num_points: Number of waypoints around circle
            
        Returns:
            List of [n, e, d, yaw] waypoints
        """
        waypoints = []
        for i in range(num_points):
            angle = 2.0 * np.pi * i / num_points
            n = center_n + radius * np.cos(angle)
            e = center_e + radius * np.sin(angle)
            d = altitude
            yaw = angle + np.pi / 2.0  # Face tangent to circle
            waypoints.append([float(n), float(e), float(d), float(yaw)])
        
        # Calculate average waypoint spacing for smoothness info
        circumference = 2.0 * np.pi * radius
        avg_spacing = circumference / num_points
        
        self.get_logger().info(
            f'Circle trajectory: center=({center_n:.1f},{center_e:.1f}), '
            f'radius={radius:.1f}m, altitude={-altitude:.1f}m, {num_points} points, '
            f'spacing={avg_spacing:.2f}m/waypoint'
        )
        return waypoints
    
    def generate_square_waypoints(self, center_n=0.0, center_e=0.0, size=40.0, 
                                   altitude=-5.0, points_per_side=3):
        """Generate waypoints in a square trajectory (NED frame).
        
        Args:
            center_n: Center North coordinate
            center_e: Center East coordinate
            size: Side length of square in meters
            altitude: Fixed altitude (NED down, negative = up)
            points_per_side: Number of intermediate points per side (excluding corners)
            
        Returns:
            List of [n, e, d, yaw] waypoints
        """
        waypoints = []
        half = size / 2.0
        
        # Define 4 corners (starting from bottom-left, counter-clockwise)
        corners = [
            [center_n - half, center_e - half],  # SW corner
            [center_n + half, center_e - half],  # SE corner
            [center_n + half, center_e + half],  # NE corner
            [center_n - half, center_e + half],  # NW corner
        ]
        
        # Define yaw for each side (facing direction of travel)
        side_yaws = [0.0, np.pi/2.0, np.pi, -np.pi/2.0]  # N, E, S, W
        
        for i in range(4):
            start_corner = corners[i]
            end_corner = corners[(i + 1) % 4]
            yaw = side_yaws[i]
            
            # Add waypoints along this side
            for j in range(points_per_side + 1):
                t = j / (points_per_side + 1)  # 0 to 1 (excluding 1)
                n = start_corner[0] + t * (end_corner[0] - start_corner[0])
                e = start_corner[1] + t * (end_corner[1] - start_corner[1])
                waypoints.append([float(n), float(e), float(altitude), float(yaw)])
        
        self.get_logger().info(
            f'Square trajectory: center=({center_n:.1f},{center_e:.1f}), '
            f'size={size:.1f}m, altitude={-altitude:.1f}m, {len(waypoints)} points'
        )
        return waypoints
    
    def generate_helix_waypoints(self, center_n=0.0, center_e=0.0, radius=20.0,
                                 start_altitude=-5.0, end_altitude=-25.0,
                                 turns=3.0, num_points=48):
        """Generate waypoints in a helix (spiral) trajectory (NED frame).
        
        Args:
            center_n: Center North coordinate
            center_e: Center East coordinate
            radius: Helix radius in meters
            start_altitude: Starting altitude (NED down, negative = up)
            end_altitude: Ending altitude (NED down, negative = up)
            turns: Number of complete 360° rotations
            num_points: Total number of waypoints
            
        Returns:
            List of [n, e, d, yaw] waypoints
        """
        waypoints = []
        for i in range(num_points):
            t = i / (num_points - 1)  # 0 to 1
            angle = 2.0 * np.pi * turns * t
            n = center_n + radius * np.cos(angle)
            e = center_e + radius * np.sin(angle)
            d = start_altitude + t * (end_altitude - start_altitude)
            yaw = angle + np.pi / 2.0  # Face tangent
            waypoints.append([float(n), float(e), float(d), float(yaw)])
        
        direction = "ascending" if end_altitude > start_altitude else "descending"
        self.get_logger().info(
            f'Helix trajectory: center=({center_n:.1f},{center_e:.1f}), '
            f'radius={radius:.1f}m, {-start_altitude:.1f}m→{-end_altitude:.1f}m ({direction}), '
            f'{turns:.1f} turns, {num_points} points'
        )
        return waypoints
    
    # def generate_helix_waypoints(self, center_n=0.0, center_e=0.0, radius=20.0,
    #                              start_altitude=-5.0, end_altitude=-25.0,
    #                              turns=3.0, num_points=48, loop=False):
    #     """
    #     NED (down positif). Jika loop=True, titik terakhir tidak sama dengan titik awal.
    #     """
    #     waypoints = []
    #     t = np.linspace(0.0, 1.0, num_points, endpoint=loop)  # endpoint=False untuk loop
    #     for ti in t:
    #         angle = 2.0 * np.pi * turns * ti
    #         n = center_n + radius * np.cos(angle)
    #         e = center_e + radius * np.sin(angle)
    #         d = start_altitude + ti * (end_altitude - start_altitude)
    #         yaw = angle + np.pi / 2.0
    #         waypoints.append([float(n), float(e), float(d), float(yaw)])

    #     direction = "ascending" if end_altitude > start_altitude else "descending"
    #     self.get_logger().info(
    #         f'Helix trajectory: center=({center_n:.1f},{center_e:.1f}), '
    #         f'radius={radius:.1f}m, {-start_altitude:.1f}m→{-end_altitude:.1f}m ({direction}), '
    #         f'{turns:.1f} turns, {num_points} points, loop={loop}'
    #     )
    #     return waypoints

    
    def next_waypoint(self):
        """Manually move to next waypoint (for external control)"""
        if self.current_waypoint_index < len(self.waypoints) - 1:
            self.current_waypoint_index += 1
            self.waypoint_reached = False
            self.get_logger().info(f'➜ Manually advanced to waypoint {self.current_waypoint_index + 1}')
        else:
            self.get_logger().info('Mission complete! Looping back to first waypoint')
            self.current_waypoint_index = 0
            self.waypoint_reached = False


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()