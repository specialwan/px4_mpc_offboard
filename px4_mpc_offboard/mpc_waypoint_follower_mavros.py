#!/usr/bin/env python3
############################################################################
#
#   MPC Position Controller with Waypoint Following (MAVROS Version)
#
#   This version uses Model Predictive Control for trajectory optimization
#   - MPC optimization: 10 Hz
#   - Control output: 50 Hz
#   - 6-state model: [x, y, z, vx, vy, vz]
#   - Output: Acceleration setpoints (ax, ay, az)
#   - PX4 computes thrust and attitude internally
#
############################################################################

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

import numpy as np
import quadprog  # For QP optimization

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, PositionTarget, AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Header


class MPCPositionController:
    """
    MPC Controller for position control
    6-state model: [x, y, z, vx, vy, vz]
    3 control inputs: [ax, ay, az] (acceleration commands)
    """
    
    def __init__(self, dt=0.1, Np=6, Nc=3):
        """
        Initialize MPC controller
        
        Args:
            dt: Sample time (s) - 0.1s = 10 Hz
            Np: Prediction horizon
            Nc: Control horizon
        """
        self.dt = dt
        self.Np = Np
        self.Nc = Nc
        self.nx = 6  # States: [x, y, z, vx, vy, vz]
        self.nu = 3  # Controls: [ax, ay, az]
        
        # System matrices (simple integrator model)
        self.A = np.array([
            [1, 0, 0, dt, 0,  0],   # x
            [0, 1, 0, 0,  dt, 0],   # y
            [0, 0, 1, 0,  0,  dt],  # z
            [0, 0, 0, 1,  0,  0],   # vx
            [0, 0, 0, 0,  1,  0],   # vy
            [0, 0, 0, 0,  0,  1],   # vz
        ])
        
        self.B = np.array([
            [0.5*dt**2, 0,         0        ],  # x
            [0,         0.5*dt**2, 0        ],  # y
            [0,         0,         0.5*dt**2],  # z
            [dt,        0,         0        ],  # vx
            [0,         dt,        0        ],  # vy
            [0,         0,         dt       ],  # vz
        ])
        
        self.C = np.eye(self.nx)  # Full state measurement
        
        # Cost matrices - ALTITUDE PRIORITY TUNING
        # Z has MUCH higher priority than X/Y to avoid horizontal drift eating all control
        self.Q = np.diag([30.0, 30.0, 100.0,    # Position weights (Z >> X,Y to prioritize altitude)
                          10.0, 10.0, 25.0])     # Velocity damping (vz >> vx,vy)
        
        # Control effort weight - VERY LOW to allow aggressive Z control
        self.R = np.diag([0.05, 0.05, 0.05])       # Lower R_z = prioritize vertical acceleration
        
        # Control rate weight - LOWER for responsive control
        self.R_delta = np.diag([0.3, 0.3, 0.2])  # Lower delta_z = allow fast Z changes
        
        # Build prediction matrices
        self._build_prediction_matrices()
        
        # Previous control for rate penalty
        self.u_prev = np.zeros(self.nu)
        
        # Max acceleration limits (m/s^2) - INCREASED for faster response
        self.a_max = 6.0  # Increased - allow faster climb
        
    def _build_prediction_matrices(self):
        """Build MPC prediction matrices Phi and Gamma"""
        
        # Phi: State prediction matrix (Np*nx x nx)
        self.Phi = np.zeros((self.Np * self.nx, self.nx))
        A_power = np.eye(self.nx)
        for i in range(self.Np):
            A_power = A_power @ self.A
            self.Phi[i*self.nx:(i+1)*self.nx, :] = A_power
        
        # Gamma: Control prediction matrix (Np*nx x Nc*nu)
        self.Gamma = np.zeros((self.Np * self.nx, self.Nc * self.nu))
        for i in range(self.Np):
            for j in range(min(i+1, self.Nc)):
                A_power = np.eye(self.nx)
                for k in range(i - j):
                    A_power = A_power @ self.A
                self.Gamma[i*self.nx:(i+1)*self.nx, j*self.nu:(j+1)*self.nu] = A_power @ self.B
        
        # Build QP matrices
        self._build_qp_matrices()
    
    def _build_qp_matrices(self):
        """Build QP cost matrices H and f"""
        
        # Q_bar: Block diagonal Q matrix
        Q_bar = np.kron(np.eye(self.Np), self.Q)
        
        # R_bar: Block diagonal R matrix
        R_bar = np.kron(np.eye(self.Nc), self.R)
        
        # R_delta_bar: Control rate penalty
        R_delta_bar = np.zeros((self.Nc * self.nu, self.Nc * self.nu))
        for i in range(self.Nc):
            R_delta_bar[i*self.nu:(i+1)*self.nu, i*self.nu:(i+1)*self.nu] = self.R_delta
            if i > 0:
                R_delta_bar[i*self.nu:(i+1)*self.nu, (i-1)*self.nu:i*self.nu] = -self.R_delta
                R_delta_bar[(i-1)*self.nu:i*self.nu, i*self.nu:(i+1)*self.nu] = -self.R_delta
        
        # H matrix for QP: H = Gamma^T * Q_bar * Gamma + R_bar + R_delta_bar
        self.H = self.Gamma.T @ Q_bar @ self.Gamma + R_bar + R_delta_bar
        
        # Make H symmetric (numerical stability)
        self.H = (self.H + self.H.T) / 2
        
        # Store for f computation
        self.Q_bar = Q_bar
        self.R_delta_bar = R_delta_bar
    
    def compute_control(self, current_state, reference_state):
        """
        Compute MPC control action
        
        Args:
            current_state: [x, y, z, vx, vy, vz]
            reference_state: [x_ref, y_ref, z_ref, vx_ref, vy_ref, vz_ref]
            
        Returns:
            control: [ax, ay, az] - acceleration commands
        """
        
        # Build reference trajectory (repeat reference for all prediction steps)
        r = np.tile(reference_state, self.Np)
        
        # Compute error prediction
        error_prediction = self.Phi @ current_state - r
        
        # Build extended previous control vector for rate penalty
        # u_prev_extended = [u_prev; u_prev; u_prev] for Nc steps
        # Then delta_u = u - u_prev_extended
        u_prev_extended = np.tile(self.u_prev, self.Nc)
        
        # Compute f vector: f = Gamma^T * Q_bar * error_prediction + R_delta_bar * (-u_prev_extended)
        # Since we want to minimize ||u - u_prev||, we need to add -R_delta_bar * u_prev_extended to f
        f_tracking = self.Gamma.T @ self.Q_bar @ error_prediction
        f_rate = -self.R_delta_bar @ u_prev_extended
        f = f_tracking + f_rate
        
        # Inequality constraints: -a_max <= u <= a_max
        # quadprog format: solve_qp(G, a, C, b, meq) solves:
        #   min 0.5 * x^T G x - a^T x
        #   s.t. C.T x >= b
        #
        # Our constraints: -a_max <= u_i <= a_max for all i
        # Split into: u_i <= a_max  AND  -u_i <= a_max
        #         =>  u_i <= a_max  AND  u_i >= -a_max
        # In quadprog format C.T x >= b:
        #   -u_i >= -a_max  => [-I].T @ u >= -a_max
        #    u_i >= -a_max  => [I].T @ u >= -a_max
        C = np.vstack([np.eye(self.Nc * self.nu),   # u >= -a_max
                       -np.eye(self.Nc * self.nu)])  # -u >= -a_max => u <= a_max
        b = np.hstack([np.full(self.Nc * self.nu, -self.a_max),  # Lower bounds
                       np.full(self.Nc * self.nu, -self.a_max)])  # Upper bounds
        
        # Solve QP: min 0.5 * u^T * H * u - (-f)^T * u
        #           s.t. C.T * u >= b
        try:
            u_opt = quadprog.solve_qp(self.H, -f, C.T, b, meq=0)[0]
            
            # Extract first control action
            control = u_opt[:self.nu]
            
            # Save for next iteration
            self.u_prev = control.copy()
            
            # Clip to limits (safety)
            control = np.clip(control, -self.a_max, self.a_max)
            
            return control
            
        except Exception as e:
            # If QP fails, return zero acceleration (hover)
            print(f"MPC QP failed: {e}")
            return np.zeros(self.nu)


def acceleration_to_attitude_thrust_px4(accel_ned, yaw_desired, hover_thrust=0.59, gravity=9.81):
    """
    Convert acceleration to attitude + thrust using SIMPLIFIED PX4-inspired algorithm
    
    Key idea:
    1. Total specific force = gravity + desired acceleration
    2. Body Z points in direction of total specific force (thrust direction)
    3. Thrust magnitude compensates gravity + provides desired acceleration
    
    Args:
        accel_ned: [ax, ay, az] in NED frame (m/s²)
        yaw_desired: Desired yaw (rad)
        hover_thrust: Hover thrust (0-1), default 0.59
        gravity: Gravity (m/s²)
    
    Returns:
        roll, pitch, yaw, thrust_normalized
    """
    
    ax, ay, az = accel_ned[0], accel_ned[1], accel_ned[2]
    
    # ========== SAFETY: LIMIT ACCELERATION ==========
    # Prevent extreme accelerations that could cause instability
    ax = np.clip(ax, -6.0, 6.0)
    ay = np.clip(ay, -6.0, 6.0)
    az = np.clip(az, -5.0, 5.0)  # Vertical: allow larger range
    
    # ========== STEP 1: Compute desired specific force ==========
    # Newton's law: F_thrust = m * (a_desired - g_vector)
    # In NED frame:
    #   - Gravity vector: [0, 0, +g] (down is positive)
    #   - Desired acceleration: [ax, ay, az] (az < 0 = up)
    #   - Thrust specific force: [ax, ay, az - g]
    #
    # Examples:
    #   Hover (az=0):  [0, 0, 0-9.81] = [0, 0, -9.81] → thrust=9.81 ✅
    #   Climb (az=-3): [0, 0, -3-9.81] = [0, 0, -12.81] → thrust=12.81 ✅
    specific_force = np.array([ax, ay, az - gravity])
    
    # Magnitude of specific force (this determines thrust)
    thrust_magnitude = np.linalg.norm(specific_force)
    
    # Safety: minimum thrust to overcome gravity
    if thrust_magnitude < gravity:  # At least gravity to hover
        thrust_magnitude = gravity
    
    # Body Z-axis: direction of thrust (opposite to specific force in body frame)
    # In NED frame, thrust points opposite to specific force
    body_z = -specific_force / np.linalg.norm(specific_force)  # Use original magnitude for direction
    
    # ========== STEP 2: Convert thrust magnitude to normalized value ==========
    # Map physical thrust (m/s²) to normalized thrust (0-1)
    # At hover: thrust = g (9.81 m/s²) → normalized = hover_thrust (0.59)
    # At climb: thrust > g → normalized > hover_thrust
    # Scaling: normalized_thrust = (thrust / g) * hover_thrust
    thrust_normalized = (thrust_magnitude / gravity) * hover_thrust
    
    # Clamp thrust to REASONABLE range (allow > hover for climb!)
    thrust_normalized = np.clip(thrust_normalized, hover_thrust * 0.5, 1.0)  # Min 0.3, Max 1.0
    
    # DEBUG: Print thrust calculation (uncomment to debug)
    print(f"🔍 THRUST: az={az:.2f} → specific_force=[{specific_force[0]:.1f},{specific_force[1]:.1f},{specific_force[2]:.1f}] → mag={thrust_magnitude:.2f} m/s² → norm={thrust_normalized:.3f}")
    
    # ========== STEP 3: Build rotation matrix from body Z and yaw ==========
    # EXACT PX4 ALGORITHM from PositionControl::bodyzToAttitude()
    
    # Normalize body_z (safety check)
    if np.linalg.norm(body_z) < 1e-8:
        # Zero vector, set safe level value
        body_z = np.array([0.0, 0.0, 1.0])
    
    body_z = body_z / np.linalg.norm(body_z)  # Normalize
    
    # Vector of desired yaw direction in XY plane, rotated by PI/2
    y_C = np.array([-np.sin(yaw_desired), np.cos(yaw_desired), 0.0])
    
    # Desired body_x axis, orthogonal to body_z
    body_x = np.cross(y_C, body_z)
    
    # Keep nose to front while inverted upside down
    if body_z[2] < 0.0:
        body_x = -body_x
    
    # Handle edge case: thrust in XY plane
    if abs(body_z[2]) < 0.000001:
        # Desired thrust is in XY plane, set X downside
        body_x = np.array([0.0, 0.0, 1.0])
    
    body_x = body_x / np.linalg.norm(body_x)  # Normalize
    
    # Desired body_y axis
    body_y = np.cross(body_z, body_x)
    
    # Build rotation matrix R = [body_x | body_y | body_z] (column-wise)
    R = np.column_stack([body_x, body_y, body_z])
    
    # ========== STEP 4: Extract Euler angles from rotation matrix ==========
    # Calculate euler angles (for logging only, must not be used for control)
    roll = np.arctan2(R[2, 1], R[2, 2])
    pitch = np.arcsin(-np.clip(R[2, 0], -1.0, 1.0))
    yaw = np.arctan2(R[1, 0], R[0, 0])
    
    # SAFETY: Limit attitude angles (prevent extreme tilts)
    max_tilt = np.radians(30)  # 30 degrees max tilt
    roll = np.clip(roll, -max_tilt, max_tilt)
    pitch = np.clip(pitch, -max_tilt, max_tilt)
    
    return roll, pitch, yaw, thrust_normalized, R


def quaternion_to_euler(q):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw)
    
    Args:
        q: Quaternion [w, x, y, z]
        
    Returns:
        roll, pitch, yaw in radians
    """
    w, x, y, z = q
    
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if np.abs(sinp) >= 1.0:
        pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


class MPCWaypointFollower(Node):
    """
    MPC-based waypoint follower that receives waypoints from custom publisher (MAVROS Version)
    """

    def __init__(self):
        super().__init__('mpc_waypoint_follower')
        
        # =================================================================
        # MPC CONTROLLER
        # =================================================================
        
        self.mpc = MPCPositionController(dt=0.1, Np=6, Nc=3)
        self.get_logger().info('MPC Controller initialized: dt=0.1s (10 Hz), Np=6, Nc=3')
        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        # =================================================================
        # MAVROS SUBSCRIBERS
        # =================================================================
        
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10)
        
        self.local_pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.local_pose_callback,
            qos_sensor
            )
        
        self.local_vel_sub = self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self.local_vel_callback,
            qos_sensor
            )
        
        # Subscriber for waypoints from custom waypoint publisher
        self.waypoint_sub = self.create_subscription(
            PoseStamped,
            '/waypoint/target',
            self.waypoint_callback,
            10
        )

        # =================================================================
        # MAVROS PUBLISHERS
        # =================================================================
        
        # For attitude control
        self.attitude_pub = self.create_publisher(
            AttitudeTarget,
            '/mavros/setpoint_raw/attitude',
            10)
        
        # For position control (backup)
        self.setpoint_pub = self.create_publisher(
            PositionTarget,
            '/mavros/setpoint_raw/local',
            10)

        # =================================================================
        # MAVROS SERVICE CLIENTS
        # =================================================================
        
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # =================================================================
        # TIMERS
        # =================================================================
        
        # MPC optimization at 10 Hz
        self.mpc_timer = self.create_timer(0.1, self.mpc_callback)
        
        # Control output at 50 Hz
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
        # State machine at 2 Hz
        self.state_timer = self.create_timer(0.5, self.state_machine_callback)
        
        # =================================================================
        # STATE VARIABLES
        # =================================================================
        
        self.current_state = State()
        self.armed = False
        self.offboard_mode = False
        
        # Current state [x, y, z, vx, vy, vz] (NED frame)
        self.current_position = np.zeros(3)
        self.current_velocity = np.zeros(3)
        self.current_orientation = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]
        
        # Target from waypoint publisher (NED)
        self.target_position = np.array([0.0, 0.0, -5.0])  # Default hover
        self.target_velocity = np.zeros(3)  # Target velocity (usually zero for waypoints)
        self.target_yaw = 0.0
        self.waypoint_received = False
        self.last_waypoint_position = np.array([0.0, 0.0, -5.0])  # Track for duplicate detection
        
        # MPC computed acceleration (from optimization)
        self.mpc_acceleration = np.zeros(3)  # [ax, ay, az] in NED frame
        
        # Attitude setpoint (computed from MPC acceleration using PX4 algorithm)
        self.attitude_roll = 0.0
        self.attitude_pitch = 0.0
        self.attitude_yaw = 0.0
        self.attitude_thrust = 0.59  # Hover thrust
        self.attitude_R_matrix = np.eye(3)  # Identity rotation (level attitude)
        
        # Waypoint acceptance radius
        self.acceptance_radius = 1.0  # meters
        
        # Counter for setpoint publishing
        self.setpoint_counter = 0
        
        self.get_logger().info('='*60)
        self.get_logger().info('MPC Attitude Control Waypoint Follower (MAVROS)')
        self.get_logger().info('MPC → Acceleration → Attitude + Thrust (PX4 Algorithm)')
        self.get_logger().info('MPC Optimization: 10 Hz | Control Output: 50 Hz')
        self.get_logger().info('Waiting for waypoints from publisher...')
        self.get_logger().info('='*60)

        self.xy_locked = False
    # =================================================================
    # MAVROS CALLBACKS
    # =================================================================
    
    def state_callback(self, msg):
        """Update MAVROS state"""
        prev_armed = self.armed
        prev_offboard = self.offboard_mode
        
        self.current_state = msg
        self.armed = msg.armed
        self.offboard_mode = (msg.mode == "OFFBOARD")
        
        if prev_armed != self.armed:
            if self.armed:
                self.get_logger().info('✓ ARMED')
            else:
                self.get_logger().warn('✗ DISARMED')
        
        if prev_offboard != self.offboard_mode:
            if self.offboard_mode:
                self.get_logger().info('✓ OFFBOARD MODE ACTIVE')
            else:
                self.get_logger().warn('✗ OFFBOARD MODE INACTIVE')
    
    def local_pose_callback(self, msg):
        """Update current position (ENU to NED conversion)"""
        # MAVROS uses ENU frame, convert to NED for MPC
        # ENU: x=east, y=north, z=up
        # NED: x=north, y=east, z=down
        self.current_position[0] = msg.pose.position.y   # ENU y → NED x
        self.current_position[1] = msg.pose.position.x   # ENU x → NED y  
        self.current_position[2] = -msg.pose.position.z  # ENU z → NED z (negative)
        
        # Store orientation
        self.current_orientation = np.array([
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z
        ])
    
    def local_vel_callback(self, msg):
        """Update current velocity (ENU to NED conversion)"""
        # Convert ENU velocity to NED
        self.current_velocity[0] = msg.twist.linear.y   # ENU vy → NED vx
        self.current_velocity[1] = msg.twist.linear.x   # ENU vx → NED vy
        self.current_velocity[2] = -msg.twist.linear.z  # ENU vz → NED vz (negative)
    
    def waypoint_callback(self, msg):
        """Receive waypoint from custom waypoint publisher (PoseStamped)"""
        # Extract position (assuming waypoint is in NED frame)
        new_position = np.array([msg.pose.position.y, msg.pose.position.x, -msg.pose.position.z])
        
        # Check if this is a new waypoint (not duplicate)
        if np.linalg.norm(new_position - self.last_waypoint_position) > 0.1:
            self.target_position = new_position.copy()
            self.last_waypoint_position = new_position.copy()
            
            # Extract yaw from quaternion
            q = [
                msg.pose.orientation.w,
                msg.pose.orientation.x, 
                msg.pose.orientation.y,
                msg.pose.orientation.z
            ]
            _, _, yaw = quaternion_to_euler(q)
            self.target_yaw = yaw
            
            # if not self.waypoint_received:
            self.waypoint_received = True
                
            self.get_logger().info(
                f'📍 New waypoint: [{self.target_position[0]:.1f}, '
                f'{self.target_position[1]:.1f}, {self.target_position[2]:.1f}] '
                f'yaw={np.degrees(self.target_yaw):.0f}°'
            )
    
    # def mpc_callback(self):
    #     """MPC optimization at 10 Hz - computes acceleration setpoint"""
        
    #     # Only run MPC when in offboard mode
    #     if not self.offboard_mode:
    #         return
        
    #     # IMPORTANT: If no waypoint received yet, hover at current XY position at -5m altitude
    #     # Lock the target when first entering offboard mode!
    #     if not self.waypoint_received:
    #         # Only set target ONCE when we first enter offboard mode
    #         # Check if target is still at default (0, 0, -5)
    #         if np.allclose(self.target_position, [0.0, 0.0, -5.0]):
    #             # Lock target to current XY, but Z = -5m
    #             self.target_position[0] = self.current_position[0]  # Lock X
    #             self.target_position[1] = self.current_position[1]  # Lock Y
    #             self.target_position[2] = -5.0  # Target altitude
    #             self.get_logger().info(
    #                 f'🎯 Target locked: [{self.target_position[0]:.2f}, '
    #                 f'{self.target_position[1]:.2f}, {self.target_position[2]:.2f}]'
    #             )
            
    #         # CRITICAL: Zero velocity target for stable hover!
    #         self.target_velocity = np.zeros(3)
        
    #     # Current state: [x, y, z, vx, vy, vz]
    #     current_state = np.concatenate([self.current_position, self.current_velocity])
        
    #     # Reference state: [x_ref, y_ref, z_ref, vx_ref, vy_ref, vz_ref]
    #     # Target velocity is zero for waypoint hovering
    #     reference_state = np.concatenate([self.target_position, self.target_velocity])
        
    #     # ============================================================
    #     # MPC OPTIMIZATION - Compute optimal acceleration
    #     # ============================================================
    #     acceleration = self.mpc.compute_control(current_state, reference_state)
        
    #     self.mpc_acceleration = acceleration
        
    #     # ============================================================
    #     # CONVERT ACCELERATION TO ATTITUDE + THRUST (PX4 Algorithm)
    #     # ============================================================
    #     roll, pitch, yaw, thrust, R_matrix = acceleration_to_attitude_thrust_px4(
    #         accel_ned=acceleration,
    #         yaw_desired=self.target_yaw,
    #         hover_thrust=0.4,  # X500 hover thrust
    #         gravity=9.81
    #     )
        
    #     # Store attitude setpoint
    #     self.attitude_roll = roll
    #     self.attitude_pitch = pitch
    #     self.attitude_yaw = yaw
    #     self.attitude_thrust = thrust
    #     self.attitude_R_matrix = R_matrix  # Store rotation matrix for quaternion conversion
        
    #     # Log MPC output with detailed altitude tracking
    #     pos_error = np.linalg.norm(self.target_position - self.current_position)
    #     z_error = self.target_position[2] - self.current_position[2]  # Z error
    #     vel_norm = np.linalg.norm(self.current_velocity)
        
    #     # COMPREHENSIVE DEBUG LOGGING
    #     self.get_logger().info(
    #         f'🎯 MPC OUTPUT:\n'
    #         f'   Position: Z={self.current_position[2]:.2f}m → Target={self.target_position[2]:.2f}m (error={z_error:.2f}m)\n'
    #         f'   Velocity: vz={self.current_velocity[2]:.2f} m/s\n'
    #         f'   Acceleration: az={acceleration[2]:.3f} m/s² ({"CLIMB" if acceleration[2] < 0 else "DESCEND/HOVER"})\n'
    #         f'   Thrust: {thrust:.3f} ({">"if thrust > 0.59 else "="if thrust == 0.59 else "<"} hover=0.59)\n'
    #         f'   Attitude: R={np.rad2deg(roll):.1f}° P={np.rad2deg(pitch):.1f}° Y={np.rad2deg(yaw):.1f}°'
    #     )
    
    def mpc_callback(self):
        """MPC optimization at 10 Hz - computes acceleration setpoint"""

        # Only run MPC when in offboard mode
        if not self.offboard_mode:
            return

        # (1) kalau belum ada waypoint → KUNCI XY ke posisi SEKARANG (tiap kali)
        if not self.waypoint_received:
            self.target_position[0] = self.current_position[0]  # kunci X
            self.target_position[1] = self.current_position[1]  # kunci Y
            self.target_position[2] = -5.0 # tetap minta 5 m
            self.target_velocity = np.zeros(3)
            self.xy_locked = True
            self.get_logger().info(
                f'🎯 XY locked to current: X={self.target_position[0]:.2f}, '
                f'Y={self.target_position[1]:.2f}, Z={self.target_position[2]:.2f}'
            )

        # Current state: [x, y, z, vx, vy, vz]
        current_state = np.concatenate([self.current_position, self.current_velocity])

        # Reference state: [x_ref, y_ref, z_ref, vx_ref, vy_ref, vz_ref]
        reference_state = np.concatenate([self.target_position, self.target_velocity])

        # ============================================================
        # MPC OPTIMIZATION - Compute optimal acceleration
        # ============================================================
        acceleration = self.mpc.compute_control(current_state, reference_state)

        # (2) PAKSA XY MATI BIAR NGGAK MAJU
        # acceleration[0] = 0.0   # matikan kontrol X
        # acceleration[1] = 0.0   # matikan kontrol Y

        self.mpc_acceleration = acceleration

        # ============================================================
        # CONVERT ACCELERATION TO ATTITUDE + THRUST (PX4 Algorithm)
        # ============================================================
        roll, pitch, yaw, thrust, R_matrix = acceleration_to_attitude_thrust_px4(
            accel_ned=acceleration,
            yaw_desired=self.target_yaw,
            hover_thrust=0.4,  # X500 hover thrust
            gravity=9.81
        )

        # simpan buat control_loop()
        self.attitude_roll = roll
        self.attitude_pitch = pitch
        self.attitude_yaw = yaw
        self.attitude_thrust = thrust
        self.attitude_R_matrix = R_matrix

        # Log MPC output dengan info Z
        z_error = self.target_position[2] - self.current_position[2]
        self.get_logger().info(
            f'🎯 MPC OUTPUT:\n'
            f'   Position: Z={self.current_position[2]:.2f}m → Target={self.target_position[2]:.2f}m (error={z_error:.2f}m)\n'
            f'   Velocity: vz={self.current_velocity[2]:.2f} m/s\n'
            f'   Acceleration: az={acceleration[2]:.3f} m/s² ({"CLIMB" if acceleration[2] < 0 else "DESCEND/HOVER"})\n'
            f'   Thrust: {thrust:.3f} ({">"if thrust > 0.59 else "="if thrust == 0.59 else "<"} hover=0.59)\n'
            f'   Attitude: R={np.rad2deg(roll):.1f}° P={np.rad2deg(pitch):.1f}° Y={np.rad2deg(yaw):.1f}°'
        )

    def rotmat_nedfrd_to_quat_enuflu(self, R_ned_from_body_frd: np.ndarray):
        """Convert NED-FRD rotation matrix to ENU-FLU quaternion for MAVROS"""
        # Transform from NED->ENU and FRD->FLU
        T_ENU_NED = np.array([[0, 1, 0],
                              [1, 0, 0],
                              [0, 0,-1]], dtype=float)
        T_FRD_FLU = np.diag([1,-1,-1]).astype(float)
        
        # body(FLU) -> ENU
        R_enu_from_body_flu = T_ENU_NED @ R_ned_from_body_frd @ T_FRD_FLU
        
        # rotation_matrix_to_quaternion returns [w,x,y,z]
        wxyz = self.rotation_matrix_to_quaternion(R=R_enu_from_body_flu)
        # geometry_msgs/Quaternion needs [x,y,z,w]
        return np.array([wxyz[1], wxyz[2], wxyz[3], wxyz[0]], dtype=float)

    
    # def control_loop(self):
    #     """Control output at 50 Hz - publishes attitude setpoint from MPC"""
        
    #     # ============================================================
    #     # PUBLISH ATTITUDE SETPOINT TO MAVROS
    #     # ============================================================
    #     attitude_msg = AttitudeTarget()
    #     attitude_msg.header = Header()
    #     attitude_msg.header.stamp = self.get_clock().now().to_msg()
    #     attitude_msg.header.frame_id = "base_link"
        
    #     # Type mask: 0 means use all fields (thrust, orientation, rates)
    #     attitude_msg.type_mask = 0
        
    #     # Convert rotation matrix to quaternion for MAVROS
    #     q = self.rotation_matrix_to_quaternion(self.attitude_R_matrix)
    #     attitude_msg.orientation.w = float(q[0])
    #     attitude_msg.orientation.x = float(q[1])
    #     attitude_msg.orientation.y = float(q[2])
    #     attitude_msg.orientation.z = float(q[3])
        
    #     # Thrust (0-1, where 0.5 is typically hover)
    #     attitude_msg.thrust = float(self.attitude_thrust)
        
    #     # Body rates (zero for attitude control)
    #     attitude_msg.body_rate.x = 0.0
    #     attitude_msg.body_rate.y = 0.0
    #     attitude_msg.body_rate.z = 0.0
        
    #     # Publish attitude setpoint
    #     self.attitude_pub.publish(attitude_msg)
        
    #     # Debug logging (every 0.5s at 50Hz)
    #     if self.setpoint_counter % 25 == 0:
    #         z_current = self.current_position[2]
    #         z_target = self.target_position[2]
    #         z_error = z_target - z_current
    #         self.get_logger().info(
    #             f'📤 Attitude Setpoint: Roll={np.rad2deg(self.attitude_roll):.1f}° '
    #             f'Pitch={np.rad2deg(self.attitude_pitch):.1f}° '
    #             f'Yaw={np.rad2deg(self.attitude_yaw):.1f}° '
    #             f'Thrust={self.attitude_thrust:.3f} | '
    #             f'Z: {z_current:.2f}→{z_target:.2f} (Δ={z_error:.2f}m)'
    #         )
        
    #     self.setpoint_counter += 1
    
    def control_loop(self):
        """Control output at 50 Hz - publishes attitude setpoint from MPC"""
        
        att = AttitudeTarget()
        att.header.stamp = self.get_clock().now().to_msg()
        att.header.frame_id = "map"

        # Abaikan body rates, kita cuma kirim orientation + thrust
        att.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE |
            AttitudeTarget.IGNORE_PITCH_RATE |
            AttitudeTarget.IGNORE_YAW_RATE
        )

        # Konversi R (body->NED) ke quat ENU/FLU
        q_xyzw = self.rotmat_nedfrd_to_quat_enuflu(self.attitude_R_matrix)
        att.orientation.x, att.orientation.y, att.orientation.z, att.orientation.w = map(float, q_xyzw)

        # Thrust MAVROS 0..1 (positif ke ATAS)
        att.thrust = float(np.clip(self.attitude_thrust, 0.0, 1.0))

        # Body rates = 0
        att.body_rate.x = att.body_rate.y = att.body_rate.z = 0.0

        self.attitude_pub.publish(att)

        if self.setpoint_counter % 25 == 0:
            self.get_logger().info(
                f'📤 AttitudeTarget ENU: thrust={att.thrust:.3f} q=[{att.orientation.x:.3f},'
                f'{att.orientation.y:.3f},{att.orientation.z:.3f},{att.orientation.w:.3f}]'
            )
        self.setpoint_counter += 1

    
    def state_machine_callback(self):
        """State machine for offboard mode activation and waypoint following"""
        
        if self.current_state.mode != "OFFBOARD" and self.setpoint_counter > 10:
            self.set_offboard_mode()
        
        if not self.current_state.armed and self.current_state.mode == "OFFBOARD":
            self.arm()
        
        # Monitor waypoint following with MPC info
        if self.current_state.mode == "OFFBOARD" and self.current_state.armed:
            error = np.linalg.norm(self.current_position - self.target_position)
            
            if self.waypoint_received:
                # Show MPC control output
                acc_norm = np.linalg.norm(self.mpc_acceleration)
                vel_norm = np.linalg.norm(self.current_velocity)
                
                self.get_logger().info(
                    f"Pos: [{self.current_position[0]:.1f}, {self.current_position[1]:.1f}, {self.current_position[2]:.1f}] | "
                    f"Target: [{self.target_position[0]:.1f}, {self.target_position[1]:.1f}, {self.target_position[2]:.1f}] | "
                    f"Error: {error:.2f}m | Vel: {vel_norm:.2f} m/s | Acc: {acc_norm:.2f} m/s²"
                )
                
                # Check if waypoint reached
                if error < self.acceptance_radius:
                    self.get_logger().info(f"✓ Waypoint reached! (error: {error:.2f}m)")
            else:
                self.get_logger().info(f"Hovering at: [{self.current_position[0]:.1f}, "
                                      f"{self.current_position[1]:.1f}, {self.current_position[2]:.1f}] | "
                                      f"Waiting for waypoints...")
    
    def rotation_matrix_to_quaternion(self, R):
        """
        Convert rotation matrix to quaternion [w, x, y, z]
        
        Uses PX4's algorithm (Shepperd's method) for numerical stability
        
        Args:
            R: 3x3 rotation matrix
            
        Returns:
            q: Quaternion [w, x, y, z]
        """
        # Shepperd's method (used by PX4)
        # Choose largest diagonal element to avoid division by small numbers
        
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        
        if trace > 0.0:
            # w is largest
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            # x is largest
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            # y is largest
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            # z is largest
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        
        return np.array([w, x, y, z])
    
    def arm(self):
        """Send arm command via MAVROS"""
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')
        
        request = CommandBool.Request()
        request.value = True
        
        future = self.arming_client.call_async(request)
        future.add_done_callback(self.arm_callback)
    
    def arm_callback(self, future):
        """Callback for arm service"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('>>> ARM command sent via MAVROS <<<')
            else:
                self.get_logger().error('Failed to arm via MAVROS')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    
    def set_offboard_mode(self):
        """Send command to switch to offboard mode via MAVROS"""
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_mode service...')
        
        request = SetMode.Request()
        request.custom_mode = "OFFBOARD"
        
        future = self.set_mode_client.call_async(request)
        future.add_done_callback(self.set_mode_callback)
    
    def set_mode_callback(self, future):
        """Callback for set_mode service"""
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info('>>> OFFBOARD mode command sent via MAVROS <<<')
            else:
                self.get_logger().error('Failed to set OFFBOARD mode via MAVROS')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = MPCWaypointFollower()
    
    print("\n" + "="*60)
    print("MPC WAYPOINT FOLLOWER (MAVROS VERSION)")
    print("="*60)
    print("This node will:")
    print("  1. Arm and enter offboard mode via MAVROS")
    print("  2. Hover at default position (0, 0, -5)")
    print("  3. Follow waypoints from waypoint publisher")
    print("")
    print("To use:")
    print("  1. Make sure MAVROS is running and connected to PX4")
    print("  2. Wait for offboard mode to activate")
    print("  3. Send waypoints to /waypoint/target topic")
    print("  4. Drone will follow the waypoints!")
    print("="*60 + "\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()