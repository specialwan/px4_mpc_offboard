#!/usr/bin/env python3
############################################################################
#
#   MPC Position Controller with Waypoint Following
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
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import numpy as np
import quadprog  # For QP optimization
from scipy.spatial.transform import Rotation

# MAVROS messages (instead of px4_msgs)
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


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
        self.Q = np.diag([1.0, 1.0, 100.0,    # Position weights (Z >> X,Y to prioritize altitude)
                          1.0, 1.0, 30.0])     # Velocity damping (vz >> vx,vy)
        
        # Control effort weight - VERY LOW to allow aggressive Z control
        self.R = np.diag([0.01, 0.01, 0.01])       # Lower R_z = prioritize vertical acceleration
        
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


# def acceleration_to_attitude_thrust_px4(accel_ned, yaw_desired, hover_thrust=0.59, gravity=9.81):
#     """
#     Convert acceleration to attitude + thrust using SIMPLIFIED PX4-inspired algorithm
    
#     Key idea:
#     1. Total specific force = gravity + desired acceleration
#     2. Body Z points in direction of total specific force (thrust direction)
#     3. Thrust magnitude compensates gravity + provides desired acceleration
    
#     Args:
#         accel_ned: [ax, ay, az] in NED frame (m/sÂ²)
#         yaw_desired: Desired yaw (rad)
#         hover_thrust: Hover thrust (0-1), default 0.59
#         gravity: Gravity (m/sÂ²)
    
#     Returns:
#         roll, pitch, yaw, thrust_normalized
#     """
    
#     ax, ay, az = accel_ned[0], accel_ned[1], accel_ned[2]
    
#     # ========== SAFETY: LIMIT ACCELERATION ==========
#     # Prevent extreme accelerations that could cause instability
#     ax = np.clip(ax, -3.0, 3.0)
#     ay = np.clip(ay, -3.0, 3.0)
#     az = np.clip(az, -5.0, 5.0)  # Vertical: allow larger range
    
#     # ========== STEP 1: Compute desired specific force ==========
#     # Newton's law: F_thrust = m * (a_desired - g_vector)
#     # In NED frame:
#     #   - Gravity vector: [0, 0, +g] (down is positive)
#     #   - Desired acceleration: [ax, ay, az] (az < 0 = up)
#     #   - Thrust specific force: [ax, ay, az - g]
#     #
#     # Examples:
#     #   Hover (az=0):  [0, 0, 0-9.81] = [0, 0, -9.81] â†’ thrust=9.81 âœ…
#     #   Climb (az=-3): [0, 0, -3-9.81] = [0, 0, -12.81] â†’ thrust=12.81 âœ…
#     specific_force = np.array([ax, ay, az - gravity])
    
#     # Magnitude of specific force (this determines thrust)
#     thrust_magnitude = np.linalg.norm(specific_force)
    
#     # Safety: minimum thrust to overcome gravity
#     if thrust_magnitude < gravity:  # At least gravity to hover
#         thrust_magnitude = gravity
    
#     # Body Z-axis: direction of thrust (opposite to specific force in body frame)
#     # In NED frame, thrust points opposite to specific force
#     body_z = -specific_force / np.linalg.norm(specific_force)  # Use original magnitude for direction
    
#     # ========== STEP 2: Convert thrust magnitude to normalized value ==========
#     # Map physical thrust (m/sÂ²) to normalized thrust (0-1)
#     # At hover: thrust = g (9.81 m/sÂ²) â†’ normalized = hover_thrust (0.59)
#     # At climb: thrust > g â†’ normalized > hover_thrust
#     # Scaling: normalized_thrust = (thrust / g) * hover_thrust
#     thrust_normalized = (thrust_magnitude / gravity) * hover_thrust
    
#     # Clamp thrust to REASONABLE range (allow > hover for climb!)
#     thrust_normalized = np.clip(thrust_normalized, hover_thrust * 0.5, 1.0)  # Min 0.3, Max 1.0
    
#     # DEBUG: Print thrust calculation (uncomment to debug)
#     print(f"ðŸ” THRUST: az={az:.2f} â†’ specific_force=[{specific_force[0]:.1f},{specific_force[1]:.1f},{specific_force[2]:.1f}] â†’ mag={thrust_magnitude:.2f} m/sÂ² â†’ norm={thrust_normalized:.3f}")
    
#     # ========== STEP 3: Build rotation matrix from body Z and yaw ==========
#     # EXACT PX4 ALGORITHM from PositionControl::bodyzToAttitude()
    
#     # Normalize body_z (safety check)
#     if np.linalg.norm(body_z) < 1e-8:
#         # Zero vector, set safe level value
#         body_z = np.array([0.0, 0.0, 1.0])
    
#     body_z = body_z / np.linalg.norm(body_z)  # Normalize
    
#     # Vector of desired yaw direction in XY plane, rotated by PI/2
#     y_C = np.array([-np.sin(yaw_desired), np.cos(yaw_desired), 0.0])
    
#     # Desired body_x axis, orthogonal to body_z
#     body_x = np.cross(y_C, body_z)
    
#     # Keep nose to front while inverted upside down
#     if body_z[2] < 0.0:
#         body_x = -body_x
    
#     # Handle edge case: thrust in XY plane
#     if abs(body_z[2]) < 0.000001:
#         # Desired thrust is in XY plane, set X downside
#         body_x = np.array([0.0, 0.0, 1.0])
    
#     body_x = body_x / np.linalg.norm(body_x)  # Normalize
    
#     # Desired body_y axis
#     body_y = np.cross(body_z, body_x)
    
#     # Build rotation matrix R = [body_x | body_y | body_z] (column-wise)
#     R = np.column_stack([body_x, body_y, body_z])
    
#     # ========== STEP 4: Extract Euler angles from rotation matrix ==========
#     # Calculate euler angles (for logging only, must not be used for control)
#     roll = np.arctan2(R[2, 1], R[2, 2])
#     pitch = np.arcsin(-np.clip(R[2, 0], -1.0, 1.0))
#     yaw = np.arctan2(R[1, 0], R[0, 0])
    
#     # SAFETY: Limit attitude angles (prevent extreme tilts)
#     max_tilt = np.radians(35)  # 30 degrees max tilt
#     roll = np.clip(roll, -max_tilt, max_tilt)
#     pitch = np.clip(pitch, -max_tilt, max_tilt)
    
#     return roll, pitch, yaw, thrust_normalized, R

def acceleration_to_attitude_thrust_ENU(accel_enu, yaw_desired, hover_thrust=0.59, gravity=9.81):
    # ENU: X=East, Y=North, Z=Up
    ax, ay, az = accel_enu

    # Batasi akselerasi (safety)
    ax = np.clip(ax, -3.0, 3.0)
    ay = np.clip(ay, -3.0, 3.0)
    az = np.clip(az, -5.0, 5.0)

    # vektor gravitasi di ENU (turun = -Z)
    g_vec = np.array([0.0, 0.0, -gravity])

    # Total specific force (arah dorong) = g_vec - a_des (di dunia ENU)
    # (interpretasi: untuk menghasilkan a_des ke atas, thrust harus melawan g dan sisa percepatan)
    f_enu = g_vec - np.array([ax, ay, az])

    # Besar gaya â†’ untuk thrust normalisasi
    f_mag = np.linalg.norm(f_enu)
    if f_mag < gravity:
        f_mag = gravity

    # Body -Z (arah thrust) searah f_enu; maka body Z (sumbu kamera ke bawah di PX4) berlawanan
    body_z_enu = -f_enu / np.linalg.norm(f_enu)

    # Yaw yang diinginkan (di ENU)
    y_C = np.array([-np.sin(yaw_desired), np.cos(yaw_desired), 0.0])

    body_x_enu = np.cross(y_C, body_z_enu)
    if np.linalg.norm(body_x_enu) < 1e-6:
        body_x_enu = np.array([1.0, 0.0, 0.0])
    body_x_enu /= np.linalg.norm(body_x_enu)

    body_y_enu = np.cross(body_z_enu, body_x_enu)

    # Rotasi kolom [x|y|z] dalam ENU
    R = np.column_stack([body_x_enu, body_y_enu, body_z_enu])

    # Euler utk logging (bukan kontrol)
    roll  = np.arctan2(R[2,1], R[2,2])
    pitch = np.arcsin(-np.clip(R[2,0], -1.0, 1.0))
    yaw   = np.arctan2(R[1,0], R[0,0])

    max_tilt = np.radians(35)
    roll  = np.clip(roll,  -max_tilt, max_tilt)
    pitch = np.clip(pitch, -max_tilt, max_tilt)

    # Thrust 0..1 (MAVROS expects UP positive; di AttitudeTarget thrust=0..1)
    thrust_norm = (f_mag / gravity) * hover_thrust
    thrust_norm = np.clip(thrust_norm, hover_thrust*0.5, 1.0)

    return roll, pitch, yaw, thrust_norm, R



class MPCWaypointFollower(Node):
    """
    MPC-based waypoint follower that receives waypoints from custom publisher
    """

    def __init__(self):
        super().__init__('mpc_waypoint_follower_mavros')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,  # Match MAVROS (not TRANSIENT_LOCAL)
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # =================================================================
        # MPC CONTROLLER
        # =================================================================
        
        self.mpc = MPCPositionController(dt=0.1, Np=6, Nc=3)
        self.get_logger().info('MPC Controller initialized: dt=0.1s (10 Hz), Np=6, Nc=3')

        # =================================================================
        # SUBSCRIBERS (MAVROS)
        # =================================================================
        
        # Vehicle state (armed, mode, etc.)
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            qos_profile
        )
        
        # Local position (NED frame)
        self.position_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.position_callback,
            qos_profile
        )
        
        # Local velocity (ENU frame)
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self.velocity_callback,
            qos_profile
        )
        
        # GPS fix type (untuk cek apakah GPS ready)
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            qos_profile
        )
        
        # Waypoints from publisher
        self.waypoint_sub = self.create_subscription(
            PoseStamped,
            '/waypoint/target',
            self.waypoint_callback,
            10
        )

        # =================================================================
        # PUBLISHERS (MAVROS)
        # =================================================================
        
        # Attitude setpoint (roll, pitch, yaw, thrust)
        self.attitude_pub = self.create_publisher(
            AttitudeTarget,
            '/mavros/setpoint_raw/attitude',
            qos_profile
        )

        # =================================================================
        # SERVICE CLIENTS (MAVROS)
        # =================================================================
        
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Wait for services
        self.get_logger().info('Waiting for MAVROS services...')
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('  Waiting for arming service...')
        
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('  Waiting for set_mode service...')
        
        self.get_logger().info('âœ“ MAVROS services ready')

        # =================================================================
        # TIMERS
        # =================================================================
        
        # MPC optimization at 10 Hz
        self.mpc_timer = self.create_timer(0.1, self.mpc_callback)
        
        # Control output at 50 Hz
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
        # State machine at 1 Hz
        self.state_machine_timer = self.create_timer(1.0, self.state_machine_callback)
        
        # =================================================================
        # STATE VARIABLES
        # =================================================================
        
        self.armed = False
        self.offboard_mode = False
        self.state = "INIT"
        self.offboard_setpoint_counter = 0
        
        # Position estimate validity flag
        self.position_valid = False
        self.gps_fix = 0  # 0=no fix, 2=2D, 3=3D
        
        # Current state [x, y, z, vx, vy, vz]
        self.current_position = np.zeros(3)
        self.current_velocity = np.zeros(3)
        self.current_yaw = 0.0
        
        # Target from waypoint publisher (ENU frame - MAVROS uses ENU!)
        self.target_position = np.array([0.0, 0.0, 5.0])  # Default hover at 5m altitude
        self.target_velocity = np.zeros(3)  # Target velocity (usually zero for waypoints)
        self.target_yaw = 0.0
        self.waypoint_received = False
        
        # MPC computed acceleration (from optimization)
        self.desired_acceleration = np.zeros(3)  # [ax, ay, az] in NED frame
        self.mpc_acceleration = np.zeros(3)  # Latest MPC output
        
        # Attitude setpoint (computed from acceleration)
        self.attitude_roll = 0.0
        self.attitude_pitch = 0.0
        self.attitude_yaw = 0.0
        self.attitude_thrust = 0.6  # Hover thrust (normalized 0-1 for MAVROS)
        
        # Control parameters
        self.g = 9.81  # Gravity (m/s^2)
        self.hover_thrust = 0.6  # Hover thrust (0-1 for MAVROS)
        
        # Waypoint tracking
        self.last_waypoint_position = np.array([0.0, 0.0, 5.0])  # Track for duplicate detection
        self.acceptance_radius = 1.0  # Waypoint reached threshold (meters)
        
        self.get_logger().info('='*60)
        self.get_logger().info('MPC WAYPOINT FOLLOWER - MAVROS VERSION')
        self.get_logger().info('For Pixhawk 1 (FMUv2) without XRCE-DDS')
        self.get_logger().info('='*60)
        self.get_logger().info('Waiting for MAVROS connection...')
        self.get_logger().info('Waiting for waypoints from publisher...')

    # =================================================================
    # CALLBACKS - MAVROS SUBSCRIBERS
    # =================================================================
    
    def state_callback(self, msg):
        """MAVROS State callback"""
        self.armed = msg.armed
        self.offboard_mode = (msg.mode == "OFFBOARD")
        
    def position_callback(self, msg):
        """MAVROS Position callback (PoseStamped)"""
        # MAVROS local_position/pose is already in local frame (consistent with PX4)
        # No conversion needed
        # self.current_position = np.array([
        #     msg.pose.position.x,
        #     msg.pose.position.y,
        #     msg.pose.position.z
        # ])
        
        self.current_position = np.array([
            0.0,
            0.0,
            msg.pose.position.z
        ])
        
        # Extract yaw from quaternion
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        # Mark position as valid if we're receiving data
        # Additional check: position should be reasonable (not NaN or huge values)
        if not np.any(np.isnan(self.current_position)) and np.linalg.norm(self.current_position) < 1000.0:
            self.position_valid = True
        else:
            self.position_valid = False
        
    def velocity_callback(self, msg):
        """MAVROS Velocity callback (TwistStamped)"""
        # MAVROS local_position/velocity_local is already in local frame
        # No conversion needed
        self.current_velocity = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z
        ])
    
    def gps_callback(self, msg):
        """GPS Fix callback to monitor GPS status"""
        # NavSatStatus.status values:
        # -1 = NO_FIX
        #  0 = FIX (unaugmented)
        #  1 = SBAS_FIX
        #  2 = GBAS_FIX
        # We need at least a 3D fix (status >= 0)
        self.gps_fix = msg.status.status
    
    def waypoint_callback(self, msg):
        """Receive waypoint from waypoint publisher (PoseStamped)"""
        # Extract position
        new_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        
        # Check if this is a new waypoint (not duplicate)
        if not hasattr(self, 'last_waypoint_position'):
            self.last_waypoint_position = np.array([0.0, 0.0, 5.0])
        
        if np.linalg.norm(new_position - self.last_waypoint_position) > 0.1:
            # Update target directly
            self.target_position = new_position.copy()
            self.last_waypoint_position = new_position.copy()
            
            # Extract yaw from quaternion
            qz = msg.pose.orientation.z
            qw = msg.pose.orientation.w
            self.target_yaw = 2.0 * np.arctan2(qz, qw)
            
            if not self.waypoint_received:
                self.waypoint_received = True
            
            self.get_logger().info(
                f'ðŸ“ New waypoint: [{self.target_position[0]:.1f}, '
                f'{self.target_position[1]:.1f}, {self.target_position[2]:.1f}] '
                f'yaw={np.degrees(self.target_yaw):.0f}Â°'
            )
    
    # =====================================================================
    # MPC OPTIMIZATION (10 Hz)
    # =====================================================================
    
    def mpc_callback(self):
        """MPC optimization at 10 Hz - computes acceleration setpoint"""
        
        # Only run MPC when in offboard mode
        if not self.offboard_mode:
            return
        
        # IMPORTANT: If no waypoint received yet, hover at current XY position at 5m altitude
        # Lock the target when first entering offboard mode!
        if not self.waypoint_received:
            # Only set target ONCE when we first enter offboard mode
            # Check if target is still at default (0, 0, 5.0)
            if np.allclose(self.target_position, [0.0, 0.0, 5.0]):
                # Lock target to current XY, but Z = 5m (ENU: positive = up)
                self.target_position[0] = self.current_position[0]  # Lock X
                self.target_position[1] = self.current_position[1]  # Lock Y
                self.target_position[2] = 5.0  # Target altitude (5m up in ENU)
                self.get_logger().info(
                    f'ðŸŽ¯ Target locked: [{self.target_position[0]:.2f}, '
                    f'{self.target_position[1]:.2f}, {self.target_position[2]:.2f}] (ENU frame)'
                )
            
            # CRITICAL: Zero velocity target for stable hover!
            self.target_velocity = np.zeros(3)
        
        # Current state: [x, y, z, vx, vy, vz]
        current_state = np.concatenate([self.current_position, self.current_velocity])
        
        # Reference state: [x_ref, y_ref, z_ref, vx_ref, vy_ref, vz_ref]
        # Target velocity is zero for waypoint hovering
        reference_state = np.concatenate([self.target_position, self.target_velocity])
        
        # ============================================================
        # MPC OPTIMIZATION - Compute optimal acceleration
        # ============================================================
        acceleration = self.mpc.compute_control(current_state, reference_state)
        
        self.mpc_acceleration = acceleration
        
        # ============================================================
        # CONVERT ACCELERATION TO ATTITUDE + THRUST (PX4 Algorithm)
        # ============================================================
        # IMPORTANT: Convert ENU acceleration to NED for PX4 algorithm
        # ENU: X=East, Y=North, Z=Up
        # NED: X=North, Y=East, Z=Down
        # accel_ned = np.array([
        #     acceleration[1],   # ENU Y (North) â†’ NED X (North)
        #     acceleration[0],   # ENU X (East) â†’ NED Y (East)
        #     -acceleration[2]   # ENU Z (Up) â†’ NED -Z (Down)
        # ])
        
        # roll, pitch, yaw, thrust, R_matrix = acceleration_to_attitude_thrust_px4(
        #     accel_ned=accel_ned,
        #     yaw_desired=self.target_yaw,
        #     hover_thrust=0.4,  # X500 hover thrust
        #     gravity=9.81
        # )

        roll, pitch, yaw, thrust, R_matrix = acceleration_to_attitude_thrust_ENU(
            accel_enu=acceleration,
            yaw_desired=self.target_yaw,
            hover_thrust=0.4,  # X500 hover thrust
            gravity=9.81
        )

        # Store attitude setpoint
        self.attitude_roll = roll
        self.attitude_pitch = pitch
        self.attitude_yaw = yaw
        self.attitude_thrust = thrust
        self.attitude_R_matrix = R_matrix  # Store rotation matrix for quaternion conversion
        
        # Log MPC output with detailed altitude tracking
        pos_error = np.linalg.norm(self.target_position - self.current_position)
        z_error = self.target_position[2] - self.current_position[2]  # Z error
        vel_norm = np.linalg.norm(self.current_velocity)
        
        # COMPREHENSIVE DEBUG LOGGING
        self.get_logger().info(
            f'ðŸŽ¯ MPC OUTPUT:\n'
            f'   Position: Z={self.current_position[2]:.2f}m â†’ Target={self.target_position[2]:.2f}m (error={z_error:.2f}m)\n'
            f'   Velocity: vz={self.current_velocity[2]:.2f} m/s\n'
            f'   Acceleration: az={acceleration[2]:.3f} m/sÂ² ({"CLIMB" if acceleration[2] < 0 else "DESCEND/HOVER"})\n'
            f'   Thrust: {thrust:.3f} ({">"if thrust > 0.59 else "="if thrust == 0.59 else "<"} hover=0.59)\n'
            f'   Attitude: R={np.rad2deg(roll):.1f}Â° P={np.rad2deg(pitch):.1f}Â° Y={np.rad2deg(yaw):.1f}Â°'
        )
    
    # =====================================================================
    # CONTROL LOOP (50 Hz)
    # =====================================================================
    
    def control_loop(self):
        """Control loop: publish attitude setpoint at 50 Hz (MAVROS)"""
        
        # Create AttitudeTarget message (MAVROS)
        attitude_msg = AttitudeTarget()
        attitude_msg.header = Header()
        attitude_msg.header.stamp = self.get_clock().now().to_msg()
        attitude_msg.header.frame_id = "base_link"
        
        # Type mask: use attitude + thrust (ignore rates)
        # Bit values from mavros_msgs/AttitudeTarget.msg:
        # IGNORE_ROLL_RATE = 1, IGNORE_PITCH_RATE = 2, IGNORE_YAW_RATE = 4
        # We want: attitude quaternion + thrust, ignore body rates
        attitude_msg.type_mask = 7  # 1 + 2 + 4 = ignore all rates, use attitude + thrust
        
        # Convert Euler angles to quaternion
        r = Rotation.from_euler('xyz', [self.attitude_roll, self.attitude_pitch, self.attitude_yaw])
        q = r.as_quat()  # [x, y, z, w]
        
        attitude_msg.orientation.w = float(q[3])
        attitude_msg.orientation.x = float(q[0])
        attitude_msg.orientation.y = float(q[1])
        attitude_msg.orientation.z = float(q[2])
        
        # Body rates (set to zero since we're using type_mask to ignore them)
        attitude_msg.body_rate.x = 0.0
        attitude_msg.body_rate.y = 0.0
        attitude_msg.body_rate.z = 0.0
        
        # Thrust (0-1, positive for upward in MAVROS)
        attitude_msg.thrust = float(self.attitude_thrust)
        
        # Publish attitude setpoint
        self.attitude_pub.publish(attitude_msg)
        
        # Increment counter
        self.offboard_setpoint_counter += 1
        
        # Debug: Log publishing rate (every 2 seconds)
        if self.offboard_setpoint_counter % 100 == 0:
            self.get_logger().info(
                f"📡 Setpoint stream: {self.offboard_setpoint_counter} samples sent | "
                f"Thrust={self.attitude_thrust:.3f} | State={self.state}"
            )
    
    def state_machine_callback(self):
        """State machine for offboard mode activation and waypoint following"""
        
        if self.state == "INIT":
            # Tunggu setpoint streaming dulu sebelum ARM
            # PX4 butuh setpoint sudah streaming SEBELUM offboard mode
            if self.offboard_setpoint_counter > 100:  # 100 samples @ 50Hz = 2 detik
                # Check GPS fix and position validity before allowing ARM
                if self.gps_fix >= 0 and self.position_valid:
                    self.state = "READY_TO_ARM"
                    self.get_logger().info("="*60)
                    self.get_logger().info(f"✅ Setpoint streaming ready ({self.offboard_setpoint_counter} samples @ 50Hz)")
                    self.get_logger().info(f"✅ GPS Fix: {self.gps_fix} | Position Valid: {self.position_valid}")
                    self.get_logger().info("Proceeding to ARM...")
                    self.get_logger().info("="*60)
                    self.get_logger().info(f"State: INIT -> READY_TO_ARM")
                else:
                    # Log GPS/position status every 1 second
                    if self.offboard_setpoint_counter % 50 == 0:
                        self.get_logger().warn(f"⚠️  Waiting for GPS fix and position estimate...")
                        self.get_logger().warn(f"   GPS Fix: {self.gps_fix} (need >= 0) | Position Valid: {self.position_valid}")
            else:
                # Log progress every 1 second
                if self.offboard_setpoint_counter % 50 == 0:
                    self.get_logger().info(f"⏳ Streaming setpoints... ({self.offboard_setpoint_counter}/100 @ 50Hz)")
                    self.get_logger().info(f"   GPS Fix: {self.gps_fix} | Position Valid: {self.position_valid}")
        
        elif self.state == "READY_TO_ARM":
            # Kirim ARM command
            self.arm()
            self.state = "ARMING"
            self.get_logger().info("📤 ARM command sent")
            self.get_logger().info("State: READY_TO_ARM -> ARMING")
        
        elif self.state == "ARMING":
            # Tunggu sampai benar-benar armed
            if self.armed:
                self.state = "ARMED_READY_OFFBOARD"
                self.get_logger().info("✅ Vehicle ARMED")
                self.get_logger().info("State: ARMING -> ARMED_READY_OFFBOARD")
            else:
                # Retry ARM command every 0.5 detik
                if self.offboard_setpoint_counter % 25 == 0:
                    self.arm()
                    self.get_logger().info("⚠️  Retrying ARM command...")
        
        elif self.state == "ARMED_READY_OFFBOARD":
            # Sudah armed, sekarang kirim offboard mode command
            self.set_offboard_mode()
            self.state = "ACTIVATING_OFFBOARD"
            self.get_logger().info("📤 OFFBOARD mode command sent")
            self.get_logger().info("State: ARMED_READY_OFFBOARD -> ACTIVATING_OFFBOARD")
        
        elif self.state == "ACTIVATING_OFFBOARD":
            if self.offboard_mode:
                # Success! Offboard mode activated
                self.state = "TAKING_OFF"
                self.get_logger().info("State: ACTIVATING_OFFBOARD -> TAKING_OFF")
                self.get_logger().info(">>> Waiting for drone to lift off with attitude control <<<")
            else:
                # Retry set_offboard_mode every 0.5 seconds
                if self.offboard_setpoint_counter % 25 == 0:
                    self.set_offboard_mode()
                    self.get_logger().info("⚠️  Retrying set_offboard_mode...")
        
        elif self.state == "TAKING_OFF":
            # Wait for altitude increasing (Z positive in ENU)
            if self.current_position[2] > 0.5:  # Above 0.5m
                self.state = "FOLLOWING_WAYPOINTS"
                self.get_logger().info("State: TAKING_OFF -> FOLLOWING_WAYPOINTS")
                self.get_logger().info("="*60)
                self.get_logger().info("âœ… AIRBORNE! MPC Position Control Active!")
                self.get_logger().info("Ready to follow waypoints from publisher")
                self.get_logger().info("="*60)
            else:
                # Still on ground, keep publishing setpoints
                if self.offboard_setpoint_counter % 50 == 0:  # Every 1 second
                    z_current = self.current_position[2]
                    self.get_logger().info(f"Taking off... Z={z_current:.2f}m (target > 0.5m) | Thrust={self.attitude_thrust:.3f}")
        
        elif self.state == "FOLLOWING_WAYPOINTS":
            # Auto-recover offboard mode if lost
            if not self.offboard_mode and self.armed:
                self.get_logger().warn("âš ï¸  Offboard mode lost! Re-activating...")
                self.set_offboard_mode()
                return
            
            # Monitor waypoint following
            error = np.linalg.norm(self.current_position - self.target_position)
            
            if self.waypoint_received:
                # Show MPC control output
                acc_norm = np.linalg.norm(self.mpc_acceleration)
                vel_norm = np.linalg.norm(self.current_velocity)
                
                self.get_logger().info(
                    f"Pos: [{self.current_position[0]:.1f}, {self.current_position[1]:.1f}, {self.current_position[2]:.1f}] | "
                    f"Target: [{self.target_position[0]:.1f}, {self.target_position[1]:.1f}, {self.target_position[2]:.1f}] | "
                    f"Error: {error:.2f}m | Vel: {vel_norm:.2f} m/s | Acc: {acc_norm:.2f} m/sÂ²"
                )
                
                # Check if waypoint reached
                if error < self.acceptance_radius:
                    self.get_logger().info(f"âœ“ Waypoint reached! (error: {error:.2f}m)")
            else:
                self.get_logger().info(
                    f"Hovering at: [{self.current_position[0]:.1f}, {self.current_position[1]:.1f}, "
                    f"{self.current_position[2]:.1f}] | Waiting for waypoints..."
                )
    
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
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles (roll, pitch, yaw) to quaternion [w, x, y, z]
        
        Args:
            roll: Roll angle in radians
            pitch: Pitch angle in radians
            yaw: Yaw angle in radians
            
        Returns:
            q: Quaternion [w, x, y, z]
        """
        # Compute half angles
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        
        # Compute quaternion (ZYX convention)
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return np.array([w, x, y, z])
    
    # =====================================================================
    # COMMAND FUNCTIONS (MAVROS)
    # =====================================================================
    
    def arm(self):
        """Send arm command via MAVROS service"""
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        self.get_logger().info('>>> ARM command sent (MAVROS) <<<')
    
    def set_offboard_mode(self):
        """Switch to OFFBOARD mode via MAVROS service"""
        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"
        future = self.set_mode_client.call_async(req)
        self.get_logger().info('>>> OFFBOARD mode command sent (MAVROS) <<<')
        self.get_logger().info('>>> TAKEOFF command sent (2m) to clear ground <<<')


def main(args=None):
    rclpy.init(args=args)
    
    node = MPCWaypointFollower()
    
    print("\n" + "="*60)
    print("MPC WAYPOINT FOLLOWER")
    print("="*60)
    print("This node will:")
    print("  1. Arm and enter offboard mode")
    print("  2. Hover at default position (0, 0, -5)")
    print("  3. Follow waypoints from QGroundControl mission")
    print("")
    print("To use:")
    print("  1. Wait for offboard mode to activate")
    print("  2. In QGC, create a mission with waypoints")
    print("  3. Upload and start the mission")
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