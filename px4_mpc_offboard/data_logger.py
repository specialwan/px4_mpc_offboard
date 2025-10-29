#!/usr/bin/env python3
"""
Data Logger Node for MPC/PX4 Waypoint Following  (CSV streaming + graceful shutdown)

Fitur:
- Logging → CSV (append, ringan) dengan urutan kolom tetap
- Waktu 'time' konsisten relatif t0 (sampel pertama), semua batch
- Graceful shutdown (Ctrl+C / SIGTERM): stop timer → flush CSV → build Excel → shutdown
- QoS depth lebih besar, steady/monotonic clock, waypoint_fresh = 0/1

Catatan:
- File aktif saat runtime: flight_log_YYYYMMDD_HHMMSS.csv
- Saat node berhenti, otomatis dibuat flight_log_YYYYMMDD_HHMMSS.xlsx (Summary + sheets)
"""

import os
import csv
import signal
import atexit
import threading
from pathlib import Path
from datetime import datetime

import numpy as np
import pandas as pd

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.clock import Clock, ClockType

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import (
    VehicleLocalPosition,
    VehicleAttitude,
    TrajectorySetpoint,
    VehicleAttitudeSetpoint,
)


class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')

        # QoS yang lebih tahan saat IO sibuk
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # =======================
        # PARAMETERS
        # =======================
        home_dir = os.path.expanduser('~')
        default_log_dir = os.path.join(home_dir, 'flight_logs')

        self.declare_parameter('output_dir', default_log_dir)
        self.declare_parameter('log_rate_hz', 20.0)         # Hz
        self.declare_parameter('auto_save_interval', 30.0)  # detik

        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.log_rate = self.get_parameter('log_rate_hz').get_parameter_value().double_value
        self.auto_save_interval = self.get_parameter('auto_save_interval').get_parameter_value().double_value

        os.makedirs(self.output_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        base = f'flight_log_{timestamp}'
        self.csv_filename = os.path.join(self.output_dir, base + '.csv')     # file utama (runtime)
        self.excel_filename = os.path.join(self.output_dir, base + '.xlsx')  # dibuat saat shutdown

        # Clock monotonic (tidak ikut / use_sim_time, tidak mundur)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)

        # =======================
        # SUBSCRIBERS
        # =======================
        self.position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.position_callback, qos_profile
        )
        self.attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self.attitude_callback, qos_profile
        )
        self.waypoint_sub = self.create_subscription(
            PoseStamped, '/waypoint/target',
            self.waypoint_callback, 10
        )
        self.trajectory_sub = self.create_subscription(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint',
            self.trajectory_callback, qos_profile
        )
        self.attitude_setpoint_sub = self.create_subscription(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint',
            self.attitude_setpoint_callback, qos_profile
        )

        # =======================
        # DATA STORAGE
        # =======================
        self.data_buffer = []           # buffer kecil; akan di-flush ke CSV
        self._csv_header_written = False
        self._buf_flush_every = 200     # flush per 200 sampel (atur sesuai kebutuhan)

        # Kolom tetap (urutan fix) supaya append stabil
        self._columns = [
            'time',
            'pos_x','pos_y','pos_z',
            'vel_x','vel_y','vel_z','vel_mag',
            'roll_deg','pitch_deg','yaw_deg',
            'target_pos_x','target_pos_y','target_pos_z',
            'target_vel_x','target_vel_y','target_vel_z','target_vel_mag',
            'target_yaw_deg',
            'control_pos_x','control_pos_y','control_pos_z',
            'control_vel_x','control_vel_y','control_vel_z','control_vel_mag',
            'control_yaw_deg',
            'error_x','error_y','error_z','error_mag',
            'error_vel_x','error_vel_y','error_vel_z','error_vel_mag',
            'error_yaw_deg',
            'waypoint_fresh',
        ]

        # current state
        self.current_pos = np.zeros(3)
        self.current_vel = np.zeros(3)
        self.current_euler = np.zeros(3)  # roll, pitch, yaw [rad]

        # target / control
        self.target_pos = np.zeros(3)
        self.target_vel = np.zeros(3)
        self.target_yaw = 0.0
        self.last_waypoint_time = None

        self.control_pos = np.zeros(3)
        self.control_vel = np.zeros(3)
        self.control_yaw = 0.0

        # flags
        self.position_received = False
        self.attitude_received = False
        self.waypoint_received = False
        self.trajectory_received = False

        # controller info (tidak mengganti nama file agar aman)
        self.controller_type = 'UNKNOWN'

        # Waktu awal (untuk normalisasi time relatif t0)
        self._t0 = None

        # Graceful shutdown state
        self._shutting_down = False
        self._io_lock = threading.Lock()

        # =======================
        # TIMERS
        # =======================
        self.log_timer = self.create_timer(1.0 / self.log_rate, self.log_data)
        self.save_timer = self.create_timer(self.auto_save_interval, self.auto_flush_csv)

        # Info
        self.get_logger().info('=' * 70)
        self.get_logger().info('📊 DATA LOGGER (CSV streaming + graceful shutdown) STARTED')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Output dir : {self.output_dir}')
        self.get_logger().info(f'CSV file   : {self.csv_filename}')
        self.get_logger().info(f'Excel file : {self.excel_filename}  (dibuat saat shutdown)')
        self.get_logger().info(f'Log rate   : {self.log_rate} Hz')
        self.get_logger().info(f'Flush intv : {self.auto_save_interval} s')
        self.get_logger().info('=' * 70)
        self.get_logger().info('Waiting for data...')

        # Signal handlers (Ctrl+C / SIGTERM) + atexit fallback
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        atexit.register(self._atexit_handler)

    # =======================
    # CALLBACKS
    # =======================
    def position_callback(self, msg: VehicleLocalPosition):
        self.current_pos = np.array([msg.x, msg.y, msg.z])
        self.current_vel = np.array([msg.vx, msg.vy, msg.vz])
        if not self.position_received:
            self.position_received = True
            self.get_logger().info('✅ Position data received')

    def attitude_callback(self, msg: VehicleAttitude):
        # px4_msgs/VehicleAttitude: q[0]=w, q[1]=x, q[2]=y, q[3]=z
        w, x, y, z = msg.q[0], msg.q[1], msg.q[2], msg.q[3]

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        pitch = np.sign(sinp) * (np.pi / 2) if abs(sinp) >= 1 else np.arcsin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        self.current_euler = np.array([roll, pitch, yaw])

        if not self.attitude_received:
            self.attitude_received = True
            self.get_logger().info('✅ Attitude data received')

    def waypoint_callback(self, msg: PoseStamped):
        self.target_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        # planar yaw dari z,w
        self.target_yaw = 2.0 * np.arctan2(msg.pose.orientation.z, msg.pose.orientation.w)
        self.target_vel = np.zeros(3)
        self.last_waypoint_time = self.steady_clock.now()

        if not self.waypoint_received:
            self.waypoint_received = True
            self.get_logger().info('✅ Waypoint data received')

    def trajectory_callback(self, msg: TrajectorySetpoint):
        self.control_pos = np.array([msg.position[0], msg.position[1], msg.position[2]])
        self.control_vel = np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]])
        self.control_yaw = msg.yaw

        if not self.trajectory_received:
            self.trajectory_received = True
            self.get_logger().info('✅ Trajectory setpoint data received (PX4 Bridge - PID)')

        # bisa dipakai untuk menandai jenis controller tanpa rename file
        self.controller_type = 'PID'

    def attitude_setpoint_callback(self, msg: VehicleAttitudeSetpoint):
        # q_d: [w, x, y, z]
        w, x, y, z = msg.q_d[0], msg.q_d[1], msg.q_d[2], msg.q_d[3]
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        self.control_yaw = np.arctan2(siny_cosp, cosy_cosp)

        self.controller_type = 'MPC'
        if not self.trajectory_received:
            self.trajectory_received = True
            self.get_logger().info('✅ Attitude setpoint data received (MPC control)')

    # =======================
    # LOGGING
    # =======================
    def log_data(self):
        if not self.position_received:
            return

        now = self.steady_clock.now()
        timestamp_sec = now.nanoseconds / 1e9
        if self._t0 is None:
            self._t0 = timestamp_sec

        waypoint_fresh = 0
        if self.last_waypoint_time is not None:
            dt_wp = (now - self.last_waypoint_time).nanoseconds / 1e9
            waypoint_fresh = 1 if dt_wp < 0.5 else 0

        pos_error = self.current_pos - self.target_pos
        vel_error = self.current_vel - self.target_vel

        yaw_error = self.current_euler[2] - self.target_yaw
        yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))

        entry = {
            'time': timestamp_sec,

            'pos_x': self.current_pos[0],
            'pos_y': self.current_pos[1],
            'pos_z': self.current_pos[2],

            'vel_x': self.current_vel[0],
            'vel_y': self.current_vel[1],
            'vel_z': self.current_vel[2],
            'vel_mag': float(np.linalg.norm(self.current_vel)),

            'roll_deg': float(np.degrees(self.current_euler[0])),
            'pitch_deg': float(np.degrees(self.current_euler[1])),
            'yaw_deg': float(np.degrees(self.current_euler[2])),

            'target_pos_x': self.target_pos[0],
            'target_pos_y': self.target_pos[1],
            'target_pos_z': self.target_pos[2],

            'target_vel_x': self.target_vel[0],
            'target_vel_y': self.target_vel[1],
            'target_vel_z': self.target_vel[2],
            'target_vel_mag': float(np.linalg.norm(self.target_vel)),

            'target_yaw_deg': float(np.degrees(self.target_yaw)),

            'control_pos_x': self.control_pos[0],
            'control_pos_y': self.control_pos[1],
            'control_pos_z': self.control_pos[2],

            'control_vel_x': self.control_vel[0],
            'control_vel_y': self.control_vel[1],
            'control_vel_z': self.control_vel[2],
            'control_vel_mag': float(np.linalg.norm(self.control_vel)),

            'control_yaw_deg': float(np.degrees(self.control_yaw)),

            'error_x': pos_error[0],
            'error_y': pos_error[1],
            'error_z': pos_error[2],
            'error_mag': float(np.linalg.norm(pos_error)),

            'error_vel_x': vel_error[0],
            'error_vel_y': vel_error[1],
            'error_vel_z': vel_error[2],
            'error_vel_mag': float(np.linalg.norm(vel_error)),

            'error_yaw_deg': float(np.degrees(yaw_error)),

            'waypoint_fresh': int(waypoint_fresh),
        }

        # Paksa urutan kolom konsisten (kolom tak ada → NaN)
        self.data_buffer.append(entry)

        # Flush periodik
        if len(self.data_buffer) >= self._buf_flush_every:
            self.flush_to_csv()

    def auto_flush_csv(self):
        self.flush_to_csv()

    def flush_to_csv(self):
        """Tulis buffer → CSV (append) dengan urutan kolom fix, normalisasi time konsisten."""
        if not self.data_buffer:
            return
        with self._io_lock:
            if not self.data_buffer:
                return
            try:
                df = pd.DataFrame(self.data_buffer, columns=self._columns)

                t0 = self._t0 if self._t0 is not None else df['time'].iloc[0]
                df['time'] = df['time'] - t0

                mode = 'a' if self._csv_header_written else 'w'
                header = not self._csv_header_written

                df.to_csv(self.csv_filename, mode=mode, header=header,
                          index=False, quoting=csv.QUOTE_NONNUMERIC)

                self._csv_header_written = True
                self.data_buffer.clear()
                self.get_logger().info(f'💾 Flushed → {Path(self.csv_filename).name}')
            except Exception as e:
                self.get_logger().error(f'❌ CSV flush failed: {e}')

    def save_to_excel(self):
        """Bangun Excel dari CSV (dipanggil saat shutdown)."""
        with self._io_lock:
            # Tulis sisa buffer (tanpa memanggil flush_to_csv lagi supaya lock tidak nested)
            if self.data_buffer:
                try:
                    df_buf = pd.DataFrame(self.data_buffer, columns=self._columns)
                    t0 = self._t0 if self._t0 is not None else df_buf['time'].iloc[0]
                    df_buf['time'] = df_buf['time'] - t0

                    mode = 'a' if self._csv_header_written else 'w'
                    header = not self._csv_header_written
                    df_buf.to_csv(self.csv_filename, mode=mode, header=header,
                                  index=False, quoting=csv.QUOTE_NONNUMERIC)
                    self._csv_header_written = True
                    self.data_buffer.clear()
                except Exception as e:
                    self.get_logger().error(f'❌ Final CSV write failed: {e}')

            if not os.path.exists(self.csv_filename):
                self.get_logger().warn('No CSV file to convert!')
                return

            try:
                df = pd.read_csv(self.csv_filename)

                with pd.ExcelWriter(self.excel_filename, engine='openpyxl') as writer:
                    df.to_excel(writer, sheet_name='Flight Data', index=False)

                    summary = pd.DataFrame({
                        'Metric': [
                            'Controller Type',
                            'Duration (s)',
                            'Samples',
                            'Avg Position Error (m)',
                            'Max Position Error (m)',
                            'Avg Velocity Error (m/s)',
                            'Max Velocity Error (m/s)',
                            'Avg Yaw Error (deg)',
                            'Max Yaw Error (deg)',
                        ],
                        'Value': [
                            self.controller_type,
                            float(df['time'].iloc[-1]) if len(df) else 0.0,
                            int(len(df)),
                            float(df['error_mag'].mean()) if 'error_mag' in df else np.nan,
                            float(df['error_mag'].max())  if 'error_mag' in df else np.nan,
                            float(df['error_vel_mag'].mean()) if 'error_vel_mag' in df else np.nan,
                            float(df['error_vel_mag'].max())  if 'error_vel_mag' in df else np.nan,
                            float(df['error_yaw_deg'].abs().mean()) if 'error_yaw_deg' in df else np.nan,
                            float(df['error_yaw_deg'].abs().max())  if 'error_yaw_deg' in df else np.nan,
                        ]
                    })
                    summary.to_excel(writer, sheet_name='Summary', index=False)

                    def write_if(cols, sheet):
                        cols = [c for c in cols if c in df.columns]
                        if cols:
                            df[cols].to_excel(writer, sheet_name=sheet, index=False)

                    write_if(
                        ['time','pos_x','pos_y','pos_z',
                         'target_pos_x','target_pos_y','target_pos_z',
                         'error_x','error_y','error_z','error_mag'],
                        'Position'
                    )
                    write_if(
                        ['time','vel_x','vel_y','vel_z','vel_mag',
                         'target_vel_x','target_vel_y','target_vel_z','target_vel_mag',
                         'error_vel_x','error_vel_y','error_vel_z','error_vel_mag'],
                        'Velocity'
                    )
                    write_if(
                        ['time','roll_deg','pitch_deg','yaw_deg',
                         'target_yaw_deg','error_yaw_deg'],
                        'Attitude'
                    )

                self.get_logger().info(f'✅ Excel built from CSV: {self.excel_filename}')

            except Exception as e:
                self.get_logger().error(f'❌ Failed to build Excel: {e}')

    # =======================
    # GRACEFUL SHUTDOWN
    # =======================
    def _signal_handler(self, signum, frame):
        if self._shutting_down:
            return
        self._shutting_down = True
        self.get_logger().info(f'🔻 Caught signal {signum}. Graceful shutdown...')
        try:
            if hasattr(self, 'log_timer'):
                self.log_timer.cancel()
            if hasattr(self, 'save_timer'):
                self.save_timer.cancel()
            self.save_to_excel()  # sudah lock-guarded & tulis sisa buffer
        finally:
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def _atexit_handler(self):
        if not self._shutting_down:
            self._shutting_down = True
            try:
                self.save_to_excel()
            except Exception:
                pass

    def destroy_node(self):
        self.get_logger().info('=' * 70)
        self.get_logger().info('Shutting down data logger...')
        # save_to_excel sudah dipanggil di signal/atexit; panggil lagi aman (idempotent)
        try:
            self.save_to_excel()
        except Exception:
            pass
        self.get_logger().info('Final CSV/Excel saved. Bye!')
        self.get_logger().info('=' * 70)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🔻 KeyboardInterrupt received.')
        try:
            node._signal_handler(signal.SIGINT, None)
        except Exception:
            pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
