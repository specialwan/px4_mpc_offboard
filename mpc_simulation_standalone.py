#!/usr/bin/env python3
"""
MPC Position Controller - Standalone Simulation
================================================

Program simulasi MPC tanpa ROS untuk mengukur waktu komputasi di Raspberry Pi.
Matrix dan parameter sama dengan implementasi ROS.

Features:
- 6-state model: [x, y, z, vx, vy, vz]
- 3 control inputs: [ax, ay, az]
- Quadratic Programming (QP) optimization
- Pengukuran waktu komputasi per iterasi
- Visualisasi trajectory dan performance

Author: Modified for Raspberry Pi computation benchmarking
Date: October 30, 2025
"""

import numpy as np
import quadprog
import time
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class SimulationConfig:
    """Konfigurasi simulasi"""
    # MPC parameters
    dt: float = 0.1          # Sample time (s) - 10 Hz
    Np: int = 6              # Prediction horizon
    Nc: int = 3              # Control horizon
    
    # Simulation parameters
    sim_duration: float = 30.0   # Durasi simulasi (detik)
    
    # Initial conditions
    x0: np.ndarray = None    # Initial state [x, y, z, vx, vy, vz]
    
    # Target/Reference
    x_ref: np.ndarray = None # Target state [x, y, z, vx, vy, vz]
    
    def __post_init__(self):
        if self.x0 is None:
            # Start at origin with zero velocity
            self.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        if self.x_ref is None:
            # Target: 10m forward, 5m right, 5m up, zero velocity
            self.x_ref = np.array([10.0, 5.0, 5.0, 0.0, 0.0, 0.0])


class MPCController:
    """
    Model Predictive Controller untuk position control
    
    State: [x, y, z, vx, vy, vz]
    Control: [ax, ay, az]
    """
    
    def __init__(self, dt=0.1, Np=6, Nc=3):
        """
        Initialize MPC controller
        
        Args:
            dt: Sample time (s)
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
        
        # Cost matrices
        self.Q = np.diag([1.0, 1.0, 100.0,    # Position weights (Z >> X,Y)
                          1.0, 1.0, 30.0])     # Velocity damping
        
        self.R = np.diag([0.05, 0.05, 0.05])       # Control effort weight
        
        self.R_delta = np.diag([0.3, 0.3, 0.2])    # Control rate weight
        
        # Build prediction matrices
        self._build_prediction_matrices()
        
        # Previous control for rate penalty
        self.u_prev = np.zeros(self.nu)
        
        # Max acceleration limits (m/s^2)
        self.a_max = 6.0
        
        # Statistics
        self.computation_times = []
        
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
        
        # H matrix for QP
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
            computation_time: Time taken for optimization (seconds)
        """
        
        # Start timing
        t_start = time.perf_counter()
        
        # Build reference trajectory
        r = np.tile(reference_state, self.Np)
        
        # Compute error prediction
        error_prediction = self.Phi @ current_state - r
        
        # Build extended previous control vector
        u_prev_extended = np.tile(self.u_prev, self.Nc)
        
        # Compute f vector
        f_tracking = self.Gamma.T @ self.Q_bar @ error_prediction
        f_rate = -self.R_delta_bar @ u_prev_extended
        f = f_tracking + f_rate
        
        # Inequality constraints: -a_max <= u <= a_max
        C = np.vstack([np.eye(self.Nc * self.nu),
                       -np.eye(self.Nc * self.nu)])
        b = np.hstack([np.full(self.Nc * self.nu, -self.a_max),
                       np.full(self.Nc * self.nu, -self.a_max)])
        
        # Solve QP
        try:
            u_opt = quadprog.solve_qp(self.H, -f, C.T, b, meq=0)[0]
            
            # Extract first control action
            control = u_opt[:self.nu]
            
            # Save for next iteration
            self.u_prev = control.copy()
            
            # Clip to limits
            control = np.clip(control, -self.a_max, self.a_max)
            
        except Exception as e:
            print(f"QP solver failed: {e}")
            control = np.zeros(self.nu)
        
        # End timing
        t_end = time.perf_counter()
        computation_time = t_end - t_start
        
        # Store computation time
        self.computation_times.append(computation_time)
        
        return control, computation_time


class DroneSimulator:
    """Simulator untuk dynamics drone"""
    
    def __init__(self, dt=0.1):
        self.dt = dt
        self.state = np.zeros(6)  # [x, y, z, vx, vy, vz]
        
        # System matrices (same as MPC)
        self.A = np.array([
            [1, 0, 0, dt, 0,  0],
            [0, 1, 0, 0,  dt, 0],
            [0, 0, 1, 0,  0,  dt],
            [0, 0, 0, 1,  0,  0],
            [0, 0, 0, 0,  1,  0],
            [0, 0, 0, 0,  0,  1],
        ])
        
        self.B = np.array([
            [0.5*dt**2, 0,         0        ],
            [0,         0.5*dt**2, 0        ],
            [0,         0,         0.5*dt**2],
            [dt,        0,         0        ],
            [0,         dt,        0        ],
            [0,         0,         dt       ],
        ])
    
    def reset(self, initial_state):
        """Reset simulator ke initial state"""
        self.state = initial_state.copy()
    
    def step(self, control):
        """
        Simulate one time step
        
        Args:
            control: [ax, ay, az] - acceleration commands
            
        Returns:
            state: New state [x, y, z, vx, vy, vz]
        """
        # State update
        self.state = self.A @ self.state + self.B @ control
        
        return self.state.copy()


def run_simulation(config: SimulationConfig):
    """
    Run MPC simulation
    
    Args:
        config: Simulation configuration
        
    Returns:
        results: Dictionary containing simulation data
    """
    
    print("="*70)
    print("MPC SIMULATION - Raspberry Pi Computation Benchmark")
    print("="*70)
    print(f"Konfigurasi:")
    print(f"  - Sample time: {config.dt}s (Frequency: {1/config.dt:.0f} Hz)")
    print(f"  - Prediction horizon (Np): {config.Np}")
    print(f"  - Control horizon (Nc): {config.Nc}")
    print(f"  - Simulation duration: {config.sim_duration}s")
    print(f"  - Initial state: {config.x0}")
    print(f"  - Target state: {config.x_ref}")
    print("="*70)
    
    # Initialize
    mpc = MPCController(dt=config.dt, Np=config.Np, Nc=config.Nc)
    simulator = DroneSimulator(dt=config.dt)
    simulator.reset(config.x0)
    
    # Storage
    n_steps = int(config.sim_duration / config.dt)
    
    states = np.zeros((n_steps + 1, 6))
    controls = np.zeros((n_steps, 3))
    computation_times = np.zeros(n_steps)
    
    states[0] = config.x0
    
    # Simulation loop
    print("\nRunning simulation...")
    sim_start = time.perf_counter()
    
    for k in range(n_steps):
        # Current state
        current_state = states[k]
        
        # Compute control
        control, comp_time = mpc.compute_control(current_state, config.x_ref)
        
        # Simulate
        next_state = simulator.step(control)
        
        # Store
        states[k + 1] = next_state
        controls[k] = control
        computation_times[k] = comp_time
        
        # Progress
        if (k + 1) % 50 == 0 or k == 0:
            error = np.linalg.norm(next_state[:3] - config.x_ref[:3])
            print(f"  Step {k+1:3d}/{n_steps}: "
                  f"Position error = {error:.3f}m, "
                  f"Computation time = {comp_time*1000:.3f}ms")
    
    sim_end = time.perf_counter()
    total_sim_time = sim_end - sim_start
    
    print(f"\nSimulation completed in {total_sim_time:.2f}s")
    
    # Compute statistics
    print("\n" + "="*70)
    print("COMPUTATION TIME STATISTICS")
    print("="*70)
    print(f"Mean:     {np.mean(computation_times)*1000:.3f} ms")
    print(f"Median:   {np.median(computation_times)*1000:.3f} ms")
    print(f"Std Dev:  {np.std(computation_times)*1000:.3f} ms")
    print(f"Min:      {np.min(computation_times)*1000:.3f} ms")
    print(f"Max:      {np.max(computation_times)*1000:.3f} ms")
    print(f"95th %%:   {np.percentile(computation_times, 95)*1000:.3f} ms")
    print(f"99th %%:   {np.percentile(computation_times, 99)*1000:.3f} ms")
    
    # Real-time feasibility check
    required_freq = 1 / config.dt
    mean_comp_time = np.mean(computation_times)
    max_comp_time = np.max(computation_times)
    
    print("\n" + "="*70)
    print("REAL-TIME FEASIBILITY ANALYSIS")
    print("="*70)
    print(f"Required update rate: {required_freq:.1f} Hz ({config.dt*1000:.1f}ms period)")
    print(f"Mean computation time: {mean_comp_time*1000:.3f}ms")
    print(f"Max computation time: {max_comp_time*1000:.3f}ms")
    print(f"Headroom (mean): {(config.dt - mean_comp_time)*1000:.3f}ms "
          f"({(1 - mean_comp_time/config.dt)*100:.1f}% CPU available)")
    
    if max_comp_time < config.dt:
        print(f"✓ REAL-TIME FEASIBLE (max < deadline)")
    else:
        print(f"✗ NOT REAL-TIME (max > deadline, deadline violations possible)")
    
    # Control effort statistics
    print("\n" + "="*70)
    print("CONTROL EFFORT STATISTICS")
    print("="*70)
    for i, axis in enumerate(['X', 'Y', 'Z']):
        print(f"{axis}-axis acceleration:")
        print(f"  Mean: {np.mean(controls[:, i]):.3f} m/s²")
        print(f"  Max:  {np.max(np.abs(controls[:, i])):.3f} m/s²")
    
    # Final tracking error
    final_error = np.linalg.norm(states[-1, :3] - config.x_ref[:3])
    final_vel = np.linalg.norm(states[-1, 3:])
    
    print("\n" + "="*70)
    print("TRACKING PERFORMANCE")
    print("="*70)
    print(f"Final position error: {final_error:.3f}m")
    print(f"Final velocity: {final_vel:.3f}m/s")
    print(f"Target position: {config.x_ref[:3]}")
    print(f"Final position: {states[-1, :3]}")
    
    # Package results
    results = {
        'states': states,
        'controls': controls,
        'computation_times': computation_times,
        'config': config,
        'total_sim_time': total_sim_time
    }
    
    return results


def plot_results(results):
    """
    Plot simulation results
    
    Args:
        results: Dictionary from run_simulation()
    """
    
    states = results['states']
    controls = results['controls']
    comp_times = results['computation_times']
    config = results['config']
    
    time_vec = np.arange(len(states)) * config.dt
    time_vec_control = np.arange(len(controls)) * config.dt
    
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle('MPC Simulation Results - Raspberry Pi Benchmark', fontsize=16)
    
    # Position
    ax = axes[0, 0]
    ax.plot(time_vec, states[:, 0], label='X', linewidth=2)
    ax.plot(time_vec, states[:, 1], label='Y', linewidth=2)
    ax.plot(time_vec, states[:, 2], label='Z', linewidth=2)
    ax.axhline(config.x_ref[0], color='C0', linestyle='--', alpha=0.5)
    ax.axhline(config.x_ref[1], color='C1', linestyle='--', alpha=0.5)
    ax.axhline(config.x_ref[2], color='C2', linestyle='--', alpha=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (m)')
    ax.set_title('Position Trajectory')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Velocity
    ax = axes[1, 0]
    ax.plot(time_vec, states[:, 3], label='Vx', linewidth=2)
    ax.plot(time_vec, states[:, 4], label='Vy', linewidth=2)
    ax.plot(time_vec, states[:, 5], label='Vz', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity (m/s)')
    ax.set_title('Velocity Profile')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Control (Acceleration)
    ax = axes[2, 0]
    ax.plot(time_vec_control, controls[:, 0], label='Ax', linewidth=2)
    ax.plot(time_vec_control, controls[:, 1], label='Ay', linewidth=2)
    ax.plot(time_vec_control, controls[:, 2], label='Az', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Acceleration (m/s²)')
    ax.set_title('Control Input')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 3D Trajectory
    ax = fig.add_subplot(3, 2, 2, projection='3d')
    ax.plot(states[:, 0], states[:, 1], states[:, 2], 'b-', linewidth=2, label='Trajectory')
    ax.scatter(config.x0[0], config.x0[1], config.x0[2], c='g', s=100, marker='o', label='Start')
    ax.scatter(config.x_ref[0], config.x_ref[1], config.x_ref[2], c='r', s=100, marker='*', label='Target')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('3D Trajectory')
    ax.legend()
    
    # Computation Time
    ax = axes[1, 1]
    ax.plot(time_vec_control, comp_times * 1000, linewidth=1, alpha=0.7)
    ax.axhline(config.dt * 1000, color='r', linestyle='--', label='Deadline', linewidth=2)
    ax.axhline(np.mean(comp_times) * 1000, color='g', linestyle='--', label='Mean', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Computation Time (ms)')
    ax.set_title('MPC Computation Time per Iteration')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Computation Time Histogram
    ax = axes[2, 1]
    ax.hist(comp_times * 1000, bins=30, alpha=0.7, edgecolor='black')
    ax.axvline(config.dt * 1000, color='r', linestyle='--', label='Deadline', linewidth=2)
    ax.axvline(np.mean(comp_times) * 1000, color='g', linestyle='--', label='Mean', linewidth=2)
    ax.set_xlabel('Computation Time (ms)')
    ax.set_ylabel('Frequency')
    ax.set_title('Computation Time Distribution')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('mpc_simulation_results.png', dpi=150)
    print("\n✓ Plots saved to 'mpc_simulation_results.png'")
    plt.show()


def main():
    """Main function"""
    
    # Configure simulation
    config = SimulationConfig(
        dt=0.1,              # 10 Hz control rate
        Np=6,                # Prediction horizon
        Nc=3,                # Control horizon
        sim_duration=30.0,   # 30 seconds
        x0=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),  # Start at origin
        x_ref=np.array([10.0, 5.0, 5.0, 0.0, 0.0, 0.0])  # Target position
    )
    
    # Run simulation
    results = run_simulation(config)
    
    # Plot results
    plot_results(results)
    
    print("\n" + "="*70)
    print("Simulation complete!")
    print("="*70)


if __name__ == '__main__':
    main()
