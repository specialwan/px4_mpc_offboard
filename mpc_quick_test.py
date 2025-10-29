#!/usr/bin/env python3
"""
MPC Quick Test - Simple Computation Time Check
==============================================

Program sederhana untuk cek cepat waktu komputasi MPC di Raspberry Pi.

Usage:
    python3 mpc_quick_test.py
"""

import numpy as np
import time
from mpc_simulation_standalone import MPCController


def quick_test():
    """Quick test untuk cek computation time"""
    
    print("="*60)
    print("MPC QUICK COMPUTATION TIME TEST")
    print("="*60)
    
    # Test configurations
    configs = [
        {'Np': 4, 'Nc': 2, 'dt': 0.1, 'name': 'Fast (Np=4)'},
        {'Np': 6, 'Nc': 3, 'dt': 0.05, 'name': 'Balanced (Np=6)'},
        {'Np': 10, 'Nc': 3, 'dt': 0.1, 'name': 'Accurate (Np=10)'},
    ]
    
    for config in configs:
        print(f"\nTesting: {config['name']}")
        print(f"  Parameters: Np={config['Np']}, Nc={config['Nc']}, dt={config['dt']}s")
        
        # Initialize MPC
        mpc = MPCController(dt=config['dt'], Np=config['Np'], Nc=config['Nc'])
        
        # Run 20 iterations
        times = []
        for i in range(20):
            # Random state
            current_state = np.random.randn(6) * 2
            reference_state = np.array([10.0, 5.0, 5.0, 0.0, 0.0, 0.0])
            
            # Compute
            _, comp_time = mpc.compute_control(current_state, reference_state)
            times.append(comp_time * 1000)  # Convert to ms
        
        # Statistics
        mean_time = np.mean(times)
        max_time = np.max(times)
        deadline = config['dt'] * 1000
        
        print(f"  Results:")
        print(f"    Mean: {mean_time:.2f}ms")
        print(f"    Max:  {max_time:.2f}ms")
        print(f"    Deadline: {deadline:.0f}ms")
        
        if max_time < deadline:
            headroom = deadline - mean_time
            cpu_pct = (mean_time / deadline) * 100
            print(f"    ✓ FEASIBLE - Headroom: {headroom:.2f}ms ({cpu_pct:.1f}% CPU)")
        else:
            print(f"    ✗ NOT FEASIBLE - Exceeds deadline!")
    
    print("\n" + "="*60)
    print("Recommendation:")
    print("  - Use 'Fast' for Raspberry Pi Zero/1")
    print("  - Use 'Balanced' for Raspberry Pi 3/4")
    print("  - Use 'Accurate' for high-end systems")
    print("="*60)


def cpu_info():
    """Display CPU information (Linux only)"""
    
    try:
        # Get CPU info
        with open('/proc/cpuinfo', 'r') as f:
            lines = f.readlines()
        
        # Extract model
        model = [l for l in lines if 'Model' in l or 'model name' in l]
        if model:
            print("\nCPU Info:")
            print("  " + model[0].strip())
        
        # Get CPU frequency
        try:
            with open('/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq', 'r') as f:
                freq = int(f.read().strip()) / 1000  # Convert to MHz
                print(f"  Current Frequency: {freq:.0f} MHz")
        except:
            pass
        
        # Get temperature (Raspberry Pi)
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp = int(f.read().strip()) / 1000  # Convert to Celsius
                print(f"  Temperature: {temp:.1f}°C")
        except:
            pass
            
    except Exception as e:
        print(f"\nCouldn't read CPU info: {e}")


if __name__ == '__main__':
    cpu_info()
    quick_test()
