#!/usr/bin/env python3
"""
MPC Benchmark Suite - Multiple Configuration Testing
====================================================

Program untuk benchmark MPC dengan berbagai konfigurasi untuk
menemukan parameter optimal di Raspberry Pi.

Author: Modified for Raspberry Pi benchmarking
Date: October 30, 2025
"""

import numpy as np
import time
import json
from datetime import datetime
from mpc_simulation_standalone import MPCController, DroneSimulator, SimulationConfig


def benchmark_single_config(config, n_iterations=100):
    """
    Benchmark single MPC configuration
    
    Args:
        config: SimulationConfig object
        n_iterations: Number of iterations to test
        
    Returns:
        results: Dictionary with benchmark statistics
    """
    
    # Initialize
    mpc = MPCController(dt=config.dt, Np=config.Np, Nc=config.Nc)
    
    # Random test states
    computation_times = []
    
    for i in range(n_iterations):
        # Random current state
        current_state = np.random.randn(6) * 2  # Random position/velocity
        
        # Fixed reference
        reference_state = config.x_ref
        
        # Compute control
        _, comp_time = mpc.compute_control(current_state, reference_state)
        computation_times.append(comp_time)
    
    # Statistics
    comp_times_ms = np.array(computation_times) * 1000
    
    results = {
        'config': {
            'dt': config.dt,
            'Np': config.Np,
            'Nc': config.Nc,
        },
        'statistics': {
            'mean_ms': float(np.mean(comp_times_ms)),
            'median_ms': float(np.median(comp_times_ms)),
            'std_ms': float(np.std(comp_times_ms)),
            'min_ms': float(np.min(comp_times_ms)),
            'max_ms': float(np.max(comp_times_ms)),
            'p95_ms': float(np.percentile(comp_times_ms, 95)),
            'p99_ms': float(np.percentile(comp_times_ms, 99)),
        },
        'feasibility': {
            'deadline_ms': config.dt * 1000,
            'is_feasible': np.max(comp_times_ms) < config.dt * 1000,
            'headroom_mean_ms': config.dt * 1000 - np.mean(comp_times_ms),
            'headroom_max_ms': config.dt * 1000 - np.max(comp_times_ms),
            'cpu_usage_mean_percent': (np.mean(comp_times_ms) / (config.dt * 1000)) * 100,
            'cpu_usage_max_percent': (np.max(comp_times_ms) / (config.dt * 1000)) * 100,
        }
    }
    
    return results


def run_benchmark_suite():
    """
    Run comprehensive benchmark suite with multiple configurations
    """
    
    print("="*80)
    print("MPC BENCHMARK SUITE - Raspberry Pi Optimization")
    print("="*80)
    print(f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print()
    
    # Test configurations
    test_configs = []
    
    # Vary prediction horizon (Np)
    print("Testing different Prediction Horizons (Np)...")
    for Np in [4, 6, 8, 10, 12]:
        config = SimulationConfig(
            dt=0.1,
            Np=Np,
            Nc=min(3, Np),  # Nc <= Np
            x_ref=np.array([10.0, 5.0, 5.0, 0.0, 0.0, 0.0])
        )
        test_configs.append(('Np_sweep', config))
    
    # Vary control horizon (Nc)
    print("Testing different Control Horizons (Nc)...")
    for Nc in [2, 3, 4, 5]:
        config = SimulationConfig(
            dt=0.1,
            Np=10,
            Nc=Nc,
            x_ref=np.array([10.0, 5.0, 5.0, 0.0, 0.0, 0.0])
        )
        test_configs.append(('Nc_sweep', config))
    
    # Vary sample time (dt) - different control frequencies
    print("Testing different Sample Times (dt)...")
    for dt in [0.05, 0.1, 0.15, 0.2]:
        config = SimulationConfig(
            dt=dt,
            Np=6,
            Nc=3,
            x_ref=np.array([10.0, 5.0, 5.0, 0.0, 0.0, 0.0])
        )
        test_configs.append(('dt_sweep', config))
    
    # Run benchmarks
    all_results = []
    
    for i, (test_name, config) in enumerate(test_configs):
        print(f"\n[{i+1}/{len(test_configs)}] Testing: {test_name} - "
              f"Np={config.Np}, Nc={config.Nc}, dt={config.dt}")
        
        result = benchmark_single_config(config, n_iterations=100)
        result['test_name'] = test_name
        all_results.append(result)
        
        # Print summary
        stats = result['statistics']
        feas = result['feasibility']
        
        print(f"    Mean: {stats['mean_ms']:.2f}ms, "
              f"Max: {stats['max_ms']:.2f}ms, "
              f"Feasible: {'✓' if feas['is_feasible'] else '✗'}, "
              f"CPU: {feas['cpu_usage_mean_percent']:.1f}%")
    
    # Print summary table
    print("\n" + "="*80)
    print("BENCHMARK SUMMARY TABLE")
    print("="*80)
    print(f"{'Test':<15} {'Np':<4} {'Nc':<4} {'dt(ms)':<8} {'Mean(ms)':<10} "
          f"{'Max(ms)':<10} {'Feasible':<10} {'CPU%':<8}")
    print("-"*80)
    
    for result in all_results:
        test_name = result['test_name']
        cfg = result['config']
        stats = result['statistics']
        feas = result['feasibility']
        
        print(f"{test_name:<15} {cfg['Np']:<4} {cfg['Nc']:<4} {cfg['dt']*1000:<8.1f} "
              f"{stats['mean_ms']:<10.2f} {stats['max_ms']:<10.2f} "
              f"{'YES' if feas['is_feasible'] else 'NO':<10} "
              f"{feas['cpu_usage_mean_percent']:<8.1f}")
    
    # Find optimal configurations
    print("\n" + "="*80)
    print("RECOMMENDED CONFIGURATIONS")
    print("="*80)
    
    # Filter feasible configurations
    feasible = [r for r in all_results if r['feasibility']['is_feasible']]
    
    if feasible:
        # Lowest CPU usage (most headroom)
        lowest_cpu = min(feasible, key=lambda x: x['feasibility']['cpu_usage_mean_percent'])
        print("\n1. Lowest CPU Usage (Most Headroom):")
        print(f"   Np={lowest_cpu['config']['Np']}, "
              f"Nc={lowest_cpu['config']['Nc']}, "
              f"dt={lowest_cpu['config']['dt']}s")
        print(f"   CPU: {lowest_cpu['feasibility']['cpu_usage_mean_percent']:.1f}%, "
              f"Headroom: {lowest_cpu['feasibility']['headroom_mean_ms']:.2f}ms")
        
        # Best performance (shortest computation time)
        fastest = min(feasible, key=lambda x: x['statistics']['mean_ms'])
        print("\n2. Fastest Computation:")
        print(f"   Np={fastest['config']['Np']}, "
              f"Nc={fastest['config']['Nc']}, "
              f"dt={fastest['config']['dt']}s")
        print(f"   Mean: {fastest['statistics']['mean_ms']:.2f}ms, "
              f"Max: {fastest['statistics']['max_ms']:.2f}ms")
        
        # Recommended balanced
        # Balance between performance and accuracy (higher Np is better for accuracy)
        balanced = max([r for r in feasible if r['feasibility']['cpu_usage_mean_percent'] < 50],
                      key=lambda x: x['config']['Np'],
                      default=lowest_cpu)
        print("\n3. Recommended Balanced (Accuracy + Performance):")
        print(f"   Np={balanced['config']['Np']}, "
              f"Nc={balanced['config']['Nc']}, "
              f"dt={balanced['config']['dt']}s")
        print(f"   CPU: {balanced['feasibility']['cpu_usage_mean_percent']:.1f}%, "
              f"Mean: {balanced['statistics']['mean_ms']:.2f}ms")
    else:
        print("\n⚠️  WARNING: No feasible configurations found!")
        print("   Consider:")
        print("   - Using faster Raspberry Pi model")
        print("   - Reducing prediction horizon (Np)")
        print("   - Increasing sample time (dt)")
    
    # Save results to JSON
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = f'mpc_benchmark_{timestamp}.json'
    
    with open(filename, 'w') as f:
        json.dump({
            'timestamp': timestamp,
            'results': all_results
        }, f, indent=2)
    
    print(f"\n✓ Results saved to: {filename}")
    print("="*80)
    
    return all_results


def analyze_scaling():
    """
    Analyze how computation time scales with problem size
    """
    
    print("\n" + "="*80)
    print("SCALING ANALYSIS")
    print("="*80)
    
    # Test different problem sizes
    test_sizes = []
    
    # Vary Np while keeping Nc proportional
    for Np in [3, 4, 6, 8, 10, 12, 15, 20]:
        Nc = min(3, Np)
        config = SimulationConfig(dt=0.1, Np=Np, Nc=Nc)
        
        print(f"Testing Np={Np}, Nc={Nc}...", end=' ')
        result = benchmark_single_config(config, n_iterations=50)
        test_sizes.append((Np, result['statistics']['mean_ms']))
        print(f"{result['statistics']['mean_ms']:.2f}ms")
    
    # Analyze scaling
    print("\nScaling Analysis:")
    Np_values = [x[0] for x in test_sizes]
    times = [x[1] for x in test_sizes]
    
    # Estimate complexity
    # MPC typically scales as O(Np^3) due to QP solver
    print(f"Np range: {min(Np_values)} to {max(Np_values)}")
    print(f"Time range: {min(times):.2f}ms to {max(times):.2f}ms")
    print(f"Scaling factor: {max(times)/min(times):.2f}x")
    
    # Calculate empirical exponent
    # time ≈ k * Np^alpha
    # log(time) ≈ log(k) + alpha * log(Np)
    log_Np = np.log(Np_values)
    log_time = np.log(times)
    
    # Linear fit
    A = np.vstack([log_Np, np.ones(len(log_Np))]).T
    alpha, log_k = np.linalg.lstsq(A, log_time, rcond=None)[0]
    
    print(f"Empirical scaling: time ≈ {np.exp(log_k):.3f} * Np^{alpha:.2f}")
    print(f"(Theoretical MPC scaling: O(Np³))")
    
    return test_sizes


def main():
    """Main benchmark function"""
    
    print("\n")
    print("█"*80)
    print("█" + " "*78 + "█")
    print("█" + " "*20 + "MPC RASPBERRY PI BENCHMARK SUITE" + " "*26 + "█")
    print("█" + " "*78 + "█")
    print("█"*80)
    print()
    
    # Run comprehensive benchmark
    results = run_benchmark_suite()
    
    # Analyze scaling
    scaling_results = analyze_scaling()
    
    print("\n")
    print("█"*80)
    print("█" + " "*78 + "█")
    print("█" + " "*30 + "BENCHMARK COMPLETE" + " "*30 + "█")
    print("█" + " "*78 + "█")
    print("█"*80)
    print()


if __name__ == '__main__':
    main()
