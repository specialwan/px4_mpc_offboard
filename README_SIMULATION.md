# MPC Simulation - Raspberry Pi Computation Benchmark

Program simulasi MPC standalone (tanpa ROS) untuk mengukur waktu komputasi di Raspberry Pi.

## Deskripsi

Program ini mensimulasikan kontroler MPC (Model Predictive Control) dengan parameter dan matrix yang sama seperti implementasi ROS, tetapi dalam bentuk standalone Python untuk:
- Mengukur waktu komputasi MPC per iterasi
- Menganalisis real-time feasibility
- Benchmark performa Raspberry Pi
- Visualisasi trajectory dan performance

## Features

### Model MPC
- **State (6-dimensi)**: `[x, y, z, vx, vy, vz]`
- **Control (3-dimensi)**: `[ax, ay, az]` (akselerasi)
- **Prediction Horizon (Np)**: 6
- **Control Horizon (Nc)**: 3
- **Sample Time**: 0.1s (10 Hz)

### Matrix System
```python
A = [1, 0, 0, dt, 0,  0 ]
    [0, 1, 0, 0,  dt, 0 ]
    [0, 0, 1, 0,  0,  dt]
    [0, 0, 0, 1,  0,  0 ]
    [0, 0, 0, 0,  1,  0 ]
    [0, 0, 0, 0,  0,  1 ]

B = [0.5*dt², 0,      0     ]
    [0,      0.5*dt², 0     ]
    [0,      0,      0.5*dt²]
    [dt,     0,      0     ]
    [0,      dt,     0     ]
    [0,      0,      dt    ]
```

### Cost Matrices
- **Q (state weight)**: `diag([1.0, 1.0, 100.0, 1.0, 1.0, 30.0])`
- **R (control effort)**: `diag([0.05, 0.05, 0.05])`
- **R_delta (control rate)**: `diag([0.3, 0.3, 0.2])`

## Instalasi

### Requirements
```bash
pip install numpy quadprog matplotlib
```

### Untuk Raspberry Pi
```bash
# Update system
sudo apt update
sudo apt upgrade

# Install dependencies
sudo apt install python3-pip python3-numpy python3-matplotlib

# Install quadprog
pip3 install quadprog

# Download program
cd ~
# Copy mpc_simulation_standalone.py ke Raspberry Pi
```

## Cara Menggunakan

### 1. Run Basic Simulation
```bash
python3 mpc_simulation_standalone.py
```

### 2. Custom Configuration
Edit di dalam `main()` function:
```python
config = SimulationConfig(
    dt=0.1,              # Sample time (s)
    Np=6,                # Prediction horizon
    Nc=3,                # Control horizon
    sim_duration=30.0,   # Duration (s)
    x0=np.array([0, 0, 0, 0, 0, 0]),        # Initial state
    x_ref=np.array([10, 5, 5, 0, 0, 0])     # Target state
)
```

### 3. Benchmark Different Configurations
```python
# Test different horizons
for Np in [4, 6, 8, 10]:
    config = SimulationConfig(Np=Np, Nc=3)
    results = run_simulation(config)
```

## Output

### Console Output
Program akan menampilkan:
1. **Progress simulasi** setiap 50 iterasi
2. **Statistik waktu komputasi**:
   - Mean, Median, Std Dev
   - Min, Max
   - 95th dan 99th percentile
3. **Real-time feasibility analysis**:
   - Apakah computation time < deadline
   - CPU headroom
4. **Control effort statistics**
5. **Tracking performance**

### Contoh Output
```
======================================================================
COMPUTATION TIME STATISTICS
======================================================================
Mean:     2.347 ms
Median:   2.312 ms
Std Dev:  0.234 ms
Min:      1.987 ms
Max:      3.456 ms
95th %:   2.789 ms
99th %:   3.123 ms

======================================================================
REAL-TIME FEASIBILITY ANALYSIS
======================================================================
Required update rate: 10.0 Hz (100.0ms period)
Mean computation time: 2.347ms
Max computation time: 3.456ms
Headroom (mean): 97.653ms (97.7% CPU available)
✓ REAL-TIME FEASIBLE (max < deadline)
```

### Plots
Program akan generate plot `mpc_simulation_results.png` berisi:
1. **Position trajectory** (X, Y, Z vs time)
2. **Velocity profile** (Vx, Vy, Vz vs time)
3. **Control input** (Ax, Ay, Az vs time)
4. **3D trajectory** dengan start/target point
5. **Computation time** per iterasi
6. **Computation time histogram**

## Interpretasi Hasil

### Real-Time Feasibility
- **✓ FEASIBLE**: Max computation time < deadline
  - MPC dapat berjalan real-time di Raspberry Pi
  - Ada headroom untuk task lain
  
- **✗ NOT FEASIBLE**: Max computation time > deadline
  - MPC terlalu lambat untuk real-time
  - Perlu optimasi atau reduce horizon

### Optimasi untuk Raspberry Pi

Jika computation time terlalu lambat:

1. **Reduce Prediction Horizon**:
   ```python
   config = SimulationConfig(Np=4, Nc=2)  # Dari Np=6, Nc=3
   ```

2. **Increase Sample Time**:
   ```python
   config = SimulationConfig(dt=0.2)  # Dari dt=0.1 (5 Hz instead of 10 Hz)
   ```

3. **Simplify Cost Function**:
   ```python
   # Remove control rate penalty
   self.R_delta = np.zeros((3, 3))
   ```

## Benchmark Different Raspberry Pi Models

### Raspberry Pi 4 (Expected)
- Mean: ~2-3 ms
- Max: ~4-5 ms
- Real-time feasible at 10 Hz

### Raspberry Pi 3 (Expected)
- Mean: ~5-8 ms
- Max: ~10-15 ms
- Real-time feasible at 10 Hz (marginal)

### Raspberry Pi Zero (Expected)
- Mean: ~15-25 ms
- Max: ~30-40 ms
- NOT real-time at 10 Hz, use 5 Hz instead

## Tips untuk Benchmark

1. **Disable WiFi/Bluetooth** saat benchmark untuk hasil konsisten:
   ```bash
   sudo rfkill block wifi
   sudo rfkill block bluetooth
   ```

2. **Set CPU to performance mode**:
   ```bash
   echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
   ```

3. **Run multiple times** dan ambil average:
   ```bash
   for i in {1..5}; do
       echo "Run $i"
       python3 mpc_simulation_standalone.py
   done
   ```

4. **Monitor CPU temperature**:
   ```bash
   vcgencmd measure_temp
   ```

## Troubleshooting

### Error: "No module named 'quadprog'"
```bash
pip3 install quadprog
```

### Error: "QP solver failed"
- Check constraint matrices
- Reduce acceleration limits (`a_max`)
- Check initial conditions

### Slow computation
- Reduce `Np` (prediction horizon)
- Reduce `Nc` (control horizon)
- Increase `dt` (sample time)

## Modifikasi Lanjutan

### 1. Test Different Trajectories
```python
# Circular trajectory
def circular_reference(t, radius=5, height=5, omega=0.1):
    x = radius * np.cos(omega * t)
    y = radius * np.sin(omega * t)
    z = height
    return np.array([x, y, z, 0, 0, 0])
```

### 2. Add Disturbances
```python
# Wind disturbance
def step(self, control):
    wind = np.random.randn(3) * 0.5  # Random wind
    total_accel = control + wind
    self.state = self.A @ self.state + self.B @ total_accel
    return self.state.copy()
```

### 3. Save Results to File
```python
import json

# Save computation times
with open('benchmark_results.json', 'w') as f:
    json.dump({
        'mean': float(np.mean(comp_times)),
        'max': float(np.max(comp_times)),
        'std': float(np.std(comp_times)),
        'config': {
            'Np': config.Np,
            'Nc': config.Nc,
            'dt': config.dt
        }
    }, f, indent=2)
```

## Reference

- **Original ROS Implementation**: `mpc_waypoint_follower_mavros.py`
- **Quadratic Programming**: `quadprog` library
- **MPC Theory**: Model Predictive Control textbooks

## Author

Modified for Raspberry Pi benchmarking - October 30, 2025

## License

Same as original px4_mpc_offboard project
