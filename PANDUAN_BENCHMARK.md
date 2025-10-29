# MPC Raspberry Pi Benchmark - Panduan Lengkap

## 📋 Ringkasan

Program ini adalah **standalone Python MPC simulator** (tanpa ROS) yang dibuat untuk:
- ✅ Mengukur waktu komputasi MPC di Raspberry Pi
- ✅ Menggunakan matrix dan parameter yang sama dengan implementasi ROS
- ✅ Benchmark berbagai konfigurasi untuk menemukan setting optimal
- ✅ Simulasi trajectory tracking dengan visualisasi

## 🎯 Tujuan

Mengetahui **apakah Raspberry Pi mampu menjalankan MPC secara real-time** untuk kontrol drone dengan mengukur:
- Waktu komputasi per iterasi (ms)
- CPU usage (%)
- Real-time feasibility
- Optimal parameter (Np, Nc, dt)

## 📁 File yang Dibuat

### 1. **mpc_simulation_standalone.py** (Program Utama)
Program simulasi lengkap dengan:
- MPC controller (sama seperti ROS version)
- Drone simulator
- Visualisasi trajectory
- Statistik computation time

**Cara pakai:**
```bash
python3 mpc_simulation_standalone.py
```

**Output:**
- Console: Statistics computation time, tracking performance
- Plot: `mpc_simulation_results.png` (6 subplots)

### 2. **mpc_quick_test.py** (Quick Test)
Test cepat (20 iterasi) untuk 3 konfigurasi:
- Fast (Np=4): Untuk Raspberry Pi lemah
- Balanced (Np=6): Untuk Raspberry Pi 3/4
- Accurate (Np=10): Untuk sistem powerful

**Cara pakai:**
```bash
python3 mpc_quick_test.py
```

**Output:** Computation time untuk setiap konfigurasi (~5 detik)

### 3. **mpc_benchmark_suite.py** (Comprehensive Benchmark)
Benchmark lengkap dengan berbagai konfigurasi:
- Sweep Prediction Horizon (Np): 4, 6, 8, 10, 12
- Sweep Control Horizon (Nc): 2, 3, 4, 5
- Sweep Sample Time (dt): 0.05, 0.1, 0.15, 0.2s
- Scaling analysis

**Cara pakai:**
```bash
python3 mpc_benchmark_suite.py
```

**Output:**
- Console: Summary table, recommendations
- JSON file: `mpc_benchmark_YYYYMMDD_HHMMSS.json`

### 4. **setup_mpc_simulation.sh** (Setup Script)
Automated setup untuk Raspberry Pi:
- Install dependencies
- Verify installation
- Run quick test
- Optional full benchmark

**Cara pakai:**
```bash
chmod +x setup_mpc_simulation.sh
./setup_mpc_simulation.sh
```

## 🚀 Quick Start

### Step 1: Copy files ke Raspberry Pi
```bash
# Di laptop (transfer via SSH)
scp mpc_*.py requirements_simulation.txt setup_mpc_simulation.sh pi@raspberrypi:~/

# Atau clone dari GitHub jika sudah di-push
git clone <your-repo> && cd <your-repo>
```

### Step 2: Run setup script
```bash
chmod +x setup_mpc_simulation.sh
./setup_mpc_simulation.sh
```

Script ini akan:
1. Install dependencies (numpy, quadprog, matplotlib)
2. Verify installation
3. Run quick test
4. Tanya apakah mau run full benchmark

### Step 3: Interpret hasil

**Quick Test Output Example:**
```
Testing: Balanced (Np=6)
  Parameters: Np=6, Nc=3, dt=0.1s
  Results:
    Mean: 15.32ms
    Max:  18.45ms
    Deadline: 100ms
    ✓ FEASIBLE - Headroom: 84.68ms (15.3% CPU)
```

**Interpretasi:**
- ✅ **FEASIBLE**: Max time (18.45ms) < Deadline (100ms)
- ✅ **Good headroom**: 84.68ms available untuk task lain
- ✅ **Low CPU**: Hanya 15.3% CPU untuk MPC

## 📊 Expected Results

### Raspberry Pi 4 (1.5 GHz quad-core)
```
Configuration: Np=6, Nc=3, dt=0.1s (10 Hz)
Mean: 8-12 ms
Max:  12-18 ms
Result: ✓ FEASIBLE (dapat run 10 Hz real-time)
```

### Raspberry Pi 3 (1.2 GHz quad-core)
```
Configuration: Np=6, Nc=3, dt=0.1s (10 Hz)
Mean: 15-20 ms
Max:  20-30 ms
Result: ✓ FEASIBLE (marginal, consider Np=4)
```

### Raspberry Pi Zero (1 GHz single-core)
```
Configuration: Np=6, Nc=3, dt=0.1s (10 Hz)
Mean: 50-70 ms
Max:  80-100 ms
Result: ✗ NOT FEASIBLE (use Np=4 atau dt=0.2)
```

## ⚙️ Optimization Tips

Jika computation time **terlalu lambat** (NOT FEASIBLE):

### 1. Reduce Prediction Horizon
```python
config = SimulationConfig(
    Np=4,  # Dari 6 → 4
    Nc=2,  # Dari 3 → 2
    dt=0.1
)
```
**Effect:** ~50% faster, sedikit kurang accurate

### 2. Increase Sample Time
```python
config = SimulationConfig(
    Np=6,
    Nc=3,
    dt=0.2  # Dari 0.1s → 0.2s (5 Hz instead of 10 Hz)
)
```
**Effect:** 2x longer deadline, lebih feasible

### 3. Simplify Cost Function
```python
# Di MPCController.__init__()
self.R_delta = np.zeros((3, 3))  # Remove control rate penalty
```
**Effect:** ~10-20% faster, lebih aggressive control

## 📈 Performance Tuning Raspberry Pi

### 1. Set CPU to Performance Mode
```bash
# Check current governor
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor

# Set to performance (temporary)
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Permanent: edit /boot/config.txt
# Add: force_turbo=1
```

### 2. Disable WiFi/Bluetooth During Benchmark
```bash
sudo rfkill block wifi
sudo rfkill block bluetooth

# Re-enable after benchmark
sudo rfkill unblock wifi
sudo rfkill unblock bluetooth
```

### 3. Monitor Temperature
```bash
# Check temperature
vcgencmd measure_temp

# If > 80°C, add cooling or reduce clock speed
```

### 4. Increase Swap (for compilation)
```bash
# Edit swap size
sudo nano /etc/dphys-swapfile
# Change: CONF_SWAPSIZE=2048

# Restart swap
sudo /etc/init.d/dphys-swapfile restart
```

## 🔍 Troubleshooting

### Error: "No module named 'quadprog'"
```bash
# Install with pip
pip3 install quadprog --user

# Or compile from source if pip fails
sudo apt install python3-dev gfortran libatlas-base-dev
pip3 install quadprog --user --no-binary :all:
```

### Error: "QP solver failed"
**Cause:** Constraint matrices numerical issue

**Solution:**
```python
# Reduce acceleration limit
self.a_max = 3.0  # Instead of 6.0
```

### Slow Compilation
**Cause:** Low memory on Raspberry Pi

**Solution:**
```bash
# Increase swap
sudo nano /etc/dphys-swapfile
# CONF_SWAPSIZE=2048

# Or install pre-compiled wheels
pip3 install quadprog --prefer-binary
```

### Plot doesn't show (no display)
**Cause:** Running via SSH without X11 forwarding

**Solution 1 - Save plot only:**
```python
# Edit mpc_simulation_standalone.py
# Comment out: plt.show()
# Plot will save to PNG file
```

**Solution 2 - Use SSH with X11:**
```bash
# Connect with X11 forwarding
ssh -X pi@raspberrypi
```

**Solution 3 - Use non-GUI backend:**
```python
# Add at top of script
import matplotlib
matplotlib.use('Agg')  # Non-GUI backend
import matplotlib.pyplot as plt
```

## 📊 Benchmark Results Format

### Console Output
```
======================================================================
COMPUTATION TIME STATISTICS
======================================================================
Mean:     15.32 ms
Median:   14.87 ms
Std Dev:  2.34 ms
Min:      12.45 ms
Max:      21.23 ms
95th %:   19.45 ms
99th %:   20.87 ms

======================================================================
REAL-TIME FEASIBILITY ANALYSIS
======================================================================
Required update rate: 10.0 Hz (100.0ms period)
Mean computation time: 15.32ms
Max computation time: 21.23ms
Headroom (mean): 84.68ms (84.7% CPU available)
✓ REAL-TIME FEASIBLE (max < deadline)
```

### JSON Output (from benchmark suite)
```json
{
  "timestamp": "20251030_143022",
  "results": [
    {
      "test_name": "Np_sweep",
      "config": {
        "dt": 0.1,
        "Np": 6,
        "Nc": 3
      },
      "statistics": {
        "mean_ms": 15.32,
        "max_ms": 21.23,
        ...
      },
      "feasibility": {
        "is_feasible": true,
        "cpu_usage_mean_percent": 15.3,
        ...
      }
    }
  ]
}
```

## 🎯 Recommended Configurations

### For Raspberry Pi 4
```python
# Balanced (Best overall)
Np=6, Nc=3, dt=0.1  # 10 Hz, good accuracy

# High accuracy
Np=10, Nc=3, dt=0.1  # Still feasible, better prediction
```

### For Raspberry Pi 3
```python
# Recommended
Np=4, Nc=2, dt=0.1  # 10 Hz, fast enough

# Alternative
Np=6, Nc=3, dt=0.15  # 6.7 Hz, more accurate
```

### For Raspberry Pi Zero
```python
# Only option
Np=3, Nc=2, dt=0.2  # 5 Hz, minimal but works
```

## 📚 Next Steps

Setelah mendapat hasil benchmark:

1. **Jika FEASIBLE**: Gunakan parameter ini di ROS implementation
2. **Jika NOT FEASIBLE**: Optimize parameters atau upgrade hardware
3. **Compare dengan requirements**: Apakah 10 Hz cukup untuk drone?
4. **Test di real hardware**: Transfer ROS node ke Raspberry Pi

## 🔗 References

- Original ROS code: `mpc_waypoint_follower_mavros.py`
- MPC Theory: Predictive Control for Linear and Hybrid Systems
- quadprog library: https://github.com/quadprog/quadprog

## 📝 Notes

- **Computation time** sangat tergantung pada CPU clock speed
- **Real-time feasibility** butuh max time < deadline (bukan average)
- **Thermal throttling** bisa terjadi jika Raspberry Pi panas
- **Background tasks** bisa affect computation time

---

**Author:** Modified for Raspberry Pi benchmarking  
**Date:** October 30, 2025  
**Version:** 1.0
