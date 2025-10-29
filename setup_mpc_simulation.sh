#!/bin/bash
################################################################################
# MPC Simulation Setup Script for Raspberry Pi
################################################################################
#
# This script will:
# 1. Install required dependencies
# 2. Run quick test to verify installation
# 3. Optionally run full benchmark
#
# Usage:
#   chmod +x setup_mpc_simulation.sh
#   ./setup_mpc_simulation.sh
#
################################################################################

set -e  # Exit on error

echo "=============================================================================="
echo "MPC Simulation Setup for Raspberry Pi"
echo "=============================================================================="
echo ""

# Check if running on Linux
if [[ "$OSTYPE" != "linux-gnu"* ]]; then
    echo "⚠️  Warning: This script is designed for Linux (Raspberry Pi OS)"
    echo "   You may need to install dependencies manually on other systems."
    echo ""
fi

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# 1. Check Python version
echo "Checking Python installation..."
if command_exists python3; then
    PYTHON_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
    echo "✓ Python 3 found: $PYTHON_VERSION"
else
    echo "✗ Python 3 not found!"
    echo "  Install with: sudo apt install python3"
    exit 1
fi

# 2. Check pip
echo "Checking pip installation..."
if command_exists pip3; then
    echo "✓ pip3 found"
else
    echo "Installing pip3..."
    sudo apt update
    sudo apt install -y python3-pip
fi

# 3. Install system dependencies (Raspberry Pi specific)
echo ""
echo "Installing system dependencies..."
if command_exists apt; then
    echo "  Updating package list..."
    sudo apt update
    
    echo "  Installing build tools and libraries..."
    sudo apt install -y \
        python3-numpy \
        python3-matplotlib \
        python3-dev \
        build-essential \
        gfortran \
        libatlas-base-dev \
        libopenblas-dev
    
    echo "✓ System dependencies installed"
else
    echo "⚠️  apt not found, skipping system package installation"
fi

# 4. Install Python packages
echo ""
echo "Installing Python packages..."
if [ -f "requirements_simulation.txt" ]; then
    pip3 install -r requirements_simulation.txt --user
    echo "✓ Python packages installed"
else
    echo "⚠️  requirements_simulation.txt not found"
    echo "  Installing packages manually..."
    pip3 install numpy quadprog matplotlib --user
fi

# 5. Verify installation
echo ""
echo "Verifying installation..."
python3 -c "import numpy; print(f'✓ NumPy {numpy.__version__}')"
python3 -c "import quadprog; print('✓ quadprog installed')"
python3 -c "import matplotlib; print(f'✓ Matplotlib {matplotlib.__version__}')"

# 6. Display system information
echo ""
echo "=============================================================================="
echo "System Information"
echo "=============================================================================="

if [ -f /proc/cpuinfo ]; then
    MODEL=$(grep -m 1 "Model" /proc/cpuinfo | cut -d':' -f2 | xargs)
    if [ -z "$MODEL" ]; then
        MODEL=$(grep -m 1 "model name" /proc/cpuinfo | cut -d':' -f2 | xargs)
    fi
    echo "CPU Model: $MODEL"
fi

if [ -f /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq ]; then
    FREQ=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq)
    FREQ_MHZ=$((FREQ / 1000))
    echo "CPU Frequency: ${FREQ_MHZ} MHz"
fi

if [ -f /sys/class/thermal/thermal_zone0/temp ]; then
    TEMP=$(cat /sys/class/thermal/thermal_zone0/temp)
    TEMP_C=$((TEMP / 1000))
    echo "CPU Temperature: ${TEMP_C}°C"
fi

TOTAL_MEM=$(free -h | awk '/^Mem:/ {print $2}')
echo "Total Memory: $TOTAL_MEM"

# 7. Run quick test
echo ""
echo "=============================================================================="
echo "Running Quick Test"
echo "=============================================================================="
echo ""

if [ -f "mpc_quick_test.py" ]; then
    python3 mpc_quick_test.py
else
    echo "⚠️  mpc_quick_test.py not found, skipping quick test"
fi

# 8. Offer to run full benchmark
echo ""
echo "=============================================================================="
echo "Setup Complete!"
echo "=============================================================================="
echo ""
echo "Available programs:"
echo "  1. Quick Test:       python3 mpc_quick_test.py"
echo "  2. Full Simulation:  python3 mpc_simulation_standalone.py"
echo "  3. Benchmark Suite:  python3 mpc_benchmark_suite.py"
echo ""

read -p "Run full benchmark suite now? (y/N): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    if [ -f "mpc_benchmark_suite.py" ]; then
        python3 mpc_benchmark_suite.py
    else
        echo "✗ mpc_benchmark_suite.py not found"
    fi
else
    echo "You can run the benchmark later with:"
    echo "  python3 mpc_benchmark_suite.py"
fi

echo ""
echo "✓ All done! Happy benchmarking! 🚀"
