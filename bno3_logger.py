#!/usr/bin/env python3
"""
BNO055 x3 Logger via TCA9548A (I2C multiplexer) on Raspberry Pi
- Bus: I2C-1 (/dev/i2c-1)
- MUX address: 0x70
- BNO055 address: 0x28 (ADR=GND) or 0x29 (ADR=3V3) — here we assume 0x28 on each MUX channel
- Channels used: 0, 1, 2  (edit CHANNELS if you wire differently)

Outputs:
- CSV file in ./logs/ with timestamp-based filename
- Columns: t_ros_ns,sensor,ax,ay,az,gx,gy,gz,roll_deg,pitch_deg,yaw_deg,temp_C,mux_ch
- Flushes every FLUSH_EVERY lines for safety

Notes:
- Operation mode: IMU (fusion of accel+gyro, magnetometer OFF) to avoid magnetic interference
- Units: m/s^2 for acceleration, dps for angular rate, degrees for Euler
- BNO055 max output rate is ~100 Hz; we loop per channel sequentially
- If you need FCU/Pixhawk time sync, integrate with MAVROS later and add an extra column t_fcu_ns
"""

import os
import csv
import time
import signal
from datetime import datetime
from typing import Tuple, List

from smbus2 import SMBus

I2C_BUS_ID = 1
MUX_ADDR   = 0x70
BNO_ADDR   = 0x28

# Use three MUX channels for three sensors mounted on arms
CHANNELS = [0, 1, 2]
SENSOR_NAMES = {0: "arm_A", 1: "arm_B", 2: "arm_C"}

# Logging params
TARGET_RATE_HZ = 100                     # BNO055 practical max is ~100 Hz
FLUSH_EVERY    = 200                     # flush CSV every N rows

# ---------------- BNO055 registers ----------------
REG_PAGE_ID     = 0x07
REG_CHIP_ID     = 0x00  # should read 0xA0
REG_ACC_DATA_X  = 0x08  # 6 bytes: X LSB, X MSB, Y LSB, Y MSB, Z LSB, Z MSB
REG_MAG_DATA_X  = 0x0E
REG_GYR_DATA_X  = 0x14  # 6 bytes
REG_EUL_HEADING = 0x1A  # 6 bytes: H, R, P (LSB/MSB)
REG_TEMP        = 0x34
REG_OPR_MODE    = 0x3D
REG_PWR_MODE    = 0x3E
REG_UNIT_SEL    = 0x3B

# Operation modes
MODE_CONFIG = 0x00
MODE_IMU    = 0x08   # Acc+Gyro fusion, magnetometer disabled

# Power modes
PWR_NORMAL  = 0x00

# UNIT_SEL bits (see datasheet). We'll set: m/s^2, dps, degrees, Celsius
# bit0 (ACC): 0 = m/s^2, 1 = mg
# bit1 (ANG): 0 = dps,   1 = rps
# bit2 (EUL): 0 = deg,   1 = rad
# bit4 (TEMP):0 = Celsius, 1 = Fahrenheit
UNIT_MS2_DPS_DEG_C = 0b00000000

def mux_select(bus: SMBus, ch: int) -> None:
    """Select a channel on TCA9548A."""
    if ch < 0 or ch > 7:
        raise ValueError("MUX channel must be 0..7")
    bus.write_byte(MUX_ADDR, 1 << ch)

def bno_write8(bus: SMBus, reg: int, val: int) -> None:
    bus.write_byte_data(BNO_ADDR, reg, val & 0xFF)

def bno_readN(bus: SMBus, reg: int, n: int) -> List[int]:
    return bus.read_i2c_block_data(BNO_ADDR, reg, n)

def to_signed_16(lsb: int, msb: int) -> int:
    """Combine two bytes into a signed 16-bit integer."""
    v = (msb << 8) | lsb
    if v >= 32768:
        v -= 65536
    return v

def bno_init(bus: SMBus) -> None:
    """Basic init sequence for BNO055 on the currently selected MUX channel."""
    # Go to CONFIG mode
    bno_write8(bus, REG_OPR_MODE, MODE_CONFIG)
    time.sleep(0.02)

    # Set power mode normal
    bno_write8(bus, REG_PWR_MODE, PWR_NORMAL)
    time.sleep(0.01)

    # Units: m/s^2, dps, degrees, Celsius
    bno_write8(bus, REG_UNIT_SEL, UNIT_MS2_DPS_DEG_C)
    time.sleep(0.01)

    # Ensure Page 0
    bno_write8(bus, REG_PAGE_ID, 0x00)
    time.sleep(0.01)

    # Switch to IMU mode
    bno_write8(bus, REG_OPR_MODE, MODE_IMU)
    time.sleep(0.05)  # give fusion time

def read_accel(bus: SMBus) -> Tuple[float, float, float]:
    """Read accelerometer m/s^2 (per UNIT_SEL)"""
    raw = bno_readN(bus, REG_ACC_DATA_X, 6)
    x = to_signed_16(raw[0], raw[1])
    y = to_signed_16(raw[2], raw[3])
    z = to_signed_16(raw[4], raw[5])
    # According to datasheet, in m/s^2 mode: LSB is 1/100 m/s^2
    return (x/100.0, y/100.0, z/100.0)

def read_gyro(bus: SMBus) -> Tuple[float, float, float]:
    """Read gyroscope dps (per UNIT_SEL)"""
    raw = bno_readN(bus, REG_GYR_DATA_X, 6)
    x = to_signed_16(raw[0], raw[1])
    y = to_signed_16(raw[2], raw[3])
    z = to_signed_16(raw[4], raw[5])
    # In dps mode: LSB is 1/16 dps
    return (x/16.0, y/16.0, z/16.0)

def read_euler(bus: SMBus) -> Tuple[float, float, float]:
    """Read Euler angles (heading, roll, pitch) in degrees"""
    raw = bno_readN(bus, REG_EUL_HEADING, 6)
    h = to_signed_16(raw[0], raw[1]) / 16.0
    r = to_signed_16(raw[2], raw[3]) / 16.0
    p = to_signed_16(raw[4], raw[5]) / 16.0
    return (h, r, p)

def read_temp_c(bus: SMBus) -> float:
    # Per datasheet: REG_TEMP is signed 8-bit Celsius
    t = bus.read_byte_data(BNO_ADDR, REG_TEMP)
    if t >= 128:
        t -= 256
    return float(t)

def main():
    os.makedirs("logs", exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = f"logs/bno3_{ts}.csv"

    with SMBus(I2C_BUS_ID) as bus, open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t_ros_ns","sensor","ax","ay","az","gx","gy","gz","roll_deg","pitch_deg","yaw_deg","temp_C","mux_ch"])

        # Init all three sensors
        for ch in CHANNELS:
            mux_select(bus, ch)
            chip = bus.read_byte_data(BNO_ADDR, REG_CHIP_ID)
            if chip != 0xA0:
                print(f"[WARN] CH{ch}: unexpected CHIP_ID=0x{chip:02X}. Check wiring/address.")
            bno_init(bus)
            time.sleep(0.02)
        print("[INFO] All BNO055 initialized (IMU mode, m/s^2+dps+deg). Logging...")
        print(f"[INFO] CSV: {out_path}")

        # graceful shutdown
        running = True
        def handle_sigint(sig, frame):
            nonlocal running
            running = False
            print("\n[INFO] Stopping... Flushing and closing.")
        signal.signal(signal.SIGINT, handle_sigint)

        # Loop
        last_flush = 0
        period = 1.0 / TARGET_RATE_HZ
        next_t = time.perf_counter()
        while running:
            for ch in CHANNELS:
                mux_select(bus, ch)
                t_ros_ns = time.time_ns()
                ax, ay, az = read_accel(bus)
                gx, gy, gz = read_gyro(bus)
                yaw, roll, pitch = read_euler(bus)  # BNO order: heading(yaw), roll, pitch
                temp_c = read_temp_c(bus)
                name = SENSOR_NAMES.get(ch, f"ch{ch}")
                writer.writerow([t_ros_ns, name, ax, ay, az, gx, gy, gz, roll, pitch, yaw, temp_c, ch])
                last_flush += 1
                if last_flush >= FLUSH_EVERY:
                    f.flush()
                    os.fsync(f.fileno())
                    last_flush = 0

            # rate control (approx overall per-channel loop ~ TARGET_RATE_HZ)
            next_t += period
            sleep_s = next_t - time.perf_counter()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                # if we're lagging, reset schedule
                next_t = time.perf_counter()

        # Final flush on exit
        f.flush()
        os.fsync(f.fileno())

if __name__ == "__main__":
    main()
