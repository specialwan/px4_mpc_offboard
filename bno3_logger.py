#!/usr/bin/env python3
"""
Patched BNO055 x3 Logger with robust probing.
- Probes each TCA9548A channel for BNO055 at 0x28 and 0x29
- Adds settling delay after mux_select
- Skips channels without a detected chip (CHIP_ID != 0xA0)
"""

import os
import csv
import time
import signal
from datetime import datetime
from typing import Tuple, List

from smbus2 import SMBus, i2c_msg

I2C_BUS_ID = 1
MUX_ADDR   = 0x70
# Channels to try:
CHANNELS = [0, 1, 2]
# Addresses to try on each channel (BNO055 supports 0x28 or 0x29 depending on ADR/SDO pin)
BNO_ADDRS = [0x28, 0x29]

TARGET_RATE_HZ = 100
FLUSH_EVERY    = 200

REG_PAGE_ID     = 0x07
REG_CHIP_ID     = 0x00  # expect 0xA0
REG_ACC_DATA_X  = 0x08
REG_GYR_DATA_X  = 0x14
REG_EUL_HEADING = 0x1A
REG_TEMP        = 0x34
REG_OPR_MODE    = 0x3D
REG_PWR_MODE    = 0x3E
REG_UNIT_SEL    = 0x3B

MODE_CONFIG = 0x00
MODE_IMU    = 0x08
PWR_NORMAL  = 0x00
UNIT_MS2_DPS_DEG_C = 0b00000000

SENSOR_NAMES = {0: "arm_A", 1: "arm_B", 2: "arm_C"}

def mux_select(bus: SMBus, ch: int) -> None:
    bus.write_byte(MUX_ADDR, 1 << ch)
    # small settle time improves reliability on some boards
    time.sleep(0.003)

def to_signed_16(lsb: int, msb: int) -> int:
    v = (msb << 8) | lsb
    if v >= 32768:
        v -= 65536
    return v

def bno_write8(bus: SMBus, addr: int, reg: int, val: int) -> None:
    bus.write_byte_data(addr, reg, val & 0xFF)

def bno_readN(bus: SMBus, addr: int, reg: int, n: int) -> List[int]:
    return bus.read_i2c_block_data(addr, reg, n)

def bno_init(bus: SMBus, addr: int) -> None:
    bno_write8(bus, addr, REG_OPR_MODE, MODE_CONFIG); time.sleep(0.02)
    bno_write8(bus, addr, REG_PWR_MODE, PWR_NORMAL);  time.sleep(0.01)
    bno_write8(bus, addr, REG_UNIT_SEL, UNIT_MS2_DPS_DEG_C); time.sleep(0.01)
    bno_write8(bus, addr, REG_PAGE_ID, 0x00);         time.sleep(0.01)
    bno_write8(bus, addr, REG_OPR_MODE, MODE_IMU);    time.sleep(0.05)

def read_accel(bus: SMBus, addr: int):
    raw = bno_readN(bus, addr, REG_ACC_DATA_X, 6)
    x = to_signed_16(raw[0], raw[1]) / 100.0
    y = to_signed_16(raw[2], raw[3]) / 100.0
    z = to_signed_16(raw[4], raw[5]) / 100.0
    return x, y, z

def read_gyro(bus: SMBus, addr: int):
    raw = bno_readN(bus, addr, REG_GYR_DATA_X, 6)
    x = to_signed_16(raw[0], raw[1]) / 16.0
    y = to_signed_16(raw[2], raw[3]) / 16.0
    z = to_signed_16(raw[4], raw[5]) / 16.0
    return x, y, z

def read_euler(bus: SMBus, addr: int):
    raw = bno_readN(bus, addr, REG_EUL_HEADING, 6)
    h = to_signed_16(raw[0], raw[1]) / 16.0
    r = to_signed_16(raw[2], raw[3]) / 16.0
    p = to_signed_16(raw[4], raw[5]) / 16.0
    return h, r, p

def read_temp_c(bus: SMBus, addr: int) -> float:
    t = bus.read_byte_data(addr, REG_TEMP)
    if t >= 128:
        t -= 256
    return float(t)

def detect_sensors(bus: SMBus):
    """Return dict ch -> (addr or None)."""
    found = {}
    for ch in CHANNELS:
        mux_select(bus, ch)
        ok = None
        for addr in BNO_ADDRS:
            try:
                chip = bus.read_byte_data(addr, REG_CHIP_ID)
                if chip == 0xA0:
                    ok = addr
                    break
            except OSError:
                continue
        found[ch] = ok
    return found

def main():
    os.makedirs("logs", exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = f"logs/bno3_{ts}.csv"

    with SMBus(I2C_BUS_ID) as bus, open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t_ros_ns","sensor","ax","ay","az","gx","gy","gz","roll_deg","pitch_deg","yaw_deg","temp_C","mux_ch","addr"])

        # Probe
        mapping = detect_sensors(bus)
        print("[INFO] Probe result:")
        for ch in CHANNELS:
            addr = mapping.get(ch)
            if addr is None:
                print(f"  CH{ch}: not detected")
            else:
                print(f"  CH{ch}: BNO055 detected at 0x{addr:02X}")
        # Init where present
        for ch, addr in mapping.items():
            if addr is None:
                continue
            mux_select(bus, ch)
            try:
                bno_init(bus, addr)
                time.sleep(0.02)
            except OSError as e:
                print(f"[WARN] Init failed on CH{ch} addr 0x{addr:02X}: {e}")

        print("[INFO] Logging started →", out_path)
        running = True
        def handle_sigint(sig, frame):
            nonlocal running
            running = False
            print("\n[INFO] Stopping...")
        signal.signal(signal.SIGINT, handle_sigint)

        last_flush = 0
        period = 1.0 / TARGET_RATE_HZ
        next_t = time.perf_counter()

        while running:
            for ch in CHANNELS:
                addr = mapping.get(ch)
                if addr is None:
                    continue
                mux_select(bus, ch)
                t_ros_ns = time.time_ns()
                try:
                    ax, ay, az = read_accel(bus, addr)
                    gx, gy, gz = read_gyro(bus, addr)
                    yaw, roll, pitch = read_euler(bus, addr)
                    temp_c = read_temp_c(bus, addr)
                except OSError as e:
                    print(f"[WARN] read fail CH{ch}@0x{addr:02X}: {e}")
                    continue
                name = SENSOR_NAMES.get(ch, f"ch{ch}")
                writer.writerow([t_ros_ns, name, ax, ay, az, gx, gy, gz, roll, pitch, yaw, temp_c, ch, f"0x{addr:02X}"])
                last_flush += 1
                if last_flush >= FLUSH_EVERY:
                    f.flush(); os.fsync(f.fileno()); last_flush = 0

            next_t += period
            sleep_s = next_t - time.perf_counter()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_t = time.perf_counter()

        f.flush(); os.fsync(f.fileno())

if __name__ == "__main__":
    main()
