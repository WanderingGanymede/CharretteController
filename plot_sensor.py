#!/usr/bin/env python3
"""
Sensor plot – reads sensor.csv from the latest log_* folder and plots
all available data channels vs time.

Assumes no header, columns:
  timestamp, raw_instant, moving_avg, [median]

If only 3 columns are present, the 3rd is labelled "filtered".
"""

import glob
import os
import sys

import matplotlib.pyplot as plt
import pandas as pd

# ---- Find latest log folder ------------------------------------------------
folders = sorted(glob.glob("logs/log_*"))
if not folders:
    print("No log_* folder found. Run client.py first.")
    sys.exit(1)

folder = folders[-1]
sensor_file = os.path.join(folder, "sensor.csv")
if not os.path.exists(sensor_file):
    print(f"Sensor file not found: {sensor_file}")
    sys.exit(1)

print(f"Reading {sensor_file} ...")

# ---- Load CSV ----------------------------------------------------------------
# Read the whole file, line by line, to handle any stray S: prefix (shouldn't be there)
data = []
with open(sensor_file, "r") as f:
    for line in f:
        line = line.strip().replace("S:", "")  # just in case
        if line:
            data.append([float(x) for x in line.split(",")])

df = pd.DataFrame(data)

# Name columns based on how many we have
if df.shape[1] == 3:
    df.columns = ["timestamp", "raw_instant", "moving_avg"]
elif df.shape[1] == 4:
    df.columns = ["timestamp", "raw_instant", "moving_avg", "median"]
else:
    # generic fallback
    df.columns = [f"col_{i}" for i in range(df.shape[1])]
    print(f"Unexpected column count ({df.shape[1]}); using generic names.")

# ---- Plot --------------------------------------------------------------------
plt.figure(figsize=(10, 5))
plt.plot(
    df["timestamp"], df["raw_instant"], label="Raw instant", alpha=0.6, linewidth=1
)

if "moving_avg" in df.columns:
    plt.plot(
        df["timestamp"],
        df["moving_avg"],
        label="Moving avg (10)",
        alpha=0.9,
        linewidth=1.5,
    )

if "median" in df.columns:
    plt.plot(
        df["timestamp"],
        df["median"],
        label="Median (3)",
        alpha=0.9,
        linewidth=1.5,
        linestyle="--",
    )

plt.xlabel("Time (ms)")
plt.ylabel("Force sensor value")
plt.title(f"Sensor data – {folder}")
plt.legend()
plt.grid(True, linestyle="--", alpha=0.5)

# Save
out_png = os.path.join(folder, "sensor_plot.png")
plt.savefig(out_png, dpi=150, bbox_inches="tight")
print(f"Plot saved: {out_png}")
plt.show()
