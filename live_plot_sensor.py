#!/usr/bin/env python3
"""
Live sensor plot – updated for 5‑column CSV:
  timestamp, raw_instant, moving_avg, median, ema

Watches the latest log_* folder and updates the plot in real time.
"""

import glob
import os
import sys

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import pandas as pd

# ---- Configuration ----
REFRESH_INTERVAL = 1000  # milliseconds

# ---- Find the latest log folder ----
folders = sorted(glob.glob("logs/log_*"))
if not folders:
    print("No log_* folder found. Run client.py first.")
    sys.exit(1)

folder = folders[-1]
sensor_file = os.path.join(folder, "sensor.csv")
print(f"Watching: {sensor_file}")

# ---- Set up the plot ----
plt.ion()
fig, ax = plt.subplots(figsize=(10, 5))
ax.set_xlabel("Time (ms)")
ax.set_ylabel("Force sensor value")
ax.set_title(f"Live sensor data – {folder}")
ax.grid(True, linestyle="--", alpha=0.5)

# Line objects (will be created on first update)
lines = {}  # key: column name, value: Line2D object

# Column names based on number of columns present
column_map = {
    3: ["timestamp", "raw_instant", "moving_avg"],
    4: ["timestamp", "raw_instant", "moving_avg", "median"],
    5: ["timestamp", "raw_instant", "moving_avg", "median", "ema"],
}

# Style for each curve
styles = {
    "raw_instant": {
        "label": "Raw instant",
        "alpha": 0.5,
        "linewidth": 1,
        "color": "C0",
    },
    "moving_avg": {
        "label": "Moving avg (10)",
        "alpha": 0.9,
        "linewidth": 1.5,
        "color": "C1",
    },
    "median": {
        "label": "Median (3)",
        "alpha": 0.9,
        "linewidth": 1.5,
        "linestyle": "--",
        "color": "C2",
    },
    "ema": {
        "label": "EMA (α=…)",
        "alpha": 0.9,
        "linewidth": 1.5,
        "linestyle": "-.",
        "color": "C3",
    },
}


def update_plot(frame):
    global lines

    try:
        df = pd.read_csv(
            sensor_file,
            names=column_map.get(5),  # try 5 columns; adjust if less
            on_bad_lines="skip",
        )
    except pd.errors.EmptyDataError:
        return

    # If fewer columns were actually read, adjust column names
    ncols = df.shape[1]
    if ncols in column_map:
        df.columns = column_map[ncols]
    else:
        # fallback: generic names
        df.columns = [f"col_{i}" for i in range(ncols)]
        print(f"Unexpected column count ({ncols}); using generic names.")

    # Ensure numeric types
    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    df = df.dropna(subset=["timestamp"])

    if df.empty:
        return

    # Plot each column that has a style defined
    for col in df.columns:
        if col == "timestamp":
            continue
        if col in styles:
            if col not in lines:
                (lines[col],) = ax.plot(df["timestamp"], df[col], **styles[col])
            else:
                lines[col].set_data(df["timestamp"], df[col])

    # Update legend when lines first appear
    if len(lines) > 0 and ax.get_legend() is None:
        ax.legend()

    ax.relim()
    ax.autoscale_view()
    fig.canvas.draw()
    fig.canvas.flush_events()


# ---- Run the animation ----
ani = animation.FuncAnimation(
    fig, update_plot, interval=REFRESH_INTERVAL, cache_frame_data=False
)
print(
    "\nLive plot running. Close the window or press Ctrl+C in the terminal to stop.\n"
)
plt.show(block=True)
