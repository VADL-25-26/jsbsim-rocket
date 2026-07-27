#!/usr/bin/env python3
"""
Simple 3D Rocket Trajectory Plotter (No Drogue)
"""

import csv
import matplotlib
matplotlib.use("TkAgg")      # Use "Agg" if running without GUI
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


def plot_rocket_trajectory(csv_file="hitl_midscale_trajectory.csv"):

    try:
        with open(csv_file, "r") as f:
            reader = csv.DictReader(f)
            data = list(reader)

        print(f"Loaded {len(data)} data points from {csv_file}")

    except FileNotFoundError:
        print(f"Error: {csv_file} not found.")
        return

    # -----------------------------
    # Convert CSV columns
    # -----------------------------
    time = [float(row["Time"]) for row in data]

    x_ft = [float(row["X_ft"]) for row in data]
    y_ft = [float(row["Y_ft"]) for row in data]
    z_ft = [float(row["Z_ft"]) for row in data]

    altitude = [float(row["Altitude"]) for row in data]
    vertical_velocity = [float(row["Vertical_Velocity"]) for row in data]

    vertical_acceleration = [
    float(row["Vertical_Acceleration"])
    for row in data
    ]

    main = [int(row["Main_Deployed"]) for row in data]

    horizontal_dist = [np.sqrt(x**2 + y**2) for x, y in zip(x_ft, y_ft)]

    max_alt_idx = np.argmax(z_ft)

    # -----------------------------
    # Find main deployment
    # -----------------------------
    main_time = None
    main_altitude = None

    for i in range(1, len(main)):
        if main[i] == 1 and main[i - 1] == 0:
            main_time = time[i]
            main_altitude = z_ft[i]
            break

    # -----------------------------
    # Figure
    # -----------------------------
    fig = plt.figure(figsize=(18,10))

    # ==========================================================
    # 1) 3D TRAJECTORY
    # ==========================================================
    ax1 = fig.add_subplot(231, projection="3d")

    ax1.plot(
        x_ft,
        y_ft,
        z_ft,
        color="blue",
        linewidth=2,
        label="Flight Path"
    )

    ax1.scatter(
        x_ft[0],
        y_ft[0],
        z_ft[0],
        color="black",
        s=80,
        label="Launch"
    )

    ax1.scatter(
        x_ft[-1],
        y_ft[-1],
        z_ft[-1],
        color="red",
        marker="X",
        s=100,
        label="Landing"
    )

    ax1.scatter(
        x_ft[max_alt_idx],
        y_ft[max_alt_idx],
        z_ft[max_alt_idx],
        color="gold",
        marker="*",
        s=180,
        label=f"Apogee ({max(z_ft):.0f} ft)"
    )

    ax1.set_title("3D Rocket Trajectory")
    ax1.set_xlabel("X [ft]")
    ax1.set_ylabel("Y [ft]")
    ax1.set_zlabel("Altitude [ft]")
    ax1.legend()

    # ==========================================================
    # 2) ALTITUDE VS TIME
    # ==========================================================
    ax2 = fig.add_subplot(232)

    ax2.plot(time, z_ft, linewidth=2)

    ax2.axhline(
        max(z_ft),
        linestyle="--",
        color="red",
        label=f"Max Alt: {max(z_ft):.0f} ft"
    )

    if main_time:
        ax2.axvline(
            main_time,
            linestyle="--",
            color="green",
            label="Main Deploy"
        )

    ax2.set_title("Altitude vs Time")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Altitude [ft]")
    ax2.grid(True)
    ax2.legend()

    # ==========================================================
    # 3) ALTITUDE VS DRIFT
    # ==========================================================
    ax3 = fig.add_subplot(233)

    ax3.plot(horizontal_dist, z_ft, linewidth=2)

    ax3.scatter(
        horizontal_dist[0],
        z_ft[0],
        color="black",
        s=80
    )

    ax3.scatter(
        horizontal_dist[max_alt_idx],
        z_ft[max_alt_idx],
        color="gold",
        marker="*",
        s=180
    )

    ax3.scatter(
        horizontal_dist[-1],
        z_ft[-1],
        color="red",
        marker="X",
        s=100
    )

    ax3.set_title("Altitude vs Horizontal Drift")
    ax3.set_xlabel("Horizontal Drift [ft]")
    ax3.set_ylabel("Altitude [ft]")
    ax3.grid(True)

    # ==========================================================
    # 4) VELOCITY VS TIME
    # ==========================================================
    ax4 = fig.add_subplot(234)

    ax4.plot(
        time,
        vertical_velocity,
        color="green",
        linewidth=2
    )

    ax4.set_title("Velocity vs Time")
    ax4.set_xlabel("Time [s]")
    ax4.set_ylabel("Velocity [ft/s]")
    ax4.grid(True)

    # ==========================================================
    # 5) DRIFT VS TIME
    # ==========================================================
    ax5 = fig.add_subplot(235)

    ax5.plot(
        time,
        horizontal_dist,
        color="magenta",
        linewidth=2
    )

    ax5.set_title("Horizontal Drift vs Time")
    ax5.set_xlabel("Time [s]")
    ax5.set_ylabel("Horizontal Drift [ft]")
    ax5.grid(True)

    # ==========================================================
    # 6) ACCELERATION VS TIME
    # ==========================================================

    ax6 = fig.add_subplot(236)

    ax6.plot(
        time,
        vertical_acceleration,
        color="red",
        linewidth=2
    )

    ax6.axhline(
        0,
        color="black",
        linestyle="--",
        linewidth=1
    )

    ax6.set_title("Vertical Acceleration vs Time")
    ax6.set_xlabel("Time [s]")
    ax6.set_ylabel("Acceleration [ft/s²]")
    ax6.grid(True)

    plt.tight_layout()

    print("\n=== FLIGHT STATISTICS ===")
    print(f"Max Altitude: {max(z_ft):.1f} ft")
    print(f"Flight Time: {time[-1]:.1f} s")
    print(f"Max Velocity: {max(vertical_velocity):.1f} ft/s")
    print(f"Horizontal Drift: {horizontal_dist[-1]:.1f} ft")

    if main_time:
        print(f"Main Deploy: {main_time:.1f} s at {main_altitude:.1f} ft")

    plt.savefig("hitl_midscale_plot.png", dpi=300)
    print("Saved figure.")
    plt.show()

if __name__ == "__main__":
    plot_rocket_trajectory("hitl_midscale_trajectory.csv")