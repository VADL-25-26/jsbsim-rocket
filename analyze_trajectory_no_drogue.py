#!/usr/bin/env python3
"""
Simple Rocket Trajectory Analyzer (No Drogue)
Analyzes CSV trajectory data and prints flight statistics.
"""

import csv
import math
import sys
import os

def analyze_trajectory(csv_file='rocket_trajectory.csv'):
    """
    Analyze rocket trajectory from CSV data and print statistics
    """
    if not os.path.exists(csv_file):
        print(f"❌ Error: CSV file '{csv_file}' not found.")
        print("   Usage: python3 analyze_trajectory_no_drogue.py <csv_file>")
        return

    # Load CSV data
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        data = list(reader)

    if not data:
        print(f"❌ Error: '{csv_file}' is empty.")
        return

    print(f"✅ Loaded {len(data)} data points from {csv_file}")

    # Convert data to lists
    time = [float(row['Time']) for row in data]
    x_ft = [float(row['X_ft']) for row in data]
    y_ft = [float(row['Y_ft']) for row in data]
    z_ft = [float(row['Z_ft']) for row in data]
    altitude = [float(row['Altitude']) for row in data]
    vertical_velocity = [float(row['Vertical_Velocity']) for row in data]
    main = [int(row['Main_Deployed']) for row in data]

    # Key statistics
    max_altitude = max(z_ft)
    max_alt_idx = z_ft.index(max_altitude)
    max_alt_time = time[max_alt_idx]

    max_velocity = max(vertical_velocity)
    max_vel_idx = vertical_velocity.index(max_velocity)
    max_vel_time = time[max_vel_idx]

    flight_time = time[-1]

    # Horizontal drift
    horizontal_dist = [math.sqrt(x**2 + y**2) for x, y in zip(x_ft, y_ft)]
    max_horizontal_drift = max(horizontal_dist)
    final_horizontal_drift = horizontal_dist[-1]

    # Parachute deployment
    main_time = None
    main_altitude = None
    for i in range(1, len(main)):
        if main[i] == 1 and main[i-1] == 0:
            main_time = time[i]
            main_altitude = z_ft[i]
            break

    # === Results ===
    print(f"\n{'='*55}")
    print(f"🚀 ROCKET TRAJECTORY ANALYSIS (NO DROGUE)")
    print(f"{'='*55}")

    print(f"\n📊 FLIGHT PERFORMANCE:")
    print(f"  Max Altitude:     {max_altitude:8.1f} ft at t={max_alt_time:.1f}s")
    print(f"  Max Velocity:     {max_velocity:8.1f} ft/s at t={max_vel_time:.1f}s")
    print(f"  Total Flight Time: {flight_time:7.1f} s")
    print(f"  Landing Velocity:  {vertical_velocity[-1]:7.1f} ft/s")

    print(f"\n🪂 PARACHUTE DEPLOYMENT:")
    if main_time:
        print(f"  Main Deploy:      t={main_time:6.1f}s at {main_altitude:6.0f} ft")
    else:
        print("  No main deployment detected.")

    print(f"\n📍 TRAJECTORY ANALYSIS:")
    print(f"  Launch Position:   X=0.0 ft, Y=0.0 ft")
    print(f"  Landing Position:  X={x_ft[-1]:6.1f} ft, Y={y_ft[-1]:6.1f} ft")
    print(f"  Horizontal Drift:  {final_horizontal_drift:6.1f} ft")
    print(f"  Max Drift:         {max_horizontal_drift:6.1f} ft")

    # Descent rate
    if main_time:
        main_idx = time.index(main_time)
        main_alt_drop = z_ft[main_idx] - z_ft[-1]
        main_time_period = flight_time - main_time
        main_descent_rate = main_alt_drop / main_time_period
        print(f"\n⬇️  DESCENT RATE:")
        print(f"  Main Descent:     {main_descent_rate:6.1f} ft/s")

    # Flight phases
    print(f"\n⏱️  FLIGHT PHASES:")
    if main_time:
        print(f"  Powered/Coast:    0.0s - {main_time:.1f}s ({main_time:.1f}s)")
        print(f"  Main Descent:     {main_time:.1f}s - {flight_time:.1f}s ({flight_time-main_time:.1f}s)")
    else:
        print(f"  Flight Duration:  0.0s - {flight_time:.1f}s")

    # Sample trajectory points
    print(f"\n📋 TRAJECTORY SAMPLE (every ~10 seconds):")
    print(f"{'Time':>6} {'X':>8} {'Y':>8} {'Z':>8} {'Vel':>8} {'Phase'}")
    print(f"{'(s)':>6} {'(ft)':>8} {'(ft)':>8} {'(ft)':>8} {'(ft/s)':>8}")
    print(f"{'-'*55}")

    sample_interval = max(1, len(time)//10)  # ~10 samples across flight
    for i in range(0, len(time), sample_interval):
        t = time[i]
        phase = "Launch"
        if main_time and t >= main_time:
            phase = "Main"
        if i == len(time)-1:
            phase = "Land"
        print(f"{t:6.1f} {x_ft[i]:8.1f} {y_ft[i]:8.1f} {z_ft[i]:8.1f} {vertical_velocity[i]:8.1f} {phase}")

if __name__ == "__main__":
    # Allow CSV filename as command-line argument
    if len(sys.argv) > 1:
        analyze_trajectory(sys.argv[1])
    else:
        analyze_trajectory()