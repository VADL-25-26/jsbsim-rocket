#!/usr/bin/env python3
"""Convert the standalone summer-subscale trajectory to Helix analyzer CSV."""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path


FT_TO_M = 0.3048
MOTOR_BURN_TIME_S = 1.12


def number(row: dict[str, str], column: str) -> float:
    return float(row[column])


def main() -> int:
    if len(sys.argv) != 3:
        print(f"Usage: {Path(sys.argv[0]).name} INPUT.csv OUTPUT.csv")
        return 2

    source = Path(sys.argv[1])
    destination = Path(sys.argv[2])

    with source.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise RuntimeError(f"No trajectory rows found in {source}")

    required = {
        "Time",
        "X_ft",
        "Y_ft",
        "Z_ft",
        "Altitude",
        "Vertical_Velocity",
        "Vertical_Acceleration",
        "Body_Accel_X_ft_s2",
        "Body_Accel_Y_ft_s2",
        "Body_Accel_Z_ft_s2",
        "Drogue_Deployed",
        "Roll_deg",
        "Pitch_deg",
        "Yaw_deg",
        "P_rad_s",
        "Q_rad_s",
        "R_rad_s",
        "RACS_Cant_deg",
        "RACS_Cmd_deg",
    }
    missing = required.difference(rows[0])
    if missing:
        raise RuntimeError(f"Missing source columns: {sorted(missing)}")

    pad_altitude_m = number(rows[0], "Altitude") * FT_TO_M
    agl_values = [
        number(row, "Altitude") * FT_TO_M - pad_altitude_m for row in rows
    ]
    apogee_index = max(range(len(rows)), key=agl_values.__getitem__)

    landing_index = len(rows)
    for index in range(apogee_index + 1, len(rows)):
        if agl_values[index] <= 1.0:
            landing_index = index
            break

    output_fields = [
        "System Time (ms)",
        "Phase",
        "Fins Deployed",
        "Fin Deployment Time (ms)",
        "Fin Return Time (ms)",
        "Burnout Time (ms)",
        "Altitude",
        "Filtered Altitude",
        "AGL Altitude",
        "Max AGL Altitude",
        "Barometer Velocity",
        "Velocity Down",
        "Position North",
        "Position East",
        "Position Down",
        "Linear Accel NED Down",
        "Accel X",
        "Accel Y",
        "Accel Z",
        "Linear Accel Body X",
        "Linear Accel Body Y",
        "Linear Accel Body Z",
        "Yaw (deg)",
        "Pitch (deg)",
        "Roll (deg)",
        "Gyro X",
        "Gyro Y",
        "Gyro Z",
        "RACS Cant (deg)",
        "RACS Command (deg)",
        "Wind East (m/s)",
    ]

    destination.parent.mkdir(parents=True, exist_ok=True)
    running_max_agl = 0.0
    with destination.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=output_fields)
        writer.writeheader()

        for index, row in enumerate(rows):
            time_s = number(row, "Time")
            altitude_m = number(row, "Altitude") * FT_TO_M
            agl_m = altitude_m - pad_altitude_m
            running_max_agl = max(running_max_agl, agl_m)

            if index >= landing_index:
                phase = 5  # LANDED
            elif index > apogee_index:
                phase = 4  # DESCENT
            elif time_s > MOTOR_BURN_TIME_S:
                phase = 3  # COAST
            else:
                phase = 2  # BURN

            vertical_velocity_mps = number(row, "Vertical_Velocity") * FT_TO_M
            upward_acceleration_mps2 = (
                number(row, "Vertical_Acceleration") * FT_TO_M
            )
            body_accel_x_mps2 = number(row, "Body_Accel_X_ft_s2") * FT_TO_M
            body_accel_y_mps2 = number(row, "Body_Accel_Y_ft_s2") * FT_TO_M
            body_accel_z_mps2 = number(row, "Body_Accel_Z_ft_s2") * FT_TO_M

            writer.writerow(
                {
                    "System Time (ms)": round(time_s * 1000.0),
                    "Phase": phase,
                    "Fins Deployed": 0,
                    "Fin Deployment Time (ms)": 0,
                    "Fin Return Time (ms)": 0,
                    "Burnout Time (ms)": (
                        round(MOTOR_BURN_TIME_S * 1000.0)
                        if time_s >= MOTOR_BURN_TIME_S
                        else 0
                    ),
                    "Altitude": altitude_m,
                    "Filtered Altitude": altitude_m,
                    "AGL Altitude": agl_m,
                    "Max AGL Altitude": running_max_agl,
                    "Barometer Velocity": vertical_velocity_mps,
                    "Velocity Down": -vertical_velocity_mps,
                    "Position North": number(row, "X_ft") * FT_TO_M,
                    "Position East": number(row, "Y_ft") * FT_TO_M,
                    "Position Down": -number(row, "Z_ft") * FT_TO_M,
                    "Linear Accel NED Down": -upward_acceleration_mps2,
                    "Accel X": body_accel_x_mps2,
                    "Accel Y": body_accel_y_mps2,
                    "Accel Z": body_accel_z_mps2,
                    "Linear Accel Body X": body_accel_x_mps2,
                    "Linear Accel Body Y": body_accel_y_mps2,
                    "Linear Accel Body Z": body_accel_z_mps2,
                    "Yaw (deg)": number(row, "Yaw_deg"),
                    "Pitch (deg)": number(row, "Pitch_deg"),
                    "Roll (deg)": number(row, "Roll_deg"),
                    "Gyro X": number(row, "P_rad_s"),
                    "Gyro Y": number(row, "Q_rad_s"),
                    "Gyro Z": number(row, "R_rad_s"),
                    "RACS Cant (deg)": number(row, "RACS_Cant_deg"),
                    "RACS Command (deg)": number(row, "RACS_Cmd_deg"),
                    "Wind East (m/s)": 7.5 * 1609.344 / 3600.0,
                }
            )

    duration_s = number(rows[-1], "Time")
    apogee_m = agl_values[apogee_index]
    if not math.isfinite(apogee_m):
        raise RuntimeError("Converted apogee is not finite")
    print(f"Wrote {len(rows)} samples to {destination}")
    print(f"Duration: {duration_s:.2f} s; AGL apogee: {apogee_m:.2f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
