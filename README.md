# JSBSim Rocket Simulation
# This assumes you are using Windows with WSL
This project simulates a small suborbital amateur rocket using JSBSim flight dynamics engine.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [JSBSim Rocket Simulation](#jsbsim-rocket-simulation)
  - [Rocket Specifications](#rocket-specifications)
  - [Center of Gravity and Pressure](#center-of-gravity-and-pressure)
  - [Build Requirements](#build-requirements)
  - [Compilation](#compilation)
  - [Running the Simulation](#running-the-simulation)
  - [Output Files](#output-files)
  - [Analysis and Plotting](#analysis-and-plotting)
    - [Example Output](#example-output)
  - [Aircraft Configuration](#aircraft-configuration)
  - [Motor Characteristics](#motor-characteristics)
  - [Recovery System](#recovery-system)

<!-- markdown-toc end -->

## Rocket Specifications

- **Diameter**: 6 inches
- **Length**: 109 inches  
- **Dry Mass**: 43.0 lbs (including recovery system)
- **Wet Mass**: 46.9 lbs (with motor propellant)
- **Motor**: Cesaroni L1720 solid rocket motor
- **Expected Apogee**: ~3200 feet
- **Recovery**: Two-stage parachute system
  - Drogue chute at apogee
  - Main chute deployment at 500 feet

## Center of Gravity and Pressure

- **Center of Gravity**: 54.9 inches from nose
- **Center of Pressure**: 68.1 inches from nose  
- **Static Stability Margin**: 2.20 calibers (on pad), 2.28 calibers (rail exit)

## Build Requirements

- JSBSim library and headers ([Clone This Repository](https://github.com/jsbsim-team/jsbsim/tree/83e3df2c49c13eae78ab7853848cdd9e02c6c5ac) in the external directory)
- C++ compiler with C++11 support
- CMake (optional)

## Compilation

```bash
./build.sh
```

## Running the Simulation

Before you run the simulation, make sure you add the jsbsim path to your terminal so that the object file that was compiled can run

```bash
nano ~/.bashrc
```

Add this to the end of the bashrc profile
```bash
export LD_LIBRARY_PATH=/home/<user>/jsbsim-rocket/external/jsbsim/build/src:$LD_LIBRARY_PATH
```

Apply the changes to the shell
```bash
source ~/.bashrc
```
To run the simulation
```bash
./build/rocket_sim
```

The simulation will:
1. Initialize the rocket on the launch pad in vertical orientation
2. Ignite the motor at t=0.1 seconds
3. Track liftoff when velocity > 0 and altitude > 10 feet
4. Deploy drogue chute at apogee detection (velocity becomes negative)
5. Deploy main chute at 500 feet AGL
6. Output trajectory data to `rocket_trajectory.csv`

## Output Files

- `rocket_trajectory.csv`: Time-series data of altitude, velocity, and parachute states
- Console output: Real-time simulation status and key events

## Analysis and Plotting

The project includes Python tools for analyzing and visualizing the rocket trajectory data:

### Python Analysis Tools

1. **`analyze_trajectory.py`** - Comprehensive trajectory analysis
   - Calculates flight performance metrics (max altitude, velocity, flight time)
   - Analyzes parachute deployment events and descent rates
   - Computes horizontal drift and landing position
   - Provides detailed flight phase breakdown
   - Outputs trajectory samples for verification

2. **`plot_simple.py`** - 3D trajectory visualization
   - Creates 4-panel plot with 3D trajectory, altitude vs time, velocity vs time, and horizontal drift
   - Marks key events (launch, apogee, parachute deployments, landing)
   - Requires matplotlib: `pip install matplotlib`

### Usage

It's recommended to use a Python virtual environment to avoid conflicts with system packages:

```bash
# Create a virtual environment
python3 -m venv env

# Activate the virtual environment
# On macOS/Linux:
source env/bin/activate
# On Windows:
# env\Scripts\activate

# Install dependencies
pip install pyserial

# Run trajectory analysis
python process_npk.py

# Deactivate virtual environment when done
deactivate
```
