# AI-assisted branch summary

AI was used to help inspect, edit, and verify this branch. The changes were reviewed in the project context and kept focused on the JSBSim HIL/simulation workflow.

This branch adds and updates the JSBSim rocket HIL simulation work for the VADL 2026-2027 workflow:

- Added a summer subscale JSBSim aircraft model under `aircraft/summer_subscale_2026/`, including engine, nozzle, controls, recovery, and landing-contact configuration.
- Added standalone and HIL summer subscale simulator entry points: `rocket_sim_summer_subscale_2026.cpp` and `hitl_sim_summer_subscale_2026.cpp`.
- Updated `CMakeLists.txt` so the new summer subscale standalone and HIL executables build with the existing JSBSim targets.
- Expanded the HIL packet path from the older acceleration/pressure packet to a VN100-like 48-byte packet containing yaw, pitch, roll, body acceleration, pressure, and gyro X/Y/Z with CRC.
- Updated `hitl_sim_midscale.cpp` so JSBSim body acceleration and body angular rates are mapped into the STM32/Teensy HIL packet.
- Added attitude replay support so recorded flight CSV attitude/gyro data can be replayed through the HIL path for Teensy roll-detection testing.
- Added summer subscale initial-condition logic, motor burn timing, single-chute recovery behavior, wind setup, and output CSV selection.
- Added RACS/roll-related simulation properties and CSV output fields so roll behavior can be inspected by the plotting and analyzer tools.
- Updated plotting/analyzer helper scripts to default toward the HIL midscale data and support the newer HIL output fields.
- Added `convert_jsbsim_to_flight_analyzer.py` to convert JSBSim trajectory outputs into the Flight Analyzer CSV format.

Generated trajectory CSVs and PNG plots are intentionally not part of the core branch summary unless explicitly committed as reference artifacts.

---

# HIL Development Notes

Last updated: 2026-07-20

This document summarizes the current Hardware-in-the-Loop development state for the
JSBSim -> STM32 -> Teensy flight-computer pipeline. It is meant to capture what has
been changed, what is physically modeled, what is still simplified, and which data
streams should be trusted right now.

## System architecture

The current test chain has three layers:

1. JSBSim runs the rocket dynamics simulation on the PC.
2. The PC-side HIL executable sends a VN100-like binary packet to the STM32.
3. The STM32 forwards/mimics the VN100 data stream to the Teensy flight computer.
4. The Teensy decodes the VN100-style packet, estimates flight state, logs CSV data,
   and commands fin deployment logic.

That means the plotted data can come from different sources:

- JSBSim truth state: direct simulator state such as altitude, vertical velocity,
  body acceleration, Euler angles, and body rates.
- STM32/VN packet data: the values packed by the PC HIL simulator and sent through
  the STM32.
- Teensy-estimated data: values the Teensy computes from the received packet, such
  as barometer altitude, barometer velocity, AGL altitude, fused velocity, phase,
  roll accumulation, and event markers.

When debugging, always identify which layer produced the field being plotted.

## Data currently sent by HIL

The HIL midscale/summer simulator now sends a 48-byte VN-style packet.

The packet contains:

- yaw angle, degrees
- pitch angle, degrees
- roll angle, degrees
- acceleration X, m/s^2
- acceleration Y, m/s^2
- acceleration Z, m/s^2
- pressure, kPa
- gyro X, rad/s
- gyro Y, rad/s
- gyro Z, rad/s
- CRC

The current HIL packet is built in:

- `hitl_sim_midscale.cpp`

The packet send happens at the end of each simulation step:

```cpp
build_packet(output_packet, yaw_deg, pitch_deg, roll_deg,
             vn_ax, vn_ay, vn_az, pressure,
             gyro_x, gyro_y, gyro_z);
stm32.send(output_packet, PACKET_LEN);
```

## What is currently accurate enough to trust

The following fields are the most trustworthy right now.

### Altitude / pressure-derived altitude

This is currently the best validated HIL signal.

JSBSim truth altitude is converted into pressure using a standard atmosphere
equation. The Teensy receives pressure, converts it back into altitude, and logs:

- `Altitude`
- `Filtered Altitude`
- `AGL Altitude`
- `Max AGL Altitude`
- `Pressure (kPa)`

This path is not perfect, because pressure conversion and filtering can introduce
small differences, but it is conceptually correct and currently the most stable
signal in the system.

Important distinction:

- JSBSim `Altitude` in the PC CSV is simulator truth.
- Teensy `Altitude` is reconstructed from pressure.
- Teensy `Barometer Velocity` is not JSBSim truth velocity. It is differentiated
  altitude and can be noisy.

### Gyro X / roll rate during boost and early coast

This is useful for roll-detection testing during the early flight window.

JSBSim body rate `P` is mapped to VN/Teensy `gyro_x`:

```cpp
float gyro_x = static_cast<float>(fdmExec->GetPropagate()->GetPQR(1));
```

The Teensy roll detection integrates `gyro_x`:

```cpp
rocketState.roll_detected += rocketState.gyro_x *
                              (rocketState.vn100_dt * 1.0e-9f);
```

This is the right quantity for the current Helix roll-detection logic. The flight
software is not relying on Euler roll angle directly; it uses raw gyro X integrated
over the roll-detection window.

Current limitation: the JSBSim roll physics is still simplified. The model uses a
fixed RACS fin cant to generate roll torque. This is physics-based, but it is not
yet a full CFD/OpenRocket-grade roll model.

### Longitudinal acceleration during boost

Acceleration X is useful enough for launch-detect and boost/coast behavior checks.

The HIL sim currently maps JSBSim body acceleration into the VN-like packet as:

```cpp
float jsb_x_forward = static_cast<float>(a_body(1)) * FT_TO_M;
float jsb_y_right   = static_cast<float>(a_body(2)) * FT_TO_M;
float jsb_z_down    = static_cast<float>(a_body(3)) * FT_TO_M;

float vn_ax = jsb_x_forward;
float vn_ay = jsb_y_right;
float vn_az = -jsb_z_down;
```

This means `accel_x` is intended to be the rocket longitudinal/boost axis used by
the Teensy flight logic.

Current limitation: acceleration depends strongly on the aerodynamic and propulsion
model. It is good for HIL behavior tests, but should not yet be treated as a fully
validated sensor replica.

## Data that is not fully accurate yet

### Barometer Velocity

The spiky velocity plots labeled `barometer` are not direct JSBSim velocity.

The Teensy computes barometer velocity by differentiating pressure-derived altitude:

```cpp
state.barometer_velocity = getVerticalVel(state.altitude, dt);
```

and:

```cpp
return (altitude - prev_altitude) / dt;
```

Differentiating altitude amplifies noise, quantization, repeated samples, timing
jitter, and pressure conversion artifacts. This explains the spikes and zero-hold
behavior seen in the Velocity plot.

To compare against JSBSim truth, use the PC simulator CSV field:

- `Vertical_Velocity`

Do not use Teensy `Barometer Velocity` as a clean JSBSim truth comparison.

### Yaw, pitch, gyro Y, and gyro Z

These are physically generated by JSBSim, but the current rocket aero model is still
too simple to trust them as real-flight-accurate.

Reasons:

- The pitch/yaw restoring coefficients are rough.
- The aerodynamic center and damping are simplified.
- The rail, wind, and launch angle model are approximate.
- The current RACS/ACS surfaces are coefficient-level approximations, not full
  individual fin hinge/flow models.

These values are useful for seeing that the packet path works, but not yet for
validating true yaw/pitch flight behavior.

### Roll angle during and after descent

Euler roll angle can look like it ramps, wraps, or freezes depending on body rates
and the attitude frame. The flight code cares more about `gyro_x` during the early
roll-detection window than about Euler roll after parachute deployment.

After chute deployment, the current JSBSim model does not simulate random canopy
motion, shock cord twisting, asymmetric parachute inflation, or swinging. It now
adds angular damping so rates decay instead of spinning forever.

Real flights can look erratic under chute. JSBSim will only reproduce that if we
explicitly add parachute asymmetry or swinging/tumbling moments.

## Major changes made so far

### STM32 flashing and basic communication

The STM32 firmware was built with the ARM embedded compiler:

```bash
make CC=arm-none-eabi-gcc
```

The board was flashed successfully using `flash.sh` with ST-Link attached to WSL.
The STM32 was verified over `/dev/ttyACM0` by observing `HELLO` on reset.

Key lesson:

- Only one process can own `/dev/ttyACM0` at a time. If `screen`, `cat`, or a serial
  monitor is open, the HIL simulator cannot open the port.

### USART roles

Current STM32 communication roles:

- USART3 talks to the host PC over the ST-Link virtual COM port.
- USART2 is the plant/Teensy-facing output side.

The HIL PC simulator talks to STM32 through `/dev/ttyACM0`. The Teensy receives the
VN100-like data stream from the STM32 side.

### STM32 receive LED test

A receive-data LED toggle was added earlier to confirm STM32 was receiving bytes.
That proved the PC-to-STM32 side was alive before debugging packet content.

### VN100 packet alignment

The original HIL packet was too short and did not match what the Teensy VN100 decode
logic expected.

The HIL packet was expanded from 36 bytes to 48 bytes to include gyro X/Y/Z in
addition to yaw, pitch, roll, acceleration, and pressure.

This was necessary because the Helix flight code uses raw gyro X for roll detection.
Without gyro X in the packet, roll detection could not be realistic.

### Teensy live output alignment

The Teensy `main.cpp` live print was adjusted to show the fields needed for HIL
debugging:

- time
- phase
- altitude
- accel X/Y/Z
- gyro X/Y/Z
- pressure
- AGL altitude
- VN timing
- packet count
- CRC/parse errors
- preflight sample count
- roll_detected
- fins_actuated

This made it possible to watch data live and verify that packet decode was working.

### Flight Analyzer GUI

A PyQt6 desktop app named `Flight Analyzer` was developed from the old
`recent_flight.py` plotting backend.

It added:

- automatic SD/removable drive detection
- newest flight CSV loading
- archive into `FlightLogs`
- interactive matplotlib plots
- graph tree and tabs
- series toggles
- event toggles
- unit conversion
- time windows
- plot export
- PDF report support
- summary panel

Important plotting lesson:

- Some plots are raw fields.
- Some plots are derived fields.
- The legend often tells the truth. If velocity says `barometer`, it is derived from
  pressure/altitude, not direct JSBSim truth.

### Summer subscale aircraft model

A new JSBSim aircraft was created:

- `aircraft/summer_subscale_2026/summer_subscale_2026.xml`
- `aircraft/summer_subscale_2026/Engines/aerotech_i600r_engine.xml`
- `aircraft/summer_subscale_2026/Engines/i600r_nozzle.xml`

Dedicated executables were added:

- `rocket_sim_summer_subscale`
- `hitl_sim_summer_subscale`

The summer subscale model uses:

- Aerotech I600R motor
- 15.1 lb wet mass target
- 13.8 lb dry mass target
- one recovery chute
- Cape Canaveral-ish launch site: 28.61 N, -80.6 W
- 2 m/s wind
- 100 cm launch rail
- 5 deg launch angle into the wind axis

### Motor and mass correction

The I600R motor was corrected using OpenRocket data:

- total impulse about 640 N*s
- burn time about 1.12 s
- launch motor mass about 617 g
- empty motor mass about 293 g

JSBSim solid rocket thrust tables use total propellant expended as the independent
variable, not time. That was an important fix.

The modeled rocket mass was adjusted so:

- airframe/no motor is about 13.8 lb
- motor casing is about 0.646 lb
- propellant is about 0.714 lb
- wet mass is about 15.1 lb

### Nozzle correction

The I600R thrust curve from OpenRocket already represents delivered thrust. JSBSim's
nozzle model subtracts ambient-pressure force from vacuum thrust. To avoid double
correcting the thrust curve, nozzle area was set near zero.

This fixed a major apogee/thrust mismatch.

### Apogee matching

After motor and mass fixes, the summer standalone sim was tuned to roughly match the
OpenRocket apogee target.

Recent standalone result:

- maximum altitude about 1204.8 ft

This is close to the OpenRocket screenshot value near 363 m / 1191 ft.

### RACS fin model

RACS fin geometry from OpenRocket was added as aerodynamic properties:

- 4 fins
- position 121 cm from nose tip
- thickness 0.483 cm
- points: (0,0), (2.54,10.2), (7.62,10.2), (10.2,0) cm
- component mass overridden to 0 g

A simplified roll model was added:

- fixed 2 deg same-handed fin cant creates roll torque
- `metrics/racs-roll-effectiveness` scales the simplified moment
- active fin commands exist as properties but currently remain zero

This lets JSBSim generate roll rate from aerodynamic moment instead of scripting a
roll path.

### RACS cant gate

The fixed RACS cant was originally active for the whole flight, so it kept injecting
roll torque even after chute deployment.

That was changed by adding:

- `aero/RACS_cant_active`

The C++ sim sets this to:

- `1.0` at initialization
- `0.0` after ACS/chute deployment

This keeps the roll-driving fin cant active during ascent/early flight but prevents
it from spinning the rocket forever during descent.

### Chute angular damping

The chute model used to be pure drag at the CG. That creates descent drag but almost
no angular torque.

Additional damping moments were added:

- `Pitch_chute_damp`
- `Roll_chute_damp`
- `Yaw_chute_damp`

These make angular rates decay after recovery deployment. They do not simulate random
canopy motion; they only stop unrealistic endless spin.

## Current command set

Build:

```bash
cd ~/jsbsim-rocket-hitl
./build.sh
```

Run standalone summer subscale:

```bash
cd ~/jsbsim-rocket-hitl
LD_LIBRARY_PATH=~/jsbsim-rocket-hitl/external/jsbsim/build/src ./build/rocket_sim_summer_subscale
```

Run HIL summer subscale:

```bash
cd ~/jsbsim-rocket-hitl
LD_LIBRARY_PATH=~/jsbsim-rocket-hitl/external/jsbsim/build/src ./build/hitl_sim_summer_subscale
```

Expected output files:

- `summer_subscale_trajectory.csv`
- `hitl_summer_subscale_trajectory.csv`

## Current known issues

### Build output vs source files

Do not edit files under:

```text
build/aircraft/...
```

Those are generated/copied build artifacts. Edit the source aircraft model under:

```text
aircraft/summer_subscale_2026/...
```

### HIL altitude mismatch risk

If Flight Analyzer shows apogee around 29 m while standalone JSBSim shows around
1200 ft, check that:

- the newest HIL CSV is being loaded
- the graph is not plotting an old Teensy flight
- units are not mixed
- the Teensy pressure-to-altitude reconstruction is using the expected pressure
- the HIL executable being run is `hitl_sim_summer_subscale`, not a legacy midscale
  executable

### Velocity plots

The `Barometer Velocity` plot is not a JSBSim truth velocity. It is computed on the
Teensy from altitude difference over time and can be very noisy.

For truth comparison, use PC-side:

- `Vertical_Velocity`

For Teensy estimator behavior, use:

- `Barometer Velocity`
- `Velocity North`
- `Velocity East`
- `Velocity Down`

### Descent attitude is simplified

Real parachute descent can be erratic. The current model damps angular rates after
chute deployment but does not simulate random swinging or asymmetric canopy loads.

To reproduce erratic descent behavior later, add one or more of:

- off-CG chute attach point
- oscillating chute side force
- asymmetric chute drag
- stochastic canopy torque
- shock-cord pendulum model

## Recommended next steps

1. Verify the HIL plot is using `hitl_summer_subscale_trajectory.csv`.
2. Compare PC truth altitude/vertical velocity against Teensy altitude/barometer
   velocity.
3. Calibrate `metrics/racs-roll-effectiveness` against real boost-phase gyro X.
4. Tune pitch/yaw restoring and damping coefficients.
5. Decide whether descent should be deterministic damping only or include explicit
   parachute chaos.
6. Keep the HIL packet stable while tuning the model so packet decode bugs do not
   get confused with physics-model bugs.
