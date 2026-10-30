# CoVAPSy Webots Simulation

Webots R2025a simulation of the 1/10 TT-02 car on the CoVAPSy track. The controller runs the identical follow-the-gap brain as the real car (`common/gap_follower.py` + `common/config.py`), ensuring complete sim-to-real parity.

## Worlds
- `worlds/standalone.wbt`: Single car (`TT02_green`). Use this to develop, benchmark, and tune the algorithm.
- `worlds/multicar.wbt`: Four cars on a 2x2 grid, all running `covapsy_sim`. Race the same code against itself (each car sees the others as dynamic LiDAR obstacles).

## Quick Run

Launch Webots:
```bash
# From repository root:
./run_sim.sh
```
Or open manually in Webots R2025a (`File > Open World > simulation/worlds/standalone.wbt`).

1. Press **Play** in Webots.
2. Click inside the 3D viewport for keyboard focus:
   - **`A`**: Enable autonomous driving
   - **`N`**: Stop (zero speed and steering)

## Features & Controller Architecture

- **Driver + LiDAR**: `controllers/covapsy_sim/covapsy_sim.py` reads the `RpLidarA2` into the canonical 360-sample range array, calls `gap_follower.gap_follow()`, and sends steering and throttle to the `Driver` library.
- **GPS Lap Timer**: Includes an integrated `LapTimer` measuring crossing splits across the virtual start/finish line at $X = 0$, debouncing false triggers, and printing lap completions in ANSI color.
- **Steering Slew-Rate Limiter**: Smooths steering changes according to `max_steer_rate_dps` (180°/s) to mimic physical RC servo transit time, eliminating unphysical instantaneous tire scrub and front-wheel slip.
- **Supervisor Protocol**: Configured with `supervisor TRUE` on the TT-02 proto for headless batch execution, automated timeout detection, and programmatic test quitting (`driver.simulationQuit()`).

## Automated Headless Tuning

Tune parameters automatically using Latin Hypercube Sampling:
```bash
# Run 20 trials in headless fast mode with live leaderboard:
python3 tune_headless.py --trials 20
```
- Automatically utilizes dedicated GPU offload.
- Results are logged in real-time to `tuning_results.csv`.
