# daredevil — 3D Spatial Scanner

Turns a 2D spinning LiDAR into a 3D scanner. A Neato Botvac DX LiDAR unit sweeps the horizontal plane while a motorized second axis tilts the whole assembly, so a sensor that only ever saw one slice of the room now reconstructs the full space as a point cloud. A single scan produces 146,000+ filtered 3D points.

Firmware runs in C on a Raspberry Pi Pico; reconstruction and visualization run in Python on the host.

This is an upscale of an older project that used an ultrasonic sensor for the same idea, rebuilt around a real LiDAR unit for far denser, cleaner scans.

## How it works

```
LiDAR (UART) ──> Pico firmware ──> packet parse + checksum ──> (angle, distance, quality)
                      │
              stepper drives tilt axis ──> tracked stage angle
                      │
                      ▼
         host ──> polar + stage angle ──> Cartesian (x, y, z) ──> filtered point cloud ──> viewer
```

1. **Capture.** The Pico reads the LiDAR over UART and parses its 22-byte packets in C, verifying each checksum and extracting four sets of angle, distance, and quality readings per packet (`lidar_scan.c/.h`, `daredevil.c`).
2. **Sweep.** A stepper motor drives the second (tilt) axis, and the firmware tracks the stage angle so every 2D sweep is tagged with the elevation it was taken at (`spin.c/.h`, `continuous_scan.c`, `merged_scan.c`).
3. **Reconstruct.** On the host, each reading's polar distance is combined with the LiDAR scan angle and the tracked stepper position and converted into Cartesian coordinates, then filtered into the final point cloud (`visualization.py`, `continuous_live_scan.py`).
4. **Evaluate.** Helper scripts benchmark packet throughput and scan quality so changes to the firmware can be measured rather than guessed (`lidar_packet_benchmark.c`, `benchmark_scan_quality.py`, `evaluate_scan_quality.py`).

## Hardware

| Part | Role | Interface |
| --- | --- | --- |
| Neato Botvac DX LiDAR | 2D range sensing | UART |
| Raspberry Pi Pico | Firmware, packet parsing, motor control | — |
| Stepper motor | Tilt (second) axis | GPIO |
| Custom Fusion 360 mounts | Hold sensor alignment through the sweep | — |

The mounts were designed and 3D printed in Fusion 360 to keep the LiDAR aligned during rotation.

## Build

**Firmware (Raspberry Pi Pico, C / CMake):**

```bash
cmake -B build
cmake --build build
# copy the generated .uf2 onto the Pico in BOOTSEL mode
```

The Pico SDK is pulled in through `pico_sdk_import.cmake`.

**Visualization (host, Python):**

```bash
python visualization.py
# on Windows, run_visualization.ps1 wraps the same step
```

## Tech stack

C, Python, Raspberry Pi Pico, Pico SDK, CMake, UART, LiDAR, Fusion 360

## Status

Scanning and 3D reconstruction work end to end. Ongoing work is on scan quality and density, which the benchmarking scripts are there to measure.
