# Anahera Guardian ADAS – Alerting Core

Anahera Guardian ADAS is a demonstration Advanced Driver Assistance System (ADAS) module written in C.  
It models an alerting core for a central NVIDIA-based ECU on a 2023–2026 Mercedes S-Class–style vehicle, detecting sudden objects in the ego lane and issuing warning and brake requests in the 30–260 km/h speed range.

> ⚠️ Safety notice  
> This software is **experimental** and provided for **research and educational purposes only**.  
> It is **not safety‑certified**, has not undergone any automotive SPICE / ISO 26262 process, and **must not be used in real vehicles without proper validation, safety engineering, and certification by qualified experts**.

---

## Author

- **Max Gecse**

---

## Features

- Ego‑lane model based on an HD‑map lane description (polynomial lane centerline and lane widths).  
- Object list interface with classification (vehicle, pedestrian, cyclist, static obstacle) and confidence scores.  
- Time‑to‑collision (TTC)–based alert logic with configurable thresholds for:
  - Warning
  - Soft brake
  - Hard brake  
- Speed range gating (active only between configurable minimum and maximum ego speeds).  
- Plausibility checks on ego state, lane model, and object list.  
- Simple sudden‑appearance detection using per‑object history.  
- Optional redundant “secondary path” for TTC estimation and mismatch detection.  

---

## Architecture overview

The core logic is implemented in `anahera_alerting_system.c` as a small C library with a clear public API.

- **Input types**
  - `AnaheraObjectList_t`: List of tracked objects in ego coordinates.  
  - `AnaheraEgoState_t`: Ego vehicle state (speed, yaw rate, steering, ABS/AEB flags).  
  - `AnaheraLaneModel_t`: HD‑map–based ego‑lane description.  

- **Configuration**
  - `AnaheraConfig_t`: Calibration data such as speed band, detection range, TTC thresholds, and plausibility limits.  

- **Output**
  - `AnaheraAlert_t`: Current alert level, requested deceleration, most critical object ID, TTC, and safety flags (status, watchdog counter, redundancy mismatch).  

Core public functions:

```c
void Anahera_Init(const AnaheraConfig_t* cfg);
void Anahera_Update(const AnaheraObjectList_t* objs,
                    const AnaheraEgoState_t*   ego,
                    const AnaheraLaneModel_t*  lane,
                    AnaheraAlert_t*            outAlert);

void Anahera_ConfigureAndInit(void);
void Anahera_Task_20ms(const AnaheraObjectList_t* objs,
                       const AnaheraEgoState_t*   ego,
                       const AnaheraLaneModel_t*  lane);
```

`Anahera_Task_20ms` is intended to be called periodically (for example every 20 ms) from your main loop or RTOS task.

---

## Getting started

### Prerequisites

- C toolchain (for example `gcc` or `clang`) on Linux, macOS, or other POSIX‑style environment.  
- Basic familiarity with C, structs, and build systems (Makefile or similar).

If your repository includes a `Makefile`, you can usually build a demo binary with:

```bash
make
```

This can be adapted to your own build system (CMake, Bazel, IDE projects, etc.).

---

## Basic usage

1. **Configure and initialize**

   Either call the helper:

   ```c
   Anahera_ConfigureAndInit();
   ```

   or create your own `AnaheraConfig_t` and call:

   ```c
   AnaheraConfig_t cfg = { /* fill your calibration */ };
   Anahera_Init(&cfg);
   ```

2. **Prepare inputs each cycle**

   - Fill `AnaheraObjectList_t` with current perception objects.  
   - Fill `AnaheraEgoState_t` with current ego state.  
   - Fill `AnaheraLaneModel_t` with the ego‑lane description from your HD map / localization.

3. **Call the update function**

   ```c
   AnaheraAlert_t alert;
   Anahera_Update(&objs, &ego, &lane, &alert);
   ```

4. **Consume the output**

   - Check `alert.valid` to see if any alert is active.  
   - Use `alert.level` and `alert.requested_decel_mps2` to drive your higher‑level ADAS / braking logic.  
   - Monitor `alert.status`, `alert.watchdog_counter`, and `alert.redundancy_mismatch` in your safety supervisor.

---

## Project structure

Typical files you might see in this repository:

- `anahera_alerting_system.c` – Main alerting core implementation.  
- `anahera_alerting_system.h` (if present) – Public API and type declarations.  
- `Makefile` – Example build configuration for a demo or test harness.  
- `README.md` – This file.  
- `LICENSE` – MIT License for the project.  

You can adapt the layout to your own project structure (for example splitting headers, sources, test harness, and documentation into separate directories).
---

## Safety and legal disclaimer

- This project is a **demo** and **research** module only.  
- It has **no warranty** of correctness, safety, or fitness for any particular purpose.[web:91][web:94]  
- Do not integrate it into real vehicles or safety‑critical systems without a full engineering and safety lifecycle (requirements, design, verification, validation, safety analysis, certification, etc.).  

For all legal terms, see the License section below.

---

## License

This project is licensed under the **MIT License**. See the `LICENSE` file in this repository for the full text.[web:91][web:94][web:117]

If you use this code in your own projects (open‑source or commercial), make sure to:

- Keep the MIT copyright and permission notice.  
- Keep the warranty disclaimer in your distributions.[web:91][web:94][web:111]
```


Sources
