# Anahera Guardian ADAS – Alerting Core & Demo Stack

Anahera Guardian ADAS is a demonstration Advanced Driver Assistance System (ADAS) module written in C.  
It models an alerting core for a central NVIDIA-based ECU on a 2023–2026 Mercedes S-Class–style vehicle, detecting sudden objects in the ego lane and issuing warning and brake requests in the 30–260 km/h speed range.

> ⚠️ Safety notice  
> This software is **experimental** and provided for **research and educational purposes only**.  
> It is **not safety‑certified**, has not undergone any automotive SPICE / ISO 26262 process, and **must not be used in real vehicles without proper validation, safety engineering, and certification by qualified experts**.

***

## Author

- **Max Gecse**

***

## Features

Core alerting module:

- Ego‑lane model based on an HD‑map lane description (polynomial lane centerline and lane widths).  
- Object list interface with classification (vehicle, pedestrian, cyclist, static obstacle) and confidence scores.  
- Time‑to‑collision (TTC)–based alert logic with configurable thresholds for:
  - Warning
  - Soft brake
  - Hard brake  
- Speed range gating (active only between configurable minimum and maximum ego speeds).  
- Plausibility checks on ego state, lane model, and object list.  
- Sudden‑appearance detection using a bounded per‑object history buffer.  
- Optional redundant “secondary path” for TTC estimation and mismatch detection.

Integrated demo stack:

- Minimal **fusion environment model** that bundles fused objects, lane model, ego state, timestamps, and sensor validity flags.  
- Simple **sensor‑fusion stub** that simulates a straight lane, ego motion, and a single lead object for testing.  
- Lightweight **longitudinal planner** stub that consumes Anahera alerts and produces a target deceleration (AEB overrides ACC when active).  
- End‑to‑end **test harness** with a `main()` that runs Fusion → Anahera → Planner in a 20 ms loop and logs alert and brake behavior.

***

## Architecture overview

The core logic is implemented as a small C library with a clear public API, with an optional demo stack layered around it.

- **Input types (core)**
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

Demo stack interfaces (optional):

```c
typedef struct
{
    AnaheraObjectList_t object_list;
    AnaheraLaneModel_t  ego_lane;
    AnaheraEgoState_t   ego_state;
    uint64_t            timestamp_ms;
    bool                vision_valid;
    bool                radar_valid;
    bool                lidar_valid;
    bool                map_valid;
} Fusion_Environment_t;

void Fusion_GetEnvironment(Fusion_Environment_t* env_out);

typedef struct
{
    Fusion_Environment_t env;
    AnaheraAlert_t       anahera_alert;
} LongiPlannerInput_t;

typedef struct
{
    float target_decel_mps2;
    bool  brake_active;
} LongiPlannerOutput_t;

void Anahera_Task_FromFusion(const Fusion_Environment_t* env,
                             AnaheraAlert_t*              alert_out);

void LongiPlanner_Update(const LongiPlannerInput_t* in,
                         LongiPlannerOutput_t*      out);
```

`Anahera_Task_20ms` and `Anahera_Task_FromFusion` are intended to be called periodically (for example every 20 ms) from your main loop or RTOS task.

***

## Getting started

### Prerequisites

- C toolchain (for example `gcc` or `clang`) on Linux, macOS, or other POSIX‑style environment.  
- Basic familiarity with C, structs, and build systems (Makefile or similar).

If your repository includes a `Makefile`, you can usually build a demo binary with:

```bash
make
```

This can be adapted to your own build system (CMake, Bazel, IDE projects, etc.).

***

## Basic usage (core library)

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

***

## Basic usage (demo stack)

1. **Run the built-in scenario**

   After building, run the demo binary (for example):

   ```bash
   ./anahera_demo
   ```

   This executes a simulated 1D scenario with a stationary object in your lane, printing time, ego speed, object distance, alert level, and brake request each cycle.

2. **Custom scenarios**

   Adapt the scenario parameters in the demo code (ego speed, initial distance, object speed, lane width) and/or extend the fusion stub to test more complex cases.

***

## Project structure

Typical files you might see in this repository:

- `anahera_alerting_system.c` – Main alerting core implementation.  
- `anahera_alerting_system.h` (if present) – Public API and type declarations.  
- `demo_stack.c` or `main.c` – Fusion stub, planner stub, and test harness for the end‑to‑end demo.  
- `Makefile` – Example build configuration for a demo or test harness.  
- `README.md` – This file.  
- `LICENSE` – Project license (for example Apache 2.0 or MIT; keep it in sync with source headers).  

You can adapt the layout to your own project structure (for example splitting headers, sources, test harness, and documentation into separate directories).

***

## Safety and legal disclaimer

- This project is a **demo** and **research** module only.  
- It has **no warranty** of correctness, safety, or fitness for any particular purpose.  
- Do not integrate it into real vehicles or safety‑critical systems without a full engineering and safety lifecycle (requirements, design, verification, validation, safety analysis, certification, etc.).  

For all legal terms, see the `LICENSE` file in this repository.


