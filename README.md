# Anahera Alerting System

**Author:** Max Gecse  
**Version:** 0.1 (Prototype / Demo)  
**License:** MIT

---

## Overview

Anahera Alerting System is a forward hazard detection module intended for integration into high‑end driver assistance or automated driving stacks.

It runs on a central vehicle computer (e.g. an ADAS / automated driving domain controller) and:

- Receives a fused **object list** (vehicles, pedestrians, cyclists, static obstacles) in the ego vehicle frame.  
- Receives an **ego‑lane model** derived from HD maps and sensor fusion (lane centerline and width).  
- Continuously computes **time‑to‑collision (TTC)** to objects in the ego lane.  
- Detects **sudden** figures/obstacles appearing in the ego lane in a speed range of approx. **30–260 km/h**.  
- Outputs graded alerts (warning / soft brake / hard brake) that can be consumed by a higher‑level longitudinal controller or AEB logic.

> ⚠️ This repository contains **prototype / educational** code only. It is **not** suitable for direct use in safety‑critical or series‑production automotive systems.

---

## Features

- Ego‑lane based detection using an **HD‑map lane model** (centerline polynomial + lane width).  
- Support for multiple object classes:
  - Vehicle  
  - Pedestrian  
  - Cyclist  
  - Static obstacle  
- “Sudden appearance” heuristic using simple object track history.  
- TTC‑based decision logic with configurable thresholds for:
  - Warning  
  - Soft braking request  
  - Hard braking request  
- Configurable parameters:
  - Speed band (e.g. 30–260 km/h)  
  - Maximum detection distance  
  - Minimum object confidence  

---

## Data Flow

### Inputs (per cycle)

- `AnaheraObjectList_t`  
  - Array of objects with:
    - Track ID  
    - Position (x, y) in ego frame  
    - Relative velocities (vx, vy)  
    - Length, width  
    - Object type  
    - Confidence  
- `AnaheraEgoState_t`  
  - Ego speed  
  - Yaw rate  
  - Steering angle  
  - ABS / AEB flags  
- `AnaheraLaneModel_t`  
  - Ego‑lane ID  
  - Lane width  
  - Lane centerline polynomial in ego frame  
  - Optional asymmetric lane half‑widths  
  - Validity flag  

### Processing

1. Filter objects by validity, confidence, and type.  
2. Use the lane model to decide if each object is in the **ego lane**.  
3. Enforce a forward distance limit.  
4. Use simple per‑ID history to detect **sudden** objects (seen now, not seen last filtered cycle).  
5. Compute TTC for relevant objects.  
6. Determine alert level based on TTC and ego speed.  
7. Select the most critical object and generate one `AnaheraAlert_t`.

### Output

- `AnaheraAlert_t`
  - Alert level:
    - `ANH_ALERT_LEVEL_NONE`  
    - `ANH_ALERT_LEVEL_WARNING`  
    - `ANH_ALERT_LEVEL_BRAKE_SOFT`  
    - `ANH_ALERT_LEVEL_BRAKE_HARD`  
  - Requested deceleration (m/s²)  
  - Track ID of most critical object  
  - TTC (s)  
  - Valid flag  

---

## API Overview

### Initialization

```c
void Anahera_Init(const AnaheraConfig_t* cfg);

/* Example helper provided in the source */
void Anahera_ConfigureAndInit(void);


 AnaheraConfig_t  allows you to configure:
	•	 min_speed_mps  /  max_speed_mps 
	•	 max_detection_distance_m 
	•	 min_confidence 
	•	 ttc_warn_s 
	•	 ttc_brake_soft_s 
	•	 ttc_brake_hard_s 
Periodic Update


void Anahera_Update(const AnaheraObjectList_t* objs,
                    const AnaheraEgoState_t*   ego,
                    const AnaheraLaneModel_t*  lane,
                    AnaheraAlert_t*            outAlert);

/* Example task wrapper (e.g. 20 ms cycle) */
void Anahera_Task_20ms(const AnaheraObjectList_t* objs,
                       const AnaheraEgoState_t*   ego,
                       const AnaheraLaneModel_t*  lane);
You are expected to adapt  Anahera_Task_20ms  to your actual scheduling and middleware (e.g. AUTOSAR RTE, RTOS task, or in‑process callback).
Usage Example
	1.	At startup, configure and initialize:

int main(void)
{
    /* System/platform init here... */

    Anahera_ConfigureAndInit();

    /* Scheduler / main loop setup... */
}

2.	Periodically (e.g. every 20 ms), call the task with current inputs:


void Periodic_20ms(void)
{
    AnaheraObjectList_t objs;
    AnaheraEgoState_t   ego;
    AnaheraLaneModel_t  lane;

    /* Fill objs, ego, lane from your perception and lane-fusion stack */

    Anahera_Task_20ms(&objs, &ego, &lane);
}
	3.	Inside  Anahera_Task_20ms , handle  AnaheraAlert_t  and forward to your:
	•	Braking / longitudinal controller
	•	HMI / warning system
	•	Logging infrastructure
Intended Use
	•	Research and prototyping of:
	•	In‑lane obstacle detection.
	•	TTC‑based hazard evaluation.
	•	Safety supervision around perception outputs.
	•	Educational and concept demonstration code for:
	•	Architecture discussions.
	•	Safety concept drafting.
	•	Code structure examples.
		❌ Not intended as a production‑ready safety function or as a certified implementation for any standard (e.g. ISO 26262).
Safety Disclaimer
	•	The software has not been safety‑validated, verified, or certified for any automotive standard.
	•	It must never be relied upon as the sole or primary safety mechanism in any real vehicle.
	•	Any real‑world automotive use requires a complete safety engineering lifecycle:
	•	Hazard and risk analysis
	•	Functional and technical safety concepts
	•	Detailed design, verification, and validation
	•	Independent assessment and certification (where applicable)
License
This project is licensed under the MIT License.

MIT License

Copyright (c) 2026 Max Gecse

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.





