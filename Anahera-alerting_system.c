/* Anahera Alerting System – Integrated ADAS Demo Stack
 * Author: Max Gecse
 *
 * Version: 0.2 (Integrated Fusion → Anahera → Planner demo)
 * Date: 2026-03-02
 *
 * DESCRIPTION
 * ===========
 * This file contains an integrated, experimental ADAS demo stack for a
 * central NVIDIA-based ECU on a 2023–2026 Mercedes S-Class–style vehicle.
 *
 * It includes:
 *  - The Anahera sudden-object alerting module.
 *    * Detects "sudden figures" (objects) in the ego lane using an
 *      HD-map-based ego-lane model.
 *    * Outputs warning / brake requests in the speed range 30–260 km/h.
 *
 *  - A minimal sensor-fusion environment model stub.
 *    * Simulates ego vehicle motion, lane geometry, and a single obstacle.
 *    * Provides a fused environment to the Anahera module.
 *
 *  - A simple longitudinal planner stub.
 *    * Consumes Anahera alerts and decides a target deceleration.
 *    * In this demo, AEB (Anahera) overrides ACC when active.
 *
 *  - A unit test harness with main().
 *    * Runs an end-to-end loop: Fusion -> Anahera -> Planner.
 *    * Logs ego speed, object distance, alert level, and brake request.
 *
 * SAFETY NOTICE
 * =============
 * This code is experimental concept code for research and education.
 * It is not safety-certified and must not be used in real vehicles
 * or other safety-critical systems without full validation, safety
 * engineering, and certification by qualified experts.
 *
 * The algorithms, interfaces, and parameters are illustrative only.
 * They omit many measures required for series-production ADAS / AD
 * systems, including but not limited to:
 *  - Complete functional safety engineering (ISO 26262 etc.).
 *  - Cybersecurity engineering (ISO/SAE 21434 etc.).
 *  - Robust diagnostics and fault handling.
 *  - Comprehensive testing, verification, and validation.
 *
 * LICENSE
 * =======
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* ================== Public Types and API ================== */

/* Object classification */
typedef enum
{
    ANH_OBJ_TYPE_UNKNOWN          = 0,
    ANH_OBJ_TYPE_VEHICLE          = 1,
    ANH_OBJ_TYPE_PEDESTRIAN       = 2,
    ANH_OBJ_TYPE_CYCLIST          = 3,
    ANH_OBJ_TYPE_STATIC_OBSTACLE  = 4
} AnaheraObjectType_t;

/* Single perceived object */
typedef struct
{
    int32_t             id;          /* stable track ID */
    float               x_m;         /* longitudinal position in ego frame (m) */
    float               y_m;         /* lateral position in ego frame (m) */
    float               vx_rel_mps;  /* relative longitudinal speed (m/s) */
    float               vy_rel_mps;  /* relative lateral speed (m/s) */
    float               length_m;    /* object length (m) */
    float               width_m;     /* object width (m) */
    AnaheraObjectType_t type;        /* classification */
    float               confidence;  /* 0.0–1.0 */
    bool                valid;       /* basic validity flag */
} AnaheraObject_t;

#define ANH_MAX_OBJECT_COUNT   64U

typedef struct
{
    AnaheraObject_t objects[ANH_MAX_OBJECT_COUNT];
    uint8_t         count;
} AnaheraObjectList_t;

/* Ego vehicle state */
typedef struct
{
    float speed_mps;        /* ego speed (m/s) */
    float yaw_rate_radps;
    float steering_angle_rad;
    bool  abs_active;
    bool  aeb_active;
} AnaheraEgoState_t;

/* HD-map-based ego-lane model input */
typedef struct
{
    int32_t lane_id;          /* ego-lane identifier from map */
    float   lane_width_m;     /* local lane width at ego position (m) */

    /* Lane centerline polynomial in ego frame:
       y_center(x) = c0 + c1*x + c2*x^2 (local segment) */
    float   center_c0;
    float   center_c1;
    float   center_c2;

    /* Optional asymmetric lane half-widths (m) */
    float   half_width_left_m;   /* positive magnitude left of centerline */
    float   half_width_right_m;  /* positive magnitude right of centerline */
    bool    valid;
} AnaheraLaneModel_t;

/* Output levels */
typedef enum
{
    ANH_ALERT_LEVEL_NONE       = 0,
    ANH_ALERT_LEVEL_WARNING    = 1,
    ANH_ALERT_LEVEL_BRAKE_SOFT = 2,
    ANH_ALERT_LEVEL_BRAKE_HARD = 3
} AnaheraAlertLevel_t;

/* Status for safety */
typedef enum
{
    ANH_STATUS_OK = 0,
    ANH_STATUS_DEGRADED,
    ANH_STATUS_FAILED
} AnaheraStatus_t;

/* Output alert structure */
typedef struct
{
    AnaheraAlertLevel_t level;
    float               requested_decel_mps2;   /* 0 = none, negative = braking */
    int32_t             object_id;              /* ID of most critical object, or -1 */
    float               ttc_s;                  /* time to collision (s), or -1 if n/a */
    bool                valid;                  /* true if alert active this cycle */

    /* Safety-related fields */
    AnaheraStatus_t     status;                 /* OK / DEGRADED / FAILED */
    uint32_t            watchdog_counter;       /* incremented each successful call */
    bool                redundancy_mismatch;    /* true if redundant paths disagree */
} AnaheraAlert_t;

/* Config (calibration) */
typedef struct
{
    float min_speed_mps;            /* active from this speed (m/s) */
    float max_speed_mps;            /* up to this speed (m/s) */
    float max_detection_distance_m; /* forward detection distance (m) */
    float min_confidence;           /* min object confidence */
    float ttc_warn_s;               /* TTC threshold for warning */
    float ttc_brake_soft_s;         /* TTC threshold for soft brake */
    float ttc_brake_hard_s;         /* TTC threshold for hard brake */

    /* Safety-related plausibility thresholds */
    float max_lane_width_m;         /* plausibility for lane model width */
    float max_abs_obj_distance_m;   /* plausibility for object distance */
    float max_abs_speed_mps;        /* plausibility for ego and relative speeds */

    /* Redundancy configuration */
    bool  redundancy_enabled;
    float redundancy_ttc_tolerance_s;
} AnaheraConfig_t;

/* API */
void Anahera_Init(const AnaheraConfig_t* cfg);
void Anahera_Update(const AnaheraObjectList_t* objs,
                    const AnaheraEgoState_t*   ego,
                    const AnaheraLaneModel_t*  lane,
                    AnaheraAlert_t*            outAlert);

/* Example configuration & task hook */
void Anahera_ConfigureAndInit(void);
void Anahera_Task_20ms(const AnaheraObjectList_t* objs,
                       const AnaheraEgoState_t*   ego,
                       const AnaheraLaneModel_t*  lane);

/* ================== Internal Implementation (Anahera) ================== */

/* Internal configuration copy */
static AnaheraConfig_t g_anhCfg;

/* Internal watchdog counter (incremented each successful update) */
static uint32_t g_anhWatchdogCounter = 0U;

/* Simple history for “sudden appearance” detection */
typedef struct
{
    int32_t obj_id;
    bool    was_seen_last_cycle;
} AnaheraHistoryEntry_t;

#define ANH_HISTORY_SIZE  64U

static AnaheraHistoryEntry_t g_anhHistory[ANH_HISTORY_SIZE];
static uint8_t               g_anhHistoryIndex = 0U;

static void Anahera_HistoryReset(void)
{
    uint8_t i;

    for (i = 0U; i < ANH_HISTORY_SIZE; ++i)
    {
        g_anhHistory[i].obj_id = 0;
        g_anhHistory[i].was_seen_last_cycle = false;
    }

    g_anhHistoryIndex = 0U;
}

/* Linear search for existing entry */
static AnaheraHistoryEntry_t* Anahera_FindHistoryEntry(int32_t id)
{
    uint8_t i;

    for (i = 0U; i < ANH_HISTORY_SIZE; ++i)
    {
        if (g_anhHistory[i].obj_id == id)
        {
            return &g_anhHistory[i];
        }
    }

    return (AnaheraHistoryEntry_t*)0;
}

/* Round-robin allocation / overwrite */
static AnaheraHistoryEntry_t* Anahera_GetOrAllocHistoryEntry(int32_t id)
{
    AnaheraHistoryEntry_t* entry;

    entry = Anahera_FindHistoryEntry(id);
    if (entry != (AnaheraHistoryEntry_t*)0)
    {
        return entry;
    }

    entry = &g_anhHistory[g_anhHistoryIndex];

    g_anhHistoryIndex++;
    if (g_anhHistoryIndex >= ANH_HISTORY_SIZE)
    {
        g_anhHistoryIndex = 0U;
    }

    entry->obj_id = id;
    entry->was_seen_last_cycle = false;

    return entry;
}

/* Compute TTC (time to collision) in seconds. */
static float Anahera_ComputeTtc(float distance_m, float vx_rel_mps)
{
    const float EPS = 0.1f; /* avoid division by zero */

    if (vx_rel_mps >= -EPS)
    {
        /* Not closing in or very slow closing, TTC not critical */
        return -1.0f;
    }

    return distance_m / (-vx_rel_mps);
}

/* Evaluate alert level given TTC and ego speed */
static AnaheraAlertLevel_t Anahera_EvaluateLevel(float ttc_s, float speed_mps)
{
    if (ttc_s < 0.0f)
    {
        return ANH_ALERT_LEVEL_NONE;
    }

    /* Only active within speed range */
    if ((speed_mps < g_anhCfg.min_speed_mps) || (speed_mps > g_anhCfg.max_speed_mps))
    {
        return ANH_ALERT_LEVEL_NONE;
    }

    if (ttc_s <= g_anhCfg.ttc_brake_hard_s)
    {
        return ANH_ALERT_LEVEL_BRAKE_HARD;
    }
    else if (ttc_s <= g_anhCfg.ttc_brake_soft_s)
    {
        return ANH_ALERT_LEVEL_BRAKE_SOFT;
    }
    else if (ttc_s <= g_anhCfg.ttc_warn_s)
    {
        return ANH_ALERT_LEVEL_WARNING;
    }
    else
    {
        return ANH_ALERT_LEVEL_NONE;
    }
}

/* In-lane check using HD-map ego-lane model */
static bool Anahera_IsObjectInEgoLane(const AnaheraObject_t* o,
                                      const AnaheraLaneModel_t* lane)
{
    float x;
    float y_center;
    float dy;
    float left_hw;
    float right_hw;

    if ((lane == (const AnaheraLaneModel_t*)0) || (!lane->valid))
    {
        return false;
    }

    if (o == (const AnaheraObject_t*)0)
    {
        return false;
    }

    /* Only consider objects ahead */
    if (o->x_m <= 0.0f)
    {
        return false;
    }

    /* Evaluate lane centerline y at object x: y_c = c0 + c1*x + c2*x^2 */
    x  = o->x_m;
    y_center = lane->center_c0 +
               lane->center_c1 * x +
               lane->center_c2 * x * x;

    /* Lateral offset from lane centerline */
    dy = o->y_m - y_center;

    /* Use provided asymmetric half-widths if >0, else symmetric from lane_width_m */
    left_hw  = (lane->half_width_left_m  > 0.0f) ?
                lane->half_width_left_m  :
                lane->lane_width_m * 0.5f;
    right_hw = (lane->half_width_right_m > 0.0f) ?
                lane->half_width_right_m :
                lane->lane_width_m * 0.5f;

    /* Right is negative, left is positive in ego y */
    if ((dy < -right_hw) || (dy > left_hw))
    {
        return false;
    }

    return true;
}

/* Input plausibility checks */
static bool Anahera_CheckInputsPlausible(const AnaheraObjectList_t* objs,
                                         const AnaheraEgoState_t*   ego,
                                         const AnaheraLaneModel_t*  lane)
{
    uint8_t i;

    if ((objs == (const AnaheraObjectList_t*)0) ||
        (ego  == (const AnaheraEgoState_t*)0)   ||
        (lane == (const AnaheraLaneModel_t*)0))
    {
        return false;
    }

    /* Ego speed plausibility */
    if ((ego->speed_mps < -1.0f) || (ego->speed_mps > g_anhCfg.max_abs_speed_mps))
    {
        return false;
    }

    /* Lane plausibility (if valid) */
    if (lane->valid)
    {
        if ((lane->lane_width_m <= 0.0f) ||
            (lane->lane_width_m > g_anhCfg.max_lane_width_m))
        {
            return false;
        }
    }

    /* Object plausibility */
    for (i = 0U; i < objs->count; ++i)
    {
        const AnaheraObject_t* o = &objs->objects[i];

        if (!o->valid)
        {
            continue;
        }

        if ( (o->x_m < -1.0f) ||
             (o->x_m > g_anhCfg.max_abs_obj_distance_m) ||
             (o->y_m < -g_anhCfg.max_abs_obj_distance_m) ||
             (o->y_m > g_anhCfg.max_abs_obj_distance_m) )
        {
            return false;
        }

        if ( (o->vx_rel_mps < -g_anhCfg.max_abs_speed_mps) ||
             (o->vx_rel_mps >  g_anhCfg.max_abs_speed_mps) )
        {
            return false;
        }
    }

    return true;
}

/* Secondary, simpler TTC path to provide redundancy */
static AnaheraAlertLevel_t Anahera_SecondaryPathCompute(const AnaheraObjectList_t* objs,
                                                        const AnaheraEgoState_t*   ego,
                                                        const AnaheraLaneModel_t*  lane,
                                                        float*                     ttc_out,
                                                        int32_t*                   obj_id_out)
{
    uint8_t i;
    AnaheraAlertLevel_t bestLevel = ANH_ALERT_LEVEL_NONE;
    float bestTtc = -1.0f;
    int32_t bestObjId = -1;

    if ((ttc_out == (float*)0) || (obj_id_out == (int32_t*)0) ||
        (objs == (const AnaheraObjectList_t*)0) ||
        (ego  == (const AnaheraEgoState_t*)0)   ||
        (lane == (const AnaheraLaneModel_t*)0))
    {
        return ANH_ALERT_LEVEL_NONE;
    }

    for (i = 0U; i < objs->count; ++i)
    {
        const AnaheraObject_t* o = &objs->objects[i];

        if (!o->valid || (o->confidence < g_anhCfg.min_confidence))
        {
            continue;
        }

        if (o->type == ANH_OBJ_TYPE_UNKNOWN)
        {
            continue;
        }

        if (!Anahera_IsObjectInEgoLane(o, lane))
        {
            continue;
        }

        if (o->x_m > g_anhCfg.max_detection_distance_m)
        {
            continue;
        }

        {
            float ttc = Anahera_ComputeTtc(o->x_m, o->vx_rel_mps);
            AnaheraAlertLevel_t level = Anahera_EvaluateLevel(ttc, ego->speed_mps);

            if (level > bestLevel)
            {
                bestLevel = level;
                bestTtc = ttc;
                bestObjId = o->id;
            }
        }
    }

    *ttc_out = bestTtc;
    *obj_id_out = bestObjId;
    return bestLevel;
}

void Anahera_Init(const AnaheraConfig_t* cfg)
{
    if (cfg != (const AnaheraConfig_t*)0)
    {
        g_anhCfg = *cfg;
    }

    Anahera_HistoryReset();
    g_anhWatchdogCounter = 0U;
}

/* Main update function */
void Anahera_Update(const AnaheraObjectList_t* objs,
                    const AnaheraEgoState_t*   ego,
                    const AnaheraLaneModel_t*  lane,
                    AnaheraAlert_t*            outAlert)
{
    uint8_t i;
    AnaheraAlertLevel_t bestLevel = ANH_ALERT_LEVEL_NONE;
    float bestTtc = -1.0f;
    int32_t bestObjId = -1;

    if ((objs == (const AnaheraObjectList_t*)0) ||
        (ego  == (const AnaheraEgoState_t*)0)   ||
        (outAlert == (AnaheraAlert_t*)0))
    {
        return;
    }

    /* Initialize output */
    outAlert->level = ANH_ALERT_LEVEL_NONE;
    outAlert->requested_decel_mps2 = 0.0f;
    outAlert->object_id = -1;
    outAlert->ttc_s = -1.0f;
    outAlert->valid = false;
    outAlert->status = ANH_STATUS_FAILED;
    outAlert->redundancy_mismatch = false;
    outAlert->watchdog_counter = g_anhWatchdogCounter;

    /* Basic plausibility check */
    if (!Anahera_CheckInputsPlausible(objs, ego, lane))
    {
        /* Stay in FAILED / safe state (no brake request) */
        return;
    }

    /* === Primary path: TTC + "sudden" logic === */

    for (i = 0U; i < objs->count; ++i)
    {
        const AnaheraObject_t* o = &objs->objects[i];

        if (!o->valid)
        {
            continue;
        }

        if (o->confidence < g_anhCfg.min_confidence)
        {
            continue;
        }

        if (o->type == ANH_OBJ_TYPE_UNKNOWN)
        {
            continue;
        }

        if (!Anahera_IsObjectInEgoLane(o, lane))
        {
            continue;
        }

        if (o->x_m > g_anhCfg.max_detection_distance_m)
        {
            continue;
        }

        {
            AnaheraHistoryEntry_t* hist = Anahera_GetOrAllocHistoryEntry(o->id);
            bool sudden = (hist->was_seen_last_cycle == false);

            float ttc = Anahera_ComputeTtc(o->x_m, o->vx_rel_mps);

            if (!sudden && ((ttc < 0.0f) || (ttc > g_anhCfg.ttc_warn_s)))
            {
                hist->was_seen_last_cycle = true;
                continue;
            }

            {
                AnaheraAlertLevel_t level = Anahera_EvaluateLevel(ttc, ego->speed_mps);

                if (level > bestLevel)
                {
                    bestLevel = level;
                    bestTtc = ttc;
                    bestObjId = o->id;
                }
            }

            hist->was_seen_last_cycle = true;
        }
    }

    /* Decay "was seen" flag for next cycle */
    for (i = 0U; i < ANH_HISTORY_SIZE; ++i)
    {
        g_anhHistory[i].was_seen_last_cycle = false;
    }

    /* Default status is OK (no internal fault detected so far) */
    outAlert->status = ANH_STATUS_OK;

    /* === Optional redundant path === */
    if (g_anhCfg.redundancy_enabled)
    {
        float sec_ttc = -1.0f;
        int32_t sec_id = -1;
        AnaheraAlertLevel_t sec_level =
            Anahera_SecondaryPathCompute(objs, ego, lane, &sec_ttc, &sec_id);

        /* Compare primary and secondary path results */
        {
            bool mismatch = false;

            if (sec_level != bestLevel)
            {
                mismatch = true;
            }
            else if ((sec_level != ANH_ALERT_LEVEL_NONE) &&
                     (sec_ttc >= 0.0f) && (bestTtc >= 0.0f))
            {
                float diff = (sec_ttc > bestTtc) ? (sec_ttc - bestTtc) : (bestTtc - sec_ttc);
                if (diff > g_anhCfg.redundancy_ttc_tolerance_s)
                {
                    mismatch = true;
                }
            }

            if (mismatch)
            {
                outAlert->redundancy_mismatch = true;
                outAlert->status = ANH_STATUS_DEGRADED;
            }
        }
    }

    /* Fill output from primary path */
    if (bestLevel != ANH_ALERT_LEVEL_NONE)
    {
        outAlert->level = bestLevel;
        outAlert->object_id = bestObjId;
        outAlert->ttc_s = bestTtc;
        outAlert->valid = true;

        switch (bestLevel)
        {
            case ANH_ALERT_LEVEL_WARNING:
                outAlert->requested_decel_mps2 = 0.0f;
                break;
            case ANH_ALERT_LEVEL_BRAKE_SOFT:
                outAlert->requested_decel_mps2 = -2.0f;
                break;
            case ANH_ALERT_LEVEL_BRAKE_HARD:
                outAlert->requested_decel_mps2 = -6.0f;
                break;
            default:
                outAlert->requested_decel_mps2 = 0.0f;
                break;
        }
    }

    g_anhWatchdogCounter++;
    outAlert->watchdog_counter = g_anhWatchdogCounter;
}

/* Example configuration: 30–260 km/h band */
void Anahera_ConfigureAndInit(void)
{
    AnaheraConfig_t cfg;

    /* 30–260 km/h => 8.33–72.22 m/s */
    cfg.min_speed_mps            = 30.0f / 3.6f;
    cfg.max_speed_mps            = 260.0f / 3.6f;
    cfg.max_detection_distance_m = 120.0f;
    cfg.min_confidence           = 0.6f;
    cfg.ttc_warn_s               = 3.0f;
    cfg.ttc_brake_soft_s         = 2.0f;
    cfg.ttc_brake_hard_s         = 1.0f;

    cfg.max_lane_width_m         = 5.0f;
    cfg.max_abs_obj_distance_m   = 250.0f;
    cfg.max_abs_speed_mps        = 90.0f;

    cfg.redundancy_enabled         = true;
    cfg.redundancy_ttc_tolerance_s = 0.3f;

    Anahera_Init(&cfg);
}

/* Original task form (unused in test harness, kept for completeness) */
void Anahera_Task_20ms(const AnaheraObjectList_t* objs,
                       const AnaheraEgoState_t*   ego,
                       const AnaheraLaneModel_t*  lane)
{
    AnaheraAlert_t alert;
    Anahera_Update(objs, ego, lane, &alert);
}

/* ================== Fusion Environment & Planner ================== */

/* Fused environment model (sensor fusion output) */
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

/* Longitudinal planner interfaces */
typedef enum
{
    LONGI_SOURCE_NONE   = 0,
    LONGI_SOURCE_ACC    = 1,
    LONGI_SOURCE_AEB    = 2,
    LONGI_SOURCE_DRIVER = 3
} LongiSource_t;

typedef struct
{
    float  requested_decel_mps2;
    bool   active;
    LongiSource_t source;
    int32_t cause_object_id;
} LongiRequest_t;

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

void LongiPlanner_Update(const LongiPlannerInput_t* in,
                         LongiPlannerOutput_t*      out);

/* Anahera binding */
void Anahera_Task_FromFusion(const Fusion_Environment_t* env,
                             AnaheraAlert_t*              alert_out)
{
    if ((env == (const Fusion_Environment_t*)0) ||
        (alert_out == (AnaheraAlert_t*)0))
    {
        return;
    }

    if ((!env->vision_valid) && (!env->radar_valid) && (!env->lidar_valid))
    {
        alert_out->valid = false;
        alert_out->status = ANH_STATUS_FAILED;
        return;
    }

    Anahera_Update(&env->object_list,
                   &env->ego_state,
                   &env->ego_lane,
                   alert_out);
}

/* Simple planner: AEB (Anahera) overrides ACC */
void LongiPlanner_Update(const LongiPlannerInput_t* in,
                         LongiPlannerOutput_t*      out)
{
    LongiRequest_t  aeb_req;
    LongiRequest_t  acc_req;

    if ((in == (const LongiPlannerInput_t*)0) ||
        (out == (LongiPlannerOutput_t*)0))
    {
        return;
    }

    aeb_req.active = in->anahera_alert.valid &&
                     (in->anahera_alert.level != ANH_ALERT_LEVEL_NONE);
    aeb_req.requested_decel_mps2 = in->anahera_alert.requested_decel_mps2;
    aeb_req.source = LONGI_SOURCE_AEB;
    aeb_req.cause_object_id = in->anahera_alert.object_id;

    acc_req.active = false;
    acc_req.requested_decel_mps2 = 0.0f;
    acc_req.source = LONGI_SOURCE_ACC;
    acc_req.cause_object_id = -1;

    if (aeb_req.active)
    {
        out->target_decel_mps2 = aeb_req.requested_decel_mps2;
        out->brake_active = (aeb_req.requested_decel_mps2 < 0.0f);
    }
    else if (acc_req.active)
    {
        out->target_decel_mps2 = acc_req.requested_decel_mps2;
        out->brake_active = (acc_req.requested_decel_mps2 < 0.0f);
    }
    else
    {
        out->target_decel_mps2 = 0.0f;
        out->brake_active = false;
    }
}

/* ================== Fusion Stub + Test Harness ================== */

typedef struct
{
    float ego_speed_mps;
    float lead_obj_initial_x_m;
    float lead_obj_speed_mps;
    float lane_width_m;
} TestScenario_t;

static TestScenario_t g_scenario;
static uint64_t       g_sim_time_ms = 0U;

/* Configure straight-road scenario with one lead object */
void TestScenario_Init(void)
{
    g_scenario.ego_speed_mps        = 30.0f / 3.6f;
    g_scenario.lead_obj_initial_x_m = 40.0f;
    g_scenario.lead_obj_speed_mps   = 0.0f;
    g_scenario.lane_width_m         = 3.5f;

    g_sim_time_ms = 0U;
}

/* Fusion_GetEnvironment stub */
void Fusion_GetEnvironment(Fusion_Environment_t* env_out)
{
    float ego_v;
    float obj_x;
    float obj_v_rel;

    if (env_out == (Fusion_Environment_t*)0)
    {
        return;
    }

    (void)memset(env_out, 0, sizeof(Fusion_Environment_t));

    ego_v = g_scenario.ego_speed_mps;

    g_sim_time_ms += 20U; /* 20 ms step */

    obj_x = g_scenario.lead_obj_initial_x_m +
            (g_scenario.lead_obj_speed_mps * ((float)g_sim_time_ms / 1000.0f));

    obj_v_rel = g_scenario.lead_obj_speed_mps - ego_v;

    env_out->ego_state.speed_mps          = ego_v;
    env_out->ego_state.yaw_rate_radps     = 0.0f;
    env_out->ego_state.steering_angle_rad = 0.0f;
    env_out->ego_state.abs_active         = false;
    env_out->ego_state.aeb_active         = false;

    env_out->ego_lane.lane_id            = 1;
    env_out->ego_lane.lane_width_m       = g_scenario.lane_width_m;
    env_out->ego_lane.center_c0          = 0.0f;
    env_out->ego_lane.center_c1          = 0.0f;
    env_out->ego_lane.center_c2          = 0.0f;
    env_out->ego_lane.half_width_left_m  = 0.0f;
    env_out->ego_lane.half_width_right_m = 0.0f;
    env_out->ego_lane.valid              = true;

    env_out->object_list.count = 1U;
    env_out->object_list.objects[0].id           = 1;
    env_out->object_list.objects[0].x_m          = obj_x;
    env_out->object_list.objects[0].y_m          = 0.0f;
    env_out->object_list.objects[0].vx_rel_mps   = obj_v_rel;
    env_out->object_list.objects[0].vy_rel_mps   = 0.0f;
    env_out->object_list.objects[0].length_m     = 4.5f;
    env_out->object_list.objects[0].width_m      = 1.8f;
    env_out->object_list.objects[0].type         = ANH_OBJ_TYPE_VEHICLE;
    env_out->object_list.objects[0].confidence   = 0.9f;
    env_out->object_list.objects[0].valid        = true;

    env_out->vision_valid = true;
    env_out->radar_valid  = true;
    env_out->lidar_valid  = false;
    env_out->map_valid    = true;

    env_out->timestamp_ms = g_sim_time_ms;
}

/* End-to-end test driver */
void Run_EndToEnd_Test(uint32_t num_steps)
{
    uint32_t step;
    Fusion_Environment_t env;
    AnaheraAlert_t       anh_alert;
    LongiPlannerInput_t  planner_in;
    LongiPlannerOutput_t planner_out;

    TestScenario_Init();
    Anahera_ConfigureAndInit();

    for (step = 0U; step < num_steps; ++step)
    {
        Fusion_GetEnvironment(&env);

        Anahera_Task_FromFusion(&env, &anh_alert);

        planner_in.env           = env;
        planner_in.anahera_alert = anh_alert;

        LongiPlanner_Update(&planner_in, &planner_out);

        printf("t=%5llu ms | v_ego=%.2f m/s | x_obj=%.2f m | "
               "alert_level=%d decel=%.2f | brake_active=%d\n",
               (unsigned long long)env.timestamp_ms,
               env.ego_state.speed_mps,
               env.object_list.objects[0].x_m,
               (int)anh_alert.level,
               planner_out.target_decel_mps2,
               (int)planner_out.brake_active);
    }
}

/* Minimal main for desktop testing */
int main(void)
{
    Run_EndToEnd_Test(200U); /* ~4 seconds at 20 ms step */
    return 0;
}
