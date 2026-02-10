/* anahera_alerting_system.c
 *
 * Anahera Alerting System
 * Author: Max Gecse
 *
 * ADAS module for a central NVIDIA-based ECU on a 2023–2026
 * Mercedes S-Class. It detects "sudden figures" (objects) in the
 * ego lane using an HD-map-based ego lane model and outputs
 * warning / brake requests in the speed range 30–260 km/h.
 *
 * WARNING: This code is provided for demonstration and educational purposes.
 * Without explicit written authorization from the author (Max Gecse),
 * it must not be used in series-production systems or safety-critical
 * applications, and no code copying, redistribution, or commercial use
 * is allowed.
 */

#include <stdint.h>
#include <stdbool.h>

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

/* Output alert structure */
typedef struct
{
    AnaheraAlertLevel_t level;
    float               requested_decel_mps2;   /* 0 = none, negative = braking */
    int32_t             object_id;              /* ID of most critical object, or -1 */
    float               ttc_s;                  /* time to collision (s), or -1 if n/a */
    bool                valid;                  /* true if alert active this cycle */
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
} AnaheraConfig_t;

/* API */
void Anahera_Init(const AnaheraConfig_t* cfg);
void Anahera_Update(const AnaheraObjectList_t* objs,
                    const AnaheraEgoState_t*   ego,
                    const AnaheraLaneModel_t*  lane,
                    AnaheraAlert_t*            outAlert);

/* ================== Internal Implementation ================== */

#ifndef NULL
#define NULL ((void*)0)
#endif

/* Internal configuration copy */
static AnaheraConfig_t g_anhCfg;

/* Simple history for “sudden appearance” detection */
typedef struct
{
    int32_t obj_id;
    bool    was_seen_last_cycle;
} AnaheraHistoryEntry_t;

#define ANH_HISTORY_SIZE  64U
static AnaheraHistoryEntry_t g_anhHistory[ANH_HISTORY_SIZE];

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
    return NULL;
}

static AnaheraHistoryEntry_t* Anahera_GetOrAllocHistoryEntry(int32_t id)
{
    AnaheraHistoryEntry_t* entry = Anahera_FindHistoryEntry(id);
    uint8_t i;

    if (entry != NULL)
    {
        return entry;
    }

    /* allocate new slot */
    for (i = 0U; i < ANH_HISTORY_SIZE; ++i)
    {
        if (g_anhHistory[i].obj_id == 0 && g_anhHistory[i].was_seen_last_cycle == false)
        {
            g_anhHistory[i].obj_id = id;
            return &g_anhHistory[i];
        }
    }

    /* fallback: overwrite first entry */
    g_anhHistory[0].obj_id = id;
    g_anhHistory[0].was_seen_last_cycle = false;
    g_anhHistory[0].obj_id = id;
    return &g_anhHistory[0];
}

void Anahera_Init(const AnaheraConfig_t* cfg)
{
    uint8_t i;

    if (cfg != NULL)
    {
        g_anhCfg = *cfg;
    }

    for (i = 0U; i < ANH_HISTORY_SIZE; ++i)
    {
        g_anhHistory[i].obj_id = 0;
        g_anhHistory[i].was_seen_last_cycle = false;
    }
}

/* Compute TTC (time to collision) in seconds.
 * vx_rel_mps is relative speed along x (m/s); negative means object approaching.
 */
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
    if ((lane == NULL) || !lane->valid)
    {
        return false;
    }

    /* Only consider objects ahead */
    if (o->x_m <= 0.0f)
    {
        return false;
    }

    /* Evaluate lane centerline y at object x: y_c = c0 + c1*x + c2*x^2 */
    float x  = o->x_m;
    float y_center = lane->center_c0 +
                     lane->center_c1 * x +
                     lane->center_c2 * x * x;

    /* Lateral offset from lane centerline */
    float dy = o->y_m - y_center;

    /* Use provided asymmetric half-widths if >0, else symmetric from lane_width_m */
    float left_hw  = (lane->half_width_left_m  > 0.0f) ?
                      lane->half_width_left_m  :
                      lane->lane_width_m * 0.5f;
    float right_hw = (lane->half_width_right_m > 0.0f) ?
                      lane->half_width_right_m :
                      lane->lane_width_m * 0.5f;

    /* Right is negative, left is positive in ego y */
    if ((dy < -right_hw) || (dy > left_hw))
    {
        return false;
    }

    return true;
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

    if ((objs == NULL) || (ego == NULL) || (outAlert == NULL))
    {
        return;
    }

    outAlert->level = ANH_ALERT_LEVEL_NONE;
    outAlert->requested_decel_mps2 = 0.0f;
    outAlert->object_id = -1;
    outAlert->ttc_s = -1.0f;
    outAlert->valid = false;

    /* Process all objects */
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

        /* type filter: skip unknowns */
        if (o->type == ANH_OBJ_TYPE_UNKNOWN)
        {
            continue;
        }

        /* In ego lane using HD-map lane model */
        if (!Anahera_IsObjectInEgoLane(o, lane))
        {
            continue;
        }

        /* Forward distance constraint */
        if (o->x_m > g_anhCfg.max_detection_distance_m)
        {
            continue;
        }

        /* History and "sudden" determination */
        AnaheraHistoryEntry_t* hist = Anahera_GetOrAllocHistoryEntry(o->id);
        bool sudden = (hist->was_seen_last_cycle == false);

        /* TTC based on relative longitudinal speed */
        float ttc = Anahera_ComputeTtc(o->x_m, o->vx_rel_mps);

        /* If not sudden and TTC not critical, skip (tuning knob) */
        if (!sudden && (ttc < 0.0f || ttc > g_anhCfg.ttc_warn_s))
        {
            hist->was_seen_last_cycle = true;
            continue;
        }

        AnaheraAlertLevel_t level = Anahera_EvaluateLevel(ttc, ego->speed_mps);

        if (level > bestLevel)
        {
            bestLevel = level;
            bestTtc = ttc;
            bestObjId = o->id;
        }

        hist->was_seen_last_cycle = true;
    }

    /* Decay "was seen" flag for next cycle */
    for (i = 0U; i < ANH_HISTORY_SIZE; ++i)
    {
        g_anhHistory[i].was_seen_last_cycle = false;
    }

    /* Fill output */
    if (bestLevel != ANH_ALERT_LEVEL_NONE)
    {
        outAlert->level = bestLevel;
        outAlert->object_id = bestObjId;
        outAlert->ttc_s = bestTtc;
        outAlert->valid = true;

        switch (bestLevel)
        {
            case ANH_ALERT_LEVEL_WARNING:
                outAlert->requested_decel_mps2 = 0.0f;   /* warning only */
                break;
            case ANH_ALERT_LEVEL_BRAKE_SOFT:
                outAlert->requested_decel_mps2 = -2.0f;  /* example soft decel */
                break;
            case ANH_ALERT_LEVEL_BRAKE_HARD:
                outAlert->requested_decel_mps2 = -6.0f;  /* example hard decel */
                break;
            default:
                outAlert->requested_decel_mps2 = 0.0f;
                break;
        }
    }
}

/* ================== Example Configuration & Task Hook ================== */

/* Initialize Anahera Alerting System with 30–260 km/h band */
void Anahera_ConfigureAndInit(void)
{
    AnaheraConfig_t cfg;

    /* 30–260 km/h => 8.33–72.22 m/s */
    cfg.min_speed_mps            = 30.0f / 3.6f;
    cfg.max_speed_mps            = 260.0f / 3.6f;
    cfg.max_detection_distance_m = 120.0f;  /* 120 m ahead */
    cfg.min_confidence           = 0.6f;
    cfg.ttc_warn_s               = 3.0f;
    cfg.ttc_brake_soft_s         = 2.0f;
    cfg.ttc_brake_hard_s         = 1.0f;

    Anahera_Init(&cfg);
}

/* Call this every cycle (e.g. 20 ms) with current objects, ego state, and lane model */
void Anahera_Task_20ms(const AnaheraObjectList_t* objs,
                       const AnaheraEgoState_t*   ego,
                       const AnaheraLaneModel_t*  lane)
{
    AnaheraAlert_t alert;
    Anahera_Update(objs, ego, lane, &alert);

    if (alert.valid)
    {
        /* Integrate here:
         * - publish alert to braking / ADAS controller via MB.OS middleware
         * - log event, update HMI, etc.
         */
    }
}

/* ================== MIT License (as requested) ==================

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

/* NOTE: The following warning is explicitly requested by the author,
 * but conceptually conflicts with the permissive nature of the MIT License.
 * It is kept here verbatim per the author’s instructions.
 *
 * WARNING: Without explicit authorization from the author (Max Gecse),
 * this Software must not be used in production systems, and no code copying,
 * redistribution, or commercial use is allowed.
 */

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR  
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,  
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE  
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER  
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING  
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER  
DEALINGS IN THE SOFTWARE.

===================================================================== *
