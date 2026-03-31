/*
  ViewPro visual landing approach controller.

  Strategy:
    APPROACH phase  – switch to GUIDED (fixed-wing), push a waypoint
                      LOOKAHEAD_M metres ahead in the direction derived from
                      camera yaw relative to aircraft heading every cycle.
                      Aircraft flies straight, altitude-holds naturally.
    ALIGN phase     – when pitch drops below PITCH_ALIGN_DEG, switch to
                      QLOITER and every cycle send velocity_match (m/s NE):
                        • fwd = pitch_error * CREEP_FWD_K  (signed, clamped)
                          positive → fly forward (pitch needs to go more negative)
                          negative → fly backward (overshot past -90°)
                      Altitude holds in QLOITER. When pitch < PITCH_DESCEND_DEG
                      for DESCEND_CONFIRM_MS, switch to LAND.
    LAND phase      – switch to QLAND. Descend at CREEP_DESCENT_MPS with small
                      pitch corrections via velocity_match. QLAND detects touchdown.

  Command 65001 (param1 > 0 = activate, param1 = 0 = deactivate).
*/

#pragma once

#include "ViewProCamReader.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/Location.h>
#include <AP_Param/AP_Param.h>
#include <stdint.h>

class AP_Vehicle;

class ViewProLandingController {
public:
    ViewProLandingController() {
        _singleton = this;
        AP_Param::setup_object_defaults(this, var_info);
    }

    static const struct AP_Param::GroupInfo var_info[];

    void update(ViewProCamReader &cam);
    void activate(float wind_azimuth_deg, ViewProCamReader &cam);
    void deactivate();
    void land_complete();
    bool is_active() const { return _state != State::IDLE; }

    static ViewProLandingController *get_singleton() { return _singleton; }

private:
    enum class State : uint8_t {
        IDLE,
        APPROACH,
        ALIGN,   // QLOITER + pitch-tracking horizontal creep (altitude hold)
        LAND
    };

    State    _state             = State::IDLE;
    uint32_t _last_update_ms    = 0;
    uint32_t _overhead_since_ms = 0;
    uint32_t _align_since_ms    = 0;
    float    _target_bearing_deg   = 0.0f;
    float    _initial_bearing_deg  = 0.0f;  // bearing at lock time (ALIGN/LAND)
    float    _initial_yaw_deg      = 0.0f;  // camera yaw at lock time (ALIGN/LAND)
    float    _last_fwd_mps         = 0.0f;  // last commanded ±creep speed m/s, for logging
    float    _into_wind_heading_deg = 0.0f;  // heading to face into wind (azimuth+180)
    bool     _has_wind_heading      = false;
    uint32_t _last_pitch_gcs_ms  = 0;
    bool     _pitch_lost_holding   = false;  // true while pitch > P_HOLD threshold in ALIGN
    uint32_t _descend_since_ms   = 0;     // first ms when pitch < PITCH_DESCEND_DEG in ALIGN
    uint32_t _land_since_ms      = 0;     // ms when LAND state entered
    float    _approach_heading_deg = 0.0f;  // locked heading for APPROACH (world bearing to target)

    static ViewProLandingController *_singleton;

    // tunables (exposed as VLA_* parameters)
    AP_Float creep_descent_mps;   // VLA_DSCNT_MPS  default 0.75
    AP_Float creep_fwd_max_mps;   // VLA_FWD_MAX    default 3.50
    AP_Float creep_fwd_k;         // VLA_FWD_K      default 0.20
    AP_Float pitch_align_deg;     // VLA_P_ALIGN    default -80
    AP_Float pitch_descend_deg;   // VLA_P_DESCND   default -88
    AP_Float pitch_target_deg;    // VLA_P_TARGET   default -90
    AP_Float pitch_lost_align_deg;// VLA_P_HOLD     default -35
    AP_Float pitch_lost_land_deg; // VLA_P_LAND     default -85
    AP_Float yaw_corr_thresh_deg; // VLA_Y_THRESH   default 5.0

    // non-tunable compile-time constants
    static constexpr float    LOOKAHEAD_M        = 5000.0f;
    static constexpr uint32_t UPDATE_MS          = 150;
    static constexpr uint32_t OVERHEAD_HOLD_MS   = 1500;   // ms at pitch<-80 before ALIGN
    static constexpr uint32_t DESCEND_CONFIRM_MS = 1000;   // ms at pitch<-89 before QLAND

    // mode numbers
    static constexpr uint8_t MODE_AUTO    = 10;   // auto mission – resume waypoints
    static constexpr uint8_t MODE_GUIDED  = 15;   // fixed-wing guided – stable flight
    static constexpr uint8_t MODE_QLOITER = 19;   // VTOL loiter – velocity_match works directly
    static constexpr uint8_t MODE_QLAND   = 20;   // VTOL land – descends, detects touchdown

    bool switch_to_mode(uint8_t mode);
    void push_guided_waypoint(float target_bearing_deg);
    void set_creep_commands(float target_bearing_deg, float pitch_deg);
    void command_into_wind_heading(AP_Vehicle *vehicle);
};
