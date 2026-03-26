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
#include <stdint.h>

class AP_Vehicle;

class ViewProLandingController {
public:
    ViewProLandingController() { _singleton = this; }

    void update(ViewProCamReader &cam);
    void activate(float wind_azimuth_deg);
    void retarget();   // re-acquire target without stopping; keeps wind heading
    void deactivate();
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
    bool     _bearing_locked      = false;
    float    _target_bearing_deg   = 0.0f;
    float    _initial_bearing_deg  = 0.0f;  // bearing at lock time
    float    _initial_yaw_deg      = 0.0f;  // camera yaw at lock time
    float    _last_fwd_mps         = 0.0f;  // last commanded ±creep speed m/s, for logging
    float    _into_wind_heading_deg = 0.0f;  // heading to face into wind (azimuth+180)
    bool     _has_wind_heading      = false;
    uint32_t _last_pitch_gcs_ms  = 0;
    uint32_t _descend_since_ms   = 0;     // first ms when pitch < PITCH_DESCEND_DEG in ALIGN
    uint32_t _land_since_ms      = 0;     // ms when LAND state entered

    static ViewProLandingController *_singleton;

    // tunables
    static constexpr float    LOOKAHEAD_M       = 300.0f;  // metres ahead to push WP
    static constexpr float    CREEP_DESCENT_MPS  =  0.75f; // QLAND descent rate m/s in LAND (increased)
    static constexpr float    CREEP_FWD_MAX_MPS  =  3.50f; // max ±horiz creep speed m/s in ALIGN (increased)
    static constexpr float    CREEP_FWD_K        =  0.20f; // pitch err deg → m/s  (10°=2.0 m/s)
    static constexpr float    PITCH_ALIGN_DEG    = -80.0f; // enter ALIGN (QLOITER) below this
    static constexpr float    PITCH_DESCEND_DEG  = -88.0f; // enter LAND (QLAND) below this
    static constexpr float    PITCH_TARGET_DEG   = -90.0f; // ALIGN feedback target
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
