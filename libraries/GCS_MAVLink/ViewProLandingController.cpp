/*
  ViewPro visual landing approach controller - implementation.
*/

#include "ViewProLandingController.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>

ViewProLandingController *ViewProLandingController::_singleton = nullptr;

// @Group: VLA_
// @Path: ViewProLandingController.cpp
const AP_Param::GroupInfo ViewProLandingController::var_info[] = {
    // @Param: DSCNT_MPS
    // @DisplayName: VLA descent rate
    // @Description: QLAND descent speed in LAND phase (m/s)
    // @Range: 0.1 3.0
    // @Units: m/s
    AP_GROUPINFO("DSCNT_MPS",  1, ViewProLandingController, creep_descent_mps,    0.75f),

    // @Param: FWD_MAX
    // @DisplayName: VLA max creep speed
    // @Description: Maximum horizontal creep speed in ALIGN phase (m/s)
    // @Range: 0.5 10.0
    // @Units: m/s
    AP_GROUPINFO("FWD_MAX",    2, ViewProLandingController, creep_fwd_max_mps,     3.50f),

    // @Param: FWD_K
    // @DisplayName: VLA creep gain
    // @Description: Pitch error (deg) to creep speed (m/s) gain; 10 deg error -> 2 m/s at 0.20
    // @Range: 0.01 1.0
    AP_GROUPINFO("FWD_K",      3, ViewProLandingController, creep_fwd_k,           0.20f),

    // @Param: P_ALIGN
    // @DisplayName: VLA ALIGN entry pitch
    // @Description: Enter ALIGN/QLOITER when camera pitch is below this value (negative = looking down)
    // @Range: -90 0
    // @Units: deg
    AP_GROUPINFO("P_ALIGN",    4, ViewProLandingController, pitch_align_deg,      -80.0f),

    // @Param: P_DESCND
    // @DisplayName: VLA LAND entry pitch
    // @Description: Enter LAND/QLAND when camera pitch drops below this value
    // @Range: -90 -70
    // @Units: deg
    AP_GROUPINFO("P_DESCND",   5, ViewProLandingController, pitch_descend_deg,    -88.0f),

    // @Param: P_TARGET
    // @DisplayName: VLA pitch target
    // @Description: Desired camera pitch for zero creep (overhead target)
    // @Range: -90 -80
    // @Units: deg
    AP_GROUPINFO("P_TARGET",   6, ViewProLandingController, pitch_target_deg,     -90.0f),

    // @Param: P_HOLD
    // @DisplayName: VLA pitch-lost threshold (ALIGN)
    // @Description: If camera pitch rises above this in ALIGN, stop creep and hold QLOITER position
    // @Range: -70 0
    // @Units: deg
    AP_GROUPINFO("P_HOLD",     7, ViewProLandingController, pitch_lost_align_deg, -35.0f),

    // @Param: P_LAND
    // @DisplayName: VLA pitch-lost threshold (LAND)
    // @Description: If camera pitch rises above this in LAND, return to ALIGN
    // @Range: -90 -70
    // @Units: deg
    AP_GROUPINFO("P_LAND",     8, ViewProLandingController, pitch_lost_land_deg,  -75.0f),

    // @Param: Y_THRESH
    // @DisplayName: VLA yaw correction threshold
    // @Description: If camera yaw exceeds this in APPROACH, correct aircraft heading to zero camera yaw
    // @Range: 1 30
    // @Units: deg
    AP_GROUPINFO("Y_THRESH",   9, ViewProLandingController, yaw_corr_thresh_deg,   5.0f),

    AP_GROUPEND
};

void ViewProLandingController::activate(float wind_azimuth_deg, ViewProCamReader &cam)
{
    if (!switch_to_mode(MODE_GUIDED)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "VLA:ERR_MODE");
        return;
    }
    _state             = State::APPROACH;
    _overhead_since_ms = 0;
    _align_since_ms    = 0;
    _descend_since_ms  = 0;
    _land_since_ms     = 0;
    _last_update_ms       = 0;
    _target_bearing_deg   = 0.0f;
    _last_pitch_gcs_ms    = 0;
    _pitch_lost_holding   = false;

    // Record camera yaw at activation and compute initial approach heading.
    // approach_heading = aircraft_heading + cam_yaw  → world bearing to target.
    const float hdg_deg = wrap_180(degrees(AP::ahrs().get_yaw()));
    const float cam_yaw = cam.get_yaw_deg();
    _approach_heading_deg = wrap_360(hdg_deg + cam_yaw);
    // Into-wind heading: azimuth = where wind blows TO, so nose = azimuth + 180 (face into wind).
    _into_wind_heading_deg = wrap_360(wind_azimuth_deg + 180.0f);
    _has_wind_heading      = true;

    if (AP_Vehicle *vehicle = AP::vehicle()) {
        vehicle->set_velocity_match(Vector2f{});
    }
    gcs().send_text(MAV_SEVERITY_INFO, "VLA:ACTIVATED");
}

void ViewProLandingController::deactivate()
{
    if (AP_Vehicle *vehicle = AP::vehicle()) {
        vehicle->set_velocity_match(Vector2f{});
    }
    _last_pitch_gcs_ms = 0;
    _state = State::IDLE;
    gcs().send_text(MAV_SEVERITY_INFO, "VLA:DEACTIVATED");
    switch_to_mode(MODE_AUTO);
}

void ViewProLandingController::land_complete()
{
    if (_state == State::IDLE) {
        return;
    }
    if (AP_Vehicle *vehicle = AP::vehicle()) {
        vehicle->set_velocity_match(Vector2f{});
    }
    _last_pitch_gcs_ms = 0;
    _state = State::IDLE;
    gcs().send_text(MAV_SEVERITY_INFO, "VLA:END");
}

void ViewProLandingController::update(ViewProCamReader &cam)
{
    if (_state == State::IDLE) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_update_ms < UPDATE_MS) {
        return;
    }
    _last_update_ms = now_ms;

    const float pitch   = cam.get_pitch_deg();
    const float yaw     = cam.get_yaw_deg();
    const float hdg_deg = wrap_180(degrees(AP::ahrs().get_yaw()));

    // ── APPROACH: fly on locked _approach_heading_deg,
    //    correct only when cam yaw drifts beyond threshold ──
    if (_state == State::APPROACH) {
        if (fabsf(yaw) > yaw_corr_thresh_deg) {
            _approach_heading_deg = wrap_360(hdg_deg + yaw);
        }
        _target_bearing_deg = wrap_180(_approach_heading_deg);
    } else {
        // ── ALIGN / LAND: delta-based bearing ──
        // Lock initial world bearing + camera yaw on the first cycle of ALIGN
        // (_align_since_ms is still 0 until the ALIGN handler runs below).
        // Decouples bearing from aircraft heading changes while following target.
        if (_align_since_ms == 0) {
            _initial_bearing_deg = wrap_180(hdg_deg + yaw);
            _initial_yaw_deg     = yaw;
        }
        _target_bearing_deg = wrap_180(_initial_bearing_deg + (yaw - _initial_yaw_deg));
    }

    if (now_ms - _last_pitch_gcs_ms >= 3000) {
        _last_pitch_gcs_ms = now_ms;
    }

    if (_state == State::LAND) {
        if (_land_since_ms == 0) {
            _land_since_ms = now_ms;
            gcs().send_text(MAV_SEVERITY_INFO, "VLA:LAND");
        }
        if (pitch > pitch_lost_land_deg) {
            if (switch_to_mode(MODE_QLOITER)) {
                _state            = State::ALIGN;
                _align_since_ms   = 0;
                _descend_since_ms = 0;
                _land_since_ms    = 0;
                if (AP_Vehicle *vehicle = AP::vehicle()) {
                    vehicle->set_velocity_match(Vector2f{}, 1);
                }
                gcs().send_text(MAV_SEVERITY_WARNING, "VLA:LAND_ABORT");
            }
            return;
        }
        // Gentle velocity correction only — don’t fight QLAND position hold
        const float pitch_error = pitch - pitch_target_deg;
        const float corr_mps    = constrain_float(pitch_error * creep_fwd_k * 0.5f,
                                                  -creep_fwd_max_mps * 0.5f,
                                                   creep_fwd_max_mps * 0.5f);
        const float brng_rad = radians(_target_bearing_deg);
        if (AP_Vehicle *vehicle = AP::vehicle()) {
            // set_velocity_match takes m/s NE (NOT cm/s). Source=1 == VelocityMatchSource::VLA.
            vehicle->set_velocity_match(Vector2f(cosf(brng_rad) * corr_mps,
                                                  sinf(brng_rad) * corr_mps), 1);
            vehicle->set_land_descent_rate(creep_descent_mps);
            command_into_wind_heading(vehicle);
        }
        return;
    }

    // ── ALIGN state: QLOITER, pitch-tracked horizontal creep, altitude holds ──
    if (_state == State::ALIGN) {
        if (_align_since_ms == 0) {
            _align_since_ms = now_ms;
            gcs().send_text(MAV_SEVERITY_INFO, "VLA:ALIGN");
        }

        if (pitch > pitch_lost_align_deg) {
            if (!_pitch_lost_holding) {
                _pitch_lost_holding = true;
                gcs().send_text(MAV_SEVERITY_WARNING, "VLA:HOLD");
            }
            // Do NOT send velocity — let vm_active expire so loiter_nav->update()
            // takes full control (XY hold + altitude hold via loiter).
            return;
        }
        if (_pitch_lost_holding) {
            _pitch_lost_holding = false;
            gcs().send_text(MAV_SEVERITY_INFO, "VLA:RESUME");
        }

        if (AP_Vehicle *vehicle = AP::vehicle()) {
            command_into_wind_heading(vehicle);
        }
        set_creep_commands(_target_bearing_deg, pitch);

        // Check for vertical alignment → switch to QLAND for final descent
        if (pitch < pitch_descend_deg) {
            if (_descend_since_ms == 0) {
                _descend_since_ms = now_ms;
            } else if (now_ms - _descend_since_ms >= DESCEND_CONFIRM_MS) {
                if (switch_to_mode(MODE_QLAND)) {
                    _state         = State::LAND;
                    _land_since_ms = 0;
                }
            }
        } else {
            _descend_since_ms = 0;
        }
        return;
    }

    // ── APPROACH state ───────────────────────────────────────────────────
    // overhead detection: pitch < PITCH_ALIGN_DEG means camera is steeply
    // down -> aircraft is directly over the target. Stop and start aligning.
    // ViewPro pitch: 0=horizon, -90=straight down (negative = looking down)
    if (pitch < pitch_align_deg) {
        if (_overhead_since_ms == 0) {
            _overhead_since_ms = now_ms;
          
        } else if (now_ms - _overhead_since_ms >= OVERHEAD_HOLD_MS) {
            if (switch_to_mode(MODE_QLOITER)) {
                _state             = State::ALIGN;
                _align_since_ms    = 0;
                _descend_since_ms  = 0;
                _overhead_since_ms = 0;
            }
        }
    } else {
        _overhead_since_ms = 0;
    }

    // push a waypoint ahead on the locked approach heading every cycle.
    // The heading was set at activation and gets corrected when cam yaw > threshold.
    if (_state == State::APPROACH) {
        push_guided_waypoint(_target_bearing_deg);
    }

    // Quiet mode: no per-cycle telemetry spam on GCS.
}

void ViewProLandingController::set_creep_commands(float target_bearing_deg, float pitch_deg)
{
    AP_Vehicle *vehicle = AP::vehicle();
    if (vehicle == nullptr) {
        return;
    }

    // Signed pitch error:
    //   positive → aircraft is short of overhead → fly FORWARD
    //   negative → aircraft overshot overhead   → fly BACKWARD
    //   ~zero    → near overhead, minimal movement
    const float pitch_error = pitch_deg - pitch_target_deg;
    const float fwd_mps = constrain_float(pitch_error * creep_fwd_k,
                                          -creep_fwd_max_mps, creep_fwd_max_mps);

    // set_velocity_match() expects m/s in NE frame (NOT cm/s).
    // QLOITER multiplies internally by 100 to get cm/s for the pos controller.
    // Source=1 == VelocityMatchSource::VLA — only this source activates creep in QLOITER.
    const float bearing_rad = radians(target_bearing_deg);
    vehicle->set_velocity_match(Vector2f(cosf(bearing_rad) * fwd_mps,
                                          sinf(bearing_rad) * fwd_mps), 1);
    _last_fwd_mps = fwd_mps;
}

void ViewProLandingController::push_guided_waypoint(float target_bearing_deg)
{
    Location current_loc;
    if (!AP::ahrs().get_location(current_loc)) {
        return;
    }

    // Project LOOKAHEAD_M metres on the locked world bearing.
    Location target = current_loc;
    target.offset_bearing(target_bearing_deg, LOOKAHEAD_M);
    target.alt          = current_loc.alt;
    target.relative_alt = current_loc.relative_alt;

    AP_Vehicle *vehicle = AP::vehicle();
    if (vehicle != nullptr) {
        vehicle->set_target_location(target);
    }
}

bool ViewProLandingController::switch_to_mode(uint8_t mode)
{
    AP_Vehicle *vehicle = AP::vehicle();
    if (vehicle == nullptr) {
        return false;
    }
    return vehicle->set_mode(mode, ModeReason::GCS_COMMAND);
}

void ViewProLandingController::command_into_wind_heading(AP_Vehicle *vehicle)
{
    if (!_has_wind_heading || vehicle == nullptr) {
        return;
    }
    vehicle->set_vtol_heading(_into_wind_heading_deg);
}
