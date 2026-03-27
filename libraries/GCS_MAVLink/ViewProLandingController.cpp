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

void ViewProLandingController::activate(float wind_azimuth_deg)
{
    if (!switch_to_mode(MODE_GUIDED)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Сannot enter GUIDED mode");
        return;
    }
    _state             = State::APPROACH;
    _overhead_since_ms = 0;
    _align_since_ms    = 0;
    _descend_since_ms  = 0;
    _land_since_ms     = 0;
    _last_update_ms       = 0;
    _bearing_locked       = false;
    _target_bearing_deg   = 0.0f;
    _initial_bearing_deg  = 0.0f;
    _initial_yaw_deg      = 0.0f;
    _last_pitch_gcs_ms    = 0;

    // Into-wind heading: azimuth = where wind blows TO, so nose = azimuth + 180 (face into wind).
    _into_wind_heading_deg = wrap_360(wind_azimuth_deg + 180.0f);
    _has_wind_heading      = true;

    if (AP_Vehicle *vehicle = AP::vehicle()) {
        vehicle->set_velocity_match(Vector2f{});
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Autoland activated, wind az=%.0f, nose->%.0f",
                    (double)wind_azimuth_deg, (double)_into_wind_heading_deg);
}

void ViewProLandingController::retarget()
{
    // Re-acquire a new landing target without a full restart.
    // Switches back to APPROACH (GUIDED) and resets bearing lock so the
    // next camera reading will lock onto the new target.
    // Wind heading is preserved from the original activate() call.
    if (_state != State::APPROACH) {
        if (!switch_to_mode(MODE_GUIDED)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Retarget: cannot enter GUIDED");
            return;
        }
        _state = State::APPROACH;
    }
    _bearing_locked      = false;
    _target_bearing_deg  = 0.0f;
    _initial_bearing_deg = 0.0f;
    _initial_yaw_deg     = 0.0f;
    _overhead_since_ms   = 0;
    _align_since_ms      = 0;
    _descend_since_ms    = 0;
    _land_since_ms       = 0;
    _last_update_ms      = 0;
    if (AP_Vehicle *vehicle = AP::vehicle()) {
        vehicle->set_velocity_match(Vector2f{});
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Autoland retarget, wind->%.0f",
                    (double)_into_wind_heading_deg);
}

void ViewProLandingController::deactivate()
{
    if (AP_Vehicle *vehicle = AP::vehicle()) {
        vehicle->set_velocity_match(Vector2f{});
    }
    _last_pitch_gcs_ms = 0;
    _state = State::IDLE;
    gcs().send_text(MAV_SEVERITY_INFO, "Autoland deactivated -> AUTO");
    switch_to_mode(MODE_AUTO);
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

    // Lock initial bearing and initial camera yaw once.
    // Then every cycle adjust bearing by the DELTA in camera yaw.
    // This decouples bearing from aircraft heading changes (no orbit)
    // while still following real target movement via camera tracker.
    if (!_bearing_locked) {
        _initial_bearing_deg = wrap_180(hdg_deg + yaw);
        _initial_yaw_deg     = yaw;
        _bearing_locked      = true;
    }
    _target_bearing_deg = wrap_180(_initial_bearing_deg + (yaw - _initial_yaw_deg));

    if (now_ms - _last_pitch_gcs_ms >= 3000) {
        _last_pitch_gcs_ms = now_ms;
        gcs().send_text(MAV_SEVERITY_INFO, "Cam p=%.1f y=%.1f dy=%.1f brg=%.1f",
                        (double)pitch, (double)yaw,
                        (double)(yaw - _initial_yaw_deg),
                        (double)_target_bearing_deg);
    }

    if (_state == State::LAND) {
        if (_land_since_ms == 0) {
            _land_since_ms = now_ms;
            gcs().send_text(MAV_SEVERITY_INFO,
                "QLAND start, descending");
        }
        if (pitch > PITCH_LOST_LAND_DEG) {
            if (switch_to_mode(MODE_QLOITER)) {
                _state            = State::ALIGN;
                _align_since_ms   = 0;
                _descend_since_ms = 0;
                _land_since_ms    = 0;
                if (AP_Vehicle *vehicle = AP::vehicle()) {
                    vehicle->set_velocity_match(Vector2f{}, 1);
                }
                gcs().send_text(MAV_SEVERITY_WARNING,
                    "LAND abort: pitch %.1f > %.0f, back to QLOITER",
                    (double)pitch, (double)PITCH_LOST_LAND_DEG);
            }
            return;
        }
        // Gentle velocity correction only — don’t fight QLAND position hold
        const float pitch_error = pitch - PITCH_TARGET_DEG;
        const float corr_mps    = constrain_float(pitch_error * CREEP_FWD_K * 0.5f,
                                                  -CREEP_FWD_MAX_MPS * 0.5f,
                                                   CREEP_FWD_MAX_MPS * 0.5f);
        const float brng_rad = radians(_target_bearing_deg);
        if (AP_Vehicle *vehicle = AP::vehicle()) {
            // set_velocity_match takes m/s NE (NOT cm/s). Source=1 == VelocityMatchSource::VLA.
            vehicle->set_velocity_match(Vector2f(cosf(brng_rad) * corr_mps,
                                                  sinf(brng_rad) * corr_mps), 1);
            vehicle->set_land_descent_rate(CREEP_DESCENT_MPS);
            command_into_wind_heading(vehicle);
        }
        return;
    }

    // ── ALIGN state: QLOITER, pitch-tracked horizontal creep, altitude holds ──
    if (_state == State::ALIGN) {
        if (_align_since_ms == 0) {
            _align_since_ms = now_ms;
            gcs().send_text(MAV_SEVERITY_INFO,
                "QLOITER start, aligning");
        }

        if (pitch > PITCH_LOST_ALIGN_DEG) {
            gcs().send_text(MAV_SEVERITY_WARNING,
                "ALIGN abort: pitch %.1f > %.0f, target lost",
                (double)pitch, (double)PITCH_LOST_ALIGN_DEG);
            deactivate();
            return;
        }

        if (AP_Vehicle *vehicle = AP::vehicle()) {
            command_into_wind_heading(vehicle);
        }
        set_creep_commands(_target_bearing_deg, pitch);

        // Check for vertical alignment → switch to QLAND for final descent
        if (pitch < PITCH_DESCEND_DEG) {
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
    if (pitch < PITCH_ALIGN_DEG) {
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

    // push a waypoint 300 m ahead in the locked world bearing every cycle.
    // This prevents the waypoint from rotating with the aircraft and causing orbiting.
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
    const float pitch_error = pitch_deg - PITCH_TARGET_DEG;
    const float fwd_mps = constrain_float(pitch_error * CREEP_FWD_K,
                                          -CREEP_FWD_MAX_MPS, CREEP_FWD_MAX_MPS);

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
