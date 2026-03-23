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

void ViewProLandingController::activate()
{
    if (!switch_to_mode(MODE_GUIDED)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "VLA: cannot enter GUIDED mode");
        return;
    }
    _state             = State::APPROACH;
    _overhead_since_ms = 0;
    _last_update_ms    = 0;
    _target_locked     = false;
    _target_bearing_deg = 0.0f;
    gcs().send_text(MAV_SEVERITY_INFO, "VLA: GUIDED approach activated");
}

void ViewProLandingController::deactivate()
{
    _state = State::IDLE;
    gcs().send_text(MAV_SEVERITY_INFO, "VLA: stopped");
}

void ViewProLandingController::update(ViewProCamReader &cam)
{
    if (_state == State::IDLE || _state == State::LAND) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_update_ms < UPDATE_MS) {
        return;
    }
    _last_update_ms = now_ms;

    const float pitch = cam.get_pitch_deg();
    const float yaw   = cam.get_yaw_deg();
    const float hdg_deg = wrap_180(degrees(AP::ahrs().get_yaw()));

    if (!_target_locked) {
        _target_bearing_deg = wrap_180(hdg_deg + yaw);
        _target_locked = true;
        gcs().send_text(MAV_SEVERITY_INFO,
            "VLA: target locked bearing=%.1f (hdg=%.1f camRel=%.1f)",
            (double)_target_bearing_deg, (double)hdg_deg, (double)yaw);
    }

    // overhead detection -> QLAND
    // ViewPro pitch: 0=horizon, -90=straight down (negative = looking down)
    if (pitch < PITCH_LAND_DEG) {
        if (_overhead_since_ms == 0) {
            _overhead_since_ms = now_ms;
            gcs().send_text(MAV_SEVERITY_INFO,
                "VLA: overhead (pitch=%.1f), confirming...", (double)pitch);
        } else if (now_ms - _overhead_since_ms >= OVERHEAD_HOLD_MS) {
            gcs().send_text(MAV_SEVERITY_INFO, "VLA: overhead confirmed -> QLAND");
            if (switch_to_mode(MODE_QLAND)) {
                _state = State::LAND;
            }
        }
    } else {
        _overhead_since_ms = 0;
    }

    // push a waypoint 300m ahead in the locked world bearing every cycle.
    // This prevents the waypoint from rotating with the aircraft and causing orbiting.
    push_guided_waypoint(_target_bearing_deg);

    // ── diagnostic log: one line tells the whole story ────────────────────
    // hdg     = aircraft nose direction (compass degrees)
    // camRel  = camera yaw relative to aircraft heading
    // target  = locked absolute bearing used for the waypoint
    // diff    = target - hdg; should trend toward 0 as the aircraft aligns
    // pitch   = 0=horizon, more negative = camera looking more steeply down
    //            QLAND fires when pitch < -83
    const float gspd     = AP::ahrs().groundspeed_vector().length();
    const float cam_rel   = wrap_180(yaw);
    const float target_bearing = _target_bearing_deg;
    const float diff      = wrap_180(target_bearing - hdg_deg);
    gcs().send_text(MAV_SEVERITY_INFO,
        "VLA: camRel=%5.1f hdg=%5.1f tgt=%5.1f diff=%+.1f gspd=%.1f pitch=%.1f",
        (double)cam_rel, (double)hdg_deg, (double)target_bearing, (double)diff,
        (double)gspd, (double)pitch);
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
