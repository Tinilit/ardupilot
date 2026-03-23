/*
  ViewPro visual landing approach controller.

  Strategy:
    APPROACH phase  – switch to GUIDED (fixed-wing), push a waypoint
                      LOOKAHEAD_M metres ahead in the direction derived from
                      camera yaw relative to aircraft heading every cycle.
                      Aircraft flies straight, altitude-holds naturally.
    OVERHEAD phase  – when pitch > PITCH_LAND_DEG for OVERHEAD_HOLD_MS,
                      switch to QLAND (VTOL land, mode 20).

  Command 65001 (param1 > 0 = activate, param1 = 0 = deactivate).
*/

#pragma once

#include "ViewProCamReader.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/Location.h>
#include <stdint.h>

class ViewProLandingController {
public:
    ViewProLandingController() { _singleton = this; }

    void update(ViewProCamReader &cam);
    void activate();
    void deactivate();
    bool is_active() const { return _state != State::IDLE; }

    static ViewProLandingController *get_singleton() { return _singleton; }

private:
    enum class State : uint8_t {
        IDLE,
        APPROACH,
        LAND
    };

    State    _state             = State::IDLE;
    uint32_t _last_update_ms    = 0;
    uint32_t _overhead_since_ms = 0;
    bool     _target_locked     = false;
    float    _target_bearing_deg = 0.0f;

    static ViewProLandingController *_singleton;

    // tunables
    static constexpr float    LOOKAHEAD_M      = 300.0f;  // metres ahead to push WP
    static constexpr float    PITCH_LAND_DEG   = -75.0f;  // trigger QLAND below this (pitch -90=straight down)
    static constexpr uint32_t UPDATE_MS        = 150;
    static constexpr uint32_t OVERHEAD_HOLD_MS = 1500;

    // mode numbers
    static constexpr uint8_t MODE_GUIDED = 15;   // fixed-wing guided – stable flight
    static constexpr uint8_t MODE_QLAND  = 20;   // VTOL land

    bool switch_to_mode(uint8_t mode);
    void push_guided_waypoint(float target_bearing_deg);
};
