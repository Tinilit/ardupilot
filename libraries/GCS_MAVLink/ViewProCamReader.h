/*
  ViewPro camera reader.
  Reads T1F1B1D1 packets directly from serial port and logs yaw/pitch/roll to DataFlash.
*/

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <stdint.h>

#define VIEWPRO_CAM_SERIAL_ID     5
#define VIEWPRO_CAM_PACKETLEN_MAX 63

class ViewProCamReader {
public:
    ViewProCamReader() { _singleton = this; }
    void update();

    float get_yaw_deg()   const { return _yaw_deg; }
    float get_pitch_deg() const { return _pitch_deg; }
    float get_roll_deg()  const { return _roll_deg; }
    uint32_t last_angle_ms() const { return _last_angle_ms; }

    static ViewProCamReader *get_singleton() { return _singleton; }

private:
    void init();
    uint8_t get_length_and_frame_count_byte(uint8_t len);
    bool send_packet(const uint8_t *data, uint8_t len);
    void send_stream_request();
    void read_incoming_packets();
    void process_packet();
    void log_angles();
    static uint8_t calc_crc(const uint8_t *buf, uint8_t len);

    AP_HAL::UARTDriver *_uart = nullptr;
    bool _initialised = false;

    enum class ParseState : uint8_t {
        HEADER1, HEADER2, HEADER3, LENGTH, FRAMEID, DATA, CRC
    };
    struct {
        ParseState state = ParseState::HEADER1;
        uint8_t data_len = 0;
        uint8_t frame_id = 0;
        uint16_t data_bytes_received = 0;
        uint8_t crc = 0;
    } _parsed_msg;

    uint8_t _msg_buff[VIEWPRO_CAM_PACKETLEN_MAX];
    uint8_t _msg_buff_len = 0;
    static const uint8_t _msg_buff_data_start = 2;

    float _yaw_deg   = 0;
    float _pitch_deg = 0;
    float _roll_deg  = 0;

    uint32_t _last_angle_ms     = 0;
    uint32_t _last_log_ms       = 0;
    uint32_t _last_stream_req_ms = 0;
    uint32_t _last_diag_ms      = 0;
    uint32_t _packets_received  = 0;
    uint32_t _bytes_received    = 0;
    uint8_t  _frame_counter     = 0;

    static ViewProCamReader *_singleton;
};
