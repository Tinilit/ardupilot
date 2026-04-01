/*
  ViewPro camera reader implementation.
    Reads T1F1B1D1 packets directly from SERIAL5.
*/

#include "ViewProCamReader.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

ViewProCamReader *ViewProCamReader::_singleton = nullptr;

#define VIEWPRO_HEADER1          0x55
#define VIEWPRO_HEADER2          0xAA
#define VIEWPRO_HEADER3          0xDC
#define VIEWPRO_OUTPUT_TO_DEG    (360.0f / 65536.0f)
#define VIEWPRO_FRAMEID_T1F1B1D1 0x40
#define VIEWPRO_STREAM_REQ_MS    2000   // resend stream request every 2s until data arrives

void ViewProCamReader::update()
{
    if (!_initialised) {
        init();
        if (!_initialised) {
            return;
        }
    }

    const uint32_t now_ms = AP_HAL::millis();

    // keep asking camera to stream T1F1B1D1 at 5 Hz
    const uint32_t retry_ms = (_packets_received == 0) ? VIEWPRO_STREAM_REQ_MS : 10000;
    if (now_ms - _last_stream_req_ms > retry_ms) {
        send_stream_request();
    }

    read_incoming_packets();

    log_angles();
}

void ViewProCamReader::init()
{
    _uart = AP::serialmanager().get_serial_by_id(VIEWPRO_CAM_SERIAL_ID);
    if (_uart == nullptr) {
        return;
    }
    _initialised = true;
    send_stream_request();
}

uint8_t ViewProCamReader::get_length_and_frame_count_byte(uint8_t length)
{
    _frame_counter = (_frame_counter + 1) & 0x03;
    return (_frame_counter << 6) | (length & 0x3F);
}

bool ViewProCamReader::send_packet(const uint8_t *databuff, uint8_t databuff_len)
{
    if (_uart == nullptr) {
        return false;
    }
    // 5 bytes overhead: header(3) + length(1) + crc(1); buf is 63 bytes
    if (databuff_len > 58) {
        return false;
    }
    if (_uart->txspace() < (uint16_t)(5 + databuff_len)) {
        return false;
    }
    uint8_t buf[63];
    uint8_t ofs = 0;
    buf[ofs++] = VIEWPRO_HEADER1;
    buf[ofs++] = VIEWPRO_HEADER2;
    buf[ofs++] = VIEWPRO_HEADER3;
    buf[ofs++] = get_length_and_frame_count_byte(databuff_len + 2);
    memcpy(&buf[ofs], databuff, databuff_len);
    ofs += databuff_len;
    buf[ofs] = calc_crc(&buf[3], ofs - 3);
    ofs++;
    return _uart->write(buf, ofs) == ofs;
}

// S2 TGCC command 0x0A: open periodic T1F1B1D1 output at 200ms = 5Hz
void ViewProCamReader::send_stream_request()
{
    const uint8_t payload[] = {0x26, 0x0A, 0x00, 0x00, 0x00, 0xC8};
    if (send_packet(payload, sizeof(payload))) {
        _last_stream_req_ms = AP_HAL::millis();
    }
}

void ViewProCamReader::read_incoming_packets()
{
    uint32_t nbytes = MIN(_uart->available(), 1024U);
    if (nbytes == 0) {
        return;
    }
    _bytes_received += nbytes;

    while (nbytes-- > 0) {
        uint8_t b;
        if (!_uart->read(b)) {
            continue;
        }

        bool reset = false;
        _msg_buff[_msg_buff_len++] = b;
        if (_msg_buff_len >= VIEWPRO_CAM_PACKETLEN_MAX) {
            reset = true;
        }

        switch (_parsed_msg.state) {
        case ParseState::HEADER1:
            if (b == VIEWPRO_HEADER1) {
                _msg_buff_len = 0;
                _parsed_msg.state = ParseState::HEADER2;
            } else {
                reset = true;
            }
            break;

        case ParseState::HEADER2:
            if (b == VIEWPRO_HEADER2) {
                _msg_buff_len = 0;
                _parsed_msg.state = ParseState::HEADER3;
            } else {
                reset = true;
            }
            break;

        case ParseState::HEADER3:
            if (b == VIEWPRO_HEADER3) {
                _msg_buff_len = 0;
                _parsed_msg.state = ParseState::LENGTH;
            } else {
                reset = true;
            }
            break;

        case ParseState::LENGTH:
            _parsed_msg.data_len = b & 0x3F;
            if (_parsed_msg.data_len < 4) {
                reset = true;   // too short to be valid
            } else {
                _parsed_msg.state = ParseState::FRAMEID;
            }
            break;

        case ParseState::FRAMEID:
            _parsed_msg.frame_id = b;
            _parsed_msg.data_bytes_received = 0;
            _parsed_msg.state = ParseState::DATA;
            break;

        case ParseState::DATA:
            _parsed_msg.data_bytes_received++;
            // data_len covers: length_byte + frame_id + data_bytes + crc
            // so data_bytes = data_len - 3
            if (_parsed_msg.data_bytes_received >= (uint16_t)(_parsed_msg.data_len - 3)) {
                _parsed_msg.state = ParseState::CRC;
            }
            break;

        case ParseState::CRC: {
            _parsed_msg.crc = b;
            const uint8_t expected = calc_crc(_msg_buff, _msg_buff_len - 1);
            if (expected == b) {
                process_packet();
            }
            reset = true;
            break;
        }
        }

        if (reset) {
            _msg_buff_len = 0;
            _parsed_msg.state = ParseState::HEADER1;
        }
    }
}

void ViewProCamReader::process_packet()
{
    if (_parsed_msg.frame_id != VIEWPRO_FRAMEID_T1F1B1D1) {
        return;
    }
    // need at least data_start + 29 bytes (indices 0..30)
    if (_msg_buff_len < _msg_buff_data_start + 29) {
        return;
    }

    _packets_received++;

    // Roll: 12-bit value at [data_start+23][data_start+24]
    // upper 4 bits from byte[23] low nibble, lower 8 bits from byte[24]
    const uint16_t roll_raw =
        (uint16_t)((_msg_buff[_msg_buff_data_start + 23] & 0x0F) << 8) |
        _msg_buff[_msg_buff_data_start + 24];
    _roll_deg = roll_raw * (180.0f / 4095.0f) - 90.0f;

    // Yaw: int16 big-endian at [data_start+25][data_start+26]
    const int16_t yaw_raw = (int16_t)(
        ((uint16_t)_msg_buff[_msg_buff_data_start + 25] << 8) |
        _msg_buff[_msg_buff_data_start + 26]);
    _yaw_deg = yaw_raw * VIEWPRO_OUTPUT_TO_DEG;

    // Pitch: int16 big-endian at [data_start+27][data_start+28], negated
    const int16_t pitch_raw = (int16_t)(
        ((uint16_t)_msg_buff[_msg_buff_data_start + 27] << 8) |
        _msg_buff[_msg_buff_data_start + 28]);
    _pitch_deg = -(pitch_raw * VIEWPRO_OUTPUT_TO_DEG);

    _last_angle_ms = AP_HAL::millis();
}

void ViewProCamReader::log_angles()
{
#if HAL_LOGGING_ENABLED
    if (_last_angle_ms == 0) {
        return;
    }
    AP::logger().Write(
        "VCAM", "TimeUS,Yaw,Pitch,Roll", "sddd", "F000", "Qfff",
        AP_HAL::micros64(),
        (double)_yaw_deg, (double)_pitch_deg, (double)_roll_deg);
#endif
}

uint8_t ViewProCamReader::calc_crc(const uint8_t *buf, uint8_t len)
{
    uint8_t res = 0;
    for (uint8_t i = 0; i < len; i++) {
        res ^= buf[i];
    }
    return res;
}
