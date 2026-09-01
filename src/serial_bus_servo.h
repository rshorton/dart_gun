#ifndef HIW_SERIAL_BUS_SERVO_H
#define HIW_SERIAL_BUS_SERVO_H

#include "Arduino.h"
#include "SoftwareSerial.h"

class SerialServo
{
public:
    SerialServo(Stream &serial, int16_t id, int16_t range_degrees, int16_t range_steps,
                float zero_deg_offset, bool invert);

    void move_to_position(int16_t position, uint16_t time);
    float move_to_angle(float angle, uint16_t time);

    bool read_position(int16_t &position);
    bool read_angle(float &angle);

    void set_id(uint8_t newID);

private:
    uint8_t calc_ck_sum(uint8_t buf[], int buf_len);
    int angle_to_steps(float angle);
    float steps_to_angle(int16_t steps);
    void read_flush();
    bool read_response(int skip_bytes, uint8_t read_buf[], int buf_len, uint32_t timeout_ms);

private:
    Stream &serial_;
    int16_t id_;
    int16_t range_degrees_;
    int16_t range_steps_;
    float zero_deg_offset_;
    bool invert_;
};

#endif // HIW_SERIAL_BUS_SERVO_H
