#include "Arduino.h"

#include "logging.h"
#include "serial_bus_servo.h"

#define GET_LOW_BYTE(A) (uint8_t)((A))
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)

namespace
{
    const uint8_t MSG_FRAME_HEADER = 0x55;
    const uint8_t MSG_CMD_MOVE_TIME_WRITE = 1;
    const uint8_t MSG_CMD_ID_WRITE = 13;
    const uint8_t MSG_CMD_POS_READ = 28;

    const uint32_t READ_TIMEOUT_MS = 200;
}

SerialServo::SerialServo(Stream &serial, int16_t id, int16_t range_degrees, int16_t range_steps,
                         float zero_deg_offset, bool invert) :
    serial_(serial),
    id_(id),
    range_degrees_(range_degrees),
    range_steps_(range_steps),
    zero_deg_offset_(zero_deg_offset),
    invert_(invert)
{
}

uint8_t SerialServo::calc_ck_sum(uint8_t buf[], int buf_len)
{
    uint8_t i;
    uint16_t temp = 0;

    // buf_len should be 1 more than the number of
    // bytes to be used for the calc
    if (buf[3] + 2 >= buf_len) {
        return 0;
    }

    for (i = 2; i < buf[3] + 2; i++)
    {
      temp += buf[i];
    }
  
    temp = ~temp;
    i = (uint8_t)temp;
    return i;
}

void SerialServo::move_to_position(int16_t position, uint16_t time)
{
    read_flush();

    const int msgLen = 10;
    uint8_t buf[msgLen];
    if (position < 0)
    {
        position = 0;
    }

    if (position > range_steps_)
    {
        position = range_steps_;
    }

    if (invert_)
    {
      position = range_steps_ - position;
    }

    buf[0] = buf[1] = MSG_FRAME_HEADER;
    buf[2] = id_;
    buf[3] = msgLen - 3;
    buf[4] = MSG_CMD_MOVE_TIME_WRITE;
    buf[5] = GET_LOW_BYTE(position);
    buf[6] = GET_HIGH_BYTE(position);
    buf[7] = GET_LOW_BYTE(time);
    buf[8] = GET_HIGH_BYTE(time);
    buf[9] = calc_ck_sum(buf, msgLen);
    serial_.write(buf, msgLen);
    serial_.flush();
}

float SerialServo::move_to_angle(float angle, uint16_t time)
{
    int16_t position = angle_to_steps(angle);
    move_to_position(position, time);
    return steps_to_angle(position);
}

void SerialServo::set_id(uint8_t new_id)
{
    const int msgLen = 7;
    uint8_t buf[msgLen];
    buf[0] = buf[1] = MSG_FRAME_HEADER;
    buf[2] = id_;
    buf[3] = msgLen - 3;
    buf[4] = MSG_CMD_ID_WRITE;
    buf[5] = new_id;
    buf[6] = calc_ck_sum(buf, msgLen);
    serial_.write(buf, msgLen);

    id_ = new_id;
}

void SerialServo::read_flush()
{
    while (serial_.available() > 0) {
        serial_.read();
    }        
}

bool SerialServo::read_position(int16_t &position)
{
    const int msgLen = 6;
    uint8_t buf[msgLen];

    // Make sure any previous write has been xmitted
    delay(5);
    read_flush();

    buf[0] = buf[1] = MSG_FRAME_HEADER;
    buf[2] = id_;
    buf[3] = msgLen - 3;
    buf[4] = MSG_CMD_POS_READ;
    buf[5] = calc_ck_sum(buf, msgLen);
    serial_.write(buf, msgLen);

    // The hardwire interface used for the servos on the dart gun
    // results in the xmitted cmd being received too.  As such it needs
    // to be ignored.
    int16_t pos = 0;
    const int msgLenRead = 8;
    uint8_t read_buf[msgLenRead];
    if (read_response(msgLen, read_buf, msgLenRead, READ_TIMEOUT_MS)) {
        position = read_buf[5] | (read_buf[6] << 8);
        return true;
    }
    return false;
}

bool SerialServo::read_angle(float &angle)
{
    angle = 0.0f;

    int16_t pos;
    if (read_position(pos)) {
        angle = steps_to_angle(pos);
        //Logging::log_message(LOG_LVL_DEBUG, "read_angle, position: %d, angle: %f", pos, angle);
        return true;
    }
    return false;
}


bool SerialServo::read_response(int skip_bytes, uint8_t read_buf[], int buf_len, uint32_t timeout_ms)
{
    if (skip_bytes < 0 || buf_len < 0) {
        return false;
    }

    uint32_t start = millis();
    int read_cnt = 0;

    while (read_cnt < buf_len) {
        if (serial_.available() > 0) {
            uint8_t b = serial_.read();
            //Serial.print(b, HEX);
            //Serial.print(" ");

            if (skip_bytes == 0) {
                read_buf[read_cnt++] = b;
                if (read_cnt == buf_len) {
                    break;
                }
            } else {
                --skip_bytes;
            }
        }
        if (millis() - start > timeout_ms) {
            return false;
        }
    }

    uint8_t ck_sum = calc_ck_sum(read_buf, buf_len);
    if (ck_sum != read_buf[buf_len - 1]) {
        return false;
    }
    return true;
}

int SerialServo::angle_to_steps(float angle)
{
    auto angle_abs = min(max(0.0f, zero_deg_offset_ + angle), (float)range_degrees_);

    int16_t steps = (float)range_steps_*angle_abs/(float)range_degrees_;
    if (steps < 0)
    {
        steps = 0;
    }
    else if (steps > range_steps_)
    {
        steps = range_steps_;
    }
    return steps;
}

float SerialServo::steps_to_angle(int16_t steps)
{
    return (float)range_degrees_*(float)steps/(float)range_steps_ - zero_deg_offset_;
}