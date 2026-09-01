#include "Arduino.h"
#include "logging.h"

namespace
{
const uint8_t MAX_LOG_MSG_LEN = 100;

// Only allow logging when not using serial command interface
uint8_t log_level = LOG_LVL_INFO;

// Serial2 exposed via header.  Need to use USB-serial adapter (3.3v)
Stream &debug_serial = Serial2;
}

namespace Logging
{

void set_serial_port(Stream &serial)
{
    debug_serial = serial;
}

void log_message(int level, const char * fmt, ...)
{
    if (level < log_level) {
        return;
    }
    char buf[MAX_LOG_MSG_LEN];
    va_list args;
    va_start(args, fmt);

    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    debug_serial.println(buf);
}

uint8_t get_log_level()
{
    return log_level;
}

void set_log_level(uint8_t level)
{
    log_level = level;
}

}
