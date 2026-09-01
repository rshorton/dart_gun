#ifndef LOGGING__H
#define LOGGING__H

#include "Arduino.h"

#define LOG_LVL_NEVER   0
#define LOG_LVL_DEBUG   1
#define LOG_LVL_INFO    2
#define LOG_LVL_WARN    3
#define LOG_LVL_ERROR   4

namespace Logging
{
void log_message(int level, const char * fmt, ...);
uint8_t get_log_level();
void set_log_level(uint8_t);

}

#endif // LOGGING__H