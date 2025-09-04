#ifndef LOGGER_H
#define LOGGER_H

#include "logger_config.h"

#ifdef ENABLE_LOG_DEBUG
#define log_debug(format, ...)  dump_log(format, ##__VA_ARGS__)
#else
#define log_debug(...) (void)(0)
#endif

#ifdef ENABLE_LOG_INFO
#define log_info(format, ...)  dump_log(format, ##__VA_ARGS__)
#else
#define log_info(...) (void)(0)
#endif

#ifdef ENABLE_LOG_ERROR
#define log_error(format, ...)  dump_log(format, ##__VA_ARGS__)
#else
#define log_error(...) (void)(0)
#endif

extern void dump_log(const char * format, ...);

#endif // LOGGER_H
