#include "logger.h"
#include <stdio.h>
#include <stdarg.h>

void dump_log(const char *format, ...)
{
    va_list argptr;
    va_start(argptr, format);
    vprintf(format, argptr);
    va_end(argptr);
}
