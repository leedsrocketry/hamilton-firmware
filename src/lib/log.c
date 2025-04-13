/*
Leeds University Rocketry Organisation - LURA
  Author Name: Evan Madurai
  Created on: 13 April 2025
  Description:
*/

#include "log.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "debug.h"

void _log(int level, const char *file, int line, const char *fmt, ...) {
  va_list va;
  va_start(va, fmt);

  switch (level) {
    case LOG_DEBUG:
      printf("\x1b[34m[DEBUG]   \x1b[0m");
      printf("\x1b[90m%s:%d \x1b[0m", file, line);
      vprintf(fmt, va);
      break;
    case LOG_INFO:
      printf("\x1b[32m[INFO]    \x1b[0m");
      printf("\x1b[90m%s:%d \x1b[0m", file, line);
      vprintf(fmt, va);
      break;
    case LOG_WARN:
      printf("\x1b[33m[WARNING] \x1b[0m");
      printf("\x1b[90m%s:%d \x1b[0m", file, line);
      vprintf(fmt, va);
      break;
    case LOG_ERROR:
      printf("\x1b[31m[ERROR]   \x1b[0m");
      printf("\x1b[90m%s:%d \x1b[0m", file, line);
      vprintf(fmt, va);
      break;
      break;
  }
  va_end(va);
}