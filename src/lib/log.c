/*
Leeds University Rocketry Organisation - LURA
  Author Name: Evan Madurai
  Created on: 13 April 2025
  Description:
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "debug.h"
#include "log.h"

void _log(int level, const char *fmt, ...) {
  va_list va;
  va_start(va, fmt);

  switch (level) {
    case LOG_DEBUG:
      printf("\x1b[34m[DEBUG]    \x1b[0m");
      vprintf(fmt, va);
      break;
    case LOG_INFO:
      printf("\x1b[32m[INFO]     \x1b[0m");
      vprintf(fmt, va);
      break;
    case LOG_WARN:
      printf("\x1b[33m[WARNING]  \x1b[0m");
      vprintf(fmt, va);
      break;
    case LOG_ERROR:
      printf("\x1b[31m[ERROR]    \x1b[0m");
      vprintf(fmt, va);
      break;
    break;
  }
  va_end(va);
}