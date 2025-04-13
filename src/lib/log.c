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
      printf("[DEBUG] ");
      vprintf(fmt, va);
    break;
  }

  va_end(va);
}



// Print just the variable provided in format 'VAR_NAME: VALUE"
//void _logvar(va_list ap) {
//  printf("%s", va_arg(ap, char *));
//}