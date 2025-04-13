/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Evan Madurai
  Created on: 13 April 2025
  Description:
*/
#ifndef LOG_H
#define LOG_H

enum { LOG_TRACE, LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR, LOG_FATAL };

void _log(int level, const char *fmt, ...);

#endif //LOG_H
