/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Evan Madurai
  Created on: 13 April 2025
  Description:
*/
#ifndef LOG_H
#define LOG_H

enum { LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR };

#define _logl(level, file, line, fmt, ...) \
_log(level, file, line, fmt "\n", ##__VA_ARGS__)


#ifdef LOGWARN
#define logw(...) _logl(LOG_WARN, __FILE__, __LINE__, __VA_ARGS__)
#else
#define logw(...)
#endif

#ifdef LOGERROR
#define loge(...) _logl(LOG_ERROR, __FILE__, __LINE__, __VA_ARGS__)
#else
#define loge(...)
#endif

#ifdef LOGDEBUG
#define logd(...) _logl(LOG_DEBUG, __FILE__, __LINE__, __VA_ARGS__)
#else
#define logd(...)
#endif

#ifdef LOGINFO
#define logi(...) _logl(LOG_INFO, __FILE__, __LINE__, __VA_ARGS__)
#else
#define logi(...)
#endif

void _log(int level, const char *file, int line, const char *fmt, ...);

#endif //LOG_H
