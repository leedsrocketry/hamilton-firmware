/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Evan Madurai
  Created on: 27 July 2024
  Description: generic header file for storing debug config settings
*/

#ifndef DEBUG_H
#define DEBUG_H

// Remove debug flag here to remove logging and improve performance
// Can also be done with compiler flags
#define DEBUG

#ifdef DEBUG
#include <stdio.h>
#define LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define LOG(fmt, ...)
#endif

#endif /* DEBUG_H */
