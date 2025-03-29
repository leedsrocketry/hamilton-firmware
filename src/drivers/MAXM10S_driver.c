/*
        Leeds University Rocketry Organisation - LURA
  Author Name:
  Created on:
  Description: Driver file for the GNSS module MAX-M10S-00B
  (https://www.mouser.co.uk/ProductDetail/u-blox/MAX-M10S-00B?qs=A6eO%252BMLsxmT0PfQYPb7LLQ%3D%3D)
*/

#include "MAXM10S_driver.h"

#include <fcntl.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>