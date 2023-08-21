/*
	Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 20 Feb 2023
  Description: Driver file for the GNSS module MAX-M10S-00B (https://www.mouser.co.uk/ProductDetail/u-blox/MAX-M10S-00B?qs=A6eO%252BMLsxmT0PfQYPb7LLQ%3D%3D)
*/

#include <stdarg.h>
#include "MAXM10S_driver.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>