# Hamilton Flight Computer Firmware
This repository holds the firmware for the Hamilton Flight Computer (HFC) designed and built by LURA - Leeds University Rocketry Association.

The Hamilton Flight Computer was designed for Gryphon 2 (G2), a high power amateur rocket designed to break the UK student altitude record.

Features:
* STM32 processor
* Sensors
  * ADXL375 - High G Accelerometer
  * BME280 - Humidity Sensor
  * LSM6DS3 - Accelerometer and Gyroscope 
  * MAX31855 - Thermocoupler
  * MAXM10S - GNSS Sensor
  * MS5611 - Barometer 
  * SI446 - Pad Radio module

Tooling:
hamilton-firmware is:
* Compiled with arm-none-eabi-gcc.
* Debugged with either openocd or probe-rs .
* Flashed with either st-flash or probe-rs.
* Formatted with clang-format.
* Statically analysed with cppcheck (or clang-analyse).


More specific documentation is stored on Athena at https://athena.leedsrocketry.co.uk/doku.php?id=g2:avionics:home

Written by Evan Madurai, Alexandra Posta, Ollie Martin, Tyler Green, and Thomas Groom.