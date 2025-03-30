# Hamilton Flight Computer Firmware
This repository holds the firmware for the Hamilton Flight Computer (HFC) designed and built by LURA - Leeds University Rocketry Association.

The Hamilton Flight Computer was designed for Gryphon 2 (G2), a high power amateur rocket designed to break the UK student altitude record.

## Features:
* STM32 processor
* Sensors
  * ADXL375 - High G Accelerometer
  * BME280 - Humidity Sensor
  * LSM6DS3 - Accelerometer and Gyroscope 
  * MAX31855 - Thermocoupler
  * MAXM10S - GNSS Sensor
  * MS5611 - Barometer 
  * SI446 - Pad Radio module

## Tooling:
hamilton-firmware is:
* Compiled with arm-none-eabi-gcc.
* Debugged with either openocd or probe-rs .
* Flashed with either st-flash or probe-rs.
* Formatted with clang-format.
* Statically analysed with cppcheck (or clang-analyse).

## Building and running:
### make hfc-flash
Builds the firmware for the HFC v2 board and flashes using probe-rs. Logs will be shown to the current terminal over RTT.

### make erase
Builds a specific version of the firmware that erases the NAND flash storage.

### make read
Builds a specific version of the firmware that reads the NAND flash storage out to the current terminal.

### make check
Runs cppcheck static analyser recursively over all .c and .h files in src/

### make format
Runs clang-format recursively over all .c and .h files in src/ using Google formatting standard + longer line widths.

The format for this formatting is defined in `.clang-format1`

### make debug-hardware
Opens openocd with default config for our chip. This allows debugging from your IDE using GDB. To use this with VSCode:
1. Install native debug extension
2. Run `make hfc-flash`
3. Run `make hard-waredebug`
4. Click the VSCode "Run and Debug" button to attach the debugger.

## Other info
More specific documentation is stored on Athena at https://athena.leedsrocketry.co.uk/doku.php?id=g2:avionics:home

Written by Evan Madurai, Alexandra Posta, Ollie Martin, Tyler Green, and Thomas Groom.