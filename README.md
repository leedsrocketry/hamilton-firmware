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

Written by Alexandra Posta, Evan Madurai, Ollie Martin, Tyler Green, and Thomas Groom.

## Building and flashing
We use GNUMake, and we have all the required dependencies in a Dockerfile.

Here is an example of building and flashing using the Docker container on MacOS/Linux
```console
> source commands.sh
> build
> run
> make hfc-flash
```

And to see the serial output, in a different terminal:
```
make serial
```
