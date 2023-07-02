/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 10 June 2023
  Description: header file for the Barometer module MS561101BA03
*/

#ifndef MS5611_DRIVER_H
#define MS5611_DRIVER_H
#include <stdint.h>

#pragma region Macros

/** @name Commands */
#define MS5611_CMD_READ_ADC       				0x00
#define MS5611_CMD_READ_PROM      				0xA0
#define MS5611_CMD_RESET          				0x1E
#define MS5611_CMD_CONVERT_D1     				0x40
#define MS5611_CMD_CONVERT_D2     				0x50	

/** @name Oversampling rates */
#define MS5611_PRESSURE_OSR_256  				0x40
#define MS5611_PRESSURE_OSR_512  				0x42
#define MS5611_PRESSURE_OSR_1024 				0x44
#define MS5611_PRESSURE_OSR_2048 				0x46
#define MS5611_PRESSURE_OSR_4096 				0x48

#define MS5611_TEMP_OSR_256      				0x50
#define MS5611_TEMP_OSR_512  	 				0x52
#define MS5611_TEMP_OSR_1024 	 				0x54
#define MS5611_TEMP_OSR_2048     				0x56
#define MS5611_TEMP_OSR_4096     				0x58

#pragma endregion Macros

#pragma region Structs/Emun
/**
	@brief The oversampling rate
	@note A higher value means a longer conversion
*/
typedef enum MS5611_OSR {
	MS5611_OSR_256,
	MS5611_OSR_512,
	MS5611_OSR_1024,
	MS5611_OSR_2048,
	MS5611_OSR_4096
}MS5611_OSR;
#pragma endregion Structs/Emun

/**
  @brief TODO
*/
int_8 init_MS5611();


#endif /* MS5611_DRIVER_H */
