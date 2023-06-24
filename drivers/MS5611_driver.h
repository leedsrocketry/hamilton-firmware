/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 10 June 2023
  Description: header file for the Barometer module MS561101BA03
*/

#ifndef MS5611_DRIVER_H
#define MS5611_DRIVER_H
#include <stdint.h>

#define MS5611_RESET			    0x1e
#define MS5611_READ_ADC			    0x00
#define MS5611_READ_PROM_WORD		0xA0
#define MS5611_PROM_WORDS_NB		8
#define	SPI_MODE_0					(0|0)		/* (original MicroWire) */


/**
  @brief OverSampling Rate descriptor.
  @warning: cmd MUST be kept aligned on a word boundary (see m5611_spi_read_adc_temp_and_pressure in ms5611_spi.c).
*/
typedef struct OversampleRate {
	unsigned long convUsec;
	uint8_t cmd;
	unsigned short rate;
} OversampleRate;


/**
  @brief TODO
*/
typedef struct State {
  	void *client;
	struct mutex lock;

	const struct OversampleRate pressureOsr;
	const struct OversampleRate tempOsr;

	uint16_t prom[MS5611_PROM_WORDS_NB];

	int (*reset)(State *st);
	int (*read_prom_word)(State *st, int index, uint16_t *word);
	int (*read_adc_temp_and_pressure)(State *st,
					  int32_t *temp, int32_t * pressure);

	int (*compensate_temp_and_pressure)(State *st, int32_t *temp,
					  int32_t *pressure);
} State;


/**
  @brief TODO
*/
int init_MS5611();


/**
  @brief TODO
*/
int probe();


#endif /* MS5611_DRIVER_H */
