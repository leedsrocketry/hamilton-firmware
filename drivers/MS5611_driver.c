/*
	Leeds University Rocketry Organisation - LURA
    Author Name: Alexandra Posta
    Created on: 10 June 2023
	Last modified on: 10 June 2023
    Description: Driver file for the Barometer module MS561101BA03 (https://www.te.com/usa-en/product-MS560702BA03-50.html)
*/

#include "MS5611_driver.h"


/**
  @brief TODO
  @param State
  @return
*/
static int reset(State *st)
{
	uint8_t cmd = MS5611_RESET;
	return spi_write_then_read(st->client, &cmd, 1, NULL, 0);
}


/**
  @brief TODO
  @param State
  @param index
  @param word
  @return
*/
static int read_prom_word(State *st, int index, uint16_t *word)
{
	int ret;

	ret = spi_w8r16be(st->client, MS5611_READ_PROM_WORD + (index << 1));
	if (ret < 0)
		return ret;

	*word = ret;

	return 0;
}


/**
  @brief TODO
  @param State
  @param val
  @return
*/
static int ms5611_spi_read_adc(State *st, int32_t *val)
{
	int ret;
	uint8_t buf[3] = { MS5611_READ_ADC };

	ret = spi_write_then_read(st->client, buf, 1, buf, 3);
	if (ret < 0)
		return ret;

	*val = get_unaligned_be24(&buf[0]);

	return 0;
}


/**
  @brief TODO
  @param State
  @param temp
  @param pressure
  @return
*/
static int read_adc_temp_and_pressure(State *st, int32_t *temp, int32_t *pressure)
{
	int ret;
	OversampleRate *osr = &(st->temp_osr);

	/*
	 * Warning: &osr->cmd MUST be aligned on a word boundary since used as
	 * 2nd argument (void*) of spi_write_then_read.
	 */
	ret = spi_write_then_read(st->client, &osr->cmd, 1, NULL, 0);
	if (ret < 0)
		return ret;

	usleep_range(osr->conv_usec, osr->conv_usec + (osr->conv_usec / 10UL));
	ret = ms5611_spi_read_adc(st, temp);
	if (ret < 0)
		return ret;

	osr = &(st->pressure_osr);
	ret = spi_write_then_read(st->client, &osr->cmd, 1, NULL, 0);
	if (ret < 0)
		return ret;

	usleep_range(osr->conv_usec, osr->conv_usec + (osr->conv_usec / 10UL));
	return ms5611_spi_read_adc(st, pressure);
}


/**
  @brief TODO
  @param spi
  @return
*/
static int probe(struct spi_device *spi)
{
	int ret;
	State *st;

    /*
	struct iio_dev *indio_dev;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return 0; // ERROR

	spi_set_drvdata(spi, indio_dev);

	spi->mode = SPI_MODE_0;
	spi->max_speed_hz = min(spi->max_speed_hz, 20000000U);
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret < 0)
		return ret;
    */
	st->reset = reset;
	st->read_prom_word = read_prom_word;
	st->read_adc_temp_and_pressure = read_adc_temp_and_pressure;
	st->client = spi;

	//return ms5611_probe(indio_dev, &spi->dev, spi_get_device_id(spi)->name, spi_get_device_id(spi)->driver_data);
}
