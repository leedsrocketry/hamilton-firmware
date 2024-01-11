/*
    Leeds University Rocketry Organisation - LURA
    Author Name: Alexandra Posta, Evan Madurai
    Created on: 10 June 2023
    Description: Driver file for the Accelerometer module ADXL375 (https://www.mouser.co.uk/ProductDetail/Analog-Devices/ADXL375BCCZ?qs=S4ILP0tmc7Q%2Fd%2FHPWf9YpQ%3D%3D)
*/

#include "ADXL375_driver.h"

// Public functions to be called from main flight computer
#pragma region Public

void init_ADXL375(SPI_TypeDef spi){
    // Get the device into 4-wire SPI mode before proceeding
    ADXL375_SPI = spi;
    ADXL375_reg_write(ADXL375_DATA_FORMAT, ADXL375_DATA_FORMAT_SETTINGS(0));

    // Power the CTL Register
    spi_write_byte(ADXL375_SPI, ADXL375_POWER_REG); 

    // Check the device name
    uint8_t	devid = spi_read_byte(ADXL375_SPI, ADXL375_DEVID);
    if (devid != ADXL375_DEVID_ID)
		exit();

    // Set the data rate
    ADXL375_reg_write(ADXL375_BW_RATE,
			    (0 << ADXL375_BW_RATE_LOW_POWER) |
			    (ADXL375_BW_RATE_RATE_200 << ADXL375_BW_RATE_RATE));

    // Set the offset to zero
    ADXL375_reg_write(ADXL375_OFSX, 0);
    ADXL375_reg_write(ADXL375_OFSY, 0);
    ADXL375_reg_write(ADXL375_OFSZ, 0);   

    // Clear interrupts
	ADXL375_reg_write(ADXL375_INT_ENABLE, 0);     

    // Place in measurement mode
    ADXL375_reg_write(ADXL375_POWER_CTL,
			    (0 << ADXL375_POWER_CTL_LINK) |
			    (0 << ADXL375_POWER_CTL_AUTO_SLEEP) |
			    (1 << ADXL375_POWER_CTL_MEASURE) |
			    (0 << ADXL375_POWER_CTL_SLEEP) |
			    (ADXL375_POWER_CTL_WAKEUP_8 << ADXL375_POWER_CTL_WAKEUP)); 

    // Perform self checks
    struct ADXL375_data	self_test_off, self_test_on;

    ADXL375_reg_write(ADXL375_DATA_FORMAT, ADXL375_DATA_FORMAT_SETTINGS(1)); 

    // Discard some samples to let it settle down
	ADXL375_get_test_value(&self_test_off, ADXL375_SELF_TEST_SETTLE);

	// Get regular values 
	ADXL375_get_test_value(&self_test_off, ADXL375_SELF_TEST_SAMPLES);

    // Turn back to normal mode
    ADXL375_reg_write(ADXL375_DATA_FORMAT, ADXL375_DATA_FORMAT_SETTINGS(0));

    // Discard some samples to let it settle down
	ADXL375_get_test_value(&self_test_off, ADXL375_SELF_TEST_SETTLE);

	// Get regular values 
	ADXL375_get_test_value(&self_test_off, ADXL375_SELF_TEST_SAMPLES);

    // Verify if Z axis values are in range
    int16_t	z_change = self_test_on.z - self_test_off.z;
    self_test_value = z_change;

    if (z_change < MIN_SELF_TEST)
		AO_SENSOR_ERROR(AO_DATA_ADXL375);
};


static ADXL375_data get_data_ADXL375(){
    struct ADXL375_data data;
    // x-axis
    spi_write_byte(ADXL375_SPI, ADXL375_X_REG_DATAX0);
    spi_write_byte(ADXL375_SPI, ADXL375_X_REG_DATAX1);
    spi_ready_read(ADXL375_SPI);

    x_0 = spi_read_byte(ADXL375_SPI, ADXL375_MEASURE);
    x_1 = spi_read_byte(ADXL375_SPI, ADXL375_MEASURE);
    data.x = (x_1 << 8) | x_0;

    // y-axis
    spi_write_byte(ADXL375_SPI, ADXL375_Y_REG_DATAX0);
    spi_write_byte(ADXL375_SPI, ADXL375_y_REG_DATAX1);
    spi_ready_read(ADXL375_SPI);

    y_0 = spi_read_byte(ADXL375_SPI, ADXL375_MEASURE);
    y_1 = spi_read_byte(ADXL375_SPI, ADXL375_MEASURE);
    data.y = (y_1 << 8) | y_0;

    // z-axis
    spi_write_byte(ADXL375_SPI, ADXL375_Z_REG_DATAX0);
    spi_write_byte(ADXL375_SPI, ADXL375_Z_REG_DATAX1);
    spi_ready_read(ADXL375_SPI);

    z_0 = spi_read_byte(ADXL375_SPI, ADXL375_MEASURE);
    z_1 = spi_read_byte(ADXL375_SPI, ADXL375_MEASURE);
    data.z = (z_1 << 8) | z_0;

    return data
};

#pragma endregion Public

// Implement private functions
#pragma region Private
#define ADXL375_DATA_FORMAT_SETTINGS(self_test) (			\
		ADXL375_DATA_FORMAT_FIXED |				\
		(self_test << ADXL375_DATA_FORMAT_SELF_TEST) |	\
		(ADXL375_DATA_FORMAT_SPI_4_WIRE << ADXL375_DATA_FORMAT_SPI) | \
		(0 << ADXL375_DATA_FORMAT_INT_INVERT) |		\
		(0 << ADXL375_DATA_FORMAT_JUSTIFY))


static void ADXL375_start(void) {
	ao_spi_get_bit(ADXL375_CS_PORT,
        ADXL375_CS_PIN,
        ADXL375_SPI_INDEX,
        ADXL375_SPI_SPEED);
}


void ADXL375_reg_write(uint8_t addr, uint8_t value)
{
    spi_write_byte(ADXL375_SPI, addr);
    spi_write_byte(ADXL375_SPI, value);
}

static void ADXL375_get_test_value(struct ADXL375_data *data, int samples)
{	
	for (int i = 0; i < samples; i++) {
		spi_write_byte(ADXL375_SPI, ADXL375_Z_REG_DATAX0);
        spi_write_byte(ADXL375_SPI, ADXL375_Z_REG_DATAX1);
        spi_ready_read(ADXL375_SPI);

        z_0 = spi_read_byte(ADXL375_SPI, ADXL375_MEASURE);
        z_1 = spi_read_byte(ADXL375_SPI, ADXL375_MEASURE);
   
		data->x += 0;
		data->y += 0;
		data->z += (z_1 << 8) | z_0;
		delay(10);
	}
}

#pragma endregion Private