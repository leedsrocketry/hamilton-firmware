/*
    Leeds University Rocketry Organisation - LURA
    Author Name: Alexandra Posta
    Created on: 15 December 2023
    Description: Driver file for the Accelerometer module ADXL375 (https://www.mouser.co.uk/ProductDetail/Analog-Devices/ADXL375BCCZ?qs=S4ILP0tmc7Q%2Fd%2FHPWf9YpQ%3D%3D)
*/

#include "ADXL375_driver.h"

// Public functions to be called from main flight computer
#pragma region Public

void ADXL375_init(SPI_TypeDef spi){
    // Set up SPI
    ADXL375_SPI = spi;

    // Get the device into 4-wire SPI mode before proceeding
    ADXL375_reg_write(ADXL375_DATA_FORMAT, ADXL375_DATA_FORMAT_SETTINGS(0));

    // Power the CTL Register
    spi_transmit_receive(ADXL375_SPI, ADXL375_CS, ADXL375_POWER_REG, 1, 1); 

    // Check the device name
    uint8_t	devid = spi_transmit_receive(ADXL375_SPI, ADXL375_CS, ADXL375_DEVID, 1, 1);
    if (devid != ADXL375_DEVID_ID)
		exit();

    printf("ADXL375 device ID: %d\n", devid);

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
};


static ADXL375_data ADXL375_get_data(){
    struct ADXL375_data data;

    // x-axis
    uint16_t commnds_x[1] = {ADXL375_X_REG_DATAX0, ADXL375_X_REG_DATAX1};
    uint16_t x[1];
    for (int i = 0; i < 2; i++){
        x[i] = spi_transmit_receive(ADXL375_SPI, ADXL375_CS, commnds_x[i], 1, 1); 
    }
    data.x = (x[1] << 8) | x[0];
    printf("x: %d, ", data.x);

    // y-axis
    uint16_t commnds_y[1] = {ADXL375_Y_REG_DATAY0, ADXL375_Y_REG_DATAY1};
    uint16_t y[1];
    for (int i = 0; i < 2; i++){
        y[i] = spi_transmit_receive(ADXL375_SPI, ADXL375_CS, commnds_y[i], 1, 1); 
    }
    data.y = (y[1] << 8) | y[0];
    printf("y: %d, ", data.y);

    // z-axis
    uint16_t commnds_z[1] = {ADXL375_Z_REG_DATAZ0, ADXL375_Z_REG_DATAZ1};
    uint16_t z[1];
    for (int i = 0; i < 2; i++){
        z[i] = spi_transmit_receive(ADXL375_SPI, ADXL375_CS, commnds_z[i], 1, 1); 
    }
    data.z = (z[1] << 8) | z[0];
    printf("z: %d,/n", data.z);

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


void ADXL375_reg_write(uint8_t addr, uint8_t value)
{
    spi_transmit_receive(ADXL375_SPI, ADXL375_CS, addr, 1, 1);
    spi_transmit_receive(ADXL375_SPI, ADXL375_CS, value, 1, 1);
}

/*
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
*/
#pragma endregion Private


/* ADD UNDER INIT
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

    // TODO: Check if the self test value is valid


static void ADXL375_start(void) {	
    TO DO: configure chip for: 
        ADXL375_CS_PIN,
        ADXL375_SPI_INDEX,
        ADXL375_SPI_SPEED);
}
*/