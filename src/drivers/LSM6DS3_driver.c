/*
    Leeds University Rocketry Organisation - LURA
    Author Name: Oliver Martin
    Created on: 01 March 2024
    Description: Driver file for the IMU module LSM6DS3 (https://www.mouser.co.uk/datasheet/2/389/dm00133076-1798402.pdf)

    ---------------------------------------
    Some basis for this logic comes from the open source Betaflight lsm6dso driver.
*/

#include "LSM6DS3_driver.h"

/**
  @brief Init LSM6DS3 IMU driver
  @param spi Selected SPI
  @param gyro LSM6DS3_data structure for returning data by reference
  @return Error code
*/
uint8_t LSM6DS3_init(SPI_TypeDef *spi, LSM6DS3_data* gyro)
{   
    //check the chip replies with correct ID
    uint8_t chip_id = 0;
    uint8_t send_data[1] = {(LSM6DSO_REG_WHO_AM_I) | 0x80}; 
    //start SPI comms
    spi_enable_cs(spi, LSM6DS3_CS);
    delay_microseconds(1);
    spi_transmit_receive(spi, &send_data, 1, 1, &chip_id);
    delay_microseconds(1);
    spi_disable_cs(spi, LSM6DS3_CS);
    //end of spi comm

    if (chip_id == LSM6DS3_WHO_AM_I_EXP) {
        delay_microseconds(10); 
        LSM6DS3_config(spi);     //configure settings
        delay_microseconds(10); //give delay after setting the settings

        //initialise variables to 0
        gyro->x_offset = 0;
        gyro->y_offset = 0;
        gyro->z_offset = 0;

        // calculate gyro offsets
        LSM6DS3_gyro_offsets(spi, gyro);
        LSM6DS3_acc_read(spi, gyro);
        
        return 1;
    } else {
        LOG("LSM6DS3 wrong chip ID: %d\r\n", chip_id);
    }
    
    return 0;
}

/**
  @brief Write to LSM6DS3 registers
  @param register_id register to write into
  @param value value to written to register
  @param delay Delay in ms
  @return Error code
*/
void LSM6DS3_write_register(SPI_TypeDef *spi, uint8_t register_id, uint8_t value, uint32_t delayMs)
{
    uint8_t send_data[2] =  {register_id, value};
    spi_enable_cs(spi, LSM6DS3_CS);
    delay_microseconds(10);
    spi_transmit_receive(spi, &(send_data[0]), 1, 0, &send_data);
    spi_transmit_receive(spi, &(send_data[1]), 1, 0, &send_data);
    delay_microseconds(1);
    spi_disable_cs(spi, LSM6DS3_CS);
    if (delayMs) {
        delay_ms(delayMs);
    }
}

void LSM6DS3_write_register_bits(SPI_TypeDef *spi, uint8_t register_id, uint8_t mask, uint8_t value, uint32_t delayMs)
{
    uint8_t new_value = 0;
    spi_enable_cs(spi, LSM6DS3_CS);
    delay_microseconds(10);
    uint8_t send_data = register_id | 80;
    spi_transmit_receive(spi, &send_data, 1, 1, &new_value); //get current data
    delay_microseconds(1);
    spi_disable_cs(spi, LSM6DS3_CS);
    
    delay_microseconds(delayMs);
    new_value = (new_value & ~mask) | value;
    LSM6DS3_write_register(spi, register_id, new_value, delayMs);
    delay_microseconds(delayMs);
}

void LSM6DS3_config(SPI_TypeDef *spi){
    // Reset the device (wait 100ms before continuing config)
    LSM6DS3_write_register_bits(spi, LSM6DSO_REG_CTRL3_C, LSM6DSO_MASK_CTRL3_C_RESET, BIT(0), 100);
    delay_ms(100);
    // Configure interrupt pin 1 for gyro data ready only
    //LSM6DS3_write_register(spi, LSM6DSO_REG_INT1_CTRL, LSM6DSO_VAL_INT1_CTRL, 1);

    // Disable interrupt pin 2
    //LSM6DS3_write_register(spi, LSM6DSO_REG_INT2_CTRL, LSM6DSO_VAL_INT2_CTRL, 1);

    // Configure the accelerometer
    // 833hz ODR, 16G scale, use LPF1 output
    LSM6DS3_write_register(spi, LSM6DSO_REG_CTRL1_XL, (LSM6DSO_VAL_CTRL1_XL_ODR833 << 4) | (LSM6DSO_VAL_CTRL1_XL_16G << 2) | (LSM6DSO_VAL_CTRL1_XL_LPF1 << 1), 1);

    // Configure the gyro
    // 6664hz ODR, 2000dps scale
    LSM6DS3_write_register(spi, LSM6DSO_REG_CTRL2_G, (LSM6DSO_VAL_CTRL2_G_ODR6664 << 4) | (LSM6DSO_VAL_CTRL2_G_2000DPS << 2), 1);

    // Configure control register 3
    // latch LSB/MSB during reads; set interrupt pins active high; set interrupt pins push/pull; set 4-wire SPI; enable auto-increment burst reads
    LSM6DS3_write_register_bits(spi, LSM6DSO_REG_CTRL3_C, LSM6DSO_MASK_CTRL3_C, (LSM6DSO_VAL_CTRL3_C_BDU | LSM6DSO_VAL_CTRL3_C_H_LACTIVE | LSM6DSO_VAL_CTRL3_C_PP_OD | LSM6DSO_VAL_CTRL3_C_SIM | LSM6DSO_VAL_CTRL3_C_IF_INC), 1);

    // Configure control register 4
    // enable accelerometer high performane mode; set gyro LPF1 cutoff to 335.5hz
    LSM6DS3_write_register_bits(spi, LSM6DSO_REG_CTRL4_C, LSM6DSO_MASK_CTRL4_C, (LSM6DSO_VAL_CTRL4_C_I2C_DISABLE | LSM6DSO_VAL_CTRL4_C_LPF1_SEL_G), 1);

    // Configure control register 6
    // disable I2C interface; enable gyro LPF1
    LSM6DS3_write_register_bits(spi, LSM6DSO_REG_CTRL6_C, LSM6DSO_MASK_CTRL6_C, (LSM6DSO_VAL_CTRL6_C_XL_HM_MODE | LSM6DSO_VAL_CTRL6_C_FTYPE_335HZ), 1);

    // Configure control register 9
    // disable I3C interface
    LSM6DS3_write_register_bits(spi, LSM6DSO_REG_CTRL9_XL, LSM6DSO_MASK_CTRL9_XL, LSM6DSO_VAL_CTRL9_XL_I3C_DISABLE, 1);
    return;
}

bool LSM6DS3_acc_read(SPI_TypeDef *spi, LSM6DS3_data* gyro)
{
    enum {
        IDX_ACCEL_XOUT_L,
        IDX_ACCEL_XOUT_H,
        IDX_ACCEL_YOUT_L,
        IDX_ACCEL_YOUT_H,
        IDX_ACCEL_ZOUT_L,
        IDX_ACCEL_ZOUT_H,
        BUFFER_SIZE, //6
    };

    // Reset both accel and gyro. Do not ask why.
    LSM6DS3_write_register(SPI1, LSM6DSO_REG_CTRL1_XL, (LSM6DSO_VAL_CTRL1_XL_ODR833 << 4) | (LSM6DSO_VAL_CTRL1_XL_16G << 2) | (LSM6DSO_VAL_CTRL1_XL_LPF1 << 1), 1);
    LSM6DS3_write_register(SPI1, LSM6DSO_REG_CTRL2_G, (LSM6DSO_VAL_CTRL2_G_ODR6664 << 4) | (LSM6DSO_VAL_CTRL2_G_2000DPS << 2), 1);

    uint8_t lsm6ds3_rx_buf[BUFFER_SIZE];
    uint8_t send_data =  LSM6DSO_REG_OUTX_L_A | 0x80;   //first reg address

    spi_enable_cs(spi, LSM6DS3_CS);
    delay_microseconds(1);
    spi_transmit_receive(spi, &(send_data), 1, 1, &(lsm6ds3_rx_buf[0]));  //send read command and get first result
    
    for (int i = 1; i < BUFFER_SIZE; i ++){
        spi_transmit_receive(spi, &(send_data), 0, 1, &(lsm6ds3_rx_buf[i]));
        delay_microseconds(100);
    } 
    delay_microseconds(1);
    spi_disable_cs(spi, LSM6DS3_CS);

    //convert raw bytes into milli G
    gyro->x_accel = LMS6DS6_ACCEL_SENSITIVITY*(int32_t)((int16_t)((lsm6ds3_rx_buf[IDX_ACCEL_XOUT_H] << 8) | lsm6ds3_rx_buf[IDX_ACCEL_XOUT_L]))/1000;
    gyro->y_accel = LMS6DS6_ACCEL_SENSITIVITY*(int32_t)((int16_t)((lsm6ds3_rx_buf[IDX_ACCEL_YOUT_H] << 8) | lsm6ds3_rx_buf[IDX_ACCEL_YOUT_L]))/1000;
    gyro->z_accel = LMS6DS6_ACCEL_SENSITIVITY*(int32_t)((int16_t)((lsm6ds3_rx_buf[IDX_ACCEL_ZOUT_H] << 8) | lsm6ds3_rx_buf[IDX_ACCEL_ZOUT_L]))/1000;
    LOG("Accel: X:%6i, \tY:%6i,\tZ:%6i\r\n", gyro->x_accel, gyro->y_accel, gyro->z_accel);

    return true;
}

bool LSM6DS3_gyro_read(SPI_TypeDef *spi, LSM6DS3_data* gyro)
{
    enum {
        IDX_GYRO_XOUT_L,
        IDX_GYRO_XOUT_H,
        IDX_GYRO_YOUT_L,
        IDX_GYRO_YOUT_H,
        IDX_GYRO_ZOUT_L,
        IDX_GYRO_ZOUT_H,
        BUFFER_SIZE,
    };

    //LSM6DS3_write_register(SPI1, LSM6DSO_REG_CTRL2_G, (LSM6DSO_VAL_CTRL2_G_ODR6664 << 4) | (LSM6DSO_VAL_CTRL2_G_2000DPS << 2), 1);

    uint8_t lsm6ds3_rx_buf[BUFFER_SIZE] = {0};
    uint8_t send_data = (LSM6DSO_REG_OUTX_L_G) | 0x80;  //first reg address

    spi_enable_cs(spi, LSM6DS3_CS);
    delay_microseconds(1);
    spi_transmit_receive(spi, &(send_data), 1, 1, &(lsm6ds3_rx_buf[0])); //send read command and get first result
    for (uint8_t i = 1; i < BUFFER_SIZE; i ++){
        spi_transmit_receive(spi, &(send_data), 0, 1, &(lsm6ds3_rx_buf[i]));
    }
    delay_microseconds(1);
    spi_disable_cs(spi, LSM6DS3_CS);

    //convert raw data into value of milli deg per second
    gyro->x_rate = LMS6DS6_ANGULAR_RATE_SENSITIVITY * (int32_t)((int16_t)((lsm6ds3_rx_buf[IDX_GYRO_XOUT_H] << 8) | lsm6ds3_rx_buf[IDX_GYRO_XOUT_L])) - gyro->x_offset;
    gyro->y_rate = LMS6DS6_ANGULAR_RATE_SENSITIVITY * (int32_t)((int16_t)((lsm6ds3_rx_buf[IDX_GYRO_YOUT_H] << 8) | lsm6ds3_rx_buf[IDX_GYRO_YOUT_L])) - gyro->y_offset;
    gyro->z_rate = LMS6DS6_ANGULAR_RATE_SENSITIVITY * (int32_t)((int16_t)((lsm6ds3_rx_buf[IDX_GYRO_ZOUT_H] << 8) | lsm6ds3_rx_buf[IDX_GYRO_ZOUT_L])) - gyro->z_offset;
    
    printf("GryoR: X:%i, \tY:%i,\tZ:%i\r\n", gyro->x_rate, gyro->y_rate, gyro->z_rate);

    return true;
}

//calculates the gyro offset values
bool LSM6DS3_gyro_offsets(SPI_TypeDef *spi, LSM6DS3_data* gyro)
{
    LSM6DS3_data buff[LSM6DSO_OFFSET_BUFF_LEN];
    int32_t avg[3] = {0,0,0};
    LSM6DS3_gyro_read(spi, gyro);
    delay_ms(300);
    do{
        avg[0] = 0, avg[1] = 0, avg[2] = 0; //reset averages
        for (uint16_t i = 0; i < LSM6DSO_OFFSET_BUFF_LEN; i++){
            LSM6DS3_gyro_read(spi, gyro);
            buff[i] = *gyro;
            avg[0] += buff[i].x_rate;
            avg[1] += buff[i].y_rate;
            avg[2] += buff[i].z_rate;
            delay_microseconds(1000000/100); //delay to read at 50Hz
        }
    }while(!LSM6DS3_gyro_standard_dev(buff, LSM6DSO_OFFSET_BUFF_LEN, 1000));   //if standard deviation of readings is not within limit then its not steady enough & try again
    
    gyro->x_offset = (avg[0] / LSM6DSO_OFFSET_BUFF_LEN);
    gyro->y_offset = (avg[1] / LSM6DSO_OFFSET_BUFF_LEN);
    gyro->z_offset = (avg[2] / LSM6DSO_OFFSET_BUFF_LEN);
    //LOG("Gyro Offsets: %li, %li, %li\r\n", gyro->x_offset, gyro->y_offset, gyro->z_offset);
    return 1;
}

// keeps angle between +-180,000 mDeg
int32_t LSM6DS3_angle_overflow(int32_t mDeg){
    if (mDeg > 180000){
        return mDeg - 360000;
    }else if (mDeg < -180000){
        return mDeg + 360000;
    }
    return mDeg;
}

bool LSM6DS3_get_acc_vector(LSM6DS3_data* _LSM6DS3_data, float vector[]){
    // Convert from milli g to g
    vector[0] = _LSM6DS3_data->x_accel/1000.0;
    vector[1] = _LSM6DS3_data->y_accel/1000.0;
    vector[2] = _LSM6DS3_data->z_accel/1000.0;

    // Check magnitude (in g)
    float magnitude = sqrtf(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2]);
    
    // Normalise the vector
    vector[0] /= magnitude;
    vector[1] /= magnitude;
    vector[2] /= magnitude;
    vector[3] = magnitude;

    if (magnitude < 0.9 || magnitude > 1.1){   // If not close to 1G
        return false;
    }
    return true;
}

bool LSM6DS3_gyro_standard_dev(LSM6DS3_data buff[], uint16_t buffer_limit, uint16_t limit) {
    // Calculate mean
    int means[3] = {0,0,0};
    for (int i = 0; i < buffer_limit; i ++){
        means[0] += buff[i].x_rate;
        means[1] += buff[i].y_rate;
        means[2] += buff[i].z_rate;
    }
    means[0] /= buffer_limit;
    means[1] /= buffer_limit;
    means[2] /= buffer_limit;

    // Calculate variance through (sum of (squares of deviations))/num_samples
    long variance[3] = {0,0,0};
    for (int i = 0; i < buffer_limit; i ++){
        variance[0] += powl(buff[i].x_rate - means[0], 2);
        variance[1] += powl(buff[i].y_rate - means[1], 2);
        variance[2] += powl(buff[i].z_rate - means[2], 2);
    }
    // Divide by samples to get variance
    variance[0] /= buffer_limit;
    variance[1] /= buffer_limit;
    variance[2] /= buffer_limit;

    // Sqrt to get standard deviation
    int std_dev[3] = {sqrt(variance[0]), sqrt(variance[1]), sqrt(variance[2])};
    if (std_dev[0] < limit && std_dev[1] < limit && std_dev[2] < limit) {
        return true;
    }
    return false;
}
