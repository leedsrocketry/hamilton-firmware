/*
	Leeds University Rocketry Organisation - LURA
    Author Name: Alexandra Posta
    Created on: 20 Feb 2023
	Last modified on: 20 Feb 2023
    Description: Driver file for the GNSS module MAX-M10S-00B (https://www.mouser.co.uk/ProductDetail/u-blox/MAX-M10S-00B?qs=A6eO%252BMLsxmT0PfQYPb7LLQ%3D%3D)
*/

#include <stdint.h>

#include "MAXM10S_driver.h"

union u_Short uShort;
union i_Short iShort;
union u_Long uLong;
union i_Long iLong;

void MAXM10S_driver::init(GNSS_StateHandle *GNSS, GNSS_Context *ctx, GNSS_Config *config)
{
	// check version

	// put in sleep mode

	// set frequency

	// initialize transmission parameters
	GNSS_Init(GNSS_StateHandle *GNSS, UART_HandleTypeDef *huart);
}


void MAXM10S_driver::config_setup(GNSS_Config *config)
{

}


void MAXM10S_driver::reset(GNSS_Context *ctx)
{

}


void MAXM10S_driver::write(GNSS_Context *ctx, char *data_buf, uint16_t len)
{

}


void MAXM10S_driver::read(GNSS_Context *ctx, char *data_buf, uint16_t max_len)
{

}


/*!
 * Structure initialization for the Global navigation satellite system.
 * @param GNSS Pointer to main GNSS structure.
 * @param huart Pointer to uart handle.
 */
void GNSS_Init(GNSS_StateHandle *GNSS, UART_HandleTypeDef *huart) {
	GNSS->huart = huart;
	GNSS->year = 0;
	GNSS->month = 0;
	GNSS->day = 0;
	GNSS->hour = 0;
	GNSS->min = 0;
	GNSS->sec = 0;
	GNSS->fixType = 0;
	GNSS->lon = 0;
	GNSS->lat = 0;
	GNSS->height = 0;
	GNSS->hMSL = 0;
	GNSS->hAcc = 0;
	GNSS->vAcc = 0;
	GNSS->gSpeed = 0;
	GNSS->headMot = 0;
}


/*!
 * Searching for a header in data buffer and matching class and message ID to buffer data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_ParseBuffer(GNSS_StateHandle *GNSS) {

	for (int var = 0; var <= 100; ++var) {
		if (GNSS->uartWorkingBuffer[var] == 0xB5
				&& GNSS->uartWorkingBuffer[var + 1] == 0x62) {
			if (GNSS->uartWorkingBuffer[var + 2] == 0x27
					&& GNSS->uartWorkingBuffer[var + 3] == 0x03) { // Look at: 32.19.1.1 u-blox 8 Receiver description
				GNSS_ParseUniqID(GNSS);
			} else if (GNSS->uartWorkingBuffer[var + 2] == 0x01
					&& GNSS->uartWorkingBuffer[var + 3] == 0x21) { // Look at: 32.17.14.1 u-blox 8 Receiver description
				GNSS_ParseNavigatorData(GNSS);
			} else if (GNSS->uartWorkingBuffer[var + 2] == 0x01
					&& GNSS->uartWorkingBuffer[var + 3] == 0x07) { // Look at: 32.17.30.1 u-blox 8 Receiver description
				GNSS_ParsePVTData(GNSS);
			} else if (GNSS->uartWorkingBuffer[var + 2] == 0x01
					&& GNSS->uartWorkingBuffer[var + 3] == 0x02) { // Look at: 32.17.15.1 u-blox 8 Receiver description
				GNSS_ParsePOSLLHData(GNSS);
			}
		}
	}
}


/*!
 * Make request for unique chip ID data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_GetUniqID(GNSS_StateHandle *GNSS) {
	HAL_UART_Transmit_DMA(GNSS->huart, getDeviceID,
			sizeof(getDeviceID) / sizeof(uint8_t));
	HAL_UART_Receive_IT(GNSS->huart, GNSS_Handle.uartWorkingBuffer, 17);
}


/*!
 * Make request for UTC time solution data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_GetNavigatorData(GNSS_StateHandle *GNSS) {
	HAL_UART_Transmit_DMA(GNSS->huart, getNavigatorData,
			sizeof(getNavigatorData) / sizeof(uint8_t));
	HAL_UART_Receive_IT(GNSS->huart, GNSS_Handle.uartWorkingBuffer, 28);
}


/*!
 * Parse data to unique chip ID standard.
 * Look at: 32.19.1.1 u-blox 8 Receiver description
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_ParseUniqID(GNSS_StateHandle *GNSS) {
	for (int var = 0; var < 5; ++var) {
		GNSS->uniqueID[var] = GNSS_Handle.uartWorkingBuffer[10 + var];
	}
}


/*!
 * Changing the GNSS mode.
 * Look at: 32.10.19 u-blox 8 Receiver description
 */
void GNSS_SetMode(GNSS_StateHandle *GNSS, short gnssMode) {
	if (gnssMode == 0) {
		HAL_UART_Transmit_DMA(GNSS->huart, setPortableMode,sizeof(setPortableMode) / sizeof(uint8_t));
	} else if (gnssMode == 1) {
		HAL_UART_Transmit_DMA(GNSS->huart, setStationaryMode,sizeof(setStationaryMode) / sizeof(uint8_t));
	} else if (gnssMode == 2) {
		HAL_UART_Transmit_DMA(GNSS->huart, setPedestrianMode,sizeof(setPedestrianMode) / sizeof(uint8_t));
	} else if (gnssMode == 3) {
		HAL_UART_Transmit_DMA(GNSS->huart, setAutomotiveMode,sizeof(setAutomotiveMode) / sizeof(uint8_t));
	} else if (gnssMode == 4) {
		HAL_UART_Transmit_DMA(GNSS->huart, setAutomotiveMode,sizeof(setAutomotiveMode) / sizeof(uint8_t));
	} else if (gnssMode == 5) {
		HAL_UART_Transmit_DMA(GNSS->huart, setAirbone1GMode,sizeof(setAirbone1GMode) / sizeof(uint8_t));
	} else if (gnssMode == 6) {
		HAL_UART_Transmit_DMA(GNSS->huart, setAirbone2GMode,sizeof(setAirbone2GMode) / sizeof(uint8_t));
	} else if (gnssMode == 7) {
		HAL_UART_Transmit_DMA(GNSS->huart, setAirbone4GMode,sizeof(setAirbone4GMode) / sizeof(uint8_t));
	} else if (gnssMode == 8) {
		HAL_UART_Transmit_DMA(GNSS->huart, setWirstMode,sizeof(setWirstMode) / sizeof(uint8_t));
	} else if (gnssMode == 9) {
		HAL_UART_Transmit_DMA(GNSS->huart, setBikeMode,sizeof(setBikeMode) / sizeof(uint8_t));
	}
}


/*!
 * Parse data to UTC time solution standard.
 * Look at: 32.17.30.1 u-blox 8 Receiver description.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_ParseNavigatorData(GNSS_StateHandle *GNSS) {
	uShort.bytes[0] = GNSS_Handle.uartWorkingBuffer[18];
	uShort.bytes[1] = GNSS_Handle.uartWorkingBuffer[19];
	GNSS->year = uShort.uShort;
	GNSS->month = GNSS_Handle.uartWorkingBuffer[20];
	GNSS->day = GNSS_Handle.uartWorkingBuffer[21];
	GNSS->hour = GNSS_Handle.uartWorkingBuffer[22];
	GNSS->min = GNSS_Handle.uartWorkingBuffer[23];
	GNSS->sec = GNSS_Handle.uartWorkingBuffer[24];
}