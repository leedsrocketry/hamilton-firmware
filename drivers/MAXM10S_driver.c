/*
	Leeds University Rocketry Organisation - LURA
    Author Name: Alexandra Posta
    Created on: 20 Feb 2023
	Last modified on: 20 Feb 2023
    Description: Driver file for the GNSS module MAX-M10S-00B (https://www.mouser.co.uk/ProductDetail/u-blox/MAX-M10S-00B?qs=A6eO%252BMLsxmT0PfQYPb7LLQ%3D%3D)
*/

#include <stdarg.h>
#include "MAXM10S_driver.h"
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

DevUBLOXGNSS::DevUBLOXGNSS(void)
{
  // Constructor
  if (debugPin >= 0)
  {
    pinMode((uint8_t)debugPin, OUTPUT);
    digitalWrite((uint8_t)debugPin, HIGH);
  }

  _logNMEA.all = 0;                             // Default to passing no NMEA messages to the file buffer
  _processNMEA.all = SFE_UBLOX_FILTER_NMEA_ALL; // Default to passing all NMEA messages to processNMEA
  _logRTCM.all = 0;                             // Default to passing no RTCM messages to the file buffer
}

DevUBLOXGNSS::~DevUBLOXGNSS(void)
{
  // Destructor

  end(); // Delete all allocated memory - excluding payloadCfg, payloadAuto and spiBuffer

  if (payloadCfg != nullptr)
  {
    delete[] payloadCfg; // Created with new[]
    payloadCfg = nullptr;
  }

  if (payloadAuto != nullptr)
  {
    delete[] payloadAuto; // Created with new[]
    payloadAuto = nullptr;
  }

  if (spiBuffer != nullptr)
  {
    delete[] spiBuffer; // Created with new[]
    spiBuffer = nullptr;
  }
}

bool DevUBLOXGNSS::init(uint16_t maxWait, bool assumeSuccess)
{
  _signsOfLife = false; // Clear the _signsOfLife flag. It will be set true if valid traffic is seen.

  // Call isConnected up to three times - tests on the NEO-M8U show the CFG RATE poll occasionally being ignored
  bool connected = isConnected(maxWait);

  if (!connected)
  {
    connected = isConnected(maxWait);
  }

  if (!connected)
  {
    connected = isConnected(maxWait);
  }

  if ((!connected) && assumeSuccess && _signsOfLife)
  {
    return (true);
  }

  return (connected);
}


// Returns true if I2C device ack's
bool DevUBLOXGNSS::isConnected(uint16_t maxWait)
{
  // Query port configuration to see whether we get a meaningful response
  uint8_t en;
  if ((_commType == COMM_TYPE_SERIAL) && (!_UART2))
    return getVal8(UBLOX_CFG_UART1INPROT_UBX, &en, VAL_LAYER_RAM, maxWait);
  else if ((_commType == COMM_TYPE_SERIAL) && (_UART2))
    return getVal8(UBLOX_CFG_UART2INPROT_UBX, &en, VAL_LAYER_RAM, maxWait);
  else
    return false;
}


void DevUBLOXGNSS::setCommunicationBus(GNSSDeviceBus &theBus)
{
  _sfeBus = &theBus;
}

uint16_t DevUBLOXGNSS::available()
{
  return _sfeBus->available();
}

bool DevUBLOXGNSS::setUART2Output(uint8_t comSettings, uint8_t layer, uint16_t maxWait)
{
  bool result = newCfgValset(layer);
  result &= addCfgValset8(UBLOX_CFG_UART2OUTPROT_UBX, (comSettings & COM_TYPE_UBX) == 0 ? 0 : 1);
  result &= addCfgValset8(UBLOX_CFG_UART2OUTPROT_NMEA, (comSettings & COM_TYPE_NMEA) == 0 ? 0 : 1);
  result &= sendCfgValset(maxWait);
  result |= setVal8(UBLOX_CFG_UART2OUTPROT_RTCM3X, (comSettings & COM_TYPE_RTCM3) == 0 ? 0 : 1, layer, maxWait); // This will be NACK'd if the module does not support RTCM3
  return result;
}


// Start defining a new (empty) UBX-CFG-VALSET ubxPacket
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
bool DevUBLOXGNSS::newCfgValset(uint8_t layer)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_VALSET;
  packetCfg.len = 4; // 4 byte header
  packetCfg.startingSpot = 0;

  _numCfgKeys = 0;

  // Clear all of packet payload
  memset(payloadCfg, 0, packetCfgPayloadSize);

  payloadCfg[0] = 0;     // Message Version - set to 0
  payloadCfg[1] = layer; // By default we ask for the BBR layer

  // All done
  return (true);
}



// SFE
SfeSerial::SfeSerial(void) : _serialPort{nullptr} {}

bool SfeSerial::init(Stream &serialPort)
{
	// if we don't have a port already
	if (!_serialPort)
	{
		_serialPort = &serialPort;
	}

	// Get rid of any stale serial data already in the processor's RX buffer
	while (_serialPort->available())
		_serialPort->read();

	return true;
}

uint16_t SfeSerial::available()
{
	if (!_serialPort)
		return 0;

	return (_serialPort->available());
}

uint8_t SfeSerial::writeBytes(uint8_t *dataToWrite, uint8_t length)
{
	if (!_serialPort)
		return 0;

	if (length == 0)
		return 0;

	return _serialPort->write(dataToWrite, length);
}

uint8_t SfeSerial::readBytes(uint8_t *data, uint8_t length)
{
	if (!_serialPort)
		return 0;

	if (length == 0)
		return 0;

	return _serialPort->readBytes(data, length);
}