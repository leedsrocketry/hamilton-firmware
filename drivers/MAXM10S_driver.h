/*
	Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 21 Mar 2023
  Last modified on: 21 Mar 2023
  Description: header file for the GNSS module MAX-M10S-00B
*/

#define MAXM10S_DRIVER_H

#include <stdint.h>

// A default of 250ms for maxWait seems fine for I2C but is not enough for SerialUSB.
// If you know you are only going to be using I2C / Qwiic communication, you can
// safely reduce kUBLOXGNSSDefaultMaxWait to 250.
#define kUBLOXGNSSDefaultMaxWait 1100 // Let's allow the user to define their own value if they want to

// These are the Bitfield layers definitions for the UBX-CFG-VALSET message (not to be confused with Bitfield deviceMask in UBX-CFG-CFG)
#ifndef UBX-CFG-VALSET messages  
const uint8_t VAL_LAYER_DEFAULT = 0x7; // ONLY valid with getVal()
const uint8_t VAL_LAYER_RAM = (1 << 0);
const uint8_t VAL_LAYER_BBR = (1 << 1);
const uint8_t VAL_LAYER_FLASH = (1 << 2);
const uint8_t VAL_LAYER_RAM_BBR = VAL_LAYER_RAM | VAL_LAYER_BBR;               // Not valid with getVal()
const uint8_t VAL_LAYER_ALL = VAL_LAYER_RAM | VAL_LAYER_BBR | VAL_LAYER_FLASH; // Not valid with getVal()
#endif

// Configuration ports
#ifndef CFG-PORTS 
// The following consts are used to configure the various ports and streams for those ports. See -CFG-PRT.
const uint8_t COM_PORT_UART1 = 1;
const uint8_t COM_PORT_UART2 = 2;

const uint32_t UBX_CFG_L = 0x01001000;                                // bool
const uint32_t UBLOX_CFG_UART1INPROT_UBX = UBX_CFG_L | 0x10730001;    // Flag to indicate if UBX should be an input protocol on UART1
const uint32_t UBLOX_CFG_UART2INPROT_UBX = UBX_CFG_L | 0x10750001;    // Flag to indicate if UBX should be an input protocol on UART2

#endif

// UBX binary specific variables
#ifndef ubxPacket 
// Global Status Returns
typedef enum
{
  SFE_UBLOX_STATUS_SUCCESS,
  SFE_UBLOX_STATUS_FAIL,
  SFE_UBLOX_STATUS_CRC_FAIL,
  SFE_UBLOX_STATUS_TIMEOUT,
  SFE_UBLOX_STATUS_COMMAND_NACK, // Indicates that the command was unrecognised, invalid or that the module is too busy to respond
  SFE_UBLOX_STATUS_OUT_OF_RANGE,
  SFE_UBLOX_STATUS_INVALID_ARG,
  SFE_UBLOX_STATUS_INVALID_OPERATION,
  SFE_UBLOX_STATUS_MEM_ERR,
  SFE_UBLOX_STATUS_HW_ERR,
  SFE_UBLOX_STATUS_DATA_SENT,     // This indicates that a 'set' was successful
  SFE_UBLOX_STATUS_DATA_RECEIVED, // This indicates that a 'get' (poll) was successful
  SFE_UBLOX_STATUS_I2C_COMM_FAILURE,
  SFE_UBLOX_STATUS_SPI_COMM_FAILURE,
  SFE_UBLOX_STATUS_DATA_OVERWRITTEN // This is an error - the data was valid but has been or _is being_ overwritten by another packet
} sfe_ublox_status_e;

// ubxPacket validity
typedef enum
{
  SFE_UBLOX_PACKET_VALIDITY_NOT_VALID,
  SFE_UBLOX_PACKET_VALIDITY_VALID,
  SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED,
  SFE_UBLOX_PACKET_NOTACKNOWLEDGED // This indicates that we received a NACK
} sfe_ublox_packet_validity_e;

struct ubxPacket
{
  uint8_t cls;
  uint8_t id;
  uint16_t len;          // Length of the payload. Does not include cls, id, or checksum bytes
  uint16_t counter;      // Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
  uint16_t startingSpot; // The counter value needed to go past before we begin recording into payload array
  uint8_t *payload;      // We will allocate RAM for the payload if/when needed.
  uint8_t checksumA;     // Given to us from module. Checked against the rolling calculated A/B checksums.
  uint8_t checksumB;
  sfe_ublox_packet_validity_e valid;           // Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
  sfe_ublox_packet_validity_e classAndIDmatch; // Goes from NOT_DEFINED to VALID or NOT_VALID when the Class and ID match the requestedClass and requestedID
};
#endif


class GNSSDeviceBus
{
  public:
    // For Serial, return Serial.available()
    virtual uint16_t available() = 0;

    // For Serial, do Serial.write
    virtual uint8_t writeBytes(uint8_t *data, uint8_t length) = 0;

    // For SPI, writing bytes will also read bytes simultaneously. Read data is returned in readData
    virtual uint8_t writeReadBytes(const uint8_t *data, uint8_t *readData, uint8_t length) = 0;
    virtual void startWriteReadByte() = 0;                                  // beginTransaction
    virtual void writeReadByte(const uint8_t *data, uint8_t *readData) = 0; // transfer
    virtual void writeReadByte(const uint8_t data, uint8_t *readData) = 0;  // transfer
    virtual void endWriteReadByte() = 0;                                    // endTransaction

    // For Serial, attempt Serial.read
    virtual uint8_t readBytes(uint8_t *data, uint8_t length) = 0;
};


// The sfeSerial device defines behavior for Serial (UART) implementation based around the Stream class.
// This is Arduino specific.
class SfeSerial : public GNSSDeviceBus
{
public:
  SfeSerial(void);

  //TODO Include Stream!!
  bool init(Stream &serialPort);

  bool ping() { return false; }

  uint16_t available();

  uint8_t writeBytes(uint8_t *data, uint8_t length);

  uint8_t writeReadBytes(const uint8_t *data, uint8_t *readData, uint8_t length)
  { (void)data; (void)readData; (void)length; return 0; }

  void startWriteReadByte(){};
  void writeReadByte(const uint8_t *data, uint8_t *readData){ (void)data; (void)readData; }
  void writeReadByte(const uint8_t data, uint8_t *readData){ (void)data; (void)readData; }
  void endWriteReadByte(){};

  uint8_t readBytes(uint8_t *data, uint8_t length);

private:
  //TODO Include Stream!!
  Stream *_serialPort;
};



class DevUBLOXGNSS
{
public:
  DevUBLOXGNSS(void);
  ~DevUBLOXGNSS(void);

  bool isConnected(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);

protected:
  enum commTypes
  {
    COMM_TYPE_I2C = 0,
    COMM_TYPE_SERIAL,
    COMM_TYPE_SPI
  } _commType = COMM_TYPE_SERIAL; // Controls which port we look to for incoming bytes

  bool init(uint16_t maxWait, bool assumeSuccess);

  void setCommunicationBus(GNSSDeviceBus &theBus);

  bool _UART2 = false; // Default to UART1

  // Variables
  GNSSDeviceBus *_sfeBus;

public:
  // Port
  bool setUART2Output(uint8_t comSettings, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Configure UART2 port to output UBX
  void connectedToUART2(bool connected = true) { _UART2 = connected; }
  bool newCfgValset(uint8_t layer = VAL_LAYER_RAM_BBR);                                                         // Create a new, empty UBX-CFG-VALSET. Add entries with addCfgValset8/16/32/64
  bool addCfgValsetN(uint32_t key, uint8_t *value, uint8_t N);                                                  // Add a new key and N-byte value to an existing UBX-CFG-VALSET ubxPacket
  bool sendCfgValset(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); 

  // VALSET
  bool setValN(uint32_t key, uint8_t *value, uint8_t N, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Sets the N-byte value at a given group/id/size location
  bool setVal8(uint32_t key, uint8_t value, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);             // Sets the 8-bit value at a given group/id/size location
  bool setVal16(uint32_t key, uint16_t value, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);           // Sets the 16-bit value at a given group/id/size location
  bool setVal32(uint32_t key, uint32_t value, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);           // Sets the 32-bit value at a given group/id/size location
  bool setVal64(uint32_t key, uint64_t value, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);           // Sets the 64-bit value at a given group/id/size location
  bool setValSigned8(uint32_t key, int8_t value, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);        // Sets the 8-bit value at a given group/id/size location
  bool setValSigned16(uint32_t key, int16_t value, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);      // Sets the 16-bit value at a given group/id/size location
  bool setValSigned32(uint32_t key, int32_t value, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);      // Sets the 32-bit value at a given group/id/size location
  bool setValSigned64(uint32_t key, int64_t value, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);      // Sets the 64-bit value at a given group/id/size location
  bool setValFloat(uint32_t key, float value, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);           // Sets the 32-bit value at a given group/id/size location
  bool setValDouble(uint32_t key, double value, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);         // Sets the 64-bit value at a given group/id/size location


  // Save configuration to BBR / Flash
  bool saveConfiguration(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);  // Save current configuration to flash and BBR (battery backed RAM)

  // Helper functions for PVT
  uint32_t getTimeOfWeek(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint16_t getYear(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint8_t getMonth(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint8_t getDay(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint8_t getHour(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint8_t getMinute(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint8_t getSecond(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint16_t getMillisecond(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  int32_t getNanosecond(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);

  uint8_t getSIV(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);         // Returns number of sats used in fix
  int32_t getLongitude(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);   // Returns the current longitude in degrees * 10-7. Auto selects between HighPrecision and Regular depending on ability of module.
  int32_t getLatitude(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);    // Returns the current latitude in degrees * 10^-7. Auto selects between HighPrecision and Regular depending on ability of module.
  int32_t getAltitude(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);    // Returns the current altitude in mm above ellipsoid
  int32_t getAltitudeMSL(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Returns the current altitude in mm above mean sea level
};


class SFE_UBLOX_GNSS_SERIAL : public DevUBLOXGNSS
{
public:
  SFE_UBLOX_GNSS_SERIAL() { _commType = COMM_TYPE_SERIAL; }

  bool begin(Stream &serialPort, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait, bool assumeSuccess = false)
  {
    // Setup Serial object and pass into the superclass
    setCommunicationBus(_serialBus);

    // Initialize the Serial bus class
    _serialBus.init(serialPort);

    // Initialize the system - return results
    return this->DevUBLOXGNSS::init(maxWait, assumeSuccess);
  }

private:
  // Serial bus class
  SfeSerial _serialBus;
};