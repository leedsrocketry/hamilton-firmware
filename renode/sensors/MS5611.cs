using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.SPI;
using Antmicro.Renode.Peripherals.Sensor;
using Antmicro.Renode.Utilities;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class MS5611 : ISPIPeripheral, IProvidesRegisterCollection<ByteRegisterCollection>, ISensor, IGPIOReceiver
    {
        public MS5611()
        {
            RegistersCollection = new ByteRegisterCollection(this);
            DefineRegisters();
            Reset();
        }

        public void OnGPIO(int number, bool value)
        {
            if(number != 0)
            {
                this.Log(LogLevel.Warning, "This model supports only CS on pin 0, but got signal on pin {0}", number);
                return;
            }

            // value is the negated CS
            if(chipSelected && value)
            {
                FinishTransmission();
            }
            chipSelected = !value;
        }

        public byte Transmit(byte b)
        {
            byte result = 0;

            if(!chipSelected)
            {
                this.Log(LogLevel.Warning, "Received data while chip is not selected");
                return result;
            }

            switch(b)
            {
                case 0x1E: // RESET command
                    Reset();
                    break;
                case 0x40: // Start pressure conversion (OSR = 256)
                    StartConversion(ConversionType.Pressure);
                    break;
                case 0x48: // Start temperature conversion (OSR = 256)
                    StartConversion(ConversionType.Temperature);
                    break;
                case 0xA0: // Read PROM data (calibration coefficients)
                    result = ReadCalibrationData();
                    break;
                case 0x00: // Read ADC result
                    result = ReadAdcResult();
                    break;
                default:
                    this.Log(LogLevel.Warning, "Unsupported command received: 0x{0:X}", b);
                    break;
            }

            this.Log(LogLevel.Noisy, "Transmitting - received 0x{0:X}, sending 0x{1:X} back", b, result);
            return result;
        }

        public void FinishTransmission()
        {
            this.NoisyLog("Finishing transmission, going to the Idle state");
            state = State.Idle;
        }

        public void Reset()
        {
            RegistersCollection.Reset();
            state = State.Idle;
            address = 0;

            // Initialize calibration data (in real hardware these would be read from PROM)
            calibrationData = new ushort[6] { 40127, 36924, 23317, 23282, 33464, 28312 };

            pressure = 0;
            temperature = 0;
        }

        private void StartConversion(ConversionType type)
        {
            state = State.Writing;
            currentConversion = type;
            conversionInProgress = true;

            // In a real sensor, here we'd start an ADC conversion.
            // For simplicity, we'll just simulate the results.
            if(type == ConversionType.Pressure)
            {
                pressure = 9085466; // Simulate a pressure result
            }
            else if(type == ConversionType.Temperature)
            {
                temperature = 2000; // Simulate a temperature result
            }
        }

        private byte ReadCalibrationData()
        {
            if(address < 6)
            {
                return (byte)(calibrationData[address] >> 8);
            }
            else
            {
                return (byte)(calibrationData[address - 6] & 0xFF);
            }
        }

        private byte ReadAdcResult()
        {
            return (byte)69;
            if(conversionInProgress)
            {
                conversionInProgress = false;
                if(currentConversion == ConversionType.Pressure)
                {
                    return (byte)(pressure >> 16);
                }
                else if(currentConversion == ConversionType.Temperature)
                {
                    return (byte)(temperature >> 8);
                }
            }
            return 0;
        }

        public double AccelerationX { get; set; }
        public double AccelerationY { get; set; }
        public double AccelerationZ { get; set; }

        public ByteRegisterCollection RegistersCollection { get; }

        private void DefineRegisters()
        {
            Registers.DeviceID.Define(this)
                .WithValueField(0, 8, FieldMode.Read, name: "DEVID_AD", valueProviderCallback: _ => 0xE5);

            Registers.PartID.Define(this)
                .WithValueField(0, 8, FieldMode.Read, name: "DEVID_PRODUCT", valueProviderCallback: _ => 0xFA);

            Registers.Status.Define(this)
                .WithFlag(0, FieldMode.Read, name: "DATA_RDY", valueProviderCallback: _ => true)
                .WithTag("FIFO_RDY", 1, 1)
                .WithTag("FIFO_FULL", 2, 1)
                .WithTag("FIFO_OVR", 3, 1)
                .WithReservedBits(4, 1)
                .WithTag("USER_NVM_BUSY", 5, 1)
                .WithTag("AWAKE", 6, 1)
                .WithTag("ERR_USER_REGS", 7, 1);
        }

        private byte address;
        private bool chipSelected;
        private State state;

        private ushort[] calibrationData;
        private uint pressure;
        private int temperature;
        private bool conversionInProgress;
        private ConversionType currentConversion;

        private enum State
        {
            Idle,
            // those two states are used in SPI mode
            Reading,
            Writing,
        }

        private enum ConversionType
        {
            Pressure,
            Temperature,
        }

        private enum Registers
        {
            DeviceID = 0x00,
            PartID =   0x02,
            Status =   0x04,
            PROM =     0xA0
        }
    }
}
