//
// Copyright (c) 2010-2020 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//

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
            this.NoisyLog("Reading register {0} (0x{0:X})", (Registers)address);
            switch(b){
                case 0x1E:
                    break;
                case 0x40:
                    break;
                case 0x42:
                    break;
                case 0x44:
                    break;
                case 0x46:
                    break;
                case 0x48:
                    break;
                case 0x50:
                    break;
                case 0x52:
                    break;
                case 0x54:
                    break;
                case 0x56:
                    break;
                case 0x58:
                    break;
                default:
                    result = RegistersCollection.Read(b);
            }
            result = RegistersCollection.Read(address);
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

            AccelerationX = 0;
            AccelerationY = 0;
            AccelerationZ = 0;
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

        private byte Convert(double value, bool upperByte)
        {
            var v = (uint)(value * 160);

            // lower byte contains only 4 bits that are left-shifted
            var result = upperByte
                ? (byte)(v >> 8)
                : (byte)(v >> 4);

            return result;
        }

        private byte address;
        private bool chipSelected;
        private State state;


        private enum State
        {
            Idle,
            // those two states are used in SPI mode
            Reading,
            Writing,
        }

        private enum Registers
        {
            DeviceID = 0x00,
            PartID =   0x02,
            Status =   0x04,

            PROM = 0xA0

        }
    }
}