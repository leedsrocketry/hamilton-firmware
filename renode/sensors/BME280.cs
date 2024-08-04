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
    public class BME280 : ISPIPeripheral, IProvidesRegisterCollection<ByteRegisterCollection>, ISensor
    {
        public BME280()
        {
            RegistersCollection = new ByteRegisterCollection(this);
            DefineRegisters();
        }


        public byte Transmit(byte b)
        {

            byte result = 0;
            switch(state)
            {
                case State.Idle:
                    result = HandleIdle(b);
                    break;

                case State.Reading:
                    this.NoisyLog("Reading register {0} (0x{0:X})", (Registers)address);
                    result = RegistersCollection.Read(address);
                    address++;
                    break;

                case State.Writing:
                    this.NoisyLog("Writing 0x{0:X} to register {1} (0x{1:X})", b, (Registers)address);
                    RegistersCollection.Write(address, b);
                    address++;
                    break;

                default:
                    this.Log(LogLevel.Error, "Received byte in an unexpected state!");
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

        }


        public ByteRegisterCollection RegistersCollection { get; }

        private void DefineRegisters()
        {
            Registers.DeviceID.Define(this)
                .WithValueField(0, 8, FieldMode.Read, name: "DEV_ID", valueProviderCallback: _ => 0x60);

            Registers.Reset.Define(this)
                .WithValueField(0, 8, FieldMode.Write, name: "RESET");

            Registers.CtrlHum.Define(this)
                .WithValueField(0, 2, FieldMode.Write, name: "HUM_CONTROL");

            Registers.Status.Define(this)
                .WithFlag(0, name: "IM_UPDATE")
                .WithFlag(3, name: "MEASURING");

            Registers.CtrlMeas.Define(this)
                .WithValueField(0, 1, FieldMode.Write, name: "MODE")
                .WithValueField(2, 4, FieldMode.Write, name: "OVERSAMP_PRESS")
                .WithValueField(5, 7, FieldMode.Write, name: "OVERSAMP_TEMP");

            Registers.Config.Define(this)
                .WithFlag(0, name: "E3SPI")
                .WithValueField(2, 4, FieldMode.Write, name: "FILTER")
                .WithValueField(5, 7, FieldMode.Write, name: "TSTNDBY");

            Registers.PressMSB.Define(this)
                .WithValueField(0, 7, FieldMode.Read, name: "PRESS_MSB");

            Registers.PressLSB.Define(this)
                .WithValueField(0, 7, FieldMode.Read, name: "PRESS_LSB");

            Registers.PressXLSB.Define(this)
                .WithReservedBits(0, 4)
                .WithValueField(4, 7, FieldMode.Read, name: "PRESS_XLSB");

            Registers.TempMSB.Define(this)
                .WithValueField(0, 7, FieldMode.Read, name: "TEMP_MSB");

            Registers.TempLSB.Define(this)
                .WithValueField(0, 7, FieldMode.Read, name: "PRESS_LSB");

            Registers.TempXLSB.Define(this)
                .WithReservedBits(0, 4)
                .WithValueField(4, 7, FieldMode.Read, name: "PRESS_XLSB");

            Registers.HumMSB.Define(this)
                .WithValueField(0, 7, FieldMode.Read, name: "HUM_MSB");

            Registers.HumLSB.Define(this)
                .WithValueField(0, 7, FieldMode.Read, name: "HUM_LSB");
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

        private byte HandleIdle(byte b)
        {
            address = (byte)(b >> 1);

            if(BitHelper.IsBitSet(b, 0)) // 0 for write, 1 for read
            {
                state = State.Reading;
            }
            else
            {
                state = State.Writing;
            }

            return 0;
        }

        private byte address;

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
            Calib00   =  0x88, // -> 0xA1
            DeviceID  =  0xD0,
            Reset     =  0xE0,
            Calib26   =  0xE1, // -> 0xF0
            CtrlHum   =  0xF2,
            Status    =  0xF3,
            CtrlMeas  =  0xF4,
            Config    =  0xF5,
            PressMSB  =  0xF7,
            PressLSB  =  0xF8,
            PressXLSB =  0xF9,
            TempMSB   =  0xFA,
            TempLSB   =  0xFB,
            TempXLSB  =  0xFC,
            HumMSB    =  0xFD,
            HumLSB    =  0xFE,
        }
    }
}