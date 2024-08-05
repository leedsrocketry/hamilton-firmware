//
// Copyright (c) 2010-2024 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//

using System.Collections.Generic;
using System.Linq;
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Utilities;

namespace Antmicro.Renode.Peripherals.SPI
{
    public class CustomSPIMultiplexer : SimpleContainer<ISPIPeripheral>, IGPIOReceiver, ISPIPeripheral
    {
        public CustomSPIMultiplexer(IMachine machine, bool suppressExplicitFinishTransmission = true) : base(machine)
        {
            this.suppressExplicitFinishTransmission = suppressExplicitFinishTransmission;
            inputState = new Dictionary<int, bool>();
        }

        public void OnGPIO(int number, bool value)
        {
            this.Log(LogLevel.Noisy, "GPIO #{0} set to {1}", number, value);
            inputState[number] = value;
            UpdateChipSelectState();
        }

        public override void Reset()
        {
            inputState.Clear();
        }

        public byte Transmit(byte data)
        {
            var deviceAddress = this.chipSelectValue;
            if(!TryGetByAddress(deviceAddress, out var device))
            {
                this.Log(LogLevel.Warning, "Tried to transmit byte 0x{0:X} to device 0x{1:X}, but it's not connected - ignoring transfer and returning a dummy byte", data, deviceAddress);
                return 0xFF;
            }
            return device.Transmit(data);
        }

        public void FinishTransmission()
        {
            if(suppressExplicitFinishTransmission)
            {
                return;
            }

            var deviceAddress = this.chipSelectValue;
            FinishTransmissionByAddress(deviceAddress);
        }

        public int readChipSelect()
        {
            return this.chipSelectValue;
        }

        private void FinishTransmissionByAddress(int deviceAddress)
        {
            if(!TryGetByAddress(deviceAddress, out var device))
            {
                this.Log(LogLevel.Warning, "Tried to finish transmission to device 0x{0:X}, but it's not connected", deviceAddress);
                return;
            }

            this.Log(LogLevel.Noisy, "Finishing transmission on device 0x{0:X}", deviceAddress);
            device.FinishTransmission();
        }

        private void UpdateChipSelectState()
        {
            int newChipSelectValue = 0;
            foreach(var state in inputState)
            {
                var number = state.Key;
                var value = state.Value;
                if (value) {
                    newChipSelectValue += (int) Math.Pow(2, number);
                } else {
                    continue;
                }
            }
            this.Log(LogLevel.Noisy, "Updated chip select to 0x{0:X}", newChipSelectValue);
            if(TryGetByAddress(this.chipSelectValue, out var device)) {
                device.FinishTransmission();
            };

            this.chipSelectValue = newChipSelectValue;
        }

        private int chipSelectValue;
        private readonly Dictionary<int, bool> inputState;
        private readonly bool suppressExplicitFinishTransmission;
    }
}

