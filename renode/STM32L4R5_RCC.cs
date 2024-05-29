// Author Morgan Thomas
//

using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Timers;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public sealed class STM32L4_RCC : IDoubleWordPeripheral, IKnownSize, IProvidesRegisterCollection<DoubleWordRegisterCollection>
    {
        public STM32L4_RCC(IMachine machine)
        {
            var registersMap = new Dictionary<long, DoubleWordRegister>
            {
                {(long)Registers.ClockControl, new DoubleWordRegister(this, 0x00000063)
                    .WithFlag(0, out var msion, name: "MSION")
                    .WithFlag(1, FieldMode.Read, valueProviderCallback: _ => msion.Value, name: "MSIRDY")
                    .WithFlag(2, name: "MSIPLLEN")
                    .WithFlag(3, name: "MSIRGSEL")
                    .WithValueField(4, 4, name: "MSIRANGE")
                    .WithFlag(8, out var hsion, name: "HSION")
                    .WithFlag(9, name: "HSIKERON")
                    .WithFlag(10, valueProviderCallback: _ => hsion.Value, name: "HSIRDY")
                    .WithFlag(11, name: "HSIAFS")
                    .WithReservedBits(12, 4)
                    .WithFlag(16, out var hseon, name: "HSEON")
                    .WithFlag(17, FieldMode.Read, valueProviderCallback: _ => hseon.Value, name: "HSERDY")
                    .WithTag("HSEBYP", 18, 1)
                    .WithTag("CSSON", 19, 1)
                    .WithReservedBits(20, 4)
                    .WithFlag(24, out var pllon, name: "PLLON")
                    .WithFlag(25, FieldMode.Read, valueProviderCallback: _ => pllon.Value, name: "PLLRDY")
                    .WithFlag(26, out var pllsai1on, name: "PLLSAI1SON")
                    .WithFlag(27, FieldMode.Read, valueProviderCallback: _ => pllsai1on.Value, name: "PLLSAI1RDY")
                    .WithFlag(28, out var pllsai2on, name: "PLLSAI2ON")
                    .WithFlag(29, FieldMode.Read, valueProviderCallback: _ => pllsai2on.Value, name: "PLLSAI2RDY")
                    .WithReservedBits(30, 2)
                },
                {(long)Registers.InternalClockSourceConfig, new DoubleWordRegister(this, 0x40000000)
                    .WithValueField(0, 8, name: "MSICAL")
                    .WithValueField(8, 8, name: "MSITRIM")
                    .WithValueField(16, 8, name: "HSICAL")
                    .WithValueField(24, 8, name: "HSITRIM")
                },
                {(long)Registers.ClockConfiguration, new DoubleWordRegister(this, 0x00000000)
                    .WithValueField(0, 2, out var systemClockSwitch, name: "SW")
                    .WithValueField(2, 2, FieldMode.Read, name: "SWS", valueProviderCallback: _ => systemClockSwitch.Value)
                    .WithValueField(4, 4, name: "HPRE")
                    .WithValueField(8, 3, name: "PPRE1")
                    .WithValueField(11, 3, name: "PPRE2")
                    .WithReservedBits(14, 1)
                    .WithFlag(15, name: "STOPWUCK")
                    .WithReservedBits(16, 8)
                    .WithValueField(24, 4, name: "MCOSEL")
                    .WithValueField(28, 3, name: "MCOPRE")
                },
                {(long)Registers.PLLConfiguration, new DoubleWordRegister(this, 0x00001000)
                    .WithValueField(0, 2, name: "PLLSRC")
                    .WithReservedBits(2, 2)
                    .WithValueField(4, 4, name: "PLLM")
                    .WithValueField(8, 7, name: "PLLN")
                    .WithReservedBits(15, 1)
                    .WithFlag(16, name: "PLLPEN")
                    .WithFlag(17, name: "PLLP")
                    .WithReservedBits(18, 2)
                    .WithFlag(20, name: "PLLQEN")
                    .WithValueField(21, 2, name: "PLLQ")
                    .WithReservedBits(23, 1)
                    .WithFlag(24, name: "PLLREN")
                    .WithValueField(25, 2, name: "PLLR")
                    .WithValueField(27, 5, name: "PLLPDIV")
                },
                {(long)Registers.PLLSAI1Configuration, new DoubleWordRegister(this, 0x00001000)
                    .WithReservedBits(0, 4)
                    .WithValueField(4, 4, name: "PLLSAI1M")
                    .WithValueField(8, 7, name: "PLLSAI1N")
                    .WithReservedBits(15, 1)
                    .WithFlag(16, name: "PLLSAI1PEN")
                    .WithFlag(17, name: "PLLSAI1P")
                    .WithReservedBits(18, 2)
                    .WithFlag(20, name: "PLLSAI1QEN")
                    .WithValueField(21, 2, name: "PLLSAI1Q")
                    .WithReservedBits(23, 1)
                    .WithFlag(24, name: "PLLSAI1REN")
                    .WithValueField(25, 2, name: "PLLSAI1R")
                    .WithValueField(27, 5, name: "PLLSAI1PDIV")
                },
                {(long)Registers.PLLSAI2Configuration, new DoubleWordRegister(this, 0x00001000)
                    .WithReservedBits(0, 4)
                    .WithValueField(4, 4, name: "PLLSAI2M")
                    .WithValueField(8, 7, name: "PLLSAI2N")
                    .WithReservedBits(15, 1)
                    .WithFlag(16, name: "PLLSAI2PEN")
                    .WithFlag(17, name: "PLLSAI2P")
                    .WithReservedBits(18, 2)
                    .WithFlag(20, name: "PLLSAI2QEN")
                    .WithValueField(21, 2, name: "PLLSAI2Q")
                    .WithReservedBits(23, 1)
                    .WithFlag(24, name: "PLLSAI2REN")
                    .WithValueField(25, 2, name: "PLLSAI2R")
                    .WithValueField(27, 5, name: "PLLSAI2PDIV")
                },
                {(long)Registers.ClockInterruptEnable, new DoubleWordRegister(this, 0x00000000)
                    .WithFlag(0, name: "LSIRDYIE")
                    .WithFlag(1, name: "LSERDYIE")
                    .WithFlag(2, name: "MSIRDYIE")
                    .WithFlag(3, name: "HSIRDYIE")
                    .WithFlag(4, name: "HSERDYIE")
                    .WithFlag(5, name: "PLLRDYIE")
                    .WithFlag(6, name: "PLLSAI1RDYIE")
                    .WithFlag(7, name: "PLLSAI2RDYIE")
                    .WithReservedBits(8, 1)
                    .WithFlag(9, name: "LSECSSIE")
                    .WithFlag(10, name: "HSI48RDYIE")
                    .WithReservedBits(11, 21)
                },
                {(long)Registers.ClockInterruptFlag, new DoubleWordRegister(this, 0x00000000)
                    .WithFlag(0, name: "LSIRDYF")
                    .WithFlag(1, name: "LSERDYF")
                    .WithFlag(2, name: "MSIRDYF")
                    .WithFlag(3, name: "HSIRDYF")
                    .WithFlag(4, name: "HSERDYF")
                    .WithFlag(5, name: "PLLRDYF")
                    .WithFlag(6, name: "PLLSAI1RDYF")
                    .WithFlag(7, name: "PLLSAI2RDYF")
                    .WithFlag(8, name: "CSSF")
                    .WithFlag(9, name: "LSECSSF")
                    .WithFlag(10, name: "HSI48RDYF")
                    .WithReservedBits(11, 21)
                },
                {(long)Registers.ClockInterruptClear, new DoubleWordRegister(this, 0x00000000)
                    .WithFlag(0, out var lsirdyc, name: "LSIRDYC")
                    .WithFlag(1, out var lserdyc, name: "LSERDYC")
                    .WithFlag(2, out var msirdyc, name: "MSIRDYC")
                    .WithFlag(3, out var hsirdyc, name: "HSIRDYC")
                    .WithFlag(4, out var hserdyc, name: "HSERDYC")
                    .WithFlag(5, out var pllrdyc, name: "PLLRDYC")
                    .WithFlag(6, out var pllsai1rdyc, name: "PLLSAI1RDYC")
                    .WithFlag(7, out var pllsai2rdyc, name: "PLLSAI2RDYC")
                    .WithFlag(8, out var cssc, name: "CSSC")
                    .WithFlag(9, out var lsecssc, name: "LSECSSC")
                    .WithFlag(10, out var hsi48rdyc, name: "HSI48RDYC")
                    .WithReservedBits(11, 21)
                },
                {(long)Registers.AHB1PeripheralReset, new DoubleWordRegister(this, 0x00000000)
                    .WithFlag(0, name: "DMA1RST")
                    .WithFlag(1, name: "DMA2RST")
                    .WithFlag(2, name: "DMAMUX1RST")
                    .WithReservedBits(3, 5)
                    .WithFlag(8, name: "FLASHRST")
                    .WithReservedBits(9, 3)
                    .WithFlag(12, name: "CRCRST")
                    .WithReservedBits(13, 3)
                    .WithFlag(16, name: "TSCRST")
                    .WithFlag(17, name: "DMA2DRST")
                    .WithFlag(18, name: "GFXMMURSTM")
                    .WithReservedBits(19, 13)
                },
                {(long)Registers.AHB2PeripheralReset, new DoubleWordRegister(this, 0x00000000)
                    .WithFlag(0, name: "GPIOARST")
                    .WithFlag(1, name: "GPIOBRST")
                    .WithFlag(2, name: "GPIOCRST")
                    .WithFlag(3, name: "GPIODRST")
                    .WithFlag(4, name: "GPIOERST")
                    .WithFlag(5, name: "GPIOFRST")
                    .WithFlag(6, name: "GPIOGRST")
                    .WithFlag(7, name: "GPIOHRST")
                    .WithFlag(8, name: "GPIOIRST")
                    .WithReservedBits(9, 3)
                    .WithFlag(12, name: "OTGFSRST")
                    .WithFlag(13, name: "ADCRST")
                    .WithFlag(14, name: "DCMIRST")
                    .WithFlag(15, name: "PKARST")
                    .WithFlag(16, name: "AESRST")
                    .WithFlag(17, name: "HASHRST")
                    .WithFlag(18, name: "RNGRST")
                    .WithReservedBits(19, 1)
                    .WithFlag(20, name: "OSPIMRST")
                    .WithReservedBits(21, 1)
                    .WithFlag(22, name: "SDMMC1RST")
                    .WithFlag(23, name: "SDMMC2RST")
                    .WithReservedBits(24,8)
                },
                {(long)Registers.AHB3PeripheralReset, new DoubleWordRegister(this, 0x00000000)
                    .WithFlag(0, name: "FMCRST")
                    .WithReservedBits(1, 7)
                    .WithFlag(8, name: "OSPI1RST")
                    .WithFlag(9, name: "OPSI2RST")
                    .WithReservedBits(10, 22)
                },
                {(long)Registers.APB1PeripheralReset, new DoubleWordRegister(this, 0x00000000)
                    .WithFlag(0, name: "TIM2RST")
                    .WithFlag(1, name: "TIM3RST")
                    .WithFlag(2, name: "TIM4RST")
                    .WithFlag(3, name: "TIM5RST")
                    .WithFlag(4, name: "TIM6RST")
                    .WithFlag(5, name: "TIM7RST")
                    .WithReservedBits(6, 8)
                    .WithFlag(14, name: "SPI2RST")
                    .WithFlag(15, name: "SPI3RST")
                    .WithReservedBits(16, 1)
                    .WithFlag(17, name: "USART2RST")
                    .WithFlag(18, name: "USART3RST")
                    .WithFlag(19, name: "UART4RST")
                    .WithFlag(20, name: "UART5RST")
                    .WithFlag(21, name: "I2C1RST")
                    .WithFlag(22, name: "I2C2RST")
                    .WithFlag(23, name: "I2C3RST")
                    .WithFlag(24, name: "CRSRST")
                    .WithFlag(25, name: "CAN1RST")
                    .WithReservedBits(26, 2)
                    .WithFlag(28, name: "PWRRST")
                    .WithFlag(29, name: "DAC1RST")
                    .WithFlag(30, name: "OPAMPRST")
                    .WithFlag(31, name: "LPTIM1RST")
                },
                {(long)Registers.APB1PeripheralReset2, new DoubleWordRegister(this, 0x00000000)
                    .WithFlag(0, name: "LPUART1RST")
                    .WithFlag(1, name: "I2C4RST")
                    .WithReservedBits(2, 3)
                    .WithFlag(5, name: "LPTIM2RST")
                    .WithReservedBits(6, 26)

                },
                {(long)Registers.APB2PeripheralReset, new DoubleWordRegister(this, 0x00000000)
                    .WithFlag(0, name: "SYSCFGRST")
                    .WithReservedBits(1, 10)
                    .WithFlag(11, name: "TIM1RST")
                    .WithFlag(12, name: "SPI1RST")
                    .WithFlag(13, name: "TIM8RST")
                    .WithFlag(14, name: "USART1RST")
                    .WithReservedBits(15, 1)
                    .WithFlag(16, name: "TIM15RST")
                    .WithFlag(17, name: "TIM16RST")
                    .WithFlag(18, name: "TIM17RST")
                    .WithReservedBits(19, 2)
                    .WithFlag(21, name: "SAI1RST")
                    .WithFlag(22, name: "SAI2RST")
                    .WithReservedBits(23, 1)
                    .WithFlag(24, name: "DFSDM1RST")
                    .WithReservedBits(25, 1)
                    .WithFlag(26, name: "LTDCRST")
                    .WithFlag(27, name: "DSIRST")
                    .WithReservedBits(28, 4)
                },
                {(long)Registers.AHB1PeripheralClockEnable, new DoubleWordRegister(this, 0x00000000)
                    .WithFlag(0, name: "DMA1EN")
                    .WithFlag(1, name: "DMA2EN")
                    .WithFlag(2, name: "DMAMUX1EN")
                    .WithReservedBits(3, 5)
                    .WithFlag(8, name: "FLASHEN")
                    .WithReservedBits(9, 3)
                    .WithFlag(12, name: "CRCEN")
                    .WithReservedBits(13, 3)
                    .WithFlag(16, name: "TSCEN")
                    .WithFlag(17, name: "DMA2DEN")
                    .WithFlag(18, name: "GFXMMUEN")
                    .WithReservedBits(19, 13)
                },
                {(long)Registers.AHB2PeripheralClockEnable, new DoubleWordRegister(this, 0x00000000)
                    .WithFlag(0, name: "GPIOAEN")
                    .WithFlag(1, name: "GPIOBEN")
                    .WithFlag(2, name: "GPIOCEN")
                    .WithFlag(3, name: "GPIODEN")
                    .WithFlag(4, name: "GPIOEEN")
                    .WithFlag(5, name: "GPIOFEN")
                    .WithFlag(6, name: "GPIOGEN")
                    .WithFlag(7, name: "GPIOHEN")
                    .WithFlag(8, name: "GPIOIEN")
                    .WithReservedBits(9, 3)
                    .WithFlag(12, name: "OTGFSEN")
                    .WithFlag(13, name: "ADCEN")
                    .WithFlag(14, name: "DCMIEN")
                    .WithFlag(15, name: "PKAEN")
                    .WithFlag(16, name: "AESEN")
                    .WithFlag(17, name: "HASHEN")
                    .WithFlag(18, name: "RNGEN")
                    .WithReservedBits(19, 1)
                    .WithFlag(20, name: "OSPIMEN")
                    .WithReservedBits(21, 1)
                    .WithFlag(22, name: "SDMMC1EN")
                    .WithFlag(23, name: "SDMMC2EN")
                    .WithReservedBits(24,8 )
                },
                {(long)Registers.AHB3PeripheralClockEnable, new DoubleWordRegister(this, 0x00000000)
                    .WithFlag(0, name: "FMCEN")
                    .WithReservedBits(1, 7)
                    .WithFlag(8, name: "OSPI1EN")
                    .WithFlag(9, name: "OPSI2EN")
                    .WithReservedBits(10, 22)
                },
                {(long)Registers.APB1PeripheralClockEnable, new DoubleWordRegister(this, 0x00000000)
                    .WithFlag(0, name: "TIM2EN")
                    .WithFlag(1, name: "TIM3EN")
                    .WithFlag(2, name: "TIM4EN")
                    .WithFlag(3, name: "TIM5EN")
                    .WithFlag(4, name: "TIM6EN")
                    .WithFlag(5, name: "TIM7EN")
                    .WithReservedBits(6, 8)
                    .WithFlag(14, name: "SPI2EN")
                    .WithFlag(15, name: "SPI3EN")
                    .WithReservedBits(16, 1)
                    .WithFlag(17, name: "USART2EN")
                    .WithFlag(18, name: "USART3EN")
                    .WithFlag(19, name: "UART4EN")
                    .WithFlag(20, name: "UART5EN")
                    .WithFlag(21, name: "I2C1EN")
                    .WithFlag(22, name: "I2C2EN")
                    .WithFlag(23, name: "I2C3EN")
                    .WithFlag(24, name: "CRSEN")
                    .WithFlag(25, name: "CAN1EN")
                    .WithReservedBits(26, 2)
                    .WithFlag(28, name: "PWREN")
                    .WithFlag(29, name: "DAC1EN")
                    .WithFlag(30, name: "OPAMPEN")
                    .WithFlag(31, name: "LPTIM1EN")
                },
                {(long)Registers.APB1PeripheralClockEnable2, new DoubleWordRegister(this, 0x00000000)
                    .WithFlag(0, name: "LPUART1EN")
                    .WithFlag(1, name: "I2C4EN")
                    .WithReservedBits(2, 3)
                    .WithFlag(5, name: "LPTIM2EN")
                    .WithReservedBits(6, 26)

                },
                {(long)Registers.APB2PeripheralClockEnable, new DoubleWordRegister(this, 0x00000000)
                    .WithFlag(0, name: "SYSCFGEN")
                    .WithReservedBits(1, 10)
                    .WithFlag(11, name: "TIM1EN")
                    .WithFlag(12, name: "SPI1EN")
                    .WithFlag(13, name: "TIM8EN")
                    .WithFlag(14, name: "USART1EN")
                    .WithReservedBits(15, 1)
                    .WithFlag(16, name: "TIM15EN")
                    .WithFlag(17, name: "TIM16EN")
                    .WithFlag(18, name: "TIM17EN")
                    .WithReservedBits(19, 2)
                    .WithFlag(21, name: "SAI1EN")
                    .WithFlag(22, name: "SAI2EN")
                    .WithReservedBits(23, 1)
                    .WithFlag(24, name: "DFSDM1EN")
                    .WithReservedBits(25, 1)
                    .WithFlag(26, name: "LTDCEN")
                    .WithFlag(27, name: "DSIEN")
                    .WithReservedBits(28, 4)
                },

                {(long)Registers.AHB1PeripheralClockEnableInLowPowerMode, new DoubleWordRegister(this, 0x00000000)
                    .WithFlag(0, name: "DMA1SMEN")
                    .WithFlag(1, name: "DMA2SMEN")
                    .WithFlag(2, name: "DMAMUX1SMEN")
                    .WithReservedBits(3, 5)
                    .WithFlag(8, name: "FLASHSMEN")
                    .WithReservedBits(9, 3)
                    .WithFlag(12, name: "CRCMEN")
                    .WithReservedBits(13, 3)
                    .WithFlag(16, name: "TSCSMEN")
                    .WithFlag(17, name: "DMA2DSMEN")
                    .WithFlag(18, name: "GFXMMUSMEN")
                    .WithReservedBits(19, 13)
                },
                {(long)Registers.AHB2PeripheralClockEnableInLowPowerMode, new DoubleWordRegister(this, 0x00000000)
                    .WithFlag(0, name: "GPIOASMEN")
                    .WithFlag(1, name: "GPIOBSMEN")
                    .WithFlag(2, name: "GPIOCSMEN")
                    .WithFlag(3, name: "GPIODSMEN")
                    .WithFlag(4, name: "GPIOESMEN")
                    .WithFlag(5, name: "GPIOFSMEN")
                    .WithFlag(6, name: "GPIOGSMEN")
                    .WithFlag(7, name: "GPIOHSMEN")
                    .WithFlag(8, name: "GPIOISMEN")
                    .WithReservedBits(9, 3)
                    .WithFlag(12, name: "OTGFSSMEN")
                    .WithFlag(13, name: "ADCSMEN")
                    .WithFlag(14, name: "DCMISMEN")
                    .WithFlag(15, name: "PKASMEN")
                    .WithFlag(16, name: "AESSMEN")
                    .WithFlag(17, name: "HASHSMEN")
                    .WithFlag(18, name: "RNGSMEN")
                    .WithReservedBits(19, 1)
                    .WithFlag(20, name: "OSPIMSMEN")
                    .WithReservedBits(21, 1)
                    .WithFlag(22, name: "SDMMC1SMEN")
                    .WithFlag(23, name: "SDMMC2SMEN")
                    .WithReservedBits(24,8 )
                },
                {(long)Registers.AHB3PeripheralClockEnableInLowPowerMode, new DoubleWordRegister(this, 0x00000000)
                    .WithFlag(0, name: "FMCSMEN")
                    .WithReservedBits(1, 7)
                    .WithFlag(8, name: "OSPI1SMEN")
                    .WithFlag(9, name: "OPSI2SMEN")
                    .WithReservedBits(10, 22)
                },
                {(long)Registers.APB1PeripheralClockEnableInLowPowerMode, new DoubleWordRegister(this, 0x00000000)
                    .WithFlag(0, name: "TIM2SMEN")
                    .WithFlag(1, name: "TIM3SMEN")
                    .WithFlag(2, name: "TIM4SMEN")
                    .WithFlag(3, name: "TIM5SMEN")
                    .WithFlag(4, name: "TIM6SMEN")
                    .WithFlag(5, name: "TIM7SMEN")
                    .WithReservedBits(6, 8)
                    .WithFlag(14, name: "SPI2SMEN")
                    .WithFlag(15, name: "SPI3SMEN")
                    .WithReservedBits(16, 1)
                    .WithFlag(17, name: "USART2SMEN")
                    .WithFlag(18, name: "USART3SMEN")
                    .WithFlag(19, name: "UART4SMEN")
                    .WithFlag(20, name: "UART5SMEN")
                    .WithFlag(21, name: "I2C1SMEN")
                    .WithFlag(22, name: "I2C2SMEN")
                    .WithFlag(23, name: "I2C3SMEN")
                    .WithFlag(24, name: "CRSSMEN")
                    .WithFlag(25, name: "CAN1SMEN")
                    .WithReservedBits(26, 2)
                    .WithFlag(28, name: "PWRSMEN")
                    .WithFlag(29, name: "DAC1SMEN")
                    .WithFlag(30, name: "OPAMPSMEN")
                    .WithFlag(31, name: "LPTIM1SMEN")
                },
                {(long)Registers.APB1PeripheralClockEnableInLowPowerMode2, new DoubleWordRegister(this, 0x00000000)
                    .WithFlag(0, name: "LPUART1SMEN")
                    .WithFlag(1, name: "I2C4SMEN")
                    .WithReservedBits(2, 3)
                    .WithFlag(5, name: "LPTIM2SMEN")
                    .WithReservedBits(6, 26)

                },
                {(long)Registers.APB2PeripheralClockEnableInLowPowerMode, new DoubleWordRegister(this, 0x00000000)
                    .WithFlag(0, name: "SYSCFGSMEN")
                    .WithReservedBits(1, 10)
                    .WithFlag(11, name: "TIM1SMEN")
                    .WithFlag(12, name: "SPI1SMEN")
                    .WithFlag(13, name: "TIM8SMEN")
                    .WithFlag(14, name: "USART1SMEN")
                    .WithReservedBits(15, 1)
                    .WithFlag(16, name: "TIM15SMEN")
                    .WithFlag(17, name: "TIM16SMEN")
                    .WithFlag(18, name: "TIM17SMEN")
                    .WithReservedBits(19, 2)
                    .WithFlag(21, name: "SAI1SMEN")
                    .WithFlag(22, name: "SAI2SMEN")
                    .WithReservedBits(23, 1)
                    .WithFlag(24, name: "DFSDM1SMEN")
                    .WithReservedBits(25, 1)
                    .WithFlag(26, name: "LTDCSMEN")
                    .WithFlag(27, name: "DSISMEN")
                    .WithReservedBits(28, 4)
                },
                {(long)Registers.PeripheralIndependentClockConfig, new DoubleWordRegister(this, 0x00000000)
                },
                {(long)Registers.BackupDomainControl, new DoubleWordRegister(this, 0x00000000)
                },
                {(long)Registers.ClockControlAndStatus, new DoubleWordRegister(this, 0x00000000)
                },
                {(long)Registers.ClockRecovery, new DoubleWordRegister(this, 0x00000000)
                },
                {(long)Registers.PeripheralIndependentClockConfig2, new DoubleWordRegister(this, 0x00000000)
                },
                {(long)Registers.OCTOSPIDelayConfig, new DoubleWordRegister(this, 0x00000000)
                },
            };

            RegistersCollection = new DoubleWordRegisterCollection(this, registersMap);
        }

        public uint ReadDoubleWord(long offset)
        {
            return RegistersCollection.Read(offset);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            RegistersCollection.Write(offset, value);
        }

        public void Reset()
        {
            RegistersCollection.Reset();
        }

        public long Size => 0x400;

        public DoubleWordRegisterCollection RegistersCollection { get; }

        private enum Registers
        {
            ClockControl = 0x0,
            InternalClockSourceConfig = 0x4,
            ClockConfiguration = 0x8,
            PLLConfiguration = 0xC,
            PLLSAI1Configuration = 0x10,
            PLLSAI2Configuration = 0x14,
            ClockInterruptEnable = 0x18,
            ClockInterruptFlag = 0x1C,
            ClockInterruptClear = 0x20,
            //gap
            AHB1PeripheralReset = 0x28,
            AHB2PeripheralReset = 0x2C,
            AHB3PeripheralReset = 0x30,
            //gap
            APB1PeripheralReset = 0x38,
            APB1PeripheralReset2 = 0x3C,
            APB2PeripheralReset = 0x40,
            //gap
            AHB1PeripheralClockEnable = 0x48,
            AHB2PeripheralClockEnable = 0x4C,
            AHB3PeripheralClockEnable = 0x50,
            //gap
            APB1PeripheralClockEnable = 0x58,
            APB1PeripheralClockEnable2 = 0x5C,
            APB2PeripheralClockEnable = 0x60,
            //gap
            AHB1PeripheralClockEnableInLowPowerMode = 0x68,
            AHB2PeripheralClockEnableInLowPowerMode = 0x6C,
            AHB3PeripheralClockEnableInLowPowerMode = 0x70,
            //gap
            APB1PeripheralClockEnableInLowPowerMode = 0x78,
            APB1PeripheralClockEnableInLowPowerMode2 = 0x7C,
            APB2PeripheralClockEnableInLowPowerMode = 0x80,
            //gap
            PeripheralIndependentClockConfig = 0x88,
            //gap
            BackupDomainControl = 0x90,
            ClockControlAndStatus = 0x94,
            ClockRecovery = 0x98,
            PeripheralIndependentClockConfig2 = 0x9C,
            //gap
            OCTOSPIDelayConfig = 0xA4,
        }
    }
}