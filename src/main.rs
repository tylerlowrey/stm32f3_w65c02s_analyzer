#![no_std]
#![no_main]

mod interrupts;

use core::sync::atomic::{Ordering};
use panic_halt as _;
use cortex_m_rt::entry;
use stm32f3_discovery::leds::Leds;
use stm32f3_discovery::stm32f3xx_hal::flash::FlashExt;
use stm32f3_discovery::stm32f3xx_hal::{
    gpio,
    gpio::GpioExt,
    gpio::PXx,
    gpio::Input,
    gpio::gpiob,
    gpio::gpioc,
    gpio::gpiod,
    gpio::AF7,
    gpio::PushPull
};
use stm32f3_discovery::stm32f3xx_hal::rcc::{APB2, Clocks, RccExt};
use stm32f3_discovery::stm32f3xx_hal::{pac, pac::USART1, pac::usart1};
use stm32f3_discovery::stm32f3xx_hal::hal::digital::v2::InputPin;
use stm32f3_discovery::stm32f3xx_hal::time::rate::Extensions;
use stm32f3_discovery::stm32f3xx_hal::serial::Serial;
use stm32f3_discovery::switch_hal::OutputSwitch;
use crate::interrupts::exti1::PA1_INTERRUPT_RECEIVED;
use crate::interrupts::{enable_exti_interrupt, ExtiInterruptInput, ExtiLine};

const ASCII_ZERO_OFFSET: u16 = 48;

#[entry]
fn main() -> ! {
    let peripherals = pac::Peripherals::take().unwrap();
    let mut rcc = peripherals.RCC.constrain();
    let mut flash = peripherals.FLASH.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut gpioa = peripherals.GPIOA.split(&mut rcc.ahb);
    let gpiob = peripherals.GPIOB.split(&mut rcc.ahb);
    let gpioc = peripherals.GPIOC.split(&mut rcc.ahb);
    let gpiod = peripherals.GPIOD.split(&mut rcc.ahb);

    let (data_pins, address_pins, rwb_pin, tx, rx) = setup_gpio_pins(gpiob, gpioc, gpiod);
    gpioa.pa1.into_floating_input(&mut gpioa.moder, &mut gpioa.pupdr);
    let usart1 = setup_serial_connection(peripherals.USART1, &mut rcc.apb2, clocks, tx, rx);

    // Setup GPIO PA1 as interrupt (6502 clock)
    enable_exti_interrupt(&peripherals.EXTI, &peripherals.SYSCFG, ExtiLine::Exti1, ExtiInterruptInput::PA);

    loop {
        // Send recorded data to pc via usart
        if PA1_INTERRUPT_RECEIVED.swap(false, Ordering::AcqRel) {
            // RWB value
            while usart1.isr.read().txe().bit_is_clear() {}
            let rwb_value = rwb_pin.is_high().unwrap();
            usart1.tdr.write(|w| w.tdr().bits((rwb_value as u16) + ASCII_ZERO_OFFSET));

            // Print out a space
            while usart1.isr.read().txe().bit_is_clear() {}
            usart1.tdr.write(|w| w.tdr().bits(u16::from(b' ')));

            // Data pins values
            for i in 0..data_pins.len() {
                while usart1.isr.read().txe().bit_is_clear() {}

                let pin_value = data_pins[i].as_ref().unwrap().is_high().unwrap();
                usart1.tdr.write(|w| w.tdr().bits((pin_value as u16) + ASCII_ZERO_OFFSET));
            }

            // Print out a space
            while usart1.isr.read().txe().bit_is_clear() {}
            usart1.tdr.write(|w| w.tdr().bits(u16::from(b' ')));

            // Address pins values
            for i in 0..address_pins.len() {
                while usart1.isr.read().txe().bit_is_clear() {}

                let pin_value = address_pins[i].as_ref().unwrap().is_high().unwrap();
                usart1.tdr.write(|w| w.tdr().bits((pin_value as u16) + ASCII_ZERO_OFFSET));
            }

            // Newline print
            while usart1.isr.read().txe().bit_is_clear() {}
            usart1.tdr.write(|w| w.tdr().bits(u16::from(b'\r')));
            while usart1.isr.read().txe().bit_is_clear() {}
            usart1.tdr.write(|w| w.tdr().bits(u16::from(b'\n')));
        }

        cortex_m::asm::wfi();
    }
}

fn setup_serial_connection(usart1: USART1, apb2: &mut APB2, clocks: Clocks, tx: gpioc::PC4<AF7<PushPull>>, rx: gpioc::PC5<AF7<PushPull>>)
    -> &'static mut usart1::RegisterBlock {
    Serial::new(usart1, (tx, rx), 115_200.Bd(), clocks, apb2);

    unsafe { &mut *(USART1::ptr() as *mut usart1::RegisterBlock) }
}

type DataPins = [Option<PXx<Input>>; 8];
type AddressPins = [Option<PXx<Input>>; 16];

fn setup_gpio_pins(mut gpiob: gpiob::Parts, mut gpioc: gpioc::Parts, mut gpiod: gpiod::Parts)
    -> (DataPins, AddressPins, gpiob::PB11<Input>, gpioc::PC4<AF7<PushPull>>, gpioc::PC5<AF7<PushPull>>) {

    let mut data_pins: DataPins = Default::default();
    let mut address_pins: AddressPins = Default::default();

    // Setup GPIO PB 12-15, PD 8-11 as inputs (for data pins)
    let pb12 = gpiob.pb12.into_floating_input(&mut gpiob.moder, &mut gpiob.pupdr);
    let pb13 = gpiob.pb13.into_floating_input(&mut gpiob.moder, &mut gpiob.pupdr);
    let pb14 = gpiob.pb14.into_floating_input(&mut gpiob.moder, &mut gpiob.pupdr);
    let pb15 = gpiob.pb15.into_floating_input(&mut gpiob.moder, &mut gpiob.pupdr);
    let pd8 = gpiod.pd8.into_floating_input(&mut gpiod.moder, &mut gpiod.pupdr);
    let pd9 = gpiod.pd9.into_floating_input(&mut gpiod.moder, &mut gpiod.pupdr);
    let pd10 = gpiod.pd10.into_floating_input(&mut gpiod.moder, &mut gpiod.pupdr);
    let pd11 = gpiod.pd11.into_floating_input(&mut gpiod.moder, &mut gpiod.pupdr);
    // Setup PB3 - PB7, PD0 - 7, PC10 - PC12 (for address outputs)
    let pb3 = gpiob.pb3.into_floating_input(&mut gpiob.moder, &mut gpiob.pupdr);
    let pb4 = gpiob.pb4.into_floating_input(&mut gpiob.moder, &mut gpiob.pupdr);
    let pb5 = gpiob.pb5.into_floating_input(&mut gpiob.moder, &mut gpiob.pupdr);
    let pb6 = gpiob.pb6.into_floating_input(&mut gpiob.moder, &mut gpiob.pupdr);
    let pb7 = gpiob.pb7.into_floating_input(&mut gpiob.moder, &mut gpiob.pupdr);
    let pd0 = gpiod.pd0.into_floating_input(&mut gpiod.moder, &mut gpiod.pupdr);
    let pd1 = gpiod.pd1.into_floating_input(&mut gpiod.moder, &mut gpiod.pupdr);
    let pd2 = gpiod.pd2.into_floating_input(&mut gpiod.moder, &mut gpiod.pupdr);
    let pd3 = gpiod.pd3.into_floating_input(&mut gpiod.moder, &mut gpiod.pupdr);
    let pd4 = gpiod.pd4.into_floating_input(&mut gpiod.moder, &mut gpiod.pupdr);
    let pd5 = gpiod.pd5.into_floating_input(&mut gpiod.moder, &mut gpiod.pupdr);
    let pd6 = gpiod.pd6.into_floating_input(&mut gpiod.moder, &mut gpiod.pupdr);
    let pd7 = gpiod.pd7.into_floating_input(&mut gpiod.moder, &mut gpiod.pupdr);
    let pc10 = gpioc.pc10.into_floating_input(&mut gpioc.moder, &mut gpioc.pupdr);
    let pc11 = gpioc.pc11.into_floating_input(&mut gpioc.moder, &mut gpioc.pupdr);
    let pc12 = gpioc.pc12.into_floating_input(&mut gpioc.moder, &mut gpioc.pupdr);
    // Setup PB11 as input for recording the bus read-write (rwb) value
    let rwb_pin = gpiob.pb11.into_floating_input(&mut gpiob.moder, &mut gpiob.pupdr);

    data_pins[0] = Some(pd10.downgrade().downgrade());
    data_pins[1] = Some(pd8.downgrade().downgrade());
    data_pins[2] = Some(pb14.downgrade().downgrade());
    data_pins[3] = Some(pb12.downgrade().downgrade());
    data_pins[4] = Some(pd11.downgrade().downgrade());
    data_pins[5] = Some(pd9.downgrade().downgrade());
    data_pins[6] = Some(pb15.downgrade().downgrade());
    data_pins[7] = Some(pb13.downgrade().downgrade());

    address_pins[0] = Some(pb7.downgrade().downgrade());
    address_pins[1] = Some(pb5.downgrade().downgrade());
    address_pins[2] = Some(pb3.downgrade().downgrade());
    address_pins[3] = Some(pd6.downgrade().downgrade());
    address_pins[4] = Some(pd4.downgrade().downgrade());
    address_pins[5] = Some(pd2.downgrade().downgrade());
    address_pins[6] = Some(pd0.downgrade().downgrade());
    address_pins[7] = Some(pc11.downgrade().downgrade());

    address_pins[8] = Some(pb6.downgrade().downgrade());
    address_pins[9] = Some(pb4.downgrade().downgrade());
    address_pins[10] = Some(pd7.downgrade().downgrade());
    address_pins[11] = Some(pd5.downgrade().downgrade());
    address_pins[12] = Some(pd3.downgrade().downgrade());
    address_pins[13] = Some(pd1.downgrade().downgrade());
    address_pins[14] = Some(pc12.downgrade().downgrade());
    address_pins[15] = Some(pc10.downgrade().downgrade());

    // Setup tx and rx pins (PC4 and PC5) for usart1
    let tx = gpioc.pc4.into_af7_push_pull(&mut gpioc.moder, &mut gpioc.otyper, &mut gpioc.afrl);
    let rx = gpioc.pc5.into_af7_push_pull(&mut gpioc.moder, &mut gpioc.otyper, &mut gpioc.afrl);

    (data_pins, address_pins, rwb_pin, tx, rx)
}



