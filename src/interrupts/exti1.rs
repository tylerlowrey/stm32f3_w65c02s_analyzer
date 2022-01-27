use core::sync::atomic::{AtomicBool, Ordering};
use stm32f3_discovery::stm32f3xx_hal::interrupt;
use crate::interrupts::{clear_exti_interrupt, ExtiLine};

pub(crate) static PA1_INTERRUPT_RECEIVED: AtomicBool = AtomicBool::new(false);

#[interrupt]
fn EXTI1() {
    unsafe { clear_exti_interrupt(ExtiLine::Exti1); }

    PA1_INTERRUPT_RECEIVED.store(true, Ordering::Relaxed)
}
