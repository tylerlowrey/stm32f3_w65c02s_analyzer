use stm32f3_discovery::stm32f3xx_hal::pac::exti::IMR1;
use stm32f3_discovery::stm32f3xx_hal::pac::{EXTI, Interrupt, NVIC, SYSCFG};

pub mod exti1;

#[derive(Clone, Copy)]
pub enum ExtiLine {
    Exti0,
    Exti1
}

#[allow(dead_code)]
pub enum ExtiInterruptInput {
    PA,
    PB,
    PC,
    PD,
    PE,
    PF,
    PG,
    PH
}

#[allow(dead_code)]
pub enum TriggerMode {
    Rising,
    Falling,
    Both
}

pub fn enable_exti_interrupt(exti: &EXTI, sysconfig: &SYSCFG, exti_line: ExtiLine,
                             exti_input: ExtiInterruptInput) {
    configure_exti(&exti.imr1, exti_line);
    map_input_to_exti(&sysconfig, exti_line, exti_input);
    set_exti_trigger_mode(&exti, exti_line, TriggerMode::Rising);

    let interrupt = match exti_line {
        ExtiLine::Exti0 => Interrupt::EXTI0,
        ExtiLine::Exti1 => Interrupt::EXTI1
    };

    unmask_interrupt(interrupt);
}

fn configure_exti(exti_imr1: &IMR1, exti_line: ExtiLine) {
    match exti_line {
        ExtiLine::Exti0 => exti_imr1.modify(|_, w| w.mr0().set_bit()),
        ExtiLine::Exti1 => exti_imr1.modify(|_, w| w.mr1().set_bit()),
    }
}

fn map_input_to_exti(syscfg: &SYSCFG, exti_line: ExtiLine, input: ExtiInterruptInput) {
    let exticr1_value;

    match input {
        ExtiInterruptInput::PA => exticr1_value = 0b000,
        ExtiInterruptInput::PB => exticr1_value = 0b001,
        ExtiInterruptInput::PC => exticr1_value = 0b010,
        ExtiInterruptInput::PD => exticr1_value = 0b011,
        ExtiInterruptInput::PE => exticr1_value = 0b100,
        ExtiInterruptInput::PF => exticr1_value = 0b101,
        ExtiInterruptInput::PG => exticr1_value = 0b110,
        ExtiInterruptInput::PH => exticr1_value = 0b111,
    }

    match exti_line {
        ExtiLine::Exti0 => unsafe { syscfg.exticr1.modify(|_,w| w.exti0().bits(exticr1_value))},
        ExtiLine::Exti1 => unsafe { syscfg.exticr1.modify(|_,w| w.exti1().bits(exticr1_value))},
    }
}

fn set_exti_trigger_mode(exti: &EXTI, exti_line: ExtiLine, trigger_mode: TriggerMode) {
    match exti_line {
        ExtiLine::Exti0 => exti.rtsr1.modify(|_,w| w.tr0().set_bit()),
        ExtiLine::Exti1 => exti.rtsr1.modify(|_,w| w.tr1().set_bit()),
    }
}

fn unmask_interrupt(interrupt: Interrupt) {
    unsafe { NVIC::unmask(interrupt); }
}

/// # Safety
///
/// This function dereferences a raw pointer to the EXTI register block. This function should
/// only be called from an interrupt context
unsafe fn clear_exti_interrupt(exti_line: ExtiLine) {
    let exti = &(*EXTI::ptr());
    match exti_line {
        ExtiLine::Exti0 => exti.pr1.write(|w| w.pr0().set_bit()),
        ExtiLine::Exti1 => exti.pr1.write(|w| w.pr1().set_bit()),
    }
}