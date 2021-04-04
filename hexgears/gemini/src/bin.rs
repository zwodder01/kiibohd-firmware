// Copyright 2021 Jacob Alexander
//
// Licensed under the Apache License, Version 2.0, <LICENSE-APACHE or
// http://apache.org/licenses/LICENSE-2.0> or the MIT license <LICENSE-MIT or
// http://opensource.org/licenses/MIT>, at your option. This file may not be
// copied, modified, or distributed except according to those terms.

#![no_std]
#![no_main]

use gemini::controller::*;

// Panic handler
extern crate panic_semihosting;
//use core::panic::PanicInfo;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;

use gemini::{
    hal::{
        clock::{ClockController, MainClock, SlowClock},
        delay::{Delay, DelayMs},
        gpio::Ports,
        pac::{CorePeripherals, Peripherals},
        prelude::*,
        watchdog::Watchdog,
        OutputPin,
    },
    Pins,
};

fn controller_setup() {
    unsafe {
        Latency_init();
        CLI_init();

        // TODO Periodic function

        //Storage_init();
        //Output_setup();
        //Macro_setup();
        //Scan_setup();

        //storage_load_settings();
    }
}

#[entry]
fn main() -> ! {
    hprintln!("Blinky example started").ok();

    let core = CorePeripherals::take().unwrap();
    let peripherals = Peripherals::take().unwrap();
    let clocks = ClockController::new(
        peripherals.PMC,
        &peripherals.SUPC,
        &peripherals.EFC0,
        MainClock::Crystal12Mhz,
        SlowClock::Crystal32Khz,
    );

    // Display why a processor reset occured.
    match peripherals.RSTC.sr.read().rsttyp().bits() {
        0 => hprintln!("Reset cause: First power up reset"),
        1 => hprintln!("Reset cause: Return from backup mode"),
        2 => hprintln!("Reset cause: Watchdog timer"),
        3 => hprintln!("Reset cause: Software"),
        4 => hprintln!("Reset cause: NRST pin detected low"),
        _ => hprintln!("Reset cause: RESERVED RESET VALUE!!"),
    }
    .ok();

    let gpio_ports = Ports::new(
        (
            peripherals.PIOA,
            clocks.peripheral_clocks.pio_a.into_enabled_clock(),
        ),
        (
            peripherals.PIOB,
            clocks.peripheral_clocks.pio_b.into_enabled_clock(),
        ),
    );
    let mut pins = Pins::new(gpio_ports);
    let mut delay = Delay::new(core.SYST);
    let mut wdt = Watchdog::new(peripherals.WDT);

    // Kiibohd Controller Setup
    controller_setup();

    loop {
        pins.debug_led.set_low().ok();
        wdt.feed();
        delay.delay_ms(1000u32);
        pins.debug_led.set_high().ok();
        wdt.feed();
        delay.delay_ms(1000u32);
    }
}

/*
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    let peripherals = Peripherals::take().unwrap();
    let clocks = ClockController::new();
    let gpio_ports = Ports::new(
        (
            peripherals.PIOA,
            clocks
                .peripheral_clocks
                .parallel_io_controller_a
                .into_enabled_clock(),
        ),
        (
            peripherals.PIOB,
            clocks
                .peripheral_clocks
                .parallel_io_controller_b
                .into_enabled_clock(),
        ),
    );
    let mut pins = Pins::new(gpio_ports);
        pins.debug_led.set_low().ok();
    loop {}
}
*/
