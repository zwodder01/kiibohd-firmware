// Copyright 2021 Jacob Alexander
//
// Licensed under the Apache License, Version 2.0, <LICENSE-APACHE or
// http://apache.org/licenses/LICENSE-2.0> or the MIT license <LICENSE-MIT or
// http://opensource.org/licenses/MIT>, at your option. This file may not be
// copied, modified, or distributed except according to those terms.

// TODO Remove this
#![allow(clippy::inconsistent_struct_constructor)]
#![no_std]
#![no_main]

use const_env::from_env;
use core::panic::PanicInfo;
use cortex_m_rt::exception;
use heapless::spsc::Queue;
use kiibohd_log::{log, Logger};
use kiibohd_usb::HidCountryCode;
use rtic::app;

use gemini::{
    hal::{
        clock::{ClockController, MainClock, SlowClock},
        gpio::*,
        pac::Peripherals,
        prelude::*,
        rtt::RealTimeTimer,
        time::duration::Extensions,
        udp::{
            usb_device,
            usb_device::{
                bus::UsbBusAllocator,
                device::{UsbDeviceBuilder, UsbVidPid},
            },
            UdpBus,
        },
        watchdog::Watchdog,
        ToggleableOutputPin,
    },
    Pins,
};

#[from_env]
const VID: u16 = 0x1c11;
#[from_env]
const PID: u16 = 0xb04d;
#[from_env]
const USB_MANUFACTURER: &str = "Unknown";
#[from_env]
const USB_PRODUCT: &str = "Kiibohd";
// TODO
const USB_SERIAL: &str = ">TODO SERIAL<";

const KBD_QUEUE_SIZE: usize = 1;
const MOUSE_QUEUE_SIZE: usize = 1;
const CTRL_QUEUE_SIZE: usize = 1;
const HIDIO_RX_QUEUE_SIZE: usize = 1;
const HIDIO_TX_QUEUE_SIZE: usize = 1;

// Define static lifetimes for USB
type UsbDevice = usb_device::device::UsbDevice<'static, UdpBus>;
type HidInterface = kiibohd_usb::HidInterface<
    'static,
    UdpBus,
    KBD_QUEUE_SIZE,
    MOUSE_QUEUE_SIZE,
    CTRL_QUEUE_SIZE,
    HIDIO_RX_QUEUE_SIZE,
    HIDIO_TX_QUEUE_SIZE,
>;

static LOGGER: Logger = Logger::new(log::LevelFilter::Trace);

#[app(device = gemini::hal::pac, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    //
    // Resources used by tasks/interrupts
    //
    struct Resources {
        debug_led: Pb0<Output<PushPull>>,
        wdt: Watchdog,
        rtt: RealTimeTimer,
        usb_dev: UsbDevice,
        usb_hid: HidInterface,
    }

    //
    // Initialization
    //
    #[init()]
    fn init(mut cx: init::Context) -> init::LateResources {
        // TODO once_cell?
        static mut USB_BUS: Option<UsbBusAllocator<UdpBus>> = None;
        static mut KBD_QUEUE: Queue<kiibohd_usb::KeyState, KBD_QUEUE_SIZE> = Queue::new();
        static mut MOUSE_QUEUE: Queue<kiibohd_usb::MouseState, MOUSE_QUEUE_SIZE> = Queue::new();
        static mut CTRL_QUEUE: Queue<kiibohd_usb::CtrlState, CTRL_QUEUE_SIZE> = Queue::new();
        static mut HIDIO_RX_QUEUE: Queue<kiibohd_usb::HidioPacket, HIDIO_RX_QUEUE_SIZE> =
            Queue::new();
        static mut HIDIO_TX_QUEUE: Queue<kiibohd_usb::HidioPacket, HIDIO_TX_QUEUE_SIZE> =
            Queue::new();

        // Initialize (enable) the monotonic timer (CYCCNT)
        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();

        // Setup RTT logging
        let channels = rtt_target::rtt_init! {
            up: {
                0: {
                    size: 1024
                    //mode: BlockIfFull
                    name: "Terminal"
                }
            }
        };
        rtt_target::set_print_channel(channels.up.0);
        log::set_logger(&LOGGER).unwrap();
        log::set_max_level(log::LevelFilter::Trace);
        log::info!(">>>> Initializing <<<<");

        // Setup main and slow clocks
        let peripherals = Peripherals::take().unwrap();
        log::trace!("Clock initialization");
        let clocks = ClockController::new(
            peripherals.PMC,
            &peripherals.SUPC,
            &peripherals.EFC0,
            MainClock::Crystal12Mhz,
            SlowClock::RcOscillator32Khz,
        );
        log::trace!("Clock initialized");

        // Setup gpios
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
        let pins = Pins::new(gpio_ports, &peripherals.MATRIX);

        // Prepare watchdog to be fed
        let mut wdt = Watchdog::new(peripherals.WDT);
        wdt.feed();
        log::trace!("Watchdog first feed");

        // Setup USB
        let (_kbd_producer, kbd_consumer) = KBD_QUEUE.split();
        let (_mouse_producer, mouse_consumer) = MOUSE_QUEUE.split();
        let (_ctrl_producer, ctrl_consumer) = CTRL_QUEUE.split();
        let (hidio_rx_producer, _hidio_rx_consumer) = HIDIO_RX_QUEUE.split();
        let (_hidio_tx_producer, hidio_tx_consumer) = HIDIO_TX_QUEUE.split();
        let udp_bus = UdpBus::new(
            peripherals.UDP,
            clocks.peripheral_clocks.udp,
            pins.udp_ddm,
            pins.udp_ddp,
        );
        *USB_BUS = Some(UsbBusAllocator::<UdpBus>::new(udp_bus));
        let usb_bus = USB_BUS.as_ref().unwrap();
        let usb_hid = HidInterface::new(
            usb_bus,
            HidCountryCode::NotSupported,
            kbd_consumer,
            mouse_consumer,
            ctrl_consumer,
            hidio_rx_producer,
            hidio_tx_consumer,
        );
        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(VID, PID))
            .manufacturer(USB_MANUFACTURER)
            .max_packet_size_0(64)
            .max_power(500)
            .product(USB_PRODUCT)
            .supports_remote_wakeup(true) // TODO Add support
            .serial_number(USB_SERIAL) // TODO how to store and format string
            .device_release(0x1234) // TODO Get git revision info (sequential commit number)
            .build();

        // Setup main timer (TODO May want to use a TC timer instead and reserve this for sleeping)
        let mut rtt = RealTimeTimer::new(peripherals.RTT, 3, false);
        rtt.start(500_000u32.microseconds());
        rtt.enable_alarm_interrupt();
        log::trace!("RTT Timer started");

        init::LateResources {
            debug_led: pins.debug_led,
            wdt,
            rtt,
            usb_dev,
            usb_hid,
        }
    }

    /// Keyscanning Task (Uses RTT)
    /// High-priority scheduled tasks as consistency is more important than speed for scanning
    /// key states
    /// Scans one strobe at a time
    #[task(binds = RTT, spawn = [macro_process], resources = [rtt, wdt], priority = 14)]
    fn rtt(cx: rtt::Context) {
        cx.resources.rtt.clear_interrupt_flags();

        // Feed watchdog
        cx.resources.wdt.feed();

        // TODO Add keyscanning as a pre-requisite to scheduling macro processing
        if cx.spawn.macro_process().is_err() {
            log::warn!("Could not schedule macro_process");
        }
    }

    /// Macro Processing Task
    /// Handles incoming key scan triggers and turns them into results (actions and hid events)
    /// Has a lower priority than keyscanning to schedule around it.
    #[task(spawn = [usb_process], priority = 10)]
    fn macro_process(cx: macro_process::Context) {
        // TODO Enable
        //unsafe { Macro_periodic() };

        if cx.spawn.usb_process().is_err() {
            log::warn!("Could not schedule usb_process");
        }
    }

    /// USB Outgoing Events Task
    /// Sends outgoing USB HID events generated by the macro_process task
    /// Has a lower priority than keyscanning to schedule around it.
    #[task(resources = [debug_led, usb_hid], priority = 10)]
    fn usb_process(mut cx: usb_process::Context) {
        // Blink debug led
        // TODO: Remove (or use feature flag)
        cx.resources.debug_led.toggle().ok();

        cx.resources.usb_hid.lock(|usb_hid| {
            usb_hid.push();
        });
    }

    /// ISSI I2C0 Interrupt
    #[task(binds = TWI0, priority = 12)]
    fn twi0(_: twi0::Context) {
        //unsafe { TWI0_Handler() };
    }

    /// ISSI I2C1 Interrupt
    #[task(binds = TWI1, priority = 12)]
    fn twi1(_: twi1::Context) {
        //unsafe { TWI1_Handler() };
    }

    /// USB Device Interupt
    #[task(binds = UDP, priority = 13, resources = [usb_dev, usb_hid])]
    fn udp(cx: udp::Context) {
        let usb_dev = cx.resources.usb_dev;
        let usb_hid = cx.resources.usb_hid;

        // Poll USB endpoints
        if usb_dev.poll(&mut usb_hid.interfaces()) {
            usb_hid.poll();
        }
    }

    // RTIC requires that unused interrupts are declared in an extern block when
    // using software tasks; these free interrupts will be used to dispatch the
    // software tasks.
    extern "C" {
        fn UART1();
        fn USART0();
        fn USART1();
        fn SPI();
        fn SSC();
        fn TC0(); // Timer Module 0, Channel 0
        fn TC1(); // Timer Module 0, Channel 1
        fn TC2(); // Timer Module 0, Channel 2
        fn ADC();
        fn PWM();
        fn ACC();
    }
};

#[exception]
fn HardFault(_ef: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("HardFault!");
}

/*
fn controller_setup() {
    unsafe {
        //Macro_setup();
    }
}
*/

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    log::error!("Panic! {}", info);
    // TODO Handle restart in non-debug mode
    loop {}
}
