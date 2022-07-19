//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::{entry, hal::usb::UsbBus};
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::ToggleableOutputPin;
use embedded_time::fixed_point::FixedPoint;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use usb_device::{
    class_prelude::UsbBusAllocator,
    device::{UsbDeviceBuilder, UsbVidPid},
    UsbError,
};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

type MessageBuffer = [u8; 256];

fn fill_from_str(bytes: &mut [u8], s: &str) {
    for (i, c) in s.bytes().enumerate() {
        bytes[i] = c as u8;
    }
}

fn process_serial_msg(buf: &MessageBuffer, buff_size: usize) -> (MessageBuffer, usize) {
    match buf {
        [b'a', b'b', ..] => {
            let text = "ayy, bee!\n";
            let mut new_buf = [0u8; 256];
            fill_from_str(&mut new_buf, text);
            (new_buf, text.bytes().len())
        }
        _ => (*buf, buff_size),
    }
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();

    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut serial = SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .product("Serial port")
        .device_class(USB_CLASS_CDC)
        .build();

    let mut message_buffer: MessageBuffer = [0u8; 256];
    let mut message_buffer_pos = 0;

    let mut did_print: bool = false;

    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }

        let mut buf = [0u8; 64];

        match serial.read(&mut buf[..]) {
            Ok(count) => {
                // count bytes were read to &buf[..count]

                // Check if a byte is \r or \n, if so, process the message
                for byte in &buf[..count] {
                    message_buffer[message_buffer_pos] = *byte;
                    message_buffer_pos = (message_buffer_pos + 1) % message_buffer.len();

                    if *byte == b'\r' || *byte == b'\n' {
                        // Process the message, then print out a possible reply
                        (message_buffer, message_buffer_pos) =
                            process_serial_msg(&message_buffer, message_buffer_pos);
                        serial.write(&[b'\r', b'\n']).unwrap();
                        serial.write(&message_buffer[..message_buffer_pos]).unwrap();
                        message_buffer_pos = 0;
                        did_print = true;
                        led_pin.toggle().unwrap();
                    }
                }

                // Writeback so we get typing feedback
                if !did_print {
                    serial.write(&buf[..count]).unwrap();
                } else {
                    did_print = false;
                }
            }
            Err(UsbError::WouldBlock) => {} // No data received
            Err(err) => {}                  // An error occurred
        };
    }
}

// End of file
