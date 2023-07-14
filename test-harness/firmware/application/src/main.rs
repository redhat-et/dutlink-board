//! CDC-ACM serial port example using polling in a busy loop.
#![no_std]
#![no_main]

use panic_halt as _;

use core::str;
use cortex_m_rt::entry;

use stm32f4xx_hal::{
    pac,
    prelude::*
};

use stm32f4xx_hal::otg_fs::{UsbBus, USB};

use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use usbd_dfu_rt::{DfuRuntimeOps,DfuRuntimeClass};

use arrform::{arrform, ArrForm};

static mut EP_MEMORY: [u32; 2048] = [0; 2048];

pub struct DFUBootloader;

const KEY_STAY_IN_BOOT: u32 = 0xb0d42b89;

impl DfuRuntimeOps for DFUBootloader {
    const DETACH_TIMEOUT_MS: u16 = 5000;
    const CAN_UPLOAD: bool = false;
    const WILL_DETACH: bool = true;
    
    fn detach(&mut self) {
        cortex_m::interrupt::disable();

        let cortex = unsafe { cortex_m::Peripherals::steal() };
    
        let p = 0x2000_0000 as *mut u32;
        unsafe { p.write_volatile(KEY_STAY_IN_BOOT) };
    
        cortex_m::asm::dsb();
        unsafe {
            // System reset request
            cortex.SCB.aircr.modify(|v| 0x05FA_0004 | (v & 0x700));
        }
        cortex_m::asm::dsb();
        loop {}
    }
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
        .use_hse(25.MHz())
        .sysclk(48.MHz())
        .require_pll48clk()
        .freeze();

    // Configure the on-board LED (PC13, blue)
    let gpioc = dp.GPIOC.split();
    let mut led = gpioc.pc13.into_push_pull_output();
    led.set_low(); // Turn on

    let gpioa = dp.GPIOA.split();

    // BlackPill board has a pull-up resistor on the D+ line.
    // Pull the D+ pin down to send a RESET condition to the USB bus.
    // This forced reset is needed only for development, without it host
    // will not reset your device when you upload new firmware.
    let mut usb_dp = gpioa.pa12.into_push_pull_output();
    usb_dp.set_low();
    cortex_m::asm::delay(1024*10);

    let usb_periph = USB {
        usb_global: dp.OTG_FS_GLOBAL,
        usb_device: dp.OTG_FS_DEVICE,
        usb_pwrclk: dp.OTG_FS_PWRCLK,
        pin_dm: gpioa.pa11.into_alternate(),
        pin_dp: usb_dp.into_alternate(),
        hclk: clocks.hclk(),
    };

    let usb_bus = unsafe {UsbBus::new(usb_periph, &mut EP_MEMORY )};

    let mut serial = SerialPort::new(&usb_bus);
    let mut serial2 = SerialPort::new(&usb_bus);
    //let mut dfu = DfuRuntimeClass::new(&usb_bus, DFUBootloader);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x2b23, 0x1012))
        .manufacturer("Red Hat Inc.")
        .product("Jumpstarter")
        .serial_number(get_serial_str())
        .device_release(0x0001) // This is 2.02, you can use a version under the xml declared version
                                // for development purposes (allows flashing many times as the version
                                // will always be "updateable")
        .self_powered(false)
        .max_power(250)
        .max_packet_size_0(64)
        .device_class(USB_CLASS_CDC)
        .build();
    
    loop {
        if !usb_dev.poll(&mut [&mut serial,&mut serial2]) {
            continue;
        }

        let mut buf = [0u8; 64];

        led.set_low(); // Turn on

        match serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                led.set_low(); // Turn on
                let af = arrform!(64, "Received {} bytes: {:02x?}\r\n", count, &buf[..count]);
                serial.write(af.as_bytes()).ok();
                serial2.write(af.as_bytes()).ok();
            }
            _ => {}
        }
        
        led.set_high(); // Turn off
    }
}


/// Returns device serial number as hex string slice.
fn get_serial_str() -> &'static str {
    static mut SERIAL: [u8; 8] = [b' '; 8];
    let serial = unsafe { SERIAL.as_mut() };

    fn hex(v: u8) -> u8 {
        match v {
            0..=9 => v + b'0',
            0xa..=0xf => v - 0xa + b'a',
            _ => b' ',
        }
    }

    let sn = read_serial();

    for (i, d) in serial.iter_mut().enumerate() {
        *d = hex(((sn >> (i * 4)) & 0xf) as u8)
    }

    unsafe { str::from_utf8_unchecked(serial) }
}


// Reads the serial number from the serial number field in the OTP
// memory.
// This is a 32-bit number that is unique to each device and
// is stored in the OTP memory.

/// Return device serial based on U_ID registers.
fn read_serial() -> u32 {
    let u_id0 = 0x1FFF_7A10 as *const u32;
    let u_id1 = 0x1FFF_7A14 as *const u32;

    unsafe { u_id0.read().wrapping_add(u_id1.read()) }
}

