//! CDC-ACM serial port example using polling in a busy loop.
#![no_std]
#![no_main]

use panic_halt as _;

use stm32f4xx_hal::interrupt::{SPI5, SPI4};

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI5, SPI4])]
mod app {

    use core::str;
    use stm32f4xx_hal::{
        gpio,
        gpio::{Input, Output, PushPull},
        otg_fs::{UsbBus, UsbBusType, USB},
        prelude::*,
    };

    use usb_device::{class_prelude::*, prelude::*};

    use usbd_dfu_rt::{DfuRuntimeClass, DfuRuntimeOps};
    use usbd_serial::SerialPort;

    use arrform::{arrform, ArrForm};

    use systick_monotonic::{*, fugit::MicrosDurationU64}; // Implements the `Monotonic` trait

    // A monotonic timer to enable scheduling in RTIC
    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<10000>; // 10000 Hz / 0.1 ms granularity

    // Resources shared between tasks
    #[shared]
    struct Shared {
        usb_dev: UsbDevice<'static, UsbBusType>,
        serial: SerialPort<'static, UsbBusType>,
        dfu: DfuRuntimeClass<DFUBootloader>,
    }

    // Local resources to specific tasks (cannot be shared)
    #[local]
    struct Local {
        button: gpio::PA0<Input>,
        led: gpio::PC13<Output<PushPull>>,
        usnd_trigger: gpio::PB9<Output<PushPull>>,
        usnd_echo: gpio::PB8<Input>,
        usnd_start: fugit::Instant<u64,1,10000> ,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>,
        > = None;
        static mut EP_MEMORY: [u32; 1024] = [0; 1024];

        let mut dp = ctx.device;
        let mut syscfg = dp.SYSCFG.constrain();
        let clocks = dp.RCC.constrain()
            .cfgr
            .use_hse(25.MHz())
            .sysclk(48.MHz())
            .require_pll48clk()
            .freeze();

        let mono = Systick::new(ctx.core.SYST, 48_000_000);

        // Configure the on-board LED (PC13, blue)
        let gpioc = dp.GPIOC.split();
        let gpiob = dp.GPIOB.split();
        let gpioa = dp.GPIOA.split();
        let mut led = gpioc.pc13.into_push_pull_output();
        let button = gpioa.pa0.into_pull_up_input();
        
        let usnd_trigger = gpiob.pb9.into_push_pull_output();
        let mut usnd_echo = gpiob.pb8.into_pull_up_input();

        usnd_echo.make_interrupt_source(&mut syscfg);
        usnd_echo.trigger_on_edge(&mut dp.EXTI, gpio::Edge::RisingFalling);
        usnd_echo.enable_interrupt(&mut dp.EXTI);
        
        // BlackPill board has a pull-up resistor on the D+ line.
        // Pull the D+ pin down to send a RESET condition to the USB bus.
        // This forced reset is needed only for development, without it host
        // will not reset your device when you upload new firmware.
        let mut usb_dp = gpioa.pa12.into_push_pull_output();
        usb_dp.set_low();
        cortex_m::asm::delay(1024 * 10);

        let usb_periph = USB {
            usb_global: dp.OTG_FS_GLOBAL,
            usb_device: dp.OTG_FS_DEVICE,
            usb_pwrclk: dp.OTG_FS_PWRCLK,
            hclk: clocks.hclk(),
            pin_dm: gpioa.pa11.into_alternate(),
            pin_dp: usb_dp.into_alternate(),
        };

        unsafe {
            USB_BUS = Some(UsbBus::new(usb_periph, &mut EP_MEMORY));
        }

        let serial = SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });
        let dfu = DfuRuntimeClass::new(unsafe { USB_BUS.as_ref().unwrap() }, DFUBootloader);

        let usb_dev = UsbDeviceBuilder::new(
            unsafe { USB_BUS.as_ref().unwrap() },
            UsbVidPid(0x2b23, 0x1012),
        )
        .manufacturer("Red Hat Inc.")
        .product("Jumpstarter")
        .serial_number(get_serial_str())
        .device_release(0x0202) // This is 2.02, you can use a version under the xml declared version
        // for development purposes (allows flashing many times as the version
        // will always be "updateable")
        .self_powered(false)
        .max_power(250)
        .max_packet_size_0(64)
        .build();

        led.set_high();

        // Schedule tasks
        dfu_tick::spawn_after((100 as u64).millis()).unwrap();
        radar::spawn_after((1000 as u64).millis()).unwrap();
        let usnd_start = MyMono::zero();

        (
            Shared {
                usb_dev,
                serial,
                dfu,
            },
            Local {
                button,
                led,
                usnd_trigger,
                usnd_echo,
                usnd_start,
            },
            // Move the monotonic timer to the RTIC run-time, this enables
            // scheduling
            init::Monotonics(mono),
        )
    }

    #[task(binds = EXTI9_5, priority = 2, shared = [serial], local = [led, usnd_echo, usnd_start])]
    fn usnd_echo(mut ctx: usnd_echo::Context) {
        let led = ctx.local.led;
        let usnd_echo = ctx.local.usnd_echo;
        let usnd_start = ctx.local.usnd_start;
        let serial = &mut ctx.shared.serial;

        if usnd_echo.is_high() {
            led.set_low();
            *usnd_start = monotonics::MyMono::now();
        } else {
            let duration = monotonics::MyMono::now() - *usnd_start;
            led.set_high();
            let speed_of_sound = 320; // m/s
            let distance_cm = (duration.to_micros() * speed_of_sound * 100)  / (2 * 1_000_000);
                    
            let af = arrform!(64, "{:?}\n\r", distance_cm);
            serial.lock(|serial| serial.write(af.as_bytes()).ok());
            
            
        }

        usnd_echo.clear_interrupt_pending_bit();
    }

    #[task(binds = OTG_FS, shared = [usb_dev, serial, dfu])]
    fn usb_task(mut cx: usb_task::Context) {
        let usb_dev = &mut cx.shared.usb_dev;
        let serial = &mut cx.shared.serial;
        let dfu = &mut cx.shared.dfu;

        (usb_dev, serial, dfu).lock(|usb_dev, serial, dfu| {
            if !usb_dev.poll(&mut [serial, dfu]) {
                return;
            }
            let mut buf = [0u8; 64];

            match serial.read(&mut buf) {
                Ok(count) if count > 0 => {
                    let af = arrform!(64, "Received {} bytes: {:02x?}\r\n", count, &buf[..count]);
                    serial.write(af.as_bytes()).ok();
                }
                _ => {}
            }
        });
    }

    #[task(shared=[dfu])]
    fn dfu_tick(mut ctx: dfu_tick::Context) {
        dfu_tick::spawn_after((100 as u64).millis()).unwrap();
        ctx.shared.dfu.lock(|dfu| dfu.tick(100));
    }

    #[task(local=[usnd_trigger])]
    fn radar(ctx: radar::Context) {
        radar::spawn_after((100 as u64).millis()).unwrap();
       
        // Trigger the radar measurement
        ctx.local.usnd_trigger.set_high();
        cortex_m::asm::delay(48_000_000/10_000); // 0.01ms up at least
        ctx.local.usnd_trigger.set_low();
    }

    // Background task, runs whenever no other tasks are running
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            // Go to sleep
            cortex_m::asm::wfi()
        }
    }

    pub struct DFUBootloader;

    const KEY_STAY_IN_BOOT: u32 = 0xb0d42b89;

    impl DfuRuntimeOps for DFUBootloader {
        const DETACH_TIMEOUT_MS: u16 = 500;
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

    /// Return device serial based on U_ID registers.
    fn read_serial() -> u32 {
        let u_id0 = 0x1FFF_7A10 as *const u32;
        let u_id1 = 0x1FFF_7A14 as *const u32;

        unsafe { u_id0.read().wrapping_add(u_id1.read()) }
    }
}
