#![no_std]
#![no_main]

use panic_halt as _;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {

    use core::str;
    use stm32f4xx_hal::{
        gpio,
        gpio::{Input, Output, PushPull},
        otg_fs::{UsbBus, UsbBusType, USB},
        pac,
        prelude::*,
        timer,
        serial::{config::Config, Event::Rxne, Event::Txe, Serial},
    };

    use heapless::spsc::{Consumer, Producer, Queue};
    use usb_device::{class_prelude::*, prelude::*};

    use usbd_dfu_rt::{DfuRuntimeClass, DfuRuntimeOps};
    use usbd_serial::SerialPort;
    use embedded_hal::digital::v2::OutputPin;

    use core::fmt::Write;

    use ushell::{
        autocomplete::StaticAutocomplete, history::LRUHistory, Input as ushell_input,
        ShellError as ushell_error, UShell,
    };

    type ShellType = UShell<SerialPort<'static, UsbBusType>, StaticAutocomplete<10>, LRUHistory<128, 4>, 128>;
    const SHELL_PROMPT: &str = "#> ";
    const CR: &str = "\r\n";
    const HELP: &str = "\r\nhelp: print this help\r\n\
        storage dut|host|off: switch storage to dut, host or power off\r\n\
        meter [monitor]: read power consumption, monitor will continue to read until CTRL+C is pressed\r\n\
        power on|off: power on or off the DUT\r\n\
        reset: reset the DUT\r\n\
        send string: send string to the DUT\r\n\
        ctl_[a,b,c,d] low|high|hiz: set CTL_A to low, high or high impedance\r\n\
        ";

    // Device control abstractions
    pub struct StorageSwitch<OEnPin, SelPin, PwDUTPin, PWHostPin>
    where
        OEnPin: OutputPin,
        SelPin: OutputPin,
        PwDUTPin: OutputPin,
        PWHostPin: OutputPin,
    {
        usb_store_oen: OEnPin,
        usb_store_sel: SelPin,
        usb_pw_dut: PwDUTPin,
        usb_pw_host: PWHostPin,
    }

    impl<OEnPin, SelPin, PwDUTPin, PWHostPin> StorageSwitch<OEnPin, SelPin, PwDUTPin, PWHostPin>
    where
        OEnPin: OutputPin,
        SelPin: OutputPin,
        PwDUTPin: OutputPin,
        PWHostPin: OutputPin,
    {
        fn new(
            usb_store_oen: OEnPin,
            usb_store_sel: SelPin,
            usb_pw_dut: PwDUTPin,
            usb_pw_host: PWHostPin,
        ) -> Self {
            Self {
                usb_store_oen,
                usb_store_sel,
                usb_pw_dut,
                usb_pw_host,
            }
        }

        fn power_off(&mut self) {
           self.usb_pw_dut.set_low().ok();
           self.usb_pw_host.set_low().ok();
           self.usb_store_oen.set_high().ok();
        }

        fn connect_to_dut(&mut self) {
            self.usb_pw_host.set_low().ok();
            self.usb_pw_dut.set_high().ok();
            self.usb_store_oen.set_low().ok();
            self.usb_store_sel.set_high().ok();
        }

        fn connect_to_host(&mut self) {
            self.usb_pw_dut.set_low().ok();
            self.usb_pw_host.set_high().ok();
            self.usb_store_oen.set_low().ok();
            self.usb_store_sel.set_high().ok();
        }
    }

    // Resources shared between tasks
    #[shared]
    struct Shared {
        timer: timer::CounterMs<pac::TIM2>,
        usb_dev: UsbDevice<'static, UsbBusType>,
        shell: ShellType,
        serial2: SerialPort<'static, UsbBusType>,
        usart:  Serial<pac::USART1, (gpio::Pin<'B', 6>, gpio::Pin<'B', 7>)>,
        dfu: DfuRuntimeClass<DFUBootloader>,
        led_tx: gpio::PC13<Output<PushPull>>,
        led_rx: gpio::PC14<Output<PushPull>>,
        led_cmd: gpio::PC15<Output<PushPull>>,
        storage: StorageSwitch<
            gpio::PA15<Output<PushPull>>,
            gpio::PB3<Output<PushPull>>,
            gpio::PB4<Output<PushPull>>,
            gpio::PB5<Output<PushPull>>>,
        power_device: gpio::PA4<Output<PushPull>>,
    }

    // Local resources to specific tasks (cannot be shared)
    #[local]
    struct Local {
        button: gpio::PA0<Input>,
        to_dut_serial: Producer<'static, u8, 128>,
        to_dut_serial_consumer: Consumer<'static, u8, 128>,
    }

    #[init(local = [q_to_dut: Queue<u8, 128> = Queue::new()])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>,
        > = None;
        static mut EP_MEMORY: [u32; 1024] = [0; 1024];

        let dp = ctx.device;
        let rcc = dp.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(25.MHz())
            .sysclk(48.MHz())
            .require_pll48clk()
            .freeze();

        // Configure the on-board LED (PC13, blue)
        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();
        
        let mut led_tx = gpioc.pc13.into_push_pull_output();
        let mut led_rx = gpioc.pc14.into_push_pull_output();
        let mut led_cmd = gpioc.pc15.into_push_pull_output();

        led_tx.set_high();
        led_rx.set_high();
        led_cmd.set_high();
        
        let button = gpioa.pa0.into_pull_up_input();
        
        let mut ctl_a = gpioa.pa5.into_open_drain_output();
        let mut ctl_b = gpioa.pa6.into_open_drain_output();
        let mut ctl_c = gpioa.pa7.into_open_drain_output();
        let mut ctl_d = gpioa.pa8.into_open_drain_output();
        let mut reset_out = gpioa.pa9.into_open_drain_output();

        let mut power_device = gpioa.pa4.into_push_pull_output();

        let pins = (gpiob.pb6, gpiob.pb7);
        let mut usart = Serial::new(
            dp.USART1,
            pins,
            Config::default().baudrate(115_200.bps()).wordlength_8(),
            &clocks,
        ).unwrap().with_u8_data();
        usart.listen(Rxne);
        //usart.listen(Txe);


        let _current_sense = gpioa.pa1.into_analog();
        let _vout_sense = gpioa.pa2.into_analog();

        let mut storage = StorageSwitch::new(
            gpioa.pa15.into_push_pull_output(), //OEn
            gpiob.pb3.into_push_pull_output(), //SEL
            gpiob.pb4.into_push_pull_output(), //PW_HOST
            gpiob.pb5.into_push_pull_output() //PW_DUT
        );

        storage.power_off();

        reset_out.set_high();
        ctl_a.set_high();
        ctl_b.set_high();
        ctl_c.set_high();
        ctl_d.set_high();

        power_device.set_high();
        
        let mut timer = dp.TIM2.counter_ms(&clocks);
        timer.start(100.millis()).unwrap();
        // Set up to generate interrupt when timer expires
        timer.listen(timer::Event::Update);

        // Pull the D+ pin down to send a RESET condition to the USB bus.
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
        let serial2 = SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });
        let dfu = DfuRuntimeClass::new(unsafe { USB_BUS.as_ref().unwrap() }, DFUBootloader);

        let usb_dev = UsbDeviceBuilder::new(
            unsafe { USB_BUS.as_ref().unwrap() },
            UsbVidPid(0x2b23, 0x1012),
        )
        .manufacturer("Red Hat Inc.")
        .product("Jupstarter")
        .serial_number(get_serial_str())
        .device_release(0x0001)
        .self_powered(false)
        .max_power(250)
        .max_packet_size_0(64)
        .build();

         // ushell
         let autocomplete = StaticAutocomplete(
            ["help", "power", "storage", "send", "reset", "ctl_a",
             "ctl_b", "ctl_c", "ctl_d", "status"]);
         let history = LRUHistory::default();
         let shell = UShell::new(serial, autocomplete, history);

         let (to_dut_serial, to_dut_serial_consumer) = ctx.local.q_to_dut.split();
        (
            Shared {
                timer,
                usb_dev,
                shell,
                serial2,
                usart,
                dfu,
                led_tx,
                led_rx,
                led_cmd,
                storage,
                power_device
            },
            Local {
                button,
                to_dut_serial,
                to_dut_serial_consumer,  
            },
            // Move the monotonic timer to the RTIC run-time, this enables
            // scheduling
            init::Monotonics(),
        )
    }

    #[task(binds = USART1, priority=1, shared = [usart, shell, led_tx, led_rx])]
    fn usart_task(cx: usart_task::Context){
        let usart = cx.shared.usart;
        let shell = cx.shared.shell;
        let led_tx = cx.shared.led_tx;
        let led_rx = cx.shared.led_rx;

        let mut buf = [0u8; 64];
        let mut count = 0;
        (usart, shell, led_tx, led_rx).lock(|usart, shell, _led_tx, led_rx| {
            while usart.is_rx_not_empty() && count<buf.len() {
                led_rx.set_low();
                match usart.read() {
                    Ok(b) => {
                        buf[count] = b;
                        count += 1;
                    },
                    Err(_e) => {
                        break;
                    }
                }
            }

            let serial = shell.get_serial_mut();
            if count > 0 {
                serial.write(&buf[..count]).unwrap();
            }
           
    
        });
    }

    #[task(binds = OTG_FS, shared = [usb_dev, serial2, shell, dfu, led_cmd, storage, power_device], local=[to_dut_serial])]
    fn usb_task(mut cx: usb_task::Context) {
        let usb_dev = &mut cx.shared.usb_dev;
        let serial2 = &mut cx.shared.serial2;
        let shell = &mut cx.shared.shell;
        let dfu = &mut cx.shared.dfu;
        let led_cmd = &mut cx.shared.led_cmd;
        let storage = &mut cx.shared.storage;
        let powerdev = &mut cx.shared.power_device;
        let to_dut_serial = cx.local.to_dut_serial;

        (usb_dev, serial2, dfu, shell, led_cmd, storage, powerdev).lock(|usb_dev, serial2, dfu, shell, led_cmd, storage, powerdev| {
            let s = shell.get_serial_mut();

            if !usb_dev.poll(&mut [s, serial2, dfu]) {
                return;
            }

            loop {
                let result = shell.poll();
                match result {
                Ok(Some(ushell_input::Command((cmd, args)))) => {
                    led_cmd.set_low();
                    match cmd {
                            "help" => {
                                shell.write_str(HELP).ok();
                            }
                            "clear" => {
                                shell.clear().ok();
                            }
                            "storage" => {
                                if args == "dut" {
                                    storage.connect_to_dut();
                                    write!(shell, "{0:}storage connected to device under test.{0:}", CR).ok();
                                } else if args == "host" {
                                    storage.connect_to_host();
                                    write!(shell, "{0:}storage connected to host.{0:}", CR).ok();
                                } else if args == "off" {
                                    storage.power_off();
                                    write!(shell, "{0:}storage disconnected.{0:}", CR).ok();
                                } else {
                                    write!(shell, "{0:}usage: storage dut|host|off{0:}",CR).ok();
                                }
                            }
                            "power" => {
                                if args == "on" {
                                    powerdev.set_high();
                                    write!(shell, "{0:}device powered on.{0:}", CR).ok();
                                } else if args == "off" {
                                    powerdev.set_low();
                                    write!(shell, "{0:}device powered off.{0:}", CR).ok();
                                } else {
                                    write!(shell, "{0:}usage: power on|off{0:}",CR).ok();
                                }
                            }
                            "send" => {   
                                for c in args.chars() {
                                    to_dut_serial.enqueue(c as u8).ok();
                                }
                                write!(shell, "{0:}sent{0:}",CR).ok();
                            }
                            "status" => {
                               // let on = led_enabled.lock(|e| *e);
                               // let status = if on { "On" } else { "Off" };
                               // write!(shell, "{0:}LED: {1:}{0:}", CR, status).ok();
                               write!(shell, "{0:}status: {0:}", CR).ok();
                            }
                            "" => {
                                shell.write_str(CR).ok();
                            }
                            _ => {
                                write!(shell, "{0:}unsupported command{0:}", CR).ok();
                            }
                        }
                        shell.write_str(SHELL_PROMPT).ok();
                    
                }
                Err(ushell_error::WouldBlock) => break,
                _ => {}
            }
        }
        });
    }

    #[task(binds = TIM2, shared=[timer, dfu,  led_rx, led_tx, led_cmd])]
    fn timer_expired(mut ctx: timer_expired::Context) {

        ctx.shared.dfu.lock(|dfu| dfu.tick(100));
        // clear all leds set in other tasts
        ctx.shared.led_rx.lock(|led_rx| led_rx.set_high());
        ctx.shared.led_tx.lock(|led_tx| led_tx.set_high());
        ctx.shared.led_cmd.lock(|led_cmd| led_cmd.set_high());
        ctx.shared
            .timer
            .lock(|tim| tim.clear_interrupt(timer::Event::Update));
    }

    // Background task, runs whenever no other tasks are running
    #[idle(local=[to_dut_serial_consumer], shared=[usart, led_tx])]
    fn idle(mut ctx: idle::Context) -> ! {

        let to_dut_serial_consumer = &mut ctx.local.to_dut_serial_consumer;

        loop {
            // Go to sleep, wake up on interrupt
            let mut escaped = false;
            cortex_m::asm::wfi();
            if to_dut_serial_consumer.len() == 0 {
                continue;
            }
                loop {
                    
                    match to_dut_serial_consumer.dequeue() {
                        Some(c) => {
                            let mut final_c:u8 = c;
                            if escaped == false && c == 0x5c { // backslash
                                escaped = true;
                                continue;
                            }
                            
                            if escaped == true {
                                escaped = false;
                                match c {
                                    0x5c => { // \\
                                        final_c = 0x5c;
                                    },
                                    0x6e => { // \n
                                        final_c = 0x0a;
                                    },
                                    0x72 => { // \r
                                        final_c = 0x0d;
                                    },
                                    0x74 => { // \t 
                                        final_c = 0x09;
                                    },
                                    0x61 => { // \a alert
                                        final_c = 0x07;
                                    },
                                    0x62 => { // \b backspace
                                        final_c = 0x08;
                                    },
                                    0x65 => { // \e escape character
                                        final_c = 0x1b;
                                    },
                                    _ => {
                                        final_c = c;
                                    }
                                }
                              
                            }

                            let usart = &mut ctx.shared.usart;
                            let led_tx = &mut ctx.shared.led_tx;
                            (usart, led_tx).lock(|usart, led_tx| {
                                led_tx.set_low();
                                loop { // Meeehh, move to a separate task ,listeing to TxISR, this will block
                                       // other tasts from running
                                    if usart.is_tx_empty() {
                                        break;
                                    }
                                }
                       
                                usart.write(final_c).ok();
                            });
                            
                        },
                        None => {
                            break;
                        }
                    }
                }
           
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
