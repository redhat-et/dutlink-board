#![no_std]
#![no_main]

// use panic_halt as _;
use panic_semihosting as _;

mod dfu;
mod storage;
mod usbserial;
mod shell;


#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {

    use stm32f4xx_hal::{
        gpio,
        gpio::{Input, Output, PushPull},
        otg_fs::{UsbBus, UsbBusType, USB},
        pac,
        prelude::*,
        timer,
        serial::{config::Config, Event::Rxne, Serial},
    };

    use heapless::spsc::{Consumer, Producer, Queue};
    use usb_device::{class_prelude::*, prelude::*};

    use usbd_serial::SerialPort;

    use crate::dfu::{DFUBootloaderRuntime, get_serial_str, new_dfu_bootloader};
    use crate::storage::*;
    use crate::usbserial::*;
    use crate::shell;

    type LedCmdType = gpio::PC15<Output<PushPull>>;
    type PowerEnableType = gpio::PA4<Output<PushPull>>;
    type StorageSwitchType = StorageSwitch<gpio::PA15<Output<PushPull>>, gpio::PB3<Output<PushPull>>,
                                           gpio::PB5<Output<PushPull>>, gpio::PB4<Output<PushPull>>>;

    // Resources shared between tasks
    #[shared]
    struct Shared {
        timer: timer::CounterMs<pac::TIM2>,
        usb_dev: UsbDevice<'static, UsbBusType>,
        shell: shell::ShellType,
        serial2: USBSerialType,
        usart:  Serial<pac::USART1, (gpio::Pin<'B', 6>, gpio::Pin<'B', 7>)>,
        dfu: DFUBootloaderRuntime,

        led_tx: gpio::PC13<Output<PushPull>>,
        led_rx: gpio::PC14<Output<PushPull>>,
        led_cmd: LedCmdType,

        storage: StorageSwitchType,

        power_device: PowerEnableType,
    }

    // Local resources to specific tasks (cannot be shared)
    #[local]
    struct Local {
        _button: gpio::PA0<Input>,
        to_dut_serial: Producer<'static, u8, 128>,          // queue of characters to send to the DUT
        to_dut_serial_consumer: Consumer<'static, u8, 128>, // consumer side of the queue
    }

    #[init(local = [q_to_dut: Queue<u8, 128> = Queue::new()])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>,> = None;
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
        
        let _button = gpioa.pa0.into_pull_up_input();
        
        let mut ctl_a = gpioa.pa5.into_open_drain_output();
        let mut ctl_b = gpioa.pa6.into_open_drain_output();
        let mut ctl_c = gpioa.pa7.into_open_drain_output();
        let mut ctl_d = gpioa.pa8.into_open_drain_output();
        let mut reset_out = gpioa.pa9.into_open_drain_output();

        let mut power_device = gpioa.pa4.into_push_pull_output();

        let pins = (gpiob.pb6, gpiob.pb7);
        let mut usart = Serial::new(
            dp.USART1,
            pins, // (tx, rx)
            Config::default().baudrate(115_200.bps()).wordlength_8(),
            &clocks,
        ).unwrap().with_u8_data();

        usart.listen(Rxne);


        let _current_sense = gpioa.pa1.into_analog();
        let _vout_sense = gpioa.pa2.into_analog();

        let mut storage = StorageSwitch::new(
            gpioa.pa15.into_push_pull_output(), //OEn
            gpiob.pb3.into_push_pull_output(), //SEL
            gpiob.pb5.into_push_pull_output(), //PW_DUT
            gpiob.pb4.into_push_pull_output(), //PW_HOST
        );

        storage.power_off();

        reset_out.set_high();
        ctl_a.set_high();
        ctl_b.set_high();
        ctl_c.set_high();
        ctl_d.set_high();

        power_device.set_high();
        
        // setup a timer for the periodic 100ms task
        let mut timer = dp.TIM2.counter_ms(&clocks);
        timer.start(100.millis()).unwrap();
        timer.listen(timer::Event::Update);

        // Pull the D+ pin down to send a RESET condition to the USB bus.
        let mut usb_dp = gpioa.pa12.into_push_pull_output();
        usb_dp.set_low();
        cortex_m::asm::delay(1024 * 50);

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
      
        let serial1 = new_usb_serial! (unsafe { USB_BUS.as_ref().unwrap() });
        let serial2 = new_usb_serial! (unsafe { USB_BUS.as_ref().unwrap() });
        let dfu = new_dfu_bootloader(unsafe { USB_BUS.as_ref().unwrap() });

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

         let shell = shell::new(serial1);
        
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
                _button,
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
                // we use .ok() instead of unwrap() to ignore the error if the host
                // isn't reading the usb serial port fast enough and the data overflows
                serial.write(&buf[..count]).ok();
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
            let serial1 = shell.get_serial_mut();

            if !usb_dev.poll(&mut [serial1, serial2, dfu]) {
                return;
            }
   
            let mut send_to_dut = |buf: &[u8]|{
                for b in buf {
                    to_dut_serial.enqueue(*b).ok();
                }
                return
            };

            shell::handle_shell_commands(shell, led_cmd, storage, powerdev, &mut send_to_dut);
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
        // the source of this queue is the send command from the shell
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
                                final_c = match escaped_char(c) {
                                    Some(c) => c,
                                    None =>  continue,
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

    fn escaped_char(c:u8) -> Option<u8> {

        match c {
            0x5c => { Some(0x5c) }, // \\
            0x6e => { Some(0x0a) }, // \n
            0x72 => { Some(0x0d) }, // \r
            0x74 => { Some(0x09) }, // \t 
            0x61 => { Some(0x07) }, // \a alert
            0x62 => { Some(0x08) }, // \b backspace
            0x65 => { Some(0x1b) }, // \e escape character
            0x63 => { Some(0x03) }, // \c // CTRL+C
            0x64 => { Some(0x04) }, // \d CTRL+D
            0x77 => { cortex_m::asm::delay(50*1000*1000); None },// \w WAIT DELAY
            _ => Some(c) 
        }
    }


}
