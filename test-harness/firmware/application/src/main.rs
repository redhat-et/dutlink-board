#![no_std]
#![no_main]

// use panic_halt as _;
use panic_semihosting as _;

mod dfu;
mod storage;
mod usbserial;
mod shell;
mod ctlpins;


#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {

    use stm32f4xx_hal::{
        gpio,
        gpio::{Input, Output, PushPull},
        otg_fs::{UsbBus, UsbBusType, USB},
        pac,
        prelude::*,
        timer,
        serial::{config::Config, Tx, Rx, Serial},
        adc::{config::{AdcConfig, Dma, SampleTime, Scan, Sequence, Resolution}, Adc},
        dma::{config::DmaConfig, PeripheralToMemory, Stream0, StreamsTuple, Transfer},
        pac::{ADC1, DMA2},
    };
    use core::fmt::Write;

    use heapless::spsc::{Consumer, Producer, Queue};
    use usb_device::{class_prelude::*, prelude::*};

    use usbd_serial::SerialPort;

    use crate::dfu::{DFUBootloaderRuntime, get_serial_str, new_dfu_bootloader};
    use crate::storage::*;
    use crate::usbserial::*;
    use crate::shell;
    use crate::ctlpins;

    type LedCmdType = gpio::PC15<Output<PushPull>>;
    type PowerEnableType = gpio::PA4<Output<PushPull>>;
    type StorageSwitchType = StorageSwitch<gpio::PA15<Output<PushPull>>, gpio::PB3<Output<PushPull>>,
                                           gpio::PB5<Output<PushPull>>, gpio::PB4<Output<PushPull>>>;
    type DMATransfer = Transfer<Stream0<DMA2>, 0, Adc<ADC1>, PeripheralToMemory, &'static mut [u16; 2]>;

    const DUT_TX_BUF_SIZE: usize = 1024;
    // Resources shared between tasks
    #[shared]
    struct Shared {
        timer: timer::CounterMs<pac::TIM2>,
        usb_dev: UsbDevice<'static, UsbBusType>,
        shell: shell::ShellType,
        shell_status: shell::ShellStatus,
        dfu: DFUBootloaderRuntime,

        led_tx: gpio::PC13<Output<PushPull>>,
        led_rx: gpio::PC14<Output<PushPull>>,
        led_cmd: LedCmdType,

        storage: StorageSwitchType,

        power_device: PowerEnableType,

        adc_dma_transfer: DMATransfer,

        ctl_pins: ctlpins::CTLPins,

        pw_a: f32,
        pw_v: f32,
        pw_w: f32,
    }

    // Local resources to specific tasks (cannot be shared)
    #[local]
    struct Local {
        _button: gpio::PA0<Input>,
        usart_rx: Rx<pac::USART1>,
        usart_tx: Tx<pac::USART1>,
        to_dut_serial: Producer<'static, u8, DUT_TX_BUF_SIZE>,          // queue of characters to send to the DUT
        to_dut_serial_consumer: Consumer<'static, u8, DUT_TX_BUF_SIZE>, // consumer side of the queue
        adc_buffer: Option<&'static mut [u16; 2]>,
    }

    #[init(local = [q_to_dut: Queue<u8, DUT_TX_BUF_SIZE> = Queue::new()])]
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

        let ctl_pins = ctlpins::CTLPins::new(gpioa.pa5.into_dynamic(),
                                             gpioa.pa6.into_dynamic(),
                                             gpioa.pa7.into_dynamic(),
                                             gpioa.pa8.into_dynamic(),
                                             gpioa.pa9.into_dynamic(),
                                            );

        let mut power_device = gpioa.pa4.into_push_pull_output();

        let pins = (gpiob.pb6, gpiob.pb7);
        let usart = Serial::new(
            dp.USART1,
            pins, // (tx, rx)
            Config::default().baudrate(115_200.bps()).wordlength_8(),
            &clocks,
        ).unwrap().with_u8_data();

        let (usart_tx, mut usart_rx) = usart.split();

        usart_rx.listen();


        let current_sense = gpioa.pa1.into_analog();
        let vout_sense = gpioa.pa2.into_analog();
        let dma = StreamsTuple::new(dp.DMA2);
        let config = DmaConfig::default()
                    .transfer_complete_interrupt(true)
                    .memory_increment(true)
                    .double_buffer(false);

        let adc_config = AdcConfig::default()
                        .dma(Dma::Continuous)
                        .scan(Scan::Enabled)
                        .resolution(Resolution::Twelve);

        let mut adc = Adc::adc1(dp.ADC1, true, adc_config);

        adc.configure_channel(&current_sense, Sequence::One, SampleTime::Cycles_480);
        adc.configure_channel(&vout_sense, Sequence::Two, SampleTime::Cycles_480);
        adc.enable_temperature_and_vref();

        let first_buffer = cortex_m::singleton!(: [u16; 2] = [0; 2]).unwrap();
        let adc_buffer = Some(cortex_m::singleton!(: [u16; 2] = [0; 2]).unwrap());
        // Give the first buffer to the DMA. The second buffer is held in an Option in `local.buffer` until the transfer is complete
        let adc_dma_transfer = Transfer::init_peripheral_to_memory(dma.0, adc, first_buffer, None, config);

        let mut storage = StorageSwitch::new(
            gpioa.pa15.into_push_pull_output(), //OEn
            gpiob.pb3.into_push_pull_output(), //SEL
            gpiob.pb5.into_push_pull_output(), //PW_DUT
            gpiob.pb4.into_push_pull_output(), //PW_HOST
        );

        storage.power_off();

        power_device.set_low();

        // setup a timer for the periodic 100ms task
        let mut timer = dp.TIM2.counter_ms(&clocks);
        timer.start(10.millis()).unwrap(); //100Hz
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
        /* I tried creating a 2nd serial port which only works on STM32F412 , 411 has not enough
           endpoints, but it didn't work well, the library probably needs some debugging */
        let mut serial1 = new_usb_serial! (unsafe { USB_BUS.as_ref().unwrap() });
        let dfu = new_dfu_bootloader(unsafe { USB_BUS.as_ref().unwrap() });

        serial1.reset();

        let usb_dev = UsbDeviceBuilder::new(
            unsafe { USB_BUS.as_ref().unwrap() },
            UsbVidPid(0x2b23, 0x1012),
        )
        .manufacturer("Red Hat Inc.")
        .product("Jumpstarter")
        .serial_number(get_serial_str())
        .device_release(0x0002)
        .self_powered(false)
        .max_power(250)
        .max_packet_size_0(64)
        .build();

         let shell = shell::new(serial1);
         let shell_status = shell::ShellStatus{
             monitor_enabled: false,
             meter_enabled: false,
             console_mode: false,};
        let pw_a = 0.0;
        let pw_v = 0.0;
        let pw_w = 0.0;

        let (to_dut_serial, to_dut_serial_consumer) = ctx.local.q_to_dut.split();
        (
            Shared {
                timer,
                usb_dev,
                shell,
                shell_status,
                dfu,
                led_tx,
                led_rx,
                led_cmd,
                storage,
                power_device,
                adc_dma_transfer,
                ctl_pins,
                pw_a,
                pw_v,
                pw_w,
            },
            Local {
                _button,
                usart_tx,
                usart_rx,
                to_dut_serial,
                to_dut_serial_consumer,
                adc_buffer,
            },
            // Move the monotonic timer to the RTIC run-time, this enables
            // scheduling
            init::Monotonics(),
        )
    }

    #[task(binds = USART1, priority=1, local = [usart_rx], shared = [shell, shell_status, led_rx])]
    fn usart_task(cx: usart_task::Context){
        let usart_rx = cx.local.usart_rx;
        let shell = cx.shared.shell;
        let shell_status = cx.shared.shell_status;
        let led_rx = cx.shared.led_rx;

        let mut buf = [0u8; 64];
        let mut count = 0;
        (shell, shell_status, led_rx).lock(|shell, shell_status, led_rx| {
            while usart_rx.is_rx_not_empty() && count<buf.len() {
                led_rx.set_low();
                match usart_rx.read() {
                    Ok(b) => {
                        buf[count] = b;
                        count += 1;
                    },
                    Err(_e) => {
                        break;
                    }
                }
            }
            // when monitor mode or console mode is enabled, we send all data to the USB serial port
            if shell_status.console_mode || shell_status.monitor_enabled {
                let serial = shell.get_serial_mut();
                if count > 0 {
                    // we use .ok() instead of unwrap() to ignore the error if the host
                    // isn't reading the usb serial port fast enough and the data overflows
                    serial.write(&buf[..count]).ok();
                }
            }
        });
    }

    #[task(binds = OTG_FS, shared = [usb_dev, shell, shell_status, dfu, led_cmd, storage, power_device, ctl_pins], local=[esc_cnt:u8 = 0, to_dut_serial])]
    fn usb_task(mut cx: usb_task::Context) {
        let usb_dev = &mut cx.shared.usb_dev;
        let shell = &mut cx.shared.shell;
        let shell_status = &mut cx.shared.shell_status;
        let dfu = &mut cx.shared.dfu;
        let led_cmd = &mut cx.shared.led_cmd;
        let storage = &mut cx.shared.storage;
        let powerdev = &mut cx.shared.power_device;
        let to_dut_serial = cx.local.to_dut_serial;
        let esc_cnt = cx.local.esc_cnt;
        let ctl_pins = &mut cx.shared.ctl_pins;

        (usb_dev, dfu, shell, shell_status, led_cmd, storage, powerdev, ctl_pins).lock(|usb_dev, dfu, shell, shell_status, led_cmd, storage, powerdev, ctl_pins| {
            let serial1 = shell.get_serial_mut();

            if !usb_dev.poll(&mut [serial1, dfu]) {
                return;
            }
            let available_to_dut = to_dut_serial.capacity()-to_dut_serial.len();

            let mut send_to_dut = |buf: &[u8]|{
                for b in buf {
                    to_dut_serial.enqueue(*b).ok();
                }
                return
            };

            if shell_status.console_mode {
                // if in console mode, send all data to the DUT, only read from the USB serial port as much as we can send to the DUT
                let mut buf = [0u8; DUT_TX_BUF_SIZE];
                match serial1.read(&mut buf[..available_to_dut]) {
                    Ok(count) => {
                        send_to_dut(&buf[..count]);

                        for c in &buf[..count] {
                            if *c == 0x02 { // CTRL+B
                                *esc_cnt = *esc_cnt + 1;
                                if *esc_cnt == 5 {
                                    shell_status.console_mode = false;
                                    shell.write_str("\r\nExiting console mode\r\n").ok();
                                    shell.write_str(shell::SHELL_PROMPT).ok();
                                    *esc_cnt = 0;
                                }
                            } else {
                                *esc_cnt = 0;
                            }
                        }
                    },
                    Err(_e) => {
                    }
                }
            } else {
                shell::handle_shell_commands(shell, shell_status, led_cmd, storage, powerdev, ctl_pins, &mut send_to_dut);
            }
        });
    }

    #[task(binds = TIM2, shared=[timer, dfu,  led_rx, led_tx, led_cmd, adc_dma_transfer])]
    fn periodic_10ms(mut ctx: periodic_10ms::Context) {

        ctx.shared.dfu.lock(|dfu| dfu.tick(10));

        // clear all leds set in other tasts
        ctx.shared.led_rx.lock(|led_rx| led_rx.set_high());
        ctx.shared.led_tx.lock(|led_tx| led_tx.set_high());
        ctx.shared.led_cmd.lock(|led_cmd| led_cmd.set_high());

        ctx.shared.adc_dma_transfer.lock(|transfer| {
            transfer.start(|adc| {
                adc.start_conversion();
            });
        });

        ctx.shared
            .timer
            .lock(|tim| tim.clear_interrupt(timer::Event::Update));
    }

    const MAVG_COUNT : usize = 100;

    #[task(binds = DMA2_STREAM0, shared=[adc_dma_transfer, pw_a, pw_v, pw_w], local=[
        adc_buffer,
        mavg_v_i:usize = 0,
        mavg_v: [f32; MAVG_COUNT] = [0.0; MAVG_COUNT],
        mavg_a_i:usize = 0,
        mavg_a: [f32; MAVG_COUNT] = [0.0; MAVG_COUNT],
        ])]
    fn adc_dma(mut cx:adc_dma::Context){
        let adc_dma_transfer = &mut cx.shared.adc_dma_transfer;
        let adc_buffer = &mut cx.local.adc_buffer;

        let mavg_v = &mut cx.local.mavg_v;
        let mavg_v_i = &mut cx.local.mavg_v_i;
        let mavg_a = &mut cx.local.mavg_a;
        let mavg_a_i = &mut cx.local.mavg_a_i;

        let buffer = adc_dma_transfer.lock(|transfer| {
            let (buffer, _) = transfer
                               .next_transfer(adc_buffer.take().unwrap())
                               .unwrap();
            buffer
        });

        let current = buffer[0];
        let vout = buffer[1];

        // leave the previous buffer ready again for next transfer
        *cx.local.adc_buffer = Some(buffer);

        let current_V = (current as f32 - 2048.0) * 3.3 / 4096.0;
        let current_A = -current_V / 0.264;

        // we get vout from the voltage divider, in 12 bits, 3.3V is 4096
        let vout_sense_V = (vout as f32) * 3.3 / 4096.0;
        // we do the reverse calculation to figure out the input voltage
        let R8 = 2400.0; // R8 is the top resistor in the voltage divider
        let R9 = 470.0; // R9 is the bottom resistor in the voltage divider
        let vin = vout_sense_V * (R8 + R9) / R9;

        // moving average filter for vin
        mavg_v[**mavg_v_i] = vin;
        **mavg_v_i = (**mavg_v_i + 1) % MAVG_COUNT;
        let mut sum = 0.0;
        // not the most efficient implementation to be seen, but it works
        for i in 0..MAVG_COUNT {
            sum += mavg_v[i];
        }
        let vin_avg = sum / MAVG_COUNT as f32;

        // moving average filter for A
        mavg_a[**mavg_a_i] = current_A;
        **mavg_a_i = (**mavg_a_i + 1) % MAVG_COUNT;
        sum = 0.0;
        // not the most efficient implementation to be seen, but it works
        for i in 0..MAVG_COUNT {
            sum += mavg_a[i];
        }
        let amps_avg = sum / MAVG_COUNT as f32;
        let watts_avg = amps_avg * vin_avg;

        let pw_a = &mut cx.shared.pw_a;
        let pw_v = &mut cx.shared.pw_v;
        let pw_w = &mut cx.shared.pw_w;
        (pw_a, pw_v, pw_w).lock(|pw_a, pw_v, pw_w| {
            *pw_a = amps_avg;
            *pw_v = vin_avg;
            *pw_w = watts_avg;
        });
    }

    fn write_power_trace(shell: &mut shell::ShellType, pw_a: f32, pw_v: f32, pw_w: f32) {
        write!(shell, "\x1b[0;31m{:.3}A {:.3}V {:.3}W>\x1b[0m ", pw_a, pw_v, pw_w).ok();
    }

    // Background task, runs whenever no other tasks are running
    #[idle(local=[to_dut_serial_consumer, usart_tx], shared=[led_tx, shell_status])]
    fn idle(mut ctx: idle::Context) -> ! {
        // the source of this queue is the send command from the shell
        let to_dut_serial_consumer = &mut ctx.local.to_dut_serial_consumer;
        let shell_status = &mut ctx.shared.shell_status;

        loop {
            // Go to sleep, wake up on interrupt
            let mut escaped = false;
            cortex_m::asm::wfi();

            // Is there any data to be sent to the device under test over USART?
            if to_dut_serial_consumer.len() == 0 {
                continue;
            }
            let should_escape = shell_status.lock(|shell_status| !shell_status.console_mode);
            loop {
                match to_dut_serial_consumer.dequeue() {
                    Some(c) => {
                        // in console mode we should not handle escape characters.
                        // this would be arguably better implemented in the shell send function
                        // but this allows for the \w wait command to delay and not block other
                        // tasts
                        let mut final_c:u8 = c;
                        if should_escape {
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
                        }

                        let usart_tx = &mut ctx.local.usart_tx;
                        let led_tx = &mut ctx.shared.led_tx;
                        led_tx.lock(|led_tx| led_tx.set_low());

                        loop {
                            if usart_tx.is_tx_empty() {
                                    break;
                            }
                        }

                        usart_tx.write(final_c).ok();

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
