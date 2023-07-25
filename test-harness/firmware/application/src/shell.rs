use core::str;
use embedded_hal::digital::v2::OutputPin;
use core::fmt::Write;

use arrayvec::ArrayString;

use crate::ctlpins::{PinState, CTLPinsTrait};
use crate::{usbserial::*, ctlpins::CTLPins};
use crate::storage::StorageSwitchTrait;
use ushell::{
    autocomplete::StaticAutocomplete, history::LRUHistory, Input as ushell_input,
    ShellError as ushell_error, UShell,
};
const N_COMMANDS: usize = 9;
const COMMANDS: [&str; 9] = ["help", "power", "storage", "send", "reset", "set", "monitor", "power", "console"];
pub type ShellType = UShell<USBSerialType, StaticAutocomplete<N_COMMANDS>, LRUHistory<128, 4>, 128>;
pub struct ShellStatus {
    pub monitor_enabled: bool,
    pub meter_enabled: bool,
    pub console_mode: bool,
}

pub const SHELL_PROMPT: &str = "#> ";
pub const CR: &str = "\r\n";

pub const HELP: &str = "\r\n\
        about               : print information about this device\r\n\
        help                : print this help\r\n\
        meter read|monitor  : read power consumption, monitor will continue to read until CTRL+C is pressed\r\n\
        monitor on|off      : enable or disable the serial console monitor in this terminal\r\n\
        console             : enter into serial console mode, exit with CTRL+A 5 times\r\n\
        power on|off        : power on or off the DUT\r\n\
        send string         : send string to the DUT\r\n\
        set r|a|b|c|d l|h|z : set RESET, CTL_A,B,C or D to low, high or high impedance\r\n\
        storage dut|host|off: connect storage to DUT, host or disconnect\r\n\
        ";

pub fn new(serial:USBSerialType) -> ShellType {
    let autocomplete = StaticAutocomplete(COMMANDS);
    let history = LRUHistory::default();
    let shell: ShellType = UShell::new(serial, autocomplete, history);
    shell
}

pub fn handle_shell_commands<L, S, P>(shell: &mut ShellType, shell_status: &mut ShellStatus, led_cmd: &mut L, storage: &mut S, ctl_pins:&mut CTLPins<P>, send_to_dut: &mut dyn FnMut(&[u8])) 
where
    L: OutputPin,
    S: StorageSwitchTrait,
    P: OutputPin,
{
    loop {
        let mut response = ArrayString::<128>::new();
        write!(response, "{0:}", CR).ok();

        let result = shell.poll();

        match result {
            Ok(Some(ushell_input::Command((cmd, args)))) => {
                led_cmd.set_low().ok();
                match cmd {
                        "help" =>    { shell.write_str(HELP).ok(); }
                        "clear" =>   { shell.clear().ok(); }
                        "console" => { handle_console_cmd(&mut response, args, shell_status); }
                        "monitor" => { handle_monitor_cmd(&mut response, args, shell_status); }
                        "meter" =>   { handle_meter_cmd(&mut response, args, shell_status); }
                        "storage" => { handle_storage_cmd(&mut response, args, storage); }
                        "power" =>   { handle_power_cmd(&mut response, args, ctl_pins); }
                        "send" =>    { handle_send_cmd(&mut response, args, send_to_dut); }
                        "set" =>     { handle_set_cmd(&mut response, args, ctl_pins); }
                        "status" =>  { handle_status_cmd(&mut response, args, shell_status); }
                        "" => {}
                        _ => {
                            write!(shell, "{0:}unsupported command{0:}", CR).ok();
                        }
                }
                // If response was added complete with an additional CR
                if response.len() > 2 {
                    write!(response, "{0:}", CR).ok();
                }
                // if console mode has been entered we should not print the SHELL PROMPT again
                if !shell_status.console_mode {
                    write!(response, "{}", SHELL_PROMPT).ok();
                }
                shell.write_str(&response).ok();

            }
            Err(ushell_error::WouldBlock) => break,
            _ => {}
        }
    }
}

fn handle_power_cmd<B, C>(response:&mut B, args: &str, ctlpins: &mut C)
where
    C: CTLPinsTrait,
    B: Write
 {
    if args == "on" {
        ctlpins.power_on();
        write!(response, "Device powered on").ok();
    } else if args == "off" {
        ctlpins.power_off();
        write!(response, "Device powered off").ok();
    } else {
        write!(response, "usage: power on|off").ok();
    }
}

fn handle_send_cmd<B>(response:&mut B, args: &str, send_to_dut: &mut dyn FnMut(&[u8]))
where
    B: Write
 {
    if args.len() > 0 {
        send_to_dut(args.as_bytes());
    } else {
        write!(response, "usage: send string").ok();
    }
}

fn handle_storage_cmd<B,S>(response:&mut B, args: &str, storage: &mut S)
where
    S: StorageSwitchTrait,
    B: Write
 {
    if args == "dut" {
        storage.connect_to_dut();
        write!(response, "storage connected to device under test").ok();
    } else if args == "host" {
        storage.connect_to_host();
        write!(response, "storage connected to host").ok();
    } else if args == "off" {
        storage.power_off();
        write!(response, "storage disconnected").ok();
    } else {
        write!(response, "usage: storage dut|host|off").ok();
    }
}

fn handle_meter_cmd<B>(response:&mut B, args: &str, shell_status: &mut ShellStatus)
where
    B: Write
 {
    if args == "monitor" {
        shell_status.meter_enabled = true;
        write!(response, "Power meter monitoring enabled").ok();
    } else if args == "read" {
        shell_status.meter_enabled = false;
        write!(response, "10 mW").ok();
    } else if args == "off" {
        shell_status.meter_enabled = false;
        write!(response, "Power monitor disabled").ok();
    } else {
        write!(response, "usage: meter monitor|read|off").ok();
    }
}

fn handle_monitor_cmd<B>(response:&mut B, args: &str, shell_status: &mut ShellStatus)
where
    B: Write
 {
    if args == "on" {
        shell_status.monitor_enabled = true;
        write!(response, "Monitor enabled").ok();
    } else if args == "off" {
        shell_status.monitor_enabled = false;
        write!(response, "Monitor disabled").ok();
    } else {
        write!(response, "usage: monitor on|off").ok();
    }
}

fn handle_console_cmd<B>(response:&mut B, args: &str, shell_status: &mut ShellStatus)
where
    B: Write
 {
    if args =="" {
        shell_status.console_mode = true;
        write!(response, "Entering console mode, type CTRL+B 5 times to exit").ok();
    } else {
        write!(response, "usage: console").ok();
    }
}

fn handle_set_cmd<B, C>(response:&mut B, args: &str, ctl_pins:&mut C)
where
    B: Write,
    C: CTLPinsTrait

 {

    if args.len() == 3 && args.as_bytes()[1] == ' ' as u8{
        let mut chars = args.chars();
        let ctl     = chars.next().unwrap();
        let _space  = chars.next().unwrap();
        let val     = chars.next().unwrap();

        if ctl != 'r' && ctl != 'a' && ctl != 'b' && ctl != 'c' && ctl != 'd' {
            write_set_usage(response);
            return;
        }

        if val != 'l' && val != 'h' && val != 'z' {
            write_set_usage(response);
            return;
        }
        // TODO: really set the pins
        let ctl_str = match ctl {
            'r' => "/RESET",
            'a' => "CTL_A",
            'b' => "CTL_B",
            'c' => "CTL_C",
            'd' => "CTL_D",
            _ => "",
        };

        let val_str = match val {
            'l' => "LOW",
            'h' => "HIGH",
            'z' => "HIGH IMPEDANCE",
            _ => "",
        };

        let ps = match val {
            'l' => PinState::Low,
            'h' => PinState::High,
            'z' => PinState::Floating,
            _ => PinState::Floating,
        };

        match ctl {
            'r' => ctl_pins.set_reset(ps),
            'a' => ctl_pins.set_ctl_a(ps),
            'b' => ctl_pins.set_ctl_b(ps),
            'c' => ctl_pins.set_ctl_c(ps),
            'd' => ctl_pins.set_ctl_d(ps),
            _ => {},
        };

        write!(response, "Set {} to {}", ctl_str, val_str).ok();
    } else {
        write_set_usage(response)
    }
}

fn write_set_usage<B>(response:&mut B)
where
    B: Write
 {
    write!(response, "usage: set r|a|b|c|d l|h|z").ok();
}

fn handle_status_cmd<B>(response:&mut B, args: &str, shell_status: &mut ShellStatus)
where
    B: Write
 {
    if args =="" {
        write!(response, "Monitor: {}, Meter: {}, Console: {}", shell_status.monitor_enabled, shell_status.meter_enabled, shell_status.console_mode).ok();
    } else {
        write!(response, "usage: status").ok();
    }
}