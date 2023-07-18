use core::str;
use embedded_hal::digital::v2::OutputPin;

use core::fmt::Write;
use crate::usbserial::*;
use crate::storage::StorageSwitchTrait;
use ushell::{
    autocomplete::StaticAutocomplete, history::LRUHistory, Input as ushell_input,
    ShellError as ushell_error, UShell,
};

pub type ShellType = UShell<USBSerialType, StaticAutocomplete<8>, LRUHistory<128, 4>, 128>;

pub const SHELL_PROMPT: &str = "#> ";
pub const CR: &str = "\r\n";

pub const HELP: &str = "\r\n\
        about                       : print information about this device\r\n\
        ctl_[a,b,c,d] low|high|hiz  : set CTL_A to low, high or high impedance\r\n\
        help                        : print this help\r\n\
        meter [monitor]             : read power consumption, monitor will continue to read until CTRL+C is pressed\r\n\
        monitor on|off              : enable or disable the serial console monitor in this terminal\r\n\
        power on|off                : power on or off the DUT\r\n\
        reset                       : reset the DUT\r\n\
        send string                 : send string to the DUT\r\n\
        set a|b|c|d low|high|hiz    : set CTL_A,B,C or D to low, high or high impedance\r\n\
        ";

pub fn new(serial:USBSerialType) -> ShellType {
    let autocomplete = StaticAutocomplete(["help", "power", "storage", "send", "reset", "set", "monitor", "power"]);
    let history = LRUHistory::default();
    let shell: ShellType = UShell::new(serial, autocomplete, history);
    shell
}

pub fn handle_shell_commands<L, S, P>(shell: &mut ShellType, led_cmd: &mut L, storage: &mut S, powerdev: &mut P, send_to_dut: &mut dyn FnMut(&[u8])) 
where
    L: OutputPin,
    S: StorageSwitchTrait,
    P: OutputPin,
{
    loop {
        let result = shell.poll();
        match result {
            Ok(Some(ushell_input::Command((cmd, args)))) => {
                led_cmd.set_low().ok();
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
                                powerdev.set_high().ok();
                                write!(shell, "{0:}device powered on.{0:}", CR).ok();
                            } else if args == "off" {
                                powerdev.set_low().ok();
                                write!(shell, "{0:}device powered off.{0:}", CR).ok();
                            } else {
                                write!(shell, "{0:}usage: power on|off{0:}",CR).ok();
                            }
                        }
                        "send" => {   
                            send_to_dut(args.as_bytes());
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
}