
use stm32f4xx_hal::otg_fs::UsbBusType;
use usbd_serial::SerialPort;

pub type USBSerialType = SerialPort<'static, UsbBusType, BufferStore512, BufferStore512>;

 // Bigger USB Serial buffer
 use core::borrow::{Borrow, BorrowMut};
 pub struct BufferStore512(pub [u8; 512]);

 impl Borrow<[u8]> for BufferStore512 {
     fn borrow(&self) -> &[u8] {
         &self.0
     }
 }

 impl BorrowMut<[u8]> for BufferStore512 {
     fn borrow_mut(&mut self) -> &mut [u8] {
         &mut self.0
     }
 }

 macro_rules! new_usb_serial {
     ($usb:expr) => {
         SerialPort::new_with_store($usb, BufferStore512([0; 512]), BufferStore512([0; 512]))
     };
 }
 pub(crate) use new_usb_serial; 
