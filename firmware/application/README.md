# STM32F103C8 DFU Enabled application

Please see the application section on the [main page](../../#application) for more details.

We implement the DFU runtime using the [usbd-dfu-rt](https://github.com/jedrzejboczar/usbd-dfu-rt)
Rust library from [jedrzejboczar](https://github.com/jedrzejboczar).

This folder contains a Makefile, the sources and an example firmware.metadata.xml.

The Makefile will help you build .cab files to work with fwupd.

```
$ make

cargo build --release
   Compiling typenum v1.16.0
 ...
   Compiling example-stm32f103c8 v0.1.0 (/home/majopela/firmware/et/firmware-on-the-edge/firmware-examples/stm32f103/application)
   Compiling usbd-serial v0.1.1
 ..
    Finished release [optimized] target(s) in 13.44s
...
arm-none-eabi-objcopy -Obinary target/thumbv7m-none-eabi/release/example-stm32f103c8 example.bin
appstream-util validate-relax firmware.metainfo.xml
firmware.metainfo.xml: OK
gcab --create --nopath stm32f103-example-vcp-2.1.cab example.bin firmware.metainfo.xml
```

The metadata xml contains information about your peripheral, and how fwupd can identify it.
```xml
<?xml version="1.0" encoding="UTF-8"?>
<component type="firmware">
  <id>org.fwupd.myproduct.firmware</id>
  <name>An example peripheral based on STM32F103C8</name>
  <summary>Actually a device that exposes a VCP and not much more</summary>
  <description>
    <p>
      Description of the firmware.
    </p>
  </description>
  <provides>
    <firmware type="flashed">e7336917-f91c-5583-a578-26336797863a</firmware> <!-- USB\VID_2B23&PID_E011 -->
    <firmware type="flashed">bdfdb394-cfb2-5558-b894-3c14e048e6e3</firmware> <!-- USB\VID_2B23&PID_E011&REV_0001 -->
  </provides>
  <url type="homepage">https://github.com/redhat-et/firmware-on-the-edge/tree/main/firmware-examples/stm32f103</url>
  <metadata_license>CC0-1.0</metadata_license>
  <project_license>MIT</project_license>
  <developer_name>Red Hat Inc.</developer_name>
  <releases>
    <release version="2.1" date="2023-02-08" urgency="medium">
      <checksum target="content" filename="example.bin" />
      <description>
        <p>Makes your device work</p>
      </description>
    </release>
  </releases>
</component>
```

Device GUIDs can be found by running:

```
$ fwupdmgr get-devices
└─A Custom Peripheral:
      Device ID:          de7b2daf304343aa2df33cac28f22aed48aff853
      Current version:    2.0
      Vendor:             Red Hat Inc. (USB:0x2B23)
      Serial Number:      23934513
      GUIDs:              e7336917-f91c-5583-a578-26336797863a ← USB\VID_2B23&PID_E011
                          97917aaf-73df-514b-8119-2a15d53805a5 ← USB\VID_2B23&PID_E011&REV_0200
      Device Flags:       • Updatable
```


The device can be flashed manually for testing with:

```
$ sudo fwupdtool install stm32f103-example-vcp-0.2.cab
```

Or can be installed into the system with:

```
sudo cp stm32f103-example-vcp-2.1.cab /usr/share/fwupd/remotes.d/vendor/firmware/
sudo fwupdmgr enable-remote vendor-directory
```

Listing with get-devices will say "Supported by remote server"

```
$ fwupdmgr get-devices
└─A Custom Peripheral:
      Device ID:          de7b2daf304343aa2df33cac28f22aed48aff853
      Current version:    2.0
      Vendor:             Red Hat Inc. (USB:0x2B23)
      Serial Number:      23934513
      Update State:       Success
      GUIDs:              e7336917-f91c-5583-a578-26336797863a ← USB\VID_2B23&PID_E011
                          97917aaf-73df-514b-8119-2a15d53805a5 ← USB\VID_2B23&PID_E011&REV_0200
      Device Flags:       • Updatable
                          • Supported on remote server
```

At that point you could update your peripheral any time with:

```
$ fwupdmgr update

╔══════════════════════════════════════════════════════════════════════════════╗
║ Upgrade A Custom Peripheral from 0.1 to 2.1?                                 ║
╠══════════════════════════════════════════════════════════════════════════════╣
║ Makes your device work                                                       ║
║                                                                              ║
║ A Custom Peripheral and all connected devices may not be usable while        ║
║ updating.                                                                    ║
╚══════════════════════════════════════════════════════════════════════════════╝
Perform operation? [Y|n]: y
Waiting…                 [***************************************] Less than one minute remaining…
Successfully installed firmware
```

If you try again you will see:
```
$ fwupdmgr update
Devices with the latest available firmware version:
 • A Custom Peripheral
```

The device will enumerate via usb:
```
[ 1920.546595] usb 2-2.12: new full-speed USB device number 21 using xhci_hcd
[ 1920.686301] usb 2-2.12: New USB device found, idVendor=2b23, idProduct=e011, bcdDevice= 2.01
[ 1920.686322] usb 2-2.12: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[ 1920.686324] usb 2-2.12: Product: A Custom Peripheral
[ 1920.686326] usb 2-2.12: Manufacturer: Red Hat Inc.
[ 1920.686328] usb 2-2.12: SerialNumber: 23934513
[ 1920.699189] cdc_acm 2-2.12:1.0: ttyACM2: USB ACM device
```
