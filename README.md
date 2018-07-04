# QSB-USB2Serial_US-digital
This project is used for getting streaming data from US Digital USB-to-Serial adapter.

# How to solve the problem of not knowing the names of usb devices when multiple USB devices are plugged in the same computer?

The solution is writing dev rules to assign names for individuals. As we know, every QSB-adapter has its own serial number, which you can find on the front part of the black adapter. For my case, I have two QSB0-adapters. One has the serial number of 81830, another one has the serial number of 81658. I will use this attribute (ATTRS{serial}) to assign names to this two adpaters.

First, run udevadm to have a look at usb devices details

$ udevadm info -a -n /dev/ttyUSB0

          Udevadm info starts with the device specified by the devpath and then
          walks up the chain of parent devices. It prints for every device
          found, all possible attributes in the udev rules key format.
          A rule to match, can be composed by the attributes of the device
          and the attributes from one single parent device.

            looking at device '/devices/pci0000:00/0000:00:14.0/usb1/1-8/1-8:1.0/ttyUSB0/tty/ttyUSB0':
              KERNEL=="ttyUSB0"
              SUBSYSTEM=="tty"
              DRIVER==""

            looking at parent device '/devices/pci0000:00/0000:00:14.0/usb1/1-8/1-8:1.0/ttyUSB0':
              KERNELS=="ttyUSB0"
              SUBSYSTEMS=="usb-serial"
              DRIVERS=="ftdi_sio"
              ATTRS{latency_timer}=="16"
              ATTRS{port_number}=="0"

            looking at parent device '/devices/pci0000:00/0000:00:14.0/usb1/1-8/1-8:1.0':
              KERNELS=="1-8:1.0"
              SUBSYSTEMS=="usb"
              DRIVERS=="ftdi_sio"
              ATTRS{authorized}=="1"
              ATTRS{bAlternateSetting}==" 0"
              ATTRS{bInterfaceClass}=="ff"
              ATTRS{bInterfaceNumber}=="00"
              ATTRS{bInterfaceProtocol}=="ff"
              ATTRS{bInterfaceSubClass}=="ff"
              ATTRS{bNumEndpoints}=="02"
              ATTRS{interface}=="USB <-> QSB"
              ATTRS{supports_autosuspend}=="1"

            looking at parent device '/devices/pci0000:00/0000:00:14.0/usb1/1-8':
              KERNELS=="1-8"
              SUBSYSTEMS=="usb"
              DRIVERS=="usb"
              ATTRS{authorized}=="1"
              ATTRS{avoid_reset_quirk}=="0"
              ATTRS{bConfigurationValue}=="1"
              ATTRS{bDeviceClass}=="00"
              ATTRS{bDeviceProtocol}=="00"
              ATTRS{bDeviceSubClass}=="00"
              ATTRS{bMaxPacketSize0}=="8"
              ATTRS{bMaxPower}=="500mA"
              ATTRS{bNumConfigurations}=="1"
              ATTRS{bNumInterfaces}==" 1"
              ATTRS{bcdDevice}=="0600"
              ATTRS{bmAttributes}=="80"
              ATTRS{busnum}=="1"
              ATTRS{configuration}==""
              ATTRS{devnum}=="5"
              ATTRS{devpath}=="8"
              ATTRS{idProduct}=="6001"
              ATTRS{idVendor}=="0403"
              ATTRS{ltm_capable}=="no"
              ATTRS{manufacturer}=="US Digital"
              ATTRS{maxchild}=="0"
              ATTRS{product}=="USB <-> QSB"
              ATTRS{quirks}=="0x0"
              ATTRS{removable}=="unknown"
              ATTRS{serial}=="81830"
              ATTRS{speed}=="12"
              ATTRS{urbnum}=="72190"
              ATTRS{version}==" 2.00"

            looking at parent device '/devices/pci0000:00/0000:00:14.0/usb1':
              KERNELS=="usb1"
              SUBSYSTEMS=="usb"
              DRIVERS=="usb"
              ATTRS{authorized}=="1"
              ATTRS{authorized_default}=="1"
              ATTRS{avoid_reset_quirk}=="0"
              ATTRS{bConfigurationValue}=="1"
              ATTRS{bDeviceClass}=="09"
              ATTRS{bDeviceProtocol}=="01"
              ATTRS{bDeviceSubClass}=="00"
              ATTRS{bMaxPacketSize0}=="64"
              ATTRS{bMaxPower}=="0mA"
              ATTRS{bNumConfigurations}=="1"
              ATTRS{bNumInterfaces}==" 1"
              ATTRS{bcdDevice}=="0404"
              ATTRS{bmAttributes}=="e0"
              ATTRS{busnum}=="1"
              ATTRS{configuration}==""
              ATTRS{devnum}=="1"
              ATTRS{devpath}=="0"
              ATTRS{idProduct}=="0002"
              ATTRS{idVendor}=="1d6b"
              ATTRS{interface_authorized_default}=="1"
              ATTRS{ltm_capable}=="no"
              ATTRS{manufacturer}=="Linux 4.4.0-128-generic xhci-hcd"
              ATTRS{maxchild}=="16"
              ATTRS{product}=="xHCI Host Controller"
              ATTRS{quirks}=="0x0"
              ATTRS{removable}=="unknown"
              ATTRS{serial}=="0000:00:14.0"
              ATTRS{speed}=="480"
              ATTRS{urbnum}=="129"
              ATTRS{version}==" 2.00"

            looking at parent device '/devices/pci0000:00/0000:00:14.0':
              KERNELS=="0000:00:14.0"
              SUBSYSTEMS=="pci"
              DRIVERS=="xhci_hcd"
              ATTRS{broken_parity_status}=="0"
              ATTRS{class}=="0x0c0330"
              ATTRS{consistent_dma_mask_bits}=="64"
              ATTRS{d3cold_allowed}=="1"
              ATTRS{device}=="0xa12f"
              ATTRS{dma_mask_bits}=="64"
              ATTRS{driver_override}=="(null)"
              ATTRS{enable}=="1"
              ATTRS{irq}=="120"
              ATTRS{local_cpulist}=="0-3"
              ATTRS{local_cpus}=="f"
              ATTRS{msi_bus}=="1"
              ATTRS{numa_node}=="-1"
              ATTRS{subsystem_device}=="0x8694"
              ATTRS{subsystem_vendor}=="0x1043"
              ATTRS{vendor}=="0x8086"

            looking at parent device '/devices/pci0000:00':
              KERNELS=="pci0000:00"
              SUBSYSTEMS==""
              DRIVERS==""
Then you will see one line below from the printed results.

                    ATTRS{serial}=="81830"
Go to 

add 

          #This rule file is used to assign names for qsb adapter from USdigital 

          KERNELS=="1-8",SUBSYSTEMS=="usb",ATTRS{serial}=="81830",NAME="qsb81830"
          KERNELS=="1-8",SUBSYSTEMS=="usb",ATTRS{serial}=="81658",NAME="qsb81658"
