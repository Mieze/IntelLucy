## IntelLucy
IntelLucy is an open source driver for the Intel X500 family of 10 GBit Ethernet controllers.
 
**Key Features of the Driver**
- Supports Intel X520, X540 and X550 Ethernet adapters.
- Support for multisegment packets relieving the network stack of unnecessary copy operations when assembling packets for transmission.
- No-copy receive and transmit. Only small packets are copied on reception because creating a copy is more efficient than allocating a new buffer.
- TCP segmentation offload with IPv4 and IPv6.
- Support for TCP/IPv4, UDP/IPv4, TCP/IPv6 and UDP/IPv6 checksum offload.
- Large Receive Offload: TCP/IPv4 packet streams are identified by the NIC and reassembled to large packets for handover to the network stack eliminating the overhead of packet header processing.
- Supports jumbo frames up to 9000 bytes (strongly recommended for 10 GBit/s operation).
- Designed to work with High Sierra and above.
- Supports Wake on LAN (untested).
- Supports VLAN (untested).
- Support for Energy Efficient Ethernet (EEE) is planned for future releaes.
- Support for the Intel 82598 controller is also work in progress.
- No need for hardware modifications anymore (used to be required for Smalltree8259x.kext). 
- The driver is published under GPLv2.
 
**Current Status**
- The driver It has been successfully tested with Catalina, Monterey, Ventura and Sonoma.
 
**Known Issues**
- Try to avoid media with flow control enabled as it might cause repeated connection drops due to transmitter hangs and serious performance issues.
- For WoL to work you need 4 things:
  - An adapter which supports WoL (only a few X520 adapters support WoL).
  - A mainboard which supports WoL from an add-in card.
  - WoL must be enabled in the UEFI setup.
  - Proper ACPI tables for MacOS to support wakeup.

**Installation**
- Use OpenCore to inject the driver:
![image](https://github.com/Mieze/IntelLucy/assets/4229650/247aec7d-200b-450f-b745-fb84b0de273f)
