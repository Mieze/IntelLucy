## IntelLucy
IntelLucy is an open source driver for the Intel X500 family of 10 GBit Ethernet controllers.
 
**Key Features of the Driver**
- Supports Intel X520, X540, X550 and 82598 Ethernet adapters.
- Support for multisegment packets relieving the network stack of unnecessary copy operations when assembling packets for transmission.
- No-copy receive and transmit. Only small packets are copied on reception because creating a copy is more efficient than allocating a new buffer.
- TCP segmentation offload with IPv4 and IPv6.
- Support for TCP/IPv4, UDP/IPv4, TCP/IPv6, UDP/IPv6 and IPv4 checksum offload.
- Large Receive Offload: TCP/IPv4 packet streams are identified by the NIC and reassembled to large packets for handover to the network stack eliminating the overhead of packet header processing.
- Supports jumbo frames up to 9000 bytes (strongly recommended for 10 GBit/s operation).
- Designed to work with High Sierra and above.
- Supports Wake on LAN (untested).
- Supports hardware VLAN tagging.
- Support for Energy Efficient Ethernet (EEE) is planned for future releaes.
- Compatible with AppleVTD but also works without.
- No need for hardware modifications anymore (used to be required for SmalltreeIntel8259x.kext). 
- The driver is published under GPLv2.
 
**Current Status**
- The driver has been successfully tested with Catalina, Monterey, Ventura and Sonoma.
- Performance is better than com.apple.DriverKit-AppleEthernetIXGBE.
- X520 adapters have been successfully tested with passive DAC cables, 10GBase-SR and 1000Base-T SFP+ modules. SFP+ modules with 10GBase-T ports are unsupported by X520 adapters as they exceed the power limits of the SFP+ slots.
- Support for X540 adapters have has been improved and tested in version 1.0.3. Both ports are working simultaneously.
- VLAN support has been fixed and tested in version 1.0.0.
- Wake on LAN is still untested as I don't have an adapter which supports WoL.
 
**Known Issues**
- Manual media selection doesn't work. The link is always established with the highest data rate both link partners support, i.e. 10 Gbit/s of both endpoints support it, 1 Gbit/s if the link partner is a Gigabit device, etc. As DAC cables and fibre optics usually are fixed-speed devices, only RJ-45 ports are affected of this limitation. As this is the specified behavior of the underlying Linux driver, a reqrite of the low level code will be required and is planned ASAP.   
- Try to avoid media with flow control enabled as it might cause repeated connection drops due to transmitter hangs and serious performance issues.
- For WoL to work you need 4 things:
  - An adapter which supports WoL (only a few X520 adapters support it).
  - A mainboard which supports WoL from an add-in card.
  - WoL must be enabled in the UEFI setup.
  - Proper ACPI tables for MacOS to support wakeup.

**Installation**
- Use OpenCore to inject the driver:
![image](https://github.com/Mieze/IntelLucy/assets/4229650/247aec7d-200b-450f-b745-fb84b0de273f)
