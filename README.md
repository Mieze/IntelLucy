## IntelLucy
IntelLucy is an open source driver for the Intel X500 family of 10GBit Ethernet controllers.

For support requests please refer to IntelLucy's project homepage at insanelymac.com
https://www.insanelymac.com/forum/topic/359009-intellucy-for-the-intel-x500-family/

**Key Features of the Driver**
- Supports Intel X520, X540, X550 and 82598 Ethernet adapters.
- Support for multisegment packets relieving the network stack of unnecessary copy operations when assembling packets for transmission.
- No-copy receive and transmit. Only small packets are copied on reception because creating a copy is more efficient than allocating a new buffer.
- TCP segmentation offload with IPv4 and IPv6.
- Support for TCP/IPv4, UDP/IPv4, TCP/IPv6, UDP/IPv6 and IPv4 checksum offload.
- Large Receive Offload: TCP/IPv4 packet streams are identified by the NIC and reassembled to large packets for handover to the network stack eliminating the overhead of packet header processing.
- Supports jumbo frames up to 9000 bytes (strongly recommended for 10 GBit/s operation).
- Designed to work with High Sierra and above.
- Supports Wake on LAN.
- Supports hardware VLAN tagging.
- Support for Energy Efficient Ethernet (EEE) is planned for future releases.
- Compatible with AppleVTD but also works without.
- No need for hardware modifications anymore (used to be required for SmalltreeIntel8259x.kext). 
- The driver is published under GPLv2.
 
**Current Status**
- The driver has been successfully tested with Catalina, Monterey, Ventura and Sonoma.
- Performance is better than com.apple.DriverKit-AppleEthernetIXGBE.
- X520 adapters have been successfully tested with passive DAC cables, 10GBase-SR and 1000Base-T SFP+ modules. SFP+ modules with 10GBase-T ports are unsupported by X520 adapters as they exceed the power limits of the SFP+ slots.
- Support for X540 adapters have has been improved and tested in version 1.0.3. Both ports are working simultaneously.
- Manual medium slecetion has been fixed for X540 and X550 in version 1.0.4.
- VLAN support has been fixed and tested in version 1.0.0.
- Wake on LAN has been fixed and confirmed to work in version 1.0.4.

**Tested Configurations**
- X520-DA1 with the following media:
  - Passive DAC cable: working
  - 10GBase-SR SFP+ modules: working
  - 1000Base-T SFP modules: working
- X540-TA1: working
- X540-TA2: both ports working
- X550-T1: working
 
**Known Issues**
- Please keep in mind that X520 adapters don't support 10GBase-T SFP+ modules because the module's power requirement (approximately 3W) exceeds the adapter's limits. Installing a 10GBase-T SFP+ module in a X520 adapter anyway may damage the device.
- Manual medium selection is working on X540 and X550 since version 1.0.4.
- Try to avoid media with flow control enabled as it might cause repeated connection drops due to transmitter hangs and serious performance issues.
- For WoL to work you need 4 things:
  - An adapter which supports WoL (only a few X520 adapters support WoL).
  - A mainboard which supports WoL from an add-in card.
  - WoL must be enabled in the UEFI setup.
  - Proper ACPI tables for MacOS to support wakeup.
    
**Installation**
- Use OpenCore to inject the driver:
![image](https://github.com/Mieze/IntelLucy/assets/4229650/247aec7d-200b-450f-b745-fb84b0de273f)
