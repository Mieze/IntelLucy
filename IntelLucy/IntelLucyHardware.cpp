/* IntelLucyHardware.cpp -- IntelLucy hardware specific routines.
 *
 * Copyright (c) 2024 Laura Müller <laura-mueller@uni-duesseldorf.de>
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * Driver for Intel PCIe 10GB ethernet controllers.
 *
 * This driver is based on Intel's ixgbe driver for Linux.
 */


#include "IntelLucy.hpp"

static const u32 rsskey[10] = {
    0xda565a6d, 0xc20e5b25, 0x3d256741, 0xb08fa343, 0xcb2bcad0,
    0xb4307bae, 0xa32dcb77, 0x0cf23080, 0x3bb7426a, 0xfa01acbe
};


#pragma mark --- PCIe initialization methods ---

bool IntelLucy::initPCIConfigSpace(IOPCIDevice *provider)
{
    struct ixgbe_adapter *adapter = &adapterData;
    struct ixgbe_hw *hw = &adapter->hw;
    UInt16 pcieLinkState = 0;
    bool result = false;
    
    /* Get vendor and device info. */
    pciDeviceData.vendor = provider->extendedConfigRead16(kIOPCIConfigVendorID);
    pciDeviceData.device = provider->extendedConfigRead16(kIOPCIConfigDeviceID);
    pciDeviceData.subsystem_vendor = provider->extendedConfigRead16(kIOPCIConfigSubSystemVendorID);
    pciDeviceData.subsystem_device = provider->extendedConfigRead16(kIOPCIConfigSubSystemID);
    pciDeviceData.revision = provider->extendedConfigRead8(kIOPCIConfigRevisionID);
    
    /* Identify the chipset. */
    if (!ixgbeIdentifyChip())
        goto done;
    
    /* Get PCIe link information. */
    if (provider->extendedFindPCICapability(kIOPCIPCIExpressCapability, &pcieCapOffset)) {
        pcieLinkState = provider->extendedConfigRead16(pcieCapOffset + kIOPCIELinkStatus);
        DebugLog("PCIe link status: 0x%04x.\n", pcieLinkState);
        
        /* Get the bus information. */
        hw->bus.type = ixgbe_bus_type_pci_express;
        hw->bus.func = pciDevice->getFunctionNumber();
        
        switch (pcieLinkState & IXGBE_PCI_LINK_WIDTH) {
            case IXGBE_PCI_LINK_WIDTH_1:
                hw->bus.width = ixgbe_bus_width_pcie_x1;
                break;
                    
            case IXGBE_PCI_LINK_WIDTH_2:
                hw->bus.width = ixgbe_bus_width_pcie_x2;
                break;
                    
            case IXGBE_PCI_LINK_WIDTH_4:
                hw->bus.width = ixgbe_bus_width_pcie_x4;
                break;
                    
            case IXGBE_PCI_LINK_WIDTH_8:
                hw->bus.width = ixgbe_bus_width_pcie_x8;
                break;
                    
            default:
                hw->bus.width = ixgbe_bus_width_unknown;
                break;
        }
    }
    switch (pcieLinkState & IXGBE_PCI_LINK_SPEED) {
        case IXGBE_PCI_LINK_SPEED_2500:
            hw->bus.speed = ixgbe_bus_speed_2500;
            break;
            
        case IXGBE_PCI_LINK_SPEED_5000:
            hw->bus.speed = ixgbe_bus_speed_5000;
            break;
            
        case IXGBE_PCI_LINK_SPEED_8000:
            hw->bus.speed = ixgbe_bus_speed_8000;
            break;
            
        default:
            hw->bus.speed = ixgbe_bus_speed_unknown;
            break;
    }

    /* Enable the device. */
    ixgbeEnablePCIDevice(provider);
    
    baseMap = provider->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0, kIOMapInhibitCache);
    
    if (!baseMap) {
        IOLog("region #0 not an MMIO resource, aborting.\n");
        goto done;
    }
    baseAddr = reinterpret_cast<volatile void *>(baseMap->getVirtualAddress());
    adapterData.hw.hw_addr = (u8 __iomem *)baseAddr;
    
    result = true;
    
done:
    return result;
    
error:
    RELEASE(baseMap);
    baseMap = NULL;
    adapterData.hw.hw_addr = NULL;
    goto done;
}

void IntelLucy::initPCIPowerManagment(IOPCIDevice *provider, const struct e1000_info *ei)
{
    IOByteCount pmCapOffset;
    UInt16 pmCap;

    /* Setup power management. */
    if (provider->extendedFindPCICapability(kIOPCIPowerManagementCapability, &pmCapOffset)) {
        pmCap = provider->extendedConfigRead16(pmCapOffset + kIOPCIPMCapability);
        DebugLog("PCI power management capabilities: 0x%x.\n", pmCap);
        
        if (pmCap & (kPCIPMCPMESupportFromD3Cold | kPCIPMCPMESupportFromD3Hot)) {
            DebugLog("PME# from D3 (cold/hot) supported.\n");
        }
        pciPMCtrlOffset = pmCapOffset + kIOPCIPMControl;
    } else {
        IOLog("PCI power management unsupported.\n");
    }
    provider->enablePCIPowerManagement();
}

unsigned long IntelLucy::ixgbeGetCompletionTimeout(struct ixgbe_adapter *adapter)
{
    unsigned long tvalue = 32000ul;
    IOByteCount dc2CapOffset;
    UInt16 devctl2;
    
    if (pciDevice->extendedFindPCICapability(kIOPCIDC2Capability, &dc2CapOffset)) {
        devctl2 = pciDevice->extendedConfigRead16(dc2CapOffset + kIOPCIEDevCtl2);

        switch (devctl2 & IXGBE_PCIDEVCTRL2_TIMEO_MASK) {
            case IXGBE_PCIDEVCTRL2_17_34s:
            case IXGBE_PCIDEVCTRL2_4_8s:
                /* For now we cap the upper limit on delay to 2 seconds
                 * as we end up going up to 34 seconds of delay in worst
                 * case timeout value.
                 */
            case IXGBE_PCIDEVCTRL2_1_2s:
                tvalue = 2000000ul;    /* 2.0 s */
                break;
                
            case IXGBE_PCIDEVCTRL2_260_520ms:
                tvalue = 520000ul;    /* 520 ms */
                break;
                
            case IXGBE_PCIDEVCTRL2_65_130ms:
                tvalue = 130000ul;    /* 130 ms */
                break;
                
            case IXGBE_PCIDEVCTRL2_16_32ms:
                tvalue = 32000ul;        /* 32 ms */
                break;
                
            case IXGBE_PCIDEVCTRL2_1_2ms:
                tvalue = 2000ul;        /* 2 ms */
                break;
                
            case IXGBE_PCIDEVCTRL2_50_100us:
                tvalue = 100ul;        /* 100 us */
                break;
                
            case IXGBE_PCIDEVCTRL2_16_32ms_def:
                tvalue = 32000ul;        /* 32 ms */
                break;
                
            default:
                break;
        }
    }
    return tvalue;
}

IOReturn IntelLucy::setPowerStateWakeAction(OSObject *owner, void *arg1, void *arg2, void *arg3, void *arg4)
{
    IntelLucy *ethCtlr = OSDynamicCast(IntelLucy, owner);
    IOPCIDevice *dev;
    UInt16 val16;
    UInt8 offset;
    
    if (ethCtlr) {
        dev = ethCtlr->pciDevice;
        offset = ethCtlr->pciPMCtrlOffset;
        
        val16 = dev->extendedConfigRead16(offset);
        
        val16 &= ~(kPCIPMCSPowerStateMask | kPCIPMCSPMEStatus | kPCIPMCSPMEEnable);
        val16 |= kPCIPMCSPowerStateD0;
        
        dev->extendedConfigWrite16(offset, val16);
        
        /* Restore the PCI Command register. */
        ethCtlr->ixgbeEnablePCIDevice(dev);
    }
    return kIOReturnSuccess;
}

IOReturn IntelLucy::setPowerStateSleepAction(OSObject *owner, void *arg1, void *arg2, void *arg3, void *arg4)
{
    IntelLucy *ethCtlr = OSDynamicCast(IntelLucy, owner);
    IOPCIDevice *dev;
    UInt16 val16;
    UInt8 offset;
    
    if (ethCtlr) {
        dev = ethCtlr->pciDevice;
        offset = ethCtlr->pciPMCtrlOffset;
        
        val16 = dev->extendedConfigRead16(offset);
        
        val16 &= ~(kPCIPMCSPowerStateMask | kPCIPMCSPMEStatus | kPCIPMCSPMEEnable);
        
        if (ethCtlr->wolActive)
            val16 |= (kPCIPMCSPMEStatus | kPCIPMCSPMEEnable | kPCIPMCSPowerStateD3);
        else
            val16 |= kPCIPMCSPowerStateD3;

        dev->extendedConfigWrite16(offset, val16);
    }
    return kIOReturnSuccess;
}

inline void IntelLucy::ixgbeEnablePCIDevice(IOPCIDevice *provider)
{
    UInt16 cmdReg;
    
    cmdReg = provider->extendedConfigRead16(kIOPCIConfigCommand);
    cmdReg |= (kIOPCICommandBusMaster | kIOPCICommandMemorySpace);
    cmdReg &= ~kIOPCICommandIOSpace;
    provider->extendedConfigWrite16(kIOPCIConfigCommand, cmdReg);
}

/**
 *  ixgbeGetPCIeMSIxCountGeneric - Gets MSI-X vector count
 *  @hw: pointer to hardware structure
 *
 *  Read PCIe configuration space, and get the MSI-X vector count from
 *  the capabilities table.
 **/
UInt16 IntelLucy::ixgbeGetPCIeMSIxCountGeneric(struct ixgbe_hw *hw)
{
    u16 msix_count;
    u16 max_msix_count;
    u16 pcie_offset;

    switch (hw->mac.type) {
        case ixgbe_mac_82598EB:
            pcie_offset = IXGBE_PCIE_MSIX_82598_CAPS;
            max_msix_count = IXGBE_MAX_MSIX_VECTORS_82598;
            break;
            
        case ixgbe_mac_82599EB:
        case ixgbe_mac_X540:
        case ixgbe_mac_X550:
        case ixgbe_mac_X550EM_x:
        case ixgbe_mac_x550em_a:
            pcie_offset = IXGBE_PCIE_MSIX_82599_CAPS;
            max_msix_count = IXGBE_MAX_MSIX_VECTORS_82599;
            break;
            
        default:
            return 1;
    }
    msix_count = pciDevice->extendedConfigRead16(pcie_offset);
    
    if (ixgbe_removed(hw->hw_addr))
        msix_count = 0;
    
    msix_count &= IXGBE_PCIE_MSIX_TBL_SZ_MASK;

    /* MSI-X count is zero-based in HW */
    msix_count++;

    if (msix_count > max_msix_count)
        msix_count = max_msix_count;

    return msix_count;
}

/**
 * ixgbePCIeTimeoutPoll - Return number of times to poll for completion
 * @hw: pointer to hardware structure
 *
 * System-wide timeout range is encoded in PCIe Device Control2 register.
 *
 *  Add 10% to specified maximum and return the number of times to poll for
 *  completion timeout, in units of 100 microsec.  Never return less than
 *  800 = 80 millisec.
 **/
u32 IntelLucy::ixgbePCIeTimeoutPoll(struct ixgbe_hw *hw)
{
    s16 devctl2;
    u32 pollcnt;

    devctl2 = pciDevice->extendedConfigRead16(IXGBE_PCI_DEVICE_CONTROL2);
    devctl2 &= IXGBE_PCIDEVCTRL2_TIMEO_MASK;

    switch (devctl2) {
        case IXGBE_PCIDEVCTRL2_65_130ms:
             pollcnt = 1300;         /* 130 millisec */
            break;
            
        case IXGBE_PCIDEVCTRL2_260_520ms:
            pollcnt = 5200;         /* 520 millisec */
            break;
            
        case IXGBE_PCIDEVCTRL2_1_2s:
            pollcnt = 20000;        /* 2 sec */
            break;
            
        case IXGBE_PCIDEVCTRL2_4_8s:
            pollcnt = 80000;        /* 8 sec */
            break;
            
        case IXGBE_PCIDEVCTRL2_17_34s:
            pollcnt = 34000;        /* 34 sec */
            break;
            
        case IXGBE_PCIDEVCTRL2_50_100us:        /* 100 microsecs */
        case IXGBE_PCIDEVCTRL2_1_2ms:           /* 2 millisecs */
        case IXGBE_PCIDEVCTRL2_16_32ms:         /* 32 millisec */
        case IXGBE_PCIDEVCTRL2_16_32ms_def:     /* 32 millisec default */
        default:
            pollcnt = 800;          /* 80 millisec minimum */
            break;
    }

    /* add 10% to spec maximum */
    return (pollcnt * 11) / 10;
}

/**
 *  ixgbeDisablePCIePrimary - Disable PCI-express primary access
 *  @hw: pointer to hardware structure
 *
 *  Disables PCI-Express primary access and verifies there are no pending
 *  requests. IXGBE_ERR_PRIMARY_REQUESTS_PENDING is returned if primary disable
 *  bit hasn't caused the primary requests to be disabled, else 0
 *  is returned signifying primary requests disabled.
 **/
SInt32 IntelLucy::ixgbeDisablePCIePrimary(struct ixgbe_hw *hw)
{
    u32 i, poll;
    u16 value;

    /* Always set this bit to ensure any future transactions are blocked */
    IXGBE_WRITE_REG(hw, IXGBE_CTRL, IXGBE_CTRL_GIO_DIS);

    /* Poll for bit to read as set */
    for (i = 0; i < IXGBE_PCI_PRIMARY_DISABLE_TIMEOUT; i++) {
        if (IXGBE_READ_REG(hw, IXGBE_CTRL) & IXGBE_CTRL_GIO_DIS)
            break;
        usleep_range(100, 120);
    }
    if (i >= IXGBE_PCI_PRIMARY_DISABLE_TIMEOUT) {
        DebugLog("GIO disable did not set - requesting resets\n");
        goto gio_disable_fail;
    }

    /* Exit if primary requests are blocked */
    if (!(IXGBE_READ_REG(hw, IXGBE_STATUS) & IXGBE_STATUS_GIO) ||
        ixgbe_removed(hw->hw_addr))
        return 0;

    /* Poll for primary request bit to clear */
    for (i = 0; i < IXGBE_PCI_PRIMARY_DISABLE_TIMEOUT; i++) {
        udelay(100);
        
        if (!(IXGBE_READ_REG(hw, IXGBE_STATUS) & IXGBE_STATUS_GIO))
            return 0;
    }

    /*
     * Two consecutive resets are required via CTRL.RST per datasheet
     * 5.2.5.3.2 Primary Disable.  We set a flag to inform the reset routine
     * of this need.  The first reset prevents new primary requests from
     * being issued by our device.  We then must wait 1usec or more for any
     * remaining completions from the PCIe bus to trickle in, and then reset
     * again to clear out any effects they may have had on our device.
     */
    DebugLog("GIO Primary Disable bit didn't clear - requesting resets\n");
    
gio_disable_fail:
    hw->mac.flags |= IXGBE_FLAGS_DOUBLE_RESET_REQUIRED;

    if (hw->mac.type >= ixgbe_mac_X550)
        return 0;

    /*
     * Before proceeding, make sure that the PCIe block does not have
     * transactions pending.
     */
    poll = ixgbePCIeTimeoutPoll(hw);
    
    for (i = 0; i < poll; i++) {
        udelay(100);
        value = pciDevice->extendedConfigRead16(IXGBE_PCI_DEVICE_STATUS);
        
        if (ixgbe_removed(hw->hw_addr))
            return 0;
        
        if (!(value & IXGBE_PCI_DEVICE_STATUS_TRANSACTION_PENDING))
            return 0;
    }
    DebugLog("PCIe transaction pending bit also did not clear.\n");
    
    return IXGBE_ERR_PRIMARY_REQUESTS_PENDING;
}

#pragma mark --- main initialization methods ---

bool IntelLucy::ixgbeStart(const struct intelDevice *devTable)
{
    struct ixgbe_adapter *adapter = &adapterData;
    struct ixgbe_hw *hw = &adapter->hw;
    const struct ixgbe_info *ii = devTable[chip].deviceInfo;
    UInt32 eec;
    int err;
    u8 part_str[IXGBE_PBANUM_LENGTH];

    bool result = false;

    /* Setup hw api */
    hw->mac.ops   = *ii->mac_ops;
    hw->mac.type  = ii->mac;
    hw->mvals     = ii->mvals;
    
    if (ii->link_ops)
        hw->link.ops  = *ii->link_ops;

    /* EEPROM */
    hw->eeprom.ops = *ii->eeprom_ops;
    eec = IXGBE_READ_REG(hw, IXGBE_EEC(hw));

    if (ixgbe_removed(hw->hw_addr))
        goto done;
    
    /* If EEPROM is valid (bit 8 = 1), use default otherwise use bit bang */
    if (!(eec & BIT(8)))
        hw->eeprom.ops.read = &ixgbe_read_eeprom_bit_bang_generic;

    /* MII bus */
    adapterData.mii_bus = &miiBus;
    
    /* PHY */
    hw->phy.ops = *ii->phy_ops;
    hw->phy.sfp_type = ixgbe_sfp_type_unknown;
    /* ixgbe_identify_phy_generic will set prtad and mmds properly */
    hw->phy.mdio.prtad = MDIO_PRTAD_NONE;
    hw->phy.mdio.mmds = 0;
    hw->phy.mdio.mode_support = MDIO_SUPPORTS_C45 | MDIO_EMULATE_C22;
    hw->phy.mdio.dev = &adapterData;
    hw->phy.mdio.mdio_read = ixgbe_mdio_read;
    hw->phy.mdio.mdio_write = ixgbe_mdio_write;

    /* setup the private structure */
    if (!ixgbeSwInit(adapter, ii))
        goto done;
    
    if (adapterData.hw.mac.type == ixgbe_mac_82599EB)
        adapterData.flags2 |= IXGBE_FLAG2_AUTO_DISABLE_VF;

    /* Make sure the SWFW semaphore is in a valid state */
    if (hw->mac.ops.init_swfw_sync)
        hw->mac.ops.init_swfw_sync(hw);

    /* Make it possible the adapter to be woken up via WOL */
    switch (adapterData.hw.mac.type) {
        case ixgbe_mac_82599EB:
        case ixgbe_mac_X540:
        case ixgbe_mac_X550:
        case ixgbe_mac_X550EM_x:
        case ixgbe_mac_x550em_a:
            IXGBE_WRITE_REG(hw, IXGBE_WUS, ~0);
            break;
            
        default:
            break;
    }
    /*
     * If there is a fan on this device and it has failed log the
     * failure.
     */
    if (adapter->flags & IXGBE_FLAG_FAN_FAIL_CAPABLE) {
        u32 esdp = IXGBE_READ_REG(hw, IXGBE_ESDP);
        
        if (esdp & IXGBE_ESDP_SDP1)
            IOLog("Fan has stopped, replace the adapter.\n");
    }

    if (allowUnsupportedSFP)
        hw->allow_unsupported_sfp = allowUnsupportedSFP;

    /* reset_hw fills in the perm_addr as well */
    hw->phy.reset_if_overtemp = true;
    
    switch (adapterData.hw.mac.type) {
        case ixgbe_mac_82599EB:
            err = ixgbeResetHw82599(hw);
            break;
            
        case ixgbe_mac_X540:
        case ixgbe_mac_X550:
            err = ixgbeResetHwX540(hw);
            break;
            
        case ixgbe_mac_X550EM_x:
        case ixgbe_mac_x550em_a:
            err = ixgbeResetHwX550em(hw);
            break;
            
        default:
            err = hw->mac.ops.reset_hw(hw);
            break;
    }
    hw->phy.reset_if_overtemp = false;
    ixgbe_set_eee_capable(adapter);
    
    if (err == IXGBE_ERR_SFP_NOT_PRESENT) {
        err = 0;
        IOLog("SFP not present: %d\n", err);
    } else if (err == IXGBE_ERR_SFP_NOT_SUPPORTED) {
        IOLog("Failed to load because an unsupported SFP+ or QSFP module type was detected. Reload the driver after installing a supported module.\n");
        goto done;
    } else if (err) {
        IOLog("HW Init failed: %d\n", err);
        goto done;
    }
    if (ixgbe_check_fw_error(adapter)) {
        err = -EIO;
        IOLog("ixgbe_check_fw_error() failed: %d\n", err);
        goto err_sw_init;
    }
    /* make sure the EEPROM is good */
    if (hw->eeprom.ops.validate_checksum(hw, NULL) < 0) {
        err = -EIO;
        IOLog("The EEPROM Checksum Is Not Valid: %d\n", err);

        goto err_sw_init;
    }
    if (!is_valid_ether_addr(hw->mac.perm_addr)) {
        err = -EIO;
        IOLog("Invalid MAC address: %d\n", err);

        goto err_sw_init;
    }
    /* Set hw->mac.addr to permanent MAC address */
    memcpy(hw->mac.addr, hw->mac.perm_addr, kIOEthernetAddressSize);
    //ixgbe_mac_set_default_filter(adapter);
    
    if (ixgbe_removed(hw->hw_addr)) {
        err = -EIO;
        IOLog("Adapter removed: %d\n", err);
        goto err_sw_init;
    }
    set_bit(__IXGBE_SERVICE_INITED, &adapter->state);
    clear_bit(__IXGBE_SERVICE_SCHED, &adapter->state);

    /* Disable DCB. */
    adapter->flags &= ~IXGBE_FLAG_DCB_ENABLED;
    adapter->temp_dcb_cfg.pfc_mode_enable = false;
    adapter->dcb_cfg.pfc_mode_enable = false;
    adapter->hw_tcs = 0;
    adapter->dcb_cfg.num_tcs.pg_tcs = 1;
    adapter->dcb_cfg.num_tcs.pfc_tcs = 1;

    /* Disable SR-IOV support */
    DebugLog("Disabling SR-IOV support\n");
    //ixgbe_disable_sriov(adapter);
    
    set_bit(__IXGBE_DOWN, &adapter->state);

    /* WOL not supported for all devices */
    adapter->wol = 0;
    hw->eeprom.ops.read(hw, 0x2c, &adapter->eeprom_cap);
    hw->wol_enabled = ixgbe_wol_supported(adapter, pciDeviceData.device,
                                          pciDeviceData.subsystem_device);
    if (hw->wol_enabled)
        adapter->wol = IXGBE_WUFC_MAG;

    this->setProperty("Wake on LAN support", hw->wol_enabled);

    /* save off EEPROM version number */
    ixgbe_set_fw_version(adapter);
    
    err = ixgbe_read_pba_string_generic(hw, part_str, sizeof(part_str));
    
    if (err)
        IOLog("Unknown part.");
        
    if (ixgbe_is_sfp(hw) && hw->phy.sfp_type != ixgbe_sfp_type_not_present)
        IOLog("MAC: %d, PHY: %d, SFP+: %d, PBA No: %s\n",
               hw->mac.type, hw->phy.type, hw->phy.sfp_type,
               part_str);
    else
        IOLog("MAC: %d, PHY: %d, PBA No: %s\n",
               hw->mac.type, hw->phy.type, part_str);

    /* reset the hardware with the new settings */
    err = hw->mac.ops.start_hw(hw);

    if (err == IXGBE_ERR_EEPROM_VERSION)
        IOLog("Warning: EEPROM version error.");

    /* power down the optics for 82599 SFP+ fiber */
    if (hw->mac.ops.disable_tx_laser)
        hw->mac.ops.disable_tx_laser(hw);

#ifdef CONFIG_IXGBE_DCA
    if (dca_add_requester(&pdev->dev) == 0) {
        adapter->flags |= IXGBE_FLAG_DCA_ENABLED;
        ixgbe_setup_dca(adapter);
    }
#endif

    /* firmware requires driver version to be 0xFFFFFFFF
     * since os does not support feature
     */
    if (hw->mac.ops.set_fw_drv_ver)
        hw->mac.ops.set_fw_drv_ver(hw, 0xFF, 0xFF, 0xFF, 0xFF,
            0, NULL);

    /* setup link for SFP devices with MNG FW, else wait for IXGBE_UP */
    if (ixgbe_mng_enabled(hw) && ixgbe_is_sfp(hw) && hw->mac.ops.setup_link)
        hw->mac.ops.setup_link(hw,
            IXGBE_LINK_SPEED_10GB_FULL | IXGBE_LINK_SPEED_1GB_FULL,
            true);

    ixgbe_mii_bus_init(adapter);

    result = true;

done:
    return result;
    
err_sw_init:
    adapter->flags2 &= ~IXGBE_FLAG2_SEARCH_FOR_SFP;
    goto done;
}

bool IntelLucy::ixgbeSwInit(struct ixgbe_adapter *adapter, const struct ixgbe_info *ii)
{
    struct ixgbe_hw *hw = &adapter->hw;
    u32 fwsm;
    bool result = false;

    /* PCI config space info */
    hw->vendor_id = pciDeviceData.vendor;
    hw->device_id = pciDeviceData.device;
    hw->revision_id = pciDeviceData.revision;
    hw->subsystem_vendor_id = pciDeviceData.subsystem_vendor;
    hw->subsystem_device_id = pciDeviceData.subsystem_device;

    /* get_invariants needs the device IDs */
    ii->get_invariants(hw);
    
    switch (hw->mac.type) {
        case ixgbe_mac_82598EB:
        case ixgbe_mac_82599EB:
        case ixgbe_mac_X540:
        case ixgbe_mac_X550:
            /* ixgbe_mac_X550EM_x */
            /* ixgbe_mac_x550em_a */
            hw->mac.max_msix_vectors = ixgbeGetPCIeMSIxCountGeneric(hw);
            break;
            
        default:
            break;
    }
    adapter->flags2 |= IXGBE_FLAG2_RSC_CAPABLE;
    adapter->max_q_vectors = MAX_Q_VECTORS_82599;
    adapter->atr_sample_rate = 20;
    adapter->fdir_pballoc = IXGBE_FDIR_PBALLOC_NONE;
    
    /* Alloc memory for MAC address table. */
    adapter->mac_table = (struct ixgbe_mac_addr*)IOMallocZero(
        hw->mac.num_rar_entries * sizeof(struct ixgbe_mac_addr));
    
    if (!adapter->mac_table)
        goto done;
    
    /* Init mac table with default values. */
    ixgbe_init_mac_table(adapter);
    
    adapter->rss_key = (u32 *)rsskey;
    
    /* Set MAC specific capability flags and exceptions */
    switch (hw->mac.type) {
        case ixgbe_mac_82598EB:
            adapter->flags2 &= ~IXGBE_FLAG2_RSC_CAPABLE;

            if (hw->device_id == IXGBE_DEV_ID_82598AT)
                adapter->flags |= IXGBE_FLAG_FAN_FAIL_CAPABLE;

            adapter->max_q_vectors = MAX_Q_VECTORS_82598;
            adapter->atr_sample_rate = 0;
            adapter->fdir_pballoc = 0;
            break;
            
        case ixgbe_mac_82599EB:
            if (hw->device_id == IXGBE_DEV_ID_82599_T3_LOM)
                adapter->flags2 |= IXGBE_FLAG2_TEMP_SENSOR_CAPABLE;
            break;
            
        case ixgbe_mac_X540:
            fwsm = IXGBE_READ_REG(hw, IXGBE_FWSM(hw));
            
            if (fwsm & IXGBE_FWSM_TS_ENABLED)
            adapter->flags2 |= IXGBE_FLAG2_TEMP_SENSOR_CAPABLE;
        break;
            
        case ixgbe_mac_x550em_a:
            switch (hw->device_id) {
                case IXGBE_DEV_ID_X550EM_A_1G_T:
                case IXGBE_DEV_ID_X550EM_A_1G_T_L:
                    adapter->flags2 |= IXGBE_FLAG2_TEMP_SENSOR_CAPABLE;
                    break;
                    
                default:
                    break;
            }
            /* fallthrough */
        case ixgbe_mac_X550EM_x:
            /* fallthrough */
        case ixgbe_mac_X550:
            if (hw->mac.type == ixgbe_mac_X550)
                adapter->flags2 |= IXGBE_FLAG2_TEMP_SENSOR_CAPABLE;
            break;
            
        default:
            break;
    }
    if ((adapter->flags2 & IXGBE_FLAG2_RSC_CAPABLE) && enableRSC)
        adapter->flags2 |= IXGBE_FLAG2_RSC_ENABLED;

    //ixgbe_init_ipsec_offload(adapter);

    /* default flow control settings */
    hw->fc.requested_mode = ixgbe_fc_none;
    hw->fc.current_mode = ixgbe_fc_full;    /* init for ethtool output */
    ixgbePbthreshSetup(adapter);
    hw->fc.pause_time = IXGBE_DEFAULT_FCPAUSE;
    hw->fc.send_xon = true;
    hw->fc.disable_fc_autoneg = ixgbe_device_supports_autoneg_fc(hw);

    /* Setup itr according to config parameters. */
    if (rxThrottleTime > 0) {
        adapter->rx_itr_setting = (rxThrottleTime << 2) & IXGBE_MAX_EITR;
    } else {
        /*Enable adative itr mode.*/
        adapter->rx_itr_setting = 1;
    }
    if (txThrottleTime > 0) {
        adapter->tx_itr_setting = (txThrottleTime << 2) & IXGBE_MAX_EITR;
    } else {
        /*Enable adative itr mode.*/
        adapter->tx_itr_setting = 1;
    }
    ixgbeSetupQueueVectors(adapter);
        
    /* set default work limits */
    adapter->tx_work_limit = IXGBE_DEFAULT_TX_WORK;

    /* initialize eeprom parameters */
    if (ixgbe_init_eeprom_params_generic(hw)) {
        IOLog("EEPROM initialization failed.\n");
        goto error_malloc;
    }

    /* PF holds first pool slot */
    //set_bit(0, adapter->fwd_bitmask);
    set_bit(__IXGBE_DOWN, &adapter->state);

    result = true;

done:
    return result;

error_malloc:
    IOFree(adapter->mac_table, hw->mac.num_rar_entries * sizeof(struct ixgbe_mac_addr));
    goto done;
}

void IntelLucy::ixgbeStop(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;

    if (adapter->mac_table) {
        IOFree(adapter->mac_table,
            hw->mac.num_rar_entries * sizeof(struct ixgbe_mac_addr));
        adapter->mac_table = NULL;
    }
}

bool IntelLucy::ixgbeEnable(struct ixgbe_adapter *adapter)
{
    bool result = false;
    
    clear_bit(__IXGBE_DISABLED, &adapter->state);

    ixgbe_get_hw_control(adapter);
    ixgbeReset(adapter);

    IXGBE_WRITE_REG(&adapter->hw, IXGBE_WUS, ~0);

    ixgbeUp(adapter);
    
    result = true;
    
done:
    return result;
}

void IntelLucy::ixgbeUp(struct ixgbe_adapter *adapter)
{
    const IONetworkMedium *selectedMedium;

    selectedMedium = getSelectedMedium();
    
    if (!selectedMedium) {
        DebugLog("No medium selected. Falling back to autonegotiation.\n");
        selectedMedium = mediaTable[MIDX_AUTO];
        setCurrentMedium(selectedMedium);
    }
    clear_bit(__POLL_MODE, &stateFlags);
    
    ixgbeConfigure(adapter);

    //ixgbe_ptp_init(adapter);

    ixgbeUpComplete(adapter);
}

void IntelLucy::ixgbeUpComplete(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;
    int err;
    u32 ctrl_ext;

    ixgbeSetupGpie(adapter);

    ixgbeConfigureMsi(adapter);

    /* enable the optics for 82599 SFP+ fiber */
    if (hw->mac.ops.enable_tx_laser)
        hw->mac.ops.enable_tx_laser(hw);

    if (hw->phy.ops.set_phy_power)
        hw->phy.ops.set_phy_power(hw, true);

    clear_bit(__IXGBE_DOWN, &adapter->state);

    if (ixgbe_is_sfp(hw)) {
        ixgbe_sfp_link_config(adapter);
    } else {
        err = ixgbe_non_sfp_link_config(hw);
        
        if (err)
            IOLog("link_config() failed %d.\n", err);
    }

    /* clear any pending interrupts, may auto mask */
    IXGBE_READ_REG(hw, IXGBE_EICR);
    ixgbe_irq_enable(adapter, rxActiveQueueMask | txActiveQueueMask, true);

    /*
     * If this adapter has a fan, check to see if we had a failure
     * before we enabled the interrupt.
     */
    if (adapter->flags & IXGBE_FLAG_FAN_FAIL_CAPABLE) {
        u32 esdp = IXGBE_READ_REG(hw, IXGBE_ESDP);
        
        if (esdp & IXGBE_ESDP_SDP1)
            IOLog("Fan has stopped, replace the adapter.\n");
    }

    /* bring the link up in the watchdog, this could race with our first
     * link up interrupt but shouldn't be a problem */
    adapter->flags |= IXGBE_FLAG_NEED_LINK_UPDATE;
    clock_get_uptime((uint64_t *)&adapter->link_check_timeout);
    
    /* Reschedule next timeout. */
    timerSource->cancelTimeout();
    timerSource->setTimeoutUS(kTimeout10us);

    //ixgbe_clear_vf_stats_counters(adapter);
    /* Set PF Reset Done bit so PF/VF Mail Ops can work */
    ctrl_ext = IXGBE_READ_REG(hw, IXGBE_CTRL_EXT);
    ctrl_ext |= IXGBE_CTRL_EXT_PFRSTD;
    IXGBE_WRITE_REG(hw, IXGBE_CTRL_EXT, ctrl_ext);

    DebugLog("ixgbeUpComplete");
}

void IntelLucy::ixgbeDown(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;

    /* signal that we are down to the interrupt handler */
    if (test_and_set_bit(__IXGBE_DOWN, &adapter->state))
        return; /* do nothing if already down */

    /* Disable Rx */
    ixgbeDisableRx(adapter);

    ixgbe_irq_disable(adapter);

    clear_bit(__IXGBE_RESET_REQUESTED, &adapter->state);
    adapter->flags2 &= ~IXGBE_FLAG2_FDIR_REQUIRES_REINIT;
    adapter->flags &= ~IXGBE_FLAG_NEED_LINK_UPDATE;

    /* disable transmits in the hardware now that interrupts are off */
    ixgbeDisableTx(adapter);

    if (pciDevice->checkLink() == kIOReturnSuccess)
        ixgbeReset(adapter);

    /* power down the optics for 82599 SFP+ fiber */
    if (hw->mac.ops.disable_tx_laser)
        hw->mac.ops.disable_tx_laser(hw);

    ixgbeClearTxRings(adapter);
    ixgbeClearRxRings(adapter);
}

void IntelLucy::ixgbeDisable(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;
    UInt32 wufc = adapter->wol;
    UInt32 fctrl;
    UInt32 ctrl;

    //ixgbe_ptp_suspend(adapter);

    if (adapter->hw.phy.ops.enter_lplu) {
        adapter->hw.phy.reset_disable = true;
        ixgbeDown(adapter);
        adapter->hw.phy.ops.enter_lplu(&adapter->hw);
        adapter->hw.phy.reset_disable = false;
    } else {
        ixgbeDown(adapter);
    }
    if (hw->mac.ops.stop_link_on_d3)
        hw->mac.ops.stop_link_on_d3(hw);

    if (wolActive) {
        ixgbeSetRxMode();

        /* enable the optics for 82599 SFP+ fiber as we can WoL */
        if (hw->mac.ops.enable_tx_laser)
            hw->mac.ops.enable_tx_laser(hw);

        /* enable the reception of multicast packets */
        fctrl = IXGBE_READ_REG(hw, IXGBE_FCTRL);
        fctrl |= IXGBE_FCTRL_MPE;
        IXGBE_WRITE_REG(hw, IXGBE_FCTRL, fctrl);

        ctrl = IXGBE_READ_REG(hw, IXGBE_CTRL);
        ctrl |= IXGBE_CTRL_GIO_DIS;
        IXGBE_WRITE_REG(hw, IXGBE_CTRL, ctrl);

        IXGBE_WRITE_REG(hw, IXGBE_WUFC, wufc);
    } else {
        IXGBE_WRITE_REG(hw, IXGBE_WUC, 0);
        IXGBE_WRITE_REG(hw, IXGBE_WUFC, 0);
    }
    if (hw->phy.ops.set_phy_power && (wufc == 0))
        hw->phy.ops.set_phy_power(hw, false);

    ixgbe_release_hw_control(adapter);
}

#pragma mark --- general initialization methods ---



/**
 *  ixgbeStopAdapterGeneric - Generic stop Tx/Rx units
 *  @hw: pointer to hardware structure
 *
 *  Sets the adapter_stopped flag within ixgbe_hw struct. Clears interrupts,
 *  disables transmit and receive units. The adapter_stopped flag is used by
 *  the shared code and drivers to determine if the adapter is in a stopped
 *  state and should not touch the hardware.
 **/
SInt32 IntelLucy::ixgbeStopAdapterGeneric(struct ixgbe_hw *hw)
{
    u32 reg_val;
    u16 i;

    /*
     * Set the adapter_stopped flag so other driver functions stop touching
     * the hardware
     */
    hw->adapter_stopped = true;

    /* Disable the receive unit */
    hw->mac.ops.disable_rx(hw);

    /* Clear interrupt mask to stop interrupts from being generated */
    IXGBE_WRITE_REG(hw, IXGBE_EIMC, IXGBE_IRQ_CLEAR_MASK);

    /* Clear any pending interrupts, flush previous writes */
    IXGBE_READ_REG(hw, IXGBE_EICR);

    /* Disable the transmit unit.  Each queue must be disabled. */
    for (i = 0; i < hw->mac.max_tx_queues; i++)
        IXGBE_WRITE_REG(hw, IXGBE_TXDCTL(i), IXGBE_TXDCTL_SWFLSH);

    /* Disable the receive unit by stopping each queue */
    for (i = 0; i < hw->mac.max_rx_queues; i++) {
        reg_val = IXGBE_READ_REG(hw, IXGBE_RXDCTL(i));
        reg_val &= ~IXGBE_RXDCTL_ENABLE;
        reg_val |= IXGBE_RXDCTL_SWFLSH;
        IXGBE_WRITE_REG(hw, IXGBE_RXDCTL(i), reg_val);
    }

    /* flush all queues disables */
    IXGBE_WRITE_FLUSH(hw);
    usleep_range(1000, 2000);

    /*
     * Prevent the PCI-E bus from hanging by disabling PCI-E primary
     * access and verify no pending requests
     */
    return ixgbeDisablePCIePrimary(hw);
}


/**  ixgbeResetHwX550em - Perform hardware reset
 **  @hw: pointer to hardware structure
 **
 **  Resets the hardware by resetting the transmit and receive units, masks
 **  and clears all interrupts, perform a PHY reset, and perform a link (MAC)
 **  reset.
 **/
SInt32 IntelLucy::ixgbeResetHwX550em(struct ixgbe_hw *hw)
{
    ixgbe_link_speed link_speed;
    s32 status;
    u32 ctrl = 0;
    u32 i;
    bool link_up = false;
    u32 swfw_mask = hw->phy.phy_semaphore_mask;

    /* Call adapter stop to disable Tx/Rx and clear interrupts */
    status = ixgbe_stop_adapter_generic(hw);
    
    if (status)
        return status;

    /* flush pending Tx transactions */
    ixgbeClearTxPending(hw);

    /* set MDIO speed before talking to the PHY in case it's the 1st time */
    ixgbe_set_mdio_speed(hw);

    /* PHY ops must be identified and initialized prior to reset */
    status = hw->phy.ops.init(hw);
    if (status == IXGBE_ERR_SFP_NOT_SUPPORTED ||
        status == IXGBE_ERR_PHY_ADDR_INVALID)
        return status;

    /* start the external PHY */
    if (hw->phy.type == ixgbe_phy_x550em_ext_t) {
        status = ixgbe_init_ext_t_x550em(hw);
        
        if (status)
            return status;
    }

    /* Setup SFP module if there is one present. */
    if (hw->phy.sfp_setup_needed) {
        status = hw->mac.ops.setup_sfp(hw);
        hw->phy.sfp_setup_needed = false;
    }

    if (status == IXGBE_ERR_SFP_NOT_SUPPORTED)
        return status;

    /* Reset PHY */
    if (!hw->phy.reset_disable && hw->phy.ops.reset)
        hw->phy.ops.reset(hw);

mac_reset_top:
    /* Issue global reset to the MAC.  Needs to be SW reset if link is up.
     * If link reset is used when link is up, it might reset the PHY when
     * mng is using it.  If link is down or the flag to force full link
     * reset is set, then perform link reset.
     */
    ctrl = IXGBE_CTRL_LNK_RST;

    if (!hw->force_full_reset) {
        hw->mac.ops.check_link(hw, &link_speed, &link_up, false);
        if (link_up)
            ctrl = IXGBE_CTRL_RST;
    }

    status = hw->mac.ops.acquire_swfw_sync(hw, swfw_mask);
    if (status) {
        DebugLog("semaphore failed with %d.", status);
        return IXGBE_ERR_SWFW_SYNC;
    }

    ctrl |= IXGBE_READ_REG(hw, IXGBE_CTRL);
    IXGBE_WRITE_REG(hw, IXGBE_CTRL, ctrl);
    IXGBE_WRITE_FLUSH(hw);
    hw->mac.ops.release_swfw_sync(hw, swfw_mask);
    usleep_range(1000, 1200);

    /* Poll for reset bit to self-clear meaning reset is complete */
    for (i = 0; i < 10; i++) {
        ctrl = IXGBE_READ_REG(hw, IXGBE_CTRL);
        if (!(ctrl & IXGBE_CTRL_RST_MASK))
            break;
        udelay(1);
    }

    if (ctrl & IXGBE_CTRL_RST_MASK) {
        status = IXGBE_ERR_RESET_FAILED;
        DebugLog("Reset polling failed to complete.\n");
    }

    msleep(50);

    /* Double resets are required for recovery from certain error
     * clear the multicast table.  Also reset num_rar_entries to 128,
     * since we modify this value when programming the SAN MAC address.
     */
    if (hw->mac.flags & IXGBE_FLAGS_DOUBLE_RESET_REQUIRED) {
        hw->mac.flags &= ~IXGBE_FLAGS_DOUBLE_RESET_REQUIRED;
        goto mac_reset_top;
    }

    /* Store the permanent mac address */
    hw->mac.ops.get_mac_addr(hw, hw->mac.perm_addr);

    /* Store MAC address from RAR0, clear receive address registers, and
     * clear the multicast table.  Also reset num_rar_entries to 128,
     * since we modify this value when programming the SAN MAC address.
     */
    hw->mac.num_rar_entries = 128;
    hw->mac.ops.init_rx_addrs(hw);

    ixgbe_set_mdio_speed(hw);

    if (hw->device_id == IXGBE_DEV_ID_X550EM_X_SFP)
        ixgbe_setup_mux_ctl(hw);

    return status;
}

#define IXGBE_X540_MAX_TX_QUEUES    128

/**
 *  ixgbe_reset_hw_X540 - Perform hardware reset
 *  @hw: pointer to hardware structure
 *
 *  Resets the hardware by resetting the transmit and receive units, masks
 *  and clears all interrupts, perform a PHY reset, and perform a link (MAC)
 *  reset.
 **/
s32 IntelLucy::ixgbeResetHwX540(struct ixgbe_hw *hw)
{
    s32 status;
    u32 ctrl, i;
    u32 swfw_mask = hw->phy.phy_semaphore_mask;

    /* Call adapter stop to disable tx/rx and clear interrupts */
    status = ixgbeStopAdapterGeneric(hw);
    
    if (status)
        return status;

    /* flush pending Tx transactions */
    ixgbeClearTxPending(hw);

mac_reset_top:
    status = hw->mac.ops.acquire_swfw_sync(hw, swfw_mask);
    if (status) {
        DebugLog("semaphore failed with %d.", status);
        return IXGBE_ERR_SWFW_SYNC;
    }

    ctrl = IXGBE_CTRL_RST;
    ctrl |= IXGBE_READ_REG(hw, IXGBE_CTRL);
    IXGBE_WRITE_REG(hw, IXGBE_CTRL, ctrl);
    IXGBE_WRITE_FLUSH(hw);
    hw->mac.ops.release_swfw_sync(hw, swfw_mask);
    usleep_range(1000, 1200);

    /* Poll for reset bit to self-clear indicating reset is complete */
    for (i = 0; i < 10; i++) {
        ctrl = IXGBE_READ_REG(hw, IXGBE_CTRL);
        if (!(ctrl & IXGBE_CTRL_RST_MASK))
            break;
        udelay(1);
    }

    if (ctrl & IXGBE_CTRL_RST_MASK) {
        status = IXGBE_ERR_RESET_FAILED;
        DebugLog("Reset polling failed to complete.\n");
    }
    msleep(100);

    /*
     * Double resets are required for recovery from certain error
     * conditions.  Between resets, it is necessary to stall to allow time
     * for any pending HW events to complete.
     */
    if (hw->mac.flags & IXGBE_FLAGS_DOUBLE_RESET_REQUIRED) {
        hw->mac.flags &= ~IXGBE_FLAGS_DOUBLE_RESET_REQUIRED;
        goto mac_reset_top;
    }

    /* Set the Rx packet buffer size. */
    IXGBE_WRITE_REG(hw, IXGBE_RXPBSIZE(0), 384 << IXGBE_RXPBSIZE_SHIFT);

    /* Store the permanent mac address */
    hw->mac.ops.get_mac_addr(hw, hw->mac.perm_addr);

    /*
     * Store MAC address from RAR0, clear receive address registers, and
     * clear the multicast table.  Also reset num_rar_entries to 128,
     * since we modify this value when programming the SAN MAC address.
     */
    hw->mac.num_rar_entries = IXGBE_X540_MAX_TX_QUEUES;
    hw->mac.ops.init_rx_addrs(hw);

    /* Store the permanent SAN mac address */
    hw->mac.ops.get_san_mac_addr(hw, hw->mac.san_addr);

    /* Add the SAN MAC address to the RAR only if it's a valid address */
    if (is_valid_ether_addr(hw->mac.san_addr)) {
        /* Save the SAN MAC RAR index */
        hw->mac.san_mac_rar_index = hw->mac.num_rar_entries - 1;

        hw->mac.ops.set_rar(hw, hw->mac.san_mac_rar_index,
                    hw->mac.san_addr, 0, IXGBE_RAH_AV);

        /* clear VMDq pool/queue selection for this RAR */
        hw->mac.ops.clear_vmdq(hw, hw->mac.san_mac_rar_index,
                       IXGBE_CLEAR_VMDQ_ALL);

        /* Reserve the last RAR for the SAN MAC address */
        hw->mac.num_rar_entries--;
    }

    /* Store the alternative WWNN/WWPN prefix */
    hw->mac.ops.get_wwn_prefix(hw, &hw->mac.wwnn_prefix,
                   &hw->mac.wwpn_prefix);

    return status;
}

#define IXGBE_82599_RAR_ENTRIES   128

/**
 *  ixgbeResetHw82599 - Perform hardware reset
 *  @hw: pointer to hardware structure
 *
 *  Resets the hardware by resetting the transmit and receive units, masks
 *  and clears all interrupts, perform a PHY reset, and perform a link (MAC)
 *  reset.
 **/
s32 IntelLucy::ixgbeResetHw82599(struct ixgbe_hw *hw)
{
    ixgbe_link_speed link_speed;
    s32 status;
    u32 ctrl, i, autoc, autoc2;
    u32 curr_lms;
    bool link_up = false;

    /* Call adapter stop to disable tx/rx and clear interrupts */
    status = ixgbeStopAdapterGeneric(hw);
    
    if (status)
        return status;

    /* flush pending Tx transactions */
    ixgbeClearTxPending(hw);

    /* PHY ops must be identified and initialized prior to reset */

    /* Identify PHY and related function pointers */
    status = hw->phy.ops.init(hw);

    if (status == IXGBE_ERR_SFP_NOT_SUPPORTED)
        return status;

    /* Setup SFP module if there is one present. */
    if (hw->phy.sfp_setup_needed) {
        status = hw->mac.ops.setup_sfp(hw);
        hw->phy.sfp_setup_needed = false;
    }

    if (status == IXGBE_ERR_SFP_NOT_SUPPORTED)
        return status;

    /* Reset PHY */
    if (hw->phy.reset_disable == false && hw->phy.ops.reset != NULL)
        hw->phy.ops.reset(hw);

    /* remember AUTOC from before we reset */
    curr_lms = IXGBE_READ_REG(hw, IXGBE_AUTOC) & IXGBE_AUTOC_LMS_MASK;

mac_reset_top:
    /*
     * Issue global reset to the MAC. Needs to be SW reset if link is up.
     * If link reset is used when link is up, it might reset the PHY when
     * mng is using it.  If link is down or the flag to force full link
     * reset is set, then perform link reset.
     */
    ctrl = IXGBE_CTRL_LNK_RST;
    if (!hw->force_full_reset) {
        hw->mac.ops.check_link(hw, &link_speed, &link_up, false);
        if (link_up)
            ctrl = IXGBE_CTRL_RST;
    }

    ctrl |= IXGBE_READ_REG(hw, IXGBE_CTRL);
    IXGBE_WRITE_REG(hw, IXGBE_CTRL, ctrl);
    IXGBE_WRITE_FLUSH(hw);
    usleep_range(1000, 1200);

    /* Poll for reset bit to self-clear indicating reset is complete */
    for (i = 0; i < 10; i++) {
        ctrl = IXGBE_READ_REG(hw, IXGBE_CTRL);
        if (!(ctrl & IXGBE_CTRL_RST_MASK))
            break;
        udelay(1);
    }

    if (ctrl & IXGBE_CTRL_RST_MASK) {
        status = IXGBE_ERR_RESET_FAILED;
        DebugLog("Reset polling failed to complete.\n");
    }

    msleep(50);

    /*
     * Double resets are required for recovery from certain error
     * conditions.  Between resets, it is necessary to stall to allow time
     * for any pending HW events to complete.
     */
    if (hw->mac.flags & IXGBE_FLAGS_DOUBLE_RESET_REQUIRED) {
        hw->mac.flags &= ~IXGBE_FLAGS_DOUBLE_RESET_REQUIRED;
        goto mac_reset_top;
    }

    /*
     * Store the original AUTOC/AUTOC2 values if they have not been
     * stored off yet.  Otherwise restore the stored original
     * values since the reset operation sets back to defaults.
     */
    autoc = IXGBE_READ_REG(hw, IXGBE_AUTOC);
    autoc2 = IXGBE_READ_REG(hw, IXGBE_AUTOC2);

    /* Enable link if disabled in NVM */
    if (autoc2 & IXGBE_AUTOC2_LINK_DISABLE_MASK) {
        autoc2 &= ~IXGBE_AUTOC2_LINK_DISABLE_MASK;
        IXGBE_WRITE_REG(hw, IXGBE_AUTOC2, autoc2);
        IXGBE_WRITE_FLUSH(hw);
    }

    if (hw->mac.orig_link_settings_stored == false) {
        hw->mac.orig_autoc = autoc;
        hw->mac.orig_autoc2 = autoc2;
        hw->mac.orig_link_settings_stored = true;
    } else {

        /* If MNG FW is running on a multi-speed device that
         * doesn't autoneg with out driver support we need to
         * leave LMS in the state it was before we MAC reset.
         * Likewise if we support WoL we don't want change the
         * LMS state either.
         */
        if ((hw->phy.multispeed_fiber && ixgbe_mng_enabled(hw)) ||
            hw->wol_enabled)
            hw->mac.orig_autoc =
                (hw->mac.orig_autoc & ~IXGBE_AUTOC_LMS_MASK) |
                curr_lms;

        if (autoc != hw->mac.orig_autoc) {
            status = hw->mac.ops.prot_autoc_write(hw,
                            hw->mac.orig_autoc,
                            false);
            if (status)
                return status;
        }

        if ((autoc2 & IXGBE_AUTOC2_UPPER_MASK) !=
            (hw->mac.orig_autoc2 & IXGBE_AUTOC2_UPPER_MASK)) {
            autoc2 &= ~IXGBE_AUTOC2_UPPER_MASK;
            autoc2 |= (hw->mac.orig_autoc2 &
                   IXGBE_AUTOC2_UPPER_MASK);
            IXGBE_WRITE_REG(hw, IXGBE_AUTOC2, autoc2);
        }
    }

    /* Store the permanent mac address */
    hw->mac.ops.get_mac_addr(hw, hw->mac.perm_addr);

    /*
     * Store MAC address from RAR0, clear receive address registers, and
     * clear the multicast table.  Also reset num_rar_entries to 128,
     * since we modify this value when programming the SAN MAC address.
     */
    hw->mac.num_rar_entries = IXGBE_82599_RAR_ENTRIES;
    hw->mac.ops.init_rx_addrs(hw);

    /* Store the permanent SAN mac address */
    hw->mac.ops.get_san_mac_addr(hw, hw->mac.san_addr);

    /* Add the SAN MAC address to the RAR only if it's a valid address */
    if (is_valid_ether_addr(hw->mac.san_addr)) {
        /* Save the SAN MAC RAR index */
        hw->mac.san_mac_rar_index = hw->mac.num_rar_entries - 1;

        hw->mac.ops.set_rar(hw, hw->mac.san_mac_rar_index,
                    hw->mac.san_addr, 0, IXGBE_RAH_AV);

        /* clear VMDq pool/queue selection for this RAR */
        hw->mac.ops.clear_vmdq(hw, hw->mac.san_mac_rar_index,
                       IXGBE_CLEAR_VMDQ_ALL);

        /* Reserve the last RAR for the SAN MAC address */
        hw->mac.num_rar_entries--;
    }

    /* Store the alternative WWNN/WWPN prefix */
    hw->mac.ops.get_wwn_prefix(hw, &hw->mac.wwnn_prefix,
                       &hw->mac.wwpn_prefix);

    return status;
}

/**
 *  ixgbe_init_hw_generic - Generic hardware initialization
 *  @hw: pointer to hardware structure
 *
 *  Initialize the hardware by resetting the hardware, filling the bus info
 *  structure and media type, clears all on chip counters, initializes receive
 *  address registers, multicast table, VLAN filter table, calls routine to set
 *  up link and flow control settings, and leaves transmit and receive units
 *  disabled and uninitialized
 **/
SInt32 IntelLucy::ixgbeInitHwGeneric(struct ixgbe_hw *hw)
{
    s32 status;

    /* Reset the hardware */
    switch (hw->mac.type) {
        case ixgbe_mac_82599EB:
            status = ixgbeResetHw82599(hw);
            break;
            
        case ixgbe_mac_X540:
        case ixgbe_mac_X550:
            status = ixgbeResetHwX540(hw);
            break;
            
        case ixgbe_mac_X550EM_x:
        case ixgbe_mac_x550em_a:
            status = ixgbeResetHwX550em(hw);
            break;
            
        default:
            status = hw->mac.ops.reset_hw(hw);
            break;
    }
    if (status == 0) {
        /* Start the HW */
        status = hw->mac.ops.start_hw(hw);
    }

    /* Initialize the LED link active for LED blink support */
    if (hw->mac.ops.init_led_link_act)
        hw->mac.ops.init_led_link_act(hw);

    return status;
}

void IntelLucy::ixgbeReset(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;
    int err;

    if (ixgbe_removed(hw->hw_addr))
        return;
    
    /* lock SFP init bit to prevent race conditions with the watchdog */
    while (test_and_set_bit(__IXGBE_IN_SFP_INIT, &adapter->state))
        usleep_range(1000, 2000);

    /* clear all SFP and link config related flags while holding SFP_INIT */
    adapter->flags2 &= ~(IXGBE_FLAG2_SEARCH_FOR_SFP |
                 IXGBE_FLAG2_SFP_NEEDS_RESET);
    adapter->flags &= ~IXGBE_FLAG_NEED_LINK_CONFIG;

    err = ixgbeInitHwGeneric(hw);
    
    switch (err) {
        case 0:
        case IXGBE_ERR_SFP_NOT_PRESENT:
        case IXGBE_ERR_SFP_NOT_SUPPORTED:
            break;
            
        case IXGBE_ERR_PRIMARY_REQUESTS_PENDING:
            e_dev_err("primary disable timed out\n");
            break;
            
        case IXGBE_ERR_EEPROM_VERSION:
            /* We are running on a pre-production device, log a warning */
            IOLog("Warning: EEPROM version error.");
            break;
            
        default:
            IOLog("Hardware Error: %d\n", err);
            break;
    }

    clear_bit(__IXGBE_IN_SFP_INIT, &adapter->state);

    /* flush entries out of MAC table */
    ixgbe_flush_sw_mac_table(adapter);
    ixgbe_sync_mac_table(adapter);

    /* do not flush user set addresses */
    ixgbe_mac_set_default_filter(adapter);

    /* update SAN MAC vmdq pool selection */
/*
    if (hw->mac.san_mac_rar_index)
        hw->mac.ops.set_vmdq_san_mac(hw, VMDQ_P(0));

    if (test_bit(__IXGBE_PTP_RUNNING, &adapter->state))
        ixgbe_ptp_reset(adapter);
*/
    if (hw->phy.ops.set_phy_power) {
        if (!test_bit(__ENABLED, &stateFlags) && !adapter->wol)
            hw->phy.ops.set_phy_power(hw, false);
        else
            hw->phy.ops.set_phy_power(hw, true);
    }
}

void IntelLucy::ixgbeConfigure(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;
    
    ixgbeConfigurePb(adapter);
    ixgbeSetRxMode();
    //ixgbe_restore_vlan(adapter);

    switch (hw->mac.type) {
        case ixgbe_mac_82599EB:
        case ixgbe_mac_X540:
            hw->mac.ops.disable_rx_buff(hw);
            break;
            
        default:
            break;
    }
/*
    if (adapter->flags & IXGBE_FLAG_FDIR_HASH_CAPABLE) {
        ixgbe_init_fdir_signature_82599(&adapter->hw,
                        adapter->fdir_pballoc);
    } else if (adapter->flags & IXGBE_FLAG_FDIR_PERFECT_CAPABLE) {
        ixgbe_init_fdir_perfect_82599(&adapter->hw,
                          adapter->fdir_pballoc);
        ixgbe_fdir_filter_restore(adapter);
    }
*/
    switch (hw->mac.type) {
        case ixgbe_mac_82599EB:
        case ixgbe_mac_X540:
            hw->mac.ops.enable_rx_buff(hw);
            break;
            
        default:
            break;
    }
    /* configure DCA */
/*
    if (adapter->flags & IXGBE_FLAG_DCA_CAPABLE)
        ixgbe_setup_dca(adapter);
*/
    ixgbeConfigureTx(adapter);
    ixgbeConfigureRx(adapter);
    //ixgbe_configure_dfwd(adapter);

}

void IntelLucy::ixgbeSetupGpie(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;
    u32 gpie = 0;

    /* legacy interrupts, use EIAM to auto-mask when reading EICR,
     * specifically only auto mask tx and rx interrupts */
    IXGBE_WRITE_REG(hw, IXGBE_EIAM, IXGBE_EICS_RTX_QUEUE);

    /* Enable Thermal over heat sensor interrupt */
    if (adapter->flags2 & IXGBE_FLAG2_TEMP_SENSOR_CAPABLE) {
        switch (adapter->hw.mac.type) {
            case ixgbe_mac_82599EB:
                gpie |= IXGBE_SDP0_GPIEN_8259X;
                break;
            default:
                break;
        }
    }
    
    /* Enable fan failure interrupt */
    if (adapter->flags & IXGBE_FLAG_FAN_FAIL_CAPABLE)
        gpie |= IXGBE_SDP1_GPIEN(hw);

    switch (hw->mac.type) {
        case ixgbe_mac_82599EB:
            gpie |= IXGBE_SDP1_GPIEN_8259X | IXGBE_SDP2_GPIEN_8259X;
            break;
        case ixgbe_mac_X550EM_x:
        case ixgbe_mac_x550em_a:
            gpie |= IXGBE_SDP0_GPIEN_X540;
            break;
        default:
            break;
    }
    IXGBE_WRITE_REG(hw, IXGBE_GPIE, gpie);
}

/**
 * ixgbeConfigureMsi - Initialize MSI interrupts
 * @adapter: board private structure
 *
 **/
void IntelLucy::ixgbeConfigureMsi(struct ixgbe_adapter *adapter)
{
    /* Initialize setting for ITR. */
    ixgbe_write_eitr(&rxRing[0].vector);
    ixgbe_write_eitr(&txRing[0].vector);

    ixgbe_set_ivar(adapter, 0, 0, 0);
    ixgbe_set_ivar(adapter, 1, 0, 1);

    DebugLog(
             "MSI interrupt IVAR setup done. rxITR: %uµs txITR: %uµs\n",
             rxRing[0].vector.itr >> 2,
             txRing[0].vector.itr >> 2
             );
    DebugLog("rxVector: %hu txVector: %hu\n",
             rxRing[0].vector.vIndex, txRing[0].vector.vIndex);

}


void IntelLucy::ixgbeReinit(struct ixgbe_adapter *adapter)
{
    /* put off any impending NetWatchDogTimeout */
    timerSource->cancelTimeout();

    while (test_and_set_bit(__IXGBE_RESETTING, &adapter->state))
        usleep_range(1000, 2000);
    
    if (adapter->hw.phy.type == ixgbe_phy_fw)
        ixgbeWatchdogLinkIsDown(adapter);
    
    ixgbeDown(adapter);
    ixgbeUp(adapter);
    
    clear_bit(__IXGBE_RESETTING, &adapter->state);
}

/*
 * ixgbePbthreshSetup - calculate and setup high low water marks
 */
void IntelLucy::ixgbePbthreshSetup(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;
    int num_tc = adapter->hw_tcs;
    int i;

    if (!num_tc)
        num_tc = 1;

    for (i = 0; i < num_tc; i++) {
        hw->fc.high_water[i] = ixgbeHpbthresh(adapter, i);
        hw->fc.low_water[i] = ixgbeLpbthresh(adapter, i);

        /* Low water marks must not be larger than high water marks */
        if (hw->fc.low_water[i] > hw->fc.high_water[i])
            hw->fc.low_water[i] = 0;
    }

    for (; i < MAX_TRAFFIC_CLASS; i++)
        hw->fc.high_water[i] = 0;
}

void IntelLucy::ixgbeConfigurePb(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;
    int hdrm;
    u8 tc = adapter->hw_tcs;

    if (adapter->flags & IXGBE_FLAG_FDIR_HASH_CAPABLE ||
        adapter->flags & IXGBE_FLAG_FDIR_PERFECT_CAPABLE)
        hdrm = 32 << adapter->fdir_pballoc;
    else
        hdrm = 0;

    hw->mac.ops.set_rxpba(hw, tc, hdrm, PBA_STRATEGY_EQUAL);
    ixgbePbthreshSetup(adapter);
}

/**
 * ixgbe_hpbthresh - calculate high water mark for flow control
 *
 * @adapter: board private structure to calculate for
 * @pb: packet buffer to calculate
 */
int IntelLucy::ixgbeHpbthresh(struct ixgbe_adapter *adapter, int pb)
{
    struct ixgbe_hw *hw = &adapter->hw;
    int link, tc, kb, marker;
    u32 dv_id, rx_pba;

    /* Calculate max LAN frame size */
    tc = link = mtu + ETH_HLEN + ETH_FCS_LEN + IXGBE_ETH_FRAMING;

    /* Calculate delay value for device */
    switch (hw->mac.type) {
        case ixgbe_mac_X540:
        case ixgbe_mac_X550:
        case ixgbe_mac_X550EM_x:
        case ixgbe_mac_x550em_a:
            dv_id = IXGBE_DV_X540(link, tc);
            break;
            
        default:
            dv_id = IXGBE_DV(link, tc);
            break;
    }

    /* Loopback switch introduces additional latency */
    if (adapter->flags & IXGBE_FLAG_SRIOV_ENABLED)
        dv_id += IXGBE_B2BT(tc);

    /* Delay value is calculated in bit times convert to KB */
    kb = IXGBE_BT2KB(dv_id);
    rx_pba = IXGBE_READ_REG(hw, IXGBE_RXPBSIZE(pb)) >> 10;

    marker = rx_pba - kb;

    /* It is possible that the packet buffer is not large enough
     * to provide required headroom. In this case throw an error
     * to user and a do the best we can.
     */
    if (marker < 0) {
        DebugLog("Packet Buffer(%i) can not provide enough"
                "headroom to support flow control."
                "Decrease MTU or number of traffic classes\n", pb);
        marker = tc + 1;
    }

    return marker;
}

/**
 * ixgbe_lpbthresh - calculate low water mark for flow control
 *
 * @adapter: board private structure to calculate for
 * @pb: packet buffer to calculate
 */
int IntelLucy::ixgbeLpbthresh(struct ixgbe_adapter *adapter, int pb)
{
    struct ixgbe_hw *hw = &adapter->hw;
    int tc;
    u32 dv_id;

    /* Calculate max LAN frame size */
    tc = mtu + ETH_HLEN + ETH_FCS_LEN;

#ifdef IXGBE_FCOE
    /* FCoE traffic class uses FCOE jumbo frames */
    if ((dev->features & NETIF_F_FCOE_MTU) &&
        (tc < IXGBE_FCOE_JUMBO_FRAME_SIZE) &&
        (pb == netdev_get_prio_tc_map(dev, adapter->fcoe.up)))
        tc = IXGBE_FCOE_JUMBO_FRAME_SIZE;
#endif

    /* Calculate delay value for device */
    switch (hw->mac.type) {
        case ixgbe_mac_X540:
        case ixgbe_mac_X550:
        case ixgbe_mac_X550EM_x:
        case ixgbe_mac_x550em_a:
            dv_id = IXGBE_LOW_DV_X540(tc);
            break;
            
        default:
            dv_id = IXGBE_LOW_DV(tc);
            break;
    }

    /* Delay value is calculated in bit times convert to KB */
    return IXGBE_BT2KB(dv_id);
}

/**
 * ixgbeUpdateStats - Update the board statistics counters.
 * @adapter: board private structure
 **/
void IntelLucy::ixgbeUpdateStats(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;
    struct ixgbe_hw_stats *hwstats = &adapter->stats;
    u64 total_mpc = 0;
    u32 i, missed_rx = 0, mpc, bprc, lxon, lxoff, xon_off_tot;
    
    if (test_bit(__IXGBE_DOWN, &adapter->state) ||
        test_bit(__IXGBE_RESETTING, &adapter->state))
        return;
    
    hwstats->crcerrs += IXGBE_READ_REG(hw, IXGBE_CRCERRS);
    
    /* 8 register reads */
    for (i = 0; i < 8; i++) {
        /* for packet buffers not used, the register should read 0 */
        mpc = IXGBE_READ_REG(hw, IXGBE_MPC(i));
        missed_rx += mpc;
        hwstats->mpc[i] += mpc;
        total_mpc += hwstats->mpc[i];
        hwstats->pxontxc[i] += IXGBE_READ_REG(hw, IXGBE_PXONTXC(i));
        hwstats->pxofftxc[i] += IXGBE_READ_REG(hw, IXGBE_PXOFFTXC(i));
        
        switch (hw->mac.type) {
            case ixgbe_mac_82598EB:
                hwstats->rnbc[i] += IXGBE_READ_REG(hw, IXGBE_RNBC(i));
                hwstats->qbtc[i] += IXGBE_READ_REG(hw, IXGBE_QBTC(i));
                hwstats->qbrc[i] += IXGBE_READ_REG(hw, IXGBE_QBRC(i));
                hwstats->pxonrxc[i] +=
                IXGBE_READ_REG(hw, IXGBE_PXONRXC(i));
                break;
            case ixgbe_mac_82599EB:
            case ixgbe_mac_X540:
            case ixgbe_mac_X550:
            case ixgbe_mac_X550EM_x:
            case ixgbe_mac_x550em_a:
                hwstats->pxonrxc[i] +=
                IXGBE_READ_REG(hw, IXGBE_PXONRXCNT(i));
                break;
            default:
                break;
        }
    }
    
    /*16 register reads */
    for (i = 0; i < 16; i++) {
        hwstats->qptc[i] += IXGBE_READ_REG(hw, IXGBE_QPTC(i));
        hwstats->qprc[i] += IXGBE_READ_REG(hw, IXGBE_QPRC(i));
        if ((hw->mac.type == ixgbe_mac_82599EB) ||
            (hw->mac.type == ixgbe_mac_X540) ||
            (hw->mac.type == ixgbe_mac_X550) ||
            (hw->mac.type == ixgbe_mac_X550EM_x) ||
            (hw->mac.type == ixgbe_mac_x550em_a)) {
            hwstats->qbtc[i] += IXGBE_READ_REG(hw, IXGBE_QBTC_L(i));
            IXGBE_READ_REG(hw, IXGBE_QBTC_H(i)); /* to clear */
            hwstats->qbrc[i] += IXGBE_READ_REG(hw, IXGBE_QBRC_L(i));
            IXGBE_READ_REG(hw, IXGBE_QBRC_H(i)); /* to clear */
        }
    }
    
    hwstats->gprc += IXGBE_READ_REG(hw, IXGBE_GPRC);
    /* work around hardware counting issue */
    hwstats->gprc -= missed_rx;
    
    //ixgbe_update_xoff_received(adapter);
    
    /* 82598 hardware only has a 32 bit counter in the high register */
    switch (hw->mac.type) {
        case ixgbe_mac_82598EB:
            hwstats->lxonrxc += IXGBE_READ_REG(hw, IXGBE_LXONRXC);
            hwstats->gorc += IXGBE_READ_REG(hw, IXGBE_GORCH);
            hwstats->gotc += IXGBE_READ_REG(hw, IXGBE_GOTCH);
            hwstats->tor += IXGBE_READ_REG(hw, IXGBE_TORH);
            break;
        case ixgbe_mac_X540:
        case ixgbe_mac_X550:
        case ixgbe_mac_X550EM_x:
        case ixgbe_mac_x550em_a:
            /* OS2BMC stats are X540 and later */
            hwstats->o2bgptc += IXGBE_READ_REG(hw, IXGBE_O2BGPTC);
            hwstats->o2bspc += IXGBE_READ_REG(hw, IXGBE_O2BSPC);
            hwstats->b2ospc += IXGBE_READ_REG(hw, IXGBE_B2OSPC);
            hwstats->b2ogprc += IXGBE_READ_REG(hw, IXGBE_B2OGPRC);
        case ixgbe_mac_82599EB:
            for (i = 0; i < 16; i++)
                adapter->hw_rx_no_dma_resources +=
                IXGBE_READ_REG(hw, IXGBE_QPRDC(i));
            hwstats->gorc += IXGBE_READ_REG(hw, IXGBE_GORCL);
            IXGBE_READ_REG(hw, IXGBE_GORCH); /* to clear */
            hwstats->gotc += IXGBE_READ_REG(hw, IXGBE_GOTCL);
            IXGBE_READ_REG(hw, IXGBE_GOTCH); /* to clear */
            hwstats->tor += IXGBE_READ_REG(hw, IXGBE_TORL);
            IXGBE_READ_REG(hw, IXGBE_TORH); /* to clear */
            hwstats->lxonrxc += IXGBE_READ_REG(hw, IXGBE_LXONRXCNT);
            hwstats->fdirmatch += IXGBE_READ_REG(hw, IXGBE_FDIRMATCH);
            hwstats->fdirmiss += IXGBE_READ_REG(hw, IXGBE_FDIRMISS);
            break;
        default:
            break;
    }
    bprc = IXGBE_READ_REG(hw, IXGBE_BPRC);
    hwstats->bprc += bprc;
    hwstats->mprc += IXGBE_READ_REG(hw, IXGBE_MPRC);
    
    if (hw->mac.type == ixgbe_mac_82598EB)
        hwstats->mprc -= bprc;
    
    hwstats->roc += IXGBE_READ_REG(hw, IXGBE_ROC);
    hwstats->prc64 += IXGBE_READ_REG(hw, IXGBE_PRC64);
    hwstats->prc127 += IXGBE_READ_REG(hw, IXGBE_PRC127);
    hwstats->prc255 += IXGBE_READ_REG(hw, IXGBE_PRC255);
    hwstats->prc511 += IXGBE_READ_REG(hw, IXGBE_PRC511);
    hwstats->prc1023 += IXGBE_READ_REG(hw, IXGBE_PRC1023);
    hwstats->prc1522 += IXGBE_READ_REG(hw, IXGBE_PRC1522);
    hwstats->rlec += IXGBE_READ_REG(hw, IXGBE_RLEC);
    lxon = IXGBE_READ_REG(hw, IXGBE_LXONTXC);
    hwstats->lxontxc += lxon;
    lxoff = IXGBE_READ_REG(hw, IXGBE_LXOFFTXC);
    hwstats->lxofftxc += lxoff;
    hwstats->gptc += IXGBE_READ_REG(hw, IXGBE_GPTC);
    hwstats->mptc += IXGBE_READ_REG(hw, IXGBE_MPTC);
    /*
     * 82598 errata - tx of flow control packets is included in tx counters
     */
    xon_off_tot = lxon + lxoff;
    hwstats->gptc -= xon_off_tot;
    hwstats->mptc -= xon_off_tot;
    hwstats->gotc -= (xon_off_tot * (ETH_ZLEN + ETH_FCS_LEN));
    hwstats->ruc += IXGBE_READ_REG(hw, IXGBE_RUC);
    hwstats->rfc += IXGBE_READ_REG(hw, IXGBE_RFC);
    hwstats->rjc += IXGBE_READ_REG(hw, IXGBE_RJC);
    hwstats->tpr += IXGBE_READ_REG(hw, IXGBE_TPR);
    hwstats->ptc64 += IXGBE_READ_REG(hw, IXGBE_PTC64);
    hwstats->ptc64 -= xon_off_tot;
    hwstats->ptc127 += IXGBE_READ_REG(hw, IXGBE_PTC127);
    hwstats->ptc255 += IXGBE_READ_REG(hw, IXGBE_PTC255);
    hwstats->ptc511 += IXGBE_READ_REG(hw, IXGBE_PTC511);
    hwstats->ptc1023 += IXGBE_READ_REG(hw, IXGBE_PTC1023);
    hwstats->ptc1522 += IXGBE_READ_REG(hw, IXGBE_PTC1522);
    hwstats->bptc += IXGBE_READ_REG(hw, IXGBE_BPTC);
    
    /* Fill out the OS statistics structures */
    netStats->inputPackets = (UInt32)adapter->stats.gprc;
    netStats->inputErrors = (UInt32)(hwstats->crcerrs + hwstats->rlec);
    netStats->outputPackets = (UInt32)adapter->stats.gptc;
    
    //etherStats->dot3StatsEntry.alignmentErrors = (UInt32)adapter->stats.algnerrc;
    etherStats->dot3StatsEntry.fcsErrors = (UInt32)adapter->stats.crcerrs;
    etherStats->dot3StatsEntry.frameTooLongs = (UInt32)adapter->stats.roc;
    etherStats->dot3StatsEntry.missedFrames = (UInt32)missed_rx;
    
    etherStats->dot3RxExtraEntry.frameTooShorts = (UInt32)adapter->stats.ruc;
}

