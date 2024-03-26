/* IntelLucy.cpp -- IntelLucy driver class implementation.
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

#pragma mark --- function prototypes ---

static inline void prepareTSO4(mbuf_t m, UInt32 *ipLength, UInt32 *tcpLength, UInt32 *mss);
static inline void prepareTSO6(mbuf_t m, UInt32 *ipLength, UInt32 *tcpLength, UInt32 *mss);

#pragma mark --- private data ---

static const struct intelDevice deviceTable[] = {
    { .pciDevId = IXGBE_DEV_ID_82598, board_82598, .deviceName = "82598", .deviceInfo = &ixgbe_82598_info },
    { .pciDevId = IXGBE_DEV_ID_82598AF_DUAL_PORT, board_82598, .deviceName = "82598AF dual port", .deviceInfo = &ixgbe_82598_info },
    { .pciDevId = IXGBE_DEV_ID_82598AF_SINGLE_PORT, board_82598, .deviceName = "82598AF single port", .deviceInfo = &ixgbe_82598_info },
    { .pciDevId = IXGBE_DEV_ID_82598AT, board_82598, .deviceName = "82598AT", .deviceInfo = &ixgbe_82598_info },
    { .pciDevId = IXGBE_DEV_ID_82598AT2, board_82598, .deviceName = "82598AT2", .deviceInfo = &ixgbe_82598_info },
    { .pciDevId = IXGBE_DEV_ID_82598EB_CX4, board_82598, .deviceName = "82598EB CX4", .deviceInfo = &ixgbe_82598_info },
    { .pciDevId = IXGBE_DEV_ID_82598_CX4_DUAL_PORT, board_82598, .deviceName = "82598EB CX4 dual port", .deviceInfo = &ixgbe_82598_info },
    { .pciDevId = IXGBE_DEV_ID_82598_DA_DUAL_PORT, board_82598, .deviceName = "82598 DA dual port", .deviceInfo = &ixgbe_82598_info },
    { .pciDevId = IXGBE_DEV_ID_82598_SR_DUAL_PORT_EM, board_82598, .deviceName = "82598 SR dual port EM", .deviceInfo = &ixgbe_82598_info },
    { .pciDevId = IXGBE_DEV_ID_82598EB_XF_LR, board_82598, .deviceName = "82598EB XF LR", .deviceInfo = &ixgbe_82598_info },
    { .pciDevId = IXGBE_DEV_ID_82598EB_SFP_LOM, board_82598, .deviceName = "82598EB SFP", .deviceInfo = &ixgbe_82598_info },
    { .pciDevId = IXGBE_DEV_ID_82598_BX, board_82598, .deviceName = "82598 BX", .deviceInfo = &ixgbe_82598_info },
    { .pciDevId = IXGBE_DEV_ID_82599_KX4, board_82599, .deviceName = "82599 KX4", .deviceInfo = &ixgbe_82599_info },
    { .pciDevId = IXGBE_DEV_ID_82599_XAUI_LOM, board_82599, .deviceName = "82599 XAUI", .deviceInfo = &ixgbe_82599_info },
    { .pciDevId = IXGBE_DEV_ID_82599_KR, board_82599, .deviceName = "82599 KR", .deviceInfo = &ixgbe_82599_info },
    { .pciDevId = IXGBE_DEV_ID_82599_SFP, board_82599, .deviceName = "X520-DA1", .deviceInfo = &ixgbe_82599_info },
    { .pciDevId = IXGBE_DEV_ID_82599_SFP_EM, board_82599, .deviceName = "82599 SFP EM", .deviceInfo = &ixgbe_82599_info },
    { .pciDevId = IXGBE_DEV_ID_82599_KX4_MEZZ, board_82599, .deviceName = "82599 KX4 MEZZ", .deviceInfo = &ixgbe_82599_info },
    { .pciDevId = IXGBE_DEV_ID_82599_CX4, board_82599, .deviceName = "82599 CX4", .deviceInfo = &ixgbe_82599_info },
    { .pciDevId = IXGBE_DEV_ID_82599_BACKPLANE_FCOE, board_82599, .deviceName = "82599 FCOE", .deviceInfo = &ixgbe_82599_info },
    { .pciDevId = IXGBE_DEV_ID_82599_SFP_FCOE, board_82599, .deviceName = "82599 SFP FCOE", .deviceInfo = &ixgbe_82599_info },
    { .pciDevId = IXGBE_DEV_ID_82599_T3_LOM, board_82599, .deviceName = "82599 T3", .deviceInfo = &ixgbe_82599_info },
    { .pciDevId = IXGBE_DEV_ID_82599_COMBO_BACKPLANE, board_82599, .deviceName = "82599 Combo", .deviceInfo = &ixgbe_82599_info },
    { .pciDevId = IXGBE_DEV_ID_X540T, board_X540, .deviceName = "X540-AT2", .deviceInfo = &ixgbe_X540_info },
    { .pciDevId = IXGBE_DEV_ID_82599_SFP_SF2, board_82599, .deviceName = "X520-DA2", .deviceInfo = &ixgbe_82599_info },
    { .pciDevId = IXGBE_DEV_ID_82599_LS, board_82599, .deviceName = "82599 LS", .deviceInfo = &ixgbe_82599_info },
    { .pciDevId = IXGBE_DEV_ID_82599_QSFP_SF_QP, board_82599, .deviceName = "82599 QSFP SF QP", .deviceInfo = &ixgbe_82599_info },
    { .pciDevId = IXGBE_DEV_ID_82599EN_SFP, board_82599, .deviceName = "82599EN SFP", .deviceInfo = &ixgbe_82599_info },
    { .pciDevId = IXGBE_DEV_ID_82599_SFP_SF_QP, board_82599, .deviceName = "82599 SFP SF QP", .deviceInfo = &ixgbe_82599_info },
    { .pciDevId = IXGBE_DEV_ID_X540T1, board_X540, .deviceName = "X540-AT1", .deviceInfo = &ixgbe_X540_info },
    { .pciDevId = IXGBE_DEV_ID_X550T, board_X550, .deviceName = "X550T", .deviceInfo = &ixgbe_X550_info },
    { .pciDevId = IXGBE_DEV_ID_X550T1, board_X550, .deviceName = "X550T1", .deviceInfo = &ixgbe_X550_info },
    { .pciDevId = IXGBE_DEV_ID_X550EM_X_KX4, board_X550EM_x, .deviceName = "X550EM X KX4", .deviceInfo = &ixgbe_X550EM_x_info },
    { .pciDevId = IXGBE_DEV_ID_X550EM_X_XFI, board_X550EM_x, .deviceName = "X550EM X XFI", .deviceInfo = &ixgbe_X550EM_x_info },
    { .pciDevId = IXGBE_DEV_ID_X550EM_X_KR, board_X550EM_x, .deviceName = "X550EM X KR", .deviceInfo = &ixgbe_X550EM_x_info },
    { .pciDevId = IXGBE_DEV_ID_X550EM_X_10G_T, board_X550EM_x, .deviceName = "X550EM X 10G T", .deviceInfo = &ixgbe_X550EM_x_info },
    { .pciDevId = IXGBE_DEV_ID_X550EM_X_SFP, board_X550EM_x, .deviceName = "X550EM X SFP", .deviceInfo = &ixgbe_X550EM_x_info },
    { .pciDevId = IXGBE_DEV_ID_X550EM_X_1G_T, board_x550em_x_fw, .deviceName = "X550EM X 1g T", .deviceInfo = &ixgbe_x550em_x_fw_info},
    { .pciDevId = IXGBE_DEV_ID_X550EM_A_KR, board_x550em_a, .deviceName = "X550EM A KR", .deviceInfo = &ixgbe_x550em_a_info },
    { .pciDevId = IXGBE_DEV_ID_X550EM_A_KR_L, board_x550em_a, .deviceName = "X550EM A KR L", .deviceInfo = &ixgbe_x550em_a_info },
    { .pciDevId = IXGBE_DEV_ID_X550EM_A_SFP_N, board_x550em_a, .deviceName = "X550EM A SFP N", .deviceInfo = &ixgbe_x550em_a_info },
    { .pciDevId = IXGBE_DEV_ID_X550EM_A_SGMII, board_x550em_a, .deviceName = "X550EM A SGMII", .deviceInfo = &ixgbe_x550em_a_info },
    { .pciDevId = IXGBE_DEV_ID_X550EM_A_SGMII_L, board_x550em_a, .deviceName = "X550EM A SGMII L", .deviceInfo = &ixgbe_x550em_a_info },
    { .pciDevId = IXGBE_DEV_ID_X550EM_A_10G_T, board_x550em_a, .deviceName = "X550EM A 10G T", .deviceInfo = &ixgbe_x550em_a_info },
    { .pciDevId = IXGBE_DEV_ID_X550EM_A_SFP, board_x550em_a, .deviceName = "X550EM A SFP", .deviceInfo = &ixgbe_x550em_a_info },
    { .pciDevId = IXGBE_DEV_ID_X550EM_A_1G_T, board_x550em_a_fw, .deviceName = "X550EM A 1G T", .deviceInfo = &ixgbe_x550em_a_fw_info },
    { .pciDevId = IXGBE_DEV_ID_X550EM_A_1G_T_L, board_x550em_a_fw, .deviceName = "X550 EM A 1G T L", .deviceInfo = &ixgbe_x550em_a_fw_info },
    { .pciDevId = 0, .device = 0, .deviceName = NULL, .deviceInfo = NULL }
};

/* Power Management Support */
static IOPMPowerState powerStateArray[kPowerStateCount] =
{
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {1, kIOPMDeviceUsable, kIOPMPowerOn, kIOPMPowerOn, 0, 0, 0, 0, 0, 0, 0, 0}
};

#pragma mark --- public methods ---

OSDefineMetaClassAndStructors(IntelLucy, super)

/* IOService (or its superclass) methods. */

bool IntelLucy::init(OSDictionary *properties)
{
    UInt32 i;
    bool result;
    
    result = super::init(properties);
    
    if (result) {
        workLoop = NULL;
        commandGate = NULL;
        pciDevice = NULL;
        mediaDict = NULL;
        
        for (i = 0; i < MIDX_COUNT; i++)
            mediaTable[i] = NULL;
            
        txQueue = NULL;
        interruptSource = NULL;
        timerSource = NULL;
        netif = NULL;
        netStats = NULL;
        etherStats = NULL;
        baseMap = NULL;
        baseAddr = NULL;
        rxMbufCursor = NULL;
        txMbufCursor = NULL;
        txActiveQueueMask = 0;

        for (i = 0; i < kNumTxRings; i++) {
            txRing[0].txDescArray = NULL;
            txRing[0].txBufArray = NULL;
            txRing[i].state = 0;
        }
        txBufArrayMem = NULL;
        
        for (i = 0; i < kNumRxRings; i++) {
            rxRing[i].rxPacketHead = NULL;
            rxRing[i].rxPacketTail = NULL;
            rxRing[i].rxBufArray = NULL;
            rxRing[i].rxPacketSize = 0;
            rxRing[i].state = 0;
        }
        rxActiveQueueMask = 0;
        rxBufferPktSize = kRxBufferPktSize2K;
        rxBufArrayMem = NULL;
        mcAddrList = NULL;
        mcListCount = 0;

        stateFlags = 0;
        chip = 0;
        powerState = 0;
        pciDeviceData.vendor = 0;
        pciDeviceData.device = 0;
        pciDeviceData.subsystem_vendor = 0;
        pciDeviceData.subsystem_device = 0;
        pciDeviceData.revision = 0;
        adapterData.pdev = &pciDeviceData;
        adapterData.link_up = false;
        mtu = ETH_DATA_LEN;
        enableWoL = false;
        wolActive = false;
        enableTSO4 = false;
        enableTSO6 = false;
        pciPMCtrlOffset = 0;
        enableRSC = false;
        allowUnsupportedSFP = false;
        nanoseconds_to_absolutetime(kTimeout2Gns - 1, &sfpPollTime);
        nanoseconds_to_absolutetime(kTimeout4Gns, &timeoutCheck);
        nanoseconds_to_absolutetime(kTimeout2Gns, &hangTimeout);
        nanoseconds_to_absolutetime(kTimespan4ms, &itrUpdatePeriod);
        rxThrottleTime = 30;
        txThrottleTime = 60;
    }
    
done:
    return result;
}

void IntelLucy::free()
{
    UInt32 i;

    DebugLog("free() ===>\n");
    
    if (workLoop) {
        if (interruptSource) {
            workLoop->removeEventSource(interruptSource);
            RELEASE(interruptSource);
        }
        if (timerSource) {
            workLoop->removeEventSource(timerSource);
            RELEASE(timerSource);
        }
        workLoop->release();
        workLoop = NULL;
    }
    RELEASE(commandGate);
    RELEASE(txQueue);
    RELEASE(mediaDict);
    
    for (i = 0; i < MIDX_COUNT; i++)
        RELEASE(mediaTable[i]);

    RELEASE(baseMap);
    baseAddr = NULL;
    adapterData.hw.hw_addr = NULL;
    
    RELEASE(pciDevice);
    freeDMADescriptors();

    ReleaseMcAddrList();
    
    DebugLog("free() <===\n");
    
    super::free();
}

bool IntelLucy::start(IOService *provider)
{
    int  i;
    bool result;
    
    result = super::start(provider);
    
    if (!result) {
        IOLog("IOEthernetController::start failed.\n");
        goto done;
    }
    mcAddrList = NULL;
    mcListCount = 0;
    clear_mask((__M_CAST_M | __PROMISC_M), &stateFlags);
    
    pciDevice = OSDynamicCast(IOPCIDevice, provider);
    
    if (!pciDevice) {
        IOLog("No provider.\n");
        goto done;
    }
    pciDevice->retain();
    
    if (!pciDevice->open(this)) {
        IOLog("Failed to open provider.\n");
        goto error_open;
    }
    if (!initPCIConfigSpace(pciDevice)) {
        goto error_open;
    }
    getParams();
    
    if (!ixgbeStart(deviceTable)) {
        goto error_open;
    }
    if (!createMediaTable()) {
        IOLog("Failed to setup media table.\n");
        goto error_table;
    }

    if (!updateMediumDict()) {
        IOLog("Failed to setup media dictionary.\n");
        goto error_dict;
    }
    commandGate = getCommandGate();
    
    if (!commandGate) {
        IOLog("getCommandGate() failed.\n");
        goto error_gate;
    }
    commandGate->retain();
    
    if (!setupDMADescriptors()) {
        IOLog("Error allocating DMA descriptors.\n");
        goto error_desc;
    }

    if (!initEventSources(provider)) {
        IOLog("initEventSources() failed.\n");
        goto error_attach;
    }
    
    result = attachInterface(reinterpret_cast<IONetworkInterface**>(&netif));
    
    if (!result) {
        IOLog("attachInterface() failed.\n");
        goto error_attach;
    }
    pciDevice->close(this);
    result = true;
    
done:
    return result;
    
error_attach:
    freeDMADescriptors();
    
error_desc:
    RELEASE(commandGate);
    
error_dict:
    for (i = 0; i < MIDX_COUNT; i++)
        RELEASE(mediaTable[i]);

error_table:
error_gate:
    pciDevice->close(this);
    
error_open:
    pciDevice->release();
    pciDevice = NULL;
    goto done;
}

void IntelLucy::stop(IOService *provider)
{
    struct ixgbe_adapter *adapter = &adapterData;
    
    if (netif) {
        detachInterface(netif);
        netif = NULL;
    }
    if (workLoop) {
        if (interruptSource) {
            workLoop->removeEventSource(interruptSource);
            RELEASE(interruptSource);
        }
        if (timerSource) {
            workLoop->removeEventSource(timerSource);
            RELEASE(timerSource);
        }
        workLoop->release();
        workLoop = NULL;
    }
    RELEASE(commandGate);
    RELEASE(txQueue);
    RELEASE(mediaDict);
    
    freeDMADescriptors();
    
    /* Free resources allocated by ixgbeStart(). */
    ixgbeStop(adapter);
    
    ReleaseMcAddrList();
    
    RELEASE(baseMap);
    baseAddr = NULL;
    adapter->hw.hw_addr = NULL;
        
    RELEASE(pciDevice);
    
    super::stop(provider);
}

IOReturn IntelLucy::registerWithPolicyMaker(IOService *policyMaker)
{
    DebugLog("registerWithPolicyMaker() ===>\n");
    
    powerState = kPowerStateOn;
    
    DebugLog("registerWithPolicyMaker() <===\n");
    
    return policyMaker->registerPowerDriver(this, powerStateArray, kPowerStateCount);
}

IOReturn IntelLucy::setPowerState(unsigned long powerStateOrdinal, IOService *policyMaker)
{
    IOReturn result = IOPMAckImplied;
    
    DebugLog("setPowerState() ===>\n");
    
    if (powerStateOrdinal == powerState) {
        DebugLog("Already in power state %lu.\n", powerStateOrdinal);
        goto done;
    }
    DebugLog("switching to power state %lu.\n", powerStateOrdinal);
    
    if (powerStateOrdinal == kPowerStateOff)
        commandGate->runAction(setPowerStateSleepAction);
    else
        commandGate->runAction(setPowerStateWakeAction);
    
    powerState = powerStateOrdinal;
    
done:
    DebugLog("setPowerState() <===\n");
    
    return result;
}

void IntelLucy::systemWillShutdown(IOOptionBits specifier)
{
    struct ixgbe_adapter *adapter = &adapterData;

    DebugLog("systemWillShutdown() ===>\n");
    
    if ((kIOMessageSystemWillPowerOff | kIOMessageSystemWillRestart) & specifier) {
        disable(netif);
        
        /* Restore the original MAC address. */
        ixgbe_restore_perm_mac(adapter);
        
        /*
         * Let the firmware know that the network interface is now closed
         */
        ixgbe_release_hw_control(adapter);
    }
    
    DebugLog("systemWillShutdown() <===\n");
    
    /* Must call super on shutdown or system will stall. */
    super::systemWillShutdown(specifier);
}

/* IONetworkController methods. */

IOReturn IntelLucy::enable(IONetworkInterface *netif)
{
    IOReturn result = kIOReturnError;
    
    DebugLog("enable() ===>\n");
    
    if (test_bit(__ENABLED, &stateFlags)) {
        DebugLog("Interface already enabled.\n");
        result = kIOReturnSuccess;
        goto done;
    }
    if (!pciDevice || pciDevice->isOpen()) {
        IOLog("Unable to open PCI device.\n");
        goto done;
    }
    if (!pciDevice->open(this)) {
        IOLog("Failed to open PCI device.\n");
        goto done;
    }

    /* As we are using an msi the interrupt hasn't been enabled by start(). */
    interruptSource->enable();
    
    if (!ixgbeEnable(&adapterData)) {
        IOLog("ixgbeEnable() failed.\n");
        goto error_enable;
    }
    set_bit(__ENABLED, &stateFlags);
    clear_bit(__POLL_MODE, &stateFlags);

    result = kIOReturnSuccess;
    
    DebugLog("enable() <===\n");
    
done:
    return result;
    
error_enable:
    pciDevice->close(this);
    goto done;
}

IOReturn IntelLucy::disable(IONetworkInterface *netif)
{
    UInt64 timeout;
    UInt64 delay;
    UInt64 now;
    UInt64 t;

    DebugLog("disable() ===>\n");
    
    if (!test_bit(__ENABLED, &stateFlags))
        goto done;
    
    netif->stopOutputThread();
    netif->flushOutputQueue();
    
    if (test_bit(__POLLING, &stateFlags)) {
        nanoseconds_to_absolutetime(5000, &delay);
        clock_get_uptime(&now);
        timeout = delay * 10;
        t = delay;

        while (test_bit(__POLLING, &stateFlags) && (t < timeout)) {
            clock_delay_until(now + t);
            t += delay;
        }
    }
    clear_mask((__ENABLED_M | __LINK_UP_M | __POLL_MODE_M | __POLLING_M), &stateFlags);
    timerSource->cancelTimeout();

    /* We are using MSI so that we have to disable the interrupt. */
    interruptSource->disable();
    
    ixgbeDisable(&adapterData);

    ReleaseMcAddrList();

    if (pciDevice && pciDevice->isOpen())
        pciDevice->close(this);
    
    DebugLog("disable() <===\n");
    
done:
    return kIOReturnSuccess;
}

IOReturn IntelLucy::outputStart(IONetworkInterface *interface, IOOptionBits options)
{
    IOPhysicalSegment txSegments[kMaxSegs];
    union ixgbe_adv_tx_desc *desc;
    struct ixgbe_adv_tx_context_desc *contDesc;
    mbuf_t m;
    IOReturn result = kIOReturnNoResources;
    UInt64 pktBytes;
    UInt32 numDescs;
    UInt32 vlanMacipLens;
    UInt32 typeTucmdMlhl;
    UInt32 mssL4lenIdx;
    UInt32 cmdTypeLen;
    UInt32 olInfoStatus;
    UInt32 cmd;
    UInt32 ois;
    UInt32 numSegs;
    UInt32 lastSeg;
    UInt32 paylen;
    UInt32 iplen;
    UInt32 tcplen;
    UInt32 mss;
    UInt32 index;
    UInt32 offloadFlags;
    UInt16 vlanTag;
    UInt16 i;
    UInt16 count;
    
    //DebugLog("outputStart() ===>\n");
    count = 0;
    
    if (!(test_mask((__ENABLED_M | __LINK_UP_M), &stateFlags)))  {
        DebugLog("Ethernet Interface down. Dropping packets.\n");
        goto done;
    }
    while ((txRing[0].txNumFreeDesc >= (kMaxSegs + kTxSpareDescs)) && (interface->dequeueOutputPackets(1, &m, NULL, NULL, &pktBytes) == kIOReturnSuccess)) {
        numDescs = 0;
        cmdTypeLen = (IXGBE_TXD_CMD_DEXT | IXGBE_ADVTXD_DTYP_DATA | IXGBE_ADVTXD_DCMD_IFCS);
        olInfoStatus = IXGBE_ADVTXD_CC;
        vlanMacipLens = 0;
        typeTucmdMlhl = (IXGBE_TXD_CMD_DEXT | IXGBE_ADVTXD_DTYP_CTXT);
        mssL4lenIdx = 0;
        tcplen = 0;
        mss = 0;
        offloadFlags = 0;

        /* Get the packet length. */
        paylen = (UInt32)mbuf_pkthdr_len(m);

        if (mbuf_get_tso_requested(m, &offloadFlags, &mss)) {
            DebugLog("Ethernet mbuf_get_tso_requested() failed. Dropping packet.\n");
            freePacket(m);
            continue;
        }
        /* First prepare the header and the command bits. */
        if (offloadFlags & (MBUF_TSO_IPV4 | MBUF_TSO_IPV6)) {
            numDescs = 1;

            if (offloadFlags & MBUF_TSO_IPV4) {
                /*
                 * Fix the pseudo header checksum, get the
                 * TCP header size and set paylen.
                 */
                prepareTSO4(m, &iplen, &tcplen, &mss);

                /* Adjust paylen for TSOv4. */
                paylen -= (kMacHdrLen + iplen + tcplen);

                //DebugLog("Ethernet paylen: %u mss: %u iplen: %u tcplen: %u\n", paylen, mss, iplen, tcplen);

                /* Prepare the context descriptor. */
                vlanMacipLens = ((kMacHdrLen << IXGBE_ADVTXD_MACLEN_SHIFT) | iplen);
                typeTucmdMlhl = (IXGBE_TXD_CMD_DEXT | IXGBE_ADVTXD_DTYP_CTXT |
                                 IXGBE_ADVTXD_TUCMD_L4T_TCP | IXGBE_ADVTXD_TUCMD_IPV4);
                mssL4lenIdx = (((mss & 0x0000ffff) << IXGBE_ADVTXD_MSS_SHIFT) |
                               (tcplen << IXGBE_ADVTXD_L4LEN_SHIFT));
                
                /* Setup the command bits for the data descriptor. */
                cmdTypeLen = (IXGBE_TXD_CMD_DEXT | IXGBE_ADVTXD_DTYP_DATA |
                              IXGBE_ADVTXD_DCMD_TSE | IXGBE_ADVTXD_DCMD_IFCS);
                olInfoStatus = ( IXGBE_ADVTXD_CC | IXGBE_ADVTXD_POPTS_TXSM | IXGBE_ADVTXD_POPTS_IXSM);
            } else {
                olInfoStatus = (IXGBE_ADVTXD_CC | IXGBE_ADVTXD_POPTS_TXSM);

                if ((paylen - kMacHdrLen) > mtu) {
                    /*
                     * Fix the pseudo header checksum, get the
                     * TCP header size and set paylen.
                     */
                    prepareTSO6(m, &iplen, &tcplen, &mss);

                    /* Adjust paylen for TSOv6. */
                    paylen -= (kMacHdrLen + iplen + tcplen);

                    /* Prepare the context descriptor. */
                    vlanMacipLens = ((kMacHdrLen << IXGBE_ADVTXD_MACLEN_SHIFT) | iplen);
                    typeTucmdMlhl = (IXGBE_TXD_CMD_DEXT | IXGBE_ADVTXD_DTYP_CTXT |
                                     IXGBE_ADVTXD_TUCMD_L4T_TCP | IXGBE_ADVTXD_TUCMD_IPV6);
                    mssL4lenIdx = (((mss & 0x0000ffff) << IXGBE_ADVTXD_MSS_SHIFT) |
                                   (tcplen << IXGBE_ADVTXD_L4LEN_SHIFT));
                    
                    /* Setup the command bits for the data descriptor. */
                    cmdTypeLen = (IXGBE_TXD_CMD_DEXT | IXGBE_ADVTXD_DTYP_DATA |
                                  IXGBE_ADVTXD_DCMD_TSE | IXGBE_ADVTXD_DCMD_IFCS);
                } else {
                    /*
                     * There is no need for a TSO6 operation as the packet can be sent in one frame.
                     */
                    offloadFlags = kChecksumTCPIPv6;
                    
                    vlanMacipLens = ((kMacHdrLen << IXGBE_ADVTXD_MACLEN_SHIFT) | kIPv6HdrLen);
                    typeTucmdMlhl = (IXGBE_TXD_CMD_DEXT | IXGBE_ADVTXD_DTYP_CTXT |
                                     IXGBE_ADVTXD_TUCMD_L4T_TCP | IXGBE_ADVTXD_TUCMD_IPV6);

                    cmdTypeLen = (IXGBE_TXD_CMD_DEXT | IXGBE_ADVTXD_DTYP_DATA | IXGBE_ADVTXD_DCMD_IFCS);
                }
            }
            wmb();
        } else {
            mbuf_get_csum_requested(m, &offloadFlags, &mss);

            if (offloadFlags & (kChecksumUDPIPv6 | kChecksumTCPIPv6 | kChecksumIP | kChecksumUDP | kChecksumTCP)) {
                numDescs = 1;
                cmdTypeLen = (IXGBE_TXD_CMD_DEXT | IXGBE_ADVTXD_DTYP_DATA | IXGBE_ADVTXD_DCMD_IFCS);
                olInfoStatus = IXGBE_ADVTXD_CC;
                typeTucmdMlhl = (IXGBE_TXD_CMD_DEXT | IXGBE_ADVTXD_DTYP_CTXT);

                if (offloadFlags & kChecksumTCP) {
                    vlanMacipLens = ((kMacHdrLen << IXGBE_ADVTXD_MACLEN_SHIFT) | kIPv4HdrLen);
                    typeTucmdMlhl |= (IXGBE_ADVTXD_TUCMD_L4T_TCP | IXGBE_ADVTXD_TUCMD_IPV4);
       
                    olInfoStatus |= (IXGBE_ADVTXD_POPTS_TXSM | IXGBE_ADVTXD_POPTS_IXSM);
                } else if (offloadFlags & kChecksumTCPIPv6) {
                    vlanMacipLens = ((kMacHdrLen << IXGBE_ADVTXD_MACLEN_SHIFT) | kIPv6HdrLen);
                    typeTucmdMlhl |= (IXGBE_ADVTXD_TUCMD_L4T_TCP | IXGBE_ADVTXD_TUCMD_IPV6);

                    olInfoStatus |= IXGBE_ADVTXD_POPTS_TXSM;
                } else if (offloadFlags & kChecksumUDP) {
                    vlanMacipLens = ((kMacHdrLen << IXGBE_ADVTXD_MACLEN_SHIFT) | kIPv4HdrLen);
                    typeTucmdMlhl |= (IXGBE_ADVTXD_TUCMD_L4T_UDP | IXGBE_ADVTXD_TUCMD_IPV4);

                    olInfoStatus |= (IXGBE_ADVTXD_POPTS_TXSM | IXGBE_ADVTXD_POPTS_IXSM);
                } else if (offloadFlags & kChecksumUDPIPv6) {
                    vlanMacipLens = ((kMacHdrLen << IXGBE_ADVTXD_MACLEN_SHIFT) | kIPv6HdrLen);
                    typeTucmdMlhl |= (IXGBE_ADVTXD_TUCMD_L4T_UDP | IXGBE_ADVTXD_TUCMD_IPV6);

                    olInfoStatus |= IXGBE_ADVTXD_POPTS_TXSM;
                } else if (offloadFlags & kChecksumIP) {
                    vlanMacipLens = ((kMacHdrLen << IXGBE_ADVTXD_MACLEN_SHIFT) | kIPv4HdrLen);
                    typeTucmdMlhl |= IXGBE_ADVTXD_TUCMD_IPV4;
                    
                    olInfoStatus |= IXGBE_ADVTXD_POPTS_IXSM;
                }
            }
        }
        /* Next get the VLAN tag and command bit. */
        if (!mbuf_get_vlan_tag(m, &vlanTag)) {
            cmdTypeLen |= IXGBE_ADVTXD_DCMD_VLE;
            vlanMacipLens |= ((vlanTag << IXGBE_TX_FLAGS_VLAN_SHIFT) & IXGBE_TX_FLAGS_VLAN_MASK);
        }
        /* Finally get the physical segments. */
        numSegs = txMbufCursor->getPhysicalSegmentsWithCoalesce(m, &txSegments[0], kMaxSegs);
        numDescs += numSegs;
        
        if (!numSegs) {
            DebugLog("Ethernet getPhysicalSegmentsWithCoalesce() failed. Dropping packet.\n");
            etherStats->dot3TxExtraEntry.resourceErrors++;
            freePacket(m);
            continue;
        }
        OSAddAtomic(-numDescs, &txRing[0].txNumFreeDesc);
        index = txRing[0].txNextDescIndex;
        txRing[0].txNextDescIndex = (txRing[0].txNextDescIndex + numDescs) & kTxDescMask;
        lastSeg = numSegs - 1;
        
        /* Setup the context descriptor for TSO or checksum offload. */
        if (offloadFlags) {
            contDesc = (struct ixgbe_adv_tx_context_desc *)&txRing[0].txDescArray[index];
            
            txRing[0].txBufArray[index].mbuf = NULL;
            txRing[0].txBufArray[index].numDescs = 0;
            txRing[0].txBufArray[index].packetBytes = 0;
            
            contDesc->vlan_macip_lens = OSSwapHostToLittleInt32(vlanMacipLens);
            contDesc->fceof_saidx = 0;
            contDesc->type_tucmd_mlhl = OSSwapHostToLittleInt32(typeTucmdMlhl);
            contDesc->mss_l4len_idx = OSSwapHostToLittleInt32(mssL4lenIdx);
            
            ++index &= kTxDescMask;
        }
        /* And finally fill in the data descriptors. */
        if (offloadFlags & (MBUF_TSO_IPV4 | MBUF_TSO_IPV6)) {
            for (i = 0; i < numSegs; i++) {
                desc = &txRing[0].txDescArray[index];
                cmd = (txSegments[i].length & 0x000fffff);
                
                if (i == 0) {
                    cmd |= cmdTypeLen;
                    ois = (olInfoStatus | (paylen << IXGBE_ADVTXD_PAYLEN_SHIFT));
                    
                    txRing[0].txBufArray[index].mbuf = NULL;
                    txRing[0].txBufArray[index].numDescs = 0;
                    txRing[0].txBufArray[index].packetBytes = 0;
                } else if (i == lastSeg) {
                    cmd |= (IXGBE_TXD_CMD_DEXT | IXGBE_ADVTXD_DTYP_DATA |
                            IXGBE_ADVTXD_DCMD_EOP | IXGBE_ADVTXD_DCMD_RS);
                    ois = IXGBE_ADVTXD_CC;

                    txRing[0].txBufArray[index].mbuf = m;
                    txRing[0].txBufArray[index].numDescs = numDescs;
                    txRing[0].txBufArray[index].packetBytes = (UInt32)pktBytes;
                } else {
                    cmd |= IXGBE_TXD_CMD_DEXT | IXGBE_ADVTXD_DTYP_DATA;
                    ois = IXGBE_ADVTXD_CC;

                    txRing[0].txBufArray[index].mbuf = NULL;
                    txRing[0].txBufArray[index].numDescs = 0;
                    txRing[0].txBufArray[index].packetBytes = 0;
                }
                desc->read.buffer_addr = OSSwapHostToLittleInt64(txSegments[i].location);
                desc->read.cmd_type_len = OSSwapHostToLittleInt32(cmd);
                desc->read.olinfo_status = OSSwapHostToLittleInt32(ois);
                
                ++index &= kTxDescMask;
            }
        } else {
            for (i = 0; i < numSegs; i++) {
                desc = &txRing[0].txDescArray[index];
                cmd = (cmdTypeLen | (txSegments[i].length & 0x0000ffff));
                ois = (olInfoStatus | (paylen << IXGBE_ADVTXD_PAYLEN_SHIFT));
                
                if (i == lastSeg) {
                    cmd |= (IXGBE_ADVTXD_DCMD_EOP | IXGBE_ADVTXD_DCMD_RS);
                    txRing[0].txBufArray[index].mbuf = m;
                    txRing[0].txBufArray[index].numDescs = numDescs;
                    txRing[0].txBufArray[index].packetBytes = (UInt32)pktBytes;
                } else {
                    txRing[0].txBufArray[index].mbuf = NULL;
                    txRing[0].txBufArray[index].numDescs = 0;
                    txRing[0].txBufArray[index].packetBytes = 0;
                }
                
                desc->read.buffer_addr = OSSwapHostToLittleInt64(txSegments[i].location);
                desc->read.cmd_type_len = OSSwapHostToLittleInt32(cmd);
                desc->read.olinfo_status = OSSwapHostToLittleInt32(ois);
                
                ++index &= kTxDescMask;
            }
        }
        count++;
    }
    if (count) {
        IXGBE_WRITE_REG(&adapterData.hw, IXGBE_TDT(0), txRing[0].txNextDescIndex);
        txRing[0].txCleanBarrierIndex = txRing[0].txNextDescIndex;
    }
    result = (txRing[0].txNumFreeDesc >= (kMaxSegs + kTxSpareDescs)) ? kIOReturnSuccess : kIOReturnNoResources;
    
    //DebugLog("outputStart() <===\n");
    
done:
    return result;
}

void IntelLucy::getPacketBufferConstraints(IOPacketBufferConstraints *constraints) const
{
    DebugLog("getPacketBufferConstraints() ===>\n");
    
    constraints->alignStart = kIOPacketBufferAlign1;
    constraints->alignLength = kIOPacketBufferAlign1;
    
    DebugLog("getPacketBufferConstraints() <===\n");
}

IOOutputQueue* IntelLucy::createOutputQueue()
{
    DebugLog("createOutputQueue() ===>\n");
    
    DebugLog("createOutputQueue() <===\n");
    
    return IOBasicOutputQueue::withTarget(this, kTransmitQueueCapacity);
}

const OSString* IntelLucy::newVendorString() const
{
    DebugLog("newVendorString() ===>\n");
    
    DebugLog("newVendorString() <===\n");
    
    return OSString::withCString("Intel");
}

const OSString* IntelLucy::newModelString() const
{
    DebugLog("newModelString() ===>\n");
    DebugLog("newModelString() <===\n");
    
    return OSString::withCString(deviceTable[chip].deviceName);
}

bool IntelLucy::configureInterface(IONetworkInterface *interface)
{
    char modelName[kNameLenght];
    IONetworkData *data;
    IOReturn error;

    bool result;
    
    DebugLog("configureInterface() ===>\n");
    
    result = super::configureInterface(interface);
    
    if (!result)
        goto done;
    
    /* Get the generic network statistics structure. */
    data = interface->getParameter(kIONetworkStatsKey);
    
    if (data) {
        netStats = (IONetworkStats *)data->getBuffer();
        
        if (!netStats) {
            IOLog("Error getting IONetworkStats\n.");
            result = false;
            goto done;
        }
    }
    /* Get the Ethernet statistics structure. */
    data = interface->getParameter(kIOEthernetStatsKey);
    
    if (data) {
        etherStats = (IOEthernetStats *)data->getBuffer();
        
        if (!etherStats) {
            IOLog("Error getting IOEthernetStats\n.");
            result = false;
            goto done;
        }
    }
    error = interface->configureOutputPullModel((kNumTxDesc/2), 0, 0, IONetworkInterface::kOutputPacketSchedulingModelNormal);
    
    if (error != kIOReturnSuccess) {
        IOLog("configureOutputPullModel() failed\n.");
        result = false;
        goto done;
    }
    error = interface->configureInputPacketPolling(kNumRxDesc, 0);
    
    if (error != kIOReturnSuccess) {
        IOLog("configureInputPacketPolling() failed\n.");
        result = false;
        goto done;
    }
    snprintf(modelName, kNameLenght, "Intel %s PCIe 10 GBit Ethernet", deviceTable[chip].deviceName);
    setProperty("model", modelName);
    
done:
    DebugLog("configureInterface() <===\n");

    return result;
}

bool IntelLucy::createWorkLoop()
{
    DebugLog("createWorkLoop() ===>\n");
    
    workLoop = IOWorkLoop::workLoop();
    
    DebugLog("createWorkLoop() <===\n");
    
    return workLoop ? true : false;
}

IOWorkLoop* IntelLucy::getWorkLoop() const
{
    DebugLog("getWorkLoop() ===>\n");
    
    DebugLog("getWorkLoop() <===\n");
    
    return workLoop;
}

IOReturn IntelLucy::getChecksumSupport(UInt32 *checksumMask, UInt32 checksumFamily, bool isOutput)
{
    IOReturn result = kIOReturnUnsupported;
    
    DebugLog("getChecksumSupport() ===>\n");
    
    if ((checksumFamily == kChecksumFamilyTCPIP) && checksumMask) {
        if (isOutput) {
            *checksumMask = (kChecksumTCP | kChecksumUDP | kChecksumIP |
                             kChecksumTCPIPv6 | kChecksumUDPIPv6);
        } else {
            *checksumMask = (kChecksumTCP | kChecksumUDP | kChecksumIP |
                             kChecksumTCPIPv6 | kChecksumUDPIPv6);
        }
        result = kIOReturnSuccess;
    }
    DebugLog("getChecksumSupport() <===\n");
    
    return result;
}

UInt32 IntelLucy::getFeatures() const
{
    UInt32 features = (kIONetworkFeatureMultiPages | kIONetworkFeatureHardwareVlan);
    
    DebugLog("getFeatures() ===>\n");
    
    if (enableTSO4)
        features |= kIONetworkFeatureTSOIPv4;
    
    if (enableTSO6)
        features |= kIONetworkFeatureTSOIPv6;
    
    if (enableRSC)
        features |= kIONetworkFeatureLRO;
        
    DebugLog("getFeatures() <===\n");
    
    return features;
}

IOReturn IntelLucy::setWakeOnMagicPacket(bool active)
{
    struct ixgbe_hw *hw = (struct ixgbe_hw *)&adapterData.hw;
    IOReturn result = kIOReturnUnsupported;
    
    DebugLog("setWakeOnMagicPacket() ===>\n");
    
    if (hw->wol_enabled) {
        wolActive = active;
        DebugLog("Wake on magic packet %s.\n", active ? "enabled" : "disabled");
        result = kIOReturnSuccess;
    }
    
    DebugLog("setWakeOnMagicPacket() <===\n");
    
    return result;
}

IOReturn IntelLucy::getPacketFilters(const OSSymbol *group, UInt32 *filters) const
{
    struct ixgbe_hw *hw = (struct ixgbe_hw *)&adapterData.hw;
    IOReturn result = kIOReturnSuccess;
    
    DebugLog("getPacketFilters() ===>\n");
    
    if ((group == gIOEthernetWakeOnLANFilterGroup) && hw->wol_enabled) {
        *filters = kIOEthernetWakeOnMagicPacket;
            
        DebugLog("kIOEthernetWakeOnMagicPacket added to filters.\n");
    } else {
        result = super::getPacketFilters(group, filters);
    }
    
    DebugLog("getPacketFilters() <===\n");
    
    return result;
}

IOReturn IntelLucy::setHardwareAddress(const IOEthernetAddress *addr)
{
    struct ixgbe_adapter *adapter = &adapterData;
    struct ixgbe_mac_addr *mac_table = &adapter->mac_table[0];
    struct ixgbe_hw *hw = &adapter->hw;
    IOReturn result = kIOReturnError;
    
    DebugLog("setHardwareAddress() ===>\n");
    
    if (addr && is_valid_ether_addr(&addr->bytes[0])) {
        memcpy(hw->mac.addr, &addr->bytes[0], kIOEthernetAddressSize);
        memcpy(&mac_table->addr[0], &addr->bytes[0], kIOEthernetAddressSize);
        mac_table->pool = 0;
        mac_table->state = (IXGBE_MAC_STATE_IN_USE | IXGBE_MAC_STATE_DEFAULT);
        hw->mac.ops.set_rar(hw, 0, hw->mac.addr, 0, IXGBE_RAH_AV);
        
        result = kIOReturnSuccess;
    }
    
    DebugLog("setHardwareAddress() <===\n");
    
    return result;
}

/* Methods inherited from IOEthernetController. */
IOReturn IntelLucy::getHardwareAddress(IOEthernetAddress *addr)
{
    struct ixgbe_hw *hw = &adapterData.hw;
    IOReturn result = kIOReturnError;
    
    DebugLog("getHardwareAddress() ===>\n");
        
    if (addr) {
        memcpy(&addr->bytes[0], hw->mac.addr, kIOEthernetAddressSize);
        
        if (is_valid_ether_addr(&addr->bytes[0]))
            result = kIOReturnSuccess;
    }
    
    DebugLog("getHardwareAddress() <===\n");
    
    return result;
}

IOReturn IntelLucy::setPromiscuousMode(bool active)
{
    struct ixgbe_hw *hw = &adapterData.hw;
    u32 fctrl, vmolr = IXGBE_VMOLR_BAM | IXGBE_VMOLR_AUPE;

    DebugLog("setPromiscuousMode() ===>\n");
    
    fctrl = IXGBE_READ_REG(hw, IXGBE_FCTRL);

    /* set all bits that we expect to always be set */
    fctrl &= ~IXGBE_FCTRL_SBP; /* disable store-bad-packets */
    fctrl |= IXGBE_FCTRL_BAM;
    fctrl |= IXGBE_FCTRL_DPF; /* discard pause frames when FC enabled */
    fctrl |= IXGBE_FCTRL_PMCF;

    /* clear the bits we are changing the status of */
    fctrl &= ~(IXGBE_FCTRL_UPE | IXGBE_FCTRL_MPE);
    
    if (active) {
        hw->addr_ctrl.user_set_promisc = true;
        fctrl |= (IXGBE_FCTRL_UPE | IXGBE_FCTRL_MPE);
        vmolr |= IXGBE_VMOLR_MPE;

        DebugLog("Promiscuous mode enabled.\n");
    } else {
        hw->addr_ctrl.user_set_promisc = false;

        DebugLog("Promiscuous mode disabled.\n");
    }
    if (hw->mac.type != ixgbe_mac_82598EB) {
        vmolr |= IXGBE_READ_REG(hw, IXGBE_VMOLR(0)) &
             ~(IXGBE_VMOLR_MPE | IXGBE_VMOLR_ROMPE |
               IXGBE_VMOLR_ROPE);
        IXGBE_WRITE_REG(hw, IXGBE_VMOLR(0), vmolr);
    }
    IXGBE_WRITE_REG(hw, IXGBE_FCTRL, fctrl);

    if (active)
        set_bit(__PROMISC, &stateFlags);
    else
        clear_bit(__PROMISC, &stateFlags);

    DebugLog("setPromiscuousMode() <===\n");
    
    return kIOReturnSuccess;
}

IOReturn IntelLucy::setMulticastMode(bool active)
{
    struct ixgbe_adapter *adapter = &adapterData;
    struct ixgbe_hw *hw = &adapter->hw;

    DebugLog("setMulticastMode() ===>\n");

    if (active) {
        if (hw->mac.ops.update_mc_addr_list)
            hw->mac.ops.update_mc_addr_list(hw, (u8 *)mcAddrList, mcListCount);
    } else {
        if (hw->mac.ops.update_mc_addr_list)
            hw->mac.ops.update_mc_addr_list(hw, (u8 *)mcAddrList, 0);

        /* Disable MTA. */
        IXGBE_WRITE_REG(hw, IXGBE_MCSTCTRL, hw->mac.mc_filter_type);
    }
    if (active)
        set_bit(__M_CAST, &stateFlags);
    else
        clear_bit(__M_CAST, &stateFlags);

    DebugLog("setMulticastMode() <===\n");
    
    return kIOReturnSuccess;
}

IOReturn IntelLucy::setMulticastList(IOEthernetAddress *addrs, UInt32 count)
{
    struct ixgbe_adapter *adapter = &adapterData;
    struct ixgbe_hw *hw = &adapter->hw;
    IOEthernetAddress *newList;
    vm_size_t newSize;
    IOReturn result = kIOReturnNoMemory;

    DebugLog("setMulticastList() ===>\n");

    if (count) {
        newSize = count * sizeof(IOEthernetAddress);
        newList = (IOEthernetAddress *)IOMalloc(newSize);
        
        if (newList) {
            ReleaseMcAddrList();
            
            memcpy(newList, addrs, newSize);
            mcAddrList = newList;
            mcListCount = count;
                        
            /*
             * Write multicast addresses to the
             * MTA hash table.
             */
            if (hw->mac.ops.update_mc_addr_list)
                hw->mac.ops.update_mc_addr_list(hw, (u8 *)addrs, count);
            
            result = kIOReturnSuccess;
        }
    } else {
        ReleaseMcAddrList();
        
        if (hw->mac.ops.update_mc_addr_list)
            hw->mac.ops.update_mc_addr_list(hw, (u8 *)addrs, count);
    }
        
    DebugLog("setMulticastList() <===\n");
    
    return result;
}

IOReturn IntelLucy::selectMedium(const IONetworkMedium *medium)
{
    IOReturn result = kIOReturnSuccess;
    
    DebugLog("selectMedium() ===>\n");

    if (medium) {
        setCurrentMedium(medium);
        ixgbeConfigLink(medium);

        ixgbeUpdateStats(&adapterData);
    }

    DebugLog("selectMedium() <===\n");
    
done:
    return result;
}

IOReturn IntelLucy::getMaxPacketSize(UInt32 * maxSize) const
{
    DebugLog("getMaxPacketSize() ===>\n");
    
    *maxSize = kMaxPacketSize;
    
    DebugLog("getMaxPacketSize() <===\n");
    
    return kIOReturnSuccess;
}

IOReturn IntelLucy::setMaxPacketSize(UInt32 maxSize)
{
    IOReturn result = kIOReturnError;

    DebugLog("setMaxPacketSize() ===>\n");
    
    if (maxSize <= kMaxPacketSize) {
        mtu = maxSize - (ETH_HLEN + ETH_FCS_LEN);

        DebugLog("maxSize: %u, mtu: %u\n", maxSize, mtu);
        
        ixgbeReinit(&adapterData);
        
        result = kIOReturnSuccess;
    }
    
    DebugLog("setMaxPacketSize() <===\n");
    
    return result;
}

#pragma mark --- common interrupt methods ---

void IntelLucy::txCleanRing(struct ixgbeTxRing *ring)
{
    union ixgbe_adv_tx_desc *desc;
    struct ixgbeTxBufferInfo *bufInfo;
    SInt32 bytes = 0;
    SInt32 packets = 0;
    UInt32 batch = 0;
    UInt32 descStatus;
    SInt32 cleaned;
    
    while (ring->txDirtyIndex != ring->txCleanBarrierIndex) {
        desc = &ring->txDescArray[ring->txDirtyIndex];
        bufInfo = &ring->txBufArray[ring->txDirtyIndex];
        
        if (bufInfo->mbuf) {
            descStatus = OSSwapLittleToHostInt32(desc->wb.status);
            
            if (!(descStatus & IXGBE_TXD_STAT_DD))
                goto done;
            
            /* First free the attached mbuf and clean up the buffer info. */
            freePacket(bufInfo->mbuf, kDelayFree);
            bufInfo->mbuf = NULL;
            
            cleaned = bufInfo->numDescs;
            bytes += bufInfo->packetBytes;
            packets += cleaned;
            batch += cleaned;
            bufInfo->numDescs = 0;
            bufInfo->packetBytes = 0;
            ring->txHangSuspected = 0;
            
            /* Finally update the number of free descriptors. */
            OSAddAtomic(cleaned, &ring->txNumFreeDesc);
            OSAddAtomic64(cleaned, &ring->txDescDone);
            
            if (batch > 16) {
                releaseFreePackets();
                batch = 0;
            }
        }
        /* Increment txDirtyIndex. */
        ++ring->txDirtyIndex &= kTxDescMask;
    }
    //DebugLog("txInterrupt txDirtyIndex=%u", ring->txDirtyIndex);
    
done:
    releaseFreePackets();
    
    if (txRing[0].txNumFreeDesc > kTxQueueWakeTreshhold)
        netif->signalOutputThread();
    
    if (ring_is_adaptive(ring)) {
        OSAddAtomic(packets, &ring->vector.totalPackets);
        OSAddAtomic(bytes, &ring->vector.totalBytes);
    }
}

UInt32 IntelLucy::rxCleanRing(IONetworkInterface *interface, struct ixgbeRxRing *ring,
                              uint32_t maxCount, IOMbufQueue *pollQueue, void *context)
{
    IOPhysicalSegment rxSegment;
    union ixgbe_adv_rx_desc *desc = &ring->rxDescArray[ring->rxNextDescIndex];
    struct ixgbe_hw *hw = &adapterData.hw;
    struct ixgbeRxBufferInfo *bufInfo;
    mbuf_t bufPkt, newPkt;
    UInt32 pktCnt = 0;
    UInt32 pktBytes = 0;
    UInt32 status;
    UInt32 pktSize;
    UInt32 pktType;
    UInt32 n;
    UInt16 vlanTag;
    UInt16 nextIdx;
    UInt8 regIndex = ring->regIndex;
    UInt8 rscCnt;
    bool replaced;

    while (((status = OSSwapLittleToHostInt32(desc->wb.upper.status_error)) & IXGBE_RXD_STAT_DD) && (pktCnt < maxCount)) {
        bufInfo = &ring->rxBufArray[ring->rxNextDescIndex];
        bufPkt = bufInfo->mbuf;
        
        pktType = OSSwapLittleToHostInt32(desc->wb.lower.lo_dword.data);
        pktSize = OSSwapLittleToHostInt16(desc->wb.upper.length);
        vlanTag = (status & IXGBE_RXD_STAT_VP) ? (OSSwapBigToHostInt16(desc->wb.upper.vlan)) : 0;
        rscCnt = (pktType & IXGBE_RXDADV_RSCCNT_MASK) >> IXGBE_RXDADV_RSCCNT_SHIFT;

        pktBytes += pktSize;
        pktCnt++;
        
        /* Skip bad packet. */
        if (status & IXGBE_RXDADV_ERR_USE) {
            DebugLog("Bad packet.\n");
            etherStats->dot3StatsEntry.internalMacReceiveErrors++;
            goto error_drop;
        }
        newPkt = replaceOrCopyPacket(&bufPkt, pktSize, &replaced);
        
        if (!newPkt) {
            /* Allocation of a new packet failed so that we must leave the original packet in place. */
            DebugLog("replaceOrCopyPacket() failed.\n");
            etherStats->dot3RxExtraEntry.resourceErrors++;
            goto error_drop;
        }
        
        /* If the packet was replaced we have to update the descriptor's buffer address. */
        if (replaced) {
            n = rxMbufCursor->getPhysicalSegments(bufPkt, &rxSegment, 1);

            if (n != 1) {
                DebugLog("getPhysicalSegments() failed.\n");
                etherStats->dot3RxExtraEntry.resourceErrors++;
                freePacket(bufPkt);
                goto error_drop;
            }
            bufInfo->mbuf = bufPkt;
            bufInfo->phyAddr = rxSegment.location;
        }
        /* Set the length of the buffer. */
        mbuf_setlen(newPkt, pktSize);

        if (status & IXGBE_RXD_STAT_EOP) {
            if (!rscCnt) {
                if (ring->rxPacketHead) {
                    /* This is the last buffer of a jumbo frame. */
                    mbuf_setflags_mask(newPkt, 0, MBUF_PKTHDR);
                    mbuf_setnext(ring->rxPacketTail, newPkt);
                    
                    pktSize += ring->rxPacketSize;
                    newPkt = ring->rxPacketHead;
                }
                mbuf_pkthdr_setlen(newPkt, pktSize);

                ring->rxPacketHead = ring->rxPacketTail = NULL;
                ring->rxPacketSize = 0;
            } else {
                /* Handle RSC of EOP descriptor. */
                if (bufInfo->rscHead) {
                    mbuf_setflags_mask(newPkt, 0, MBUF_PKTHDR);
                    mbuf_setnext(bufInfo->rscTail, newPkt);
                    mbuf_pkthdr_adjustlen(bufInfo->rscHead, pktSize);
                    
                    newPkt = bufInfo->rscHead;
                    bufInfo->rscHead = NULL;
                    bufInfo->rscTail = NULL;
                } else {
                    mbuf_pkthdr_setlen(newPkt, pktSize);
                }
            }
            ixgbeGetChecksumResult(newPkt, status);

            /* Also get the VLAN tag if there is any. */
            if (vlanTag)
                setVlanTag(newPkt, vlanTag);

            interface->enqueueInputPacket(newPkt, pollQueue);
        } else {
            if (!rscCnt) {
                if (ring->rxPacketHead) {
                    /*
                     * We are in the middle of a packet spanning multiple buffers.
                     */
                    mbuf_setflags_mask(newPkt, 0, MBUF_PKTHDR);
                    mbuf_setnext(ring->rxPacketTail, newPkt);

                    ring->rxPacketTail = newPkt;
                    ring->rxPacketSize += pktSize;
                } else {
                    /* This is the first buffer of a packet which spans multiple buffers. */
                    ring->rxPacketHead = ring->rxPacketTail = newPkt;
                    ring->rxPacketSize = pktSize;
                }
            } else {
                /* Handle RSC of non EOP descriptor. */
                nextIdx = (status & IXGBE_RXDADV_NEXTP_MASK) >> IXGBE_RXDADV_NEXTP_SHIFT;

                if (bufInfo->rscHead) {
                    /* Append to an existing RSC buffer chain. */
                    mbuf_setflags_mask(newPkt, 0, MBUF_PKTHDR);
                    mbuf_setnext(bufInfo->rscTail, newPkt);
                    mbuf_pkthdr_adjustlen(bufInfo->rscHead, pktSize);
                    
                    ring->rxBufArray[nextIdx].rscHead = bufInfo->rscHead;
                    ring->rxBufArray[nextIdx].rscTail = newPkt;

                    bufInfo->rscHead = NULL;
                    bufInfo->rscTail = NULL;
                } else {
                    /* Start a new RSC buffer chain. */
                    mbuf_pkthdr_setlen(newPkt, pktSize);
                    
                    ring->rxBufArray[nextIdx].rscHead = newPkt;
                    ring->rxBufArray[nextIdx].rscTail = newPkt;
                }
             }
        }
        
        /* Finally update the descriptor and get the next one to examine. */
    nextDesc:
        desc->read.hdr_addr = 0;
        desc->read.pkt_addr = OSSwapHostToLittleInt64(bufInfo->phyAddr);
        
        ++ring->rxNextDescIndex &= kRxDescMask;
        desc = &ring->rxDescArray[ring->rxNextDescIndex];
        ring->rxCleanedCount++;
    }
    /*
     * Return descriptors to hardware as a batch
     * to improve efficiency.
     */
    if (ring->rxCleanedCount >= IXGBE_RX_BUFFER_WRITE) {
        /*
         * Prevent the tail from reaching the head in order to avoid a false
         * buffer queue full condition.
         */
        IXGBE_WRITE_REG(hw, IXGBE_RDT(regIndex), (ring->rxNextDescIndex - 1) & kRxDescMask);
        
        ring->rxCleanedCount = 0;
    }
    if (ring_is_adaptive(ring)) {
        OSAddAtomic((SInt32)pktBytes, &ring->vector.totalBytes);
        OSAddAtomic((SInt32)pktCnt, &ring->vector.totalPackets);
    }

    //DebugLog("rxInterrupt goodPkts=%u", pktCnt);

    return pktCnt;
    
error_drop:
    ixgbeDropPktFragment(ring);
    
    if (bufInfo->rscHead) {
        freePacket(bufInfo->rscHead);
        bufInfo->rscHead = NULL;
        bufInfo->rscTail = NULL;
    }
    goto nextDesc;
}

void IntelLucy::interruptOccurred(OSObject *client, IOInterruptEventSource *src, int count)
{
    struct ixgbe_adapter *adapter = &adapterData;
    struct ixgbe_hw *hw = &adapter->hw;
    UInt64 qmask = 0;
    UInt32 eicr;
    UInt32 packets;
    
    /*
     * Workaround for silicon errata #26 on 82598.  Mask the interrupt
     * before the read of EICR.
     */
    IXGBE_WRITE_REG(hw, IXGBE_EIMC, IXGBE_IRQ_CLEAR_MASK);
    
    eicr = IXGBE_READ_REG(hw, IXGBE_EICR);
    
    if (!test_bit(__POLL_MODE, &stateFlags) &&
        !test_and_set_bit(__POLLING, &stateFlags)) {
        qmask = rxActiveQueueMask | txActiveQueueMask;

        if (eicr & rxActiveQueueMask) {
            packets = rxCleanRing(netif, &rxRing[0], kNumRxDesc, NULL, NULL);
            
            if (packets)
                netif->flushInputQueue();
            
            etherStats->dot3RxExtraEntry.interrupts++;
        }
        if (eicr & txActiveQueueMask) {
            txCleanRing(&txRing[0]);
            etherStats->dot3TxExtraEntry.interrupts++;
        }
        clear_bit(__POLLING, &stateFlags);
    }
    
    if (eicr & IXGBE_EICR_LSC) {
        adapter->lsc_int++;
        adapter->flags |= IXGBE_FLAG_NEED_LINK_UPDATE;
        clock_get_uptime((uint64_t *)&adapter->link_check_timeout);
        
        if (!test_bit(__IXGBE_DOWN, &adapter->state)) {
            IXGBE_WRITE_REG(hw, IXGBE_EIMC, IXGBE_EIMC_LSC);
            IXGBE_WRITE_FLUSH(hw);
            
            ixgbeServiceEventSchedule(adapter);
        }
    }
    switch (hw->mac.type) {
        case ixgbe_mac_82599EB:
            ixgbeCheckSfpEvent(adapter, eicr);
            /* fallthrough */
        case ixgbe_mac_X540:
        case ixgbe_mac_X550:
        case ixgbe_mac_X550EM_x:
        case ixgbe_mac_x550em_a:
            if (eicr & IXGBE_EICR_ECC) {
                IOLog("Uncorrectable ECC error. Reseting chip.\n");
                set_bit(__IXGBE_RESET_REQUESTED, &adapter->state);
                etherStats->dot3TxExtraEntry.resets++;
                IXGBE_WRITE_REG(hw, IXGBE_EICR, IXGBE_EICR_ECC);
                
                /* Reset on uncorrectable ECC error */
                ixgbeServiceEventSchedule(adapter);
            }
            ixgbeCheckOvertempEvent(adapter, eicr);
            break;
            
        default:
            break;
    }
    
    /*
     * Re-enable link(maybe) and non-queue interrupts, no flush.
     * Re-enable tx/rx interupts, provided we aren't in poll mode.
     */
    if (adapter->rx_itr_setting == 1)
        ixgbe_set_itr(&rxRing[0].vector);

    if (adapter->tx_itr_setting == 1)
        ixgbe_set_itr(&txRing[0].vector);

    if (!test_bit(__IXGBE_DOWN, &adapter->state)) {
        ixgbe_irq_enable(adapter, qmask, false);
    }
}


#pragma mark --- rx poll methods ---

IOReturn IntelLucy::setInputPacketPollingEnable(IONetworkInterface *interface, bool enabled)
{
    struct ixgbe_adapter *adapter = &adapterData;
    struct ixgbe_hw *hw = &adapter->hw;
    UInt32 mask = 0;

    //DebugLog("setInputPacketPollingEnable() ===>\n");
    
    if (test_bit(__ENABLED, &stateFlags)) {
        mask = rxActiveQueueMask | txActiveQueueMask;
        
        if (enabled) {
            set_bit(__POLL_MODE, &stateFlags);
            IXGBE_WRITE_REG(hw, IXGBE_EIMC, mask);
        } else {
            clear_bit(__POLL_MODE, &stateFlags);
            IXGBE_WRITE_REG(hw, IXGBE_EIMS, mask);
        }
        IXGBE_WRITE_FLUSH(hw);
    }
    DebugLog("Input polling %s.\n", enabled ? "enabled" : "disabled");
    
    //DebugLog("setInputPacketPollingEnable() <===\n");
    
    return kIOReturnSuccess;
}

void IntelLucy::pollInputPackets(IONetworkInterface *interface, uint32_t maxCount, IOMbufQueue *pollQueue, void *context )
{
    //DebugLog("pollInputPackets() ===>\n");
    if (test_bit(__POLL_MODE, &stateFlags) &&
        !test_and_set_bit(__POLLING, &stateFlags)) {
        rxCleanRing(interface, &rxRing[0], maxCount, pollQueue, context);
        
        /* Finally cleanup the transmitter ring. */
        txCleanRing(&txRing[0]);
        
        clear_bit(__POLLING, &stateFlags);
    }
    
    //DebugLog("pollInputPackets() <===\n");
}

#pragma mark --- timer action methods ---

void IntelLucy::timerAction(IOTimerEventSource *timer)
{
    struct ixgbe_adapter *adapter = &adapterData;
    UInt32 nextTimeout;
        
    /* poll faster when waiting for link */
    if (adapter->flags & IXGBE_FLAG_NEED_LINK_UPDATE) {
        nextTimeout = kTimeout100ms;
    } else {
        nextTimeout = kTimeout2000ms;
    }
    /* Restart the timer. */
    timer->setTimeoutMS(nextTimeout);
    
    /* Run the service task, provided it has been scheduled. */
    if (!test_bit(__IXGBE_DOWN, &adapter->state) &&
        !test_bit(__IXGBE_REMOVING, &adapter->state)) {
        /* Schedule service task to run.*/
        set_bit(__IXGBE_SERVICE_SCHED, &adapter->state);

        ixgbeServiceTask();
    }
}

#pragma mark --- other methods ---

inline void IntelLucy::ixgbeGetChecksumResult(mbuf_t m, UInt32 status)
{
    mbuf_csum_performed_flags_t performed = 0;
    UInt32 value = 0;

    if ((status & (IXGBE_RXD_STAT_IPCS | IXGBE_RXDADV_ERR_IPE)) == IXGBE_RXD_STAT_IPCS)
        performed |= (MBUF_CSUM_DID_IP | MBUF_CSUM_IP_GOOD);
    
    if ((status & (IXGBE_RXD_STAT_L4CS | IXGBE_RXDADV_ERR_TCPE)) == IXGBE_RXD_STAT_L4CS){
        performed |= (MBUF_CSUM_DID_DATA | MBUF_CSUM_PSEUDO_HDR);
        value = 0xffff; // fake a valid checksum value
    }
    if (performed)
        mbuf_set_csum_performed(m, performed, value);
}

bool IntelLucy::ixgbeIdentifyChip()
{
    struct ixgbe_hw *hw;

    UInt32 i = 0;
    UInt16 id = deviceTable[i].pciDevId;
    bool result = false;
    
    while (id) {
        if (id == pciDeviceData.device) {
            chip = i;
            hw = &adapterData.hw;
            hw->back = &adapterData;
            hw->hw_addr = (volatile UInt8*)baseAddr;
            adapterData.io_addr = hw->hw_addr;
            
            result = true;
            break;
        }
        id = deviceTable[++i].pciDevId;
    }
    
done:
    return result;
}

#pragma mark --- miscellaneous functions ---
/*
static inline unsigned add32_with_carry(unsigned a, unsigned b)
{
    asm("addl %2,%0\n\t"
        "adcl $0,%0"
        : "=r" (a)
        : "0" (a), "rm" (b));
    return a;
}
*/
static inline void prepareTSO4(mbuf_t m, UInt32 *ipLength, UInt32 *tcpLength, UInt32 *mss)
{
    UInt8 *p = (UInt8 *)mbuf_data(m) + kMacHdrLen;
    struct ip4_hdr_be *ip = (struct ip4_hdr_be *)p;
    struct tcp_hdr_be *tcp;
    UInt32 csum32 = 6;
    UInt32 max;
    UInt32 i, il, tl;
    
    for (i = 0; i < 4; i++) {
        csum32 += ntohs(ip->addr[i]);
        csum32 += (csum32 >> 16);
        csum32 &= 0xffff;
    }
    il = ((ip->hdr_len & 0x0f) << 2);
    
    tcp = (struct tcp_hdr_be *)(p + il);
    tl = ((tcp->dat_off & 0xf0) >> 2);
    max = 1600 - (il + tl);

    /* Fill in the pseudo header checksum for TSOv4. */
    tcp->csum = htons((UInt16)csum32);

    *ipLength = il;
    *tcpLength = tl;

    if (*mss > max)
        *mss = max;
}

static void prepareTSO6(mbuf_t m, UInt32 *ipLength, UInt32 *tcpLength, UInt32 *mss)
{
    UInt8 *p = (UInt8 *)mbuf_data(m) + kMacHdrLen;
    struct ip6_hdr_be *ip6 = (struct ip6_hdr_be *)p;
    struct tcp_hdr_be *tcp;
    UInt32 csum32 = 6;
    UInt32 i, tl;
    
    ip6->pay_len = 0;

    for (i = 0; i < 16; i++) {
        csum32 += ntohs(ip6->addr[i]);
        csum32 += (csum32 >> 16);
        csum32 &= 0xffff;
    }
    /* Get the length of the TCP header. */
    tcp = (struct tcp_hdr_be *)(p + kIPv6HdrLen);
    tl = ((tcp->dat_off & 0xf0) >> 2);

    /* Fill in the pseudo header checksum for TSOv6. */
    tcp->csum = htons((UInt16)csum32);

    *ipLength = kIPv6HdrLen;
    *tcpLength = tl;
}
