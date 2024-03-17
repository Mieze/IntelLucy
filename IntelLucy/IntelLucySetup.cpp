/* IntelLucySetup.cpp -- IntelLucy driver data structure setup.
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
 * Driver for Intel PCIe gigabit ethernet controllers.
 *
 * This driver is based on Intel's ixgbe driver for Linux.
 */

#include "IntelLucy.hpp"

struct mediumTable mediumArray[MIDX_COUNT] = {
    { .type = kIOMediumEthernetAuto, .speed = 10000 * MBit, .idx = MIDX_AUTO },
    
    /* copper media without EEE */
    { .type =(kIOMediumEthernet10BaseT | IFM_FDX), .speed = 10 * MBit, .idx = MIDX_10 },
    { .type =(kIOMediumEthernet10BaseT | IFM_FDX | IFM_FLOW), .speed = 10 * MBit, .idx = MIDX_10FC },
    { .type =(kIOMediumEthernet100BaseTX | IFM_FDX), .speed = 100 * MBit, .idx = MIDX_100 },
    { .type =(kIOMediumEthernet100BaseTX | IFM_FDX | IFM_FLOW), .speed = 100 * MBit, .idx = MIDX_100FC },
    { .type =(kIOMediumEthernet1000BaseT | IFM_FDX), .speed = 1000 * MBit, .idx = MIDX_1000 },
    { .type =(kIOMediumEthernet1000BaseT | IFM_FDX | IFM_FLOW), .speed = 1000 * MBit, .idx = MIDX_1000FC },
    { .type =(kIOMediumEthernet10GBaseT | IFM_FDX), .speed = 10000 * MBit, .idx = MIDX_10GB },
    { .type =(kIOMediumEthernet10GBaseT | IFM_FDX | IFM_FLOW), .speed = 10000 * MBit, .idx = MIDX_10GBFC },
    { .type =(kIOMediumEthernet2500BaseT | IFM_FDX), .speed = 2500 * MBit, .idx = MIDX_2500 },
    { .type =(kIOMediumEthernet2500BaseT | IFM_FDX | IFM_FLOW), .speed = 2500 * MBit, .idx = MIDX_2500FC },
    { .type =(kIOMediumEthernet5000BaseT | IFM_FDX), .speed = 5000 * MBit, .idx = MIDX_5000 },
    { .type =(kIOMediumEthernet5000BaseT | IFM_FDX | IFM_FLOW), .speed = 5000 * MBit, .idx = MIDX_5000FC },
    
    /* copper media with EEE */
    { .type =(kIOMediumEthernet10BaseT | IFM_FDX), .speed = 10 * MBit, .idx = MIDX_10_EEE },
    { .type =(kIOMediumEthernet10BaseT | IFM_FDX | IFM_FLOW), .speed = 10 * MBit, .idx = MIDX_10FC_EEE },
    { .type =(kIOMediumEthernet100BaseTX | IFM_FDX), .speed = 100 * MBit, .idx = MIDX_100_EEE },
    { .type =(kIOMediumEthernet100BaseTX | IFM_FDX | IFM_FLOW), .speed = 100 * MBit, .idx = MIDX_100FC_EEE },
    { .type =(kIOMediumEthernet1000BaseT | IFM_FDX), .speed = 1000 * MBit, .idx = MIDX_1000_EEE },
    { .type =(kIOMediumEthernet1000BaseT | IFM_FDX | IFM_FLOW), .speed = 1000 * MBit, .idx = MIDX_1000FC_EEE },
    { .type =(kIOMediumEthernet10GBaseT | IFM_FDX), .speed = 10000 * MBit, .idx = MIDX_10GB_EEE },
    { .type =(kIOMediumEthernet10GBaseT | IFM_FDX | IFM_FLOW), .speed = 10000 * MBit, .idx = MIDX_10GBFC_EEE },
    { .type =(kIOMediumEthernet2500BaseT | IFM_FDX), .speed = 2500 * MBit, .idx = MIDX_2500_EEE },
    { .type =(kIOMediumEthernet2500BaseT | IFM_FDX | IFM_FLOW), .speed = 2500 * MBit, .idx = MIDX_2500FC_EEE },
    { .type =(kIOMediumEthernet5000BaseT | IFM_FDX), .speed = 5000 * MBit, .idx = MIDX_5000_EEE },
    { .type =(kIOMediumEthernet5000BaseT | IFM_FDX | IFM_FLOW), .speed = 5000 * MBit, .idx = MIDX_5000FC_EEE },

    /* fiber and DAC media */
    { .type =(IFM_1000_SX | IFM_ETHER | IFM_FDX), .speed = 1000 * MBit, .idx = MIDX_1GB_SX },
    { .type =(IFM_1000_SX | IFM_ETHER | IFM_FDX | IFM_FLOW), .speed = 1000 * MBit, .idx = MIDX_1GB_SX_FC },
    { .type =(IFM_10G_SR | IFM_ETHER | IFM_FDX), .speed = 10000 * MBit, .idx = MIDX_10GB_SR },
    { .type =(IFM_10G_SR | IFM_ETHER | IFM_FDX | IFM_FLOW), .speed = 10000 * MBit, .idx = MIDX_10GB_SR_FC },
    { .type =(IFM_1000_LX | IFM_ETHER | IFM_FDX), .speed = 1000 * MBit, .idx = MIDX_1GB_LX },
    { .type =(IFM_1000_LX | IFM_ETHER | IFM_FDX | IFM_FLOW), .speed = 1000 * MBit, .idx = MIDX_1GB_LX_FC },
    { .type =(IFM_10G_LR | IFM_ETHER | IFM_FDX), .speed = 10000 * MBit, .idx = MIDX_10GB_LR },
    { .type =(IFM_10G_LR | IFM_ETHER | IFM_FDX | IFM_FLOW), .speed = 10000 * MBit, .idx = MIDX_10GB_LR_FC },
    { .type =(IFM_10G_TWINAX | IFM_ETHER | IFM_FDX), .speed = 10000 * MBit, .idx = MIDX_10GB_DA },
    { .type =(IFM_10G_TWINAX | IFM_ETHER | IFM_FDX | IFM_FLOW), .speed = 10000 * MBit, .idx = MIDX_10GB_DA_FC },
    { .type =(IFM_10G_TWINAX_LONG | IFM_ETHER | IFM_FDX), .speed = 10000 * MBit, .idx = MIDX_10GB_DAL },
    { .type =(IFM_10G_TWINAX_LONG | IFM_ETHER | IFM_FDX | IFM_FLOW), .speed = 10000 * MBit, .idx = MIDX_10GB_DAL_FC },
};

static const char *onName = "enabled";
static const char *offName = "disabled";

#pragma mark --- data structure initialization methods ---

void IntelLucy::getParams()
{
    OSDictionary *params;
    OSString *versionString;
    OSBoolean *tsoV4;
    OSBoolean *tsoV6;
    OSBoolean *wake;
    OSBoolean *rsc;
    OSBoolean *buf4k;
    OSNumber *tv;
    UInt32 interval;
    bool use4kRxBuf;
    
    versionString = OSDynamicCast(OSString, getProperty(kDriverVersionName));

    params = OSDynamicCast(OSDictionary, getProperty(kParamName));
    
    if (params) {
        tsoV4 = OSDynamicCast(OSBoolean, params->getObject(kEnableTSO4Name));
        enableTSO4 = (tsoV4 != NULL) ? tsoV4->getValue() : false;
        
        IOLog("TCP/IPv4 segmentation offload %s.\n", enableTSO4 ? onName : offName);
        
        tsoV6 = OSDynamicCast(OSBoolean, params->getObject(kEnableTSO6Name));
        enableTSO6 = (tsoV6 != NULL) ? tsoV6->getValue() : false;
        
        IOLog("TCP/IPv6 segmentation offload %s.\n", enableTSO6 ? onName : offName);
        
        wake = OSDynamicCast(OSBoolean, params->getObject(kEnableWakeOnLAN));
        enableWoL = (wake != NULL) ? wake->getValue() : false;
        
        IOLog("Force WoL %s.\n", enableWoL ? onName : offName);

        rsc = OSDynamicCast(OSBoolean, params->getObject(kRxCoalescingName));
        enableRSC = (rsc != NULL) ? rsc->getValue() : false;
        
        IOLog("Receive Side Coalescing %s.\n", enableRSC ? onName : offName);

        buf4k = OSDynamicCast(OSBoolean, params->getObject(kRxBufferSize4kName));
        
        if (buf4k != NULL) {
            use4kRxBuf = buf4k->getValue();
            rxBufferPktSize = (use4kRxBuf || enableRSC) ? kRxBufferPktSize4K : kRxBufferPktSize2K;
        } else {
            use4kRxBuf = false;
        }
        IOLog("4KB receive buffers %s.\n", use4kRxBuf ? onName : offName);

        tv = OSDynamicCast(OSNumber, params->getObject(kPollTime10GName));

        if (tv != NULL) {
            interval = tv->unsigned32BitValue();
            
            if (interval > 100)
                pollTime10G = 100000;
            else if (interval < 10)
                pollTime10G = 10000;
            else
                pollTime10G = interval * 1000;
        } else {
            pollTime10G = 50000;
        }
        tv = OSDynamicCast(OSNumber, params->getObject(kPollTime5GName));

        if (tv != NULL) {
            interval = tv->unsigned32BitValue();
            
            if (interval > 125)
                pollTime5G = 125000;
            else if (interval < 75)
                pollTime5G = 75000;
            else
                pollTime5G = interval * 1000;
        } else {
            pollTime5G = 75000;
        }
        tv = OSDynamicCast(OSNumber, params->getObject(kPollTime2GName));

        if (tv != NULL) {
            interval = tv->unsigned32BitValue();
            
            if (interval > 175)
                pollTime2G = 175000;
            else if (interval < 100)
                pollTime2G = 100000;
            else
                pollTime2G = interval * 1000;
        } else {
            pollTime2G = 130000;
        }
        tv = OSDynamicCast(OSNumber, params->getObject(kRxItrTimeName));

        if (tv != NULL) {
            interval = tv->unsigned32BitValue();
            
            if (interval > IXGBE_ITR_ADAPTIVE_MAX_USECS)
                rxThrottleTime = IXGBE_ITR_ADAPTIVE_MAX_USECS;
            else if (interval < IXGBE_ITR_ADAPTIVE_MIN_USECS)
                rxThrottleTime = 0;
            else
                rxThrottleTime = interval & 0xfe;
        } else {
            rxThrottleTime = 30;
        }
        tv = OSDynamicCast(OSNumber, params->getObject(kTxItrTimeName));

        if (tv != NULL) {
            interval = tv->unsigned32BitValue();
            
            if (interval > IXGBE_ITR_ADAPTIVE_MAX_USECS)
                txThrottleTime = IXGBE_ITR_ADAPTIVE_MAX_USECS;
            else if (interval < IXGBE_ITR_ADAPTIVE_MIN_USECS)
                txThrottleTime = IXGBE_ITR_ADAPTIVE_MIN_USECS;
            else
                txThrottleTime = interval & 0xfe;
        } else {
            txThrottleTime = 60;
        }
    } else {
        /* Use default values in case of missing config data. */
        enableTSO4 = false;
        enableTSO6 = false;
        enableWoL = false;
        enableRSC = false;
        rxBufferPktSize = kRxBufferPktSize2K;
        pollTime10G = 55000;
        pollTime5G = 90000;
        pollTime2G = 130000;
        rxThrottleTime = 30;
        txThrottleTime = 60;
    }
    /* Fix features. */
    if ((rxThrottleTime < IXGBE_MIN_RSC_ITR) ||
        (txThrottleTime < IXGBE_MIN_RSC_ITR)) {
        enableRSC = false;
    }
    if (versionString)
        IOLog("Version %s\n", versionString->getCStringNoCopy());
}

bool IntelLucy::createMediaTable()
{
    IONetworkMedium *medium;
    bool result = false;
    UInt32 i;

    for (i = 0; i < MIDX_COUNT; i++) {
        medium = IONetworkMedium::medium(mediumArray[i].type,
                    mediumArray[i].speed, 0, mediumArray[i].idx);
        
        if (!medium)
            goto error;

        mediaTable[i] = medium;
    }
    result = true;
    
done:
    return result;
    
error:
    for (i = 0; i < MIDX_COUNT; i++)
        RELEASE(mediaTable[i]);
    
    goto done;
}

bool IntelLucy::updateMediumDict()
{
    struct ixgbe_hw *hw = &adapterData.hw;
    OSDictionary *dict;
    UInt32 start, end, i;
    UInt32 start2, end2;
    bool result = false;

    dict = OSDictionary::withCapacity(MIDX_COUNT);
    
    if (dict) {
        /*
         * Always add the default medium first to the mediaDict.
         */
        result = IONetworkMedium::addMedium(dict, mediaTable[MIDX_AUTO]);

        if (!result)
            goto error_medium;

        start2 = end2 = 0;
        /*
         * Select the range of media to add to the dict according
         * to the media type.
         */
        switch (hw->phy.media_type) {
            case ixgbe_media_type_fiber:
            case ixgbe_media_type_fiber_qsfp:
                switch (hw->phy.sfp_type) {
                    case ixgbe_sfp_type_da_cu:
                    case ixgbe_sfp_type_da_cu_core0:
                    case ixgbe_sfp_type_da_cu_core1:
                        start = MIDX_10GB_DA;
                        end = MIDX_10GB_DAL;
                        break;
                        
                    case ixgbe_sfp_type_sr:
                        if (hw->phy.multispeed_fiber) {
                            start = MIDX_1GB_SX;
                            end = MIDX_1GB_LX;
                        } else {
                            start = MIDX_10GB_SR;
                            end = MIDX_1GB_LX;
                        }
                        break;
                        
                    case ixgbe_sfp_type_lr:
                        if (hw->phy.multispeed_fiber) {
                            start = MIDX_1GB_LX;
                            end = MIDX_10GB_DA;
                        } else {
                            start = MIDX_10GB_LR;
                            end = MIDX_10GB_DA;
                        }
                        break;

                    case ixgbe_sfp_type_srlr_core0:
                    case ixgbe_sfp_type_srlr_core1:
                        if (hw->phy.multispeed_fiber) {
                            start = MIDX_1GB_SX;
                            end = MIDX_10GB_DA;
                        } else {
                            start = MIDX_10GB_SR;
                            end = MIDX_1GB_LX;
                            start2 = MIDX_10GB_LR;
                            end2 = MIDX_10GB_DA;
                        }
                        break;
                    
                    case ixgbe_sfp_type_da_act_lmt_core0:
                    case ixgbe_sfp_type_da_act_lmt_core1:
                        start = MIDX_10GB_DAL;
                        end = MIDX_COUNT;
                        break;

                    case ixgbe_sfp_type_1g_cu_core0:
                    case ixgbe_sfp_type_1g_cu_core1:
                        start = MIDX_1000;
                        end = MIDX_10GB;
                        break;
                        
                    case ixgbe_sfp_type_1g_sx_core0:
                    case ixgbe_sfp_type_1g_sx_core1:
                        start = MIDX_1GB_SX;
                        end = MIDX_1GB_LX;
                        break;
                        
                    case ixgbe_sfp_type_1g_lx_core0:
                    case ixgbe_sfp_type_1g_lx_core1:
                        start = MIDX_1GB_LX;
                        end = MIDX_10GB_SR;
                        break;

                    case ixgbe_sfp_type_not_present:
                    case ixgbe_sfp_type_unknown:
                    default:
                        start = 0;
                        end = 0;
                        break;
                }
                break;
                
            case ixgbe_media_type_copper:
                start = MIDX_10;

                /* Only the X550 supports N-Base-T and EEE. */
                if (hw->mac.type <= ixgbe_mac_X540)
                    end = MIDX_2500;
                else
                    end = MIDX_1GB_SX;
                
                break;
                
            default:
                start = 0;
                end = 0;
                IOLog("Unsupported medium found.\n");
                break;
        }
        for (i = start; i < end; i++) {
            result = IONetworkMedium::addMedium(dict, mediaTable[i]);
            
            if (!result)
                goto error_medium;
        }
        for (i = start2; i < end2; i++) {
            result = IONetworkMedium::addMedium(dict, mediaTable[i]);
            
            if (!result)
                goto error_medium;
        }
        result = publishMediumDictionary(dict);
        
        if (!result)
            goto error_medium;
        
        RELEASE(mediaDict);
        mediaDict = dict;
        
        result = true;
    }
done:
    return result;
    
error_medium:
    IOLog("Error creating medium dictionary.\n");
    
    RELEASE(dict);
    goto done;
}

void IntelLucy::updateSelectedMedium()
{
    const IONetworkMedium *medium;
    UInt32 idx;
    
    medium = getSelectedMedium();

    if (medium) {
        idx = medium->getIndex();
        
        if (IONetworkMedium::getMediumWithIndex(mediaDict, idx) == NULL)
            setCurrentMedium(mediaTable[MIDX_AUTO]);
    } else {
        setCurrentMedium(mediaTable[MIDX_AUTO]);
    }
}

bool IntelLucy::initEventSources(IOService *provider)
{
    IOReturn intrResult;
    int msiIndex = -1;
    int intrIndex = 0;
    int intrType = 0;
    bool result = false;
    
    txQueue = reinterpret_cast<IOBasicOutputQueue *>(getOutputQueue());
    
    if (txQueue == NULL) {
        IOLog("Failed to get output queue.\n");
        goto done;
    }
    txQueue->retain();
    
    while ((intrResult = pciDevice->getInterruptType(intrIndex, &intrType)) == kIOReturnSuccess) {
        if (intrType & kIOInterruptTypePCIMessaged){
            msiIndex = intrIndex;
            break;
        }
        intrIndex++;
    }
    if (msiIndex != -1) {
        DebugLog("MSI interrupt index: %d\n", msiIndex);
        
        interruptSource = IOInterruptEventSource::interruptEventSource(this, OSMemberFunctionCast(IOInterruptEventSource::Action, this, &IntelLucy::interruptOccurred), provider, msiIndex);
    }
    if (!interruptSource) {
        IOLog("MSI interrupt could not be enabled.\n");
        goto error_intr;
    }
    workLoop->addEventSource(interruptSource);
    
    timerSource = IOTimerEventSource::timerEventSource(this, OSMemberFunctionCast(IOTimerEventSource::Action, this, &IntelLucy::timerAction));
    
    if (!timerSource) {
        IOLog("Failed to create timer IOTimerEventSource.\n");
        goto error_timer;
    }
    workLoop->addEventSource(timerSource);
    
    result = true;
    
done:
    return result;

error_timer:
    workLoop->removeEventSource(interruptSource);
    RELEASE(interruptSource);
    
error_intr:
    IOLog("Error initializing event sources.\n");
    txQueue->release();
    txQueue = NULL;
    goto done;
}

bool IntelLucy::setupDMADescriptors()
{
    IODMACommand::Segment64 seg;
    UInt64 offset = 0;
    UInt32 numSegs = 1;
    bool result = false;
    
    ixgbeSetNumQueues(&adapterData);

    /* Create transmitter descriptor array. */
    txBufDesc = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task, (kIODirectionInOut | kIOMemoryPhysicallyContiguous | kIOMapInhibitCache), kTxDescMemSize, 0xFFFFFFFFFFFFF000ULL);
    
    if (!txBufDesc) {
        IOLog("Couldn't alloc txBufDesc.\n");
        goto done;
    }
    if (txBufDesc->prepare() != kIOReturnSuccess) {
        IOLog("txBufDesc->prepare() failed.\n");
        goto error_tx_prepare;
    }
    txRing[0].txDescArray = (union ixgbe_adv_tx_desc *)txBufDesc->getBytesNoCopy();
    
    /* I don't know if it's really necessary but the documenation says so and Apple's drivers are also doing it this way. */
    txDescDmaCmd = IODMACommand::withSpecification(kIODMACommandOutputHost64, 64, 0, IODMACommand::kMapped, 0, 1);
    
    if (!txDescDmaCmd) {
        IOLog("Couldn't alloc txDescDmaCmd.\n");
        goto error_tx_dma;
    }
    
    if (txDescDmaCmd->setMemoryDescriptor(txBufDesc) != kIOReturnSuccess) {
        IOLog("setMemoryDescriptor() failed.\n");
        goto error_tx_set_desc;
    }
    
    if (txDescDmaCmd->gen64IOVMSegments(&offset, &seg, &numSegs) != kIOReturnSuccess) {
        IOLog("gen64IOVMSegments() failed.\n");
        goto error_tx_segm;
    }
    /* Now get tx ring's physical address. */
    txPhyAddr = seg.fIOVMAddr;
    txRing[0].txPhyRingAddr = txPhyAddr;
    
    /* Initialize txDescArray. */
    bzero(txRing[0].txDescArray, kTxDescMemSize);
            
    txRing[0].txNextDescIndex = txRing[0].txDirtyIndex = txRing[0].txCleanBarrierIndex = 0;
    txRing[0].txNumFreeDesc = kNumTxDesc;
    txRing[0].regIndex = 0;
    
    txMbufCursor = IOMbufNaturalMemoryCursor::withSpecification(0x4000, kMaxSegs);
    
    if (!txMbufCursor) {
        IOLog("Couldn't create txMbufCursor.\n");
        goto error_tx_segm;
    }
    
    if (!allocTxBufferInfo()) {
        goto error_tx_buf;
    }

    /* Create receiver descriptor array. */
    rxBufDesc = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task, (kIODirectionInOut | kIOMemoryPhysicallyContiguous | kIOMapInhibitCache), kRxDescMemSize, 0xFFFFFFFFFFFFF000ULL);
    
    if (!rxBufDesc) {
        IOLog("Couldn't alloc rxBufDesc.\n");
        goto error_rx_mem;
    }
    
    if (rxBufDesc->prepare() != kIOReturnSuccess) {
        IOLog("rxBufDesc->prepare() failed.\n");
        goto error_rx_prepare;
    }
    rxRing[0].rxDescArray = (union ixgbe_adv_rx_desc *)rxBufDesc->getBytesNoCopy();
    
    /* I don't know if it's really necessary but the documenation says so and Apple's drivers are also doing it this way. */
    rxDescDmaCmd = IODMACommand::withSpecification(kIODMACommandOutputHost64, 64, 0, IODMACommand::kMapped, 0, 1);
    
    if (!rxDescDmaCmd) {
        IOLog("Couldn't alloc rxDescDmaCmd.\n");
        goto error_rx_dma;
    }
    
    if (rxDescDmaCmd->setMemoryDescriptor(rxBufDesc) != kIOReturnSuccess) {
        IOLog("setMemoryDescriptor() failed.\n");
        goto error_rx_set_desc;
    }
    offset = 0;
    numSegs = 1;
    
    if (rxDescDmaCmd->gen64IOVMSegments(&offset, &seg, &numSegs) != kIOReturnSuccess) {
        IOLog("gen64IOVMSegments() failed.\n");
        goto error_rx_segm;
    }
    /* And the rx ring's physical address too. */
    rxPhyAddr = seg.fIOVMAddr;
    
    /* Initialize rxDescArray. */
    bzero((void *)rxRing[0].rxDescArray, kRxDescMemSize);
    rxRing[0].rxCleanedCount = rxRing[0].rxNextDescIndex = 0;
    rxRing[0].regIndex = 0;

    rxMbufCursor = IOMbufNaturalMemoryCursor::withSpecification(PAGE_SIZE, 1);
    
    if (!rxMbufCursor) {
        IOLog("Couldn't create rxMbufCursor.\n");
        goto error_rx_segm;
    }
    result = allocRxBuffers();
    
done:
    return result;
    
error_rx_buf:
    RELEASE(rxMbufCursor);

error_rx_segm:
    rxDescDmaCmd->clearMemoryDescriptor();

error_rx_set_desc:
    RELEASE(rxDescDmaCmd);

error_rx_dma:
    rxBufDesc->complete();

error_rx_prepare:
    RELEASE(rxBufDesc);

error_rx_mem:
    IOFree(txRing[0].txBufArray, kTxBufMemSize);
    txRing[0].txBufArray = NULL;
    
error_tx_buf:
    RELEASE(txMbufCursor);

error_tx_segm:
    txDescDmaCmd->clearMemoryDescriptor();

error_tx_set_desc:
    RELEASE(txDescDmaCmd);

error_tx_dma:
    txBufDesc->complete();

error_tx_prepare:
    RELEASE(txBufDesc);
    goto done;
}

bool IntelLucy::allocTxBufferInfo()
{
    struct ixgbeTxBufferInfo *m;
    UInt32 i, j;
    bool result = false;

    /* Alloc ixgbeTxBufferInfo array. */
    txBufArrayMem = IOMallocZero(kTxBufArraySize);
    m = (struct ixgbeTxBufferInfo *)txBufArrayMem;
    
    if (!txBufArrayMem) {
        IOLog("Couldn't alloc transmit buffer array.\n");
        goto done;
    }
    for (i = 0; i < kNumTxRings; i++) {
        txRing[i].txBufArray = m;
        m += kTxBufArraySize;
    }
    for (i = 0; i < kNumTxRings; i++) {
        for (j = 0; j < kNumTxDesc; j++) {
            txRing[i].txBufArray[j].mbuf = NULL;
            txRing[i].txBufArray[j].numDescs = 0;
            txRing[i].txBufArray[j].packetBytes = 0;
        }
    }
    result = true;
    
done:
    return result;
}

bool IntelLucy::allocRxBuffers()
{
    mbuf_t spareMbuf[kRxNumSpareMbufs];
    mbuf_t m;
    struct ixgbeRxBufferInfo *rm;
    struct ixgbeRxRing *ring;
    IOPhysicalSegment rxSegment;
    UInt32 n;
    UInt32 i, j;
    bool result = false;
    
    /* Alloc ixgbeRxBufferInfo array. */
    rxBufArrayMem = IOMallocZero(kRxBufMemSize);
    rm = (struct ixgbeRxBufferInfo *)rxBufArrayMem;
    
    if (!rxBufArrayMem) {
        IOLog("Couldn't alloc receive buffer array.\n");
        goto done;
    }
    for (i = 0; i < kNumRxRings; i++) {
        rxRing[i].rxBufArray = rm;
        rm += kRxBufArraySize;
    }

    /* Alloc receive buffers. */
    for (j = 0; j < kNumRxRings; j++) {
        ring = &rxRing[j];
        
        for (i = 0; i < kNumRxDesc; i++) {
            m = allocatePacket(rxBufferPktSize);
            
            if (!m) {
                IOLog("Couldn't alloc receive buffer.\n");
                goto error_rx_buf;
            }
            n = rxMbufCursor->getPhysicalSegments(m, &rxSegment, 1);
            
            if (n != 1) {
                freePacket(m);
                IOLog("getPhysicalSegments() for receive buffer failed.\n");
                goto error_rx_buf;
            }
            /* We have to keep the physical address of the buffer too
             * as descriptor write back overwrites it in the descriptor
             * so that it must be refreshed when the descriptor is
             * prepared for reuse.
             */
            ring->rxBufArray[i].mbuf = m;
            ring->rxBufArray[i].phyAddr = rxSegment.location;
            
            ring->rxDescArray[i].read.pkt_addr = OSSwapHostToLittleInt64(rxSegment.location);
            ring->rxDescArray[i].read.hdr_addr = 0;
        }
    }
    /* Allocate some spare mbufs and free them in order to increase the buffer pool.
     * This seems to avoid the replaceOrCopyPacket() errors under heavy load.
     */
    for (i = 0; i < kRxNumSpareMbufs; i++)
        spareMbuf[i] = allocatePacket(rxBufferPktSize);
    
    for (i = 0; i < kRxNumSpareMbufs; i++) {
        if (spareMbuf[i])
            freePacket(spareMbuf[i]);
    }
    result = true;
    
done:
    return result;

error_rx_buf:
    if (rxBufArrayMem) {
        for (j = 0; j < kNumRxRings; j++) {
            ring = &rxRing[j];
            
            for (i = 0; i < kNumRxDesc; i++) {
                if (ring->rxBufArray[i].mbuf) {
                    freePacket(ring->rxBufArray[i].mbuf);
                    ring->rxBufArray[i].mbuf = NULL;
                    ring->rxBufArray[i].phyAddr = 0;
                }
            }
        }
        IOFree(rxBufArrayMem, kRxBufMemSize);
        rxBufArrayMem = NULL;
    }
    goto done;
}

void IntelLucy::freeDMADescriptors()
{
    struct ixgbeRxRing *ring;
    UInt32 i, j;
    
    if (txBufDesc) {
        txBufDesc->complete();
        txBufDesc->release();
        txBufDesc = NULL;
        txPhyAddr = 0;
    }
    if (txDescDmaCmd) {
        txDescDmaCmd->clearMemoryDescriptor();
        txDescDmaCmd->release();
        txDescDmaCmd = NULL;
    }
    RELEASE(txMbufCursor);
    
    if (rxBufDesc) {
        rxBufDesc->complete();
        rxBufDesc->release();
        rxBufDesc = NULL;
        rxPhyAddr = 0;
    }
    if (rxDescDmaCmd) {
        rxDescDmaCmd->clearMemoryDescriptor();
        rxDescDmaCmd->release();
        rxDescDmaCmd = NULL;
    }
    RELEASE(rxMbufCursor);
    
    if (txRing[0].txBufArray) {
        IOFree(txRing[0].txBufArray, kTxBufMemSize);
        txRing[0].txBufArray = NULL;
    }
    if (rxBufArrayMem) {
        for (j = 0; j < kNumRxRings; j++) {
            ring = &rxRing[j];
            
            for (i = 0; i < kNumRxDesc; i++) {
                if (ring->rxBufArray[i].mbuf) {
                    freePacket(ring->rxBufArray[i].mbuf);
                    ring->rxBufArray[i].mbuf = NULL;
                    ring->rxBufArray[i].phyAddr = 0;
                }
            }
        }
        IOFree(rxBufArrayMem, kRxBufMemSize);
        rxBufArrayMem = NULL;
    }
}

void IntelLucy::ixgbeSetupQueueVectors(struct ixgbe_adapter *adapter)
{
    struct ixgbeQueueVector *v;
    UInt16 i, idx;
    
    for (i = 0; i < kNumRxRings; i++) {
        v = &rxRing[i].vector;
        idx = 2 * i;
        
        v->adapter = adapter;
        v->updatePeriod = itrUpdatePeriod;
        v->vIndex = idx;
        v->itr = IXGBE_20K_ITR;
        
        if (adapter->rx_itr_setting == 1) {
            set_ring_adaptive(&rxRing[i]);

            v->itr = IXGBE_20K_ITR;
            v->newItr = (IXGBE_ITR_ADAPTIVE_MAX_USECS |
                         IXGBE_ITR_ADAPTIVE_LATENCY);
        } else {
            v->itr = adapter->rx_itr_setting;
            v->newItr = adapter->rx_itr_setting;
        }
    }
    for (i = 0; i < kNumTxRings; i++) {
        v = &txRing[i].vector;
        idx = 2 * i + 1;
        
        v->adapter = adapter;
        v->updatePeriod = itrUpdatePeriod;
        v->vIndex = idx;
        v->itr = IXGBE_20K_ITR;
        
        if (adapter->tx_itr_setting == 1) {
            set_ring_adaptive(&txRing[i]);

            v->itr = IXGBE_20K_ITR;
            v->newItr = (IXGBE_ITR_ADAPTIVE_MAX_USECS |
                         IXGBE_ITR_ADAPTIVE_LATENCY);
        } else {
            v->itr = adapter->tx_itr_setting;
            v->newItr = adapter->tx_itr_setting;
        }
    }
}
