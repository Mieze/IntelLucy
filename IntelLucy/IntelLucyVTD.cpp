/* IntelLucyVTD.cpp -- IntelLucy driver class implementation.
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
 * Driver for Intel PCIe 10Gbit ethernet controllers.
 *
 * This driver is based on Intel's ixgbe driver for Linux.
 */

#include "IntelLucy.hpp"

#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#endif /* MIN */

#define next_page(x) trunc_page(x + PAGE_SIZE)

#pragma mark --- interrupt methods for AppleVTD support ---

void IntelLucy::interruptOccurredVTD(OSObject *client, IOInterruptEventSource *src, int count)
{
    struct ixgbe_adapter *adapter = &adapterData;
    struct ixgbe_hw *hw = &adapter->hw;
    UInt64 qmask = 0;
    UInt32 packets = 0;
    UInt32 eicr;
    
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
            packets = rxCleanRingVTD(netif, &rxRing[0], kNumRxDesc, NULL, NULL);
            
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
        ixgbe_set_rx_itr(&rxRing[0].vector);

    if (adapter->tx_itr_setting == 1)
        ixgbe_set_tx_itr(&txRing[0].vector);

    if (!test_bit(__IXGBE_DOWN, &adapter->state)) {
        ixgbe_irq_enable(adapter, qmask, false);
    }
}

#pragma mark --- tx methods for AppleVTD support ---

UInt32 IntelLucy::txMapPacket(mbuf_t packet,
                            IOPhysicalSegment *vector,
                            UInt32 maxSegs,
                            struct ixgbeTxRing *ring)
{
    IOMemoryDescriptor *md = NULL;
    struct ixgbeTxMapInfo *map;
    IOAddressRange *srcRange;
    IOAddressRange *dstRange;
    mbuf_t m;
    IOVirtualAddress d;
    IOByteCount offset;
    UInt64 len, l;
    UInt32 segIndex = 0;
    UInt32 i;
    UInt16 saveMem;
    bool result = false;

    if (packet && vector && maxSegs && ring) {
        map = ring->txMapInfo;
        srcRange = map->txSCRange;
        m = packet;
        
        /*
         * Split up the packet into virtual contiguos segments.
         */
        if (mbuf_next(m) == 0) {
            d = (IOVirtualAddress)mbuf_data(m);
            len = mbuf_len(m);
            
            if ( trunc_page(d) == trunc_page(d + len - 1) ) {
                srcRange[0].address = d;
                srcRange[0].length = len;
                segIndex = 1;
                goto map;
            }
        }
        do {
            d = (IOVirtualAddress)mbuf_data(m);
            
            for (len = mbuf_len(m); len; d += l, len -= l) {
                l = MIN(len, PAGE_SIZE);
                l = MIN(next_page(d), d + l) - d;
                
                if (segIndex < maxSegs) {
                    srcRange[segIndex].address = d;
                    srcRange[segIndex].length = l;
                } else {
                    segIndex = 0;
                    goto done;
                }
                segIndex++;
            }
            m = mbuf_next(m);
        } while (m);
map:
        /*
         * Get IORanges, fill in the virtual segments and grab
         * an IOMemoryDescriptor to map the packet.
         */
        if (map->txNumFreeMem > 1) {
            dstRange = &map->txMemRange[ring->txNextDescIndex];
            
            for (i = 0; i < segIndex; i++) {
                dstRange[i].address = (srcRange[i].address & ~PAGE_MASK);
                dstRange[i].length = PAGE_SIZE;
                srcRange[i].address &= PAGE_MASK;
            }

            OSAddAtomic16(-1, &map->txNumFreeMem);
            saveMem = map->txNextMem2Use++;
            map->txNextMem2Use &= kTxMemDescMask;
            md = map->txMemIO[saveMem];
            
            if (md) {
                result = md->initWithOptions(dstRange, segIndex, 0, kernel_task, (kIOMemoryTypeVirtual | kIODirectionOut | kIOMemoryAsReference), mapper);
            } else {
                md = IOMemoryDescriptor::withAddressRanges(dstRange, segIndex, (kIOMemoryTypeVirtual | kIODirectionOut | kIOMemoryAsReference), kernel_task);
                
                if (!md) {
                    DebugLog("Couldn't alloc IOMemoryDescriptor for tx packet.");
                    goto error_map;
                }
                map->txMemIO[saveMem] = md;
                result = true;
            }
            if (!result) {
                DebugLog("Failed to init IOMemoryDescriptor for tx packet.");
                goto error_map;
            }
            if (md->prepare() != kIOReturnSuccess) {
                DebugLog("Failed to prepare() tx packet.");
                goto error_map;
            }
            md->setTag(kIOMemoryActive);
            offset = 0;

            /*
             * Get the physical segments and fill in the vector.
             */
            for (i = 0; i < segIndex; i++) {
                vector[i].location = md->getPhysicalSegment(offset, NULL) + srcRange[i].address;
                vector[i].length = srcRange[i].length;

                //DebugLog("Phy. Segment %u addr: %llx, len: %llu\n", i, vector[i].location, vector[i].length);
                offset += PAGE_SIZE;
            }
        }
    }
done:
    return segIndex;

error_map:
    map->txNextMem2Use = saveMem;
    OSAddAtomic16(1, &map->txNumFreeMem);

    segIndex = 0;
    goto done;
}

/*
 * Unmap a tx packet. Complete the IOMemoryDecriptor and free it
 * for reuse.
 */
void IntelLucy::txUnmapPacket(struct ixgbeTxRing *ring)
{
    if (ring) {
        struct ixgbeTxMapInfo *map = ring->txMapInfo;
        IOMemoryDescriptor *md = map->txMemIO[map->txNextMem2Free];
        
        md->complete();
        md->setTag(kIOMemoryInactive);
        
        ++(map->txNextMem2Free) &= kTxMemDescMask;
        OSAddAtomic16(1, &map->txNumFreeMem);
    }
}

#pragma mark --- rx methods for AppleVTD support ---

/*
 * Unmap a batch of rx buffers, replace them with new ones and map them.
 * @ring        The ring to map for
 * @index       The index of the first buffer in a batch to map.
 * @count       Number of batches to map.
 * @updateType  The update strategy for the ring.
 * @result      The index of the next batch to map.
 */
UInt16 IntelLucy::rxMapBuffers(struct ixgbeRxRing *ring, UInt16 index,
                               UInt16 count, bool update)
{
    union ixgbe_adv_rx_desc *desc = ring->rxDescArray;
    ixgbeRxBufferInfo *bufInfo = ring->rxBufArray;
    ixgbeRxMapInfo *mapInfo = ring->rxMapInfo;
    struct ixgbe_hw *hw = &adapterData.hw;
    IOPhysicalAddress pa;
    IOMemoryDescriptor *md;
    IOByteCount offset;
    UInt32 batch = count;
    UInt32 rdt = 0;
    UInt16 end, i;
    bool result;
    
    while (batch--) {
        /*
         * Get the coresponding IOMemoryDescriptor and complete
         * the mapping;
         */
        md = mapInfo->rxMemIO[index >> kRxMemBaseShift];
        md->complete();
        
        /*
         * Update IORanges with the addresses of the replaced buffers.
         */
        for (i = index, end = index + kRxMemBatchSize; i < end; i++) {
            if (bufInfo[i].phyAddr == 0) {
                mapInfo->rxMemRange[i].address = (IOVirtualAddress)mbuf_datastart(bufInfo[i].mbuf);
            }
        }
        /*
         * Prepare IOMemoryDescriptor with updated buffer addresses.
         */
        result = md->initWithOptions(&mapInfo->rxMemRange[index], kRxMemBatchSize, 0, kernel_task, kIOMemoryTypeVirtual | kIODirectionIn | kIOMemoryAsReference, mapper);

        if (!result) {
            IOLog("Failed to reinit rx IOMemoryDescriptor.\n");
            goto done;
        }
        if (md->prepare() != kIOReturnSuccess) {
            IOLog("Failed to prepare rx IOMemoryDescriptor.\n");
            goto done;
        }
        /*
         * Get physical addresses of the buffers and update buffer info,
         * as well as the descriptor ring with new addresses.
         */
        offset = 0;
        
        for (i = index, end = index + kRxMemBatchSize; i < end; i++) {
            pa = md->getPhysicalSegment(offset, NULL);
            bufInfo[i].phyAddr = pa;
            
            desc[i].read.hdr_addr = 0;
            desc[i].read.pkt_addr = OSSwapHostToLittleInt64(pa);

            offset += PAGE_SIZE;
        }
        
next_batch:
        rdt = index + kRxMemDescMask;
        index = (index + kRxMemBatchSize) & kRxDescMask;
    }
    /*
     * Update descriptor ring after all batches have been mapped.
     */
    if (update) {
        ring->rxMapNextIndex = index;
        
        IXGBE_WRITE_REG(hw, IXGBE_RDT(ring->regIndex), rdt);
    }
done:
    return index;
}

void IntelLucy::rxRunWorkThread(thread_call_param_t param0, thread_call_param_t param1)
{
    ((IntelLucy *) param0)->rxWorkThread((UInt64) param1);
}

/*
 * Perform delayed mapping of a defined number of batches
 * and set the ring state to indicate, that mapping is
 * in progress.
 */
void IntelLucy::rxWorkThread(UInt64 work)
{
    struct ixgbeRxRing *ring = &rxRing[0];
    UInt16 index = (work & 0xffff);
    UInt16 count = (work >> 16);
    
    if (count) {
        rxMapBuffers(ring, index, count, false);
    }
    clear_ring_mapping(ring);
}

UInt32 IntelLucy::rxCleanRingVTD(IONetworkInterface *interface, struct ixgbeRxRing *ring, uint32_t maxCount, IOMbufQueue *pollQueue, void *context)
{
    union ixgbe_adv_rx_desc *desc = &ring->rxDescArray[ring->rxNextDescIndex];
    struct ixgbeRxBufferInfo *bufInfo;
    mbuf_t bufPkt, newPkt;
    UInt32 pktCnt = 0;
    UInt32 pktBytes = 0;
    UInt32 status;
    UInt32 pktSize;
    UInt32 pktType;
    UInt16 vlanTag;
    UInt16 nextIdx;
    UInt8 rscCnt;
    bool replaced;

    while (((status = OSSwapLittleToHostInt32(desc->wb.upper.status_error)) & IXGBE_RXD_STAT_DD) && (pktCnt < maxCount)) {
        bufInfo = &ring->rxBufArray[ring->rxNextDescIndex];
        bufPkt = bufInfo->mbuf;
        
        pktType = OSSwapLittleToHostInt32(desc->wb.lower.lo_dword.data);
        pktSize = OSSwapLittleToHostInt16(desc->wb.upper.length);
        vlanTag = (status & IXGBE_RXD_STAT_VP) ? (desc->wb.upper.vlan) : 0;
        rscCnt = (pktType & IXGBE_RXDADV_RSCCNT_MASK) >> IXGBE_RXDADV_RSCCNT_SHIFT;

        pktBytes += pktSize;
        pktCnt++;
        
        /* Skip bad packet. */
        if (status & IXGBE_RXDADV_ERR_USE) {
            DebugLog("Bad packet.\n");
            etherStats->dot3StatsEntry.internalMacReceiveErrors++;
            goto error_drop;
        }
        newPkt = rxPool->replaceOrCopyPacket(&bufPkt, pktSize, &replaced);

        if (!newPkt) {
            /*
             * No  packets available in the pool, so that we
             * must leave the original packet in place as a
             * last resort.
             */
            DebugLog("replaceOrCopyPacket() failed: pktSize=%u\n", pktSize);
            etherStats->dot3RxExtraEntry.resourceErrors++;
            goto error_drop;
        }
handle_pkt:
        /* If the packet was replaced we have to update the descriptor's buffer address. */
        if (replaced) {
            if (mbuf_next(bufPkt) != NULL) {
                DebugLog("getPhysicalSegments() failed.\n");
                etherStats->dot3RxExtraEntry.resourceErrors++;
                mbuf_freem_list(bufPkt);
                goto error_drop;
            }
            bufInfo->mbuf = bufPkt;
            bufInfo->phyAddr = 0;
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
            ixgbe_get_checksum_result(newPkt, status);

            /* Also get the VLAN tag if there is any. */
            if (vlanTag)
                setVlanTag(newPkt, (UInt32)vlanTag);

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
        
    nextDesc:
        /*
         * If a batch has been completed, increment the number of
         * batches, which need to be mapped.
         */
        if ((ring->rxNextDescIndex & kRxMemDescMask) == kRxMemDescMask)
            ring->rxCleanedCount++;

        /* Get the next descriptor to process. */
        ++ring->rxNextDescIndex &= kRxDescMask;
        desc = &ring->rxDescArray[ring->rxNextDescIndex];
    }
    /*
     * Map and return descriptors to hardware as a batch
     * to improve efficiency.
     */
    if (ring->rxCleanedCount) {
        SInt16 now = ring->rxCleanedCount;
        SInt16 defer = now - 11;
        
        if ((defer > 0) && !ring_is_mapping(ring))
            now -= defer;
        else
            defer = 0;
        
        rxMapBuffers(ring, ring->rxMapNextIndex, now, true);

        /* Schedule some batches for delayed mapping, because there
         * is too much work to be done now.
         */
        if (defer > 0) {
            UInt64 work = (ring->rxMapNextIndex | (defer << 16));
            
            set_ring_mapping(ring);
            thread_call_enter1_delayed(callEntry, (thread_call_param_t) work, callDelay);
            
            /*
             * Update the indices now in the hope, that the mapping
             * of the corresponding batches will complete until we'll
             * be polling again.
             */
            
            ring->rxMapNextIndex = ((ring->rxMapNextIndex + (defer * kRxMemBatchSize)) & kRxDescMask);
        }
        ring->rxCleanedCount = 0;
    }
    if (ring_is_adaptive(ring)) {
        OSAddAtomic((SInt32)pktBytes, &ring->vector.totalBytes);
        OSAddAtomic((SInt32)pktCnt, &ring->vector.totalPackets);
    }
    return pktCnt;
    
error_drop:
    ixgbeDropPktFragment(ring);
    
    if (bufInfo->rscHead) {
        mbuf_freem_list(bufInfo->rscHead);
        bufInfo->rscHead = NULL;
        bufInfo->rscTail = NULL;
    }
    goto nextDesc;
}
