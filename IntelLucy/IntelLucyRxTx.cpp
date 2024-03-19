/* IntelLucyRxTx.cpp -- IntelLucy rx & tx routines.
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

/**
 * ixgbe_set_num_queues - Allocate queues for device, feature dependent
 * @adapter: board private structure to initialize
 *
 * This is the top level queue allocation routine.  The order here is very
 * important, starting with the "most" number of features turned on at once,
 * and ending with the smallest set of features.  This way large combinations
 * can be allocated if they're turned on, and smaller combinations are the
 * fallthrough conditions.
 *
 **/
void IntelLucy::ixgbeSetNumQueues(struct ixgbe_adapter *adapter)
{
    /* Start with base case */
    adapter->num_rx_queues = 1;
    adapter->num_tx_queues = 1;
    adapter->num_xdp_queues = 0;
    adapter->num_rx_pools = 1;
    adapter->num_rx_queues_per_pool = 1;
}

#pragma mark --- rx configuration methods ---

#define IXGBE_SRRCTL_BSIZEHDRSIZE_SHIFT 2

void IntelLucy::ixgbeConfigureSrrctl(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;
    u32 srrctl = 0;
    u8 reg_idx = 0;

    if (hw->mac.type == ixgbe_mac_82598EB) {
        u16 mask = 0x000f;

        /*
         * if VMDq is not active we must program one srrctl register
         * per RSS queue since we have enabled RDRXCTL.MVMEN
         */
        reg_idx &= mask;
    }

    /* configure header buffer length, needed for RSC */
    srrctl = IXGBE_RX_HDR_SIZE << IXGBE_SRRCTL_BSIZEHDRSIZE_SHIFT;

    /* configure the packet buffer length */
    srrctl |= (rxBufferPktSize >> IXGBE_SRRCTL_BSIZEPKT_SHIFT);

    /* configure descriptor type */
    srrctl |= IXGBE_SRRCTL_DESCTYPE_ADV_ONEBUF;

    IXGBE_WRITE_REG(hw, IXGBE_SRRCTL(reg_idx), srrctl);
}

void IntelLucy::ixgbeSetupMtqc(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;
    u32 rttdcs, mtqc;
    u8 tcs = adapter->hw_tcs;

    if (hw->mac.type == ixgbe_mac_82598EB)
        return;

    /* disable the arbiter while setting MTQC */
    rttdcs = IXGBE_READ_REG(hw, IXGBE_RTTDCS);
    rttdcs |= IXGBE_RTTDCS_ARBDIS;
    IXGBE_WRITE_REG(hw, IXGBE_RTTDCS, rttdcs);

    /* set transmit pool layout */
    if (tcs > 4) {
        mtqc = IXGBE_MTQC_RT_ENA | IXGBE_MTQC_8TC_8TQ;
    } else if (tcs > 1) {
        mtqc = IXGBE_MTQC_RT_ENA | IXGBE_MTQC_4TC_4TQ;
    } else {
        u8 max_txq = adapter->num_tx_queues +
            adapter->num_xdp_queues;
        if (max_txq > 63)
            mtqc = IXGBE_MTQC_RT_ENA | IXGBE_MTQC_4TC_4TQ;
        else
            mtqc = IXGBE_MTQC_64Q_1PB;
    }

    IXGBE_WRITE_REG(hw, IXGBE_MTQC, mtqc);

    /* Enable Security TX Buffer IFG for multiple pb */
    if (tcs) {
        u32 sectx = IXGBE_READ_REG(hw, IXGBE_SECTXMINIFG);
        sectx |= IXGBE_SECTX_DCB;
        IXGBE_WRITE_REG(hw, IXGBE_SECTXMINIFG, sectx);
    }

    /* re-enable the arbiter */
    rttdcs &= ~IXGBE_RTTDCS_ARBDIS;
    IXGBE_WRITE_REG(hw, IXGBE_RTTDCS, rttdcs);
}

void IntelLucy::ixgbeRxDescQueueEnable(struct ixgbe_adapter *adapter,
                            struct ixgbeRxRing *ring)
{
    struct ixgbe_hw *hw = &adapter->hw;
    SInt32 waitLoop = IXGBE_MAX_RX_DESC_POLL;
    UInt32 rxdctl;
    UInt8 regIdx = ring->regIndex;

    if (ixgbe_removed(hw->hw_addr))
        return;
    
    /* RXDCTL.EN will return 0 on 82598 if link is down, so skip it */
    if (hw->mac.type == ixgbe_mac_82598EB &&
        !(IXGBE_READ_REG(hw, IXGBE_LINKS) & IXGBE_LINKS_UP))
        return;

    do {
        usleep_range(1000, 2000);
        rxdctl = IXGBE_READ_REG(hw, IXGBE_RXDCTL(regIdx));
    } while (--waitLoop && !(rxdctl & IXGBE_RXDCTL_ENABLE));

    if (!waitLoop)
        IOLog("RXDCTL.ENABLE on Rx queue %d not set within the polling period\n", regIdx);
}

void IntelLucy::ixgbeConfigureRxRing(struct ixgbe_adapter *adapter,
                                     struct ixgbeRxRing *ring)
{
    struct ixgbe_hw *hw = &adapter->hw;
    UInt32 rxdctl;
    UInt8 regIdx = ring->regIndex;

    /* disable queue to avoid use of these values while updating state */
    rxdctl = IXGBE_READ_REG(hw, IXGBE_RXDCTL(regIdx));
    rxdctl &= ~IXGBE_RXDCTL_ENABLE;

    /* write value back with RXDCTL.ENABLE bit cleared */
    IXGBE_WRITE_REG(hw, IXGBE_RXDCTL(regIdx), rxdctl);
    IXGBE_WRITE_FLUSH(hw);

    ring->rxCleanedCount = ring->rxNextDescIndex = 0;
    
    IXGBE_WRITE_REG(hw, IXGBE_RDBAL(regIdx), (rxPhyAddr & 0xffffffff));
    IXGBE_WRITE_REG(hw, IXGBE_RDBAH(regIdx), (rxPhyAddr >> 32));
    IXGBE_WRITE_REG(hw, IXGBE_RDLEN(regIdx), kRxDescRingSize);
    /* Force flushing of IXGBE_RDLEN to prevent MDD */
    IXGBE_WRITE_FLUSH(hw);

    IXGBE_WRITE_REG(hw, IXGBE_RDH(regIdx), 0);
    IXGBE_WRITE_REG(hw, IXGBE_RDT(regIdx), 0);

    ixgbeConfigureSrrctl(adapter);
    ixgbe_configure_rscctl(adapter, ring);

    if (hw->mac.type == ixgbe_mac_82598EB) {
        /*
         * enable cache line friendly hardware writes:
         * PTHRESH=32 descriptors (half the internal cache),
         * this also removes ugly rx_no_buffer_count increment
         * HTHRESH=4 descriptors (to minimize latency on fetch)
         * WTHRESH=8 burst writeback up to two cache lines
         */
        rxdctl &= ~0x3FFFFF;
        rxdctl |=  0x080420;
    /* RXDCTL.RLPML does not work on 82599 */
    } else if (hw->mac.type != ixgbe_mac_82599EB) {
        rxdctl &= ~(IXGBE_RXDCTL_RLPMLMASK | IXGBE_RXDCTL_RLPML_EN);
    }
    /* enable receive descriptor ring */
    rxdctl |= IXGBE_RXDCTL_ENABLE;
    IXGBE_WRITE_REG(hw, IXGBE_RXDCTL(regIdx), rxdctl);

    /* Update rxActiveQueueMask to mark the queue as active. */
    rxActiveQueueMask |= BIT(ring->vector.vIndex);

    ixgbeRxDescQueueEnable(adapter, ring);
}

/**
 * ixgbeConfigureRx - Configure 8259x Receive Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Rx unit of the MAC after a reset.
 **/
void IntelLucy::ixgbeConfigureRx(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;
    UInt32 rxctrl, rfctl;
    int i;

    /* disable receives while setting up the descriptors */
    hw->mac.ops.disable_rx(hw);

    ixgbe_setup_psrtype(adapter);
    ixgbe_setup_rdrxctl(adapter);

    /* RSC Setup */
    rfctl = IXGBE_READ_REG(hw, IXGBE_RFCTL);
    rfctl &= ~IXGBE_RFCTL_RSC_DIS;
    
    if (!(adapter->flags2 & IXGBE_FLAG2_RSC_ENABLED))
        rfctl |= IXGBE_RFCTL_RSC_DIS;

    /* disable NFS filtering */
    rfctl |= (IXGBE_RFCTL_NFSW_DIS | IXGBE_RFCTL_NFSR_DIS);
    IXGBE_WRITE_REG(hw, IXGBE_RFCTL, rfctl);

    /* Program registers for the distribution of queues */
    ixgbe_setup_mrqc(adapter);

    /* ixgbeSetRxBufferLen() must be called before ring initialization */
    ixgbeSetRxBufferLen(adapter);

    rxActiveQueueMask = 0;

    /*
     * Setup the HW Rx Head and Tail Descriptor Pointers and
     * the Base and Length of the Rx Descriptor Ring
     */
    for (i = 0; i < adapter->num_rx_queues; i++)
        ixgbeConfigureRxRing(adapter, &rxRing[i]);

    rxctrl = IXGBE_READ_REG(hw, IXGBE_RXCTRL);
    /* disable drop enable for 82598 parts */
    if (hw->mac.type == ixgbe_mac_82598EB)
        rxctrl |= IXGBE_RXCTRL_DMBYPS;

    /* enable all receives */
    rxctrl |= IXGBE_RXCTRL_RXEN;
    hw->mac.ops.enable_rx_dma(hw, rxctrl);
    
    /* Bump RDT to enable descriptor fetching. */
    for (i = 0; i < adapter->num_rx_queues; i++)
        IXGBE_WRITE_REG(hw, IXGBE_RDT(rxRing[i].regIndex), kRxLastDesc);

}

/**
 * ixgbeSetRxMode - Unicast, Multicast and Promiscuous mode set
 *
 * The set_rx_mode entry point is called whenever the unicast/multicast
 * address list or the network interface flags are updated.  This routine is
 * responsible for configuring the hardware for proper unicast, multicast and
 * promiscuous mode.
 **/
void IntelLucy::ixgbeSetRxMode()
{
    struct ixgbe_adapter *adapter = &adapterData;
    struct ixgbe_hw *hw = &adapter->hw;
    UInt32 fctrl, vmolr = IXGBE_VMOLR_BAM | IXGBE_VMOLR_AUPE;
    int count;

    /* Check for Promiscuous and All Multicast modes */
    fctrl = IXGBE_READ_REG(hw, IXGBE_FCTRL);

    /* set all bits that we expect to always be set */
    fctrl &= ~IXGBE_FCTRL_SBP; /* disable store-bad-packets */
    fctrl |= IXGBE_FCTRL_BAM;
    fctrl |= IXGBE_FCTRL_DPF; /* discard pause frames when FC enabled */
    fctrl |= IXGBE_FCTRL_PMCF;

    /* clear the bits we are changing the status of */
    fctrl &= ~(IXGBE_FCTRL_UPE | IXGBE_FCTRL_MPE);
    
    if (test_bit(__PROMISC, &stateFlags)) {
        hw->addr_ctrl.user_set_promisc = true;
        fctrl |= (IXGBE_FCTRL_UPE | IXGBE_FCTRL_MPE);
        vmolr |= IXGBE_VMOLR_MPE;
    } else {
        hw->addr_ctrl.user_set_promisc = false;
    }

    /*
     * Write addresses to available RAR registers, if there is not
     * sufficient space to store all the addresses then enable
     * unicast promiscuous mode
     */
    //ixgbe_sync_mac_table(adapter);

    /* Write addresses to the MTA, if the attempt fails
     * then we should just turn on promiscuous mode so
     * that we can at least receive multicast traffic
     */
    count = ixgbe_sync_mc_addr_list(adapter);
    
    if (count)
        vmolr |= IXGBE_VMOLR_ROMPE;

    if (hw->mac.type != ixgbe_mac_82598EB) {
        vmolr |= IXGBE_READ_REG(hw, IXGBE_VMOLR(0)) &
             ~(IXGBE_VMOLR_MPE | IXGBE_VMOLR_ROMPE |
               IXGBE_VMOLR_ROPE);
        IXGBE_WRITE_REG(hw, IXGBE_VMOLR(0), vmolr);
    }

    IXGBE_WRITE_REG(hw, IXGBE_FCTRL, fctrl);

    ixgbeVlanStripEnable(adapter);
    ixgbe_vlan_promisc_enable(adapter);
}

void IntelLucy::ixgbeDisableRx(struct ixgbe_adapter *adapter)
{
    unsigned long wait_delay, delay_interval;
    struct ixgbe_hw *hw = &adapter->hw;
    struct ixgbeRxRing *ring;
    int i, wait_loop;
    UInt32 rxdctl;
    UInt8 regIdx;
    
    /* disable receives */
    hw->mac.ops.disable_rx(hw);

    if (ixgbe_removed(hw->hw_addr))
        return;

    /* disable all enabled Rx queues */
    for (i = 0; i < adapter->num_rx_queues; i++) {
        ring = &rxRing[i];
        regIdx = ring->regIndex;

        rxdctl = IXGBE_READ_REG(hw, IXGBE_RXDCTL(regIdx));
        rxdctl &= ~IXGBE_RXDCTL_ENABLE;
        rxdctl |= IXGBE_RXDCTL_SWFLSH;

        /* write value back with RXDCTL.ENABLE bit cleared */
        IXGBE_WRITE_REG(hw, IXGBE_RXDCTL(regIdx), rxdctl);
        
        /* Update rxActiveQueueMask to mark the queue as inactive. */
        rxActiveQueueMask &= ~BIT(ring->vector.vIndex);
    }

    /* RXDCTL.EN may not change on 82598 if link is down, so skip it */
    if (hw->mac.type == ixgbe_mac_82598EB &&
        !(IXGBE_READ_REG(hw, IXGBE_LINKS) & IXGBE_LINKS_UP))
        return;

    /* Determine our minimum delay interval. We will increase this value
     * with each subsequent test. This way if the device returns quickly
     * we should spend as little time as possible waiting, however as
     * the time increases we will wait for larger periods of time.
     *
     * The trick here is that we increase the interval using the
     * following pattern: 1x 3x 5x 7x 9x 11x 13x 15x 17x 19x. The result
     * of that wait is that it totals up to 100x whatever interval we
     * choose. Since our minimum wait is 100us we can just divide the
     * total timeout by 100 to get our minimum delay interval.
     */
    delay_interval = ixgbeGetCompletionTimeout(adapter) / 100;

    wait_loop = IXGBE_MAX_RX_DESC_POLL;
    wait_delay = delay_interval;

    while (wait_loop--) {
        usleep_range(wait_delay, wait_delay + 10);
        wait_delay += delay_interval * 2;
        rxdctl = 0;

        /* OR together the reading of all the active RXDCTL registers,
         * and then test the result. We need the disable to complete
         * before we start freeing the memory and invalidating the
         * DMA mappings.
         */
        for (i = 0; i < adapter->num_rx_queues; i++) {
            ring = &rxRing[i];
            regIdx = ring->regIndex;

            rxdctl |= IXGBE_READ_REG(hw, IXGBE_RXDCTL(regIdx));
        }

        if (!(rxdctl & IXGBE_RXDCTL_ENABLE))
            goto done;;
    }
    IOLog("RXDCTL.ENABLE for one or more queues not cleared within the polling period.\n");
    
done:
    return;
}

/**
 * ixgbeDropPktFragment - Drop packet fragment from rx queue
 * @ring: rx queue
 **/
void IntelLucy::ixgbeDropPktFragment(struct ixgbeRxRing *ring)
{
    /*
     * In case there is a packet fragment which hasn't been enqueued yet
     * we have to free it in order to prevent a memory leak.
     */
    if (ring->rxPacketHead)
            freePacket(ring->rxPacketHead);
        
    ring->rxPacketHead = ring->rxPacketTail = NULL;
    ring->rxPacketSize = 0;
}

/**
 * ixgbeClearRxRings - Free Rx Buffers for all queues
 * @adapter: board private structure
 **/
void IntelLucy::ixgbeClearRxRings(struct ixgbe_adapter *adapter)
{
    struct ixgbeRxBufferInfo *bufInfo;
    struct ixgbeRxRing *ring;
    UInt32 i, j;

    /* On descriptor writeback the buffer addresses are overwritten so that
     * we must restore them in order to make sure that we leave the ring in
     * a usable state.
     */
    for (i = 0; i < adapter->num_rx_queues; i++) {
        ring = &rxRing[i];
        
        for (j = 0; j < kNumRxDesc; j++) {
            bufInfo = &ring->rxBufArray[j];
            
            ring->rxDescArray[j].read.pkt_addr = OSSwapHostToLittleInt64(bufInfo->phyAddr);
            ring->rxDescArray[j].read.hdr_addr = 0;
            
            /*
             * Also free RSC packet fragments which might be stuck in
             * the rx ring's buffer info structures.
             */
            if (bufInfo->rscHead) {
                freePacket(bufInfo->rscHead);
                
                bufInfo->rscHead = NULL;
                bufInfo->rscTail = NULL;
            }
        }
        ring->rxCleanedCount = ring->rxNextDescIndex = 0;
        
        /*
         * Also free packet fragments which haven't
         * been upstreamed yet.
         */
        if (ring->rxPacketHead)
            freePacket(ring->rxPacketHead);
        
        ring->rxPacketHead = ring->rxPacketTail = NULL;
        ring->rxPacketSize = 0;
    }
}

void IntelLucy::ixgbeSetRxBufferLen(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;
    struct ixgbeRxRing *ring;
    UInt32 max_frame = mtu + ETH_HLEN + ETH_FCS_LEN;
    UInt32 mhadd, hlreg0;
    int i;

    /* adjust max frame to be at least the size of a standard frame */
    if (max_frame < (ETH_FRAME_LEN + ETH_FCS_LEN))
        max_frame = (ETH_FRAME_LEN + ETH_FCS_LEN);

    mhadd = IXGBE_READ_REG(hw, IXGBE_MHADD);
    if (max_frame != (mhadd >> IXGBE_MHADD_MFS_SHIFT)) {
        mhadd &= ~IXGBE_MHADD_MFS_MASK;
        mhadd |= max_frame << IXGBE_MHADD_MFS_SHIFT;

        IXGBE_WRITE_REG(hw, IXGBE_MHADD, mhadd);
    }

    hlreg0 = IXGBE_READ_REG(hw, IXGBE_HLREG0);
    /* set jumbo enable since MHADD.MFS is keeping size locked at max_frame */
    hlreg0 |= IXGBE_HLREG0_JUMBOEN;
    IXGBE_WRITE_REG(hw, IXGBE_HLREG0, hlreg0);

    /*
     * Setup the HW Rx Head and Tail Descriptor Pointers and
     * the Base and Length of the Rx Descriptor Ring
     */
    for (i = 0; i < adapter->num_rx_queues; i++) {
        ring = &rxRing[i];
        
        if (adapter->flags2 & IXGBE_FLAG2_RSC_ENABLED)
            set_ring_rsc_enabled(ring);
    }
}

/**
 * ixgbeVlanStripEnable - helper to enable hw vlan stripping
 * @adapter: driver data
 */
void IntelLucy::ixgbeVlanStripEnable(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;
    struct ixgbeRxRing *ring;
    UInt32 vlnctrl;
    int i, j;

    switch (hw->mac.type) {
        case ixgbe_mac_82598EB:
            vlnctrl = IXGBE_READ_REG(hw, IXGBE_VLNCTRL);
            vlnctrl |= IXGBE_VLNCTRL_VME;
            IXGBE_WRITE_REG(hw, IXGBE_VLNCTRL, vlnctrl);
            break;
            
        case ixgbe_mac_82599EB:
        case ixgbe_mac_X540:
        case ixgbe_mac_X550:
        case ixgbe_mac_X550EM_x:
        case ixgbe_mac_x550em_a:
            for (i = 0; i < adapter->num_rx_queues; i++) {
                ring = &rxRing[i];
                j = ring->regIndex;
                
                vlnctrl = IXGBE_READ_REG(hw, IXGBE_RXDCTL(j));
                vlnctrl |= IXGBE_RXDCTL_VME;
                IXGBE_WRITE_REG(hw, IXGBE_RXDCTL(j), vlnctrl);
            }
            break;
            
        default:
            break;
    }
}

#pragma mark --- tx configuration methods ---

/**
 * ixgbe_configure_tx_ring - Configure 8259x Tx ring after Reset
 * @adapter: board private structure
 * @ring: structure containing ring specific data
 *
 * Configure the Tx descriptor ring after a reset.
 **/
void IntelLucy::ixgbeConfigureTxRing(struct ixgbe_adapter *adapter,
                                     struct ixgbeTxRing *ring)
{
    struct ixgbe_hw *hw = &adapter->hw;
    UInt32 txdctl = IXGBE_TXDCTL_ENABLE;
    int wait_loop = 10;
    int i;
    UInt16 itr = ring->vector.itr;
    UInt8 regIdx = ring->regIndex;
    
    /* disable queue to avoid issues while updating state */
    IXGBE_WRITE_REG(hw, IXGBE_TXDCTL(regIdx), 0);
    IXGBE_WRITE_FLUSH(hw);

    IXGBE_WRITE_REG(hw, IXGBE_TDBAL(regIdx), (ring->txPhyRingAddr & 0xffffffff));
    IXGBE_WRITE_REG(hw, IXGBE_TDBAH(regIdx), (ring->txPhyRingAddr >> 32));
    IXGBE_WRITE_REG(hw, IXGBE_TDLEN(regIdx), kTxDescRingSize);
    IXGBE_WRITE_REG(hw, IXGBE_TDH(regIdx), 0);
    IXGBE_WRITE_REG(hw, IXGBE_TDT(regIdx), 0);
    
    ring->txNextDescIndex = ring->txDirtyIndex = ring->txCleanBarrierIndex = 0;
    ring->txNumFreeDesc = kNumTxDesc;

    /*
     * set WTHRESH to encourage burst writeback, it should not be set
     * higher than 1 when:
     * - ITR is 0 as it could cause false TX hangs
     * - ITR is set to > 100k int/sec and BQL is enabled
     *
     * In order to avoid issues WTHRESH + PTHRESH should always be equal
     * to or less than the number of on chip descriptors, which is
     * currently 40.
     */
    if ((itr == 0) || (itr < IXGBE_100K_ITR))
        txdctl |= 1u << 16;    /* WTHRESH = 1 */
    else
        txdctl |= 8u << 16;    /* WTHRESH = 8 */

    /*
     * Setting PTHRESH to 32 both improves performance
     * and avoids a TX hang with DFP enabled
     */
    txdctl |= (1u << 8) |    /* HTHRESH = 1 */
           32;        /* PTHRESH = 32 */

    /* reinitialize flowdirector state */
/*
    if (adapter->flags & IXGBE_FLAG_FDIR_HASH_CAPABLE) {
        ring->atr_sample_rate = adapter->atr_sample_rate;
        ring->atr_count = 0;
        set_bit(__IXGBE_TX_FDIR_INIT_DONE, &ring->state);
    } else {
        ring->atr_sample_rate = 0;
    }
*/
    ring->txHangSuspected = 0;
    
    /* reinitialize tx_buffer_info */
    for (i = 0; i < kNumTxDesc; i++) {
        ring->txBufArray[i].mbuf = NULL;
        ring->txBufArray[i].numDescs = 0;
        ring->txBufArray[i].packetBytes = 0;
    }

    /* enable queue */
    IXGBE_WRITE_REG(hw, IXGBE_TXDCTL(regIdx), txdctl);

    /* Update txActiveQueueMask to mark the queue as active. */
    txActiveQueueMask |= BIT(ring->vector.vIndex);
    
    /* TXDCTL.EN will return 0 on 82598 if link is down, so skip it */
    if (hw->mac.type == ixgbe_mac_82598EB &&
        !(IXGBE_READ_REG(hw, IXGBE_LINKS) & IXGBE_LINKS_UP))
        return;

    /* poll to verify queue is enabled */
    do {
        usleep_range(1000, 2000);
        txdctl = IXGBE_READ_REG(hw, IXGBE_TXDCTL(regIdx));
    } while (--wait_loop && !(txdctl & IXGBE_TXDCTL_ENABLE));
    
    if (!wait_loop)
        IOLog("Failed to enable Tx Queue %d.\n", regIdx);
}

/**
 * ixgbeConfigureTx - Configure 8259x Transmit Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Tx unit of the MAC after a reset.
 **/
void IntelLucy::ixgbeConfigureTx(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;
    UInt32 dmatxctl;
    UInt32 i;

    ixgbeSetupMtqc(adapter);

    if (hw->mac.type != ixgbe_mac_82598EB) {
        /* DMATXCTL.EN must be before Tx queues are enabled */
        dmatxctl = IXGBE_READ_REG(hw, IXGBE_DMATXCTL);
        dmatxctl |= IXGBE_DMATXCTL_TE;
        IXGBE_WRITE_REG(hw, IXGBE_DMATXCTL, dmatxctl);
    }
    txActiveQueueMask = 0;
    
    /* Setup the HW Tx Head and Tail descriptor pointers */
    for (i = 0; i < adapter->num_tx_queues; i++)
        ixgbeConfigureTxRing(adapter, &txRing[i]);
}

/**
 * ixgbeClearTxPending - Clear pending TX work from the PCIe fifo
 * @hw: pointer to the hardware structure
 *
 * The 82599 and x540 MACs can experience issues if TX work is still pending
 * when a reset occurs.  This function prevents this by flushing the PCIe
 * buffers on the system.
 **/
void IntelLucy::ixgbeClearTxPending(struct ixgbe_hw *hw)
{
    u32 gcr_ext, hlreg0, i, poll;
    u16 value;

    /*
     * If double reset is not requested then all transactions should
     * already be clear and as such there is no work to do
     */
    if (!(hw->mac.flags & IXGBE_FLAGS_DOUBLE_RESET_REQUIRED))
        return;

    /*
     * Set loopback enable to prevent any transmits from being sent
     * should the link come up.  This assumes that the RXCTRL.RXEN bit
     * has already been cleared.
     */
    hlreg0 = IXGBE_READ_REG(hw, IXGBE_HLREG0);
    IXGBE_WRITE_REG(hw, IXGBE_HLREG0, hlreg0 | IXGBE_HLREG0_LPBK);

    /* wait for a last completion before clearing buffers */
    IXGBE_WRITE_FLUSH(hw);
    usleep_range(3000, 6000);

    /* Before proceeding, make sure that the PCIe block does not have
     * transactions pending.
     */
    poll = ixgbePCIeTimeoutPoll(hw);
    
    for (i = 0; i < poll; i++) {
        usleep_range(100, 200);
        value = pciDevice->extendedConfigRead16(IXGBE_PCI_DEVICE_STATUS);

        if (ixgbe_removed(hw->hw_addr))
            break;
        if (!(value & IXGBE_PCI_DEVICE_STATUS_TRANSACTION_PENDING))
            break;
    }

    /* initiate cleaning flow for buffers in the PCIe transaction layer */
    gcr_ext = IXGBE_READ_REG(hw, IXGBE_GCR_EXT);
    IXGBE_WRITE_REG(hw, IXGBE_GCR_EXT,
            gcr_ext | IXGBE_GCR_EXT_BUFFERS_CLEAR);

    /* Flush all writes and allow 20usec for all transactions to clear */
    IXGBE_WRITE_FLUSH(hw);
    udelay(20);

    /* restore previous register values */
    IXGBE_WRITE_REG(hw, IXGBE_GCR_EXT, gcr_ext);
    IXGBE_WRITE_REG(hw, IXGBE_HLREG0, hlreg0);
}

/**
 * ixgbeClearTxRings - Free Tx Buffers for all queues
 * @adapter: board private structure
 **/

void IntelLucy::ixgbeClearTxRings(struct ixgbe_adapter *adapter)
{
    struct ixgbeTxRing *ring;
    mbuf_t m;
    UInt32 i, j;
    
    /* Cleanup the tx descriptor ring. */
    for (j = 0; j < adapter->num_tx_queues; j++) {
        ring = &txRing[j];
        
        for (i = 0; i < kNumTxDesc; i++) {
            m = ring->txBufArray[i].mbuf;
            
            if (m) {
                freePacket(m);
                ring->txBufArray[i].mbuf = NULL;
                ring->txBufArray[i].numDescs = 0;
            }
        }
        ring->txNextDescIndex = ring->txDirtyIndex = ring->txCleanBarrierIndex = 0;
        ring->txNumFreeDesc = kNumTxDesc;
    }
}

void IntelLucy::ixgbeDisableTx(struct ixgbe_adapter *adapter)
{
    unsigned long wait_delay, delay_interval;
    struct ixgbe_hw *hw = &adapter->hw;
    struct ixgbeTxRing *ring;
    int i, wait_loop;
    UInt32 txdctl;
    UInt8 regIdx;
    
    if (ixgbe_removed(hw->hw_addr))
        return;

    /* disable all enabled Tx queues */
    for (i = 0; i < adapter->num_tx_queues; i++) {
       ring = &txRing[i];
        regIdx  = ring->regIndex;

        IXGBE_WRITE_REG(hw, IXGBE_TXDCTL(regIdx), IXGBE_TXDCTL_SWFLSH);
        
        /* Update txActiveQueueMask to mark the queue as inactive. */
        txActiveQueueMask &= ~BIT(ring->vector.vIndex);
    }

    /* If the link is not up there shouldn't be much in the way of
     * pending transactions. Those that are left will be flushed out
     * when the reset logic goes through the flush sequence to clean out
     * the pending Tx transactions.
     */
    if (!(IXGBE_READ_REG(hw, IXGBE_LINKS) & IXGBE_LINKS_UP))
        goto dma_engine_disable;

    /* Determine our minimum delay interval. We will increase this value
     * with each subsequent test. This way if the device returns quickly
     * we should spend as little time as possible waiting, however as
     * the time increases we will wait for larger periods of time.
     *
     * The trick here is that we increase the interval using the
     * following pattern: 1x 3x 5x 7x 9x 11x 13x 15x 17x 19x. The result
     * of that wait is that it totals up to 100x whatever interval we
     * choose. Since our minimum wait is 100us we can just divide the
     * total timeout by 100 to get our minimum delay interval.
     */
    delay_interval = ixgbeGetCompletionTimeout(adapter) / 100;

    wait_loop = IXGBE_MAX_RX_DESC_POLL;
    wait_delay = delay_interval;

    while (wait_loop--) {
        usleep_range(wait_delay, wait_delay + 10);
        wait_delay += delay_interval * 2;
        txdctl = 0;

        /* OR together the reading of all the active TXDCTL registers,
         * and then test the result. We need the disable to complete
         * before we start freeing the memory and invalidating the
         * DMA mappings.
         */
        for (i = 0; i < adapter->num_tx_queues; i++) {
            ring = &txRing[i];
            regIdx  = ring->regIndex;

            txdctl |= IXGBE_READ_REG(hw, IXGBE_TXDCTL(regIdx));
        }

        if (!(txdctl & IXGBE_TXDCTL_ENABLE))
            goto dma_engine_disable;
    }

    IOLog("TXDCTL.ENABLE for one or more queues not cleared within the polling period.\n");

dma_engine_disable:
    /* Disable the Tx DMA engine on 82599 and later MAC */
    switch (hw->mac.type) {
        case ixgbe_mac_82599EB:
        case ixgbe_mac_X540:
        case ixgbe_mac_X550:
        case ixgbe_mac_X550EM_x:
        case ixgbe_mac_x550em_a:
            IXGBE_WRITE_REG(hw, IXGBE_DMATXCTL,
                    (IXGBE_READ_REG(hw, IXGBE_DMATXCTL) &
                     ~IXGBE_DMATXCTL_TE));
            /* fallthrough */
        default:
            break;
    }
}
