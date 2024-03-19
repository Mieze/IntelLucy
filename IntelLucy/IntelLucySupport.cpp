
/* IntelLucySupport.c -- IntelLucy hardware specific routines.
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

#pragma mark --- MAC filter functions ---

void ixgbe_restore_perm_mac(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;

    hw->mac.ops.set_rar(hw, 0, hw->mac.perm_addr, 0,
                IXGBE_RAH_AV);
}

/* this function destroys the first RAR entry */
void ixgbe_mac_set_default_filter(struct ixgbe_adapter *adapter)
{
    struct ixgbe_mac_addr *mac_table = &adapter->mac_table[0];
    struct ixgbe_hw *hw = &adapter->hw;

    memcpy(&mac_table->addr, hw->mac.addr, ETH_ALEN);
    mac_table->pool = 0;

    mac_table->state = IXGBE_MAC_STATE_DEFAULT | IXGBE_MAC_STATE_IN_USE;

    hw->mac.ops.set_rar(hw, 0, mac_table->addr, mac_table->pool,
                IXGBE_RAH_AV);
}

void ixgbe_sync_mac_table(struct ixgbe_adapter *adapter)
{
    struct ixgbe_mac_addr *mac_table = &adapter->mac_table[0];
    struct ixgbe_hw *hw = &adapter->hw;
    int i;
    
    for (i = 0; i < hw->mac.num_rar_entries; i++, mac_table++) {
        if (!(mac_table->state & IXGBE_MAC_STATE_MODIFIED))
            continue;

        if (mac_table->state & IXGBE_MAC_STATE_IN_USE) {
            hw->mac.ops.set_rar(hw, i, mac_table->addr,
                mac_table->pool, IXGBE_RAH_AV);
        } else {
            hw->mac.ops.clear_rar(hw, i);
        }
        mac_table->state &= ~IXGBE_MAC_STATE_MODIFIED;
    }
}

void ixgbe_clear_mac_table(struct ixgbe_adapter *adapter)
{
    struct ixgbe_mac_addr *mac_table = &adapter->mac_table[1];
    struct ixgbe_hw *hw = &adapter->hw;
    int available = hw->mac.num_rar_entries - 1;
    int i;

    for (i = 0; i < available; i++, mac_table++) {
        if (!(mac_table->state & IXGBE_MAC_STATE_IN_USE))
            continue;
        
        hw->mac.ops.clear_rar(hw, i);
    }
}

void ixgbe_init_mac_table(struct ixgbe_adapter *adapter)
{
    struct ixgbe_mac_addr *mac_table = &adapter->mac_table[1];
    struct ixgbe_hw *hw = &adapter->hw;
    int available = hw->mac.num_rar_entries - 1;

    int i;

    for (i = 0; i < available; i++, mac_table++) {
        mac_table->pool = 0;
        mac_table->state = IXGBE_MAC_STATE_MODIFIED;
    }
}

int ixgbe_update_mac_table(struct ixgbe_adapter *adapter, u8 *addrs,
                           UInt32 count)
{
    struct ixgbe_mac_addr *mac_table = &adapter->mac_table[1];
    struct ixgbe_hw *hw = &adapter->hw;
    int available = hw->mac.num_rar_entries - 1;
    int i, written = 0;

    for (i = 0; i < available; i++, mac_table++) {
        if (i < count) {
            memcpy(&mac_table->addr, addrs, ETH_ALEN);
            mac_table->pool = 0;
            mac_table->state = (IXGBE_MAC_STATE_MODIFIED | IXGBE_MAC_STATE_IN_USE);
            written++;
        } else {
            if (mac_table->state & IXGBE_MAC_STATE_IN_USE)
                mac_table->state = IXGBE_MAC_STATE_MODIFIED;
        }
        addrs  += ETH_ALEN;
    }
    return written;
}

void ixgbe_flush_sw_mac_table(struct ixgbe_adapter *adapter)
{
    struct ixgbe_mac_addr *mac_table = &adapter->mac_table[1];
    struct ixgbe_hw *hw = &adapter->hw;
    int available = hw->mac.num_rar_entries - 1;

    int i;

    for (i = 0; i < available; i++, mac_table++) {
        mac_table->state |= IXGBE_MAC_STATE_MODIFIED;
        mac_table->state &= ~IXGBE_MAC_STATE_IN_USE;
    }
    ixgbe_sync_mac_table(adapter);
}

/**
 * ixgbe_write_mc_addr_list - write multicast addresses to MTA
 * @netdev: network interface device structure
 *
 * Writes multicast address list to the MTA hash table.
 * Returns:
 *                0 on no addresses written
 *                X on writing X addresses to MTA
 **/
int ixgbe_write_mc_addr_list(struct ixgbe_adapter *adapter,
                             u8 *addrs, UInt32 count)
{
    struct ixgbe_hw *hw = &adapter->hw;
    u32 i;
    
    /*
     * Set the new number of MC addresses that we are being requested to
     * use.
     */
    hw->addr_ctrl.num_mc_addrs = count;
    hw->addr_ctrl.mta_in_use = 0;

    /* Clear mta_shadow */
    memset(&hw->mac.mta_shadow, 0, sizeof(hw->mac.mta_shadow));

    for (i = 0; i < count; i++) {
        ixgbe_set_mta(hw, addrs);
        addrs += ETH_ALEN;
    }
    return count;
}

int ixgbe_sync_mc_addr_list(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;
    u32 mcstctrl = hw->mac.mc_filter_type;
    u32 i;

    /* Enable mta */
    for (i = 0; i < hw->mac.mcft_size; i++)
        IXGBE_WRITE_REG_ARRAY(hw, IXGBE_MTA(0), i,
                      hw->mac.mta_shadow[i]);

    if (hw->addr_ctrl.mta_in_use > 0)
        mcstctrl |= IXGBE_MCSTCTRL_MFE;
    
    IXGBE_WRITE_REG(hw, IXGBE_MCSTCTRL, mcstctrl);
    
    return hw->addr_ctrl.mta_in_use;
}

void ixgbe_vlan_promisc_enable(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;
    u32 reg_offset;
    u32 vlnctrl;
    u32 vlvfb;
    int i;

    vlnctrl = IXGBE_READ_REG(hw, IXGBE_VLNCTRL);
    vlnctrl &= ~IXGBE_VLNCTRL_VFE;
    IXGBE_WRITE_REG(hw, IXGBE_VLNCTRL, vlnctrl);
    
    /*
     * If we are already in VLAN promisc or have an 82598,
     * there is nothing more to do now.
     */
    if ((hw->mac.type != ixgbe_mac_82598EB) &&
        !(adapter->flags2 & IXGBE_FLAG2_VLAN_PROMISC)) {
        adapter->flags2 |= IXGBE_FLAG2_VLAN_PROMISC;

        /* Add PF to all active pools */
        for (i = IXGBE_VLVF_ENTRIES; --i;) {
            reg_offset = IXGBE_VLVFB(i * 2);
            vlvfb = IXGBE_READ_REG(hw, reg_offset);

            vlvfb |= BIT(0);
            IXGBE_WRITE_REG(hw, reg_offset, vlvfb);
        }

        /* Set all bits in the VLAN filter table array */
        for (i = hw->mac.vft_size; i--;)
            IXGBE_WRITE_REG(hw, IXGBE_VFTA(i), ~0U);
    }
    return;
}

#pragma mark --- rx functions ---

void ixgbe_setup_reta(struct ixgbe_adapter *adapter)
{
    u32 i, j;
    u32 reta_entries = ixgbe_rss_indir_tbl_entries(adapter);

    /* Fill out hash function seeds */
    ixgbe_store_key(adapter);

    /* Fill out redirection table */
    memset(adapter->rss_indir_tbl, 0, sizeof(adapter->rss_indir_tbl));

    /* Send all packets to queue0. */
    for (i = 0, j = 0; i < reta_entries; i++, j++) {
        adapter->rss_indir_tbl[i] = 0;
    }
    ixgbe_store_reta(adapter);
}

void ixgbe_setup_mrqc(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;
    u32 mrqc = 0, rss_field = 0;
    u32 rxcsum;

    /* Disable indicating checksum in descriptor, enables RSS hash */
    rxcsum = IXGBE_READ_REG(hw, IXGBE_RXCSUM);
    rxcsum |= IXGBE_RXCSUM_PCSD;
    IXGBE_WRITE_REG(hw, IXGBE_RXCSUM, rxcsum);

    if (adapter->hw.mac.type == ixgbe_mac_82598EB) {
        mrqc = IXGBE_MRQC_RSSEN;
    } else {
        u8 tcs = adapter->hw_tcs;

        if (tcs > 4)
            mrqc = IXGBE_MRQC_RTRSS8TCEN;
        else if (tcs > 1)
            mrqc = IXGBE_MRQC_RTRSS4TCEN;
        else
            mrqc = IXGBE_MRQC_RSSEN;
    }

    /* Perform hash on these packet types */
    rss_field |= IXGBE_MRQC_RSS_FIELD_IPV4 |
             IXGBE_MRQC_RSS_FIELD_IPV4_TCP |
             IXGBE_MRQC_RSS_FIELD_IPV6 |
             IXGBE_MRQC_RSS_FIELD_IPV6_TCP;

    if (adapter->flags2 & IXGBE_FLAG2_RSS_FIELD_IPV4_UDP)
        rss_field |= IXGBE_MRQC_RSS_FIELD_IPV4_UDP;
    if (adapter->flags2 & IXGBE_FLAG2_RSS_FIELD_IPV6_UDP)
        rss_field |= IXGBE_MRQC_RSS_FIELD_IPV6_UDP;

    ixgbe_setup_reta(adapter);
    mrqc |= rss_field;
    IXGBE_WRITE_REG(hw, IXGBE_MRQC, mrqc);
}

void ixgbe_setup_psrtype(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;
    u32 psrtype;
    int i;
    
    /* PSRTYPE must be initialized in non 82598 adapters */
    psrtype = IXGBE_PSRTYPE_TCPHDR |
              IXGBE_PSRTYPE_UDPHDR |
              IXGBE_PSRTYPE_IPV4HDR |
              IXGBE_PSRTYPE_L2HDR |
              IXGBE_PSRTYPE_IPV6HDR;

    if (hw->mac.type != ixgbe_mac_82598EB) {
        for (i = 0; i < 64; i++)
            IXGBE_WRITE_REG(hw, IXGBE_PSRTYPE(i), psrtype);
    }
}

void ixgbe_setup_rdrxctl(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;
    u32 rdrxctl = IXGBE_READ_REG(hw, IXGBE_RDRXCTL);

    switch (hw->mac.type) {
        case ixgbe_mac_82598EB:
            /*
             * For VMDq support of different descriptor types or
             * buffer sizes through the use of multiple SRRCTL
             * registers, RDRXCTL.MVMEN must be set to 1
             *
             * also, the manual doesn't mention it clearly but DCA hints
             * will only use queue 0's tags unless this bit is set.  Side
             * effects of setting this bit are only that SRRCTL must be
             * fully programmed [0..15]
             */
            rdrxctl |= IXGBE_RDRXCTL_MVMEN;
            break;
            
        case ixgbe_mac_X550:
        case ixgbe_mac_X550EM_x:
        case ixgbe_mac_x550em_a:
            if (adapter->num_vfs)
                rdrxctl |= IXGBE_RDRXCTL_PSP;
            /* fallthrough */
        case ixgbe_mac_82599EB:
        case ixgbe_mac_X540:
            /* Disable RSC for ACK packets */
            IXGBE_WRITE_REG(hw, IXGBE_RSCDBU,
               (IXGBE_RSCDBU_RSCACKDIS | IXGBE_READ_REG(hw, IXGBE_RSCDBU)));
            rdrxctl &= ~IXGBE_RDRXCTL_RSCFRSTSIZE;
            /* hardware requires some bits to be set by default */
            rdrxctl |= (IXGBE_RDRXCTL_RSCACKC | IXGBE_RDRXCTL_FCOE_WRFIX);
            rdrxctl |= IXGBE_RDRXCTL_CRCSTRIP;
            break;
            
        default:
            /* We should do nothing since we don't know this hardware */
            return;
    }

    IXGBE_WRITE_REG(hw, IXGBE_RDRXCTL, rdrxctl);
}

/**
 * ixgbe_configure_rscctl - enable RSC for the indicated ring
 * @adapter: address of board private structure
 * @ring: structure containing ring specific data
 **/
void ixgbe_configure_rscctl(struct ixgbe_adapter *adapter,
                            struct ixgbeRxRing *ring)
{
    struct ixgbe_hw *hw = &adapter->hw;
    u32 rscctrl;
    u8 reg_idx = 0;

    if (!ring_is_rsc_enabled(ring))
        return;

    rscctrl = IXGBE_READ_REG(hw, IXGBE_RSCCTL(reg_idx));
    rscctrl |= IXGBE_RSCCTL_RSCEN;
    /*
     * we must limit the number of descriptors so that the
     * total size of max desc * buf_len is not greater
     * than 65536
     */
    rscctrl |= IXGBE_RSCCTL_MAXDESC_8;
    IXGBE_WRITE_REG(hw, IXGBE_RSCCTL(reg_idx), rscctrl);
}


#pragma mark --- interrupt functions ---

inline void ixgbe_irq_enable_queues(struct ixgbe_adapter *adapter,
                       u64 qmask)
{
    u32 mask;
    struct ixgbe_hw *hw = &adapter->hw;

    switch (hw->mac.type) {
        case ixgbe_mac_82598EB:
            mask = (IXGBE_EIMS_RTX_QUEUE & qmask);
            IXGBE_WRITE_REG(hw, IXGBE_EIMS, mask);
            break;
        case ixgbe_mac_82599EB:
        case ixgbe_mac_X540:
        case ixgbe_mac_X550:
        case ixgbe_mac_X550EM_x:
        case ixgbe_mac_x550em_a:
            mask = (qmask & 0xFFFFFFFF);
                
            if (mask)
                IXGBE_WRITE_REG(hw, IXGBE_EIMS_EX(0), mask);
                
            mask = (qmask >> 32);
                
            if (mask)
                IXGBE_WRITE_REG(hw, IXGBE_EIMS_EX(1), mask);
            break;
                
        default:
            break;
    }
    /* skip the flush */
}

/**
 * ixgbe_irq_enable - Enable default interrupt generation settings
 * @adapter: board private structure
 * @queues: enable irqs for queues
 * @flush: flush register write
 **/
void ixgbe_irq_enable(struct ixgbe_adapter *adapter, u64 qmask,
                    bool flush)
{
    struct ixgbe_hw *hw = &adapter->hw;
    u32 mask = (IXGBE_EIMS_ENABLE_MASK & ~IXGBE_EIMS_RTX_QUEUE);

    /* don't reenable LSC while waiting for link */
    if (adapter->flags & IXGBE_FLAG_NEED_LINK_UPDATE)
        mask &= ~IXGBE_EIMS_LSC;

    if (adapter->flags2 & IXGBE_FLAG2_TEMP_SENSOR_CAPABLE)
        switch (adapter->hw.mac.type) {
            case ixgbe_mac_82599EB:
                mask |= IXGBE_EIMS_GPI_SDP0(hw);
                break;
                    
            case ixgbe_mac_X540:
            case ixgbe_mac_X550:
            case ixgbe_mac_X550EM_x:
            case ixgbe_mac_x550em_a:
                mask |= IXGBE_EIMS_TS;
                break;
                    
            default:
                break;
        }
    if (adapter->flags & IXGBE_FLAG_FAN_FAIL_CAPABLE)
        mask |= IXGBE_EIMS_GPI_SDP1(hw);
    
    switch (adapter->hw.mac.type) {
        case ixgbe_mac_82599EB:
            mask |= IXGBE_EIMS_GPI_SDP1(hw);
            mask |= IXGBE_EIMS_GPI_SDP2(hw);
            /* fallthrough */
        case ixgbe_mac_X540:
        case ixgbe_mac_X550:
        case ixgbe_mac_X550EM_x:
        case ixgbe_mac_x550em_a:
            if (adapter->hw.device_id == IXGBE_DEV_ID_X550EM_X_SFP ||
                adapter->hw.device_id == IXGBE_DEV_ID_X550EM_A_SFP ||
                adapter->hw.device_id == IXGBE_DEV_ID_X550EM_A_SFP_N)
                mask |= IXGBE_EIMS_GPI_SDP0(&adapter->hw);
            
            if (adapter->hw.phy.type == ixgbe_phy_x550em_ext_t)
                mask |= IXGBE_EICR_GPI_SDP0_X540;
            
            mask |= IXGBE_EIMS_ECC;
            mask |= IXGBE_EIMS_MAILBOX;
            break;
            
        default:
            break;
    }
    if ((adapter->flags & IXGBE_FLAG_FDIR_HASH_CAPABLE) &&
        !(adapter->flags2 & IXGBE_FLAG2_FDIR_REQUIRES_REINIT))
        mask |= IXGBE_EIMS_FLOW_DIR;

    IXGBE_WRITE_REG(&adapter->hw, IXGBE_EIMS, mask);
    
    if (qmask)
        ixgbe_irq_enable_queues(adapter, qmask);
    
    if (flush)
        IXGBE_WRITE_FLUSH(&adapter->hw);
}

/**
 * ixgbe_irq_disable - Mask off interrupt generation on the NIC
 * @adapter: board private structure
 **/
void ixgbe_irq_disable(struct ixgbe_adapter *adapter)
{
    switch (adapter->hw.mac.type) {
        case ixgbe_mac_82598EB:
            IXGBE_WRITE_REG(&adapter->hw, IXGBE_EIMC, ~0);
            break;
            
        case ixgbe_mac_82599EB:
        case ixgbe_mac_X540:
        case ixgbe_mac_X550:
        case ixgbe_mac_X550EM_x:
        case ixgbe_mac_x550em_a:
            IXGBE_WRITE_REG(&adapter->hw, IXGBE_EIMC, 0xFFFF0000);
            IXGBE_WRITE_REG(&adapter->hw, IXGBE_EIMC_EX(0), ~0);
            IXGBE_WRITE_REG(&adapter->hw, IXGBE_EIMC_EX(1), ~0);
            break;
            
        default:
            break;
    }
    IXGBE_WRITE_FLUSH(&adapter->hw);
}

/**
 * ixgbe_update_itr - update the dynamic ITR value based on statistics
 * @q_vector: structure containing interrupt and ring information
 *
 *      Stores a new ITR value based on packets and byte
 *      counts during the last interrupt.  The advantage of per interrupt
 *      computation is faster updates and more accurate ITR for the current
 *      traffic pattern.  Constants in this function were computed
 *      based on theoretical maximum wire speed and thresholds were set based
 *      on testing data as well as attempting to minimize response time
 *      while increasing bulk throughput.
 **/
void ixgbe_update_itr(struct ixgbeQueueVector *vector)
{
    unsigned int itr = IXGBE_ITR_ADAPTIVE_MIN_USECS |
               IXGBE_ITR_ADAPTIVE_LATENCY;
    unsigned int avg_wire_size, packets, bytes;
    u64 nextUpdate = 0;

    clock_get_uptime(&nextUpdate);

    /* If we didn't update within up to 1 - 2 jiffies we can assume
     * that either packets are coming in so slow there hasn't been
     * any work, or that there is so much work that NAPI is dealing
     * with interrupt moderation and we don't need to do anything.
     */
    if (nextUpdate > vector->nextUpdate)
        goto clear_counts;

    packets = vector->totalPackets;

    /* We have no packets to actually measure against. This means
     * either one of the other queues on this vector is active or
     * we are a Tx queue doing TSO with too high of an interrupt rate.
     *
     * When this occurs just tick up our delay by the minimum value
     * and hope that this extra delay will prevent us from being called
     * without any work on our queue.
     */
    if (!packets) {
        itr = (vector->itr >> 2) + IXGBE_ITR_ADAPTIVE_MIN_INC;
        if (itr > IXGBE_ITR_ADAPTIVE_MAX_USECS)
            itr = IXGBE_ITR_ADAPTIVE_MAX_USECS;
        itr += vector->newItr & IXGBE_ITR_ADAPTIVE_LATENCY;
        goto clear_counts;
    }

    bytes = vector->totalBytes;

    /* If packets are less than 4 or bytes are less than 9000 assume
     * insufficient data to use bulk rate limiting approach. We are
     * likely latency driven.
     */
    if (packets < 4 && bytes < 9000) {
        itr = IXGBE_ITR_ADAPTIVE_LATENCY;
        goto adjust_by_size;
    }

    /* Between 4 and 48 we can assume that our current interrupt delay
     * is only slightly too low. As such we should increase it by a small
     * fixed amount.
     */
    if (packets < 48) {
        itr = (vector->itr >> 2) + IXGBE_ITR_ADAPTIVE_MIN_INC;
        if (itr > IXGBE_ITR_ADAPTIVE_MAX_USECS)
            itr = IXGBE_ITR_ADAPTIVE_MAX_USECS;
        goto clear_counts;
    }

    /* Between 48 and 96 is our "goldilocks" zone where we are working
     * out "just right". Just report that our current ITR is good for us.
     */
    if (packets < 96) {
        itr = vector->itr >> 2;
        goto clear_counts;
    }

    /* If packet count is 96 or greater we are likely looking at a slight
     * overrun of the delay we want. Try halving our delay to see if that
     * will cut the number of packets in half per interrupt.
     */
    if (packets < 256) {
        itr = vector->itr >> 3;
        if (itr < IXGBE_ITR_ADAPTIVE_MIN_USECS)
            itr = IXGBE_ITR_ADAPTIVE_MIN_USECS;
        goto clear_counts;
    }

    /* The paths below assume we are dealing with a bulk ITR since number
     * of packets is 256 or greater. We are just going to have to compute
     * a value and try to bring the count under control, though for smaller
     * packet sizes there isn't much we can do as NAPI polling will likely
     * be kicking in sooner rather than later.
     */
    itr = IXGBE_ITR_ADAPTIVE_BULK;

adjust_by_size:
    /* If packet counts are 256 or greater we can assume we have a gross
     * overestimation of what the rate should be. Instead of trying to fine
     * tune it just use the formula below to try and dial in an exact value
     * give the current packet size of the frame.
     */
    avg_wire_size = bytes / packets;

    /* The following is a crude approximation of:
     *  wmem_default / (size + overhead) = desired_pkts_per_int
     *  rate / bits_per_byte / (size + ethernet overhead) = pkt_rate
     *  (desired_pkt_rate / pkt_rate) * usecs_per_sec = ITR value
     *
     * Assuming wmem_default is 212992 and overhead is 640 bytes per
     * packet, (256 skb, 64 headroom, 320 shared info), we can reduce the
     * formula down to
     *
     *  (170 * (size + 24)) / (size + 640) = ITR
     *
     * We first do some math on the packet size and then finally bitshift
     * by 8 after rounding up. We also have to account for PCIe link speed
     * difference as ITR scales based on this.
     */
    if (avg_wire_size <= 60) {
        /* Start at 50k ints/sec */
        avg_wire_size = 5120;
    } else if (avg_wire_size <= 316) {
        /* 50K ints/sec to 16K ints/sec */
        avg_wire_size *= 40;
        avg_wire_size += 2720;
    } else if (avg_wire_size <= 1084) {
        /* 16K ints/sec to 9.2K ints/sec */
        avg_wire_size *= 15;
        avg_wire_size += 11452;
    } else if (avg_wire_size < 1968) {
        /* 9.2K ints/sec to 8K ints/sec */
        avg_wire_size *= 5;
        avg_wire_size += 22420;
    } else {
        /* plateau at a limit of 8K ints/sec */
        avg_wire_size = 32256;
    }

    /* If we are in low latency mode half our delay which doubles the rate
     * to somewhere between 100K to 16K ints/sec
     */
    if (itr & IXGBE_ITR_ADAPTIVE_LATENCY)
        avg_wire_size >>= 1;

    /* Resultant value is 256 times larger than it needs to be. This
     * gives us room to adjust the value as needed to either increase
     * or decrease the value based on link speeds of 10G, 2.5G, 1G, etc.
     *
     * Use addition as we have already recorded the new latency flag
     * for the ITR value.
     */
    switch (vector->adapter->link_speed) {
        case IXGBE_LINK_SPEED_10GB_FULL:
        case IXGBE_LINK_SPEED_100_FULL:
        default:
            itr += DIV_ROUND_UP(avg_wire_size,
                        IXGBE_ITR_ADAPTIVE_MIN_INC * 256) *
                   IXGBE_ITR_ADAPTIVE_MIN_INC;
            break;
            
        case IXGBE_LINK_SPEED_2_5GB_FULL:
        case IXGBE_LINK_SPEED_1GB_FULL:
        case IXGBE_LINK_SPEED_10_FULL:
            if (avg_wire_size > 8064)
                avg_wire_size = 8064;
            itr += DIV_ROUND_UP(avg_wire_size,
                        IXGBE_ITR_ADAPTIVE_MIN_INC * 64) *
                   IXGBE_ITR_ADAPTIVE_MIN_INC;
            break;
    }

clear_counts:
    /* write back value */
    vector->newItr = itr;

    /* next update should occur within next jiffy */
    vector->nextUpdate = nextUpdate + vector->updatePeriod;

    vector->totalBytes = 0;
    vector->totalPackets = 0;
}

/**
 * ixgbe_write_eitr - write EITR register in hardware specific way
 * @q_vector: structure containing interrupt and ring information
 *
 * This function is made to be called by ethtool and by the driver
 * when it needs to update EITR registers at runtime.  Hardware
 * specific quirks/differences are taken care of here.
 */
void ixgbe_write_eitr(struct ixgbeQueueVector *vector)
{
    struct ixgbe_adapter *adapter = vector->adapter;
    struct ixgbe_hw *hw = &adapter->hw;
    int v_idx = vector->vIndex;
    u32 itr_reg = vector->itr & IXGBE_MAX_EITR;

    switch (adapter->hw.mac.type) {
        case ixgbe_mac_82598EB:
            /* must write high and low 16 bits to reset counter */
            itr_reg |= (itr_reg << 16);
            break;
            
        case ixgbe_mac_82599EB:
        case ixgbe_mac_X540:
        case ixgbe_mac_X550:
        case ixgbe_mac_X550EM_x:
        case ixgbe_mac_x550em_a:
            /*
             * set the WDIS bit to not clear the timer bits and cause an
             * immediate assertion of the interrupt
             */
            itr_reg |= IXGBE_EITR_CNT_WDIS;
            break;
            
        default:
            break;
    }
    IXGBE_WRITE_REG(hw, IXGBE_EITR(v_idx), itr_reg);
}

void ixgbe_set_itr(struct ixgbeQueueVector *vector)
{
    u32 new_itr;

    ixgbe_update_itr(vector);

    /* use the smallest value of new ITR delay calculations */
    new_itr = vector->newItr;

    /* Clear latency flag if set, shift into correct position */
    new_itr &= ~IXGBE_ITR_ADAPTIVE_LATENCY;
    new_itr <<= 2;

    if (new_itr != vector->itr) {
        /* save the algorithm value here */
        vector->itr = new_itr;

        ixgbe_write_eitr(vector);
    }
}

#pragma mark --- RSS functions ---

/**
 * ixgbe_rss_indir_tbl_entries - Return RSS indirection table entries
 * @adapter: device handle
 *
 *  - 82598/82599/X540:     128
 *  - X550(non-SRIOV mode): 512
 *  - X550(SRIOV mode):     64
 */
u32 ixgbe_rss_indir_tbl_entries(struct ixgbe_adapter *adapter)
{
    u32 entries;
    
    if (adapter->hw.mac.type < ixgbe_mac_X550)
        entries = 128;
    else if (adapter->flags & IXGBE_FLAG_SRIOV_ENABLED)
        entries =  64;
    else
        entries =  512;
    
    return entries;
}

/**
 * ixgbe_store_key - Write the RSS key to HW
 * @adapter: device handle
 *
 * Write the RSS key stored in adapter.rss_key to HW.
 */
void ixgbe_store_key(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;
    int i;

    for (i = 0; i < 10; i++)
        IXGBE_WRITE_REG(hw, IXGBE_RSSRK(i), adapter->rss_key[i]);
}

/**
 * ixgbe_store_reta - Write the RETA table to HW
 * @adapter: device handle
 *
 * Write the RSS redirection table stored in adapter.rss_indir_tbl[] to HW.
 */
void ixgbe_store_reta(struct ixgbe_adapter *adapter)
{
    u32 i, reta_entries = ixgbe_rss_indir_tbl_entries(adapter);
    struct ixgbe_hw *hw = &adapter->hw;
    u32 reta = 0;
    u32 indices_multi;
    u8 *indir_tbl = adapter->rss_indir_tbl;

    /* Fill out the redirection table as follows:
     *  - 82598:      8 bit wide entries containing pair of 4 bit RSS
     *    indices.
     *  - 82599/X540: 8 bit wide entries containing 4 bit RSS index
     *  - X550:       8 bit wide entries containing 6 bit RSS index
     */
    if (adapter->hw.mac.type == ixgbe_mac_82598EB)
        indices_multi = 0x11;
    else
        indices_multi = 0x1;

    /* Write redirection table to HW */
    for (i = 0; i < reta_entries; i++) {
        reta |= indices_multi * indir_tbl[i] << (i & 0x3) * 8;
        if ((i & 3) == 3) {
            if (i < 128)
                IXGBE_WRITE_REG(hw, IXGBE_RETA(i >> 2), reta);
            else
                IXGBE_WRITE_REG(hw, IXGBE_ERETA((i >> 2) - 32),
                        reta);
            reta = 0;
        }
    }
}
