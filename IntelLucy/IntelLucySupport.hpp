
/* IntelLucySupport.h -- IntelLucy hardware specific routines.
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

#ifndef IntelLucySupport_h
#define IntelLucySupport_h

#include "ixgbe.h"

struct ixgbeQueueVector {
    struct ixgbe_adapter *adapter;
    UInt64 nextUpdate;      /* time value of last update */
    UInt64 updatePeriod;    /* interval of adaptive ITR updates */
    SInt32 totalBytes;      /* total bytes processed this int */
    SInt32 totalPackets;    /* total packets processed this int */
    UInt16 vIndex;          /* vector index mapping the vector to a bit in EICR */
    UInt16 itr;             /* current ITR setting for ring */
    UInt16 newItr;          /* new ITR setting for ring in adaptive mode */
    UInt16 pad;
#ifdef DEBUG
    SInt32 maxBytes;
    SInt32 maxPackets;
#endif
};

struct ip4_hdr_be {
    UInt8 hdr_len;
    UInt8 tos;
    UInt16 tot_len;
    UInt16 id;
    UInt16 frg_off;
    UInt8 ttl;
    UInt8 prot;
    UInt16 csum;
    UInt16 addr[4];
};

struct ip6_hdr_be {
    UInt32 vtc_fl;
    UInt16 pay_len;
    UInt8 nxt_hdr;
    UInt8 hop_l;
    UInt16 addr[16];
};

struct tcp_hdr_be {
    UInt16 src_prt;
    UInt16 dst_prt;
    UInt32 seq_num;
    UInt32 ack_num;
    UInt8 dat_off;
    UInt8 flags;
    UInt16 wnd;
    UInt16 csum;
    UInt16 uptr;
};

#define IXGBE_MAX_RX_DESC_POLL 10

void ixgbe_remove_adapter(struct ixgbe_hw *hw);
u32 ixgbe_check_remove(struct ixgbe_hw *hw, u32 reg);

/* MAC filter support functions*/
void ixgbe_restore_perm_mac(struct ixgbe_adapter *adapter);
void ixgbe_mac_set_default_filter(struct ixgbe_adapter *adapter);
void ixgbe_flush_sw_mac_table(struct ixgbe_adapter *adapter);
void ixgbe_sync_mac_table(struct ixgbe_adapter *adapter);
void ixgbe_clear_mac_table(struct ixgbe_adapter *adapter);
void ixgbe_init_mac_table(struct ixgbe_adapter *adapter);
int ixgbe_update_mac_table(struct ixgbe_adapter *adapter, u8 *addrs,
                           u32 count);

void ixgbe_set_mta(struct ixgbe_hw *hw, u8 *mc_addr);

int ixgbe_sync_mc_addr_list(struct ixgbe_adapter *adapter);
int ixgbe_write_mc_addr_list(struct ixgbe_adapter *adapter,
                             u8 *addrs, UInt32 count);

void ixgbe_vlan_promisc_enable(struct ixgbe_adapter *adapter);

/* Rx support functions*/
void ixgbe_setup_psrtype(struct ixgbe_adapter *adapter);
void ixgbe_configure_rscctl(struct ixgbe_adapter *adapter,
                            struct ixgbeRxRing *ring, u32 maxDesc);
void ixgbe_rx_desc_queue_enable(struct ixgbe_adapter *adapter);
void ixgbe_setup_rdrxctl(struct ixgbe_adapter *adapter);
void ixgbe_setup_mrqc(struct ixgbe_adapter *adapter);
void ixgbe_get_checksum_result(mbuf_t m, UInt32 status);

/* interrupt functions */
inline void ixgbe_irq_enable_queues(struct ixgbe_adapter *adapter,
                                    u64 qmask);
void ixgbe_irq_enable(struct ixgbe_adapter *adapter, u64 qmask,
                                    bool flush);
void ixgbe_irq_disable(struct ixgbe_adapter *adapter);
void ixgbe_update_rx_itr(struct ixgbeQueueVector *vector);
void ixgbe_update_tx_itr(struct ixgbeQueueVector *vector);
void ixgbe_write_eitr(struct ixgbeQueueVector *vector);
void ixgbe_set_rx_itr(struct ixgbeQueueVector *vector);
void ixgbe_set_tx_itr(struct ixgbeQueueVector *vector);

/* RSS functions */
u32 ixgbe_rss_indir_tbl_entries(struct ixgbe_adapter *adapter);
void ixgbe_store_key(struct ixgbe_adapter *adapter);
void ixgbe_store_reta(struct ixgbe_adapter *adapter);
void ixgbe_setup_reta(struct ixgbe_adapter *adapter);

#endif /* IntelLucySupport_h */
