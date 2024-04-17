/* IntelLucyServiceTask.cpp -- IntelLucy service task routines.
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

static const char ixgbe_overheat_msg[] = "Network adapter has been stopped because it has over heated. Restart the computer. If the problem persists, power off the system and replace the adapter.";

static const char *flowControlNames[kFlowControlTypeCount] = {
    "",
    ", Rx flow-control",
    ", Tx flow-control",
    ", Rx/Tx flow-control",
};

static const char* eeeNames[kEEETypeCount] = {
    "",
    ", energy-efficient-ethernet"
};

#ifdef DEBUG

struct ixgbe_reg_info {
    u32 ofs;
    char *name;
};

static const struct ixgbe_reg_info ixgbe_reg_info_tbl[] = {

    /* General Registers */
    {IXGBE_CTRL, (char *)"CTRL"},
    {IXGBE_STATUS, (char *)"STATUS"},
    {IXGBE_CTRL_EXT, (char *)"CTRL_EXT"},

    /* Interrupt Registers */
    {IXGBE_EICR, (char *)"EICR"},

    /* RX Registers */
    {IXGBE_SRRCTL(0), (char *)"SRRCTL"},
    {IXGBE_DCA_RXCTRL(0), (char *)"DRXCTL"},
    {IXGBE_RDLEN(0), (char *)"RDLEN"},
    {IXGBE_RDH(0), (char *)"RDH"},
    {IXGBE_RDT(0), (char *)"RDT"},
    {IXGBE_RXDCTL(0), (char *)"RXDCTL"},
    {IXGBE_RDBAL(0), (char *)"RDBAL"},
    {IXGBE_RDBAH(0), (char *)"RDBAH"},

    /* TX Registers */
    {IXGBE_TDBAL(0), (char *)"TDBAL"},
    {IXGBE_TDBAH(0), (char *)"TDBAH"},
    {IXGBE_TDLEN(0), (char *)"TDLEN"},
    {IXGBE_TDH(0), (char *)"TDH"},
    {IXGBE_TDT(0), (char *)"TDT"},
    {IXGBE_TXDCTL(0), (char *)"TXDCTL"},

    /* List Terminator */
    { .name = NULL }
};


/*
 * ixgbe_regdump - register printout routine
 */
static void ixgbe_regdump(struct ixgbe_hw *hw, struct ixgbe_reg_info *reginfo)
{
    int i;
    char rname[16];
    u32 regs[64];

    switch (reginfo->ofs) {
    case IXGBE_SRRCTL(0):
        for (i = 0; i < 64; i++)
            regs[i] = IXGBE_READ_REG(hw, IXGBE_SRRCTL(i));
        break;
    case IXGBE_DCA_RXCTRL(0):
        for (i = 0; i < 64; i++)
            regs[i] = IXGBE_READ_REG(hw, IXGBE_DCA_RXCTRL(i));
        break;
    case IXGBE_RDLEN(0):
        for (i = 0; i < 64; i++)
            regs[i] = IXGBE_READ_REG(hw, IXGBE_RDLEN(i));
        break;
    case IXGBE_RDH(0):
        for (i = 0; i < 64; i++)
            regs[i] = IXGBE_READ_REG(hw, IXGBE_RDH(i));
        break;
    case IXGBE_RDT(0):
        for (i = 0; i < 64; i++)
            regs[i] = IXGBE_READ_REG(hw, IXGBE_RDT(i));
        break;
    case IXGBE_RXDCTL(0):
        for (i = 0; i < 64; i++)
            regs[i] = IXGBE_READ_REG(hw, IXGBE_RXDCTL(i));
        break;
    case IXGBE_RDBAL(0):
        for (i = 0; i < 64; i++)
            regs[i] = IXGBE_READ_REG(hw, IXGBE_RDBAL(i));
        break;
    case IXGBE_RDBAH(0):
        for (i = 0; i < 64; i++)
            regs[i] = IXGBE_READ_REG(hw, IXGBE_RDBAH(i));
        break;
    case IXGBE_TDBAL(0):
        for (i = 0; i < 64; i++)
            regs[i] = IXGBE_READ_REG(hw, IXGBE_TDBAL(i));
        break;
    case IXGBE_TDBAH(0):
        for (i = 0; i < 64; i++)
            regs[i] = IXGBE_READ_REG(hw, IXGBE_TDBAH(i));
        break;
    case IXGBE_TDLEN(0):
        for (i = 0; i < 64; i++)
            regs[i] = IXGBE_READ_REG(hw, IXGBE_TDLEN(i));
        break;
    case IXGBE_TDH(0):
        for (i = 0; i < 64; i++)
            regs[i] = IXGBE_READ_REG(hw, IXGBE_TDH(i));
        break;
    case IXGBE_TDT(0):
        for (i = 0; i < 64; i++)
            regs[i] = IXGBE_READ_REG(hw, IXGBE_TDT(i));
        break;
    case IXGBE_TXDCTL(0):
        for (i = 0; i < 64; i++)
            regs[i] = IXGBE_READ_REG(hw, IXGBE_TXDCTL(i));
        break;
    default:
        DebugLog("[IntelLucy] %-15s %08x\n",
            reginfo->name, IXGBE_READ_REG(hw, reginfo->ofs));
        return;
    }

    i = 0;
    while (i < 64) {
        int j;
        char buf[100];
        char *p = buf;

        snprintf(rname, 16, "%s[%d-%d]", reginfo->name, i, i + 7);
        for (j = 0; j < 8; j++)
            p += snprintf(p, 96, " %08x", regs[i++]);
        DebugLog("[IntelLucy] %-15s%s\n", rname, buf);
    }

}
#endif /* DEBUG */

/**
 * ixgbeTxTimeoutReset - initiate reset due to Tx timeout
 * @adapter: driver private struct
 **/
void IntelLucy::ixgbeTxTimeoutReset(struct ixgbe_adapter *adapter)
{
    /* Do the reset outside of interrupt context */
    if (!test_bit(__IXGBE_DOWN, &adapter->state)) {
        set_bit(__IXGBE_RESET_REQUESTED, &adapter->state);
        IOLog("Initiating reset due to tx timeout.\n");
        ixgbeServiceEventSchedule(adapter);
    }
}

void IntelLucy::ixgbeCheckHangSubtask(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;
    struct ixgbeTxRing *ring;
    SInt64 txDone;
    UInt32 txdctl;
    UInt32 i;
    UInt8 regIndex;
    
    /* If we're down, removing or resetting, just bail */
    if (test_bit(__IXGBE_DOWN, &adapter->state) ||
        test_bit(__IXGBE_REMOVING, &adapter->state) ||
        test_bit(__IXGBE_RESETTING, &adapter->state))
        return;

    if (test_bit(__LINK_UP, &stateFlags)) {
        for (i = 0; i < adapter->num_tx_queues; i++) {
            ring = &txRing[i];
            txDone = ring->txDescDone;
            /*
              * If the descriptor ring isn't empty and there
              * is no progress, the ring might be hung.
              */
             if ((txDone == ring->txDescDoneLast) &&
                 (ring->txNumFreeDesc < kNumTxDesc)) {
                 if (++ring->txHangSuspected > kTxHungTreshhold) {
                     etherStats->dot3TxExtraEntry.resets++;

                     ixgbeTxTimeoutReset(adapter);
                     
                     goto done;
                 } else {
                     regIndex = ring->regIndex;
                     
                     /* Trigger descriptor write-back. */
                     txdctl = IXGBE_READ_REG(hw, IXGBE_TXDCTL(regIndex));
                     txdctl |= IXGBE_TXDCTL_SWFLSH;
                     IXGBE_WRITE_REG(hw, IXGBE_TXDCTL(regIndex), txdctl);
                     
                     /*
                      * Cause interrupt to clean the ring provided we aren't
                      * operating in poll mode.
                      */
                     if (!test_bit(__POLL_MODE, &stateFlags))
                         IXGBE_WRITE_REG(hw, IXGBE_EICS, BIT(ring->vector.vIndex));
                 }
             } else {
                 ring->txHangSuspected = 0;
             }
            ring->txDescDoneLast = txDone;
        }
    }
done:
    return;
}

 void IntelLucy::ixgbeCheckSfpEvent(struct ixgbe_adapter *adapter, u32 eicr)
{
    struct ixgbe_hw *hw = &adapter->hw;
    u32 eicr_mask = IXGBE_EICR_GPI_SDP2(hw);

    if (!ixgbe_is_sfp(hw))
        return;

    /* Later MAC's use different SDP */
    if (hw->mac.type >= ixgbe_mac_X540)
        eicr_mask = IXGBE_EICR_GPI_SDP0_X540;

    if (eicr & eicr_mask) {
        /* Clear the interrupt */
        IXGBE_WRITE_REG(hw, IXGBE_EICR, eicr_mask);
        
        if (!test_bit(__IXGBE_DOWN, &adapter->state)) {
            adapter->flags2 |= IXGBE_FLAG2_SFP_NEEDS_RESET;
            adapter->sfp_poll_time = 0;
            
            ixgbeServiceEventSchedule(adapter);
        }
    }

    if (adapter->hw.mac.type == ixgbe_mac_82599EB &&
        (eicr & IXGBE_EICR_GPI_SDP1(hw))) {
        /* Clear the interrupt */
        IXGBE_WRITE_REG(hw, IXGBE_EICR, IXGBE_EICR_GPI_SDP1(hw));
        
        if (!test_bit(__IXGBE_DOWN, &adapter->state)) {
            adapter->flags |= IXGBE_FLAG_NEED_LINK_CONFIG;
            
            ixgbeServiceEventSchedule(adapter);
        }
    }
}

/**
 * ixgbe_check_overtemp_subtask - check for over temperature
 * @adapter: pointer to adapter
 **/
 void IntelLucy::ixgbeCheckOvertempSubtask(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;
    u32 eicr = adapter->interrupt_event;
    s32 rc;

    if (test_bit(__IXGBE_DOWN, &adapter->state))
        return;

    if (!(adapter->flags2 & IXGBE_FLAG2_TEMP_SENSOR_EVENT))
        return;

    adapter->flags2 &= ~IXGBE_FLAG2_TEMP_SENSOR_EVENT;

    switch (hw->device_id) {
        case IXGBE_DEV_ID_82599_T3_LOM:
            /*
             * Since the warning interrupt is for both ports
             * we don't have to check if:
             *  - This interrupt wasn't for our port.
             *  - We may have missed the interrupt so always have to
             *    check if we  got a LSC
             */
            if (!(eicr & IXGBE_EICR_GPI_SDP0_8259X) &&
                !(eicr & IXGBE_EICR_LSC))
                return;

            if (!(eicr & IXGBE_EICR_LSC) && hw->mac.ops.check_link) {
                u32 speed;
                bool link_up = false;

                hw->mac.ops.check_link(hw, &speed, &link_up, false);

                if (link_up)
                    return;
            }

            /* Check if this is not due to overtemp */
            if (hw->phy.ops.check_overtemp(hw) != IXGBE_ERR_OVERTEMP)
                return;

            break;
            
        case IXGBE_DEV_ID_X550EM_A_1G_T:
        case IXGBE_DEV_ID_X550EM_A_1G_T_L:
            rc = hw->phy.ops.check_overtemp(hw);
            if (rc != IXGBE_ERR_OVERTEMP)
                return;
            break;
            
        default:
            if (adapter->hw.mac.type >= ixgbe_mac_X540)
                return;
            if (!(eicr & IXGBE_EICR_GPI_SDP0(hw)))
                return;
            break;
    }
     IOLog("%s\n", ixgbe_overheat_msg);

    adapter->interrupt_event = 0;
}

void IntelLucy::ixgbeCheckOvertempEvent(struct ixgbe_adapter *adapter, u32 eicr)
{
    struct ixgbe_hw *hw = &adapter->hw;

    if (!(adapter->flags2 & IXGBE_FLAG2_TEMP_SENSOR_CAPABLE))
        return;

    switch (adapter->hw.mac.type) {
        case ixgbe_mac_82599EB:
            /*
             * Need to check link state so complete overtemp check
             * on service task
             */
            if (((eicr & IXGBE_EICR_GPI_SDP0(hw)) || (eicr & IXGBE_EICR_LSC)) &&
                (!test_bit(__IXGBE_DOWN, &adapter->state))) {
                adapter->interrupt_event = eicr;
                adapter->flags2 |= IXGBE_FLAG2_TEMP_SENSOR_EVENT;
                
                ixgbeServiceEventSchedule(adapter);
                return;
            }
            return;
            
        case ixgbe_mac_x550em_a:
            if (eicr & IXGBE_EICR_GPI_SDP0_X550EM_a) {
                adapter->interrupt_event = eicr;
                adapter->flags2 |= IXGBE_FLAG2_TEMP_SENSOR_EVENT;
                
                ixgbeServiceEventSchedule(adapter);

                IXGBE_WRITE_REG(&adapter->hw, IXGBE_EIMC,
                        IXGBE_EICR_GPI_SDP0_X550EM_a);
                IXGBE_WRITE_REG(&adapter->hw, IXGBE_EICR,
                        IXGBE_EICR_GPI_SDP0_X550EM_a);
            }
            return;
            
    case ixgbe_mac_X550:
    case ixgbe_mac_X540:
        if (!(eicr & IXGBE_EICR_TS))
            return;
        break;
            
    default:
        return;
    }
    IOLog("%s\n", ixgbe_overheat_msg);
}

/**
 * ixgbe_sfp_detection_subtask - poll for SFP+ cable
 * @adapter: the ixgbe adapter structure
 **/
void IntelLucy::ixgbeSfpDetectionSubtask(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;
    uint64_t uptime;
    s32 err;

    /* not searching for SFP so there is nothing to do here */
    if (!(adapter->flags2 & IXGBE_FLAG2_SEARCH_FOR_SFP) &&
        !(adapter->flags2 & IXGBE_FLAG2_SFP_NEEDS_RESET))
        return;

    clock_get_uptime(&uptime);
    
    if (adapter->sfp_poll_time &&
        (adapter->sfp_poll_time > uptime))
        return; /* If not yet time to poll for SFP */

    /* someone else is in init, wait until next service event */
    if (test_and_set_bit(__IXGBE_IN_SFP_INIT, &adapter->state))
        return;

    adapter->sfp_poll_time = uptime + sfpPollTime;

    err = hw->phy.ops.identify_sfp(hw);
    if (err == IXGBE_ERR_SFP_NOT_SUPPORTED)
        goto sfp_out;

    if (err == IXGBE_ERR_SFP_NOT_PRESENT) {
        /* If no cable is present, then we need to reset
         * the next time we find a good cable. */
        adapter->flags2 |= IXGBE_FLAG2_SFP_NEEDS_RESET;
    }

    /* exit on error */
    if (err)
        goto sfp_out;

    /* exit if reset not needed */
    if (!(adapter->flags2 & IXGBE_FLAG2_SFP_NEEDS_RESET))
        goto sfp_out;

    adapter->flags2 &= ~IXGBE_FLAG2_SFP_NEEDS_RESET;

    /*
     * A module may be identified correctly, but the EEPROM may not have
     * support for that module.  setup_sfp() will fail in that case, so
     * we should not allow that module to load.
     */
    if (hw->mac.type == ixgbe_mac_82598EB)
        err = hw->phy.ops.reset(hw);
    else
        err = hw->mac.ops.setup_sfp(hw);

    if (err == IXGBE_ERR_SFP_NOT_SUPPORTED)
        goto sfp_out;

    adapter->flags |= IXGBE_FLAG_NEED_LINK_CONFIG;
    DebugLog("detected SFP+: %d\n", hw->phy.sfp_type);

sfp_out:
    clear_bit(__IXGBE_IN_SFP_INIT, &adapter->state);

    /*
     * Update the media dictionary matching the detected SPF+ type
     * and, in case the selected medium is not a member of the
     * new media dictionary, fall back to the default medium.
     */
    updateMediumDict();
    updateSelectedMedium();
    
    if (err == IXGBE_ERR_SFP_NOT_SUPPORTED)
        IOLog("Failed to initialize because an unsupported SFP+ module type was detected.\n");
}

/**
 * ixgbe_sfp_link_config_subtask - set up link SFP after module install
 * @adapter: the ixgbe adapter structure
 **/
void IntelLucy::ixgbeSfpLinkConfigSubtask(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;
    u32 cap_speed;
    u32 speed;
    bool autoneg = false;

    if (!(adapter->flags & IXGBE_FLAG_NEED_LINK_CONFIG))
        return;

    /* someone else is in init, wait until next service event */
    if (test_and_set_bit(__IXGBE_IN_SFP_INIT, &adapter->state))
        return;

    adapter->flags &= ~IXGBE_FLAG_NEED_LINK_CONFIG;

    hw->mac.ops.get_link_capabilities(hw, &cap_speed, &autoneg);

    /* advertise highest capable link speed */
    if (!autoneg && (cap_speed & IXGBE_LINK_SPEED_10GB_FULL))
        speed = IXGBE_LINK_SPEED_10GB_FULL;
    else
        speed = cap_speed & (IXGBE_LINK_SPEED_10GB_FULL |
                     IXGBE_LINK_SPEED_1GB_FULL);

    if (hw->mac.ops.setup_link)
        hw->mac.ops.setup_link(hw, speed, true);

    adapter->flags |= IXGBE_FLAG_NEED_LINK_UPDATE;
    clock_get_uptime((uint64_t *)&adapter->link_check_timeout);
    clear_bit(__IXGBE_IN_SFP_INIT, &adapter->state);
}

void IntelLucy::ixgbePhyInterruptSubtask(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;
    u32 status;

    if (!(adapter->flags2 & IXGBE_FLAG2_PHY_INTERRUPT))
        return;

    adapter->flags2 &= ~IXGBE_FLAG2_PHY_INTERRUPT;

    if (!hw->phy.ops.handle_lasi)
        return;

    status = hw->phy.ops.handle_lasi(&adapter->hw);
    if (status != IXGBE_ERR_OVERTEMP)
        return;

    IOLog("%s\n", ixgbe_overheat_msg);
}

void IntelLucy::ixgbeResetSubtask(struct ixgbe_adapter *adapter)
{
    if (!test_and_clear_bit(__IXGBE_RESET_REQUESTED, &adapter->state))
        return;

    /* If we're already down, removing or resetting, just bail */
    if (test_bit(__IXGBE_DOWN, &adapter->state) ||
        test_bit(__IXGBE_REMOVING, &adapter->state) ||
        test_bit(__IXGBE_RESETTING, &adapter->state)) {
        return;
    }

    //ixgbe_dump(adapter);
    IOLog("Reset adapter.\n");
    adapter->tx_timeout_count++;

    ixgbeReinit(adapter);
}

/**
 * ixgbeWatchdogLinkIsDown - update netif_carrier status and
 *                               print link down message
 * @adapter: pointer to the adapter structure
 **/
void IntelLucy::ixgbeWatchdogLinkIsDown(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;

    adapter->link_up = false;
    adapter->link_speed = 0;

    /* only continue if link was up previously */
    if (!test_bit(__LINK_UP, &stateFlags))
        return;

    /* poll for SFP+ cable when link is down */
    if (ixgbe_is_sfp(hw) && hw->mac.type == ixgbe_mac_82598EB)
        adapter->flags2 |= IXGBE_FLAG2_SEARCH_FOR_SFP;
/*
    if (test_bit(__IXGBE_PTP_RUNNING, &adapter->state))
        ixgbe_ptp_start_cyclecounter(adapter);
*/
    /* Stop output thread and flush output queue. */
    netif->stopOutputThread();
    netif->flushOutputQueue();
    
    /* Update link status. */
    clear_mask((__LINK_UP_M | __POLL_MODE_M), &stateFlags);
    setLinkStatus(kIONetworkLinkValid);

    requireMaxBusStall(0);
    requireMaxInterruptDelay(0);

    ixgbeClearRxRings(adapter);
    
    IOLog("Link down on en%u\n", netif->getUnitNumber());
    
    DebugLog("Initiating reset to clear Tx work after link loss.\n");
    set_bit(__IXGBE_RESET_REQUESTED, &adapter->state);
}

/**
 * ixgbeWatchdogLinkIsUp - update netif_carrier status and
 *                             print link up message
 * @adapter: pointer to the device adapter structure
 **/
void IntelLucy::ixgbeWatchdogLinkIsUp(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;
    IONetworkMedium *medium;
    UInt32 midx = MIDX_AUTO;
    u32 link_speed = adapter->link_speed;
    const char *speed_str;
    const char *flow_str;
    const char *eee_str;
    bool flow_rx, flow_tx;
    
    /* only continue if link was previously down */
    if (test_bit(__LINK_UP, &stateFlags))
        return;
    
    adapter->flags2 &= ~IXGBE_FLAG2_SEARCH_FOR_SFP;
    
    switch (hw->mac.type) {
        case ixgbe_mac_82598EB: {
            u32 frctl = IXGBE_READ_REG(hw, IXGBE_FCTRL);
            u32 rmcs = IXGBE_READ_REG(hw, IXGBE_RMCS);
            flow_rx = !!(frctl & IXGBE_FCTRL_RFCE);
            flow_tx = !!(rmcs & IXGBE_RMCS_TFCE_802_3X);
        }
            break;
            
        case ixgbe_mac_X540:
        case ixgbe_mac_X550:
        case ixgbe_mac_X550EM_x:
        case ixgbe_mac_x550em_a:
        case ixgbe_mac_82599EB: {
            u32 mflcn = IXGBE_READ_REG(hw, IXGBE_MFLCN);
            u32 fccfg = IXGBE_READ_REG(hw, IXGBE_FCCFG);
            flow_rx = !!(mflcn & IXGBE_MFLCN_RFCE);
            flow_tx = !!(fccfg & IXGBE_FCCFG_TFCE_802_3X);
        }
            break;
            
        default:
            flow_tx = false;
            flow_rx = false;
            break;
    }
    
    clock_get_uptime((uint64_t *)&adapter->last_rx_ptp_check);
    /*
     if (test_bit(__IXGBE_PTP_RUNNING, &adapter->state))
     ixgbe_ptp_start_cyclecounter(adapter);
     */
    bzero(&pollParams, sizeof(IONetworkPacketPollingParameters));
    pollParams.lowThresholdPackets = 10;
    pollParams.highThresholdPackets = 40;
    pollParams.lowThresholdBytes = 0x1000;
    pollParams.highThresholdBytes = 0x10000;
    pollParams.pollIntervalTime = 1000000;  /* 1ms */
    
    switch (link_speed) {
        case IXGBE_LINK_SPEED_10GB_FULL:
            switch (hw->phy.media_type) {
                case ixgbe_media_type_fiber:
                case ixgbe_media_type_fiber_qsfp:
                    switch (adapter->hw.phy.sfp_type) {
                        case ixgbe_sfp_type_da_cu:
                        case ixgbe_sfp_type_da_cu_core0:
                        case ixgbe_sfp_type_da_cu_core1:
                            if (flow_tx || flow_rx) {
                                midx = MIDX_10GB_DA_FC;
                            } else {
                                midx = MIDX_10GB_DA;
                            }
                            break;
                            
                        case ixgbe_sfp_type_sr:
                        case ixgbe_sfp_type_srlr_core0:
                        case ixgbe_sfp_type_srlr_core1:
                            if (flow_tx || flow_rx) {
                                midx = MIDX_10GB_SR_FC;
                            } else {
                                midx = MIDX_10GB_SR;
                            }
                            break;
                            
                        case ixgbe_sfp_type_lr:
                            if (flow_tx || flow_rx) {
                                midx = MIDX_10GB_LR_FC;
                            } else {
                                midx = MIDX_10GB_LR;
                            }
                            break;
                            
                        case ixgbe_sfp_type_da_act_lmt_core0:
                        case ixgbe_sfp_type_da_act_lmt_core1:
                            if (flow_tx || flow_rx) {
                                midx = MIDX_10GB_DAL_FC;
                            } else {
                                midx = MIDX_10GB_DAL;
                            }
                            break;

                        default:
                            midx = MIDX_AUTO;
                            IOLog("Default medium.\n");
                            break;
                    }
                    break;
                    
                case ixgbe_media_type_copper:
                    if (flow_tx || flow_rx) {
                        midx = MIDX_10GBFC;
                    } else {
                        midx = MIDX_10GB;
                    }
                    break;
                    
                default:
                    midx = MIDX_AUTO;
                    IOLog("Unsupported medium found.\n");
                    break;
            }
            speed_str = "10 Gbps";
            
            pollParams.pollIntervalTime = pollTime10G;
            break;
            
            
        case IXGBE_LINK_SPEED_5GB_FULL:
            if (flow_tx || flow_rx) {
                midx = MIDX_5000FC;
            } else {
                midx = MIDX_5000;
            }
            speed_str = "5 Gbps";
            
            pollParams.pollIntervalTime = pollTime5G;
            break;
            
        case IXGBE_LINK_SPEED_2_5GB_FULL:
            if (flow_tx || flow_rx) {
                midx = MIDX_2500FC;
            } else {
                midx = MIDX_2500;
            }
            speed_str = "2.5 Gbps";
            
            pollParams.pollIntervalTime = pollTime2G;
            break;
            
        case IXGBE_LINK_SPEED_1GB_FULL:
            switch (hw->phy.media_type) {
                case ixgbe_media_type_fiber:
                case ixgbe_media_type_fiber_qsfp:
                    switch (adapter->hw.phy.sfp_type) {
                        case ixgbe_sfp_type_1g_cu_core0:
                        case ixgbe_sfp_type_1g_cu_core1:
                            if (flow_tx || flow_rx) {
                                midx = MIDX_1000FC;
                            } else {
                                midx = MIDX_1000;
                            }
                            break;
                            
                        case ixgbe_sfp_type_sr:
                        case ixgbe_sfp_type_1g_sx_core0:
                        case ixgbe_sfp_type_1g_sx_core1:
                            if (flow_tx || flow_rx) {
                                midx = MIDX_1GB_SX_FC;
                            } else {
                                midx = MIDX_1GB_SX;
                            }
                            break;
                            
                        case ixgbe_sfp_type_lr:
                        case ixgbe_sfp_type_1g_lx_core0:
                        case ixgbe_sfp_type_1g_lx_core1:
                            if (flow_tx || flow_rx) {
                                midx = MIDX_1GB_LX_FC;
                            } else {
                                midx = MIDX_1GB_LX;
                            }
                            break;

                        default:
                            if (flow_tx || flow_rx) {
                                midx = MIDX_1GB_SX_FC;
                            } else {
                                midx = MIDX_1GB_SX;
                            }
                            break;
                    }
                    pollParams.pollIntervalTime = 170000;
                    break;
                    
                case ixgbe_media_type_copper:
                    if (flow_tx || flow_rx) {
                        midx = MIDX_1000FC;
                    } else {
                        midx = MIDX_1000;
                    }
                    break;
                    
                default:
                    midx = MIDX_AUTO;
                    IOLog("Unsupported medium found.\n");
                    break;
            }
            speed_str = "1 Gbps";
            break;
            
        case IXGBE_LINK_SPEED_100_FULL:
            if (flow_tx || flow_rx) {
                midx = MIDX_100FC;
            } else {
                midx = MIDX_100;
            }
            speed_str = "100 Mbps";
            break;
                        
        default:
            speed_str = "unknown speed";
            break;
    }
    set_bit(__LINK_UP, &stateFlags);
    
    medium = mediaTable[midx];
    
    setLinkStatus((kIONetworkLinkValid | kIONetworkLinkActive), medium, medium->getSpeed(), NULL);
    
    netif->setPacketPollingParameters(&pollParams, 0);
    DebugLog("pollIntervalTime: %lluµs\n", (pollParams.pollIntervalTime / 1000));
    
    if (flow_rx && flow_tx)
        flow_str = flowControlNames[kFlowControlTypeRxTx];
    else if (flow_rx)
        flow_str = flowControlNames[kFlowControlTypeRx];
    else if (flow_tx)
        flow_str = flowControlNames[kFlowControlTypeTx];
    else
        flow_str = flowControlNames[kFlowControlTypeNone];
    
    eee_str = eeeNames[kEEETypeNo];
    
    IOLog("Link up on en%u, %s, full-duplex%s%s\n", netif->getUnitNumber(), speed_str, flow_str, eee_str);
    
    requireMaxBusStall(kMaxDmaLatency);
    requireMaxInterruptDelay(kMaxIntrLatency);
    
    /* Start output thread, statistics update and hang check. */
    netif->startOutputThread();
    
#ifdef DEBUG_OFF
    struct ixgbe_reg_info *reginfo;
    
    for (reginfo = (struct ixgbe_reg_info *)ixgbe_reg_info_tbl;
         reginfo->name; reginfo++) {
        ixgbe_regdump(hw, reginfo);
    }
#endif /* DEBUG */
    DebugLog("EIMS: 0x%0x IVAR[0]: 0x%0x\n", (unsigned int)IXGBE_READ_REG(hw, IXGBE_EIMS), (unsigned int)IXGBE_READ_REG(hw, IXGBE_IVAR(0)));
    DebugLog("rxActiveQueueMask: 0x%0x txActiveQueueMask[0]: 0x%0x\n", rxActiveQueueMask, txActiveQueueMask);
}

/**
 * ixgbeWatchdogUpdateLink - update the link status
 * @adapter: pointer to the device adapter structure
 **/
void IntelLucy::ixgbeWatchdogUpdateLink(struct ixgbe_adapter *adapter)
{
    struct ixgbe_hw *hw = &adapter->hw;
    uint64_t uptime;
    u32 link_speed = adapter->link_speed;
    bool link_up = adapter->link_up;
    bool pfc_en = adapter->dcb_cfg.pfc_mode_enable;

    if (!(adapter->flags & IXGBE_FLAG_NEED_LINK_UPDATE))
        return;

    if (hw->mac.ops.check_link) {
        hw->mac.ops.check_link(hw, &link_speed, &link_up, false);
    } else {
        /* always assume link is up, if no check link function */
        link_speed = IXGBE_LINK_SPEED_10GB_FULL;
        link_up = true;
    }

    if (adapter->ixgbe_ieee_pfc)
        pfc_en |= !!(adapter->ixgbe_ieee_pfc->pfc_en);

    if (link_up && !((adapter->flags & IXGBE_FLAG_DCB_ENABLED) && pfc_en)) {
        hw->mac.ops.fc_enable(hw);
        //ixgbe_set_rx_drop_en(adapter);
    }

    clock_get_uptime((uint64_t *)&uptime);
    
    if (link_up || (uptime > (adapter->link_check_timeout + timeoutCheck))) {
        adapter->flags &= ~IXGBE_FLAG_NEED_LINK_UPDATE;
        IXGBE_WRITE_REG(hw, IXGBE_EIMS, IXGBE_EIMC_LSC);
        IXGBE_WRITE_FLUSH(hw);
    }
    adapter->link_up = link_up;
    adapter->link_speed = link_speed;
}

void IntelLucy::ixgbeConfigLink(const IONetworkMedium *medium)
{
    struct ixgbe_adapter *adapter = &adapterData;
    struct ixgbe_hw *hw = &adapter->hw;
    int err = IXGBE_ERR_LINK_SETUP;
    UInt32 index = medium->getIndex();
    UInt32 speed = IXGBE_LINK_SPEED_82599_AUTONEG;
    UInt32 fc = ixgbe_fc_none;
    bool autoneg = false;
    
    if (index == MIDX_AUTO) {
        if (hw->mac.ops.get_link_capabilities) {
            err = hw->mac.ops.get_link_capabilities(hw, &speed, &autoneg);
            
            if (err)
                goto done;
            
            fc = ixgbe_fc_none;
        }
    } else {
        medium2Advertise(medium, &speed, &fc);
    }
    hw->phy.autoneg_advertised = speed;
    
    if (adapter->link_up && (speed == adapter->link_speed))
        goto config_fc;

    /* This sets the link speed and restarts auto-neg. */
    while (test_and_set_bit(__IXGBE_IN_SFP_INIT, &adapter->state))
        usleep_range(1000, 2000);

    hw->mac.autotry_restart = true;

    if (hw->mac.ops.setup_link)
        err = hw->mac.ops.setup_link(hw, speed, true);

    clear_bit(__IXGBE_IN_SFP_INIT, &adapter->state);

    if (err) {
        IOLog("Failed to set up link: %d.\n", err);
        goto done;
    }
    
config_fc:
    if (autoneg) {
        hw->mac.ops.fc_autoneg(hw);
    } else {
        hw->fc.requested_mode = (enum ixgbe_fc_mode)fc;
        hw->mac.ops.setup_fc(hw);
    }
    adapter->flags |= IXGBE_FLAG_NEED_LINK_UPDATE;
    clock_get_uptime((uint64_t *)&adapter->link_check_timeout);
    
    /* Reschedule next timeout. */
    ixgbeServiceEventSchedule(adapter);

done:
    return;
}

bool IntelLucy::ixgbeRingTxPending(struct ixgbe_adapter *adapter)
{
    int i;
    
    for (i = 0; i < adapter->num_tx_queues; i++) {
        if (txRing[i].txNumFreeDesc < kNumTxDesc)
            return true;
    }
    return false;
}

/**
 * ixgbeWatchdogFlushTx - flush queues on link down
 * @adapter: pointer to the device adapter structure
 **/
void IntelLucy::ixgbeWatchdogFlushTx(struct ixgbe_adapter *adapter)
{
    if (!test_bit(__LINK_UP, &stateFlags)) {
        if (ixgbeRingTxPending(adapter)){
            /* We've lost link, so the controller stops DMA,
             * but we've got queued Tx work that's never going
             * to get done, so reset controller to flush Tx.
             * (Do the reset outside of interrupt context).
             */
            DebugLog("Initiating reset to clear Tx work after link loss.\n");
            set_bit(__IXGBE_RESET_REQUESTED, &adapter->state);
        }
    }
}

/**
 * ixgbeWatchdogSubtask - check and bring link up
 * @adapter: pointer to the device adapter structure
 **/
void IntelLucy::ixgbeWatchdogSubtask(struct ixgbe_adapter *adapter)
{
    /* if interface is down, removing or resetting, do nothing */
    if (test_bit(__IXGBE_DOWN, &adapter->state) ||
        test_bit(__IXGBE_REMOVING, &adapter->state) ||
        test_bit(__IXGBE_RESETTING, &adapter->state))
        return;

    ixgbeWatchdogUpdateLink(adapter);

    if (adapter->link_up)
        ixgbeWatchdogLinkIsUp(adapter);
    else
        ixgbeWatchdogLinkIsDown(adapter);

    ixgbeUpdateStats(adapter);

    //ixgbeWatchdogFlushTx(adapter);
}


/**
 * ixgbeServiceTask - manages and runs subtasks
 **/
void IntelLucy::ixgbeServiceTask()
{
    struct ixgbe_adapter *adapter = &adapterData;
    
    //DebugLog("Service Task: state: %x\n", adapter->state);

    if (ixgbe_removed(adapter->hw.hw_addr)) {
        if (!test_bit(__IXGBE_DOWN, &adapter->state)) {
            ixgbeDown(adapter);
        }
        goto done;
    }
    if (ixgbe_check_fw_error(adapter)) {
        goto done;
    }
    ixgbeResetSubtask(adapter);
    ixgbePhyInterruptSubtask(adapter);
    ixgbeSfpDetectionSubtask(adapter);
    ixgbeSfpLinkConfigSubtask(adapter);
    ixgbeCheckOvertempSubtask(adapter);
    ixgbeWatchdogSubtask(adapter);
    //ixgbe_fdir_reinit_subtask(adapter);
    ixgbeCheckHangSubtask(adapter);
/*
    if (test_bit(__IXGBE_PTP_RUNNING, &adapter->state)) {
        ixgbe_ptp_overflow_check(adapter);
        
        if (adapter->flags & IXGBE_FLAG_RX_HWTSTAMP_IN_REGISTER)
            ixgbe_ptp_rx_hang(adapter);
        
        ixgbe_ptp_tx_hang(adapter);
    }*/
done:
    ixgbeServiceEventComplete(adapter);
}

void IntelLucy::ixgbeServiceEventSchedule(struct ixgbe_adapter *adapter)
{
    DebugLog("ixgbeServiceEventSchedule\n");

    if (!test_bit(__IXGBE_DOWN, &adapter->state) &&
        !test_bit(__IXGBE_REMOVING, &adapter->state) &&
        !test_and_set_bit(__IXGBE_SERVICE_SCHED, &adapter->state)) {
        timerSource->cancelTimeout();
        timerSource->setTimeoutUS(kTimeout10us);
    }
}

void IntelLucy::ixgbeServiceEventComplete(struct ixgbe_adapter *adapter)
{
    clear_bit(__IXGBE_SERVICE_SCHED, &adapter->state);
}

