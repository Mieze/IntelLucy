/* IntelLucyPhy.cpp -- IntelLucy phy specific routines.
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


#include "uapi-mdio.h"
#include "ixgbe.h"
#include "ixgbe_phy.h"
#include "IntelLucy.hpp"

static int __mdiobus_c45_read(struct mii_bus *bus, int addr, int devad, u32 regnum);
static int __mdiobus_read(struct mii_bus *bus, int addr, u32 regnum);
static int __mdiobus_c45_write(struct mii_bus *bus, int addr, int devad, u32 regnum,
                               u16 val);
static int __mdiobus_write(struct mii_bus *bus, int addr, u32 regnum, u16 val);

static int mdiobus_read(struct mii_bus *bus, int addr, u32 regnum);
static int mdiobus_c45_read(struct mii_bus *bus, int addr, int devad, u32 regnum);
static int mdiobus_c45_write(struct mii_bus *bus, int addr, int devad, u32 regnum,
                             u16 val);
static int mdiobus_write(struct mii_bus *bus, int addr, u32 regnum, u16 val);

static s32 ixgbe_mii_bus_read_c45(struct mii_bus *bus, int devad, int addr,
                                  int regnum);
static s32 ixgbe_mii_bus_write_c22(struct mii_bus *bus, int addr, int regnum,
                                   u16 val);
static s32 ixgbe_mii_bus_write_c45(struct mii_bus *bus, int addr, int devad,
                                   int regnum, u16 val);

#pragma mark --- auxiliary functions ---

/**
 *  ixgbe_msca_cmd - Write the command register and poll for completion/timeout
 *  @hw: pointer to hardware structure
 *  @cmd: command register value to write
 **/
s32 ixgbe_msca_cmd(struct ixgbe_hw *hw, u32 cmd)
{
    s32 ret = -ETIMEDOUT;
    int i;
    
    IXGBE_WRITE_REG(hw, IXGBE_MSCA, cmd);

    for (i = 0; i < IXGBE_MDIO_COMMAND_TIMEOUT; i++) {
        IODelay(10);

        cmd = IXGBE_READ_REG(hw, IXGBE_MSCA);
        
        if (!(cmd & IXGBE_MSCA_MDI_COMMAND)) {
            ret = 0;
            goto done;
        }

    }
done:
    return ret;
}

#pragma mark --- mdiobus functions ---

/**
 * __mdiobus_c45_read - Unlocked version of the mdiobus_c45_read function
 * @bus: the mii_bus struct
 * @addr: the phy address
 * @devad: device address to read
 * @regnum: register number to read
 *
 * Read a MDIO bus register. Caller must hold the mdio bus lock.
 *
 * NOTE: MUST NOT be called from interrupt context.
 */
static int __mdiobus_c45_read(struct mii_bus *bus, int addr, int devad, u32 regnum)
{
    int retval;

    if (bus->read_c45)
        retval = bus->read_c45(bus, addr, devad, regnum);
    else
        retval = -EOPNOTSUPP;

    return retval;
}

/**
 * mdiobus_c45_read - Convenience function for reading a given MII mgmt register
 * @bus: the mii_bus struct
 * @addr: the phy address
 * @devad: device address to read
 * @regnum: register number to read
 *
 * NOTE: MUST NOT be called from interrupt context,
 * because the bus read/write functions may wait for an interrupt
 * to conclude the operation.
 */
static int mdiobus_c45_read(struct mii_bus *bus, int addr, int devad, u32 regnum)
{
    int retval;

    mutex_lock(&bus->mdio_lock);
    retval = __mdiobus_c45_read(bus, addr, devad, regnum);
    mutex_unlock(&bus->mdio_lock);

    return retval;
}

/**
 * __mdiobus_read - Unlocked version of the mdiobus_read function
 * @bus: the mii_bus struct
 * @addr: the phy address
 * @regnum: register number to read
 *
 * Read a MDIO bus register. Caller must hold the mdio bus lock.
 *
 * NOTE: MUST NOT be called from interrupt context.
 */
static int __mdiobus_read(struct mii_bus *bus, int addr, u32 regnum)
{
    int retval;

    if (bus->read)
        retval = bus->read(bus, addr, regnum);
    else
        retval = -EOPNOTSUPP;

    return retval;
}

/**
 * mdiobus_read - Convenience function for reading a given MII mgmt register
 * @bus: the mii_bus struct
 * @addr: the phy address
 * @regnum: register number to read
 *
 * NOTE: MUST NOT be called from interrupt context,
 * because the bus read/write functions may wait for an interrupt
 * to conclude the operation.
 */
static int mdiobus_read(struct mii_bus *bus, int addr, u32 regnum)
{
    int retval;

    mutex_lock(&bus->mdio_lock);
    retval = __mdiobus_read(bus, addr, regnum);
    mutex_unlock(&bus->mdio_lock);

    return retval;
}

/**
 * __mdiobus_c45_write - Unlocked version of the mdiobus_write function
 * @bus: the mii_bus struct
 * @addr: the phy address
 * @devad: device address to read
 * @regnum: register number to write
 * @val: value to write to @regnum
 *
 * Write a MDIO bus register. Caller must hold the mdio bus lock.
 *
 * NOTE: MUST NOT be called from interrupt context.
 */
static int __mdiobus_c45_write(struct mii_bus *bus, int addr, int devad, u32 regnum,
            u16 val)
{
    int err;

    if (bus->write_c45)
        err = bus->write_c45(bus, addr, devad, regnum, val);
    else
        err = -EOPNOTSUPP;

    return err;
}

/**
 * mdiobus_c45_write - Convenience function for writing a given MII mgmt register
 * @bus: the mii_bus struct
 * @addr: the phy address
 * @devad: device address to read
 * @regnum: register number to write
 * @val: value to write to @regnum
 *
 * NOTE: MUST NOT be called from interrupt context,
 * because the bus read/write functions may wait for an interrupt
 * to conclude the operation.
 */
static int mdiobus_c45_write(struct mii_bus *bus, int addr, int devad, u32 regnum,
              u16 val)
{
    int err;

    mutex_lock(&bus->mdio_lock);
    err = __mdiobus_c45_write(bus, addr, devad, regnum, val);
    mutex_unlock(&bus->mdio_lock);

    return err;
}

/**
 * __mdiobus_write - Unlocked version of the mdiobus_write function
 * @bus: the mii_bus struct
 * @addr: the phy address
 * @regnum: register number to write
 * @val: value to write to @regnum
 *
 * Write a MDIO bus register. Caller must hold the mdio bus lock.
 *
 * NOTE: MUST NOT be called from interrupt context.
 */
static int __mdiobus_write(struct mii_bus *bus, int addr, u32 regnum, u16 val)
{
    int err;

    if (bus->write)
        err = bus->write(bus, addr, regnum, val);
    else
        err = -EOPNOTSUPP;

    return err;
}

/**
 * mdiobus_write - Convenience function for writing a given MII mgmt register
 * @bus: the mii_bus struct
 * @addr: the phy address
 * @regnum: register number to write
 * @val: value to write to @regnum
 *
 * NOTE: MUST NOT be called from interrupt context,
 * because the bus read/write functions may wait for an interrupt
 * to conclude the operation.
 */
static int mdiobus_write(struct mii_bus *bus, int addr, u32 regnum, u16 val)
{
    int err;

    mutex_lock(&bus->mdio_lock);
    err = __mdiobus_write(bus, addr, regnum, val);
    mutex_unlock(&bus->mdio_lock);

    return err;
}


#pragma mark --- mdio functions ---

int ixgbe_mdio_read(struct net_device *netdev, int prtad, int devad, u16 addr)
{
    struct ixgbe_adapter *adapter = netdev_priv(netdev);
    struct ixgbe_hw *hw = &adapter->hw;
    u16 value;
    int rc;

    if (adapter->mii_bus) {
        int regnum = addr;

        if (devad != MDIO_DEVAD_NONE)
            return mdiobus_c45_read(adapter->mii_bus, prtad,
                        devad, regnum);

        return mdiobus_read(adapter->mii_bus, prtad, regnum);
    }

    if (prtad != hw->phy.mdio.prtad)
        return -EINVAL;
    rc = hw->phy.ops.read_reg(hw, addr, devad, &value);
    if (!rc)
        rc = value;
    return rc;
}

int ixgbe_mdio_write(struct net_device *netdev, int prtad, int devad,
                u16 addr, u16 value)
{
    struct ixgbe_adapter *adapter = netdev_priv(netdev);
    struct ixgbe_hw *hw = &adapter->hw;

    if (adapter->mii_bus) {
        int regnum = addr;

        if (devad != MDIO_DEVAD_NONE)
            return mdiobus_c45_write(adapter->mii_bus, prtad, devad,
                         regnum, value);

        return mdiobus_write(adapter->mii_bus, prtad, regnum, value);
    }

    if (prtad != hw->phy.mdio.prtad)
        return -EINVAL;
    return hw->phy.ops.write_reg(hw, addr, devad, value);
}

/**
 * mdio45_probe - probe for an MDIO (clause 45) device
 * @mdio: MDIO interface
 * @prtad: Expected PHY address
 *
 * This sets @prtad and @mmds in the MDIO interface if successful.
 * Returns 0 on success, negative on error.
 */
int mdio45_probe(struct mdio_if_info *mdio, int prtad)
{
    int mmd, stat2, devs1, devs2;

    /* Assume PHY must have at least one of PMA/PMD, WIS, PCS, PHY
     * XS or DTE XS; give up if none is present. */
    for (mmd = 1; mmd <= 5; mmd++) {
        /* Is this MMD present? */
        stat2 = mdio->mdio_read(mdio->dev, prtad, mmd, MDIO_STAT2);
        if (stat2 < 0 ||
            (stat2 & MDIO_STAT2_DEVPRST) != MDIO_STAT2_DEVPRST_VAL)
            continue;

        /* It should tell us about all the other MMDs */
        devs1 = mdio->mdio_read(mdio->dev, prtad, mmd, MDIO_DEVS1);
        devs2 = mdio->mdio_read(mdio->dev, prtad, mmd, MDIO_DEVS2);
        if (devs1 < 0 || devs2 < 0)
            continue;

        mdio->prtad = prtad;
        mdio->mmds = devs1 | (devs2 << 16);
        return 0;
    }

    return -ENODEV;
}

#pragma mark --- mii bus functions ---

/**
 *  ixgbe_mii_bus_read_c22 - Read a clause 22 register
 *  @bus: pointer to mii_bus structure which points to our driver private
 *  @addr: address
 *  @regnum: register number
 **/
s32 ixgbe_mii_bus_read_c22(struct mii_bus *bus, int addr, int regnum)
{
    struct ixgbe_adapter *adapter = (ixgbe_adapter *)bus->priv;
    struct ixgbe_hw *hw = &adapter->hw;
    u32 gssr = hw->phy.phy_semaphore_mask;

    return ixgbe_mii_bus_read_generic_c22(hw, addr, regnum, gssr);
}

/**
 *  ixgbe_mii_bus_read_c45 - Read a clause 45 register
 *  @bus: pointer to mii_bus structure which points to our driver private
 *  @devad: device address to read
 *  @addr: address
 *  @regnum: register number
 **/
static s32 ixgbe_mii_bus_read_c45(struct mii_bus *bus, int devad, int addr,
                  int regnum)
{
    struct ixgbe_adapter *adapter = (ixgbe_adapter *)bus->priv;
    struct ixgbe_hw *hw = &adapter->hw;
    u32 gssr = hw->phy.phy_semaphore_mask;

    return ixgbe_mii_bus_read_generic_c45(hw, addr, devad, regnum, gssr);
}

/**
 *  ixgbe_mii_bus_write_c22 - Write a clause 22 register
 *  @bus: pointer to mii_bus structure which points to our driver private
 *  @addr: address
 *  @regnum: register number
 *  @val: value to write
 **/
static s32 ixgbe_mii_bus_write_c22(struct mii_bus *bus, int addr, int regnum,
                   u16 val)
{
    struct ixgbe_adapter *adapter = (ixgbe_adapter *)bus->priv;
    struct ixgbe_hw *hw = &adapter->hw;
    u32 gssr = hw->phy.phy_semaphore_mask;

    return ixgbe_mii_bus_write_generic_c22(hw, addr, regnum, val, gssr);
}

/**
 *  ixgbe_mii_bus_write_c45 - Write a clause 45 register
 *  @bus: pointer to mii_bus structure which points to our driver private
 *  @addr: address
 *  @devad: device address to read
 *  @regnum: register number
 *  @val: value to write
 **/
static s32 ixgbe_mii_bus_write_c45(struct mii_bus *bus, int addr, int devad,
                   int regnum, u16 val)
{
    struct ixgbe_adapter *adapter = (ixgbe_adapter *)bus->priv;
    struct ixgbe_hw *hw = &adapter->hw;
    u32 gssr = hw->phy.phy_semaphore_mask;

    return ixgbe_mii_bus_write_generic_c45(hw, addr, devad, regnum, val,
                           gssr);
}

/**
 * ixgbe_mii_bus_init - mii_bus structure setup
 * @hw: pointer to hardware structure
 *
 * Returns 0 on success, negative on failure
 *
 * ixgbe_mii_bus_init initializes a mii_bus structure in adapter
 **/
void ixgbe_mii_bus_init(struct ixgbe_adapter *adapter)
{
    //struct ixgbe_hw *hw = &adapter->hw;
    struct mii_bus *bus = adapter->mii_bus;
    
    bus->read = ixgbe_mii_bus_read_c22;
    bus->write = ixgbe_mii_bus_write_c22;
    bus->read_c45 = ixgbe_mii_bus_read_c45;
    bus->write_c45 = ixgbe_mii_bus_write_c45;
    
    bus->priv = adapter;
}
