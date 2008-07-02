/*
 * Copyright(c) 2007 Atheros Corporation. All rights reserved.
 *
 * Derived from Intel e1000 driver
 * Copyright(c) 1999 - 2005 Intel Corporation. All rights reserved.
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
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59
 * Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * There are a lot of defines in here that are unused and/or have cryptic
 * names.  Please leave them alone, as they're the closest thing we have
 * to a spec from Atheros at present. *ahem* -- CHS
 */

#include <linux/mii.h>

#include "atl1e.h"
/*
 * The little-endian AUTODIN II ethernet CRC calculations.
 * A big-endian version is also available.
 * This is slow but compact code.  Do not use this routine
 * for bulk data, use a table-based routine instead.
 * This is common code and should be moved to net/core/crc.c.
 * Chips may use the upper or lower CRC bits, and may reverse
 * and/or invert them.  Select the endian-ness that results
 * in minimal calculations.
 */
static int get_permanent_address(struct atl1e_hw *hw);

static u32 ether_crc_le(int length, unsigned char *data)
{
       u32 crc = ~0;  /* Initial value. */
       while (--length >= 0) {
               unsigned char current_octet = *data++;
               int bit;
               for (bit = 8; --bit >= 0; current_octet >>= 1) {
                       if ((crc ^ current_octet) & 1) {
                               crc >>= 1;
                               crc ^= 0xedb88320;
                       } else {
                               crc >>= 1;
                       }
               }
       }
       return ~crc;
}

/********************************************************************
 * Reset the transmit and receive units; mask and clear all interrupts.
 *
 * hw - Struct containing variables accessed by shared code
 * return : AT_SUCCESS  or  idle status (if error)
 ********************************************************************/
int atl1e_reset_hw(struct atl1e_hw *hw)
{
#define AT_HW_MAX_IDLE_DELAY 10
       u32 idle_status_data = 0;
       u16 pci_cfg_cmd_word = 0;
       int timeout = 0;

       DEBUGFUNC("atl1e_reset_hw");

       /* Workaround for PCI problem when BIOS sets MMRBC incorrectly. */
       atl1e_read_pci_cfg(hw, PCI_REG_COMMAND, &pci_cfg_cmd_word);
       if ((pci_cfg_cmd_word & (CMD_IO_SPACE |
                               CMD_MEMORY_SPACE | CMD_BUS_MASTER))
                       != (CMD_IO_SPACE | CMD_MEMORY_SPACE | CMD_BUS_MASTER)) {
               pci_cfg_cmd_word |= (CMD_IO_SPACE |
                                    CMD_MEMORY_SPACE | CMD_BUS_MASTER);
               atl1e_write_pci_cfg(hw, PCI_REG_COMMAND, &pci_cfg_cmd_word);
       }

       /* Issue Soft Reset to the MAC.  This will reset the chip's
        * transmit, receive, DMA.  It will not effect
        * the current PCI configuration.  The global reset bit is self-
        * clearing, and should clear within a microsecond.
        */
       AT_WRITE_REG(hw, REG_MASTER_CTRL,
                       MASTER_CTRL_LED_MODE | MASTER_CTRL_SOFT_RST);
       wmb();
       msec_delay(1);

       /* Wait at least 10ms for All module to be Idle */
       for (timeout = 0; timeout < AT_HW_MAX_IDLE_DELAY; timeout++) {
               idle_status_data = AT_READ_REG(hw, REG_IDLE_STATUS);
               if (idle_status_data == 0)
                       break;
               msec_delay(1);
               cpu_relax();
       }

       if (timeout >= AT_HW_MAX_IDLE_DELAY) {
               DEBUGOUT("MAC state machine cann't be idle since"
                         " disabled for 10ms second\n");
               return AT_ERR_TIMEOUT;
       }

       return AT_SUCCESS;
#undef AT_HW_MAX_IDLE_DELAY
}


/*********************************************************************
 * Reads the adapter's MAC address from the EEPROM
 *
 * hw - Struct containing variables accessed by shared code
 *********************************************************************/
int atl1e_read_mac_addr(struct atl1e_hw *hw)
{
       int err = 0;

       DEBUGFUNC("atl1e_read_mac_addr");

       err = get_permanent_address(hw);
       if (err)
               return AT_ERR_EEPROM;
       memcpy(hw->mac_addr, hw->perm_mac_addr, sizeof(hw->perm_mac_addr));
       return AT_SUCCESS;
}

/*********************************************************************
 * Hashes an address to determine its location in the multicast table
 *
 * hw - Struct containing variables accessed by shared code
 * mc_addr - the multicast address to hash
 *********************************************************************/
/*
 * atl1e_hash_mc_addr
 *  purpose
 *      set hash value for a multicast address
 *      hash calcu processing :
 *          1. calcu 32bit CRC for multicast address
 *          2. reverse crc with MSB to LSB
 */
u32 atl1e_hash_mc_addr(struct atl1e_hw *hw, u8 *mc_addr)
{
       u32 crc32;
       u32 value = 0;
       int i;

       crc32 = ether_crc_le(6, mc_addr);
       crc32 = ~crc32;
       for (i = 0; i < 32; i++)
               value |= (((crc32 >> i) & 1) << (31 - i));

       return value;
}


/********************************************************************
 * Sets the bit in the multicast table corresponding to the hash value.
 *
 * hw - Struct containing variables accessed by shared code
 * hash_value - Multicast address hash value
 ********************************************************************/
void atl1e_hash_set(struct atl1e_hw *hw, u32 hash_value)
{
       u32 hash_bit, hash_reg;
       u32 mta;

       /* The HASH Table  is a register array of 2 32-bit registers.
        * It is treated like an array of 64 bits.  We want to set
        * bit BitArray[hash_value]. So we figure out what register
        * the bit is in, read it, OR in the new bit, then write
        * back the new value.  The register is determined by the
        * upper 7 bits of the hash value and the bit within that
        * register are determined by the lower 5 bits of the value.
        */
       hash_reg = (hash_value >> 31) & 0x1;
       hash_bit = (hash_value >> 26) & 0x1F;

       mta = AT_READ_REG_ARRAY(hw, REG_RX_HASH_TABLE, hash_reg);

       mta |= (1 << hash_bit);

       AT_WRITE_REG_ARRAY(hw, REG_RX_HASH_TABLE, hash_reg, mta);
}


/*
 * atl1e_init_pcie - init PCIE module
 */
static void atl1e_init_pcie(struct atl1e_hw *hw)
{
       u32 value;
       /* comment 2lines below to save more power when sususpend
          value = LTSSM_TEST_MODE_DEF;
          AT_WRITE_REG(hw, REG_LTSSM_TEST_MODE, value);
        */

       /* pcie flow control mode change */
       value = AT_READ_REG(hw, 0x1008);
       value |= 0x8000;
       AT_WRITE_REG(hw, 0x1008, value);
}

/********************************************************************
 * Performs basic configuration of the adapter.
 *
 * hw - Struct containing variables accessed by shared code
 * Assumes that the controller has previously been reset and is in a
 * post-reset uninitialized state. Initializes multicast table,
 * and  Calls routines to setup link
 * Leaves the transmit and receive units disabled and uninitialized.
 ********************************************************************/
int atl1e_init_hw(struct atl1e_hw *hw)
{
       s32 ret_val = 0;

       DEBUGFUNC("atl1e_init_hw");

       atl1e_init_pcie(hw);


       /* Zero out the Multicast HASH table */
       /* clear the old settings from the multicast hash table */
       AT_WRITE_REG(hw, REG_RX_HASH_TABLE, 0);
       AT_WRITE_REG_ARRAY(hw, REG_RX_HASH_TABLE, 1, 0);

       ret_val = atl1e_phy_init(hw);

       DEBUGOUT1("atl1e_init_hw: ret: %d", ret_val);

       return ret_val;
}

int atl1e_restart_autoneg(struct atl1e_hw *hw)
{
       int err = 0;

       err = atl1e_write_phy_reg(hw, MII_ADVERTISE, hw->mii_autoneg_adv_reg);
       if (err)
               return err;

       if (hw->nic_type == athr_l1e || hw->nic_type == athr_l2e_revA) {
               err = atl1e_write_phy_reg(hw, MII_AT001_CR,
                                      hw->mii_1000t_ctrl_reg);
               if (err)
                       return err;
       }

       err = atl1e_write_phy_reg(hw, MII_BMCR,
                       MII_CR_RESET | MII_CR_AUTO_NEG_EN |
                       MII_CR_RESTART_AUTO_NEG);
       return err;
}


/******************************************************************************
 * Detects the current speed and duplex settings of the hardware.
 *
 * hw - Struct containing variables accessed by shared code
 * speed - Speed of the connection
 * duplex - Duplex setting of the connection
 *****************************************************************************/
int atl1e_get_speed_and_duplex(struct atl1e_hw *hw, u16 *speed, u16 *duplex)
{
       int err;
       u16 phy_data;

       DEBUGFUNC("atl1e_get_speed_and_duplex");

       /* Read   PHY Specific Status Register (17) */
       err = atl1e_read_phy_reg(hw, MII_AT001_PSSR, &phy_data);
       if (err)
               return err;

       if (!(phy_data & MII_AT001_PSSR_SPD_DPLX_RESOLVED))
               return AT_ERR_PHY_RES;

       switch (phy_data & MII_AT001_PSSR_SPEED) {
       case MII_AT001_PSSR_1000MBS:
               *speed = SPEED_1000;
               DEBUGOUT("1000 Mbps");
               break;
       case MII_AT001_PSSR_100MBS:
               *speed = SPEED_100;
               DEBUGOUT("100 Mbs, ");
               break;
       case MII_AT001_PSSR_10MBS:
               *speed = SPEED_10;
               DEBUGOUT("10 Mbs, ");
               break;
       default:
               DEBUGOUT("Error Speed !\n");
               return AT_ERR_PHY_SPEED;
               break;
       }

       if (phy_data & MII_AT001_PSSR_DPLX) {
               *duplex = FULL_DUPLEX;
               DEBUGOUT("Full Duplex");
       } else {
               *duplex = HALF_DUPLEX;
               DEBUGOUT(" Half Duplex");
       }

       return AT_SUCCESS;
}

/*********************************************************************
 * Reads the value from a PHY register
 * hw - Struct containing variables accessed by shared code
 * reg_addr - address of the PHY register to read
 *********************************************************************/
int atl1e_read_phy_reg(struct atl1e_hw *hw, u16 reg_addr, u16 *phy_data)
{
       u32 val;
       int i;

       DEBUGFUNC("atl1e_read_phy_reg");

       val = ((u32)(reg_addr&MDIO_REG_ADDR_MASK)) << MDIO_REG_ADDR_SHIFT |
               MDIO_START | MDIO_SUP_PREAMBLE | MDIO_RW |
               MDIO_CLK_25_4 << MDIO_CLK_SEL_SHIFT;

       AT_WRITE_REG(hw, REG_MDIO_CTRL, val);

       wmb();

       for (i = 0; i < MDIO_WAIT_TIMES; i++) {
               usec_delay(2);
               val = AT_READ_REG(hw, REG_MDIO_CTRL);
               if (!(val & (MDIO_START | MDIO_BUSY)))
                       break;
               wmb();
       }
       if (!(val & (MDIO_START | MDIO_BUSY))) {
               *phy_data = (u16)val;
               return AT_SUCCESS;
       }

       return AT_ERR_PHY;
}

/********************************************************************
 * Writes a value to a PHY register
 * hw - Struct containing variables accessed by shared code
 * reg_addr - address of the PHY register to write
 * data - data to write to the PHY
 ********************************************************************/
int atl1e_write_phy_reg(struct atl1e_hw *hw, u32 reg_addr, u16 phy_data)
{
       int i;
       u32 val;

       val = ((u32)(phy_data & MDIO_DATA_MASK)) << MDIO_DATA_SHIFT |
              (reg_addr&MDIO_REG_ADDR_MASK) << MDIO_REG_ADDR_SHIFT |
              MDIO_SUP_PREAMBLE |
              MDIO_START |
              MDIO_CLK_25_4 << MDIO_CLK_SEL_SHIFT;

       AT_WRITE_REG(hw, REG_MDIO_CTRL, val);
       wmb();

       for (i = 0; i < MDIO_WAIT_TIMES; i++) {
               usec_delay(2);
               val = AT_READ_REG(hw, REG_MDIO_CTRL);
               if (!(val & (MDIO_START | MDIO_BUSY)))
                       break;
               wmb();
       }

       if (!(val & (MDIO_START | MDIO_BUSY)))
               return AT_SUCCESS;

       return AT_ERR_PHY;
}

/********************************************************************
 * Configures PHY autoneg and flow control advertisement settings
 *
 * hw - Struct containing variables accessed by shared code
 ********************************************************************/
static int atl1e_phy_setup_autoneg_adv(struct atl1e_hw *hw)
{
       s32 ret_val;
       u16 mii_autoneg_adv_reg;
       u16 mii_1000t_ctrl_reg;

       DEBUGFUNC("atl1e_phy_setup_autoneg_adv");

       if (0 != hw->mii_autoneg_adv_reg)
               return AT_SUCCESS;
       /* Read the MII Auto-Neg Advertisement Register (Address 4/9). */
       mii_autoneg_adv_reg = MII_AR_DEFAULT_CAP_MASK;
       mii_1000t_ctrl_reg  = MII_AT001_CR_1000T_DEFAULT_CAP_MASK;

       /* Need to parse autoneg_advertised  and set up
        * the appropriate PHY registers.  First we will parse for
        * autoneg_advertised software override.  Since we can advertise
        * a plethora of combinations, we need to check each bit
        * individually.
        */

       /* First we clear all the 10/100 mb speed bits in the Auto-Neg
        * Advertisement Register (Address 4) and the 1000 mb speed bits in
        * the  1000Base-T control Register (Address 9).
        */
       mii_autoneg_adv_reg &= ~MII_AR_SPEED_MASK;
       mii_1000t_ctrl_reg  &= ~MII_AT001_CR_1000T_SPEED_MASK;

       /* Need to parse MediaType and setup the
        * appropriate PHY registers.
        */
       switch (hw->media_type) {
       case MEDIA_TYPE_AUTO_SENSOR:
               mii_autoneg_adv_reg |= (MII_AR_10T_HD_CAPS   |
                                       MII_AR_10T_FD_CAPS   |
                                       MII_AR_100TX_HD_CAPS |
                                       MII_AR_100TX_FD_CAPS);
               hw->autoneg_advertised = ADVERTISE_10_HALF  |
                                        ADVERTISE_10_FULL  |
                                        ADVERTISE_100_HALF |
                                        ADVERTISE_100_FULL;
               if (hw->nic_type == athr_l1e) {
                       mii_1000t_ctrl_reg |=
                               MII_AT001_CR_1000T_FD_CAPS;
                       hw->autoneg_advertised |= ADVERTISE_1000_FULL;
               }
               break;

       case MEDIA_TYPE_100M_FULL:
               mii_autoneg_adv_reg   |= MII_AR_100TX_FD_CAPS;
               hw->autoneg_advertised = ADVERTISE_100_FULL;
               break;

       case MEDIA_TYPE_100M_HALF:
               mii_autoneg_adv_reg   |= MII_AR_100TX_HD_CAPS;
               hw->autoneg_advertised = ADVERTISE_100_HALF;
               break;

       case MEDIA_TYPE_10M_FULL:
               mii_autoneg_adv_reg   |= MII_AR_10T_FD_CAPS;
               hw->autoneg_advertised = ADVERTISE_10_FULL;
               break;

       default:
               mii_autoneg_adv_reg   |= MII_AR_10T_HD_CAPS;
               hw->autoneg_advertised = ADVERTISE_10_HALF;
               break;
       }

       /* flow control fixed to enable all */
       mii_autoneg_adv_reg |= (MII_AR_ASM_DIR | MII_AR_PAUSE);

       hw->mii_autoneg_adv_reg = mii_autoneg_adv_reg;
       hw->mii_1000t_ctrl_reg  = mii_1000t_ctrl_reg;

       ret_val = atl1e_write_phy_reg(hw, MII_ADVERTISE, mii_autoneg_adv_reg);
       if (ret_val)
               return ret_val;

       if (hw->nic_type == athr_l1e || hw->nic_type == athr_l2e_revA) {
               ret_val = atl1e_write_phy_reg(hw, MII_AT001_CR,
                                          mii_1000t_ctrl_reg);
               if (ret_val)
                       return ret_val;
       }

       return AT_SUCCESS;
}

int atl1e_phy_init(struct atl1e_hw *hw)
{
       s32 ret_val;
       u16 phy_val;

       if (hw->phy_configured) {
               if (hw->re_autoneg) {
                       hw->re_autoneg = false;
                       return atl1e_restart_autoneg(hw);
               }
               return AT_SUCCESS;
       }

       /* RESET GPHY Core */
       AT_WRITE_REGW(hw, REG_GPHY_CTRL, GPHY_CTRL_DEFAULT);
       msec_delay(2);
       AT_WRITE_REGW(hw, REG_GPHY_CTRL, GPHY_CTRL_DEFAULT |
                     GPHY_CTRL_EXT_RESET);
       msec_delay(2);

       /* patches */
       /* p1. eable hibernation mode */
       ret_val = atl1e_write_phy_reg(hw, MII_DBG_ADDR, 0xB);
       if (ret_val)
               return ret_val;
       ret_val = atl1e_write_phy_reg(hw, MII_DBG_DATA, 0xBC00);
       if (ret_val)
               return ret_val;
       /* p2. set Class A/B for all modes */
       ret_val = atl1e_write_phy_reg(hw, MII_DBG_ADDR, 0);
       if (ret_val)
               return ret_val;
       phy_val = 0x02ef;
       /* remove Class AB */
       /* phy_val = hw->emi_ca ? 0x02ef : 0x02df; */
       ret_val = atl1e_write_phy_reg(hw, MII_DBG_DATA, phy_val);
       if (ret_val)
               return ret_val;
       /* p3. 10B ??? */
       ret_val = atl1e_write_phy_reg(hw, MII_DBG_ADDR, 0x12);
       if (ret_val)
               return ret_val;
       ret_val = atl1e_write_phy_reg(hw, MII_DBG_DATA, 0x4C04);
       if (ret_val)
               return ret_val;
       /* p4. 1000T power */
       ret_val = atl1e_write_phy_reg(hw, MII_DBG_ADDR, 0x4);
       if (ret_val)
               return ret_val;
       ret_val = atl1e_write_phy_reg(hw, MII_DBG_DATA, 0x8BBB);
       if (ret_val)
               return ret_val;

       ret_val = atl1e_write_phy_reg(hw, MII_DBG_ADDR, 0x5);
       if (ret_val)
               return ret_val;
       ret_val = atl1e_write_phy_reg(hw, MII_DBG_DATA, 0x2C46);
       if (ret_val)
               return ret_val;

       msec_delay(1);

       /*Enable PHY LinkChange Interrupt */
       ret_val = atl1e_write_phy_reg(hw, MII_INT_CTRL, 0xC00);
       if (ret_val) {
               DEBUGOUT("Error enable PHY linkChange Interrupt");
               return ret_val;
       }
       /* setup AutoNeg parameters */
       ret_val = atl1e_phy_setup_autoneg_adv(hw);
       if (ret_val) {
               DEBUGOUT("Error Setting up Auto-Negotiation");
               return ret_val;
       }
       /* SW.Reset & En-Auto-Neg to restart Auto-Neg*/
       DEBUGOUT("Restarting Auto-Neg");
       ret_val = atl1e_phy_commit(hw);
       if (ret_val) {
               DEBUGOUT("Error Resetting the phy");
               return ret_val;
       }

       hw->phy_configured = true;

       return AT_SUCCESS;
}

/*******************************************************************
 * Resets the PHY and make all config validate
 *
 * hw - Struct containing variables accessed by shared code
 *
 * Sets bit 15 and 12 of the MII control regiser (for F001 bug)
 *******************************************************************/
int atl1e_phy_commit(struct atl1e_hw *hw)
{
       int ret_val;
       u16 phy_data;

       DEBUGFUNC("atl1e_phy_commit");

       phy_data = MII_CR_RESET | MII_CR_AUTO_NEG_EN | MII_CR_RESTART_AUTO_NEG;

       ret_val = atl1e_write_phy_reg(hw, MII_BMCR, phy_data);
       if (ret_val) {
               u32 val;
               int i;
               /**************************************
                * pcie serdes link may be down !
                **************************************/
               DEBUGOUT("Auto-Neg make pcie phy link down !");

               for (i = 0; i < 25; i++) {
                       msec_delay(1);
                       val = AT_READ_REG(hw, REG_MDIO_CTRL);
                       if (!(val & (MDIO_START | MDIO_BUSY)))
                               break;
               }

               if (0 != (val & (MDIO_START | MDIO_BUSY))) {
                       AT_ERR("pcie linkdown at least for 25ms !\n");
                       return ret_val;
               }

               DEBUGOUT1("pcie linkup after %dms", i);
       }
       return AT_SUCCESS;
}

void set_mac_addr(struct atl1e_hw *hw)
{
       u32 value;
       /*
        * 00-0B-6A-F6-00-DC
        * 0:  6AF600DC 1: 000B
        * low dword
        */
       value = (((u32)hw->mac_addr[2]) << 24) |
               (((u32)hw->mac_addr[3]) << 16) |
               (((u32)hw->mac_addr[4]) << 8) |
               (((u32)hw->mac_addr[5])) ;
       AT_WRITE_REG_ARRAY(hw, REG_MAC_STA_ADDR, 0, value);
       /* hight dword */
       value = (((u32)hw->mac_addr[0]) << 8) |
               (((u32)hw->mac_addr[1])) ;
       AT_WRITE_REG_ARRAY(hw, REG_MAC_STA_ADDR, 1, value);
}


/**************** function about EEPROM ************************/
/*
 * check_eeprom_exist
 * return 0 if eeprom exist
 */
int check_eeprom_exist(struct atl1e_hw *hw)
{
       u32 value;

       value = AT_READ_REG(hw, REG_SPI_FLASH_CTRL);
       if (value & SPI_FLASH_CTRL_EN_VPD) {
               value &= ~SPI_FLASH_CTRL_EN_VPD;
               AT_WRITE_REG(hw, REG_SPI_FLASH_CTRL, value);
       }
       value = AT_READ_REGW(hw, REG_PCIE_CAP_LIST);
       return ((value & 0xFF00) == 0x6C00) ? 0 : 1;
}

/*
 * get_permanent_address
 * return 0 if get valid mac address,
 */
static int get_permanent_address(struct atl1e_hw *hw)
{
       u32 addr[2];
       u32 i, control;
       u16 reg_addr;
       u8  eth_addr[ETH_ALEN];
       bool key_valid;

       if (is_valid_ether_addr(hw->perm_mac_addr))
               return 0;

       /* init */
       addr[0] = addr[1] = 0;

       if (!check_eeprom_exist(hw)) {
               /* eeprom exist */
               reg_addr = 0;
               key_valid = false;
               /* Read out all EEPROM content */
               i = 0;
               while (1) {
                       if (read_eeprom(hw, i+0, &control)) {
                               if (key_valid) {
                                       if (reg_addr == REG_MAC_STA_ADDR)
                                               addr[0] = control;
                                       else if (reg_addr ==
                                                       (REG_MAC_STA_ADDR+4))
                                               addr[1] = control;

                                       key_valid = false;
                               } else if ((control & 0xff) == 0x5A) {
                                       key_valid = true;
                                       reg_addr = (u16)(control >> 16);
                               } else
                                       break;
                                       /*
                                        * assume data end while encount
                                        * an invalid KEYWORD
                                        */
                       } else {
                               break; /* read error */
                       }
                       i += 4;
               }

               *(u32 *) &eth_addr[2] = swab32(addr[0]);
               *(u16 *) &eth_addr[0] = swab16(*(u16 *) &addr[1]);

               if (is_valid_ether_addr(eth_addr)) {
                       memcpy(hw->perm_mac_addr, eth_addr, ETH_ALEN);
                       return 0;
               }
       }

       /* maybe MAC-address is from BIOS */
       addr[0] = AT_READ_REG(hw, REG_MAC_STA_ADDR);
       addr[1] = AT_READ_REG(hw, REG_MAC_STA_ADDR + 4);
       *(u32 *) &eth_addr[2] = swab32(addr[0]);
       *(u16 *) &eth_addr[0] = swab16(*(u16 *)&addr[1]);

       if (is_valid_ether_addr(eth_addr)) {
               memcpy(hw->perm_mac_addr, eth_addr, ETH_ALEN);
               return 0;
       }

       return 1;
}

bool write_eeprom(struct atl1e_hw *hw, u32 offset, u32 value)
{
       return true;
}

bool read_eeprom(struct atl1e_hw *hw, u32 offset, u32 *p_value)
{
       int i;
       u32 control;

       if (offset & 3)
               return false; /* address do not align */

       AT_WRITE_REG(hw, REG_VPD_DATA, 0);
       control = (offset & VPD_CAP_VPD_ADDR_MASK) << VPD_CAP_VPD_ADDR_SHIFT;
       AT_WRITE_REG(hw, REG_VPD_CAP, control);

       for (i = 0; i < 10; i++) {
               msec_delay(2);
               control = AT_READ_REG(hw, REG_VPD_CAP);
               if (control & VPD_CAP_VPD_FLAG)
                       break;
       }
       if (control & VPD_CAP_VPD_FLAG) {
               *p_value = AT_READ_REG(hw, REG_VPD_DATA);
               return true;
       }
       return false; /* timeout */
}

void atl1e_force_ps(struct atl1e_hw *hw)
{
       AT_WRITE_REGW(hw, REG_GPHY_CTRL,
                       GPHY_CTRL_PW_WOL_DIS | GPHY_CTRL_EXT_RESET);
}
