/*
 * Copyright(c) 2007 Atheros Corporation. All rights reserved.
 * Copyright(c) 2007 xiong huang <xiong.huang@atheros.com>
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

#ifndef _ATHEROS_H__
#define _ATHEROS_H__

#include <linux/version.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/udp.h>
#include <linux/mii.h>
#include <linux/io.h>


#include <linux/vmalloc.h>
#include <linux/pagemap.h>
#include <linux/tcp.h>
#include <net/checksum.h>
#include <net/ip6_checksum.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/if_vlan.h>


#define BAR_0   0
#define BAR_1   1
#define BAR_5   5

#define AT_TX_WATCHDOG  (5 * HZ)

#define AT_VLAN_TAG_TO_TPD_TAG(_vlan, _tpd)    \
       (_tpd) = (((_vlan) << 4) | (((_vlan) >> 13) & 7) |\
                 (((_vlan) >> 9) & 8))

#define AT_TPD_TAG_TO_VLAN_TAG(_tpd, _vlan)    \
       (_vlan) = (((_tpd) >> 8) | (((_tpd) & 0x77) << 9) |\
                  (((_tdp) & 0x88) << 5))

#include "atl1e_hw.h"

/* wrapper around a pointer to a socket buffer,
 * so a DMA handle can be stored along with the buffer */

#define AT_MAX_RECEIVE_QUEUE    4
#define AT_PAGE_NUM_PER_QUEUE   2

#define AT_DMA_HI_ADDR_MASK     0xffffffff00000000ULL
#define AT_DMA_LO_ADDR_MASK     0x00000000ffffffffULL


struct atl1e_tx_buffer {
       struct sk_buff *skb;
       u16 length;
       dma_addr_t dma;
};

struct atl1e_rx_page {
       dma_addr_t      dma;    /* receive rage DMA address */
       u8              *addr;   /* receive rage virtual address */
       dma_addr_t      write_offset_dma;  /* the DMA address which contain the
                                             receive data offset in the page */
       u32             *write_offset_addr; /* the virtaul address which contain
                                            the receive data offset in the page */
       u32             read_offset;       /* the offset where we have read */
};

struct atl1e_rx_page_desc {
       struct atl1e_rx_page   rx_page[AT_PAGE_NUM_PER_QUEUE];
       u8  rx_using;
       u16 rx_nxseq;
};

/* transmit packet descriptor (tpd) ring */
struct atl1e_tx_ring {
       struct atl1e_tpd_desc *desc;  /* descriptor ring virtual address  */
       dma_addr_t         dma;    /* descriptor ring physical address */
       u16                count;  /* the count of transmit rings  */
       rwlock_t           tx_lock;
       u16                next_to_use;
       atomic_t           next_to_clean;
       struct atl1e_tx_buffer *tx_buffer;
       dma_addr_t         cmb_dma;
       u32                *cmb;
};


/* receive packet descriptor ring */
struct atl1e_rx_ring {
       void            *desc;
       dma_addr_t      dma;
       int             size;
       u32             page_size; /* bytes length of rxf page */
       u32             real_page_size; /* real_page_size = page_size + jumbo + aliagn */
       struct atl1e_rx_page_desc       rx_page_desc[AT_MAX_RECEIVE_QUEUE];
};
/* board specific private data structure */

struct atl1e_adapter {
       struct net_device   *netdev;
       struct pci_dev      *pdev;
       struct vlan_group   *vlgrp;
       struct atl1e_hw        hw;
       struct atl1e_hw_stats  hw_stats;
       struct net_device_stats net_stats;

       bool pci_using_64;
       bool have_msi;
       u32 wol;
       u16 link_speed;
       u16 link_duplex;

       spinlock_t mdio_lock;
       spinlock_t tx_lock;
       atomic_t irq_sem;

       struct work_struct reset_task;
       struct work_struct link_chg_task;
       struct timer_list watchdog_timer;
       struct timer_list phy_config_timer;

       /* All Descriptor memory */
       dma_addr_t      ring_dma;
       void            *ring_vir_addr;
       int             ring_size;

       struct atl1e_tx_ring tx_ring;
       struct atl1e_rx_ring rx_ring;
       int num_rx_queues;

#ifdef CONFIG_ATL1E_NAPI
       struct napi_struct  napi;
#endif

       struct mii_if_info  mii;    /* MII interface info */
       unsigned long flags;
#define __AT_TESTING        0x0001
#define __AT_RESETTING      0x0002
#define __AT_DOWN           0x0003
       u32 bd_number;     /* board number;*/

       u32 pci_state[16];

       u32 *config_space;
};

#define AT_MII_LOCK(_adapter)                          \
       do {                                            \
               spin_lock(&(_adapter)->mdio_lock);      \
       } while (0)

#define AT_MII_UNLOCK(_adapter)                                \
       do {                                            \
               spin_unlock(&(_adapter)->mdio_lock);    \
       } while (0)

#define AT_MII_LOCK_IRQSAVE(_adapter, _flags)                          \
       do {                                                            \
               spin_lock_irqsave(&(_adapter)->mdio_lock, (_flags));    \
       } while (0)

#define AT_MII_UNLOCK_IRQRESTORE(_adapter, _flags)                      \
       do {                                                             \
               spin_unlock_irqrestore(&(_adapter)->mdio_lock, (_flags));\
       } while (0)

extern char atl1e_driver_name[];
extern char atl1e_driver_version[];

extern void atl1e_check_options(struct atl1e_adapter *adapter);
extern int cancel_work_sync(struct work_struct *work);
extern int atl1e_up(struct atl1e_adapter *adapter);
extern void atl1e_down(struct atl1e_adapter *adapter);
extern void atl1e_reinit_locked(struct atl1e_adapter *adapter);
extern s32 atl1e_reset_hw(struct atl1e_hw *hw);


#endif /* _ATHEROS_H__ */

