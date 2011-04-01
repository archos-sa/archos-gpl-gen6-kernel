/** @file wlan_main.c
  *  
  * @brief This file contains the major functions in WLAN
  * driver. It includes init, exit, open, close and main
  * thread etc..
  * 
  * (c) Copyright � 2003-2007, Marvell International Ltd. 
  *   
  * This software file (the "File") is distributed by Marvell International 
  * Ltd. under the terms of the GNU General Public License Version 2, June 1991 
  * (the "License").  You may use, redistribute and/or modify this File in 
  * accordance with the terms and conditions of the License, a copy of which 
  * is available along with the File in the gpl.txt file or by writing to 
  * the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 
  * 02111-1307 or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
  *
  * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE 
  * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE 
  * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about 
  * this warranty disclaimer.
  *
  */
/**
  * @mainpage M-WLAN Linux Driver
  *
  * @section overview_sec Overview
  *
  * The M-WLAN is a Linux reference driver for Marvell
  * 802.11 (a/b/g) WLAN chipset.
  * 
  * @section copyright_sec Copyright
  *
  * Copyright � Marvell International Ltd. and/or its affiliates, 2003-2007
  *
  */
/********************************************************
Change log:
	09/30/05: Add Doxygen format comments
	12/09/05: Add TX_QUEUE support	
	01/05/06: Add kernel 2.6.x support	
	01/11/06: Conditionalize new scan/join functions.
	01/12/06: Add TxLockFlag for UAPSD power save mode 
	          and Proprietary Periodic sleep support
********************************************************/

#include	"include.h"
//#include	"/usr/src/hufh/include/net/iw_handler.h" //hufh

/********************************************************
		Local Variables
********************************************************/

#ifdef ENABLE_PM
static struct pm_dev *wlan_pm_dev = NULL;
#endif

spinlock_t driver_lock = SPIN_LOCK_UNLOCKED;
//raw_spinlock_t driver_lock = RAW_SPIN_LOCK_UNLOCKED(driver_lock);
ulong driver_flags;

#define WLAN_TX_PWR_DEFAULT		20      /*100mW */
#define WLAN_TX_PWR_US_DEFAULT		20      /*100mW */
#define WLAN_TX_PWR_JP_DEFAULT		16      /*50mW */
#define WLAN_TX_PWR_FR_100MW		20      /*100mW */
#define WLAN_TX_PWR_EMEA_DEFAULT	20      /*100mW */

/* Format { Channel, Frequency (MHz), MaxTxPower } */
/* Band: 'B/G', Region: USA FCC/Canada IC */
static CHANNEL_FREQ_POWER channel_freq_power_US_BG[] = {
    {1, 2412, WLAN_TX_PWR_US_DEFAULT},
    {2, 2417, WLAN_TX_PWR_US_DEFAULT},
    {3, 2422, WLAN_TX_PWR_US_DEFAULT},
    {4, 2427, WLAN_TX_PWR_US_DEFAULT},
    {5, 2432, WLAN_TX_PWR_US_DEFAULT},
    {6, 2437, WLAN_TX_PWR_US_DEFAULT},
    {7, 2442, WLAN_TX_PWR_US_DEFAULT},
    {8, 2447, WLAN_TX_PWR_US_DEFAULT},
    {9, 2452, WLAN_TX_PWR_US_DEFAULT},
    {10, 2457, WLAN_TX_PWR_US_DEFAULT},
    {11, 2462, WLAN_TX_PWR_US_DEFAULT}
};

/* Band: 'B/G', Region: Europe ETSI */
static CHANNEL_FREQ_POWER channel_freq_power_EU_BG[] = {
    {1, 2412, WLAN_TX_PWR_EMEA_DEFAULT},
    {2, 2417, WLAN_TX_PWR_EMEA_DEFAULT},
    {3, 2422, WLAN_TX_PWR_EMEA_DEFAULT},
    {4, 2427, WLAN_TX_PWR_EMEA_DEFAULT},
    {5, 2432, WLAN_TX_PWR_EMEA_DEFAULT},
    {6, 2437, WLAN_TX_PWR_EMEA_DEFAULT},
    {7, 2442, WLAN_TX_PWR_EMEA_DEFAULT},
    {8, 2447, WLAN_TX_PWR_EMEA_DEFAULT},
    {9, 2452, WLAN_TX_PWR_EMEA_DEFAULT},
    {10, 2457, WLAN_TX_PWR_EMEA_DEFAULT},
    {11, 2462, WLAN_TX_PWR_EMEA_DEFAULT},
    {12, 2467, WLAN_TX_PWR_EMEA_DEFAULT},
    {13, 2472, WLAN_TX_PWR_EMEA_DEFAULT}
};

/* Band: 'B/G', Region: Spain */
static CHANNEL_FREQ_POWER channel_freq_power_SPN_BG[] = {
    {10, 2457, WLAN_TX_PWR_DEFAULT},
    {11, 2462, WLAN_TX_PWR_DEFAULT}
};

/* Band: 'B/G', Region: France */
static CHANNEL_FREQ_POWER channel_freq_power_FR_BG[] = {
    {10, 2457, WLAN_TX_PWR_FR_100MW},
    {11, 2462, WLAN_TX_PWR_FR_100MW},
    {12, 2467, WLAN_TX_PWR_FR_100MW},
    {13, 2472, WLAN_TX_PWR_FR_100MW}
};

/* Band: 'B/G', Region: Japan */
static CHANNEL_FREQ_POWER channel_freq_power_JPN41_BG[] = {
    {1, 2412, WLAN_TX_PWR_JP_DEFAULT},
    {2, 2417, WLAN_TX_PWR_JP_DEFAULT},
    {3, 2422, WLAN_TX_PWR_JP_DEFAULT},
    {4, 2427, WLAN_TX_PWR_JP_DEFAULT},
    {5, 2432, WLAN_TX_PWR_JP_DEFAULT},
    {6, 2437, WLAN_TX_PWR_JP_DEFAULT},
    {7, 2442, WLAN_TX_PWR_JP_DEFAULT},
    {8, 2447, WLAN_TX_PWR_JP_DEFAULT},
    {9, 2452, WLAN_TX_PWR_JP_DEFAULT},
    {10, 2457, WLAN_TX_PWR_JP_DEFAULT},
    {11, 2462, WLAN_TX_PWR_JP_DEFAULT},
    {12, 2467, WLAN_TX_PWR_JP_DEFAULT},
    {13, 2472, WLAN_TX_PWR_JP_DEFAULT}
};

/* Band: 'B/G', Region: Japan */
static CHANNEL_FREQ_POWER channel_freq_power_JPN40_BG[] = {
    {14, 2484, WLAN_TX_PWR_JP_DEFAULT}
};

/********************************************************
		Global Variables
********************************************************/

/* add by feng for thread conflict debug */
int threadrunning = 0;

/**
 * the structure for channel, frequency and power
 */
typedef struct _region_cfp_table
{
    u8 region;
    CHANNEL_FREQ_POWER *cfp_BG;
    int cfp_no_BG;
} region_cfp_table_t;

/**
 * the structure for the mapping between region and CFP
 */
region_cfp_table_t region_cfp_table[] = {
    {0x10,                      /*US FCC */
     channel_freq_power_US_BG,
     sizeof(channel_freq_power_US_BG) / sizeof(CHANNEL_FREQ_POWER),
     }
    ,
    {0x20,                      /*CANADA IC */
     channel_freq_power_US_BG,
     sizeof(channel_freq_power_US_BG) / sizeof(CHANNEL_FREQ_POWER),
     }
    ,
    {0x30, /*EU*/ channel_freq_power_EU_BG,
     sizeof(channel_freq_power_EU_BG) / sizeof(CHANNEL_FREQ_POWER),
     }
    ,
    {0x31, /*SPAIN*/ channel_freq_power_SPN_BG,
     sizeof(channel_freq_power_SPN_BG) / sizeof(CHANNEL_FREQ_POWER),
     }
    ,
    {0x32, /*FRANCE*/ channel_freq_power_FR_BG,
     sizeof(channel_freq_power_FR_BG) / sizeof(CHANNEL_FREQ_POWER),
     }
    ,
    {0x40, /*JAPAN*/ channel_freq_power_JPN40_BG,
     sizeof(channel_freq_power_JPN40_BG) / sizeof(CHANNEL_FREQ_POWER),
     }
    ,
    {0x41, /*JAPAN*/ channel_freq_power_JPN41_BG,
     sizeof(channel_freq_power_JPN41_BG) / sizeof(CHANNEL_FREQ_POWER),
     }
    ,
/*Add new region here */
};

/**
 * the rates supported by the card
 */
u8 WlanDataRates[WLAN_SUPPORTED_RATES] =
    { 0x02, 0x04, 0x0B, 0x16, 0x00, 0x0C, 0x12,
    0x18, 0x24, 0x30, 0x48, 0x60, 0x6C, 0x00
};

/**
 * the rates supported
 */
u8 SupportedRates[G_SUPPORTED_RATES] =
    { 0x82, 0x84, 0x8b, 0x96, 0x0c, 0x12, 0x18, 0x24, 0x30, 0x48, 0x60, 0x6c,
0 };

/**
 * the rates supported for ad-hoc G mode
 */
u8 AdhocRates_G[G_SUPPORTED_RATES] =
    { 0x82, 0x84, 0x8b, 0x96, 0x0c, 0x12, 0x18, 0x24, 0x30, 0x48, 0x60, 0x6c,
0 };

/**
 * the rates supported for ad-hoc B mode
 */
u8 AdhocRates_B[4] = { 0x82, 0x84, 0x8b, 0x96 };

/**
 * the global variable of a pointer to wlan_private
 * structure variable
 */
wlan_private *wlanpriv = NULL;

u32 DSFreqList[15] = {
    0, 2412000, 2417000, 2422000, 2427000, 2432000, 2437000, 2442000,
    2447000, 2452000, 2457000, 2462000, 2467000, 2472000, 2484000
};

/**
 * the table to keep region code
 */
u16 RegionCodeToIndex[MRVDRV_MAX_REGION_CODE] =
    { 0x10, 0x20, 0x30, 0x31, 0x32, 0x40, 0x41 };

/********************************************************
		Local Functions
********************************************************/

/** 
 *  @brief This function opens the network device
 *  
 *  @param dev     A pointer to net_device structure
 *  @return 	   WLAN_STATUS_SUCCESS
 */
static int
wlan_open(struct net_device *dev)
{
    wlan_private *priv = (wlan_private *) dev->priv;
    wlan_adapter *adapter = priv->adapter;

    ENTER();

    MODULE_GET;

    priv->open = TRUE;

    if (adapter->MediaConnectStatus == WlanMediaStateConnected)
        os_carrier_on(priv);
    else
        os_carrier_off(priv);

    os_start_queue(priv);

    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/** 
 *  @brief This function closes the network device
 *  
 *  @param dev     A pointer to net_device structure
 *  @return 	   WLAN_STATUS_SUCCESS
 */
static int
wlan_close(struct net_device *dev)
{
    wlan_private *priv = dev->priv;

    ENTER();

    /* Flush all the packets upto the OS before stopping */
    wlan_send_rxskbQ(priv);
    os_stop_queue(priv);
    os_carrier_off(priv);

    MODULE_PUT;

    priv->open = FALSE;

    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

#ifdef ENABLE_PM

/** 
 *  @brief This function is a callback function. it is called by
 *  kernel to enter or exit power saving mode.
 *  
 *  @param pmdev   A pointer to pm_dev
 *  @param pmreq   pm_request_t
 *  @param pmdata  A pointer to pmdata
 *  @return 	   WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
wlan_pm_callback(struct pm_dev *pmdev, pm_request_t pmreq, void *pmdata)
{
    wlan_private *priv = wlanpriv;
    wlan_adapter *Adapter = priv->adapter;
    struct net_device *dev = priv->wlan_dev.netdev;
    static BOOLEAN OS_Enable_DS = FALSE;

    PRINTM(INFO, "WPRM_PM_CALLBACK: pmreq = %d.\n", pmreq);

    switch (pmreq) {
    case PM_SUSPEND:
        PRINTM(INFO, "WPRM_PM_CALLBACK: enter PM_SUSPEND.\n");

#ifdef WPRM_DRV
        /* check WLAN_HOST_WAKEB */
        if (wprm_wlan_host_wakeb_is_triggered()) {
            PRINTM(MSG, "exit on GPIO-1 triggered.\n");
            return WLAN_STATUS_FAILURE;
        }
#endif

        /* in associated mode */
        if (Adapter->MediaConnectStatus == WlanMediaStateConnected) {
            if ((Adapter->PSState != PS_STATE_SLEEP)
                || !Adapter->bWakeupDevRequired || (Adapter->WakeupTries != 0)
                ) {
                PRINTM(INFO, "wlan_pm_callback: can't enter sleep mode\n");
                return WLAN_STATUS_FAILURE;
            } else {

                /*
                 * Detach the network interface
                 * if the network is running
                 */
                if (netif_running(dev)) {
                    netif_device_detach(dev);
                    PRINTM(INFO, "netif_device_detach().\n");
                }
                /* Stop SDIO bus clock */
                sbi_set_bus_clock(priv, FALSE);
                sbi_suspend(priv);
            }
            break;
        }

        /* in non associated mode */

        /*
         * Detach the network interface 
         * if the network is running
         */
        if (netif_running(dev))
            netif_device_detach(dev);

        /* 
         * Storing and restoring of the regs be taken care 
         * at the driver rest will be done at wlan driver
         * this makes driver independent of the card
         */
        if (Adapter->IsDeepSleep == FALSE) {
            SetDeepSleep(priv, TRUE);
            OS_Enable_DS = TRUE;
        }

        sbi_suspend(priv);

        break;

    case PM_RESUME:
        /* in associated mode */
        if (Adapter->MediaConnectStatus == WlanMediaStateConnected) {
            if (Adapter->bWakeupDevRequired == FALSE) {
                /* could never happen */
                PRINTM(MSG, "wlan_pm_callback: serious error.\n");
            } else {
                /*
                 * Bring the inteface up first
                 * This case should not happen still ...
                 */
                sbi_resume(priv);

                /*
                 * Start SDIO bus clock
                 */
                sbi_set_bus_clock(priv, TRUE);
                /*
                 * Attach the network interface
                 * if the network is running
                 */
                if (netif_running(dev)) {
                    netif_device_attach(dev);
                    PRINTM(INFO, "after netif_device_attach().\n");
                }
                PRINTM(INFO, "After netif attach, in associated mode.\n");
            }
            break;
        }

        /* in non associated mode */

#ifdef WPRM_DRV
        /* Background scan support */
        WPRM_DRV_TRACING_PRINT();
        /* check if WLAN_HOST_WAKEB triggered, turn on SDIO_CLK */
        if (wprm_wlan_host_wakeb_is_triggered()) {      /* WLAN_HSOT_WAKEB is triggered */
            if (sbi_set_bus_clock(priv, TRUE)) {
                PRINTM(MSG,
                       "wlan_pm_callback: in PM_RESUME, wlan sdio clock turn on fail\n");
            }
            WPRM_DRV_TRACING_PRINT();
        }
#endif
        /*
         * Bring the inteface up first 
         * This case should not happen still ...
         */

        if (OS_Enable_DS == TRUE) {
#ifdef WPRM_DRV
            /* if need to wakeup FW, then trigger HOST_WLAN_WAKEB first */
            wprm_trigger_host_wlan_wakeb(1);
#endif
        }

        sbi_resume(priv);

        if (OS_Enable_DS == TRUE) {
            SetDeepSleep(priv, FALSE);
            OS_Enable_DS = FALSE;
        }

        if (netif_running(dev))
            netif_device_attach(dev);

        PRINTM(INFO, "after netif attach, in NON associated mode.\n");
        break;
    }

    return WLAN_STATUS_SUCCESS;
}
#endif /* ENABLE_PM */

/** 
 *  @brief This function handles packet transmission
 *  
 *  @param skb     A pointer to sk_buff structure
 *  @param dev     A pointer to net_device structure
 *  @return 	   WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
wlan_hard_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
    int ret;
    wlan_private *priv = dev->priv;

    ENTER();

    PRINTM(DATA, "Data <= kernel\n");

    if (wlan_tx_packet(priv, skb)) {
        /* Transmit failed */
        ret = WLAN_STATUS_FAILURE;
        goto done;
    } else {
        /* Transmit succeeded */
        if (!priv->adapter->wmm.enabled) {
            if (priv->adapter->TxSkbNum >= MAX_NUM_IN_TX_Q) {
                UpdateTransStart(dev);
                os_stop_queue(priv);
            }
        }
    }

    ret = WLAN_STATUS_SUCCESS;
  done:

    LEAVE();
    return ret;
}

/** 
 *  @brief This function handles the timeout of packet
 *  transmission
 *  
 *  @param dev     A pointer to net_device structure
 *  @return 	   n/a
 */
static void
wlan_tx_timeout(struct net_device *dev)
{
    wlan_private *priv = (wlan_private *) dev->priv;

    ENTER();

    PRINTM(DATA, "Tx timeout\n");
    umd_dbg( "Tx timeout!!!!");

    UpdateTransStart(dev);

    priv->adapter->dbg.num_tx_timeout++;

    priv->adapter->IntCounter++;
    wake_up_interruptible(&priv->MainThread.waitQ);

    LEAVE();
}

/** 
 *  @brief This function returns the network statistics
 *  
 *  @param dev     A pointer to wlan_private structure
 *  @return 	   A pointer to net_device_stats structure
 */
static struct net_device_stats *
wlan_get_stats(struct net_device *dev)
{
    wlan_private *priv = (wlan_private *) dev->priv;

    return &priv->stats;
}

/** 
 *  @brief This function sets the MAC address to firmware.
 *  
 *  @param priv    A pointer to wlan_private structure
 *  @param pRxPD   A pointer to RxPD structure of received packet
 *  @return 	   WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
wlan_set_mac_address(struct net_device *dev, void *addr)
{
    int ret = WLAN_STATUS_SUCCESS;
    wlan_private *priv = (wlan_private *) dev->priv;
    wlan_adapter *Adapter = priv->adapter;
    struct sockaddr *pHwAddr = (struct sockaddr *) addr;

    ENTER();

    memset(Adapter->CurrentAddr, 0, MRVDRV_ETH_ADDR_LEN);

    /* dev->dev_addr is 8 bytes */
    HEXDUMP("dev->dev_addr:", dev->dev_addr, ETH_ALEN);

    HEXDUMP("addr:", pHwAddr->sa_data, ETH_ALEN);
    memcpy(Adapter->CurrentAddr, pHwAddr->sa_data, ETH_ALEN);

    ret = PrepareAndSendCommand(priv, HostCmd_CMD_802_11_MAC_ADDRESS,
                                HostCmd_ACT_SET,
                                HostCmd_OPTION_WAITFORRSP, 0, NULL);

    if (ret) {
        PRINTM(INFO, "set mac address failed.\n");
        ret = WLAN_STATUS_FAILURE;
        goto done;
    }

    HEXDUMP("Adapter->MacAddr:", Adapter->CurrentAddr, ETH_ALEN);
    memcpy(dev->dev_addr, Adapter->CurrentAddr, ETH_ALEN);

  done:
    LEAVE();
    return ret;
}

/** 
 *  @brief This function sets multicast addresses to firmware
 *  
 *  @param dev     A pointer to net_device structure
 *  @return 	   n/a
 */
static void
wlan_set_multicast_list(struct net_device *dev)
{
    wlan_private *priv = dev->priv;
    wlan_adapter *Adapter = priv->adapter;
    int OldPacketFilter;

    ENTER();

    OldPacketFilter = Adapter->CurrentPacketFilter;

    if (dev->flags & IFF_PROMISC) {
        PRINTM(INFO, "Enable Promiscuous mode\n");
        Adapter->CurrentPacketFilter |= HostCmd_ACT_MAC_PROMISCUOUS_ENABLE;
        Adapter->CurrentPacketFilter &=
            ~(HostCmd_ACT_MAC_ALL_MULTICAST_ENABLE |
              HostCmd_ACT_MAC_MULTICAST_ENABLE);
    } else {
        /* Multicast */
        Adapter->CurrentPacketFilter &= ~HostCmd_ACT_MAC_PROMISCUOUS_ENABLE;

        if (dev->flags & IFF_ALLMULTI || dev->mc_count >
            MRVDRV_MAX_MULTICAST_LIST_SIZE) {
            PRINTM(INFO, "Enabling All Multicast!\n");
            Adapter->CurrentPacketFilter |=
                HostCmd_ACT_MAC_ALL_MULTICAST_ENABLE;
            Adapter->CurrentPacketFilter &= ~HostCmd_ACT_MAC_MULTICAST_ENABLE;
        } else {
            Adapter->CurrentPacketFilter &=
                ~HostCmd_ACT_MAC_ALL_MULTICAST_ENABLE;

            if (!dev->mc_count) {
                PRINTM(INFO, "No multicast addresses - "
                       "disabling multicast!\n");
                Adapter->CurrentPacketFilter &=
                    ~HostCmd_ACT_MAC_MULTICAST_ENABLE;
            } else {
                int i;

                Adapter->CurrentPacketFilter |=
                    HostCmd_ACT_MAC_MULTICAST_ENABLE;

                Adapter->NumOfMulticastMACAddr =
                    CopyMulticastAddrs(Adapter, dev);

                PRINTM(INFO, "Multicast addresses: %d\n", dev->mc_count);

                for (i = 0; i < dev->mc_count; i++) {
                    PRINTM(INFO, "Multicast address %d:"
                           "%x %x %x %x %x %x\n", i,
                           Adapter->MulticastList[i][0],
                           Adapter->MulticastList[i][1],
                           Adapter->MulticastList[i][2],
                           Adapter->MulticastList[i][3],
                           Adapter->MulticastList[i][4],
                           Adapter->MulticastList[i][5]);
                }
                /* set multicast addresses to firmware */
                PrepareAndSendCommand(priv, HostCmd_CMD_MAC_MULTICAST_ADR,
                                      HostCmd_ACT_GEN_SET, 0, 0, NULL);
            }
        }
    }

    if (Adapter->CurrentPacketFilter != OldPacketFilter) {
        SetMacPacketFilter(priv);
    }

    LEAVE();
}

/** 
 *  @brief This function pops rx_skb from the rx queue.
 *  
 *  @param RxSkbQ  A pointer to rx_skb queue
 *  @return 	   A pointer to skb
 */
static struct sk_buff *
wlan_pop_rx_skb(struct sk_buff *RxSkbQ)
{
    struct sk_buff *skb_data = NULL;

    if (!list_empty((struct list_head *) RxSkbQ)) {
        skb_data = RxSkbQ->next;
        list_del((struct list_head *) RxSkbQ->next);
    }

    return skb_data;
}

/** 
 *  @brief This function hanldes the major job in WLAN driver.
 *  it handles the event generated by firmware, rx data received
 *  from firmware and tx data sent from kernel.
 *  
 *  @param data    A pointer to wlan_thread structure
 *  @return 	   WLAN_STATUS_SUCCESS
 */
static int wlan_service_main_thread(void *data)
{
    wlan_thread *thread = data;
    wlan_private *priv = thread->priv;
    wlan_adapter *Adapter = priv->adapter;
    wait_queue_t wait;
    u8 ireg = 0;
    int ret;

    OS_INTERRUPT_SAVE_AREA;

    ENTER();

    wlan_activate_thread(thread);

    init_waitqueue_entry(&wait, current);

    for (;;) {
        add_wait_queue(&thread->waitQ, &wait);
        OS_SET_THREAD_STATE(TASK_INTERRUPTIBLE);

        TX_DISABLE;

        if ((Adapter->WakeupTries) ||
            (Adapter->PSState == PS_STATE_SLEEP
             && !Adapter->bWakeupDevRequired) ||
            (!Adapter->IntCounter
             && (priv->wlan_dev.dnld_sent || !Adapter->wmm.enabled ||
                 Adapter->TxLockFlag || !os_queue_is_active(priv) ||
                 wmm_lists_empty(priv))
             && (priv->wlan_dev.dnld_sent || Adapter->TxLockFlag ||
                 !Adapter->TxSkbNum)
             && (priv->wlan_dev.dnld_sent || Adapter->CurCmd ||
                 list_empty(&Adapter->CmdPendingQ))
            )
            ) {
            umd_dbg("main-thread sleeping... "
                   "WakeupReq=%s Conn=%s PS_Mode=%d PS_State=%d\n",
                   (Adapter->bWakeupDevRequired) ? "Y" : "N",
                   (Adapter->MediaConnectStatus) ? "Y" : "N",
                   Adapter->PSMode, Adapter->PSState);

            TX_RESTORE;

		/* add by feng for thread debug */
	    threadrunning = 0;

            schedule();

		/* add by feng for thread debug */
	    threadrunning = 1;

	    umd_dbg("");
            umd_dbg("main-thread waking up: IntCnt=%d "
                   "CurCmd=%s CmdPending=%s"
                   " Connect=%s CurTxSkb=%s dnld_sent=%d",
                   Adapter->IntCounter,
                   (Adapter->CurCmd) ? "Y" : "N",
                   list_empty(&Adapter->CmdPendingQ) ? "N" : "Y",
                   (Adapter->MediaConnectStatus) ? "Y" : "N",
                   (Adapter->CurrentTxSkb) ? "Y" : "N",
                   priv->wlan_dev.dnld_sent);
        } else {
            TX_RESTORE;
        }

        OS_SET_THREAD_STATE(TASK_RUNNING);
        remove_wait_queue(&thread->waitQ, &wait);

        if (kthread_should_stop()
            || Adapter->SurpriseRemoved) {
           umd_dbg("main-thread: break from main thread: SurpriseRemoved=0x%x",
                   Adapter->SurpriseRemoved);
            break;
        }

	/* if some ISR wakes card up, need read card's INT reg to
	   get the detail information */
        if (Adapter->IntCounter) {
            OS_INT_DISABLE;
            Adapter->IntCounter = 0;
            OS_INT_RESTORE;

            if (sbi_get_int_status(priv, &ireg)) {
                PRINTM(ERROR,
                       "main-thread: reading HOST_INT_STATUS_REG failed\n");
                continue;
            }
            OS_INT_DISABLE;
            Adapter->HisRegCpy |= ireg;
            OS_INT_RESTORE;
            umd_dbg("INT: status = 0x%x", Adapter->HisRegCpy);
        } else if (Adapter->bWakeupDevRequired
                   && (Adapter->PSState == PS_STATE_SLEEP)
            ) {
            Adapter->WakeupTries++;
            PRINTM(CMND,
                   "Wakeup device... WakeupReq=%s Conn=%s PS_Mode=%d PS_State=%d\n",
                   (Adapter->bWakeupDevRequired) ? "Y" : "N",
                   (priv->adapter->MediaConnectStatus) ? "Y" : "N",
                   priv->adapter->PSMode, priv->adapter->PSState);

            /* Wake up device */
            if (sbi_exit_deep_sleep(priv))
                PRINTM(MSG, "main-thread: wakeup dev failed\n");
            continue;
        }

        /* Command response? */
        if (Adapter->HisRegCpy & HIS_CmdUpLdRdy) {
            OS_INT_DISABLE;
            Adapter->HisRegCpy &= ~HIS_CmdUpLdRdy;
            OS_INT_RESTORE;

            ret = wlan_process_rx_command(priv);
		umd_dbg("ret for wlan_process_rx_command() is %d !!!", ret);
        }

        /* Any received data? */
        if (Adapter->HisRegCpy & HIS_RxUpLdRdy) {
            OS_INT_DISABLE;
            Adapter->HisRegCpy &= ~HIS_RxUpLdRdy;
            OS_INT_RESTORE;
            wlan_send_rxskbQ(priv);
        }

        /* Any Card Event */
        if (Adapter->HisRegCpy & HIS_CardEvent) {
            OS_INT_DISABLE;
            Adapter->HisRegCpy &= ~HIS_CardEvent;
            OS_INT_RESTORE;

            if (sbi_read_event_cause(priv)) {
                PRINTM(ERROR, "main-thread: sbi_read_event_cause failed.\n");
                umd_dbg( "main-thread: sbi_read_event_cause failed.\n");
                continue;
            }
            ret = wlan_process_event(priv);
		umd_dbg("ret for wlan_process_event() is %d !!!", ret);
        }

        /* Check if we need to confirm Sleep Request received previously */
        if (Adapter->PSState == PS_STATE_PRE_SLEEP) {
            if (!priv->wlan_dev.dnld_sent && !Adapter->CurCmd) {
                ASSERT(Adapter->MediaConnectStatus ==
                       WlanMediaStateConnected);
                PSConfirmSleep(priv, (u16) Adapter->PSMode);
            }
        }

        /* The PS state is changed during processing of Sleep Request event above */
        if ((priv->adapter->PSState == PS_STATE_SLEEP)
            || (priv->adapter->PSState == PS_STATE_PRE_SLEEP)
            ) {
            continue;
        }

        /* Execute the next command */
        if (!priv->wlan_dev.dnld_sent && !Adapter->CurCmd) {
            ExecuteNextCommand(priv);
        }

        if (Adapter->wmm.enabled) {
            if (!wmm_lists_empty(priv) && os_queue_is_active(priv)) {
                if ((Adapter->PSState == PS_STATE_FULL_POWER) ||
                    (Adapter->sleep_period.period == 0)
                    || (Adapter->TxLockFlag == FALSE))
                    wmm_process_tx(priv);
            }
        } else {
            if (!priv->wlan_dev.dnld_sent && (Adapter->TxLockFlag == FALSE)
                && !list_empty((struct list_head *) &priv->adapter->TxSkbQ)) {
                wlan_process_txqueue(priv);
            }
        }
    }

    wlan_deactivate_thread(thread);

    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/**
 * @brief This function adds the card. it will probe the
 * card, allocate the wlan_priv and initialize the device. 
 *  
 *  @param card    A pointer to card
 *  @return 	   A pointer to wlan_private structure
 */
static wlan_private *
wlan_add_card(void *card)
{
    struct net_device *dev = NULL;
    wlan_private *priv = NULL;

	umd_dbg("enter");

    /* probe the card */
    if (sbi_probe_card(card) < 0) {
        PRINTM(MSG, "NO card found!\n");
        return NULL;
    }

    /* Allocate an Ethernet device and register it */
   if (!(dev = alloc_netdev(sizeof(wlan_private), "wlan%d", ether_setup))){
        PRINTM(MSG, "Init ethernet device failed!\n");
        return NULL;
    }

    priv = dev->priv;

    /* allocate buffer for wlan_adapter */
    if (!(priv->adapter = kmalloc(sizeof(wlan_adapter), GFP_KERNEL))) {
        PRINTM(MSG, "Allocate buffer for wlan_adapter failed!\n");
        goto err_kmalloc;
    }

    /* init wlan_adapter */
    memset(priv->adapter, 0, sizeof(wlan_adapter));

    priv->wlan_dev.netdev = dev;
    priv->wlan_dev.card = card;
    wlanpriv = priv;

    SET_MODULE_OWNER(dev);

    /* Setup the OS Interface to our functions */
    dev->open = wlan_open;
    dev->hard_start_xmit = wlan_hard_start_xmit;
    dev->stop = wlan_close;
    dev->do_ioctl = wlan_do_ioctl;
    dev->set_mac_address = wlan_set_mac_address;

#define	WLAN_WATCHDOG_TIMEOUT	(2 * HZ)

    dev->tx_timeout = wlan_tx_timeout;
    dev->get_stats = wlan_get_stats;
    dev->watchdog_timeo = WLAN_WATCHDOG_TIMEOUT;

#ifdef	WIRELESS_EXT
    //hufh dev->get_wireless_stats = wlan_get_wireless_stats;
    dev->wireless_handlers = (struct iw_handler_def *) &wlan_handler_def;
#endif
#define NETIF_F_DYNALLOC 16
    dev->features |= NETIF_F_DYNALLOC;
    dev->flags |= IFF_BROADCAST | IFF_MULTICAST;
    dev->set_multicast_list = wlan_set_multicast_list;

#ifdef MFG_CMD_SUPPORT
    /* Required for the mfg command */
    init_waitqueue_head(&priv->adapter->mfg_cmd_q);
#endif
    init_waitqueue_head(&priv->adapter->ds_awake_q);

#ifdef ENABLE_PM
    if (!(wlan_pm_dev = pm_register(PM_UNKNOWN_DEV, 0, wlan_pm_callback)))
        PRINTM(MSG, "Failed to register PM callback\n");
#endif

    INIT_LIST_HEAD(&priv->adapter->CmdFreeQ);
    INIT_LIST_HEAD(&priv->adapter->CmdPendingQ);

    umd_dbg("Starting kthread...");
    priv->MainThread.priv = priv;
    wlan_create_thread(wlan_service_main_thread,
                       &priv->MainThread, "wlan_main_service");

    ConfigureThreadPriority();

#ifdef REASSOCIATION
    priv->ReassocThread.priv = priv;
    wlan_create_thread(wlan_reassociation_thread,
                       &priv->ReassocThread, "wlan_reassoc_service");
#endif /* REASSOCIATION */

    /*
     * Register the device. Fillup the private data structure with
     * relevant information from the card and request for the required
     * IRQ. 
     */
    if (sbi_register_dev(priv) < 0) {
        PRINTM(FATAL, "Failed to register wlan device!\n");
        goto err_registerdev;
    }

    /* Build MMC bus with Device connection for Intel FB */
    SET_NETDEV_DEV(dev, priv->hotplug_device);

    PRINTM(WARN, "%s: Marvell Wlan 802.11 Adapter "
           "revision 0x%02X at IRQ %i\n", dev->name,
           priv->adapter->chip_rev, dev->irq);

    wlan_proc_entry(priv, dev);
#ifdef PROC_DEBUG
    wlan_debug_entry(priv, dev);
#endif

    /* Get the CIS Table */
    sbi_get_cis_info(priv);

	umd_dbg("succeed in get the cis info");

    /* init FW and HW */
    if (wlan_init_fw(priv)) {
        PRINTM(FATAL, "Firmware Init Failed\n");
        goto err_init_fw;
    }
	umd_dbg("succeed in init HW");

    if (register_netdev(dev)) {
        printk(KERN_ERR "Cannot register network device!\n");
	// add by Feng
	//unregister_netdev(dev);
        goto err_init_fw;
    }
	umd_dbg("exit! priv = 0x%08x", priv);

    return priv;

  err_init_fw:
    sbi_unregister_dev(priv);

#ifdef PROC_DEBUG
    wlan_debug_remove(priv);
#endif
    wlan_proc_remove(priv);

  err_registerdev:
    /* Stop the thread servicing the interrupts */
    wake_up_interruptible(&priv->MainThread.waitQ);
    wlan_terminate_thread(&priv->MainThread);

#ifdef REASSOCIATION
    wake_up_interruptible(&priv->ReassocThread.waitQ);
    wlan_terminate_thread(&priv->ReassocThread);
#endif /* REASSOCIATION */

  err_kmalloc:
	// modified by Feng
	free_netdev(dev);
    wlan_free_adapter(priv);
    wlanpriv = NULL;

    LEAVE();
    return NULL;
}

/** 
 *  @brief This function removes the sdio function.
 *  
 *  @param func    A pointer to sdio_function 
 *  @return 	   WLAN_STATUS_SUCCESS
 */
static int
wlan_remove_card(void *func)
{
    wlan_private *priv = wlanpriv;
    wlan_adapter *Adapter;
    struct net_device *dev;
    union iwreq_data wrqu;

    if (!priv) {
        LEAVE();
        return WLAN_STATUS_SUCCESS;
    }

    Adapter = priv->adapter;

    if (!Adapter) {
        LEAVE();
        return WLAN_STATUS_SUCCESS;
    }

    dev = priv->wlan_dev.netdev;

    wake_up_interruptible(&Adapter->ds_awake_q);

    if (Adapter->CurCmd) {
        PRINTM(INFO, "Wake up current cmdwait_q\n");
        wake_up_interruptible(&Adapter->CurCmd->cmdwait_q);
    }

    Adapter->CurCmd = NULL;

    if (Adapter->MediaConnectStatus == WlanMediaStateConnected) {
        wlan_clean_txrx(priv);
        Adapter->MediaConnectStatus = WlanMediaStateDisconnected;
    }
    if (Adapter->PSMode == Wlan802_11PowerModeMAX_PSP) {
        Adapter->PSMode = Wlan802_11PowerModeCAM;
        PSWakeup(priv, HostCmd_OPTION_WAITFORRSP);
    }
    if (Adapter->IsDeepSleep == TRUE) {
        Adapter->IsDeepSleep = FALSE;
        sbi_exit_deep_sleep(priv);
    }

    memset(wrqu.ap_addr.sa_data, 0xaa, ETH_ALEN);
    wrqu.ap_addr.sa_family = ARPHRD_ETHER;
    wireless_send_event(priv->wlan_dev.netdev, SIOCGIWAP, &wrqu, NULL);

    /* Disable interrupts on the card as we cannot handle them after RESET */
    sbi_disable_host_int(priv);

    PrepareAndSendCommand(priv, HostCmd_CMD_802_11_RESET, 0, 0, 0, NULL);

    os_sched_timeout(200);

#ifdef ENABLE_PM
    pm_unregister(wlan_pm_dev);
#endif

    /* Flush all the packets upto the OS before stopping */
    wlan_send_rxskbQ(priv);
    cleanup_txqueues(priv);
    os_stop_queue(priv);
    os_carrier_off(priv);

    Adapter->SurpriseRemoved = TRUE;

    /* Stop the thread servicing the interrupts */
    wake_up_interruptible(&priv->MainThread.waitQ);
    //wlan_terminate_thread(&priv->MainThread);

#ifdef REASSOCIATION
    wake_up_interruptible(&priv->ReassocThread.waitQ);
    //wlan_terminate_thread(&priv->ReassocThread);
#endif /* REASSOCIATION */

#ifdef PROC_DEBUG
    wlan_debug_remove(priv);
#endif
    wlan_proc_remove(priv);

    PRINTM(INFO, "unregester dev\n");
    sbi_unregister_dev(priv);

    PRINTM(INFO, "Free Adapter\n");
    wlan_free_adapter(priv);

    /* Last reference is our one */
    PRINTM(INFO, "refcnt = %d\n", atomic_read(&dev->refcnt));

    PRINTM(INFO, "netdev_finish_unregister: %s%s.\n", dev->name,
           (dev->features & NETIF_F_DYNALLOC) ? "" : ", old style");

    unregister_netdev(dev);

    PRINTM(INFO, "Unregister finish\n");

    priv->wlan_dev.netdev = NULL;
    free_netdev(dev);
    wlanpriv = NULL;

    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/********************************************************
		Global Functions
********************************************************/
/** 
 *  @brief Cleanup TX queue
 *  @param priv       pointer to wlan_private
 *  @return 	      N/A
*/
void
cleanup_txqueues(wlan_private * priv)
{
    struct sk_buff *delNode, *Q;

    Q = &priv->adapter->TxSkbQ;
    while (!list_empty((struct list_head *) Q)) {
        delNode = Q->next;
        list_del((struct list_head *) delNode);
        kfree_skb(delNode);
    }
    priv->adapter->TxSkbNum = 0;
}

/** 
 *  @brief handle TX Queue
 *  @param priv       pointer to wlan_private
 *  @return 	      N/A
*/
void
wlan_process_txqueue(wlan_private * priv)
{
    wlan_adapter *Adapter = priv->adapter;
    ulong flags;
    struct sk_buff *Q;
    OS_INTERRUPT_SAVE_AREA;
    ENTER();
    spin_lock_irqsave(&Adapter->CurrentTxLock, flags);
    if (Adapter->TxSkbNum > 0) {
        Q = &priv->adapter->TxSkbQ;
        Adapter->CurrentTxSkb = Q->next;
        list_del((struct list_head *) Adapter->CurrentTxSkb);
        Adapter->TxSkbNum--;
    }
    spin_unlock_irqrestore(&Adapter->CurrentTxLock, flags);
    if (Adapter->CurrentTxSkb) {
        wlan_process_tx(priv);
    }

    LEAVE();
}

/**
 * @brief This function sends the rx packets to the os from the skb queue
 *
 * @param priv	A pointer to wlan_private structure
 * @return	n/a
 */
void
wlan_send_rxskbQ(wlan_private * priv)
{
    struct sk_buff *skb;

    ENTER();
    if (priv->adapter) {
        while ((skb = wlan_pop_rx_skb(&priv->adapter->RxSkbQ))) {
            if (ProcessRxedPacket(priv, skb) == -ENOMEM)
                break;
        }
    }
    LEAVE();
}

/** 
 *  @brief This function finds the CFP in 
 *  region_cfp_table based on region and band parameter.
 *  
 *  @param region  The region code
 *  @param band	   The band
 *  @param cfp_no  A pointer to CFP number
 *  @return 	   A pointer to CFP
 */
CHANNEL_FREQ_POWER *
wlan_get_region_cfp_table(u8 region, u8 band, int *cfp_no)
{
    int i;

    ENTER();

    for (i = 0; i < sizeof(region_cfp_table) / sizeof(region_cfp_table_t);
         i++) {
        PRINTM(INFO, "region_cfp_table[i].region=%d\n",
               region_cfp_table[i].region);
        if (region_cfp_table[i].region == region) {
            {
                *cfp_no = region_cfp_table[i].cfp_no_BG;
                LEAVE();
                return region_cfp_table[i].cfp_BG;
            }
        }
    }

    LEAVE();
    return NULL;
}

/** 
 *  @brief This function sets region table. 
 *  
 *  @param priv    A pointer to wlan_private structure
 *  @param region  The region code
 *  @param band	   The band
 *  @return 	   WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
int
wlan_set_regiontable(wlan_private * priv, u8 region, u8 band)
{
    wlan_adapter *Adapter = priv->adapter;
    int i = 0;

    CHANNEL_FREQ_POWER *cfp;
    int cfp_no;

    ENTER();

    memset(Adapter->region_channel, 0, sizeof(Adapter->region_channel));

    {
        cfp = wlan_get_region_cfp_table(region, band, &cfp_no);
        if (cfp != NULL) {
            Adapter->region_channel[i].NrCFP = cfp_no;
            Adapter->region_channel[i].CFP = cfp;
        } else {
            PRINTM(INFO, "wrong region code %#x in Band B-G\n", region);
            return WLAN_STATUS_FAILURE;
        }
        Adapter->region_channel[i].Valid = TRUE;
        Adapter->region_channel[i].Region = region;
        Adapter->region_channel[i].Band = band;
        i++;
    }
    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/** 
 *  @brief This function handles the interrupt. it will change PS
 *  state if applicable. it will wake up main_thread to handle
 *  the interrupt event as well.
 *  
#ifdef PXA3XX
 *  @param priv    A pointer to wlan_private structure
#else // !PXA3XX
 *  @param dev     A pointer to net_device structure
#endif // PXA3XX
 *  @return 	   n/a
 */
void
wlan_interrupt(struct net_device *dev)
{
    wlan_private *priv = dev->priv;

    //umd_dbg("enter");

    priv->adapter->IntCounter++;

    priv->adapter->WakeupTries = 0;

    if (priv->adapter->PSState == PS_STATE_SLEEP) {
        priv->adapter->PSState = PS_STATE_AWAKE;
    }
    wake_up_interruptible(&priv->MainThread.waitQ);

    //umd_dbg("exit");
}

/** 
 *  @brief This function initializes module.
 *  
 *  @param	   n/a    A pointer to wlan_private structure
 *  @return 	   WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
int
wlan_init_module(void)
{
    int ret = WLAN_STATUS_SUCCESS;

    ENTER();

    if (sbi_register(wlan_add_card, wlan_remove_card, NULL) == NULL) {
        ret = WLAN_STATUS_FAILURE;
        goto done;
    }

  done:
    LEAVE();
    return ret;
}

/** 
 *  @brief This function cleans module
 *  
 *  @param priv    n/a
 *  @return 	   n/a
 */
void
wlan_cleanup_module(void)
{
    ENTER();

    sbi_unregister();

    LEAVE();
}

module_init(wlan_init_module);
module_exit(wlan_cleanup_module);

MODULE_DESCRIPTION("M-WLAN Driver");
MODULE_AUTHOR("Marvell International Ltd.");
MODULE_LICENSE("GPL");
