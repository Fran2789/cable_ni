/*
 *
 * cni.c
 * Description:
 * DOCSIS datapath driver implementation
 *
 * Copyright (C) 2008 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#define DRV_NAME    "Cable Modem Network Interface driver"
#define DRV_VERSION    "0.0.1"


#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/mii.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <linux/proc_fs.h>

#include "pal.h"
#include "pal_cppi41.h"
#include <linux/kernel.h>

/*
 * Related session router definitions
 * ----------------------------------
 */
//#define PP_MERGE_HOST_DESC    /* direct to accumulator, not thru session router */
#define NOT_ENOUGH_REGIONS      /* use same q to all channels, no region resources !!!*/

#ifdef CONFIG_ARM_AVALANCHE_PPD
static int cni_pp_prepare_pid (struct net_device *dev);
static int cni_pp_set_pid_flags (struct net_device *dev, int flags);
#define PP_SUPPORT_SYNC_Q
#endif

/*
 * old defines from old puma5_cppi.h
 */
/* Cable network interface driver */
#define CNID_BD_SIZE                    64
#define CNID_NUM_TX_BD                  256

extern struct net_device * wanDev;
extern struct net_device * mtaDev;

#define CNID_RXINT_NUM        (AVALANCHE_INTD_BASE_INT + CNI_ACC_RX_INTV) 
#define CNID_TXINT_NUM        (AVALANCHE_INTD_BASE_INT + CNI_ACC_TXCMPL_INTV)
#define CNID_INTD_HOST_NUM    0
#define CPPI4_BD_LENGTH_FOR_CACHE   64

#define CIND_TX_SERVICE_MAX         64
#define CIND_RX_SERVICE_MAX         64

#define CNID_ACC_PAGE_NUM_ENTRY     (32)
#define CNID_ACC_NUM_PAGE           (2)
#define CNID_ACC_ENTRY_TYPE         (PAL_CPPI41_ACC_ENTRY_TYPE_D)
/* Byte size of page = number of entries per page * size of each entry */
#define CNID_ACC_PAGE_BYTE_SZ       (CNID_ACC_PAGE_NUM_ENTRY * ((CNID_ACC_ENTRY_TYPE + 1) * sizeof(unsigned int*)))
/* byte size of list = byte size of page * number of pages * */
#define CNID_ACC_LIST_BYTE_SZ       (CNID_ACC_PAGE_BYTE_SZ * CNID_ACC_NUM_PAGE)


#define GET_BD_PTR(base, num)       ((base) + ((num) * CNID_BD_SIZE))
#define CNID_QM_DESC_SIZE_CODE      4 /* ((sizeof(Cppi4HostDesc) - 24)/4) */
#define CNID_QM_EMB_DESC_SIZE_CODE  7 /* ((sizeof(Cppi4EmbDesc) - 24)/4) */


/* define to enable copious debugging info */
/* #define CNID_DEBUG */

#ifdef CNID_DEBUG
/* note: prints function name for you */
#  define DPRINTK(fmt, args...) printk(KERN_DEBUG "%s: " fmt, __FUNCTION__ , ## args)
#else
#  define DPRINTK(fmt, args...)
#endif

#ifdef CNID_ASSERTS
#  define assert(expr) do {} while (0)
#else
#  define assert(expr) \
        if(unlikely(!(expr))) {                        \
        DPRINTK(KERN_ERR "Assertion failed! %s,%s,%s,line=%d\n",    \
        #expr,__FILE__,__FUNCTION__,__LINE__);                \
        }
#endif

/* max supported frame size */
#define MAX_DOCSIS_FRAME_SIZE    1600
#define RX_BUF_SIZE    MAX_DOCSIS_FRAME_SIZE

MODULE_AUTHOR ("Texas Instruments, Inc");
MODULE_DESCRIPTION (DRV_NAME);
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);


/************************************************************************/
/*     CNI HOST Descriptor type definition                              */
/************************************************************************/
struct cnid_desc {

    Cppi4HostDesc       hw;         /* The Hardware Descriptor */
    int                 psi[3];     /* protocol specific information for fw  */
    struct cnid_desc*   next;       /* The next BD in chain */
    struct sk_buff*     skb;        /* The data pointer virtual address */
};
/************************************************************************/


/************************************************************************/
/*     CNI driver private data                                          */
/************************************************************************/
struct cnid_private {

    PAL_Handle              pal_hnd;        /* The handle to PAL layer */
    PAL_Cppi4AccChHnd       tx_acc_hnd;     /* The Tx accumulator channel handle */
    Ptr                     tx_list_base;
    PAL_Cppi4QueueHnd       tx_queue[CNI_CPPI4x_TX_Q_COUNT];    
    PAL_Cppi4QueueHnd       tx_qos_queue;    
    struct cnid_desc*       tx_bdlist;      /* The Tx BD software queue. May be a h/w queue can be used instead */
    struct cnid_desc**      tx_list;            
    spinlock_t              txlock;    
    struct tasklet_struct   tx_tasklet;     /* Tx completion processing tasklet */    
    Ptr                     tx_bdpool;
    
    /* 
     * CNI Rx Accumulator info 
     */
    PAL_Cppi4AccChHnd       rxAcc_chan_hnd      [CNI_ACC_RX_CH_COUNT];  /* The Rx accumulator channel handle */
    unsigned int            rxAcc_chan          [CNI_ACC_RX_CH_COUNT];  /* The Rx accumulator channel numbers */
    struct cnid_desc**      rxAcc_chan_list     [CNI_ACC_RX_CH_COUNT];  /* Rx acc channel lists */
    Ptr                     rxAcc_chan_list_base[CNI_ACC_RX_CH_COUNT];  /* Rx acc channel lists base*/
    PAL_Cppi4QueueHnd       rxAcc_queue_hnd     [CNI_ACC_RX_CH_COUNT];  /* Rx acc queue handles */

    /* 
     * CNI Proxy channels info 
     */
    PAL_Cppi4QueueHnd       rxProxy_free_queue_hnd[CNI_CPPI4x_DOC2HOST_PROXY_CH_COUNT]; /* Rx Proxy free queue handles */
    PAL_Cppi4QueueHnd       rxProxy_queue_hnd     [CNI_CPPI4x_DOC2HOST_PROXY_CH_COUNT]; /* Rx Proxy queue handles */
    Ptr                     rxProxy_bdpool        [CNI_CPPI4x_DOC2HOST_PROXY_CH_COUNT]; /* Rx Proxy bd pools */    
    PAL_Cppi4TxChHnd        rxProxy_chan_hnd[4];    /* Rx Proxy channel handles */
    PAL_Cppi4TxChHnd        txProxy_chan_hnd[4];    /* Tx Proxy channel handles */

    PAL_Cppi4AccChHnd       rxCni_chan_hnd      [CNI_CPPI4x_RX_DMA_CH_COUNT];   /* The Rx Cni channel handle */
    PAL_Cppi4QueueHnd       rxCni_free_queue_hnd[CNI_CPPI4x_RX_DMA_CH_COUNT];   /* Rx Cni free queue handles */
    Ptr                     rxCni_bdpool        [CNI_CPPI4x_RX_DMA_CH_COUNT];   /* Rx Cni bd pools */

#ifdef INIT_EXT_DMA
    PAL_Cppi4RxChHnd        cppi_low_rx_chan_hdl[4];
    PAL_Cppi4RxChHnd        cppi_high_rx_chan_hdl;
#endif
    struct net_device_stats stats;
    spinlock_t              devlock;    
    unsigned long           state;
    
};
/************************************************************************/



/************************************************************************/
/*                        proxy channels information                    */
/************************************************************************/
static  Cppi4Queue rxProxy_queue[CNI_CPPI4x_DOC2HOST_PROXY_CH_COUNT]=
            {
                {CNI_CPPI4x_RX_QMGR, CNI_CPPI4x_RX_QNUM(0)},
                {CNI_CPPI4x_RX_QMGR, CNI_CPPI4x_RX_QNUM(1)},
                {CNI_CPPI4x_RX_QMGR, CNI_CPPI4x_RX_QNUM(2)},
            };

static  Cppi4Queue rxProxy_free_queue[CNI_CPPI4x_DOC2HOST_PROXY_CH_COUNT]=
            {
                {CNI_CPPI4x_FBD_QMGR, CNI_CPPI4x_FBD_QNUM(0)},
                {CNI_CPPI4x_FBD_QMGR, CNI_CPPI4x_FBD_QNUM(0)}, //cable_pp: should be Q per channel
                {CNI_CPPI4x_FBD_QMGR, CNI_CPPI4x_FBD_QNUM(0)},
            };

static  int rxProxy_numbd[CNI_CPPI4x_DOC2HOST_PROXY_CH_COUNT]= 
            {
                DMAC_CNI_RX_HOST_BD_NUM_LO,
                DMAC_CNI_TX_HOST_BD_NUM_HI,
                DMAC_CNI_TX_HOST_BD_NUM_ML
            };
/************************************************************************/



/************************************************************************/
/*  CNI channels information                                            */
/************************************************************************/
static  Cppi4Queue rxCni_queue[CNI_CPPI4x_RX_DMA_CH_COUNT]=
            {
                {CNI_CPPI4x_RX_QMGR, PPFW_CPPI4x_RX_INGRESS_QNUM(3)}, // ch09, Q56
                {CNI_CPPI4x_RX_QMGR, PPFW_CPPI4x_RX_INGRESS_QNUM(3)}, // ch10, Q56 (same prio)
                {CNI_CPPI4x_RX_QMGR, PPFW_CPPI4x_RX_INGRESS_QNUM(3)}, // ch11, Q56 (same prio)
                {CNI_CPPI4x_RX_QMGR, PPFW_CPPI4x_RX_INGRESS_QNUM(3)}, // ch12, Q56 (same prio)
                {CNI_CPPI4x_RX_QMGR, PPFW_CPPI4x_RX_INGRESS_QNUM(3)}, // cable_pp: need to be high priority Q // ch13, Q57 (same prio) 
            };
    
static  Cppi4Queue rxCni_free_queue[CNI_CPPI4x_RX_DMA_CH_COUNT]=
            {
                {CNI_CPPI4x_FD_QMGR, CNI_CPPI4x_FD_QNUM(0)},
                {CNI_CPPI4x_FD_QMGR, CNI_CPPI4x_FD_QNUM(0)},
                {CNI_CPPI4x_FD_QMGR, CNI_CPPI4x_FD_QNUM(0)},
                {CNI_CPPI4x_FD_QMGR, CNI_CPPI4x_FD_QNUM(0)},
                {CNI_CPPI4x_FD_QMGR, CNI_CPPI4x_FD_QNUM(0)}, // cable_pp: need to be free Q for each channel
            };

static  int rxCni_numbd[CNI_CPPI4x_RX_DMA_CH_COUNT]=
            {
                DMAC_CNI_RX_EMBEDDED_BD_NUM_LO,
                DMAC_CNI_RX_EMBEDDED_BD_NUM_LO, 
                DMAC_CNI_RX_EMBEDDED_BD_NUM_LO,
                DMAC_CNI_RX_EMBEDDED_BD_NUM_LO,
                DMAC_CNI_RX_EMBEDDED_BD_NUM_HI,
            };
/************************************************************************/



/************************************************************************/
/*                                                                      */
/* Translate acc channel index to rx proxy channel index                */
/* In order to use Proxy rx queues as input queueus to the Accumulator  */
/* And to return back after use the packet to free Proxy queue          */
/*                                                                      */
/************************************************************************/
int acc_to_proxy[CNI_ACC_RX_CH_COUNT] = {0, 1, 2};

/* Use this MAC address if none is supplied on the command line */
static unsigned char defmac[] = {0x00, 0x50, 0xF1, 0x80, 0x00, 0x00};
/* Allocation for MAC string supplied on the command line */
static char *inpmac = NULL;

static int __devinit    cnid_probe              (struct device *        dev);
static int __devexit    cnid_remove             (struct device *        dev);
static int              cnid_open               (struct net_device *    dev);
static void             cnid_tx_timeout         (struct net_device *    dev);
static int              cnid_start_xmit         (struct sk_buff *skb, struct net_device *dev);
static int              cnid_poll               (struct net_device *dev, int *budget);
static int              cnid_close              (struct net_device *dev);
static int              cnid_ioctl              (struct net_device *dev, struct ifreq *rq, int cmd);
static struct net_device_stats *cnid_get_stats  (struct net_device *dev);
static void             cnid_set_multicast      (struct net_device *dev);
static irqreturn_t      cnid_tx_interrupt       (int irq, void *dev, struct pt_regs *regs);
static irqreturn_t      cnid_rx_interrupt       (int irq, void *dev, struct pt_regs *regs);
static void             cnid_do_tx_complete     (unsigned long data);

/*
 * TODO: This should not be here. It should be in a board/platform
 * specific file which is *actually* aware of what devices are
 * present on board.
 */
static ssize_t cnid_show_version(struct device_driver *drv, char *buf)
{
    return sprintf(buf, "%s, version: %s", DRV_NAME, DRV_VERSION);
}


static DRIVER_ATTR(version, S_IRUGO, cnid_show_version, NULL);

/**************************************************************************/
/*! \fn         cnid_bd_init
 **************************************************************************
 *
 *  \brief      INIT TX Descriptor
 *
 *  \param[in]  Pointer to Descriptor
 *  \param[in]  Queue Manager
 *  \param[in]  Queue Number
 *  \return     None
 **************************************************************************/
static inline void cnid_bd_init(struct cnid_desc* bd, int qmgr, int qnum)
{
    bd->hw.descInfo     =  PAL_CPPI4_HOSTDESC_DESC_TYPE_HOST << PAL_CPPI4_HOSTDESC_DESC_TYPE_SHIFT;
    bd->hw.pktInfo      = 
              (PAL_CPPI4_HOSTDESC_PKT_TYPE_ETH       << PAL_CPPI4_HOSTDESC_PKT_TYPE_SHIFT)
            | (PAL_CPPI4_HOSTDESC_PKT_RETPLCY_LINKED << PAL_CPPI4_HOSTDESC_PKT_RETPLCY_SHIFT)
            | (PAL_CPPI4_HOSTDESC_DESC_LOC_OFFCHIP   << PAL_CPPI4_HOSTDESC_DESC_LOC_SHIFT)
            | (qmgr                                  << PAL_CPPI4_HOSTDESC_PKT_RETQMGR_SHIFT)
            | (qnum                                  << PAL_CPPI4_HOSTDESC_PKT_RETQNUM_SHIFT);
}


/**************************************************************************/
/*! \fn         cnid_tx_bd_link_skb
 **************************************************************************
 *
 *  \brief      Links an skb to a Tx Descriptor
 *
 *  \param[in]  Net Device
 *  \param[in]  Descriptor
 *  \param[in]  SK buff
 *  \return     length of final packet
 **************************************************************************/
static inline unsigned int cnid_tx_bd_link_skb(struct net_device* dev, struct cnid_desc* bd, struct sk_buff *skb)
{
    unsigned int len = ((skb->len < ETH_ZLEN)?ETH_ZLEN:skb->len);
	unsigned int   queuePrio = 1;  /* Default queue is 182 normal priority */
        
    bd->hw.descInfo = PAL_CPPI4_HOSTDESC_DESC_TYPE_HOST << PAL_CPPI4_HOSTDESC_DESC_TYPE_SHIFT | len;
    bd->hw.bufPtr = PAL_CPPI4_VIRT_2_PHYS((unsigned int)skb->data);
    bd->hw.buffLen = len;
    bd->skb = skb;
    bd->psi[0] = skb->ti_meta_info; 
    bd->psi[1] = 0; /* MG just a patch to test the driver  */
    bd->psi[2] = 0; /* MG just a patch to test the driver  */

#ifdef PP_SUPPORT_SYNC_Q
	if ((skb->input_dev == wanDev)||(skb->input_dev == mtaDev))
	{
		/* Send packets from wan to high priority queue (181) - rest of data packets will go to Q182*/
		queuePrio = 0;
	}
    /* Set SYNC Q PTID info in egress descriptor */
    if(skb->pp_packet_info.ti_pp_flags == TI_PPM_SESSION_INGRESS_RECORDED)
    {
        memcpy(&(bd->hw.netInfoWord0), skb->pp_packet_info.ti_epi_header, 8);
        bd->hw.netInfoWord1 &= ~(0xFFFF);
        bd->hw.netInfoWord1 |= CNI_CPPI4x_TX_QNUM(queuePrio); /* after QPDSP, push to CNI Queues */
    }
    else
    {
		bd->hw.netInfoWord1 = CNI_CPPI4x_TX_QNUM(queuePrio); 
    }
#endif

    PAL_CPPI4_CACHE_WRITEBACK((unsigned long)skb->data, skb->len);        
    PAL_CPPI4_CACHE_WRITEBACK((unsigned long)bd, CPPI4_BD_LENGTH_FOR_CACHE);

    return len;
}


/**************************************************************************/
/*! \fn         cnid_rx_bd_link_skb
 **************************************************************************
 *
 *  \brief      Links an skb to a Rx Descriptor
 *
 *  \param[in]  Net Device
 *  \param[in]  Descriptor
 *  \param[in]  SK buff
 *  \return     length of final packet
 **************************************************************************/
static inline void cnid_rx_bd_link_skb(struct net_device* dev, struct cnid_desc* bd, struct sk_buff *skb)
{
    /* Invalidate the skb data cache: Do it before the data is presented to the DMA */
    PAL_CPPI4_CACHE_INVALIDATE(skb->data, RX_BUF_SIZE);
    skb->dev            = dev;
    skb_reserve (skb, NET_IP_ALIGN);    /* 16 byte align the IP fields. */        
    bd->hw.orgBuffLen  = RX_BUF_SIZE - NET_IP_ALIGN;
    bd->hw.orgBufPtr   = PAL_CPPI4_VIRT_2_PHYS(skb->data);
    bd->skb            = skb;

    bd->psi[0] = 0; 
    bd->psi[1] = 0;    
    bd->psi[2] = 0;

    /* Write the BD to the RAM */
    PAL_CPPI4_CACHE_WRITEBACK(bd, CPPI4_BD_LENGTH_FOR_CACHE);                
    
}

/** Tx processing function go here **/

#define cnid_is_tx_pool_empty(dev) !(((struct cnid_private*)netdev_priv(dev))->tx_bdlist)

static inline struct cnid_desc* cnid_get_tx_bd(struct cnid_private* priv)
{
    struct cnid_desc* ret = NULL;

    spin_lock_irq(&priv->txlock);        

    if(priv->tx_bdlist) {
        ret             = priv->tx_bdlist;
        priv->tx_bdlist = priv->tx_bdlist->next;
    }

    spin_unlock_irq(&priv->txlock);            

    return ret;
}

/* Put BD back to free Tx bd list */
static inline void cnid_put_tx_bd(struct cnid_private* priv, struct cnid_desc* bd)
{
    spin_lock_irq(&priv->txlock);
        
    bd->next         = priv->tx_bdlist;
    priv->tx_bdlist  = bd;

    spin_unlock_irq(&priv->txlock);        
}


/**************************************************************************/
/*! \fn         cnid_do_tx_complete
 **************************************************************************
 *
 *  \brief      Links an skb to a Rx Descriptor
 *
 *  \param[in]  Net Device
 *  \return     None
 **************************************************************************/
static void cnid_do_tx_complete(unsigned long data)
{
    struct net_device*      dev = (struct net_device*) data; 
    struct cnid_private*    priv = netdev_priv(dev);
    struct cnid_desc*       bd;
    int packets_processed = 0;
    int done = 1;

    DPRINTK(KERN_DEBUG " Enter %s \n", __FUNCTION__);
    while(avalanche_intd_get_interrupt_count(CNID_INTD_HOST_NUM, CNI_ACC_TXCMPL_CHNUM(0)))
    {
        while((bd = (struct cnid_desc*)((unsigned long)*priv->tx_list & QMGR_QUEUE_N_REG_D_DESC_ADDR_MASK)))
        {
            priv->stats.tx_packets++;
            priv->stats.tx_bytes += bd->skb->len;
            kfree_skb(bd->skb);
            bd->skb = NULL;

            /* Queue back the BD to free pool */
            cnid_put_tx_bd(priv, bd);

            packets_processed++;
            priv->tx_list++;            

            /* did enough work, move now.. */
            if(!test_bit(0, &priv->state) && (packets_processed == CIND_TX_SERVICE_MAX)) 
            {
                done = 0;
                goto out;
            }
        }

        /* Update the list entry for next time */
        priv->tx_list = PAL_cppi4AccChGetNextList(priv->tx_acc_hnd);

        avalanche_intd_set_interrupt_count (CNID_INTD_HOST_NUM, CNI_ACC_TXCMPL_CHNUM(0), 1);
    }

out:

    if(packets_processed && netif_queue_stopped(dev))
    {
        netif_start_queue(dev);
    }

    if(done)
    {
        avalanche_intd_write_eoi (CNI_ACC_TXCMPL_INTV);                
    }
    else 
    {
        tasklet_schedule(&priv->tx_tasklet);                            
    }
}


/**************************************************************************/
/*! \fn         cnid_start_xmit
 **************************************************************************
 *
 *  \brief      Transmit Function
 *
 *  \param[in]  SK buff
 *  \param[in]  Net Device
 *  \return     OK or error
 **************************************************************************/
static int cnid_start_xmit (struct sk_buff *skb, struct net_device *dev)
{
    struct cnid_private* priv = netdev_priv(dev);
    struct cnid_desc* bd;
    unsigned int len;
#ifndef PP_SUPPORT_SYNC_Q
    unsigned int prio = 0;
#endif

    DPRINTK(KERN_DEBUG " Enter %s \n", __FUNCTION__);
    /* get a free Tx descriptor */
    if(!(bd = cnid_get_tx_bd(priv)))
    {
        /* This should not occur in this driver 
         * (because of what is done later in this function) */            
        priv->stats.tx_dropped++;
        goto out_err;
    }

    len = cnid_tx_bd_link_skb(dev, bd, skb);

    dev->trans_start = jiffies;

#ifdef PP_SUPPORT_SYNC_Q
    /* Push to QPDSP Q - then after handle PTID it will be send to the CNI Tx queues */
    PAL_cppi4QueuePush(priv->tx_qos_queue, (Uint32 *) PAL_CPPI4_VIRT_2_PHYS(bd), CNID_QM_DESC_SIZE_CODE, len);
#else
    PAL_cppi4QueuePush(priv->tx_queue[prio], (Uint32 *) PAL_CPPI4_VIRT_2_PHYS(bd), CNID_QM_DESC_SIZE_CODE, len);
#endif    
    if(cnid_is_tx_pool_empty(dev))
    {
        netif_stop_queue(dev);  /* Cant do more packets now. Sorry. */
    }
      
    return NETDEV_TX_OK;

out_err:
    return NETDEV_TX_BUSY;    
}


/**************************************************************************/
/*! \fn         cnid_tx_interrupt
 **************************************************************************
 *
 *  \brief      Transmit ISR
 *
 *  \param[in]  IRQ
 *  \param[in]  Device
 *  \param[in]  regs
 *  \return     OK or error
 **************************************************************************/
static irqreturn_t cnid_tx_interrupt (int irq, void *dev, struct pt_regs *regs)
{
    struct cnid_private* priv = netdev_priv((struct net_device*) dev);

    DPRINTK(KERN_DEBUG " Enter %s \n", __FUNCTION__);
    tasklet_schedule(&priv->tx_tasklet);

    return IRQ_RETVAL(1);
}


/** Rx processing functions go here **/

/**************************************************************************/
/*! \fn         cnid_do_rx_complete
 **************************************************************************
 *
 *  \brief      Rx Complete handler
 *
 *  \param[in]  Net Device
 *  \param[in]  Processed packets budget
 *  \return     Number of processed packets
 **************************************************************************/
static int cnid_do_rx_complete(struct net_device* dev, int budget)
{
    struct cnid_private* priv = netdev_priv(dev);
    struct cnid_desc* bd;
    int packets_processed = 0, i;

    /* process high priority packets first */    
    for(i = 0; i < CNI_ACC_RX_CH_COUNT; i++)
    {            
        while(avalanche_intd_get_interrupt_count(CNID_INTD_HOST_NUM, priv->rxAcc_chan[i]))
        {
            while((bd = (struct cnid_desc*)((unsigned long)*priv->rxAcc_chan_list[i] 
                                            & QMGR_QUEUE_N_REG_D_DESC_ADDR_MASK)))
            {
                struct sk_buff *newskb;

                bd = PAL_CPPI4_PHYS_2_VIRT(bd);

                /* get a new skb for this bd */

                if(!test_bit(0, &priv->state)) {   /* if not cleaning up .. */
                        
                    if((newskb = dev_alloc_skb(RX_BUF_SIZE))) { /* .. and able to get a new buffer */

                        struct sk_buff* rxskb = bd->skb;                        

                        PAL_CPPI4_CACHE_INVALIDATE(bd, CPPI4_BD_LENGTH_FOR_CACHE);                        
                        skb_put(rxskb, bd->hw.buffLen - 4); /* remove CRC from length */
                        dev->last_rx = jiffies;
                        priv->stats.rx_packets++;
                        priv->stats.rx_bytes += bd->hw.buffLen;
                    
#ifdef PP_SUPPORT_SYNC_Q
                        /* Keep SYNC Q PTID info in skb for egress */
                        if(bd->hw.netInfoWord1)
                        {
                            memcpy(rxskb->pp_packet_info.ti_epi_header, &(bd->hw.netInfoWord0), 8);
                            rxskb->pp_packet_info.ti_pp_flags = TI_PPM_SESSION_INGRESS_RECORDED;
                        }
#endif
                        /* ... then send the packet up */
                        {
                            extern int DBridge_receive (struct sk_buff *skb);
                            
                            /* This is the DOCSIS Interface; pass the packet to the DOCSIS Bridge; initialize the
                            * MAC RAW Pointer before doing so. */
                            rxskb->mac.raw = rxskb->data;
                            rxskb->ti_meta_info = bd->psi[0];
                            DBridge_receive (rxskb);
                            DPRINTK(KERN_DEBUG "cnid_do_rx_complete: %d packets received\n",(int)(priv->stats.rx_packets));
                        }

                        /* Prepare to return to free Proxy queue */                                    
                        cnid_bd_init(bd, rxProxy_free_queue[acc_to_proxy[i]].qMgr, rxProxy_free_queue[acc_to_proxy[i]].qNum);

                        cnid_rx_bd_link_skb(dev, bd, newskb);
                    }
                }

                packets_processed++;                                    
                priv->rxAcc_chan_list[i]++;                                    
                /* Return to free queue */                                    
#ifdef PP_MERGE_HOST_DESC
                PAL_cppi4QueuePush(priv->rxCni_free_queue_hnd[i], (Uint32 *) PAL_CPPI4_VIRT_2_PHYS(bd),
                                CNID_QM_DESC_SIZE_CODE, 0);            
#else
                PAL_cppi4QueuePush(priv->rxProxy_free_queue_hnd[acc_to_proxy[i]], (Uint32 *) PAL_CPPI4_VIRT_2_PHYS(bd),
                                CNID_QM_DESC_SIZE_CODE, 0);            
#endif                
                /* thats it, we did enough. Jump out now! */
                if(budget == packets_processed) goto out;
            }   

            /* Update the list entry for next time */        
            priv->rxAcc_chan_list[i] = PAL_cppi4AccChGetNextList(priv->rxAcc_chan_hnd[i]);        
            avalanche_intd_set_interrupt_count (CNID_INTD_HOST_NUM, priv->rxAcc_chan[i], 1);                
        }
    }
     
out:    
    return packets_processed;
}


/**************************************************************************/
/*! \fn         cnid_poll
 **************************************************************************
 *
 *  \brief      Polling function
 *
 *  \param[in]  Net Device
 *  \param[in]  Processed packets budget
 *  \return     Number of processed packets
 **************************************************************************/
static int cnid_poll(struct net_device *dev, int *budget)
{
    int orig_budget = min(*budget, dev->quota);
    int done = 1;
    int work_done;

    DPRINTK(KERN_DEBUG " Enter %s \n", __FUNCTION__);
    work_done = cnid_do_rx_complete(dev, orig_budget);
    
    if (likely(work_done > 0))
    {
        *budget -= work_done;
        dev->quota -= work_done;
        done = (work_done < orig_budget); /* "done" would be zero if we hit budget */
    }

    /* order is important here. If we do EOI before calling netif_rx_complete, an interrupt
     * can occur just before we take ourselves out of the poll list; we will not 
     * schedule NAPI thread on that interrupt, no further Rx interrupts and 
     * Rx will stall forever. Scary...
     * */
    if (done)
    {
        netif_rx_complete(dev);        
        avalanche_intd_write_eoi (CNI_ACC_RX_INTV);            
    }

    return !done; /* poll should return 0 if finished, 1 if not */        
}



/**************************************************************************/
/*! \fn         cnid_rx_interrupt
 **************************************************************************
 *
 *  \brief      Receive ISR
 *
 *  \param[in]  IRQ
 *  \param[in]  Device
 *  \param[in]  regs
 *  \return     OK or error
 **************************************************************************/
static irqreturn_t cnid_rx_interrupt (int irq, void *dev_instance, struct pt_regs *regs)
{
    struct net_device *dev = (struct net_device *) dev_instance;
    
    DPRINTK(KERN_DEBUG " Enter %s \n", __FUNCTION__);

    /* if poll routine is not running, start it now. */
    netif_rx_schedule(dev);
        
    return IRQ_RETVAL(1);
}

/** Driver init/open/close functions go here **/

/**************************************************************************/
/*! \fn         cnid_free_rxpool
 **************************************************************************
 *
 *  \brief      Rx Pool FREE function
 *              Note this will not rewind the queue pushes
 *              (not needed as queues are reset at start)
 *
 *  \param[in]  Net Device
 *  \param[in]  Pool Base Address
 *  \param[in]  SK buffers number
 *  \param[in]  Queue Manager
 *  \return     None
 **************************************************************************/
static inline void cnid_free_rxpool(struct net_device *dev, Ptr pool_base, int num_skb, int qmgr)
{
    struct cnid_desc *desc;
    struct cnid_private* priv = netdev_priv(dev);            
    int j;

    if(pool_base)
    {
    /*-----------------------------------*/
    /* Decrease number of the Rx BD pool */
    /*  ??? !!!     Temporary    !!! ??? */
    /*-----------------------------------*/
        for(j = 0; j < num_skb/2; j++)
        {
            desc = GET_BD_PTR(pool_base, j);
            dev_kfree_skb(desc->skb);       
        }

        PAL_cppi4DeallocDesc(priv->pal_hnd, qmgr, pool_base);
    }
}


/**************************************************************************/
/*! \fn         cnid_init_rxpool
 **************************************************************************
 *
 *  \brief      Allocate and initialize Rx BD pool
 *
 *  \param[in]  Net Device
 *  \param[in]  Queue Handle
 *  \param[in]  Queue Manager
 *  \param[in]  Descriptors number
 *  \param[in]  Queue Number
 *  \return     Pool Address
 **************************************************************************/
static inline Ptr cnid_init_rxpool(struct net_device *dev, PAL_Cppi4QueueHnd hnd, int qMgr, int num_bd, Uint32 qNum)
{
    struct cnid_private* priv = netdev_priv(dev);        
    Ptr ptr;
    int i=0;
    
    /* Allocate BD pools.. */ 
    if(!(ptr = PAL_cppi4AllocDesc (priv->pal_hnd, qMgr, num_bd, CNID_BD_SIZE)))
    {
        DPRINTK(KERN_ERR "%s: Unable to allocate Rx BD pool!\n", __FUNCTION__);
        goto rewind;
    }
    
    /* .. and prepare the BDs... */
    /*-----------------------------------*/
    /* Decrease number of the Rx BD pool */
    /*  ??? !!!     Temporary    !!! ??? */
    /*-----------------------------------*/
    for (i = 0; i < num_bd/2; i++)
    {
        struct sk_buff* skb = dev_alloc_skb(RX_BUF_SIZE);
        struct cnid_desc* bd = GET_BD_PTR(ptr, i);

        if(!skb)
        {
            DPRINTK(KERN_ERR "%s: Unable to allocate %dth Rx buffers\n", __FUNCTION__, i);
            goto rewind;
        }

        PAL_osMemSet(bd, 0, CNID_BD_SIZE);

        cnid_bd_init(bd, qMgr, qNum);  /* return policy used only in case of Tx */
        cnid_rx_bd_link_skb(dev, bd, skb);

        PAL_cppi4QueuePush (hnd, (Ptr) PAL_CPPI4_VIRT_2_PHYS(bd), CNID_QM_DESC_SIZE_CODE, 0);        
    }

    return ptr;

rewind:     
    cnid_free_rxpool(dev, ptr, i, qMgr);        
    return NULL;
}


/**************************************************************************/
/*! \fn         cnid_free_txpool
 **************************************************************************
 *
 *  \brief      Tx Pool FREE function
 *              Note this will not rewind the queue pushes
 *              (not needed as queues are reset at start)
 *
 *  \param[in]  Net Device
 *  \param[in]  Pool Base Address
 *  \param[in]  SK buffers number
 *  \param[in]  Queue Manager
 *  \return     None
 **************************************************************************/
static inline void cnid_free_txpool(struct net_device *dev, Ptr pool_base, int num_skb, int qmgr)
{
    struct cnid_desc *desc;
    struct cnid_private* priv = netdev_priv(dev);            
    int j;

    if(pool_base) 
    {            
        for(j = 0; j < num_skb; j++)
        {
            desc = GET_BD_PTR(pool_base, j);
            if(desc->skb) dev_kfree_skb(desc->skb);       
        }

        PAL_cppi4DeallocDesc(priv->pal_hnd, qmgr, pool_base);
    }
}


/**************************************************************************/
/*! \fn         cnid_init_txpool
 **************************************************************************
 *
 *  \brief      Allocate and initialize Tx BD pool
 *
 *  \param[in]  Net Device
 *  \return     Pool Address
 **************************************************************************/
static inline Ptr cnid_init_txpool(struct net_device *dev)
{
    struct cnid_private* priv = netdev_priv(dev);        
    Ptr ptr;
    int i;
        
    /* Allocate BD pool, and maintain them in a software queue */        
    if(!(ptr = PAL_cppi4AllocDesc (priv->pal_hnd, CNI_CPPI4x_TX_QMGR, CNID_NUM_TX_BD, CNID_BD_SIZE)))
    {
        return NULL;
    }

    for (i = 0; i < CNID_NUM_TX_BD; i++)
    {
        struct cnid_desc* bd = GET_BD_PTR(ptr, i);
        PAL_osMemSet(bd, 0, CNID_BD_SIZE);
        cnid_bd_init(bd, CNI_CPPI4x_TX_COMP_QMGR, CNI_CPPI4x_TX_COMP_QNUM(0));
        cnid_put_tx_bd(priv, bd);   /* Queue it up to the global free Tx BD pool */
    }    
    return ptr;
}


/*
 * In the Tx path, there is no DMA channel to configure.
 * The NI driver just queues packets to one of the 8 Tx queues,
 * the docsis MAC COP picks up the descriptors and queues the
 * Tx completed path to the Tx complete queue. The accumulator
 * works on this queue to accumulate packets in the list buffer
 */
static inline int cnid_init_acc_chan(PAL_Handle pal_hnd, int chan_num, Cppi4Queue queue, PAL_Cppi4AccChHnd* acc_hnd)
{
    Cppi4AccumulatorCfg cfg;
    *acc_hnd = NULL;

    cfg.accChanNum             = chan_num;
    cfg.list.maxPageEntry      = CNID_ACC_PAGE_NUM_ENTRY;        /* This is entries per page (and we have 2 pages) */
    cfg.list.listEntrySize     = CNID_ACC_ENTRY_TYPE;            /* Only interested in register 'D' which has the desc pointer */
    cfg.list.listCountMode     = 0;                              /* Zero indicates null terminated list. */
    cfg.list.pacingMode        = 1;                              /* Wait for time since last interrupt */
    cfg.pacingTickCnt          = 40;                             /* Wait for 1000uS == 1ms */
    cfg.list.maxPageCnt        = CNID_ACC_NUM_PAGE;              /* Use two pages */
    cfg.list.stallAvoidance    = 1;                              /* Use the stall avoidance feature */
    cfg.queue                  = queue;                
    cfg.mode                   = 0;
   
    /* kmalloc returns cache line aligned memory unless you are debugging the slab allocator (2.6.18) */
    if(!(cfg.list.listBase = kzalloc(CNID_ACC_LIST_BYTE_SZ, GFP_KERNEL)))
    {
        DPRINTK(KERN_ERR "%s: Unable to allocate list page\n", __FUNCTION__);
        return -1;
    }

    PAL_CPPI4_CACHE_WRITEBACK((unsigned long)cfg.list.listBase, CNID_ACC_LIST_BYTE_SZ);

    cfg.list.listBase = (Ptr) PAL_CPPI4_VIRT_2_PHYS((Ptr)cfg.list.listBase);

    if(!(*acc_hnd = PAL_cppi4AccChOpen(pal_hnd, &cfg)))
    {
        DPRINTK(KERN_ERR "%s: Unable to open accumulator channel #%d\n", __FUNCTION__, chan_num);
        kfree(cfg.list.listBase);
        return -1;
    }

    return 0;
}


/**************************************************************************/
/*! \fn         cnid_open_tx
 **************************************************************************
 *
 *  \brief      Open Tx routine
 *
 *  \param[in]  Net Device
 *  \return     OK or error
 **************************************************************************/
static int cnid_open_tx(struct net_device* dev)
{
    struct cnid_private* priv = netdev_priv(dev);                  
    Cppi4Queue tx_cmpl  = {CNI_CPPI4x_TX_COMP_QMGR, CNI_CPPI4x_TX_COMP_QNUM(0)};
    Cppi4Queue queue;   /* used generically */     
    int i = 0, j;
    
    if(!(priv->tx_bdpool = cnid_init_txpool(dev)))
    {
        DPRINTK(KERN_ERR "%s: unable to allocate descriptor pool!\n", __FUNCTION__);                
        goto err_pool;
    }

    /* Initialize Tx queues */
    queue.qMgr = CNI_CPPI4x_TX_QMGR;

    for(i = 0; i < CNI_CPPI4x_TX_Q_COUNT; i++)
    {
        queue.qNum = CNI_CPPI4x_TX_QNUM(i);
        if(!(priv->tx_queue[i] = PAL_cppi4QueueOpen(priv->pal_hnd, queue)))
        {
            DPRINTK(KERN_ERR "%s: unable to open Tx Queue #%d!\n", __FUNCTION__, i);                
            goto err;
        }
    }

#ifdef PP_SUPPORT_SYNC_Q
    /* cable_pp: need to do it once in puma5_pp.c and use handle from there */
    /* cable_pp: need to do it for all QOS queues */
    queue.qMgr = PPFW_CPPI4x_RX_HOST_QMGR;
    queue.qNum = PPFW_CPPI4x_RX_HOST_QNUM(3);
    if(!(priv->tx_qos_queue = PAL_cppi4QueueOpen(priv->pal_hnd, queue)))
    {
        printk(KERN_ERR "%s: unable to open Tx QOS Queue #%d!\n", __FUNCTION__, i);                
        goto err;
    }
#endif

    /* reset Tx completion queue */
    PAL_cppi4QueueClose(priv->pal_hnd, PAL_cppi4QueueOpen(priv->pal_hnd, tx_cmpl));

    tasklet_init(&priv->tx_tasklet, cnid_do_tx_complete, (unsigned long) dev);

    /* Init the Tx complete accumulator channel */    
    if(cnid_init_acc_chan(priv->pal_hnd, CNI_ACC_TXCMPL_CHNUM(0), tx_cmpl, &priv->tx_acc_hnd))
    {
        DPRINTK(KERN_ERR "%s: unable to open accumulator channel!\n", __FUNCTION__);            
        goto err;  
    }

    priv->tx_list_base = priv->tx_list = PAL_cppi4AccChGetNextList(priv->tx_acc_hnd);    

#ifdef INIT_EXT_DMA
        /* temporary code to init the DMA , no error handling is done, no channel close*/
        /* docsis sw team will need to remove it from the driver !!! */
        {
             volatile Cppi4RxChInitCfg  chInfo;
             Cppi4BufPool               bufPool = {0,13}; //cable_pp: use CNI pool
             Cppi4Queue                 fdQueue = {1,60}; //cable_pp: QMGR = 1 !!!!!!!
             PAL_Cppi4RxChHnd           chHdl;
             volatile Cppi4TxChInitCfg  txchInfo;
             PAL_Cppi4TxChHnd           txchHdl;
             static Cppi4Queue          tdQueue = {0,0}; /* we don't have teardown one so don't care */
             Ptr                        bufPoolPtr;

//#ifdef PP_MERGE_HOST_DESC
            /* Init buffer manager for US (these buffers used by US co-processor)*/

            bufPool.bMgr  = 0;
            bufPool.bPool = 3;
            bufPoolPtr = PAL_cppi4BufPoolInit (priv->pal_hnd, bufPool, True, 2048, 256);
            if(bufPoolPtr == NULL)
            {
                DPRINTK(KERN_ERR "%s: Unable to init buffer pool \n", __FUNCTION__);
                goto err;
            }
//#endif                
        /* init co-proc TX channel */
            txchInfo.chNum = 15;
            txchInfo.dmaNum = 1;
            txchInfo.tdQueue = tdQueue;
            
            txchHdl = PAL_cppi4TxChOpen (priv->pal_hnd, (Cppi4TxChInitCfg *)(&txchInfo), NULL);
            if(txchHdl == NULL)
            {
                DPRINTK(KERN_ERR "%s: Unable to open %d channel \n", __FUNCTION__,txchInfo.chNum);
                goto err;
            }
            PAL_cppi4EnableTxChannel (txchHdl, NULL);

        /* init co-proc RX channel qman 1 queue */
            chInfo.chNum = 15;
            chInfo.dmaNum = 1;
            chInfo.defDescType = CPPI41_DESC_TYPE_EMBEDDED;
            chInfo.sopOffset=0;

            chInfo.u.embeddedPktCfg.fdQueue = fdQueue;
            chInfo.u.embeddedPktCfg.numBufSlot = 2;
            chInfo.u.embeddedPktCfg.sopSlotNum = 0;
            chInfo.u.embeddedPktCfg.fBufPool[0] = bufPool;  
            chInfo.u.embeddedPktCfg.fBufPool[1] = bufPool;  
            chInfo.u.embeddedPktCfg.fBufPool[2] = bufPool;  
            chInfo.u.embeddedPktCfg.fBufPool[3] = bufPool;  

            chHdl = PAL_cppi4RxChOpen (priv->pal_hnd, (Cppi4RxChInitCfg *)(&chInfo), NULL);
            if(chHdl == NULL)
            {
                DPRINTK(KERN_ERR "%s: Unable to open %d channel \n", __FUNCTION__,chInfo.chNum);
                goto err;
            }
            PAL_cppi4EnableRxChannel (chHdl, NULL);
        }
#endif

    /* request the Tx IRQs */        
    if(request_irq (CNID_TXINT_NUM, cnid_tx_interrupt, IRQF_DISABLED, dev->name, dev))
    {
        DPRINTK(KERN_ERR "%s: unable to get IRQ #%d!\n", __FUNCTION__, CNID_TXINT_NUM);            
        goto err;
    }

    return 0;

err:
    PAL_cppi4DeallocDesc (priv->pal_hnd, CNI_CPPI4x_TX_QMGR, priv->tx_bdpool);
    for(j = 0; j < i ; j++)
    {
        queue.qNum = CNI_CPPI4x_TX_QNUM(j);
        PAL_cppi4QueueClose(priv->pal_hnd, priv->tx_queue[j]);
    }    
err_pool:
    return -1;
}


/**************************************************************************/
/*! \fn         cnid_open_rx
 **************************************************************************
 *
 *  \brief      Open Rx routine
 *
 *  \param[in]  Net Device
 *  \return     Ok or error
 **************************************************************************/
static int cnid_open_rx(struct net_device* dev)
{
    int i;
    struct cnid_private* priv = netdev_priv(dev);

    /*
     * Prepare DOCSIS proxy channels
     * =============================
     */
    {
        int iProxyChan;

        /* 
         * Open Docsis Proxy Tx channels
         * -----------------------------
         * Input queues are predefined from Q[222]..
         * TxCompl queues are from Q[122+2]
         */
        for (iProxyChan=0; iProxyChan<CNI_CPPI4x_DOC2HOST_PROXY_CH_COUNT; iProxyChan++)
        {
            volatile Cppi4TxChInitCfg ProxyTxChInfo;
            PAL_Cppi4TxChHnd ProxyTxChHdl;

            ProxyTxChInfo.chNum = CNI_CPPI4x_DOC2HOST_PROXY_CHNUM(iProxyChan);
            ProxyTxChInfo.dmaNum = 1;
            ProxyTxChInfo.tdQueue.qMgr = CNI_CPPI4x_TX_COMP_QMGR;
            ProxyTxChInfo.tdQueue.qNum = CNI_CPPI4x_TX_COMP_QNUM(iProxyChan);
            ProxyTxChInfo.defDescType = CPPI41_DESC_TYPE_EMBEDDED;

            DPRINTK("%s: Call PAL_cppi4TxChOpen channel=%d\n", __FUNCTION__, ProxyTxChInfo.chNum);
            ProxyTxChHdl = PAL_cppi4TxChOpen (priv->pal_hnd, (Cppi4TxChInitCfg *)(&ProxyTxChInfo), NULL);
            if(ProxyTxChHdl == NULL)
            {
                DPRINTK(KERN_ERR "%s: Unable to open %d channel \n", __FUNCTION__, ProxyTxChInfo.chNum);
                goto err;
            }
            PAL_cppi4EnableTxChannel (ProxyTxChHdl, NULL);
            priv->txProxy_chan_hnd[iProxyChan] = ProxyTxChHdl;
        }

        /* 
         * Prepare Proxy rx queues
         * -----------------------
         * Free queues are from Q[128+8] 
         * Rx complete queues are Q[100+5]
         */
#ifdef NOT_ENOUGH_REGIONS
        iProxyChan=0;
#else
        for (iProxyChan=0; iProxyChan<CNI_CPPI4x_DOC2HOST_PROXY_CH_COUNT; iProxyChan++)
#endif
        {
            priv->rxProxy_queue_hnd[iProxyChan] = PAL_cppi4QueueOpen(priv->pal_hnd, rxProxy_queue[iProxyChan]);

            if(!(priv->rxProxy_free_queue_hnd[iProxyChan] = PAL_cppi4QueueOpen(priv->pal_hnd, rxProxy_free_queue[iProxyChan])))
            {
                DPRINTK(KERN_ERR "%s: Unable to open free desc queue %d priority channel\n", __FUNCTION__, rxProxy_free_queue[iProxyChan].qNum);
                goto err;
            }

            if(!(priv->rxProxy_bdpool[iProxyChan] = cnid_init_rxpool(dev, priv->rxProxy_free_queue_hnd[iProxyChan], rxProxy_free_queue[iProxyChan].qMgr, rxProxy_numbd[iProxyChan], rxProxy_free_queue[iProxyChan].qNum)))
            {
                DPRINTK(KERN_ERR "%s: Unable to init BD pool for %d priority channel\n", __FUNCTION__, iProxyChan);
                goto err;        
            }
        }

#ifdef NOT_ENOUGH_REGIONS
        priv->rxProxy_free_queue_hnd[1] = priv->rxProxy_free_queue_hnd[0];
        priv->rxProxy_free_queue_hnd[2] = priv->rxProxy_free_queue_hnd[0];
#endif
        /* 
         * Open Docsis Proxy Rx channels
         * -----------------------------
         * Free queues are from Q[128+8]
         * Output queues are Q[100+5]
         */
        for (iProxyChan=0; iProxyChan<CNI_CPPI4x_DOC2HOST_PROXY_CH_COUNT; iProxyChan++)
        {
            volatile Cppi4RxChInitCfg ProxyRxChInfo;
            PAL_Cppi4RxChHnd ProxyRxChHdl;

            ProxyRxChInfo.chNum = CNI_CPPI4x_DOC2HOST_PROXY_CHNUM(iProxyChan);
            ProxyRxChInfo.dmaNum = 1;
            ProxyRxChInfo.rxCompQueue.qMgr = rxProxy_queue[iProxyChan].qMgr;
            ProxyRxChInfo.rxCompQueue.qNum = rxProxy_queue[iProxyChan].qNum;
            ProxyRxChInfo.sopOffset = 0; // SOF skip=0
            ProxyRxChInfo.defDescType = CPPI41_DESC_TYPE_HOST;
            ProxyRxChInfo.u.hostPktCfg.fdbQueue[0] = rxProxy_free_queue[iProxyChan];
            ProxyRxChInfo.u.hostPktCfg.fdbQueue[1] = rxProxy_free_queue[iProxyChan];
            ProxyRxChInfo.u.hostPktCfg.fdbQueue[2] = rxProxy_free_queue[iProxyChan];
            ProxyRxChInfo.u.hostPktCfg.fdbQueue[3] = rxProxy_free_queue[iProxyChan];

            DPRINTK("%s: Call PAL_cppi4RxChOpen channel=%d\n", __FUNCTION__, ProxyRxChInfo.chNum);
            ProxyRxChHdl = PAL_cppi4RxChOpen (priv->pal_hnd, (Cppi4RxChInitCfg *)(&ProxyRxChInfo), NULL);
            if(ProxyRxChHdl == NULL)
            {
                DPRINTK(KERN_ERR "%s: Unable to open %d channel \n", __FUNCTION__, ProxyRxChInfo.chNum);
                goto err;
            }
            PAL_cppi4EnableRxChannel (ProxyRxChHdl, NULL);
            priv->rxProxy_chan_hnd[iProxyChan] = ProxyRxChHdl;
        }
    }

    /*
     * Prepare Accumulator channels
     * ============================
     */
    {
        int iAccChan;

        for(iAccChan=0; iAccChan < CNI_ACC_RX_CH_COUNT; iAccChan++) 
        {
            /* 
             * Open Docsis Accumulator channels 
             * --------------------------------
             * Acc channels are 28..30
             * !!!! queues are the same as rxProxy queues, Q[100+5]
             */
            priv->rxAcc_chan_hnd[iAccChan] = NULL;
            priv->rxAcc_chan[iAccChan] = CNI_ACC_RX_CHNUM(iAccChan);
            if(cnid_init_acc_chan(priv->pal_hnd, priv->rxAcc_chan[iAccChan], rxProxy_queue[acc_to_proxy[iAccChan]], &priv->rxAcc_chan_hnd[iAccChan]))
            {
                DPRINTK(KERN_ERR "%s: Unable to open accumulator channel for %d priority channel\n", __FUNCTION__, iAccChan);
                goto err;                        
            }
            priv->rxAcc_chan_list_base[iAccChan] = priv->rxAcc_chan_list[iAccChan] = PAL_cppi4AccChGetNextList(priv->rxAcc_chan_hnd[iAccChan]);
        }
    }
    /*
     * Prepare CNI Rx channels
     * ====================
     */
    {
        int iCniChan;

        /* 
         * Prepare Cni Rx queues
         * ---------------------
         * Free queues are from Q[144+7].. 
         * Rx complete queues are from Q[56].. - Prefetcher queues
         */
#ifdef NOT_ENOUGH_REGIONS
        iCniChan=0;
#else
        for (iCniChan=0; iCniChan<CNI_CPPI4x_RX_DMA_CH_COUNT; iCniChan++)
#endif

        {
            int i_bd;
            // priv->rxCni_queue_hnd[iCniChan] = PAL_cppi4QueueOpen(priv->pal_hnd, rxCni_queue[iCniChan]);

            if(!(priv->rxCni_free_queue_hnd[iCniChan] = PAL_cppi4QueueOpen(priv->pal_hnd, rxCni_free_queue[iCniChan])))
            {
                DPRINTK(KERN_ERR "%s: Unable to open free desc queue %d priority channel\n", __FUNCTION__, rxCni_free_queue[iCniChan].qNum);
                goto err;
            }
#ifdef PP_MERGE_HOST_DESC
            /*
             * Define free host descriptors for channel
             * --------------------------------------------
             */
            DPRINTK("%s: Allocate descriptors to the host free queues\n", __FUNCTION__);
            if(!(priv->rxCni_bdpool[iCniChan] = cnid_init_rxpool(dev, priv->rxCni_free_queue_hnd[iCniChan], rxCni_free_queue[iCniChan].qMgr, rxCni_numbd[iCniChan], rxCni_free_queue[iCniChan].qNum))) {
                DPRINTK(KERN_ERR "%s: Unable to init BD pool for %d priority channel\n", __FUNCTION__, iCniChan);
                goto err;        
            }
#else
            /*
             * Define free embedded descriptors for channel
             * --------------------------------------------
             */
            DPRINTK("%s: Allocate descriptors to the embedded free queues\n", __FUNCTION__);
    
            if(!(priv->rxCni_bdpool[iCniChan] = PAL_cppi4AllocDesc (priv->pal_hnd, rxCni_free_queue[iCniChan].qMgr, rxCni_numbd[iCniChan], CNID_BD_SIZE)))
            {
                DPRINTK(KERN_ERR "%s: Unable to init BD pool for %d priority channel\n", __FUNCTION__, iCniChan);
                goto err;        
            }

            for (i_bd = 0; i_bd < rxCni_numbd[iCniChan]; i_bd++) 
            {
                Cppi4EmbdDesc *bd = GET_BD_PTR(priv->rxCni_bdpool[iCniChan], i_bd);

                PAL_osMemSet(bd, 0, CNID_BD_SIZE);
                bd->descInfo     = CPPI41_EM_DESCINFO_DTYPE_EMBEDDED | CPPI41_EM_DESCINFO_SLOTCNT_MYCNT;
                bd->tagInfo      = 0;
                bd->pktInfo      = (PAL_CPPI4_HOSTDESC_PKT_TYPE_ETH << CPPI41_EM_PKTINFO_PKTTYPE_SHIFT) 
                        | (CPPI41_EM_PKTINFO_RETPOLICY_RETURN) 
                        | (1 << CPPI41_EM_PKTINFO_PROTSPEC_SHIFT) 
                        | (rxCni_free_queue[iCniChan].qMgr << CPPI41_EM_PKTINFO_RETQMGR_SHIFT) 
                        | (rxCni_free_queue[iCniChan].qNum << CPPI41_EM_PKTINFO_RETQ_SHIFT);
                PAL_CPPI4_CACHE_WRITEBACK(bd, CPPI4_BD_LENGTH_FOR_CACHE);
                PAL_cppi4QueuePush (priv->rxCni_free_queue_hnd[iCniChan], (Ptr) PAL_CPPI4_VIRT_2_PHYS(bd), CNID_QM_EMB_DESC_SIZE_CODE, 0);        
            }
#endif
        }

#ifdef NOT_ENOUGH_REGIONS
        priv->rxCni_free_queue_hnd[1] = priv->rxCni_free_queue_hnd[0];
        priv->rxCni_free_queue_hnd[2] = priv->rxCni_free_queue_hnd[0];
        priv->rxCni_free_queue_hnd[3] = priv->rxCni_free_queue_hnd[0];
        priv->rxCni_free_queue_hnd[4] = priv->rxCni_free_queue_hnd[0];
#endif
        /* 
         * Open Docsis Cni Rx channels
         * -----------------------------
         * Free queues are from Q[128+8]
         * Output queues are from Q[100+5]
         */
        for (iCniChan=0; iCniChan<CNI_CPPI4x_RX_DMA_CH_COUNT; iCniChan++)
        {
            volatile Cppi4RxChInitCfg CniRxChInfo;
            PAL_Cppi4RxChHnd CniRxChHdl;

            CniRxChInfo.chNum = CNI_CPPI4x_RX_DMA_CHNUM(iCniChan);
            CniRxChInfo.dmaNum = 1;
            CniRxChInfo.rxCompQueue.qMgr = rxCni_queue[iCniChan].qMgr;
            CniRxChInfo.rxCompQueue.qNum = rxCni_queue[iCniChan].qNum;
            CniRxChInfo.sopOffset = 0; // SOF skip=0

            CniRxChInfo.defDescType = CPPI41_DESC_TYPE_EMBEDDED;
            CniRxChInfo.u.embeddedPktCfg.fdQueue.qMgr = rxCni_free_queue[iCniChan].qMgr;
            CniRxChInfo.u.embeddedPktCfg.fdQueue.qNum = rxCni_free_queue[iCniChan].qNum;
            CniRxChInfo.u.embeddedPktCfg.numBufSlot = (EMSLOTCNT-1);
            CniRxChInfo.u.embeddedPktCfg.sopSlotNum = 1;
            CniRxChInfo.u.embeddedPktCfg.fBufPool[0].bMgr = 0;  

			CniRxChInfo.u.embeddedPktCfg.fBufPool[0].bPool = CNI_CPPI4x_POOL_NUM(0); 
            CniRxChInfo.u.embeddedPktCfg.fBufPool[1].bMgr = 0;
            CniRxChInfo.u.embeddedPktCfg.fBufPool[1].bPool = CNI_CPPI4x_POOL_NUM(0); 
            CniRxChInfo.u.embeddedPktCfg.fBufPool[2].bMgr = 0;
            CniRxChInfo.u.embeddedPktCfg.fBufPool[2].bPool = CNI_CPPI4x_POOL_NUM(0); 
            CniRxChInfo.u.embeddedPktCfg.fBufPool[3].bMgr = 0;
            CniRxChInfo.u.embeddedPktCfg.fBufPool[3].bPool = CNI_CPPI4x_POOL_NUM(0); 

            DPRINTK("%s: Call PAL_cppi4RxChOpen channel=%d\n", __FUNCTION__, CniRxChInfo.chNum);
            CniRxChHdl = PAL_cppi4RxChOpen (priv->pal_hnd, (Cppi4RxChInitCfg *)(&CniRxChInfo), NULL);
            if(CniRxChHdl == NULL)
            {
                DPRINTK(KERN_ERR "%s: Unable to open %d channel \n", __FUNCTION__, CniRxChInfo.chNum);
                goto err;
            }
            PAL_cppi4EnableRxChannel (CniRxChHdl, NULL);
            priv->rxCni_chan_hnd[iCniChan] = CniRxChHdl;
        }
    }

    /* request the Rx IRQs */            
    if(request_irq (CNID_RXINT_NUM, cnid_rx_interrupt, IRQF_DISABLED, dev->name, dev))
    {
        DPRINTK(KERN_ERR "%s: unable to get IRQ #%d!\n", __FUNCTION__, CNID_RXINT_NUM);
        goto err;
    }

    return 0;

err:
    for(i=0; i < CNI_CPPI4x_DOC2HOST_PROXY_CH_COUNT; i++) 
    {
        if(priv->rxProxy_free_queue_hnd[i]) 
            PAL_cppi4QueueClose(priv->pal_hnd, priv->rxProxy_free_queue_hnd[i]);
        if(priv->rxProxy_bdpool[i]) 
            cnid_free_rxpool(dev, priv->rxProxy_bdpool[i], rxProxy_numbd[i], rxProxy_free_queue[i].qMgr);
    }
    
    for(i=0; i < CNI_ACC_RX_CH_COUNT; i++) 
    {
        if(priv->rxAcc_chan_hnd[i]) 
        {
            kfree(priv->rxAcc_chan_list_base[i]);
            PAL_cppi4AccChClose(priv->rxAcc_chan_hnd[i], NULL);
        }
    }

    return -1;
}


/**************************************************************************/
/*! \fn         cnid_open
 **************************************************************************
 *
 *  \brief      CNI Device Open API
 *
 *  \param[in]  Net Device
 *  \return     Ok
 **************************************************************************/
static int cnid_open (struct net_device *dev)
{
    DPRINTK(KERN_DEBUG " Enter %s \n", __FUNCTION__);

#ifdef CONFIG_ARM_AVALANCHE_PPD
    cni_pp_set_pid_flags(dev, 0);
#endif

    if(cnid_open_tx(dev)) return -1;
    if(cnid_open_rx(dev)) return -1;

    netif_start_queue (dev);    
        
    return 0;
}


static inline int cnid_close_tx_prep(struct net_device* dev)
{
    struct cnid_private* priv = netdev_priv(dev);                          

    /* de-init the Tx complete accumulator channel */    
    PAL_cppi4AccChClose(priv->tx_acc_hnd, NULL);                

    /* disable the Tx IRQs */        
    disable_irq (CNID_TXINT_NUM);    

    return 0;
}


static inline int cnid_close_tx_finish(struct net_device* dev)
{
    struct cnid_private* priv = netdev_priv(dev);                  
    int i;
      
    /* de-init the Tx queues */
    for(i = 0; i < CNI_CPPI4x_TX_Q_COUNT; i++) {
        PAL_cppi4QueueClose(priv->pal_hnd, priv->tx_queue[i]);
    }

    cnid_free_txpool(dev, priv->tx_bdpool, CNID_NUM_TX_BD, CNI_CPPI4x_TX_COMP_QMGR);    
    
    tasklet_kill(&priv->tx_tasklet);

    kfree(priv->tx_list_base);

    free_irq (CNID_TXINT_NUM, dev);        

    return 0;
}


static inline int cnid_close_rx_prep(struct net_device* dev)
{
    struct cnid_private* priv = netdev_priv(dev);                          
    int i;

    /* disable the Rx IRQs */        
    disable_irq (CNID_RXINT_NUM);    

    for(i = 0; i < CNI_ACC_RX_CH_COUNT; i++) {
        /* De-init the Rx high and low priority channels */    
        PAL_cppi4AccChClose(priv->rxAcc_chan_hnd[i], NULL);
    }

    return 0;
}


static inline int cnid_close_rx_finish(struct net_device* dev)
{
    int i;        
    struct cnid_private* priv = netdev_priv(dev);                          

    for(i=0; i < CNI_CPPI4x_DOC2HOST_PROXY_CH_COUNT; i++) 
    {
        if(priv->rxProxy_free_queue_hnd[i]) 
            PAL_cppi4QueueClose(priv->pal_hnd, priv->rxProxy_free_queue_hnd[i]);
        if(priv->rxProxy_bdpool[i]) 
            cnid_free_rxpool(dev, priv->rxProxy_bdpool[i], rxProxy_numbd[i], rxProxy_free_queue[i].qMgr);
    }
    
    for(i=0; i < CNI_ACC_RX_CH_COUNT; i++) 
    {
        if(priv->rxAcc_chan_hnd[i]) 
        {
            kfree(priv->rxAcc_chan_list_base[i]);
            PAL_cppi4AccChClose(priv->rxAcc_chan_hnd[i], NULL);
        }
    }

    /* free the Rx IRQs */        
    free_irq (CNID_RXINT_NUM, dev);    

    return 0;
}


static int cnid_close (struct net_device *dev)
{
    struct cnid_private* priv = netdev_priv(dev);                              
    unsigned long flags;
    
    netif_stop_queue(dev);

#ifdef CONFIG_ARM_AVALANCHE_PPD
    cni_pp_set_pid_flags(dev, TI_PP_PID_DISCARD_ALL_RX);
#endif

    /*
     * Do the urgent stuff first
     */
    cnid_close_tx_prep(dev);
    cnid_close_rx_prep(dev);

    set_bit(0, &priv->state);
    spin_lock_irqsave(&priv->devlock, flags);    

    if(cnid_do_rx_complete(dev, rxProxy_numbd[0]+rxProxy_numbd[1]+rxProxy_numbd[2]))
        avalanche_intd_write_eoi(CNI_ACC_RX_INTV);

    cnid_do_tx_complete((unsigned long)dev);

    clear_bit(0, &priv->state);        
    spin_unlock_irqrestore(&priv->devlock, flags);    

    /* 
     * finish rest of janitorial stuff now
     */
    cnid_close_tx_finish(dev);
    cnid_close_rx_finish(dev);

    return 0;
}


static int cnid_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
    return 0;
}


static struct net_device_stats *cnid_get_stats (struct net_device *dev)
{
    struct cnid_private* priv = netdev_priv((struct net_device*) dev);
        
    return &priv->stats;
}


static void cnid_set_multicast (struct net_device *dev)
{    

}


static void cnid_tx_timeout (struct net_device *dev)
{
    struct cnid_private* priv = netdev_priv(dev);                              
    unsigned long flags;

    netif_stop_queue(dev);    

    spin_lock_irqsave(&priv->devlock, flags);           
    set_bit(0, &priv->state);    
        
    /* reset Tx processing */
    cnid_close_tx_prep(dev);    
            
    cnid_close_tx_finish(dev);  

    cnid_open_tx(dev);

    clear_bit(0, &priv->state);
    spin_unlock_irqrestore(&priv->devlock, flags);    
    
}


/* TODO: Hack this function to setup device address according to taste */
static void cnid_hard_addr_setup(struct net_device *dev)
{
    memcpy(dev->dev_addr, defmac, dev->addr_len);     
    memcpy(dev->perm_addr, dev->dev_addr, dev->addr_len);
}


static void cnid_netdev_setup(struct net_device *dev)
{
    dev->open = cnid_open;
    dev->hard_start_xmit = cnid_start_xmit;
    dev->poll = cnid_poll;
    dev->weight = CIND_RX_SERVICE_MAX;
    dev->stop = cnid_close;
    dev->get_stats = cnid_get_stats;
    dev->set_multicast_list = cnid_set_multicast;
    dev->do_ioctl = cnid_ioctl;
    // dev->tx_timeout = cnid_tx_timeout;            

    /* TODO: study the effects of these features */
    /* dev->features |= NETIF_F_SG | NETIF_F_HW_CSUM | NETIF_F_HIGHDMA */

    ether_setup(dev);
    cnid_hard_addr_setup(dev);
}


/* structure describing the CNID driver */
static struct device_driver cnid_driver = {
    .name       = "cni",
    .bus        = NULL,
    .probe      = cnid_probe,
    .remove     = cnid_remove,
    .suspend    = NULL,
    .resume     = NULL,
};


static int
cnid_read_proxy(char *buf, char **start, off_t offset, int count,
                  int *eof, void *data)
{
    struct net_device *dev = (struct net_device *) data;
    struct cnid_desc* currBD = NULL;
    struct cnid_private* priv = netdev_priv(dev);        
    int len = 0;

    len += sprintf(buf + len, "cni proxy \n");

    currBD = (struct cnid_desc*) PAL_cppi4QueuePop(priv->rxProxy_free_queue_hnd[0]);
    len += sprintf(buf + len, "read %x from %x \n",(unsigned int)currBD, (unsigned int)(priv->rxProxy_free_queue_hnd[0]));
    if(currBD != NULL)
    {

        currBD->hw.bufPtr = currBD->hw.orgBufPtr;
        currBD->hw.buffLen = 20;
        
        PAL_cppi4QueuePush (priv->rxProxy_queue_hnd[0],(Ptr) PAL_CPPI4_VIRT_2_PHYS(currBD), 
                                  CNID_QM_DESC_SIZE_CODE, 0);

    }

    return(len);
    
}



#ifdef CONFIG_ARM_AVALANCHE_PPD

/*
 * CNI PID handles
 * ---------------
 */

/* PID/VPID definitions */
#define PP_CNI_SR_DELAY     200
#include <asm-arm/arch-avalanche/puma5/puma5_pp.h> /* For PID base config */
#ifndef CONFIG_TI_PACKET_PROCESSOR
static TI_PP_PID cni_pid_list[PP_CNI_PID_COUNT];
#endif

/*
 * Set PID Flags
 * -------------
 */
static int cni_pp_set_pid_flags(struct net_device *dev, int flags)
{
#ifdef CONFIG_TI_PACKET_PROCESSOR
    ti_ppm_set_pid_flags (dev->pid_handle, flags);
#else
    ti_ppd_set_pid_flags (&cni_pid_list[0], flags);
#endif  
    /* this delay is to make sure all the packets with the PID successfully egress throgh the respective ports.*/
    mdelay(PP_CNI_SR_DELAY);
    return 0;
}

/*
 * Create CNI PID range, PID, VPID
 * -------------------------------
 * Defualt Q when no match - 222 (proxy ch queue)
 * Defualt Q when match - 182 (DOS US)
 */
static int cni_pp_prepare_pid(struct net_device *dev)
{
    TI_PP_PID_RANGE  pid_range_cni;
#ifdef CONFIG_TI_PACKET_PROCESSOR
    TI_PP_PID        cni_pid_list[PP_CNI_PID_COUNT];
#endif
    int ret_val;
    int iPid;

    /*
     * Config CNI PID range
     * --------------------
     */
    pid_range_cni.type        = TI_PP_PID_TYPE_DOCSIS;
    pid_range_cni.port_num    = CPPI41_SRCPORT_DOCSISMACPHY;
    pid_range_cni.count       = PP_CNI_PID_COUNT;
    pid_range_cni.base_index  = PP_CNI_PID_BASE;

#ifdef CONFIG_TI_PACKET_PROCESSOR
    if (ti_ppm_config_pid_range (&pid_range_cni))
#else
    if (ti_ppd_config_pid_range (&pid_range_cni))
#endif
        DPRINTK ("%s: config_pid_range failed\n", __FUNCTION__);

    /*
     * Create CNI PIDs
     * ---------------
     */
    // for (iPid=0; iPid<PP_CNI_PID_COUNT; iPid++) /* prepare for couple of CNI PIDs */
    iPid = 0;
    {
        cni_pid_list[iPid].type            = TI_PP_PID_TYPE_DOCSIS;    
        cni_pid_list[iPid].ingress_framing = TI_PP_PID_INGRESS_ETHERNET
                                           | TI_PP_PID_INGRESS_IPV6
                                           | TI_PP_PID_INGRESS_IPV4;
        cni_pid_list[iPid].pri_mapping     = 1;    /* Num prio Qs for fwd */
        cni_pid_list[iPid].dflt_pri_drp    = 0;
        cni_pid_list[iPid].dflt_dst_tag    = 0x3FFF;
        cni_pid_list[iPid].dflt_fwd_q      = CNI_CPPI4x_DOC2HOST_PROXY_QNUM(0);
        cni_pid_list[iPid].tx_pri_q_map[0] = CNI_CPPI4x_TX_QNUM(1); /* default Q for Egress #182 */
        cni_pid_list[iPid].tx_hw_data_len  = 0;
        cni_pid_list[iPid].pid_handle      = PP_CNI_PID_BASE+iPid;

#ifdef CONFIG_TI_PACKET_PROCESSOR
        if ((ret_val = ti_ppm_create_pid (&cni_pid_list[iPid])) < 0)
#else
        if ((ret_val = ti_ppd_create_pid (&cni_pid_list[iPid])))
#endif
        {
            DPRINTK ("%s: create_pid failed with error code %d.\n", __FUNCTION__, ret_val);
            cni_pid_list[iPid].pid_handle = -1;
        }
    
        dev->pid_handle = cni_pid_list[iPid].pid_handle;
    }

    /*
     * Create CNI VPIDs
     * ----------------
     */
#ifdef CONFIG_TI_PACKET_PROCESSOR
    dev->vpid_block.type               = TI_PP_ETHERNET;   
    dev->vpid_block.parent_pid_handle  = dev->pid_handle;
    dev->vpid_block.egress_mtu         = 0;
    dev->vpid_block.priv_tx_data_len   = 0;
#endif
    return 0;
}

#endif




unsigned int    cppiQueuePop(int qnum)
{
    volatile unsigned int* qPopAddr = (volatile unsigned int *)0xD906000C;

    qPopAddr += (qnum * 4);

    return *qPopAddr;
}



/**************************************************************************
 * FUNCTION NAME : cppi_write_cmds
 **************************************************************************
 * DESCRIPTION   :
 *  This is used to debug and display various 
 *  CPPI entity information from the console.
 *
 * RETURNS       :
 *  -1              - Error.
 *  Non-Zero        - Success.
 ***************************************************************************/
static int cppi_write_cmds (struct file *file, const char *buffer, unsigned long count, void *data) 
{
    char    proc_cmd[100];
    char*   argv[10];
    int     argc = 0;
    char*   ptr_cmd;
    char*   delimitters = " \n\t";
    char*   ptr_next_tok;

    /* Validate the length of data passed. */
    if (count > 100)
        count = 100; 

    /* Initialize the buffer before using it. */
    memset ((void *)&proc_cmd[0], 0, sizeof(proc_cmd));
    memset ((void *)&argv[0], 0, sizeof(argv));

    /* Copy from user space. */ 
    if (copy_from_user (&proc_cmd, buffer, count))
        return -EFAULT;

    ptr_next_tok = &proc_cmd[0];

    /* Tokenize the command. Check if there was a NULL entry. If so be the case the
     * user did not know how to use the entry. Print the help screen. */
    ptr_cmd = strsep(&ptr_next_tok, delimitters);
    if (ptr_cmd == NULL)
        return -1;

    /* Parse all the commands typed. */
    do
    {
        /* Extract the first command. */
        argv[argc++] = ptr_cmd;

        /* Validate if the user entered more commands.*/
        if (argc >=10)
        {
            printk ("ERROR: Incorrect too many parameters dropping the command\n");
            return -EFAULT;
        }

        /* Get the next valid command. */
        ptr_cmd = strsep(&ptr_next_tok, delimitters);
    } while (ptr_cmd != NULL);

    /* We have an extra argument when strsep is used instead of strtok */
    argc--;

    /******************************* Command Handlers *******************************/

    /* Display Command Handlers */
    if (strncmp(argv[0], "show", strlen("show")) == 0)
    {
        if (strncmp(argv[1], "mem", strlen("mem")) == 0)
        {
            unsigned int dmem = simple_strtol(argv[2], NULL, 16);
            int i;

            for (i=0; i<16; i++)
            {
                printk("[0x%08X]: 0x%08lx 0x%08lx 0x%08lx 0x%08lx\n", dmem+i*16,
                    *((unsigned long *)(dmem+i*16)),
                    *((unsigned long *)(dmem+i*16+4)),
                    *((unsigned long *)(dmem+i*16+8)),
                    *((unsigned long *)(dmem+i*16+12))
                    );
            }
        }
    }

    if (strncmp(argv[0], "pop", strlen("pop")) == 0)
    {
        unsigned int desc;
        int queue = (int) simple_strtol(argv[2], NULL, 0);
        int entries = 1;
        int i;

        if (argc >= 4)
        {
            entries = (int) simple_strtol(argv[3], NULL, 0);
        }

        do 
        {
            desc = cppiQueuePop(queue);

            printk(" POP Descriptor Q[%3d] = [0x%08x] \n", queue, (unsigned int)(desc));
        
            if (desc >= 0x80000000)
            {
                desc &= 0xFFFFFFe0;
                desc = (unsigned int)PAL_CPPI4_PHYS_2_VIRT(desc);
            
                for (i=0; i<4; i++)
                {
                    printk("\t0x%08lx 0x%08lx 0x%08lx 0x%08lx\n",
                        *((unsigned long *)(desc+i*16)),
                        *((unsigned long *)(desc+i*16+4)),
                        *((unsigned long *)(desc+i*16+8)),
                        *((unsigned long *)(desc+i*16+12))
                        );
                }
            }
            entries--;
        } while(desc && entries);
    }

    return count;
}




/* TODO: in a fully linux framework compliant driver,
 * we will receive the resources: io, mem and intr
 * in the device structure. 
 */
static int __devinit cnid_probe(struct device *dev)
{
    struct net_device *netdev = NULL;
    struct cnid_private *priv = NULL;
    int ret;

    driver_create_file(&cnid_driver, &driver_attr_version);    

    /* dev and priv zeroed in alloc_netdev. Thank God for small mercies... */
    netdev = alloc_netdev (sizeof(struct cnid_private), "cni%d", cnid_netdev_setup);
    if (netdev == NULL) {
        printk("cni0: Unable to alloc new net device\n");
        return -ENOMEM;
    }

    SET_MODULE_OWNER(netdev);
    SET_NETDEV_DEV(netdev, dev);
    platform_set_drvdata(to_platform_device(dev), netdev);

    priv = netdev_priv(netdev);

    ret = register_netdev (netdev);
    if(ret) {
        DPRINTK(KERN_DEBUG "Unable to register device named %s (%p)...\n", netdev->name, netdev);            
        return ret;
    }
    
    DPRINTK(KERN_DEBUG "Registered device named %s (%p)...\n", netdev->name, netdev);            

    /* get a PAL handle */
    priv->pal_hnd = PAL_cppi4Init(NULL, NULL);

#ifdef CONFIG_ARM_AVALANCHE_PPD
    /*
     * Config CNI PID/VPID
     */
    cni_pp_prepare_pid (netdev);
#endif

    spin_lock_init(&priv->txlock);
    spin_lock_init(&priv->devlock);

    create_proc_read_entry("cnid", 0, NULL, cnid_read_proxy, netdev);

    {
        struct proc_dir_entry *res=create_proc_entry("cppi",0,NULL);
        if (res) 
        {
            res->write_proc = cppi_write_cmds;
        }
    }
    return 0;
}


static int __devexit cnid_remove (struct device *dev)
{
    struct net_device *netdev = platform_get_drvdata (to_platform_device(dev));

    unregister_netdev (netdev);

    platform_device_unregister(to_platform_device(dev));    

    return 0;
}


static int __init cnid_init_module (void)
{
    static struct platform_device *cnid_dev;
        
    DPRINTK (KERN_INFO DRV_NAME "\n");

    if(sizeof(struct cnid_desc) > CNID_BD_SIZE) {
       DPRINTK(KERN_ERR "%s fundamentally broken. Contact maintainer!\n", DRV_NAME);
       return -1;
    }

    /* Initialize MAC address */
    if (inpmac && strlen(inpmac) == 17)
    {
        int i;
        int m[6];

        /* Translate MAC address from ASCII to binary */
        sscanf(inpmac, "%x:%x:%x:%x:%x:%x", &(m[0]), &(m[1]), &(m[2]), &(m[3]), &(m[4]), &(m[5]));
        for (i = 0; i < 6; i++)
        {
            defmac[i] = (Uint8)m[i];
        }
    }
    DPRINTK(KERN_INFO"cnid : Using MAC %x:%x:%x:%x:%x:%x", defmac[0], defmac[1], defmac[2], defmac[3], defmac[4], defmac[5]);

    /* TODO: this should be in a board file not here.
     * No fun registering driver and device in the same file. */
    cnid_dev = platform_device_register_simple("cni", -1, NULL, 0);

    cnid_driver.bus = platform_bus_type_ptr;
    if (driver_register(&cnid_driver)) {
        platform_device_unregister(cnid_dev);
        return -1;
    } 

    return 0;
}


static void __exit cnid_cleanup_module (void)
{
    driver_remove_file(&cnid_driver, &driver_attr_version);
    driver_unregister(&cnid_driver);
}

module_param(inpmac, charp, 0);
module_init(cnid_init_module);
module_exit(cnid_cleanup_module);

