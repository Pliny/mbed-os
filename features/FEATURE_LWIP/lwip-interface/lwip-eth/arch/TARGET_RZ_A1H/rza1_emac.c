//TODO #include "lwip/opt.h"
//TODO #include "lwip/tcpip.h"
//TODO #include "netif/etharp.h"
//TODO #include "lwip/ethip6.h"
#include "mbed_interface.h"

// TODO ADDED
#include "cmsis_os.h"
#include "fsl_phy.h"

#include "emac_api.h"
#include "emac_stack_mem.h"
#include "mbed_assert.h"
#include "mbed_error.h"
#include "nsapi_types.h"

// **********************************

#include "ethernet_api.h"
#include "ethernetext_api.h"

enet_handle_t g_handle;
// TX Buffer descriptors
uint8_t *tx_desc_start_addr;
// RX Buffer descriptors
uint8_t *rx_desc_start_addr;
// RX packet buffer pointers
emac_stack_mem_t *rx_buff[ENET_RX_RING_LEN];
// TX packet buffer pointers
emac_stack_mem_t *tx_buff[ENET_RX_RING_LEN];
// RX packet payload pointers
uint32_t *rx_ptr[ENET_RX_RING_LEN];

#define RZA1H_ETH_MTU_SIZE 1500
#define RZA1H_ETH_IF_NAME "en"

#define RECV_TASK_PRI           (osPriorityNormal)
#define PHY_TASK_PRI            (osPriorityNormal)
#define PHY_TASK_WAIT           (200)

/* memory */
//static osSemaphoreId (RxReadySem);    /* receive ready semaphore */
//static osSemaphoreDef(RxReadySem_def);

/* RZ_A1H EMAC driver data structure */
struct rza1h_enetdata {
    osSemaphoreId    RxReadySem; /**< RX packet ready semaphore */
    osSemaphoreDef_t RxReadySem_def; /**< RX semaphore: definition struct */
    uint32_t         RxReadySem_data[2]; /**< RX semaphore: object mem */

    osSemaphoreId    TxCleanSem; /**< TX cleanup thread wakeup semaphore */
    osSemaphoreDef_t TxCleanSem_def; /**< TX semaphore: definition struct */
    uint32_t         TxCleanSem_data[2]; /**< TX semaphore: object mem */

    osMutexId TXLockMutex; /**< TX critical section mutex */
    osMutexDef_t TXLockMutex_def; /**< TX mutex: definition struct */
    int32_t TXLockMutex_data[4]; /**< TX mutex: object mem */

    osSemaphoreId    xTXDCountSem; /**< TX free buffer counting semaphore */
    osSemaphoreDef_t xTXDCountSem_def; /**< TX free buffer semaphore: definition struct */
    uint32_t         xTXDCountSem_data[2]; /**< TX free buffer semaphore: object mem */

    osThreadId    RxThread; /**< Packet reception thread */
    osThreadDef_t RxThread_def; /**< Packet reception thread: definition struct */

    osThreadId    TxCleanThread; /**< Packet transmission cleanup thread */
    osThreadDef_t TxCleanThread_def; /**< Packet transmission cleanup thread: definition struct */

    osThreadId    PhyThread; /**< PHY monitoring thread */
    osThreadDef_t PhyThread_def; /**< PHY monitoring thread: definition struct */

    uint8_t tx_consume_index, tx_produce_index; /**< TX buffers ring */
    emac_link_input_fn emac_link_input_cb; /**< Callback for incoming data */
    void *emac_link_input_cb_data; /**< Data to be passed to input cb */
    emac_link_state_change_fn emac_link_state_cb; /**< Link state change callback */
    void *emac_link_state_cb_data; /**< Data to be passed to link state cb */
};

static struct rza1h_enetdata rza1h_enetdata;

/* function */
static void rza1_recv_task(void *arg);
static void rza1_phy_task(void *arg);
#if LWIP_IPV4
static err_t rza1_etharp_output_ipv4(struct netif *netif, struct pbuf *q, const ip4_addr_t *ipaddr);
#endif
#if LWIP_IPV6
static err_t rza1_etharp_output_ipv6(struct netif *netif, struct pbuf *q, const ip6_addr_t *ipaddr);
#endif
static err_t rza1_low_level_output(struct netif *netif, struct pbuf *p);
static void rza1_recv_callback(void);

static void rza1_recv_task(void *arg) {
    struct netif   *netif = (struct netif*)arg;
    u16_t          recv_size;
    struct pbuf    *p;
    int            cnt;

    while (1) {
        osSemaphoreWait(&RxReadySem, 0);
        for (cnt = 0; cnt < 16; cnt++) {
            recv_size = ethernet_receive();
            if (recv_size != 0) {
                p = pbuf_alloc(PBUF_RAW, recv_size, PBUF_RAM);
                if (p != NULL) {
                    (void)ethernet_read((char *)p->payload, p->len);
                    /* full packet send to tcpip_thread to process */
                    if (netif->input(p, netif) != ERR_OK) {
                        /* Free buffer */
                        pbuf_free(p);
                    }
                }
            } else {
                break;
            }
        }
    }
}

static void rza1_phy_task(void *arg) {
    struct netif *netif = (struct netif*)arg;
    s32_t        connect_sts = 0;   /* 0: disconnect, 1:connect */
    s32_t        link_sts;
    s32_t        link_mode_new = NEGO_FAIL;
    s32_t        link_mode_old = NEGO_FAIL;

    while (1) {
        link_sts = ethernet_link();
        if (link_sts == 1) {
            link_mode_new = ethernetext_chk_link_mode();
            if (link_mode_new != link_mode_old) {
                if (connect_sts == 1) {
                    tcpip_callback_with_block((tcpip_callback_fn)netif_set_link_down, (void*) netif, 1);
                }
                if (link_mode_new != NEGO_FAIL) {
                    ethernetext_set_link_mode(link_mode_new);
                    tcpip_callback_with_block((tcpip_callback_fn)netif_set_link_up, (void*) netif, 1);
                    connect_sts = 1;
                }
            }
        } else {
            if (connect_sts != 0) {
                tcpip_callback_with_block((tcpip_callback_fn)netif_set_link_down, (void*) netif, 1);
                link_mode_new = NEGO_FAIL;
                connect_sts   = 0;
            }
        }
        link_mode_old = link_mode_new;
        osDelay(PHY_TASK_WAIT);
    }
}

#if LWIP_IPV4
static err_t rza1_etharp_output_ipv4(struct netif *netif, struct pbuf *q, const ip4_addr_t *ipaddr) {
    /* Only send packet is link is up */
    if (netif->flags & NETIF_FLAG_LINK_UP) {
        return etharp_output(netif, q, ipaddr);
    }

    return ERR_CONN;
}
#endif

#if LWIP_IPV6
static err_t rza1_etharp_output_ipv6(struct netif *netif, struct pbuf *q, const ip6_addr_t *ipaddr) {
    /* Only send packet is link is up */
    if (netif->flags & NETIF_FLAG_LINK_UP) {
        return ethip6_output(netif, q, ipaddr);
    }

    return ERR_CONN;
}
#endif

static err_t rza1_low_level_output(struct netif *netif, struct pbuf *p) {
    struct pbuf *q;
    s32_t       cnt;
    err_t       err        = ERR_MEM;
    s32_t       write_size = 0;

    if ((p->payload != NULL) && (p->len != 0)) {
        /* If the first data can't be written, transmit descriptor is full. */
        for (cnt = 0; cnt < 100; cnt++) {
            write_size = ethernet_write((char *)p->payload, p->len);
            if (write_size != 0) {
                break;
            }
            osDelay(1);
        }
        if (write_size != 0) {
            for (q = p->next; q != NULL; q = q->next) {
                (void)ethernet_write((char *)q->payload, q->len);
            }
            if (ethernet_send() == 1) {
                err = ERR_OK;
            }
        }
    }

    return err;
}

static void rza1_recv_callback(void) {
    osSemaphoreRelease(&RxReadySem);
}

/** \brief  Low level init of the MAC and PHY.
 *
 *  \param[in]      enet       Pointer to RZ_A1H enet structure
 *  \param[in]      hwaddr     MAC address
 */
static err_t low_level_init(struct rza1h_enetdata *enet, char *hwaddr)
{
  uint8_t i;
  uint32_t sysClock;
  phy_speed_t phy_speed;
  phy_duplex_t phy_duplex;
  uint32_t phyAddr = 0;
  bool link = false;
  enet_config_t config;
  void *payload;

  // Allocate RX descriptors
  rx_desc_start_addr = (uint8_t *)calloc(1, sizeof(enet_rx_bd_struct_t) * ENET_RX_RING_LEN + ENET_BUFF_ALIGNMENT);
  if(!rx_desc_start_addr)
    return ERR_MEM;

  // Allocate TX descriptors
  tx_desc_start_addr = (uint8_t *)calloc(1, sizeof(enet_tx_bd_struct_t) * ENET_TX_RING_LEN + ENET_BUFF_ALIGNMENT);
  if(!tx_desc_start_addr)
    return ERR_MEM;

  rx_desc_start_addr = (uint8_t *)ENET_ALIGN(rx_desc_start_addr, ENET_BUFF_ALIGNMENT);
  tx_desc_start_addr = (uint8_t *)ENET_ALIGN(tx_desc_start_addr, ENET_BUFF_ALIGNMENT);

  /* Create buffers for each receive BD */
  for (i = 0; i < ENET_RX_RING_LEN; i++) {
    rx_buff[i] = emac_stack_mem_alloc(ENET_ETH_MAX_FLEN, ENET_BUFF_ALIGNMENT);
    if (NULL == rx_buff[i])
      return ERR_MEM;

    /* RZ_A1H note: the next line ensures that the RX buffer is properly aligned for the RZ_A1H
       RX descriptors (16 bytes alignment). However, by doing so, we're effectively changing
       a data structure which is internal to lwIP. This might not prove to be a good idea
       in the long run, but a better fix would probably involve modifying lwIP itself */
    payload = emac_stack_mem_ptr(rx_buff[i]);
    payload = (void*)ENET_ALIGN((uint32_t)payload, ENET_BUFF_ALIGNMENT);
    rx_ptr[i] = payload;
  }

  enet->tx_consume_index = enet->tx_produce_index = 0;

  /* prepare the buffer configuration. */
  enet_buffer_config_t buffCfg = {
    ENET_RX_RING_LEN,
    ENET_TX_RING_LEN,
    ENET_ALIGN(ENET_ETH_MAX_FLEN, ENET_BUFF_ALIGNMENT),
    0,
    (volatile enet_rx_bd_struct_t *)rx_desc_start_addr,
    (volatile enet_tx_bd_struct_t *)tx_desc_start_addr,
    (uint8_t *)&rx_ptr,
    NULL,
  };

  rza1h_init_eth_hardware();

  sysClock = CLOCK_GetFreq(kCLOCK_CoreSysClk);

  ENET_GetDefaultConfig(&config);

  PHY_Init(ENET, 0, sysClock);
  PHY_GetLinkStatus(ENET, phyAddr, &link);
  if (link)
  {
    /* Get link information from PHY */
    PHY_GetLinkSpeedDuplex(ENET, phyAddr, &phy_speed, &phy_duplex);
    /* Change the MII speed and duplex for actual link status. */
    config.miiSpeed = (enet_mii_speed_t)phy_speed;
    config.miiDuplex = (enet_mii_duplex_t)phy_duplex;
    config.interrupt = kENET_RxFrameInterrupt | kENET_TxFrameInterrupt;
  }
  config.rxMaxFrameLen = ENET_ETH_MAX_FLEN;
  config.macSpecialConfig = kENET_ControlFlowControlEnable;
  config.txAccelerConfig = kENET_TxAccelIsShift16Enabled;
  config.rxAccelerConfig = kENET_RxAccelisShift16Enabled | kENET_RxAccelMacCheckEnabled;
  ENET_Init(ENET, &g_handle, &config, &buffCfg, (uint8_t*)hwaddr, sysClock);
  ENET_SetCallback(&g_handle, ethernet_callback, enet);
  ENET_ActiveRead(ENET);

  return ERR_OK;
}

err_t eth_arch_enetif_init(emac_interface_t *emac)
{
    ethernet_cfg_t ethcfg;
    err_t err;

    err = low_level_init(struct rza1h_enetdata *enet, emac->hwaddr);
    if (err != ERR_OK)
        return false;

    /* CMSIS-RTOS, start tasks */
  memset(rza1h_enetdata.xTXDCountSem_data, 0, sizeof(rza1h_enetdata.xTXDCountSem_data));
  rza1h_enetdata.xTXDCountSem_def.semaphore = rza1h_enetdata.xTXDCountSem_data;
  rza1h_enetdata.xTXDCountSem = osSemaphoreCreate(&rza1h_enetdata.xTXDCountSem_def, ENET_TX_RING_LEN);
  MBED_ASSERT(rza1h_enetdata.xTXDCountSem != NULL);

  /* Transmission lock mutex */
  memset(rza1h_enetdata.TXLockMutex_data, 0, sizeof(rza1h_enetdata.TXLockMutex_data));
  rza1h_enetdata.TXLockMutex_def.mutex = rza1h_enetdata.TXLockMutex_data;
  rza1h_enetdata.TXLockMutex = osMutexCreate(&rza1h_enetdata.TXLockMutex_def);
  MBED_ASSERT(rza1h_enetdata.TXLockMutex != NULL);

  /* Packet receive task */
  memset(rza1h_enetdata.RxReadySem_data, 0, sizeof(rza1h_enetdata.RxReadySem_data));
  rza1h_enetdata.RxReadySem_def.semaphore = rza1h_enetdata.RxReadySem_data;
  rza1h_enetdata.RxReadySem = osSemaphoreCreate(&rza1h_enetdata.RxReadySem_def, 0);
  MBED_ASSERT(rza1h_enetdata.RxReadySem != NULL);

  /* Packet reception thread */
  rza1h_enetdata.RxThread_def.pthread = (os_pthread)packet_rx;
  rza1h_enetdata.RxThread_def.tpriority = RX_PRIORITY;
#ifdef LWIP_DEBUG
  rza1h_enetdata.RxThread_def.stacksize = DEFAULT_THREAD_STACKSIZE*5;
#else
  rza1h_enetdata.RxThread_def.stacksize = DEFAULT_THREAD_STACKSIZE;
#endif
  rza1h_enetdata.RxThread_def.stack_pointer = (uint32_t*)malloc(rza1h_enetdata.RxThread_def.stacksize);
  if (rza1h_enetdata.RxThread_def.stack_pointer == NULL)
    error("RxThread: Error allocating the stack memory");
  rza1h_enetdata.RxThread = osThreadCreate(&rza1h_enetdata.RxThread_def, &rza1h_enetdata);
  if (rza1h_enetdata.RxThread == NULL)
    error("RxThread: create error\n");

  /* Transmit cleanup task */
  memset(rza1h_enetdata.TxCleanSem_data, 0, sizeof(rza1h_enetdata.TxCleanSem_data));
  rza1h_enetdata.TxCleanSem_def.semaphore = rza1h_enetdata.TxCleanSem_data;
  rza1h_enetdata.TxCleanSem = osSemaphoreCreate(&rza1h_enetdata.TxCleanSem_def, 0);
  MBED_ASSERT(rza1h_enetdata.TxCleanSem != NULL);

  /* Transmission cleanup thread */
  rza1h_enetdata.TxCleanThread_def.pthread = (os_pthread)packet_tx;
  rza1h_enetdata.TxCleanThread_def.tpriority = TX_PRIORITY;
  rza1h_enetdata.TxCleanThread_def.stacksize = DEFAULT_THREAD_STACKSIZE;
  rza1h_enetdata.TxCleanThread_def.stack_pointer = (uint32_t*)malloc(rza1h_enetdata.TxCleanThread_def.stacksize);
  if (rza1h_enetdata.TxCleanThread_def.stack_pointer == NULL)
    error("TxCleanThread: Error allocating the stack memory");
  rza1h_enetdata.TxCleanThread = osThreadCreate(&rza1h_enetdata.TxCleanThread_def, &rza1h_enetdata);
  if (rza1h_enetdata.TxCleanThread == NULL)
    error("TxCleanThread: create error\n");

  /* PHY monitoring task */
  rza1h_enetdata.PhyThread_def.pthread = (os_pthread)rza1h_phy_task;
  rza1h_enetdata.PhyThread_def.tpriority = PHY_PRIORITY;
  rza1h_enetdata.PhyThread_def.stacksize = DEFAULT_THREAD_STACKSIZE;
  rza1h_enetdata.PhyThread_def.stack_pointer = (uint32_t*)malloc(rza1h_enetdata.PhyThread_def.stacksize);
  if (rza1h_enetdata.PhyThread_def.stack_pointer == NULL)
    error("PhyThread: Error allocating the stack memory");
  rza1h_enetdata.PhyThread = osThreadCreate(&rza1h_enetdata.PhyThread_def, &rza1h_enetdata);
  if (rza1h_enetdata.PhyThread == NULL)
    error("PhyThread: create error\n");

  /* Allow the PHY task to detect the initial link state and set up the proper flags */
  osDelay(10);

  return true;

//    /* set MAC hardware address */
//#if (MBED_MAC_ADDRESS_SUM != MBED_MAC_ADDR_INTERFACE)
//    netif->hwaddr[0] = MBED_MAC_ADDR_0;
//    netif->hwaddr[1] = MBED_MAC_ADDR_1;
//    netif->hwaddr[2] = MBED_MAC_ADDR_2;
//    netif->hwaddr[3] = MBED_MAC_ADDR_3;
//    netif->hwaddr[4] = MBED_MAC_ADDR_4;
//    netif->hwaddr[5] = MBED_MAC_ADDR_5;
//#else
//    mbed_mac_address((char *)netif->hwaddr);
//#endif
//    netif->hwaddr_len = ETH_HWADDR_LEN;
//
//    /* maximum transfer unit */
//    netif->mtu = 1500;
//
//    /* device capabilities */
//    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET;
//#ifdef LWIP_IGMP
//    netif->flags |= NETIF_FLAG_IGMP;
//#endif
//#if LWIP_IPV6_MLD
//    netif->flags |= NETIF_FLAG_MLD6;
//#endif
//
//#if LWIP_NETIF_HOSTNAME
//    /* Initialize interface hostname */
//    netif->hostname = "lwiprza1";
//#endif /* LWIP_NETIF_HOSTNAME */
//
//    netif->name[0] = 'e';
//    netif->name[1] = 'n';
//
//#if LWIP_IPV4
//    netif->output = rza1_etharp_output_ipv4;
//#endif
//#if LWIP_IPV6
//  netif->output_ip6 = rza1_etharp_output_ipv6;
//#endif
//
//    netif->linkoutput = rza1_low_level_output;
//
//    /* Initialize the hardware */
//    ethcfg.int_priority = 6;
//    ethcfg.recv_cb      = &rza1_recv_callback;
//    ethcfg.ether_mac    = (char *)netif->hwaddr;
//    ethernetext_init(&ethcfg);
//
//    /* semaphore */
//    RxReadySem = osSemaphoreCreate(&RxReadySem_def, 0);
//
//    /* task */
//    sys_thread_new("rza1_recv_task", rza1_recv_task, netif, DEFAULT_THREAD_STACKSIZE, RECV_TASK_PRI);
//    sys_thread_new("rza1_phy_task", rza1_phy_task, netif, DEFAULT_THREAD_STACKSIZE, PHY_TASK_PRI);
//
//    return ERR_OK;
}

void eth_arch_enable_interrupts(void) {
    ethernetext_start_stop(1);
}

void eth_arch_disable_interrupts(void) {
    ethernetext_start_stop(0);
}

static uint32_t rza1h_eth_get_mtu_size(emac_interface_t *emac)
{
  return K64_ETH_MTU_SIZE;
}

static void rza1h_eth_get_ifname(emac_interface_t *emac, char *name, uint8_t size)
{
  memcpy(name, K64_ETH_IF_NAME, (size < sizeof(K64_ETH_IF_NAME)) ? size : sizeof(K64_ETH_IF_NAME));
}

static uint8_t rza1h_eth_get_hwaddr_size(emac_interface_t *emac)
{
    return NSAPI_MAC_BYTES;
}

static void rza1h_eth_get_hwaddr(emac_interface_t *emac, uint8_t *addr)
{
  mbed_mac_address((char*)addr);
}

static void rza1h_eth_set_hwaddr(emac_interface_t *emac, uint8_t *addr)
{
  /* No-op at this stage */
}

static void rza1h_eth_set_link_input_cb(emac_interface_t *emac, emac_link_input_fn input_cb, void *data)
{
  struct rza1h_enetdata *enet = emac->hw;

  enet->emac_link_input_cb = input_cb;
  enet->emac_link_input_cb_data = data;
}

static void rza1h_eth_set_link_state_cb(emac_interface_t *emac, emac_link_state_change_fn state_cb, void *data)
{
  struct rza1h_enetdata *enet = emac->hw;

  enet->emac_link_state_cb = state_cb;
  enet->emac_link_state_cb_data = data;
}

static void rza1h_eth_add_multicast_group(emac_interface_t *emac, uint8_t *addr)
{
  ENET_AddMulticastGroup(ENET, addr);
}

static void k64f_eth_power_down(emac_interface_t *emac)
{
  /* No-op at this stage */
}

const emac_interface_ops_t rza1h_eth_emac_ops = {
    .get_mtu_size = rza1h_eth_get_mtu_size,
    .get_ifname = rza1h_eth_get_ifname,
    .get_hwaddr_size = rza1h_eth_get_hwaddr_size,
    .get_hwaddr = rza1h_eth_get_hwaddr,
    .set_hwaddr = rza1h_eth_set_hwaddr,
    .link_out = rza1h_eth_link_out,
    .power_up = rza1h_eth_power_up,
    .power_down = rza1h_eth_power_down,
    .set_link_input_cb = rza1h_eth_set_link_input_cb,
    .set_link_state_cb = rza1h_eth_set_link_state_cb,
    .add_multicast_group = rza1h_eth_add_multicast_group
};

static emac_interface_t mbed_emac_eth_default = {&rza1h_eth_emac_ops, &rza1h_enetdata};