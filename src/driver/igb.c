#include "igb_type.h"
#include "igb.h"

#include "log.h"
#include "pci.h"

#include <stddef.h>
#include <stdint.h>

#define IXY_TO_IGB(ixy_device) container_of(ixy_device, struct igb_device, ixy)

static char const * driver_name = "ixy-igb";

typedef struct __attribute__((packed)) {
  uint64_t physical_addr;
  uint16_t length;
  uint8_t  cso;

  // cmd bits
  uint8_t  EOP   : 1; // end of packet
  uint8_t  IFCS  : 1; // insert FCS
  uint8_t  IC    : 1; // insert checksum
  uint8_t  RS    : 1; // report status
  uint8_t  _rsv1 : 1; // reserved
  uint8_t  DEXT  : 1; // descriptor description (0 for legacy)
  uint8_t  vle   : 1; // vlan insertion enabled
  uint8_t  _rsv2 : 1; // reserved

  uint8_t  sta;
  uint8_t  _rsv3;
  uint8_t  css;
  uint8_t  vlan;
} tx_descriptor_t; // "legacy" version

typedef struct __attribute__((packed)) {
  uint64_t physical_addr;
  uint16_t length;
  uint16_t checksum;

  // status bits
  uint8_t DD     : 1; // descritor done
  uint8_t EOP    : 1; // end of packet (fragmenetation support)
  uint8_t  _rsv1 : 1; // resererved
  uint8_t VP     : 1; // packet is vlan tagged, stripped vlan and matched with VET register
  uint8_t UDPCS  : 1; // udp or ip payload checksum comuputed on packet
  uint8_t L4CS   : 1; // L4 checksum (UDP or TCP) computed on packet
  uint8_t IPCS   : 1; // IPv4 checksum computed on packet
  uint8_t PIF    : 1; // passed imperfect filter only

  /* notes on EOP/DD (table 7-6)
     DD   | EOP  | meaning
     0b0  | 0b0  | software should init to these values
     0b0  | 0b1  | invalid
     0b1  | 0b0  | non-last descriptor of a packet that spans mutliple dds
     0b1  | 0b1  | competion finished for entire packet

     Bunch of other wild details about CP and the checksum fields.

     PIF field indicates that the card wasn't sure if we should have recieved
     the packet or not. Does this has something to do with promiscuous mode?
  */

  // errors, most only reported when RECV.SBP set (store bad packets)
  uint8_t _rsv2 : 5;
  uint8_t L4E   : 1; // TCP/UDP checksum error
  uint8_t IPE   : 1; // IPv4 checksum error
  uint8_t RXE   : 1; // rx data error

  uint16_t vlan;  // vlan tag, only set when VET (filter) and CTRL.VME = 0b1
} rx_descriptor_t;

typedef struct {
  uint16_t         n;
  tx_descriptor_t* descriptors;
  uint16_t         tail;
  void*            vaddr[1024]; // FIXME sz
} tx_q_t;

typedef struct {
  uint16_t         n;
  rx_descriptor_t* descriptors;
  uint16_t         tail;              // last packet we've processed
	struct mempool*  mempool;           // pool to allocate recv buffers from
  struct pkt_buf*  stored_bufs[1024]; // pointers to buffers in use
} rx_q_t;

static void
print_status( struct igb_device* dev )
{
  uint32_t st = get_reg32(dev->addr, E1000_STATUS);

  // FIXME switch to logger

  printf("status: ");
  if (st & E1000_STATUS_FD)    printf(" FULL_DUPLEX"); else printf(" HALF_DUPLEX");
  if (st & E1000_STATUS_LU)    printf(" LINK_UP");     else printf(" LINK_DOWN");
  if (st & E1000_STATUS_TXOFF) printf(" TRANSMISSION_PAUSED"); else printf(" TRANSMISSION_ACTIVE"); // FIXME add recv status

  if ((st & E1000_STATUS_SPEED_MASK) == E1000_STATUS_SPEED_10)   printf(" 10 mbps");
  if ((st & E1000_STATUS_SPEED_MASK) == E1000_STATUS_SPEED_100)  printf(" 100 mbps");
  if ((st & E1000_STATUS_SPEED_MASK) == E1000_STATUS_SPEED_1000) printf(" 1000 mbps");

  st = get_reg32(dev->addr, E1000_CTRL_EXT);
  if ((st & (0b11 << 22)) == (0b00 << 22)) printf(" link=copper");
  if ((st & (0b11 << 22)) == (0b01 << 22)) printf(" link=1000BASE-KX");
  if ((st & (0b11 << 22)) == (0b10 << 22)) printf(" link=SGMII");
  if ((st & (0b11 << 22)) == (0b11 << 22)) printf(" link=SerDes");

  printf("\n");
}

static inline void
disable_interrupts( struct igb_device* dev )
{
	set_reg32(dev->addr, E1000_EIMC, 0x7FFFFFFF);
}

static void
init_tx( struct igb_device* dev )
{
  /* 4.6.10 */
  size_t qid = 0;
  tx_q_t* q = &((tx_q_t*)dev->tx_queues)[qid]; // FIXME more than one q

  /* disable q, 0 is enabled by default so this is safest */
  clear_flags32(dev->addr, E1000_TXDCTL(qid), 0b1<<25); // FIXME consntant

  /* allocate descriptors:
     - 32k max (not sure what tradeoffs are)
     - addr must be multiple of 128 for DMA */

  q->n = 128; // FIXME how to set this? on chip, there are 24 descs avail
  struct dma_memory mem = memory_allocate_dma(q->n * sizeof(tx_descriptor_t), true); // huge page
  q->descriptors = mem.virt;

  /* set addr (physical) */
  uint32_t hi = mem.phy>>32;
  uint32_t lo = mem.phy<<32>>32;
  set_reg32(dev->addr, E1000_TDBAH(qid), hi);
  set_reg32(dev->addr, E1000_TDBAL(qid), lo);
  info("bound virt %p (phys %p) to card", mem.virt, (void*)mem.phy);

  /* set length */
  set_reg32(dev->addr, E1000_TDLEN(qid), q->n);

  /* set TXDCTL, currently not doing anything because the writeback would add
     latency */
  /* set_flags32(dev->addr, E1000_TXDCTL(qid), 0b1<<16); */

  /* Enable all queues needed (zero enabled by default).
     Poll enable register until it enable true. */

  /* enable transmit after all other queues inintialized */
  set_flags32(dev->addr, E1000_TXDCTL(qid), 0b1<<25); // FIXME consntant

  set_flags32(dev->addr, E1000_TXDCTL(qid), 0b1<<25); // FIXME consntant

  wait_set_reg32(dev->addr, E1000_TXDCTL(qid), 0b1<<25); // FIXME consntant

  // fake successful send on all exising pkt
  for (size_t i = 0; i < q->n; ++i) {
    q->descriptors[i].sta = 1;
  }

  // FIXME enable crc offloading?
  // FIXME set buffer sizes?
}

void
init_rx( struct igb_device* dev )
{
  size_t qid = 0;
  rx_q_t* q = &((rx_q_t*)dev->rx_queues)[qid]; // FIXME more than one q?

  /* queue zero is enabled by default, so disable the queue no matter what */

  /* configure multicast.
     Currently clearing the entire table, promisc mode means we still get packets */

  for (size_t i = 0; i < 128; ++i) { // FIXME magic number
    set_reg32(dev->addr, E1000_MTA_N(i), 0);
  }

  /* configure RCTL */
  clear_flags32(dev->addr, E1000_RXDCTL(qid), 0b1<<25);
  /* docs claim there is no reset, but the header claims there is? */
  set_flags32(  dev->addr, E1000_RCTL, E1000_RCTL_RST);   // try it anyway
  clear_flags32(dev->addr, E1000_RCTL, E1000_RCTL_EN);    // disable
  clear_flags32(dev->addr, E1000_RCTL, E1000_RCTL_SBP);   // do not store bad packet
  set_flags32(  dev->addr, E1000_RCTL, E1000_RCTL_UPE);   // unicast promiscuous mode FIXME disable
  set_flags32(  dev->addr, E1000_RCTL, E1000_RCTL_MPE);   // multicast promiscuous mode FIXME disable
  clear_flags32(dev->addr, E1000_RCTL, E1000_RCTL_LPE);   // drop long packets
  clear_flags32(dev->addr, E1000_RCTL, 0b11<<6);          // no loopback FIXME sane?
  set_flags32(  dev->addr, E1000_RCTL, E1000_RCTL_BAM);   // allow recv broadcast
  clear_flags32(dev->addr, E1000_RCTL, 0b11<<16);         // clear all bits in size to indicate 2048 byte recv buffers
  //clear_flags32(dev->addr, E1000_RCTL, E1000_VFE);      // disable vlan filter
  // FIXME configure multicast bits?
  // remainder left at default values, but should perhaps be revisted

  /* get a DMA region and program the descriptor base address with region of
     descriptors */

  q->n = 128; // FIXME what is hardware limit?
  struct dma_memory mem = memory_allocate_dma(q->n * sizeof(rx_descriptor_t), true); // huge page
  q->descriptors = mem.virt;
  uint32_t hi = mem.phy>>32;
  uint32_t lo = mem.phy<<32>>32;
  set_reg32(dev->addr, E1000_RDBAH(qid), hi);
  set_reg32(dev->addr, E1000_RDBAL(qid), lo);
  info("bound virt %p (phys %p) to card", mem.virt, (void*)mem.phy);
  q->mempool = memory_allocate_mempool(1024, 2048); // size is wrong since we are using the first bit of the buffer for pkt_buf

  memset(mem.virt, 0, q->n * sizeof(rx_descriptor_t));
  for (size_t i = 0; i < q->n; ++i) {
    rx_descriptor_t* desc = q->descriptors + i;
    memset(desc, 0, sizeof(*desc));

    struct pkt_buf* next_buf = pkt_buf_alloc(q->mempool);
    if (!next_buf) error("failed to allocate buf");
    void* phy = (void*)(next_buf->buf_addr_phy + offsetof(struct pkt_buf, data));
    desc->physical_addr = (uint64_t)phy;
    q->stored_bufs[i] = next_buf;
  }

  /* setup descritpors with pointers to adequately sized regions of memory */

  /* set length */
  set_reg32(dev->addr, E1000_RDLEN(qid), q->n);

  /* set SRRCTL */
  // leave size default (using 2048 default for all pkts)
  set_flags32(dev->addr, E1000_SRRCTL(qid), 0b1<<31); /* allow drops */

  // does this need to happen before the queue enable?
  set_flags32(dev->addr, E1000_RCTL, E1000_RCTL_EN); // don't do one per q

  set_flags32(dev->addr, E1000_RXDCTL(qid), 0b1<<25);    // enable queue
  wait_set_reg32(dev->addr, E1000_RXDCTL(qid), 0b1<<25); // wait for enable
}

static void
reset_and_init( struct igb_device* dev )
{
  info("Resetting device %s", dev->ixy.pci_addr);

	/* section 4.6.3 - initialize the device
     1) disable interrupts
     2) issue global reset and disable interrupts again 4.6.4
     3) perform general configuration
     4) setup PHY and link
     5) init counters
     6) init recv
     7) init tx
     8) enable interrupts (optional, don't want them here) */

  disable_interrupts(dev);                                /* (1) */
  set_flags32(dev->addr, E1000_CTRL, E1000_CTRL_RST);     /* (2) */
	usleep(4 * 1000);                                       /* wait at least three millis */

  /* section 4.3.1 - wait for success */
  wait_set_reg32(dev->addr, E1000_STATUS, 1<<21); // FIXME add constant
  wait_set_reg32(dev->addr, E1000_EECD,   E1000_EECD_AUTO_RD);

  disable_interrupts(dev);  /* disabled for remainder of setup */

  /* (3) general configuration
     these settings are probably stored in EPROM, but setting them anyway. */

  set_flags32  (dev->addr, E1000_CTRL, E1000_CTRL_ASDE);   /* auto-detect speed */
  clear_flags32(dev->addr, E1000_CTRL, E1000_CTRL_FRCDPX); /* don't force a duplex setting */
  clear_flags32(dev->addr, E1000_CTRL, E1000_CTRL_FRCSPD); /* don't force a speed setting */
  clear_flags32(dev->addr, E1000_CTRL, E1000_CTRL_ILOS);   /* zero for copper */
  // anything else?

  print_status(dev);

  /* (4) setup PHY and link */
  clear_flags32(dev->addr, E1000_CTRL_EXT, 0b11 << E1000_CTRL_EXT_LINK_MODE_OFFSET); /* copper */

  /* Bring link up */
  set_flags32(   dev->addr, E1000_CTRL,   E1000_CTRL_SLU);
  wait_set_reg32(dev->addr, E1000_STATUS, E1000_STATUS_LU);

  /* note: it is very difficult to set anything other than auto negotiation.
     Fortunately specificity isn't terribly important here. */

  /* (5) init statistics - read them to zero everything */
  igb_read_stats(&dev->ixy, NULL);

  /* (6) init rx */
  init_rx(dev);

  /* (7) init tx */
  init_tx(dev);

  /* Not reenabling interrupts, we'll be polling */

  print_status(dev);
}

struct ixy_device* igb_init( char const * pci_addr,
                             uint16_t     rx_queues,
                             uint16_t     tx_queues )
{
  // FIXME validate inputs

  struct igb_device* dev = malloc(sizeof(*dev));
  dev->ixy.pci_addr = strdup(pci_addr);

	dev->ixy.driver_name = driver_name;
	dev->ixy.num_rx_queues = rx_queues;
	dev->ixy.num_tx_queues = tx_queues;

  // FIXME check queue sizes

  // pointers off to important functions
	dev->ixy.rx_batch       = igb_rx_batch;
	dev->ixy.tx_batch       = igb_tx_batch;
	dev->ixy.read_stats     = igb_read_stats;
	dev->ixy.set_promisc    = NULL;
	dev->ixy.get_link_speed = NULL;

  // FIXME check for iommu
  info("Init device as igb using raw pci");
  dev->addr = pci_map_resource(pci_addr);

  // FIXME should this be elsewhere?

  /* allocate the queue metadata regions */
  dev->tx_queues = calloc(8, sizeof(tx_q_t)); // FIXME 8?
  if (!dev->tx_queues) error("allocation");

  dev->rx_queues = calloc(8, sizeof(rx_q_t));
  if (!dev->rx_queues) error("allocation");

  reset_and_init(dev);
  set_flags32(dev->addr, E1000_TCTL, E1000_TCTL_EN);

  return &dev->ixy;
}

// stats registers reset on read, passing NULL allowed to reset the counters
void
igb_read_stats(struct ixy_device* ixy, struct device_stats* stats)
{
  struct igb_device* dev = IXY_TO_IGB(ixy);

  // there are many options for reading these counters, each has subtle behavior
  // probably doesn't matter for this project at all
  uint32_t rx_pkts = get_reg32(dev->addr, E1000_TPR);
  uint32_t tx_pkts = get_reg32(dev->addr, E1000_TPT);
  uint32_t rx_bytes = ((uint64_t)get_reg32(dev->addr, E1000_TORH)<<32) +  (uint64_t)get_reg32(dev->addr, E1000_TORL);
  uint32_t tx_bytes = ((uint64_t)get_reg32(dev->addr, E1000_TOTH)<<32) +  (uint64_t)get_reg32(dev->addr, E1000_TOTL);

  // stats jump to uint32 max when they overflow, detect this?

  if (stats) {
		stats->rx_pkts += rx_pkts;
		stats->tx_pkts += tx_pkts;
		stats->rx_bytes += rx_bytes;
		stats->tx_bytes += tx_bytes;
  }

  /* additonal available and relevant(ish):
     - CRC error recv
     - recv error count
     - missed packet count (recv FIFO has insufficient space)
     - defer count (could not send packet because something was busy)
     - host transmit discarded packets (host tried to send, packet was discarded by MAC)
     - receive length error (pkt too big or something)
     - wow there's a lot, also interested in the counts by type which are availabe as well.
     - RQDPC (rx queue drop count, when drop allowed)
  */
}

// advance index with wrap-around
// FIXME power of two actually checked somewhere
#define wrap_ring(index, ring_size) (uint16_t) ((index + 1) & (ring_size - 1))

uint32_t
igb_rx_batch( struct ixy_device* _dev,
              uint16_t           queue_id,
              struct pkt_buf*    bufs[],
              uint32_t           num_bufs )
{
  struct igb_device* dev = IXY_TO_IGB(_dev);
  if (queue_id != 0) error("bad q");

  rx_q_t* q = &((rx_q_t*)dev->tx_queues)[queue_id];

  /* We can consume packets at the head of the buffer, if there are any
     available. Keep going until we run out of space in the storage or until we
     run out of packets */

  size_t cnt = 0;
  while (1) {
    if (cnt >= num_bufs) break; // off by one? FIXME
    uint16_t idx = wrap_ring((size_t)q->tail + (size_t)cnt, q->n); // Do I need to copy this?
    rx_descriptor_t volatile* desc = &(q->descriptors[idx]);
    // info("idx: %u dd: %u", idx, desc->DD);
    // info("chk: %x", desc->checksum);
    // info("length: %u", desc->length);
    if (!desc->length) break;
    if (!desc->DD) break;
    if (!desc->EOP) error("non-EOP packet");

    // we have a valid packet, store this into one of the buffers and swap the
    // descriptor's address. ixy wants packet buffers so we'll give it packet
    // buffers
    struct pkt_buf* card_buf = q->stored_bufs[cnt];
    card_buf->size = desc->length;
    bufs[cnt] = card_buf; // store the pointer for client use

    // swap the pointer with a new buffer
    struct pkt_buf* next_buf = pkt_buf_alloc(q->mempool);
    if (!next_buf) error("failed to allocate buf");
    void* phy = (void*)(next_buf->buf_addr_phy + offsetof(struct pkt_buf, data));
    desc->physical_addr = (uint64_t)phy;
    desc->DD = 0; // reset
    desc->EOP = 0; // just in case
    q->stored_bufs[idx] = next_buf;

    cnt += 1;
  }

  /* move tail forward */
  /* update tail, and our version of tail */
  q->tail = wrap_ring((size_t)q->tail + (size_t)cnt, q->n);
  set_reg32(dev->addr, E1000_RDT(queue_id), q->tail);

  return cnt;
}

uint32_t
igb_tx_batch( struct ixy_device* _dev,
              uint16_t           queue_id,
              struct pkt_buf*    bufs[],
              uint32_t           num_bufs )
{
  struct igb_device* dev = IXY_TO_IGB(_dev);
  if (queue_id != 0) error("bad q");

  tx_q_t* q = &((tx_q_t*)dev->tx_queues)[queue_id];

  /* we can only use free spots that are at the end of the ring buffer.
     using the head ptr doesn't really work (it is updated before packet is
     fully sent), just make sure to set rs bit then
     we can just use those.

     Theres room for optimziation here, we could use writeback mode to batch
     these calls (with latency-to-return-buffer tradeoffs) */

  uint16_t cnt = 0; // ends up with cnt that are usable
  while (1) {
    if (cnt == q->n) break;

    uint16_t idx = wrap_ring((size_t)q->tail + (size_t)cnt, q->n);
    tx_descriptor_t volatile* curr = &q->descriptors[idx];
    if (0 == curr->sta) break; // can't keep going, nothing else usable

    if (q->vaddr[idx]) pkt_buf_free(q->vaddr[idx]); // not init when fresh, all null
    cnt += 1;
  }

  // starting at tail, we have cnt packets of space to write stuff
  // add them all to the queue, then move the ptr

  cnt = num_bufs < cnt ? num_bufs : cnt;
  for (size_t i = 0; i < cnt; ++i) {
    uint16_t                  idx  = wrap_ring((size_t)q->tail + (size_t)i, q->n);
    struct pkt_buf*           buf  = bufs[i];
    void*                     phys = (void*)(buf->buf_addr_phy + offsetof(struct pkt_buf, data));
    tx_descriptor_t volatile* desc = &q->descriptors[idx];

    q->vaddr[idx] = (void*)buf;
    desc->physical_addr = (uint64_t)phys;
    desc->length        = buf->size;
    desc->EOP           = 1; // ????
    desc->IFCS          = 1;
    desc->IC            = 0;
    desc->RS            = 1; // yes pls
    desc->DEXT          = 0;
    desc->vle           = 0;
    desc->sta           = 0; // clear this, just in case
    desc->css           = 0;
    desc->vlan          = 0;

  }

  /* update tail, and our version of tail */
  q->tail = wrap_ring((size_t)q->tail + (size_t)cnt, q->n);
  set_reg32(dev->addr, E1000_TDT(queue_id), q->tail);

  return cnt;
}

// FIXME
// - ip checksum support
// - hardware timestamping (ptp)

