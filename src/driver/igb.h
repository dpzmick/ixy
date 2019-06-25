#pragma once

#include <stdbool.h>
#include "stats.h"
#include "memory.h"

struct igb_device {
	struct ixy_device ixy;
	uint8_t* addr;
	void*    rx_queues;
	void*    tx_queues;
};

struct ixy_device*
igb_init( const char* pci_addr,
          uint16_t    rx_queues,
          uint16_t    tx_queues );

void
igb_read_stats( struct ixy_device* dev,
                struct device_stats* stats );

uint32_t
igb_tx_batch( struct ixy_device* dev,
              uint16_t           queue_id,
              struct pkt_buf*    bufs[],
              uint32_t           num_bufs );

uint32_t
igb_rx_batch( struct ixy_device* dev,
              uint16_t           queue_id,
              struct pkt_buf*    bufs[],
              uint32_t           num_bufs );

// uint32_t ixgbe_get_link_speed(const struct ixy_device* dev);
// void ixgbe_set_promisc(struct ixy_device* dev, bool enabled);
