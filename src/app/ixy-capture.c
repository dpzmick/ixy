#include <stdio.h>
#include <unistd.h>

#include "stats.h"
#include "log.h"
#include "memory.h"
#include "driver/device.h"

const int BATCH_SIZE = 32;

static void recv(struct ixy_device* rx_dev, uint16_t rx_queue) {
	struct pkt_buf* bufs[BATCH_SIZE];
	uint32_t num_rx = ixy_rx_batch(rx_dev, rx_queue, bufs, BATCH_SIZE);
	if (num_rx > 0) {
		for (uint32_t i = num_rx; i < num_rx; i++) {
			pkt_buf_free(bufs[i]);
		}
	}
}

int main(int argc, char* argv[]) {
	if (argc != 2) {
		printf("Usage: %s <pci bus id2>\n", argv[0]);
		return 1;
	}

	struct ixy_device* dev = ixy_init(argv[1], 1, 1);

	uint64_t last_stats_printed = monotonic_time();
	struct device_stats stats, stats_old;
	stats_init(&stats_old, dev);
	stats_init(&stats, dev);

	uint64_t counter = 0;
	while (true) {
		recv(dev, 0);

		// don't poll the time unnecessarily
		if ((counter++ & 0xFFF) == 0) {
			uint64_t time = monotonic_time();
			if (time - last_stats_printed > 1000 * 1000 * 1000) {
				// every second
				ixy_read_stats(dev, &stats);
				print_stats_diff(&stats, &stats_old, time - last_stats_printed);
				stats_old = stats;
				last_stats_printed = time;
			}
		}
	}
}

