/*
 * LTE MIMO-OFDM UE Receiver
 *
 * Copyright (C) 2015 Ettus Research LLC
 * Author Tom Tsou <tom.tsou@ettus.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <vector>
#include <thread>
#include <cstdio>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <complex>

#include "../src/Resampler.h"
#include "queue.h"

extern "C" {
#include "openphy/lte.h"
#include "openphy/pdcch.h"
#include "openphy/gold.h"
#include "openphy/ref.h"
#include "openphy/sync.h"
#include "openphy/fft.h"
#include "../src/pcfich.h"
#include "../src/pdsch.h"
#include "../src/pbch.h"
#include "../src/scramble.h"
#include "../src/slot.h"
#include "../src/si.h"
#include "../src/dci.h"
#include "../src/subframe.h"
#include "../src/ofdm.h"
#include "../src/log.h"
#include "../src/pdsch_block.h"
#include "../src/sigproc/convert.h"
}

#include "openphy/io.h"

/*
 * Number of LTE subframe buffers passed between PDSCH processing threads
 */
#define NUM_RECV_SUBFRAMES		64

uint16_t g_rnti;

/* PDSCH queue */
lte_buffer_q *pdsch_q = NULL;
lte_buffer_q *pdsch_return_q = NULL;

/* Externals */
int sync_loop(int q, int chans, bool mib);
int pdsch_loop();
void rrc_loop();

void enable_prio(float prio)
{
	int min, max;
	struct sched_param param;

	if (prio > 1.0)
		prio = 1.0;
	if (prio < 0)
		prio = 0.0;

	max = sched_get_priority_max(SCHED_FIFO);
	min = sched_get_priority_min(SCHED_FIFO);
	sched_getparam(0, &param);
	param.sched_priority = (int) ((max - min) * prio + min);
	sched_setscheduler(0, SCHED_FIFO, &param);
}

struct lte_config {
	std::string args;
	double freq;
	double gain;
	int chans;
	int rbs;
	int threads;
	uint16_t rnti;
	bool extref;
};

static void print_help()
{
        fprintf(stdout, "\nOptions:\n"
		"  -h    This text\n"
		"  -a    UHD device args\n"
		"  -c    Number of receive channels (1 or 2)\n"
		"  -f    Downlink frequency\n"
		"  -g    RF receive gain\n"
		"  -j    Number of PDSCH decoding threads (default = 1)\n"
		"  -b    Number of LTE resource blocks (default = auto)\n"
		"  -r    LTE RNTI (default = 0xFFFF)\n"
		"  -x    Enable external device reference (default = off)\n\n");
}

static void print_config(struct lte_config *config)
{
	fprintf(stdout,
		"Config:\n"
		"    Device args.............. \"%s\"\n"
		"    Downlink frequency....... %.3f MHz\n"
		"    Receive gain............. %.2f dB\n"
		"    Receive antennas......... %i\n"
		"    External reference....... %s\n"
		"    PDSCH decoding threads... %i\n"
		"    LTE resource blocks...... %i\n"
		"    LTE RNTI................. 0x%04x\n"
		"\n",
		config->args.c_str(),
		config->freq / 1e6,
		config->gain,
		config->chans,
		config->extref ? "On" : "Off",
		config->threads,
		config->rbs,
		config->rnti);
}

static bool valid_rbs(int rbs)
{
	switch (rbs) {
	case 6:
	case 15:
	case 25:
	case 50:
	case 75:
	case 100:
		return true;
	}

	return false;
}

static int handle_options(int argc, char **argv, struct lte_config *config)
{
	int option;

	config->freq = -1.0;
	config->gain = 0.0;
	config->chans = 1;
	config->rbs = 0;
	config->threads = 1;
	config->rnti = 0xffff;
	config->extref = false;

	while ((option = getopt(argc, argv, "ha:c:f:g:j:b:r:x")) != -1) {
		switch (option) {
		case 'h':
			print_help();
			return -1;
		case 'a':
			config->args = optarg;
			break;
		case 'c':
			config->chans = atoi(optarg);
			if ((config->chans != 1) && (config->chans != 2)) {
				printf("Invalid number of channels\n");
				return -1;
			}
			break;
		case 'f':
			config->freq = atof(optarg);
			break;
		case 'g':
			config->gain = atof(optarg);
			break;
		case 'j':
			config->threads = atoi(optarg);
			break;
		case 'b':
			config->rbs = atoi(optarg);
			break;
		case 'r':
			config->rnti = atoi(optarg);
			break;
		case 'x':
			config->extref = true;
			break;
		default:
			print_help();
			return -1;
		}
	}

	if (config->threads < 1) {
		printf("\nInvalid number of PDSCH decoding threads %i\n",
		       config->threads);
		return -1;
	}

	if (config->freq < 0.0) {
		print_help();
		printf("\nPlease specify downlink frequency\n");
		return -1;
	}

	if (config->rbs && !valid_rbs(config->rbs)) {
		print_help();
		printf("\nPlease specify valid number of resource blocks\n\n");
		printf("    LTE bandwidth      Resource Blocks\n");
		printf("       1.4 MHz                 6\n");
		printf("         3 MHz                15\n");
		printf("         5 MHz                25\n");
		printf("        10 MHz                50\n");
		printf("        15 MHz                75\n");
		printf("        20 MHz               100\n\n");

		return -1;
	}

	return 0;
}

int mib_search(int chans);

int main(int argc, char **argv)
{
	struct lte_config config;
	struct lte_buffer *buf;
	std::vector<std::thread> threads;

	if (handle_options(argc, argv, &config) < 0)
		return -1;

	g_rnti = config.rnti;

	print_config(&config);

	if (!config.rbs) {
		if (lte_radio_iface_init(config.freq, config.chans,
					 config.gain, 6, config.extref,
					 config.args) < 0) {
			fprintf(stderr, "Radio: Failed to initialize\n");
			return -1;
		}

		config.rbs = sync_loop(0, config.chans, true);
		lte_radio_iface_reset();
	}

	if (lte_radio_iface_init(config.freq, config.chans,
				 config.gain, config.rbs,
				 config.extref, config.args) < 0) {
		fprintf(stderr, "Radio: Failed to initialize\n");
		return -1;
	}

	pdsch_q = new lte_buffer_q();
	pdsch_return_q = new lte_buffer_q();

	/* Prime the interthread queue */
	for (int i = 0; i < NUM_RECV_SUBFRAMES; i++)
		pdsch_return_q->write(new lte_buffer(config.chans));

	/* Launch threads */
	threads.push_back(std::thread(rrc_loop));

	for (int i = 0; i < config.threads; i++)
		threads.push_back(std::thread(pdsch_loop));

	sync_loop(config.rbs, config.chans, false);

	for (auto &thread : threads)
		thread.join();

	delete pdsch_q;
	delete pdsch_return_q;

	return 0;
}
