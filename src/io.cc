/*
 * LTE Subframe I/O Interface
 *
 * Copyright (C) 2015 Ettus Research LLC
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
 *
 * Author: Tom Tsou <tom.tsou@ettus.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>

#include "openphy/io.h"
#include "uhd.h"
#include "log.h"

extern "C" {
#include "slot.h"
}

#define LTE_SLOT_LEN            15360
#define LTE_SYM_LEN             2048

static struct uhd_dev *dev = NULL;
static int64_t subframe0_ts = 0;
static int prev_subframe = -1;

static int pss_adj = 0;
static int subframe_len = 0;
static int frame_len = 0;

void lte_radio_iface_reset()
{
	uhd_reset(dev);

	subframe0_ts = 0;
	prev_subframe = -1;
	pss_adj = 0;
	subframe_len = 0;
	frame_len = 0;
}

/*
 * Decimation factor for a given number of resource blocks
 *
 * Resource Blocks   Sample Rate   Decimation
 *         6           1.92 Msps       16
 *        15           3.84 Msps        8
 *        25           5.76 Msps        4
 *        50          11.52 Msps        2
 *        75          15.36 Msps        2
 *       100          23.04 Msps        1
 */
static int get_decim(int rbs)
{
	switch (rbs) {
	case 6:
		return 16;
	case 15:
		return 8;
	case 25:
		return 4;
	case 50:
		return 2;
	case 75:
		return 2;
	case 100:
		return 1;
	}

	return -1;
}

static int use_fft_1536(int rbs)
{
	switch (rbs) {
	case 6:
		return 0;
	case 15:
		return 0;
	case 25:
		return 1;
	case 50:
		return 1;
	case 75:
		return 0;
	case 100:
		return 1;
	}

	return -1;
}

#define TIMING_OFFSET_FUNC(N,LIM0,LIM1) \
	static int timing_offset_rb##N(int coarse, int fine) \
	{ \
		/* Sample 0 adjustment */ \
		if (!coarse) { \
			if (fine < LIM0) \
				return -1; \
			else \
				return 0; \
		} \
		/* Sample 1 adjustment */ \
		if (fine > LIM1) \
			return 1; \
		else \
			return 0; \
	} \

TIMING_OFFSET_FUNC(100,32,6)
TIMING_OFFSET_FUNC(75,30,9)
TIMING_OFFSET_FUNC(50,29,9)
TIMING_OFFSET_FUNC(25,26,13)
TIMING_OFFSET_FUNC(15,22,14)
TIMING_OFFSET_FUNC(6,22,16)

int (*fine_timing_offset)(int coarse, int fine) = NULL;

int lte_radio_iface_init(double freq, int chans, double gain,
			 int rbs, int ext, const std::string &args)
{
	dev = uhd_init(&subframe0_ts, freq, args, rbs, chans, gain, ext);
	if (!dev) {
		fprintf(stderr, "UHD failed to init\n");
		return -1;
	}

	int base_q = get_decim(rbs);
	if (use_fft_1536(rbs))
		pss_adj = 32 * 3 / 4 / base_q;
	else
		pss_adj = 32 / base_q;

	subframe_len = lte_subframe_len(rbs);
	frame_len = lte_frame_len(rbs);

	subframe0_ts += subframe_len;

	std::cout << "Initial timestamp" << subframe0_ts << std::endl;

	switch (rbs) {
	case 6:
		fine_timing_offset = timing_offset_rb6;
		break;
	case 15:
		fine_timing_offset = timing_offset_rb15;
		break;
	case 25:
		fine_timing_offset = timing_offset_rb25;
		break;
	case 50:
		fine_timing_offset = timing_offset_rb50;
		break;
	case 75:
		fine_timing_offset = timing_offset_rb75;
		break;
	case 100:
		fine_timing_offset = timing_offset_rb100;
		break;
	default:
		fprintf(stderr, "IO : Invalid resource block %i\n", rbs);
		return -1;
	}

	return 0;
}

static int comp_timing_offset(int coarse, int fine, int state)
{
	int adjust = 0;
	int pss_offset = LTE_N0_SLOT_LEN - LTE_N0_CP0_LEN - 1;

	if (fine == 9999)
		return -1;

	if (fine && ((coarse == 0) || (coarse == 1))) {
		fine += 32;
		adjust = fine_timing_offset(coarse, fine);
	} else if ((coarse >= -5) && (coarse <= 5)) {
		if (!state)
			adjust = coarse / 2;
		else
			adjust = coarse * pss_adj;
	} else if (coarse) {
		adjust = (coarse - pss_offset) * pss_adj;
	}

	return adjust;
}

int lte_read_subframe(std::vector<short *> &bufs,
		      int sf, int coarse, int fine, int state)
{
	int64_t ts;

	if (sf <= prev_subframe)
		subframe0_ts += frame_len;

	int offset = comp_timing_offset(coarse, fine, state);
	subframe0_ts += offset;

	ts = subframe0_ts + sf * subframe_len;

	while (ts + subframe_len > uhd_get_ts_high(dev))
		uhd_reload(dev);

	if (uhd_pull(dev, bufs, subframe_len, ts) < 0) {
		fprintf(stderr, "Failed to pull subframe data\n");
		std::cout << ts << ", " << subframe0_ts << ", " << sf << std::endl;
		exit(1);
		return -1;
	}

	prev_subframe = sf;

	return offset;
}

int lte_commit_subframe(std::vector<short *> &bufs)
{
	return uhd_commit(dev, bufs);
}

int lte_write_subframe(int16_t *buf, int len, int dec, int zero)
{
	return 0;
}

int lte_offset_freq(double offset)
{
	return uhd_shift(dev, offset);
}

int lte_offset_reset()
{
	return uhd_freq_reset(dev);
}
