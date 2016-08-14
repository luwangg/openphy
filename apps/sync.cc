/*
 * LTE UE Synchronization 
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
#include <cstdio>
#include <complex>
#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "../src/Resampler.h"
#include "queue.h"
#include "rrc.h"
#include "io_subframe.h"

extern "C" {
#include "openphy/lte.h"
#include "openphy/pdcch.h"
#include "openphy/gold.h"
#include "openphy/ref.h"
#include "openphy/sync.h"
#include "openphy/fft.h"
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

#define DETECT_THRSH			50.0f
#define AVG_FREQ			2
#define HIST_LEN			220

static int favg_cnt;
static float favg[AVG_FREQ];
static float fwid[AVG_FREQ];

struct lte_ref_map *pbch_map[2][4];

int gn_id_cell = -1;

/* Forward declarations */
void gen_sequences(int n_id_cell);
int gen_pdcch_refs(int n_id_cell, int rbs);

/*
 * PDSCH queues
 */
extern lte_buffer_q *pdsch_q;
extern lte_buffer_q *pdsch_return_q;
extern struct subframe_state subframe_table[10];

struct type_string {
        int type;
        const char *str;
};

static struct type_string sync_state_str[LTE_NUM_STATES] = {
	{ LTE_STATE_PSS_SYNC,    "PSS-Sync0" },
	{ LTE_STATE_PSS_SYNC2,   "PSS-Sync1" },
	{ LTE_STATE_SSS_SYNC,    "SSS-Sync" },
	{ LTE_STATE_PBCH_SYNC,   "PBCH-Sync" },
	{ LTE_STATE_PBCH,        "PBCH-Decode" },
	{ LTE_STATE_PDSCH_SYNC,  "PBCH-Sync" },
	{ LTE_STATE_PDSCH,       "PDSCH-Decode" },
};

/* Log state change */
static void log_state_chg(int cstate, int nstate)
{
	char sbuf[80];
	snprintf(sbuf, 80, "STATE : State change from %s to %s",
		 sync_state_str[cstate].str, sync_state_str[nstate].str);
	LOG_APP(sbuf);
}

/* Log PSS detection magnitude */
static void log_pss_mag(float mag, int offset)
{
	char sbuf[80];
	snprintf(sbuf, 80, "PSS   : PSS detected, "
		 "Magnitude %f, Timing offset %i",
		 mag, offset);
	LOG_SYNC(sbuf);
}

/* Log SSS frequency offset */
static void log_sss_comp_offset(float offset)
{
	char sbuf[80];
	snprintf(sbuf, 80, "SSS   : "
		 "Frequency offset %f Hz", offset);
	LOG_SYNC(sbuf);
}

static int gen_pbch_refs(int n_id_cell)
{
	for (int i = 0; i < 4; i++) {
		lte_free_ref_map(pbch_map[0][i]);
		lte_free_ref_map(pbch_map[1][i]);
	}

	pbch_map[0][0] = lte_gen_ref_map(n_id_cell, 0, 0, 0, 6);
	pbch_map[0][1] = lte_gen_ref_map(n_id_cell, 1, 0, 0, 6);
	pbch_map[0][2] = lte_gen_ref_map(n_id_cell, 0, 0, 4, 6);
	pbch_map[0][3] = lte_gen_ref_map(n_id_cell, 1, 0, 4, 6);

	pbch_map[1][0] = lte_gen_ref_map(n_id_cell, 0, 1, 0, 6);
	pbch_map[1][1] = lte_gen_ref_map(n_id_cell, 1, 1, 0, 6);
	pbch_map[1][2] = lte_gen_ref_map(n_id_cell, 0, 1, 4, 6);
	pbch_map[1][3] = lte_gen_ref_map(n_id_cell, 1, 1, 4, 6);

	return 0;
}

/* Time checks */
int lte_subframe_pss(struct lte_time *time)
{
	if ((time->subframe == 0) || (time->subframe == 5))
		return 1;
	else
		return 0;
}

int lte_subframe_sss(struct lte_time *time)
{
	return lte_subframe_pss(time);
}

int lte_subframe_pbch(struct lte_time *time)
{
	if (!time->subframe)
		return 1;
	else
		return 0;
}

int lte_subframe_pdcch(struct lte_time *time)
{
	if ((time->subframe < 0) || (time->subframe > 9))
		return 0;

	switch (subframe_table[time->subframe].enable) {
	case FRAME_ENABLE_OFF:
		return 0;
	case FRAME_ENABLE_ALL:
		return 1;
	case FRAME_ENABLE_EVEN:
		return !(time->frame % 2);
	case FRAME_ENABLE_ODD:
		return time->frame % 2;
	default:
		break;
	}

	return 0;
}

static bool preprocess_pdsch(struct io_subframe *subframe,
			     struct lte_buffer *lbuf, int adjust)
{
	int lbuf_len = subframe->len;

	for (size_t i = 0; i < subframe->chans; i++) {
		if (!lbuf->bufs[i])
			lbuf->bufs[i] = (short *) malloc(lbuf_len * 2 * sizeof(short));

		if (!subframe->delay(i, lbuf->bufs[i], lbuf_len, adjust))
			return false;
	}

	return true;
}

static bool pss_sync(struct lte_rx *rx, struct lte_sync *sync,
		     struct io_subframe *subframe, int adjust)
{
	int target = LTE_N0_SLOT_LEN - LTE_N0_CP0_LEN - 1;

	subframe->preprocess_pss();

	lte_pss_search(rx, &subframe->pss[0], subframe->chans, sync);
	if (sync->mag > 900) {
		if (sync->coarse < target)
			sync->coarse += LTE_N0_SLOT_LEN * 10;

		rx->sync.coarse = sync->coarse;
		rx->time.slot = 0;
		rx->time.subframe = 0;
		rx->sync.n_id_2 = sync->n_id_2;

		return true;
	}

	return false;
}

static int pss_sync2(struct lte_rx *rx, struct lte_sync *sync,
		     struct io_subframe *subframe)
{
	int target = LTE_N0_SLOT_LEN - LTE_N0_CP0_LEN - 1;
	int min = target - 4;
	int max = target + 4;
	int miss = 0;

	subframe->preprocess_pss();

	int rc = lte_pss_detect(rx, &subframe->pss[0], subframe->chans);
	if (rc != rx->sync.n_id_2) {
		miss++;
		LOG_PSS("Frequency domain detection failed");
	}

	lte_pss_sync(rx, &subframe->pss[0], subframe->chans,
		     sync, rx->sync.n_id_2);

	if ((sync->coarse > min) && (sync->coarse < max)) {
		rx->sync.coarse = sync->coarse - target;
		log_pss_mag(sync->mag, sync->coarse);
	} else {
		miss++;
		LOG_PSS("Time domain detection failed");
	}

	return miss;
}

static int sss_sync(struct lte_rx *rx, struct lte_sync *sync,
		    struct io_subframe *subframe, int cnt)
{
	int target = LTE_N0_SLOT_LEN - LTE_N0_CP0_LEN - 1;
	int min = target - 4;
	int max = target + 4;
	bool ready = false;
	int dn;
	int miss = 0;

	subframe->preprocess_pss();
;
	lte_pss_sync(rx, &subframe->pss[0], subframe->chans,
		     sync, rx->sync.n_id_2);

	if ((sync->coarse > min) && (sync->coarse < max))
		rx->sync.coarse = sync->coarse - target;
	else {
		miss++;
	}

	if (lte_pss_detect(rx, &subframe->pss[0],
			   subframe->chans) != rx->sync.n_id_2) {
		LOG_PSS_ARG("Frequency domain detection failed ", cnt);
		miss++;
	}

	dn = lte_sss_detect(rx, rx->sync.n_id_2,
			    &subframe->pss[0],
			    subframe->chans, sync);
	if (dn < 0) {
		LOG_SSS_ARG("No matching sequence found, ", cnt);
		miss++;
	} else if (!dn) {
		/* Averaging cycle */
	} else {
		ready = true;
	}

	if (ready)
		return dn;

	return -miss;
}

static void set_global_cell_id(int n_id_cell, int rbs)
{
	LOG_PBCH_ARG("Setting Cell ID to ", n_id_cell);
	gen_pbch_refs(n_id_cell);
	gen_pdcch_refs(n_id_cell, rbs);
	gen_sequences(n_id_cell);
	gn_id_cell = n_id_cell;
}

static int handle_pbch(struct lte_rx *rx, struct lte_time *ltime,
		       struct io_subframe *subframe, struct lte_mib *mib)
{
	int rc;
	struct lte_subframe *lsub[subframe->chans];

	for (int i = 0; i < subframe->chans; i++) {
		lsub[i] = lte_subframe_alloc(6, gn_id_cell, 2,
					     pbch_map[0], pbch_map[1]);

		subframe->preprocess_pbch(i, lsub[i]->samples);
	}

	rc = lte_decode_pbch(mib, lsub, subframe->chans);
	if (rc < 0) {
		LOG_PBCH_ERR("Internal error");
	} else if (rc == 0) {
		LOG_PBCH("MIB decoding failed");
	} else {
		ltime->frame = mib->fn;
	}

	for (size_t i = 0; i < subframe->chans; i++)
		lte_subframe_free(lsub[i]);

	if (rc < 0)
		return rc;
	else if (rc > 0)
		return 1;
	else
		return 0;
}

static bool pbch_sync(struct lte_rx *rx, struct lte_sync *sync,
		      struct io_subframe *subframe, int cnt)
{
	/* Why is this different from the PDSCH case? */
	int target = LTE_N0_SLOT_LEN - LTE_N0_CP0_LEN - 1;
	int min = target - 4;
	int max = target + 4;
	int n_id_2;
	bool found = false;

	subframe->preprocess_pss();

	lte_pss_sync(rx, &subframe->pss[0], subframe->chans,
		     sync, rx->sync.n_id_2);
	log_pss_mag(sync->mag, sync->coarse);

	n_id_2 = lte_pss_detect(rx, &subframe->pss[0], subframe->chans);
	if (n_id_2 == rx->sync.n_id_2) {
		if ((sync->coarse > min) && (sync->coarse < max))
			found = true;
	}

	if (!found) {
		LOG_PSS_ARG("PSS detection failed, ", cnt);
		return false;
	}

	rx->sync.coarse = sync->coarse - target;
	return true;
}

enum {
	SYNC_ERR_NONE,
	SYNC_ERR_PSS_TIME,
	SYNC_ERR_PSS_FREQ,
	SYNC_ERR_SSS,
};

static int pdsch_sync(struct lte_rx *rx,
		      struct lte_time *ltime,
		      struct io_subframe *subframe, int cnt)
{
	struct lte_sync sync;
	int target = LTE_N0_SLOT_LEN - LTE_N0_CP0_LEN - 1;
	int min = target - 4;
	int max = target + 4;

	subframe->preprocess_pss();

	lte_pss_fine_sync(rx, &subframe->pss[0], subframe->chans,
			  &sync, rx->sync.n_id_2);

	if ((sync.coarse <= min) || (sync.coarse >= max)) {
		return -SYNC_ERR_PSS_TIME;
	}

	rx->sync.coarse = sync.coarse - target;
	rx->sync.fine = sync.fine - 32;

	if (lte_pss_detect3(rx, &subframe->pss[0], subframe->chans) < 0) {
		return -SYNC_ERR_PSS_FREQ;
	}

	return 1;
}

int drive_common(struct lte_rx *rx, struct io_subframe *subframe,
		 struct lte_time *ltime, int adjust)
{
	struct lte_sync sync;
	static int pss_miss_cnt = 0;
	static int sss_miss_cnt = 0;

	switch (rx->state) {
	case LTE_STATE_PSS_SYNC:
		if (pss_sync(rx, &sync, subframe, adjust)) {
			lte_log_time(ltime);
			log_pss_mag(sync.mag, sync.coarse);
			rx->state = LTE_STATE_PSS_SYNC2;
			log_state_chg(LTE_STATE_PSS_SYNC, LTE_STATE_PSS_SYNC2);
		} else {
			rx->sync.fine = 9999;
		}
		break;
	case LTE_STATE_PSS_SYNC2:
		if (!ltime->subframe) {
			int miss = pss_sync2(rx, &sync, subframe);
			if (miss > 1)
				rx->state = LTE_STATE_PSS_SYNC;
			else
				rx->state = LTE_STATE_SSS_SYNC;

			lte_log_time(ltime);
			log_state_chg(LTE_STATE_PSS_SYNC2, rx->state);
		}
		break;
	case LTE_STATE_SSS_SYNC:
		if (!ltime->subframe) {
			int rc = sss_sync(rx, &sync, subframe, pss_miss_cnt);
			if (rc <= 0) {
				pss_miss_cnt += -rc;

				if (pss_miss_cnt >= 4) {
					rx->state = LTE_STATE_PSS_SYNC;
					log_state_chg(LTE_STATE_SSS_SYNC,
						      LTE_STATE_PSS_SYNC);
					lte_offset_reset();
					pss_miss_cnt = 0;
				}
				break;
			}

			lte_offset_freq(sync.f_offset);

			ltime->subframe = sync.dn;
			rx->sync.n_id_1 = sync.n_id_1;
			rx->sync.n_id_cell = sync.n_id_cell;
			rx->state = LTE_STATE_PBCH_SYNC;

			if (gn_id_cell != sync.n_id_cell)
				set_global_cell_id(sync.n_id_cell, rx->rbs);

			lte_log_time(ltime);
			log_state_chg(LTE_STATE_SSS_SYNC, LTE_STATE_PBCH_SYNC);
		}
		break;
	case LTE_STATE_PBCH_SYNC:
		if (!ltime->subframe) {
			if (!pbch_sync(rx, &sync, subframe, pss_miss_cnt))
				pss_miss_cnt++;

			if (pss_miss_cnt > 10) {
				rx->state = LTE_STATE_PSS_SYNC;
				pss_miss_cnt = 0;
				log_state_chg(LTE_STATE_PBCH_SYNC,
					      LTE_STATE_PSS_SYNC);
				lte_offset_reset();
				break;
			}
		}
		rx->state = LTE_STATE_PBCH;
		break;
	}

	return 0;
}

int drive_pbch(struct lte_rx *rx, struct io_subframe *subframe, int adjust)
{
	struct lte_time *ltime = &rx->time;
	int mib_found = 0;

	static struct lte_mib mib;
	static int pss_miss_cnt = 0;

	ltime->subframe = (ltime->subframe + 1) % 10;
	if (!ltime->subframe)
		ltime->frame = (ltime->frame + 1) % 1024;

	drive_common(rx, subframe, ltime, adjust);

	switch (rx->state) {
	case LTE_STATE_PBCH:
		if (lte_subframe_pbch(ltime)) {
			if (handle_pbch(rx, ltime, subframe, &mib) <= 0) {
				pss_miss_cnt++;
				if (pss_miss_cnt > 10) {
					rx->state = LTE_STATE_PSS_SYNC;
					pss_miss_cnt = 0;
					log_state_chg(LTE_STATE_PBCH_SYNC,
						      LTE_STATE_PSS_SYNC);
					lte_offset_reset();
				}
				break;
			}

			mib_found = mib.rbs;
		}
		rx->state = LTE_STATE_PBCH_SYNC;
	}

	subframe->update();

	return mib_found;
}

int drive_pdsch(struct lte_rx *rx, struct io_subframe *subframe, int adjust)
{
	struct lte_time *ltime = &rx->time;

	static struct lte_mib mib;
	static int pss_miss_cnt = 0;
	static int sss_miss_cnt = 0;

	ltime->subframe = (ltime->subframe + 1) % 10;
	if (!ltime->subframe)
		ltime->frame = (ltime->frame + 1) % 1024;

	drive_common(rx, subframe, ltime, adjust);

	switch (rx->state) {
	case LTE_STATE_PBCH:
		if (lte_subframe_pbch(ltime)) {
			int rc = handle_pbch(rx, ltime, subframe, &mib);
			if (rc <= 0) {
				pss_miss_cnt++;
				if (pss_miss_cnt > 10) {
					rx->state = LTE_STATE_PSS_SYNC;
					pss_miss_cnt = 0;
					log_state_chg(LTE_STATE_PBCH_SYNC,
						      LTE_STATE_PSS_SYNC);
					lte_offset_reset();
				}
				break;
			}

			rx->state = LTE_STATE_PDSCH_SYNC;
			pss_miss_cnt = 0;

			lte_log_time(ltime);
			log_state_chg(LTE_STATE_PBCH, LTE_STATE_PDSCH);
		}
		break;
	case LTE_STATE_PDSCH_SYNC:
		/* SSS must match so we only check timing/frequency on 0 */
		if (ltime->subframe == 5) {
			int rc = pdsch_sync(rx, ltime, subframe, pss_miss_cnt);
			if (rc == -SYNC_ERR_SSS) {
				LOG_SSS_ARG("Detected inconsistent SSS ",
					    sss_miss_cnt++);
				sss_miss_cnt++;
			} else if (rc < 0) {
				pss_miss_cnt++;
			}

			if ((pss_miss_cnt > 100) || (sss_miss_cnt > 5)) {
				rx->state = LTE_STATE_PSS_SYNC;
				pss_miss_cnt = 0;
				sss_miss_cnt = 0;
				log_state_chg(LTE_STATE_PDSCH_SYNC,
					      LTE_STATE_PSS_SYNC);
				lte_offset_reset();
				break;
			}
		}
	case LTE_STATE_PDSCH:
		if (lte_subframe_pdcch(ltime)) {
			lte_buffer *lbuf = pdsch_return_q->read();
			if (!lbuf) {
				LOG_ERR("SYNC  : Dropped frame");
				break;
			}

			if (lbuf->crc_pass) {
				pss_miss_cnt = 0;
				sss_miss_cnt = 0;
				lbuf->crc_pass = false;
			}

			lbuf->rbs = rx->rbs;
			lbuf->n_id_cell = gn_id_cell;
			lbuf->ng = mib.phich_ng;
			lbuf->tx_ants = mib.ant;
			lbuf->time.subframe = ltime->subframe;
			lbuf->time.frame = ltime->frame;

			preprocess_pdsch(subframe, lbuf, adjust);

			pdsch_q->write(lbuf);
		}
	}

	subframe->update();

	return 0;
}

/* External hacks */
void enable_prio(float prio);

static int pdsch_loop(struct lte_rx *rx, io_subframe *subframe)
{
	int cnt = 0;

	for (;;) {
		int shift = lte_read_subframe(subframe->raw, cnt,
					      rx->sync.coarse,
					      rx->sync.fine,
					      rx->state == LTE_STATE_PDSCH_SYNC);
		rx->sync.coarse = 0;
		rx->sync.fine = 0;

		if (drive_pdsch(rx, subframe, shift) < 0) {
			fprintf(stderr, "Drive: Fatal error\n");
			break;
		}

		subframe->reset();

		if (lte_commit_subframe(subframe->raw) < 0) {
			fprintf(stderr, "Drive: Fatal I/O error\n");
			break;
		}

		cnt = (cnt + 1) % 10;
	}
}

static int pbch_loop(struct lte_rx *rx, io_subframe *subframe)
{
	int rc, rbs, cnt = 0;

	for (;;) {
		int shift = lte_read_subframe(subframe->raw, cnt,
					      rx->sync.coarse,
					      rx->sync.fine, 0);
		rx->sync.coarse = 0;
		rx->sync.fine = 0;

		rc = drive_pbch(rx, subframe, shift);
		if (rc < 0) {
			fprintf(stderr, "Drive: Fatal loop error\n");
			break;
		} else if (rc > 0) {
			rbs = rc;
			break;
		}

		subframe->reset();

		if (lte_commit_subframe(subframe->raw) < 0) {
			fprintf(stderr, "Drive: Fatal I/O error\n");
			break;
		}

		cnt = (cnt + 1) % 10;
	}

	if (rc < 0)
		return rc;
	else
		return rbs;
}

int sync_loop(int rbs, int chans, bool mib)
{
	struct lte_rx *rx;
	struct io_subframe subframe(chans);

	if (mib)
		rbs = 6;

	subframe.init(rbs);

	rx = lte_init();
	rx->state = LTE_STATE_PSS_SYNC;
	rx->last_state = LTE_STATE_PSS_SYNC;
	rx->rbs = rbs;

	for (int i = 0; i < 4; i++) {
		pbch_map[0][i] = NULL;
		pbch_map[1][i] = NULL;
	}

	memset(favg, 0, sizeof(float) * AVG_FREQ);
	memset(fwid, 0, sizeof(float) * AVG_FREQ);

	enable_prio(0.7f);

	if (mib)
		rbs = pbch_loop(rx, & subframe);
	else
		pdsch_loop(rx, &subframe);

	lte_free(rx);

	gn_id_cell = -1;

	if (mib)
		return rbs;
	else
		return 0;
}
