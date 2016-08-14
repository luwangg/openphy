#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
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

#define LTE_PDCCH_MAX_BITS		6269

int8_t pdcch_scram_seq[10][LTE_PDCCH_MAX_BITS];
int8_t pcfich_scram_seq[10][32];
struct lte_ref_map *pdcch_map[20][4];

extern lte_buffer_q *pdsch_q;
extern lte_buffer_q *pdsch_return_q;

#define AVG_FREQ2			200
static int favg_cnt2;
static float favg2[AVG_FREQ2];

extern uint16_t g_rnti;
extern int gn_id_cell;

static void log_ofdm_comp_offset(float offset)
{
	char sbuf[80];
	snprintf(sbuf, 80, "REF   : "
		 "Frequency offset %f Hz", offset);
	LOG_SYNC(sbuf);
}

void gen_sequences(int n_id_cell)
{
	unsigned c_init = (unsigned) n_id_cell;

	for (int i = 0; i < 10; i++) {
		c_init = (2 * i / 2 + 1) * (2 * n_id_cell + 1) * (1 << 9) + n_id_cell;
		lte_pbch_gen_scrambler(c_init, pcfich_scram_seq[i], 32);

		c_init = (2 * i / 2) * (1 << 9) + n_id_cell;
		lte_pdcch_gen_scrambler(c_init, pdcch_scram_seq[i],
					LTE_PDCCH_MAX_BITS);
	}

}

int gen_pdcch_refs(int n_id_cell, int rbs)
{
	for (int i = 0; i < 20; i++) {
		for (int n = 0; n < 4; n++)
			lte_free_ref_map(pdcch_map[i][n]);
	}

	for (int i = 0; i < 20; i++) {
		pdcch_map[i][0] = lte_gen_ref_map(n_id_cell, 0, i, 0, rbs);
		pdcch_map[i][1] = lte_gen_ref_map(n_id_cell, 1, i, 0, rbs);
		pdcch_map[i][2] = lte_gen_ref_map(n_id_cell, 0, i, 4, rbs);
		pdcch_map[i][3] = lte_gen_ref_map(n_id_cell, 1, i, 4, rbs);
	}

	return 0;
}

static int preprocess_pdcch(short *buf, struct cxvec *vec)
{
	float scale = 1 / 32000.0f;
	int len = cxvec_len(vec);

	convert_short_float((float *) cxvec_data(vec), buf, 2 * len, scale);

	return 0;
}

int pdsch_loop()
{
	int i, rc;
	lte_buffer *lbuf;
	struct cxvec *samples;

	struct lte_time time;
	struct lte_dci dci;
	struct lte_pcfich_info info;

	struct lte_pdsch_blk *pdsch_blk = lte_pdsch_blk_alloc();

	bool cell_id_change;

	for (int i = 0; i < 4; i++) {
		for (int n = 0; n < 20; n++)
			pdcch_map[n][i] = NULL;
	}

	for (;;) {
		lbuf = pdsch_q->read();
		if (!lbuf) {
			usleep(10);
			continue;
		}

		time.subframe = lbuf->time.subframe;
		time.frame = lbuf->time.frame;

		/* Force subframe reallocation on Cell ID change */
		if ((lbuf->rx_ants == 2) && (!lbuf->subframe[0] || !lbuf->subframe[1])) {
				cell_id_change = true;
		} else if ((lbuf->rx_ants == 1) && !lbuf->subframe[0]) {
				cell_id_change = true;
		} else if ((lbuf->n_id_cell != lbuf->subframe[0]->cell_id))
			cell_id_change = true;
		else
			cell_id_change = false;

		for (i = 0; i < lbuf->rx_ants; i++) {
			if (cell_id_change) {
				lte_subframe_free(lbuf->subframe[i]);
				lbuf->subframe[i] = NULL;
			}

			if (!lbuf->subframe[i]) {
				lbuf->subframe[i] = lte_subframe_alloc(
					lbuf->rbs, lbuf->n_id_cell, lbuf->tx_ants, 
					pdcch_map[time.subframe * 2 + 0],
					pdcch_map[time.subframe * 2 + 1]);
			} else {
				rc = lte_subframe_reset(lbuf->subframe[i],
					pdcch_map[time.subframe * 2 + 0],
					pdcch_map[time.subframe * 2 + 1]);
				if (rc < 0)
					fprintf(stderr, "PDSCH: Subframe reset failed\n");
			}

			preprocess_pdcch(lbuf->bufs[i], lbuf->subframe[i]->samples);

			lbuf->subframe[i]->time.subframe = time.subframe;
		}
#if 1
		rc = lte_decode_pcfich(&info, &lbuf->subframe[0],
				       lbuf->n_id_cell,
				       pcfich_scram_seq[time.subframe], lbuf->rx_ants);

		float offset = 0.0f;
		float offsets[lbuf->rx_ants];

		for (i = 0; i < lbuf->rx_ants; i++) {
			offsets[i] = lte_ofdm_offset(lbuf->subframe[i]);
			offset += offsets[i];
		}

		offset /= (float) lbuf->rx_ants;

		favg2[favg_cnt2++] = offset;
		if (favg_cnt2 >= AVG_FREQ2) {
			favg_cnt2 = 0;

			float avg = 0.0f;

			for (int i = 0; i < AVG_FREQ2; i++)
				avg += favg2[i];

			avg = avg / (float) AVG_FREQ2;

			log_ofdm_comp_offset(avg);
			lte_offset_freq(avg);
		}
#if 1
		if ((rc > 0) && (info.cfi > 0) && (info.cfi < 4)) {
			int num_dci;
			num_dci = lte_decode_pdcch(lbuf->subframe.data(),
					     lbuf->rx_ants,
					     info.cfi,
					     lbuf->n_id_cell,
					     lbuf->ng,
					     g_rnti,
					     pdcch_scram_seq[time.subframe]);
#if 1
			for (int i = 0; i < num_dci; i++) {
				lte_log_time(&time);
				rc = lte_decode_pdsch(&lbuf->subframe[0],
						      lbuf->rx_ants, pdsch_blk,
						      info.cfi, i, &time);
				if (rc > 0)
					lbuf->crc_pass = true;
			}
#endif
		}
#endif
#endif
		pdsch_return_q->write(lbuf);
	}

	lte_pdsch_blk_free(pdsch_blk);

	return 0;
}


