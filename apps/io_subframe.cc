/*
 * LTE I/O Subframe 
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

#include <string.h>
#include "io_subframe.h"

extern "C" {
#include "../src/slot.h"
#include "../src/sigproc/convert.h"
#include "openphy/sigvec.h"
}

#include "../src/Resampler.h"

#define OFFSET_LIMIT	64

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
	case 25:
	case 50:
	case 100:
		return 1;
	}

	return 0;
}

io_subframe::io_subframe(size_t chans)
	 : raw(chans, NULL), base(chans, NULL), pss(chans, NULL),
	   pbch(chans, NULL), convert_on(false), pss_on(false),
	   history(chans, NULL), pss_resampler(chans, NULL),
	   pbch_resampler(chans, NULL)
{
	this->chans = chans;
}

io_subframe::~io_subframe()
{
	for (size_t i = 0; i < chans; i++) {
		cxvec_free(base[i]);
		cxvec_free(pss[i]);
		cxvec_free(pbch[i]);

		delete[] history[i];
		delete pss_resampler[i];
		delete pbch_resampler[i];
	}
}

bool io_subframe::init(size_t rbs, size_t taps)
{
	unsigned flags = CXVEC_FLG_FFT_ALIGN;

	/* Subframe lengths */
	int pdsch_len = lte_subframe_len(rbs);
	int pss_len = LTE_BASE_SUBFRAME_LEN / 32;
	int pbch_len = lte_subframe_len(6);

	int p = 1;
	int base_q = get_decim(rbs);
	if (base_q < 0)
		return false;

	int pss_q = use_fft_1536(rbs) ? 32 * 3 / 4 / base_q : 32 / base_q;
	int pbch_q = pss_q / 2;

	for (size_t i = 0; i < chans; i++) {
		history[i] = new short[2 * (taps / 2 + OFFSET_LIMIT)];
		base[i] = cxvec_alloc(pdsch_len, taps, 0, NULL, flags);
		pss[i] = cxvec_alloc(pss_len, 0, 0, NULL, flags);
		pbch[i] = cxvec_alloc(pbch_len, 0, 0, NULL, flags);

		pss_resampler[i] = new Resampler(p, pss_q, taps, 0, 1.0);
		pbch_resampler[i] = new Resampler(p, pbch_q, taps, 0, 1.0);
		pss_resampler[i]->init();
		pbch_resampler[i]->init();
	}

	this->hlen = taps / 2 + OFFSET_LIMIT;
	this->len = pdsch_len;
	this->taps = taps;

	return true;
}

/* Converters and amplitude scaling */
void io_subframe::convert(size_t start, size_t len)
{
	float scale = 1.0 / 127.0;

	for (size_t i = 0; i < chans; i++) {
		short *_raw = &raw[i][2 * start];
		float *_base = (float *) cxvec_data(base[i]) + 2 * start;

		convert_short_float(_base, _raw, 2 * len, scale);
	}
}

bool io_subframe::convert()
{
	if (convert_on)
		return false;

	convert(0, this->len);

	convert_on = true;

	return true;
}

void io_subframe::reset()
{
	convert_on = false;
	pss_on = false;
}

bool io_subframe::preprocess_pss()
{
	if (pss_on)
		return false;

	if (!convert_on)
		convert();

	for (size_t i = 0; i < chans; i++)
		pss_resampler[i]->rotate(base[i], pss[i]);

	pss_on = true;

	return true;
}

bool io_subframe::preprocess_pbch(size_t chan, struct cxvec *vec)
{
	if (!convert_on)
		convert();

	pbch_resampler[chan]->rotate(base[chan], vec);

	return 0;
}

bool io_subframe::update()
{
	if (!convert_on)
		convert(this->len - taps, taps);

	if (!pss_on) {
		for (size_t i = 0; i < chans; i++)
			pss_resampler[i]->update(base[i]);
	}

	for (size_t i = 0; i < chans; i++) {
		int index = 2 * (this->len - this->hlen);
		int size = this->hlen * 2 * sizeof(short);

		memcpy(history[i], &raw[i][index], size);
		pbch_resampler[i]->update(base[i]);
	}

	return true;
}

static void interp_sample(short *buf, int index)
{
	short a, b, c, d;

	a = buf[2 * (index - 1) + 0];
	b = buf[2 * (index - 1) + 1];
	c = buf[2 * (index + 1) + 0];
	d = buf[2 * (index + 1) + 1];

	buf[2 * index + 0] = (a + c) / 2;
	buf[2 * index + 1] = (b + d) / 2;
}

bool io_subframe::delay(size_t chan, short *buf, size_t len, int offset)
{
	int head, body, delay = taps / 2;
	short *_history = history[chan] + 2 * OFFSET_LIMIT;
	short *_buf = buf;

	if (len > this->len)
		return false;

	if (offset < -OFFSET_LIMIT) {
		int gap = -offset - OFFSET_LIMIT;
		head = delay - gap;
		_buf = buf + 2 * gap;
	} else if (offset > 0) {
		head = delay - offset;
	} else {
		head = delay;
	}

	/* Set history in the head location */
	memcpy(_buf, _history + 2 * offset, head * 2 * sizeof(short));

	/* Set the main body */
	body = this->len - delay;
	memcpy(buf + 2 * delay, raw[chan], body * 2 * sizeof(short));

	if (offset == 1)
		interp_sample(buf, delay - offset);

	return true;
}
