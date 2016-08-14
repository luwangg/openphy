/*
 * Polyphase Rational Resampler 
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

#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#include <iostream>
#include <complex>

#include "Resampler.h"

extern "C" {
#include "openphy/sigproc.h"
}

#define MAX_OUTPUT_LEN		(4096 * 8)

typedef std::complex<float> _complex;

static _complex *_cxvec_data(struct cxvec *vec)
{
        return (_complex *) cxvec_data(vec);
}

bool Resampler::initFilters()
{
	int i, n;
	int cutoff;
	int protoLen = mP * mFiltLen;

	float sum = 0.0f;
	float scale = 0.0f;
	float midpt = protoLen / 2;

	/* 
	 * Allocate partition filters and the temporary prototype filter
	 * according to numerator of the rational rate. Coefficients are
	 * real only and must be 16-byte memory aligned for SSE usage.
	 */
	int flags = CXVEC_FLG_REAL_ONLY | CXVEC_FLG_MEM_ALIGN;

	float proto[protoLen];

	partitions = (struct cxvec **) malloc(sizeof(struct cxvec *) * mP);
	if (!partitions)
		return false;

	for (i = 0; i < mP; i++)
		partitions[i] = cxvec_alloc(mFiltLen, 0, 0, NULL, flags);

	/* 
	 * Generate the prototype filter with a Blackman-harris window.
	 * Scale coefficients with DC filter gain set to unity divided
	 * by the number of filter partitions. 
	 */
	float a0 = 0.35875;
	float a1 = 0.48829;
	float a2 = 0.14128;
	float a3 = 0.01168;

	if (mP > mQ)
		cutoff = mP;
	else
		cutoff = mQ;

	for (i = 0; i < protoLen; i++) {
		proto[i] = cxvec_sinc(((float) i - midpt) / cutoff / mFactor);
		proto[i] *= a0 -
			    a1 * cos(2 * M_PI * i / (protoLen - 1)) +
			    a2 * cos(4 * M_PI * i / (protoLen - 1)) -
			    a3 * cos(6 * M_PI * i / (protoLen - 1));

		sum += proto[i];
	}
	scale = mP / sum;

	/* 
	 * Populate partition filters and reverse the coefficients per
	 * convolution requirements.
	 */
	for (i = 0; i < mFiltLen; i++) {
		for (n = 0; n < mP; n++) {
			_cxvec_data(partitions[n])[i] = proto[i * mP + n] * scale;
		}
	}

	for (i = 0; i < mP; i++)
		cxvec_rvrs(partitions[i], partitions[i]);

	return true;
}

void Resampler::releaseFilters()
{
	int i;

	for (i = 0; i < mP; i++) {
		cxvec_free(partitions[i]);
	}

	free(partitions);
}

static bool check_vec_len(struct cxvec *in,
			 struct cxvec *out,
			 int p, int q)
{
	if (cxvec_len(in) % q) {
		std::cout << "Invalid input length " << cxvec_len(in)
			 <<  " is not multiple of " << q << std::endl;
		return false;
	}

	if (!out)
		return true;

	if (cxvec_len(out) % p) {
		std::cout << "Invalid output length " << cxvec_len(out)
			 <<  " is not multiple of " << p << std::endl;
		return false;
	}

	if ((cxvec_len(in) / q) != (cxvec_len(out) / p)) {
		std::cout << "Input/output block length mismatch" << std::endl;
		return false;
	}

	if (cxvec_len(out) > MAX_OUTPUT_LEN) {
		std::cout << "Block length of " << cxvec_len(out) << " exceeds max" << std::endl;
		return false;
	}

	return true;
}

void Resampler::computePath()
{
	int i;

	for (i = 0; i < MAX_OUTPUT_LEN; i++) {
		inputIndex[i] = (mQ * i) / mP;
		outputPath[i] = (mQ * i) % mP;
	}
}

int Resampler::rotate(struct cxvec *in, struct cxvec *out, int len)
{
	int n, path;
	int hlen = cxvec_len(history);
	int ilen = cxvec_len(in);

	if (!check_vec_len(in, out, mP, mQ))
		return -1;

	/* Insert history */
	memcpy(_cxvec_data(in) - hlen,
	       _cxvec_data(history), hlen * 2 * sizeof(float));

	/* Generate output from precomputed input/output paths */
	for (int i = 0; i < len; i++) {
		n = inputIndex[i];
		path = outputPath[i];

		single_convolve((float *) &_cxvec_data(in)[n - mDelay],
				partitions[path],
				(float *) &_cxvec_data(out)[i]);
	}

	/* Save history */
	memcpy(_cxvec_data(history),
	       &_cxvec_data(in)[ilen - hlen],
	       hlen * 2 * sizeof(float));

	return cxvec_len(out);
}

int Resampler::update(struct cxvec *in)
{
	int ilen = cxvec_len(in);
	int hlen = cxvec_len(history);

	if (!check_vec_len(in, NULL, mP, mQ))
		return -1;

	/* Save history */
	memcpy(_cxvec_data(history),
	       &_cxvec_data(in)[ilen - hlen],
	       hlen * 2 * sizeof(float));

	return 0;
}

int Resampler::rotate(struct cxvec *in, struct cxvec *out)
{
	return rotate(in, out, cxvec_len(out));
}

bool Resampler::init()
{
	/* Filterbank internals */
	int rc = initFilters();
	if (rc < 0) {
		std::cout << "Failed to create filterbank" << std::endl;
		return false;
	}

	history = cxvec_alloc_simple(mFiltLen + mDelay);
	cxvec_reset(history);

	/* Precompute filterbank path */
	inputIndex = (int *) malloc(sizeof(int) * MAX_OUTPUT_LEN);
	outputPath = (int *) malloc(sizeof(int) * MAX_OUTPUT_LEN);
	computePath();

	return true;
}

Resampler::Resampler(int wP, int wQ, int wFiltLen,
		     int wDelay, float wFactor)
	: mP(wP), mQ(wQ), mFiltLen(wFiltLen),
	  mDelay(wDelay), mFactor(wFactor), inputIndex(NULL), outputPath(NULL)
{
}

Resampler::~Resampler()
{
	releaseFilters();

	cxvec_free(history);

	free(inputIndex);
	free(outputPath);
}
