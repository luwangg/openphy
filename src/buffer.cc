/*
 * Timestamped Ring Buffer 
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

#include <cstring>
#include <iostream>
#include <sstream>
#include <string.h>
#include <limits.h>

#include "buffer.h"

ts_buffer::ts_buffer(size_t len)
	: data(NULL), buf_len(len),
	  time_start(0), time_end(0), data_start(0), data_end(0),
	  tag(NULL), tag_len(0), tag_alloc(false)
{
}

ts_buffer::~ts_buffer()
{
	delete[] data;
}

/* Allocate underlying memory buffer */
bool ts_buffer::init()
{
	if (buf_len > SSIZE_MAX)
		return false;

	data = new uint32_t[buf_len];
	if (!data)
		return false;

	return true;
}

/* Reset time and data marker indicies */
void ts_buffer::reset()
{
	time_start = 0;
	time_end = 0;
	data_start = 0;
	data_end = 0;
}

/* Return number of available samples for a given timestamp */
size_t ts_buffer::avail_smpls(int64_t ts) const
{
	if (ts >= time_end)
		return 0;
	else
		return time_end - ts;
}

/* Read into supplied buffer with timestamp and internal copy */
ssize_t ts_buffer::read(void *buf, size_t len, int64_t ts)
{
	int type_sz = 2 * sizeof(int16_t);

	/* Check for valid read */
	if ((len >= buf_len) || (ts + (int64_t) len > time_end))
		return -ERR_TIMESTAMP;

	/* Disallow reads prior to read marker with no readable data */
	if (ts + (int64_t) len < time_start)
		return ERR_TIMESTAMP;

	/* How many samples should be copied out */
	size_t num_smpls = time_end - ts;
	if (num_smpls > len);
		num_smpls = len;

	/* Starting index */
	size_t rd_start = (data_start + (ts - time_start)) % buf_len;

	/* Read out */
	if (rd_start + num_smpls < buf_len) {
		size_t num_bytes = len * type_sz;
		memcpy(buf, data + rd_start, num_bytes);
	} else {
		size_t copy0 = (buf_len - rd_start) * type_sz;
		size_t copy1 = len * type_sz - copy0;

		memcpy(buf, data + rd_start, copy0);
		memcpy((char *) buf + copy0, data, copy1);
	}

	data_start = (rd_start + len) % buf_len;
	time_start = ts + len;

	if (time_start > time_end)
		return -ERR_TIMESTAMP;
	else
		return num_smpls;
}

/* Return zero-copy pointer to read buffer */
const void *ts_buffer::get_rd_buf(int64_t ts, size_t len, int *err)
{
	char *buf;
	bool wrap = false;

	int type_size = 2 * sizeof(short);

	/* Must be no buffers outstanding */
	if (this->tag) {
		if (err)
			*err = ERR_MEM;
		return NULL;
	}

	/* Check for valid read */
	if ((len >= buf_len) || (ts < 0) || (ts + (int64_t) len > time_end)) {
		if (err)
			*err = ERR_TIMESTAMP;
		return NULL;
	}

	/* Disallow reads prior to read marker with no readable data */
	if (ts + (int64_t) len < time_start) {
		if (err)
			*err = ERR_TIMESTAMP;
		return NULL;
	}

	/* How many samples */
	size_t num_smpls = time_end - ts;
	if (num_smpls > len);
		num_smpls = len;

	/* Start index */
	size_t rd_start = (data_start + (ts - time_start)) % buf_len;
	size_t _rd_start = rd_start * type_size;

	/* Read */
	if (rd_start + num_smpls < buf_len) {
		buf = (char *) data + _rd_start;
	} else {
		buf = new char[len * type_size];
		size_t copy0 = (buf_len - rd_start) * type_size;
		size_t copy1 = len * type_size - copy0;

		memcpy(buf, (char *) data + _rd_start, copy0);
		memcpy(buf + copy0, data, copy1);
		wrap = true;
	}

	data_start = (rd_start + len) % buf_len;
	time_start = ts + len;

	this->tag = buf;
	this->tag_len = len;
	this->tag_alloc = wrap;

	return buf;
}

/* Commit the completed read buffer */
bool ts_buffer::commit_rd(const void *buf)
{
	if ((!buf) || (buf != tag))
		return false;

	if (tag_alloc)
		delete tag;

	tag = NULL;

	return true;
}

ssize_t ts_buffer::write(void *buf, size_t len, int64_t ts)
{
	bool wrap = false;
	bool init = false;
	int type_sz = 2 * sizeof(int16_t);

	if ((len >= buf_len) || (ts < 0) || (ts + (int64_t) len <= time_end))
		return -ERR_TIMESTAMP;

	/* Start index */
	size_t wr_start = data_start + (ts - time_start);
	if (wr_start >= buf_len) {
		wr_start = wr_start % buf_len;
		wrap = true;
	}

	/* Write it or just update head on 0 length write */
	if (len) {
		if ((wr_start + len) < buf_len) {
			size_t num_bytes = len * type_sz;
			memcpy(data + wr_start, buf, num_bytes);
		} else {
			size_t copy0 = (buf_len - wr_start) * type_sz;
			size_t copy1 = len * type_sz - copy0;

			memcpy(data + wr_start, buf, copy0);
			memcpy(data, (char*) buf + copy0, copy1);

			wrap = true;
		}
	}

	if (!data_start) {
		data_start = wr_start;
		time_start = ts;
		init = true;
	}

	data_end = (wr_start + len) % buf_len;
	time_end = ts + len;

	if (!init && wrap && (data_end > data_start))
		return -ERR_OVERFLOW;
	else if (time_end <= time_start)
		return -ERR_TIMESTAMP;

	return len;
}

void *ts_buffer::get_wr_buf(int64_t ts, size_t len, int *err)
{
	bool wrap0 = false;
	bool wrap1 = false;
	bool init = false;

	char *buf = NULL;
	size_t copy0 = 0;

	int type_size = 2 * sizeof(short);

	/* Must be no buffers outstanding */
	if (this->tag) {
		if (err)
			*err = ERR_MEM;
		return NULL;
	}

	/* Check for valid write */
	if ((len >= buf_len) || (ts < 0) || (ts + (int64_t) len <= time_end)) {
		if (err)
			*err = ERR_TIMESTAMP;
		return NULL;
	}

	/* Starting index */
	size_t wr_start = data_start + (ts - time_start);
	if (wr_start >= buf_len) {
		wr_start = wr_start % buf_len;
		wrap0 = true;
	}

	size_t _wr_start = wr_start * type_size;

	/* Write or update head on 0 length write */
	if (len) {
		if ((wr_start + len) < buf_len) {
			buf = (char *) data + _wr_start;
		} else {
			buf = new char[len * type_size];
			copy0 = (buf_len - wr_start) * type_size;
			wrap1 = true;
		}
	}

	if (!data_start) {
		data_start = wr_start;
		time_start = ts;
		init = true;
	}

	data_end = (wr_start + len) % buf_len;
	time_end = ts + len;

	if (!init && wrap0 && (data_end > data_start))
		return NULL;
	if (time_end <= time_start)
		return NULL;

	this->tag = buf;
	this->tag_len = len;
	this->tag_alloc = wrap1;

	if (wrap1) {
		this->tag_split = copy0;
		this->tag_start = (char *) data + _wr_start;
	} else {
		this->tag_split = 0;
		this->tag_start = 0;
	}

	return buf;
}

bool ts_buffer::commit_wr(void *buf)
{
	char *_data;
	int type_size = 2 * sizeof(short);

	if ((!buf) || (buf != tag))
		return false;

	if (tag_alloc) {
		_data = tag_start;
		memcpy(_data, buf, tag_split);
		memcpy(data, (char *) buf + tag_split,
		       tag_len * type_size - tag_split);

		delete tag;
	}

	tag = NULL;

	return true;
}

ssize_t ts_buffer::write(void *buf, size_t len)
{
	return write(buf, len, time_end);
}

ssize_t ts_buffer::write(int64_t timestamp)
{
	return write(NULL, 0, timestamp);
}

std::string ts_buffer::str_status() const
{
	std::ostringstream ost("Sample buffer: ");

	ost << "length = " << buf_len;
	ost << ", time_start = " << time_start;
	ost << ", time_end = " << time_end;
	ost << ", data_start = " << data_start;
	ost << ", data_end = " << data_end;

	return ost.str();
}

std::string ts_buffer::str_code(ssize_t code)
{
	switch (code) {
	case ERR_TIMESTAMP:
		return "Sample buffer: Requested timestamp is not valid";
	case ERR_MEM:
		return "Sample buffer: Memory error";
	case ERR_OVERFLOW:
		return "Sample buffer: Overrun";
	default:
		return "Sample buffer: Unknown error";
	}
}
