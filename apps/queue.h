#ifndef SUBFRAME_QUEUE_H
#define SUBFRAME_QUEUE_H

#include <thread>
#include <mutex>
#include <queue>
#include <cstdio>

extern "C" {
#include "openphy/lte.h"
#include "../src/buffer.h"
}

struct lte_subframe;

struct lte_buffer {
	lte_buffer(size_t chans)
	 : tx_ants(0), rx_ants(chans), rbs(0), n_id_cell(0), ng(0),
	   bufs(chans, NULL), crc_pass(false), subframe(chans, NULL)
	{
	}

	~lte_buffer()
	{
		for (size_t i = 0; i < bufs.size(); i++) {
			free(bufs[i]);
			free(subframe[i]);
		}
	}

	size_t tx_ants, rx_ants;
	int rbs;
	int n_id_cell;
	int ng;
	struct lte_time time;

	bool crc_pass;
	std::vector<short *> bufs;
	std::vector<struct lte_subframe *> subframe;
};

class lte_buffer_q {
public:
	lte_buffer_q()
	{
	}

	~lte_buffer_q()
	{
		lte_buffer *buf;

		while (buf = read())
			delete buf;
	}

	size_t size()
	{
		std::lock_guard<std::mutex> guard(mutex);
		return q.size();
	};

	lte_buffer *read()
	{
		std::lock_guard<std::mutex> guard(mutex);

		if (q.empty())
			return NULL;

		lte_buffer *buf = q.front();
		q.pop();

		return buf;
	}

	bool write(lte_buffer *buf)
	{
		std::lock_guard<std::mutex> guard(mutex);

		q.push(buf);

		return true;
	}

private:
	std::mutex mutex;
	std::queue<lte_buffer *> q;
};

#endif /* SUBFRAME_QUEUE_H */
