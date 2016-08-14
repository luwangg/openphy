/*
 * LTE Ettus UHD Device Interface
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

#include <iostream>
#include <iomanip>
#include <stdint.h>

#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread_priority.hpp>
#include "uhd.h"
#include "buffer.h"
#include "log.h"

#define RX_BUFLEN		(1 << 20)

#define DEV_ARGS_X300		",master_clock_rate=184.32e6"
#define DEV_ARGS_DEFAULT	""

enum {
	DEV_TYPE_B200,
	DEV_TYPE_B210,
	DEV_TYPE_X300,
	DEV_TYPE_UNKNOWN,
};

struct uhd_dev {
	uhd_dev() : type(DEV_TYPE_UNKNOWN) { }

	int type;
	size_t chans;
	size_t spp;
	double rate;
	double base_freq;
	double offset_freq;
	uhd::usrp::multi_usrp::sptr dev;
	uhd::rx_streamer::sptr stream;
	std::vector<ts_buffer *> rx_bufs;
};

static int64_t last = 0;
static bool pps_init = false;

void uhd_reset(struct uhd_dev *dev)
{
	uhd_stop_rx(dev);

	for (size_t i = 0; i < dev->rx_bufs.size(); i++)
		delete dev->rx_bufs[i];

	dev->rx_bufs.resize(0);

	last = 0;
}

static double uhd_get_rate(int rbs)
{
	switch (rbs) {
	case 6:
		return 1.92e6;
	case 15:
		return 3.84e6;
	case 25:
		return 5.76e6;
	case 50:
		return 11.52e6;
	case 75:
		return 15.36e6;
	case 100:
		return 23.04e6;
	default:
		std::cerr << "** Invalid sample rate selection" << std::endl;
	}

	return 0.0;
}

static bool uhd_init_freq(struct uhd_dev *dev, double freq)
{
	uhd::tune_request_t treq(freq);
	uhd::tune_result_t tres;

	std::cout << "-- Setting frequency to " << freq << " Hz" << std::endl;
	try {
		for (size_t i = 0; i < dev->chans; i++)
			tres = dev->dev->set_rx_freq(treq, i);

		dev->base_freq = tres.actual_rf_freq;

		treq.rf_freq_policy = uhd::tune_request_t::POLICY_MANUAL;
		treq.rf_freq = dev->base_freq;

		for (size_t i = 0; i < dev->chans; i++)
			dev->dev->set_rx_freq(treq, i);
	} catch (const std::exception &ex) {
		std::cerr << "** Frequency setting failed" << std::endl;
		std::cerr << ex.what() << std::endl;
		return false;
	}

	dev->offset_freq = dev->base_freq;

	return true;
}

static bool uhd_init_rates(struct uhd_dev *dev, int rbs)
{
	double mcr, rate = uhd_get_rate(rbs);
	if (rate == 0.0)
		return false;

	std::cout << "-- Setting rates to " << rate << " Hz" << std::endl;
	try {
		if (dev->type != DEV_TYPE_X300) {
			if (rbs < 25)
				mcr = 8.0 * rate;
			else
				mcr = rate;

			dev->dev->set_master_clock_rate(mcr);
		}
		dev->dev->set_rx_rate(rate);
	} catch (const std::exception &ex) {
		std::cerr << "** Sample rate setting failed" << std::endl;
		std::cerr << ex.what() << std::endl;
		return false;
	}

	dev->rate = dev->dev->get_rx_rate();

	return true;
}

static bool uhd_init_rx(struct uhd_dev *dev, int64_t *ts)
{
	uhd::stream_args_t stream_args("sc16", "sc16");
	dev->rx_bufs.resize(dev->chans);

	for (size_t i = 0; i < dev->chans; i++) {
		stream_args.channels.push_back(i);
		dev->rx_bufs[i] = new ts_buffer(RX_BUFLEN);
		dev->rx_bufs[i]->init();
	}

	dev->stream = dev->dev->get_rx_stream(stream_args);

	dev->spp = dev->stream->get_max_num_samps();
	std::cout << "-- Samples per packet " << dev->spp << std::endl;

	std::vector<std::vector<int16_t> >
		pkt_bufs(dev->chans, std::vector<int16_t>(2 * dev->spp));

	std::vector<int16_t *> pkt_ptrs;
	for (size_t i = 0; i < pkt_bufs.size(); i++)
		pkt_ptrs.push_back(&pkt_bufs[i].front());

	uhd::time_spec_t current = dev->dev->get_time_now();
	uhd::stream_cmd_t cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
	cmd.stream_now = false;
	cmd.time_spec = uhd::time_spec_t(current + 0.2);
	dev->dev->issue_stream_cmd(cmd);

	uhd::rx_metadata_t md;
	while (!dev->stream->recv(pkt_ptrs, dev->spp, md, 1.0, true));

	*ts = md.time_spec.to_ticks(dev->rate);

	return true;
}

void uhd_stop_rx(struct uhd_dev *dev)
{
	uhd::stream_cmd_t cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
	dev->dev->issue_stream_cmd(cmd);

	std::vector<std::vector<int16_t> >
		pkt_bufs(dev->chans, std::vector<int16_t>(2 * dev->spp));

	std::vector<int16_t *> pkt_ptrs;
	for (size_t i = 0; i < pkt_bufs.size(); i++)
		pkt_ptrs.push_back(&pkt_bufs[i].front());

	uhd::rx_metadata_t md;
	while (dev->stream->recv(pkt_ptrs, dev->spp, md, 0.1, true) > 0);
}

static bool uhd_init_gains(struct uhd_dev *dev, double gain)
{
	std::cout << "-- Setting gain to " << gain << " dB" << std::endl;
	try {
		for (size_t i = 0; i < dev->chans; i++)
			dev->dev->set_rx_gain(gain, i);
	} catch (const std::exception &ex) {
		std::cerr << "** Gain setting failed" << std::endl;
		std::cerr << ex.what() << std::endl;
		return false;
	}

	return true;
}

static int get_dev_type(const uhd::device_addr_t &addr)
{
	size_t b200_str, b210_str, x300_str, x310_str;

	b200_str = addr.to_string().find("B200");
	b210_str = addr.to_string().find("B210");
	x300_str = addr.to_string().find("X300");
	x310_str = addr.to_string().find("X310");

	if (b200_str != std::string::npos)
		return DEV_TYPE_B200;
	else if (b210_str != std::string::npos)
		return DEV_TYPE_B210;
	else if ((x300_str != std::string::npos) ||
		 (x310_str != std::string::npos))
		return DEV_TYPE_X300;

	return DEV_TYPE_UNKNOWN;
}

static std::string get_dev_args(int type)
{
	switch (type) {
	case DEV_TYPE_X300:
		return DEV_ARGS_X300;
	case DEV_TYPE_B200:
	case DEV_TYPE_B210:
	default:
		return DEV_ARGS_DEFAULT;
	}
}

struct uhd_dev *uhd_init(int64_t *ts, double freq, const std::string &args,
			 size_t rbs, size_t chans, double gain, bool ext)
{
	struct uhd_dev *dev = new struct uhd_dev();

	uhd::device_addr_t addr(args);
	uhd::device_addrs_t addrs = uhd::device::find(addr);
	if (!addrs.size()) {
		std::cerr << "** No UHD device found" << std::endl;
		return NULL;
	}

	std::cout << "-- Opening device " << addrs[0].to_string() << std::endl;
	dev->type = get_dev_type(addrs[0]);
	if (dev->type == DEV_TYPE_UNKNOWN) {
		std::cerr << "** Unsupported or unknown device" << std::endl;
		delete dev;
		return NULL;
	}

	addr = uhd::device_addr_t(args + get_dev_args(dev->type));
	try {
		dev->dev = uhd::usrp::multi_usrp::make(addr);
	} catch (const std::exception &ex) {
		std::cerr << "** UHD make failed" << std::endl;
		std::cerr << ex.what() << std::endl;
		return NULL;
	}

	dev->chans = chans;
	if (!pps_init && chans > 1) {
		dev->dev->set_time_unknown_pps(uhd::time_spec_t());
		pps_init = true;
	}

        uhd::set_thread_priority_safe();

	if (ext)
		dev->dev->set_clock_source("external");

	if (!uhd_init_rates(dev, rbs) || !uhd_init_freq(dev, freq))
		return NULL;

	if (!uhd_init_gains(dev, gain) || !uhd_init_rx(dev, ts))
		return NULL;

	return dev;
}

int uhd_freq_reset(struct uhd_dev *dev)
{
	uhd::tune_request_t treq(dev->base_freq);
	treq.rf_freq_policy = uhd::tune_request_t::POLICY_MANUAL;
	treq.rf_freq = dev->base_freq;

	std::ostringstream ost;

	try {
		for (size_t i = 0; i < dev->chans; i++)
			dev->dev->set_rx_freq(treq, i);

		dev->offset_freq = dev->base_freq;
	} catch (const std::exception &ex) {
		ost << "DEV   : Frequency setting failed - " << ex.what();
		LOG_ERR(ost.str().c_str());
		return -1;
	}

	ost << "DEV   : Resetting RF frequency to "
	    << dev->base_freq / 1e6 << " MHz";

	LOG_DEV(ost.str().c_str());

	return 0;
}

int uhd_shift(struct uhd_dev *dev, double offset)
{
	uhd::tune_request_t treq(dev->offset_freq + offset);
	treq.rf_freq_policy = uhd::tune_request_t::POLICY_MANUAL;
	treq.rf_freq = dev->base_freq;

	std::ostringstream ost;

	try {
		for (size_t i = 0; i < dev->chans; i++)
			dev->dev->set_rx_freq(treq, i);

		dev->offset_freq = dev->dev->get_rx_freq();
	} catch (const std::exception &ex) {
		ost << "DEV   : Frequency setting failed - " << ex.what();
		LOG_ERR(ost.str().c_str());
		return -1;
	}

	ost << "DEV   : Adjusting DDC " << offset << " Hz"
	    << ", DDC offset " << dev->base_freq - dev->offset_freq  << " Hz";
	LOG_DEV(ost.str().c_str());

	return 0;
}

int64_t uhd_get_ts_high(struct uhd_dev *dev)
{
	return dev->rx_bufs[0]->get_last_time();
}

int64_t uhd_get_ts_low(struct uhd_dev *dev)
{
	return dev->rx_bufs[0]->get_first_time();
}

bool dump = false;

int uhd_reload(struct uhd_dev *dev)
{
	int rc;
	uhd::rx_metadata_t md;
	size_t total = 0;

	std::vector<std::vector<int16_t> >
		pkt_bufs(dev->chans, std::vector<int16_t>(2 * dev->spp));

	std::vector<int16_t *> pkt_ptrs;
	for (size_t i = 0; i < pkt_bufs.size(); i++)
		pkt_ptrs.push_back(&pkt_bufs[i].front());

	for (;;) {
		size_t num = dev->stream->recv(pkt_ptrs, dev->spp, md, 1.0, true);
		if (num <= 0) {
			std::cout << "Receive timed out " <<  std::endl;
			dump = true;
			continue;
		} else if (num < dev->spp) {
			std::cout << "Short packet" <<  std::endl;
		}

		total += num;

		int64_t ts = md.time_spec.to_ticks(dev->rate);

		if (dump) {
			std::cout << "ts : " << ts << std::endl;
			dump = false;
		}

		if (ts < last) {
			std::cout << "ts   : " << ts << std::endl;
			std::cout << "last : " << last << std::endl;
			std::cout << "Non-monotonic TIME" << std::endl;
			exit(1);
			continue;
		}

		if ((size_t) (ts - last) != dev->spp) {
			std::cout << "UHD Timestamp Jump" << std::endl;
			std::cout << "expected : " << dev->spp << std::endl;
			std::cout << "got      : " << ts - last << std::endl;
		}

		for (size_t i = 0; i < pkt_ptrs.size(); i++) {
			rc = dev->rx_bufs[i]->write(pkt_ptrs[i], num, ts);
			if (rc < 0) {
				if (rc == -2) {
					std::cout << "Internal overflow" << std::endl;
					continue;
				}

				std::cout << "Fatal buffer reload error " << rc << std::endl;
				std::cout << "ts   : " << ts << std::endl;
				std::cout << "last : " << last << std::endl;
				exit(1);
			}
		}

		last = ts;

		if (total >= dev->spp)
			break;
	}

	return 0;
}

int uhd_pull(struct uhd_dev *dev,
	     std::vector<short *> &bufs,
	     size_t len, int64_t ts)
{
	int err;

	if (bufs.size() != dev->chans) {
		std::cerr << "UHD: Invalid buffer " << bufs.size() << std::endl;
		return -1;
	}

	if (dev->rx_bufs[0]->avail_smpls(ts) < len) {
		std::cerr << "Insufficient samples in buffer " << std::endl;
		return -1;
	}

	for (size_t i = 0; i < bufs.size(); i++) {
		bufs[i] = (int16_t *) dev->rx_bufs[i]->get_rd_buf(ts, len, &err);
		if (!bufs[i]) {
			std::cerr << "Fatal buffer pull error " << err << std::endl;
			return -1;
//			exit(1);
		}
	}

	return len;
}

int uhd_commit(struct uhd_dev *dev, std::vector<short *> &bufs)
{
	if (bufs.size() != dev->rx_bufs.size()) {
		std::cerr << "Fatal I/O error" << std::endl;
		return -1;
	}

	for (size_t i = 0; i < bufs.size(); i++) {
		if (!dev->rx_bufs[i]->commit_rd(bufs[i])) {
			std::cerr << "Fatal commit error" << std::endl;
			return -1;
		}
	}

	return 0;
}

