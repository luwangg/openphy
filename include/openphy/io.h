#ifndef _LTE_IO_
#define _LTE_IO_

#include <stdint.h>
#include <vector>

struct lte_dbuf;

void lte_radio_iface_reset();

int lte_radio_iface_init(double freq, int chans, double gain,
			 int rbs, int ext, const std::string &args);

int lte_read_subframe_burst(std::vector<short *> buf,
			    int num, int coarse, int fine);

int lte_read_subframe(std::vector<short *> &bufs, int num,
		      int coarse, int fine, int state);
int lte_offset_freq(double offset);
int lte_offset_reset();
void lte_set_freq(double freq);

int lte_commit_subframe(std::vector<short *> &bufs);

int lte_write_subframe(int16_t *buf, int len, int dec, int zero);

#endif /* _LTE_IO_ */
