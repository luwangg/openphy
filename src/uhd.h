#include <complex.h>
#include <vector>
#include <string>

struct uhd_dev;

void uhd_reset(struct uhd_dev *dev);

void uhd_go(struct uhd_dev *dev);

void uhd_stop_rx(struct uhd_dev *dev);

struct uhd_dev *uhd_init(int64_t *ts, double freq, const std::string &args,
			 size_t rbs, size_t chans, double gain, bool ext);
int uhd_pull(struct uhd_dev *dev,
	     std::vector<short *> &buf,
	     size_t len, int64_t ts);
int uhd_commit(struct uhd_dev *dev, std::vector<short *> &bufs);

int64_t uhd_get_ts_high(struct uhd_dev *dev);
int64_t uhd_get_ts_low(struct uhd_dev *dev);

int uhd_reload(struct uhd_dev *dev);
int uhd_write(struct uhd_dev *dev, int16_t *buf, size_t len, int64_t ts);

int uhd_shift(struct uhd_dev *dev, double offset);
int uhd_freq_reset(struct uhd_dev *dev);
