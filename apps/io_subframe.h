#include <stddef.h>
#include <vector>


struct cxvec;
struct Resampler;

class io_subframe {
public:
	io_subframe(size_t chans = 1);
	~io_subframe();

	bool init(size_t rbs, size_t taps = 384);

	bool preprocess_pss();
	bool preprocess_pbch(size_t chan, struct cxvec *vec);
	bool update();

	bool delay(size_t chan, short *buf, size_t len, int offset);

	void reset();

	short **get_raw();
	const struct cxvec **get_pss();
	const struct cxvec **get_pbch();

	size_t len;
	size_t chans;
	std::vector<short *> raw;
	std::vector<struct cxvec *> pss;
	std::vector<struct cxvec *> pbch;

private:
	void convert(size_t start, size_t len);
	bool convert();

	size_t taps, hlen;
	std::vector<struct cxvec *> base;
	bool convert_on, pss_on;

	std::vector<short *> history;
	std::vector<Resampler *> pss_resampler;
	std::vector<Resampler *> pbch_resampler;
};
