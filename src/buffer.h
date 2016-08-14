#ifndef _LTE_BUFFER_H_
#define _LTE_BUFFER_H_

#include <stdint.h>
#include <unistd.h>
#include <string>

class ts_buffer {
public:
	ts_buffer(size_t len);
	~ts_buffer();

	bool init();
	void reset();

	size_t avail_smpls(int64_t ts) const;

	ssize_t read(void *buf, size_t len, int64_t ts);
	ssize_t write(void *buf, size_t len, int64_t ts);
	ssize_t write(void *buf, size_t len);
	ssize_t write(int64_t ts);

	const void *get_rd_buf(int64_t ts, size_t len, int *err = NULL);
	void *get_wr_buf(int64_t ts, size_t len, int *err = NULL);

	bool commit_rd(const void *buf);
	bool commit_wr(void *buf);

	std::string str_status() const;

	static std::string str_code(ssize_t code);

	enum err {
		ERR_MEM,
		ERR_TIMESTAMP,
		ERR_OVERFLOW,
	};

	int64_t get_last_time() const { return time_end; }
	int64_t get_first_time() const { return time_start; }

private:
	uint32_t *data;
	size_t buf_len;

	int64_t time_start;
	int64_t time_end;
	int64_t data_start;
	int64_t data_end;

	char *tag;
	int tag_len;
	char *tag_start;
	int tag_split;
	bool tag_alloc;
};

#endif /* _LTE_BUFFER_H_ */
