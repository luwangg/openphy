#ifndef RRC_H
#define RRC_H

#define CMD_CODE		"open-ue-cmd"
#define CMD_CODE_LEN		11

enum rrc_state {
	RRC_STATE_IDLE,
	RRC_STATE_SYNC,
	RRC_STATE_RA,
	RRC_STATE_RAR,
};

enum rrc_cmd {
	RRC_CMD_CHG_STATE,
	RRC_CMD_SET_RNTI,
	RRC_CMD_SET_PDCCH,
	RRC_CMD_PRACH,
	RRC_CMD_NUM,
};

struct rrc_hdr {
	char code[CMD_CODE_LEN];
	uint16_t size;
	uint8_t cmd;
	char data[0];
} __attribute__((packed));

struct rrc_prach_hdr {
	uint8_t config_index;
	uint16_t root_seq_index;
	uint8_t high_speed;
	uint8_t zero_corr_zone;
	uint8_t freq_offset;
} __attribute__((packed));

enum frame_enable {
	FRAME_ENABLE_OFF,
	FRAME_ENABLE_ALL,
	FRAME_ENABLE_EVEN,
	FRAME_ENABLE_ODD,
	FRAME_ENABLE_NUM,
};

struct rrc_subframe_hdr {
	uint8_t num;
	uint8_t enable;
} __attribute__((packed));

struct rrc_rnti_hdr {
	uint16_t rnti;
} __attribute__((packed));

struct subframe_state {
	int num;
	int enable;
};

#endif /* RRC_H */
