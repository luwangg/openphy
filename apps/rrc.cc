/*
 * LTE RRC Interface (Incomplete) 
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

#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstdio>
#include <string.h>

extern "C" {
#include "rrc.h"
}

struct rrc_cmd_str {
	int type;
	const char *str;
};

static const struct rrc_cmd_str rrc_cmd_table[RRC_CMD_NUM] = {
	{ RRC_CMD_CHG_STATE,	"RRC State Change" },
	{ RRC_CMD_SET_RNTI,	"RRC Set RNTI" },
	{ RRC_CMD_SET_PDCCH,	"RRC Set PDCCH" },
	{ RRC_CMD_PRACH,	"RRC PRACH" },
};

int prach_root_seq_index = -1;
int prach_config_index = -1;
int prach_freq_offset = -1;

int prach_loop_init(int sf, int u, int N_cs, int offset, int hs);

extern int g_rnti;

struct subframe_state subframe_table[10];

static const char *rrc_cmd_str(int cmd)
{
	if ((cmd < 0) || (cmd >= RRC_CMD_NUM))
		return NULL;

	return rrc_cmd_table[cmd].str;
}

static int rrc_set_rnti(struct rrc_rnti_hdr *hdr)
{
	g_rnti = hdr->rnti;

	return 0;
}

static int rrc_set_pdcch(struct rrc_subframe_hdr *hdr)
{
	if ((hdr->num >= 10) || (hdr->enable >= FRAME_ENABLE_NUM)) {
		fprintf(stderr, "RRC: Invalid PDCCH command\n");
		return -1;
	}

	subframe_table[hdr->num].enable = hdr->enable;

	return 0;
}

static int handle_state_chg()
{

}

static int handle_enable_pdcch()
{

}

static int handle_disable_pdcch()
{

}

static int handle_prach(struct rrc_prach_hdr *info)
{
	printf("RRC: Config Index............ %i\n", info->config_index);
	printf("RRC: Root Sequence Index .... %i\n", info->root_seq_index);
	printf("RRC: High Speed.............. %i\n", info->high_speed);
	printf("RRC: Zero Correlation Zone... %i\n", info->zero_corr_zone);
	printf("RRC: Frequency Offset........ %i\n", info->freq_offset);
}

void rrc_loop()
{
	int rc;
	struct sockaddr_in addr;

	int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (fd < 0) {
		fprintf(stderr, "RRC: Failed to create socket\n");
		return;
	}

	memset(&addr, 0, sizeof(struct sockaddr_in));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(4444);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);

	int on;
	setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(int));

	rc = bind(fd, (struct sockaddr *) &addr, sizeof(addr));
	if (rc < 0) {
		fprintf(stderr, "RRC: Failed to bind socket\n");
		return;
	}

	fprintf(stdout, "RRC: Socket created successfully\n");

	socklen_t len;
	char buf[2048];

	for (;;) {
		fprintf(stdout, "RRC: Waiting...\n");
		int num = recvfrom(fd, (void *) &buf, 2048, 0,
				   (struct sockaddr *) NULL, &len);
		if (num < 0) {
			fprintf(stderr, "RRC: Socket receive error\n");
			return;
		}

		fprintf(stdout, "RRC: Got %i bytes\n", num);

		struct rrc_hdr *hdr = (struct rrc_hdr *) buf;
		if (strncmp(hdr->code, CMD_CODE, CMD_CODE_LEN)) {
			fprintf(stderr, "RRC: Received invalid header\n");
			continue;
		}

		if ((hdr->size != num)) {
			fprintf(stderr, "RRC: Header size mismatch\n");
			continue;
		}

		const char *cmd_str = rrc_cmd_str(hdr->cmd);
		if (!cmd_str) {
			fprintf(stderr, "RRC: Invalid command %i\n", hdr->cmd);
			continue;
		}

		fprintf(stdout, "RRC: Received %s\n", cmd_str);

		switch (hdr->cmd) {
		case RRC_CMD_CHG_STATE:
			handle_state_chg();
			break;
		case RRC_CMD_SET_RNTI:
			rrc_set_rnti((struct rrc_rnti_hdr *) hdr->data);
			break;
		case RRC_CMD_SET_PDCCH:
			rrc_set_pdcch((struct rrc_subframe_hdr *) hdr->data);
			break;
		case RRC_CMD_PRACH:
			handle_prach((struct rrc_prach_hdr *) hdr->data);
			break;
		default:
			fprintf(stderr, "RRC: Unhandled command %i\n", hdr->cmd);
			continue;
		}
	}

	return;
}

/*
 * Enable all subframes by default. This essentially controls the transceiver
 * entirely from the command line.
 */
static void __attribute__((constructor)) init_state()
{
	int i;

	for (i = 0; i < 10; i++) {
		subframe_table[i].num = i;
		subframe_table[i].enable = FRAME_ENABLE_ALL;
	}
}
