#ifndef _VITERBI_H_
#define _VITERBI_H_

#define VITERBI_LEN		40
#define VITERBI_STATES		64

struct vstate {
	unsigned state;
	unsigned val;
	unsigned prev[2];
	unsigned output[2];
};

struct vtrellis {
	int num_states;
	struct vstate *states;
};

struct vdecoder {
	int len;
	int num_states;
	int *metrics;
	int *sums;
	int **paths;
	struct vtrellis *trellis;
};

struct lte_conv_gen {
	unsigned mask[3];
};


struct vdecoder *alloc_viterbi(int len, int num_states,
			       struct lte_conv_gen *gen);
int viterbi_decode(signed char *seq, unsigned char *out,
		   int len, struct vdecoder *dec);
void free_viterbi(struct vdecoder *dec);

#endif /* _VITERBI_H_ */
