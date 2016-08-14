#ifndef _CONVOLVE_H_
#define _CONVOLVE_H_

#include "sigvec.h"

int cxvec_convolve(const struct cxvec *in,
		   const struct cxvec *h, struct cxvec *out);
int cxvec_convolve_nodelay(const struct cxvec *in,
			   const struct cxvec *h, struct cxvec *out);
int single_convolve(const float *in, const struct cxvec *h, float *out);

#endif /* _CONVOLVE_H_ */
