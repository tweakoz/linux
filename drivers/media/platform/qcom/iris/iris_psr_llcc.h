/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef IRIS_PSR_LLCC_H
#define IRIS_PSR_LLCC_H
#include <linux/types.h>
struct llcc_slice_desc;
struct iris_psr_llcc {
	struct llcc_slice_desc *desc[4];
	bool active[4];
	unsigned int count;
	bool submitted;
	bool quarantined;
};
int iris_psr_llcc_acquire(struct iris_psr_llcc *cache, unsigned int count);
int iris_psr_llcc_activate(struct iris_psr_llcc *cache);
int iris_psr_llcc_deactivate(struct iris_psr_llcc *cache);
int iris_psr_llcc_release(struct iris_psr_llcc *cache);
struct iris_core;
int iris_psr_llcc_probe(struct iris_core *core);
int iris_psr_llcc_enable(struct iris_core *core);
void iris_psr_llcc_quiesced(struct iris_core *core);
int iris_psr_llcc_submit(struct iris_core *core);
#endif
