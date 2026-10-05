// SPDX-License-Identifier: GPL-2.0-only
/* Unintegrated Kaanapali candidate. Caller must supply a source-verified SKU count. */
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/soc/qcom/llcc-qcom.h>
#include "iris_psr_llcc.h"

static const u32 psr_llcc_ids[] = {
	LLCC_VIDSC0, LLCC_VIDVSP, LLCC_VIDDEC, LLCC_VIDEO_APV,
};

int iris_psr_llcc_acquire(struct iris_psr_llcc *cache, unsigned int count)
{
	unsigned int i;
	int ret;

	if (count != 2 && count != 4)
		return -EINVAL;
	if (cache->count)
		return -EBUSY;
	for (i = 0; i < count; i++) {
		cache->desc[i] = llcc_slice_getd(psr_llcc_ids[i]);
		if (IS_ERR_OR_NULL(cache->desc[i])) {
			ret = cache->desc[i] ? PTR_ERR(cache->desc[i]) : -ENODEV;
			cache->desc[i] = NULL;
			while (i) {
				i--;
				llcc_slice_putd(cache->desc[i]);
				cache->desc[i] = NULL;
			}
			return ret;
		}
	}
	cache->count = count;
	return 0;
}

/* Call only after firmware can no longer access these slices. */
int iris_psr_llcc_deactivate(struct iris_psr_llcc *cache)
{
	int i, ret, first = 0;

	for (i = (int)cache->count - 1; i >= 0; i--) {
		if (!cache->active[i])
			continue;
		ret = llcc_slice_deactivate(cache->desc[i]);
		if (ret) {
			if (!first)
				first = ret;
		} else {
			cache->active[i] = false;
		}
	}
	return first;
}

int iris_psr_llcc_activate(struct iris_psr_llcc *cache)
{
	bool acquired[4] = { false };
	unsigned int i;
	int ret, cleanup;

	if (cache->count != 2 && cache->count != 4)
		return -EINVAL;
	/* A previous failed deactivation is not permission to add another vote. */
	for (i = 0; i < cache->count; i++)
		if (cache->active[i])
			return -EBUSY;
	for (i = 0; i < cache->count; i++) {
		ret = llcc_slice_activate(cache->desc[i]);
		if (ret)
			goto rollback;
		cache->active[i] = true;
		acquired[i] = true;
	}
	return 0;
rollback:
	while (i) {
		i--;
		if (!acquired[i])
			continue;
		cleanup = llcc_slice_deactivate(cache->desc[i]);
		if (!cleanup)
			cache->active[i] = false;
		/* Retain failed cleanup ownership; caller logs and stops startup. */
	}
	return ret;
}

/* Refuse to drop descriptor ownership while an activation remains unresolved. */
int iris_psr_llcc_release(struct iris_psr_llcc *cache)
{
	unsigned int i;

	for (i = 0; i < cache->count; i++)
		if (cache->active[i])
			return -EBUSY;
	for (i = cache->count; i; i--) {
		llcc_slice_putd(cache->desc[i - 1]);
		cache->desc[i - 1] = NULL;
	}
	cache->count = 0;
	return 0;
}

/* Core integration; firmware lifecycle owns the point where deactivation is safe. */
#include <linux/of.h>
#include <linux/soc/qcom/ubwc.h>
#include "iris_core.h"
#include "iris_hfi_gen2_packet.h"
#include "iris-psr-socinfo.h"

static void iris_psr_llcc_put_action(void *data)
{
	struct iris_core *core = data;

	if (iris_psr_llcc_release(&core->psr_llcc))
		dev_err(core->dev, "LLCC descriptors retained: unresolved activation\n");
}

int iris_psr_llcc_probe(struct iris_core *core)
{
	u32 part;
	unsigned int count;
	int ret;

	if (!of_device_is_compatible(core->dev->of_node, "qcom,kaanapali-iris"))
		return 0;
	ret = iris_psr_video_part(&part);
	if (ret)
		return dev_err_probe(core->dev, ret, "PART_VIDEO unavailable; no guessed cache SKU\n");
	switch (part) {
	case 0: case 1: case 2: case 0x10:
		count = 4;
		break;
	case 0x11: case 0x12:
		count = 2;
		break;
	default:
		return dev_err_probe(core->dev, -EINVAL, "Unknown PART_VIDEO %#x\n", part);
	}
	ret = iris_psr_llcc_acquire(&core->psr_llcc, count);
	if (ret)
		return dev_err_probe(core->dev, ret, "LLCC descriptor acquisition failed\n");
	ret = devm_add_action_or_reset(core->dev, iris_psr_llcc_put_action, core);
	if (ret)
		return ret;
	dev_info(core->dev, "PSR cache PART_VIDEO=%#x slices=%u UBWC_HBB=%u\n",
		 part, count, core->ubwc_cfg->highest_bank_bit);
	return 0;
}

int iris_psr_llcc_enable(struct iris_core *core)
{
	if (!core->psr_llcc.count)
		return 0;
	return iris_psr_llcc_activate(&core->psr_llcc);
}

void iris_psr_llcc_quiesced(struct iris_core *core)
{
	int ret = iris_psr_llcc_deactivate(&core->psr_llcc);

	if (ret)
		dev_err(core->dev, "LLCC deactivate failed after firmware quiesce: %d\n", ret);
}

int iris_psr_llcc_submit(struct iris_core *core)
{
	struct iris_psr_llcc *cache = &core->psr_llcc;
	struct iris_hfi_header *hdr;
	unsigned int i;
	int ret;

	if (!cache->count || cache->submitted)
		return 0;
	for (i = 0; i < cache->count; i++)
		if (!cache->active[i])
			return -EINVAL;
	hdr = kzalloc(sizeof(*hdr) + cache->count *
		      (sizeof(struct iris_hfi_packet) + sizeof(struct iris_hfi_buffer)), GFP_KERNEL);
	if (!hdr)
		return -ENOMEM;
	iris_hfi_gen2_packet_psr_subcaches(core, hdr);
	ret = iris_hfi_queue_cmd_write_locked(core, hdr, hdr->size);
	kfree(hdr);
	if (ret)
		return ret;
	cache->submitted = true;
	for (i = 0; i < cache->count; i++)
		dev_info(core->dev, "PSR cache submitted SCID=%u sizeKiB=%zu\n",
			 cache->desc[i]->slice_id, cache->desc[i]->slice_size);
	return 0;
}
