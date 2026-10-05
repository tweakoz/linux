// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/clk.h>
#include <linux/devfreq.h>
#include <linux/interconnect.h>
#include <linux/iris_vpu_bus.h>
#include <linux/iommu.h>
#include <linux/of.h>
#include <linux/pm_domain.h>
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>

#include "iris_core.h"
#include "iris_instance.h"
#include "iris_resources.h"

#define BW_THRESHOLD 50000

int iris_set_icc_bw(struct iris_core *core, unsigned long icc_bw)
{
	unsigned long bw_kbps, bw_prev = core->power.icc_bw;
	u32 cfg_prev = 0, mem_prev = 0;
	const struct icc_info *icc_tbl = core->iris_platform_data->icc_tbl;
	int cfg = -1, mem = -1, ret, i;
	bool restore_cfg = false;

	for (i = 0; i < core->icc_count; i++) {
		if (!strcmp(core->icc_tbl[i].name, "cpu-cfg"))
			cfg = i;
		else if (!strcmp(core->icc_tbl[i].name, "video-mem"))
			mem = i;
	}
	/* Power-off clears every vote, including the fixed configuration path. */
	if (cfg >= 0) {
		cfg_prev = core->icc_tbl[cfg].avg_bw;
		restore_cfg = cfg_prev != icc_tbl[cfg].bw_min_kbps;
		core->icc_tbl[cfg].avg_bw = icc_tbl[cfg].bw_min_kbps;
	}
	if (mem >= 0) {
		mem_prev = core->icc_tbl[mem].avg_bw;
		bw_kbps = clamp_t(unsigned long, icc_bw,
				  icc_tbl[mem].bw_min_kbps, icc_tbl[mem].bw_max_kbps);
		if (abs(bw_kbps - bw_prev) < BW_THRESHOLD && bw_prev) {
			if (!restore_cfg)
				return 0;
			/* Restore configuration without changing the existing memory vote. */
		} else {
			core->icc_tbl[mem].avg_bw = bw_kbps;
			core->power.icc_bw = bw_kbps;
		}
	}
	ret = icc_bulk_set_bw(core->icc_count, core->icc_tbl);
	if (ret) {
		if (cfg >= 0)
			core->icc_tbl[cfg].avg_bw = cfg_prev;
		if (mem >= 0)
			core->icc_tbl[mem].avg_bw = mem_prev;
		/* Bulk failure can unvote earlier paths: force the next full retry. */
		core->power.icc_bw = 0;
	}
	return ret;
}

int iris_unset_icc_bw(struct iris_core *core)
{
	u32 i;

	core->power.icc_bw = 0;

	for (i = 0; i < core->icc_count; i++) {
		core->icc_tbl[i].avg_bw = 0;
		core->icc_tbl[i].peak_bw = 0;
	}

	return icc_bulk_set_bw(core->icc_count, core->icc_tbl);
}

int iris_opp_set_rate(struct device *dev, unsigned long freq)
{
	struct dev_pm_opp *opp __free(put_opp) =
		devfreq_recommended_opp(dev, &freq, 0);

	if (IS_ERR(opp))
		return PTR_ERR(opp);

	return dev_pm_opp_set_opp(dev, opp);
}

int iris_enable_power_domains(struct iris_core *core, struct device *pd_dev)
{
	int ret;

	ret = iris_opp_set_rate(core->dev, ULONG_MAX);
	if (ret)
		return ret;

	return pm_runtime_resume_and_get(pd_dev);
}

int iris_disable_power_domains(struct iris_core *core, struct device *pd_dev)
{
	int ret;
	int pm_ret;

	ret = iris_opp_set_rate(core->dev, 0);

	pm_ret = pm_runtime_put_sync(pd_dev);
	if (!ret)
		ret = pm_ret;

	return ret;
}

static struct clk *iris_get_clk_by_type(struct iris_core *core, enum platform_clk_type clk_type)
{
	const struct platform_clk_data *clk_tbl;
	u32 clk_cnt, i, j;

	clk_tbl = core->iris_platform_data->clk_tbl;
	clk_cnt = core->iris_platform_data->clk_tbl_size;

	for (i = 0; i < clk_cnt; i++) {
		if (clk_tbl[i].clk_type == clk_type) {
			for (j = 0; core->clock_tbl && j < core->clk_count; j++) {
				if (!strcmp(core->clock_tbl[j].id, clk_tbl[i].clk_name))
					return core->clock_tbl[j].clk;
			}
		}
	}

	return NULL;
}

int iris_prepare_enable_clock(struct iris_core *core, enum platform_clk_type clk_type)
{
	struct clk *clock;

	clock = iris_get_clk_by_type(core, clk_type);
	if (!clock)
		return -ENOENT;

	return clk_prepare_enable(clock);
}

int iris_disable_unprepare_clock(struct iris_core *core, enum platform_clk_type clk_type)
{
	struct clk *clock;

	clock = iris_get_clk_by_type(core, clk_type);
	if (!clock)
		return -EINVAL;

	clk_disable_unprepare(clock);

	return 0;
}

struct iris_context_device {
	struct device dev;
	struct device_dma_parameters dma_parms;
};

static void iris_device_release(struct device *dev)
{
	struct iris_context_device *ctx = container_of(dev, struct iris_context_device, dev);

	of_node_put(dev->of_node);
	kfree(ctx);
}

int iris_create_child_device_and_map(struct iris_core *core, const struct iris_context_bank *cb)
{
	u32 index, region_mask = cb->region_mask;
	struct iris_context_device *ctx;
	struct device *dev;
	int ret;

	ctx = kzalloc_obj(*ctx);
	if (!ctx)
		return -ENOMEM;

	dev = &ctx->dev;
	device_initialize(dev);
	dev->release = iris_device_release;
	dev->bus = &iris_vpu_bus_type;
	dev->parent = core->dev;
	dev->dma_parms = &ctx->dma_parms;
	if (cb->of_node_name) {
		dev->of_node = of_get_child_by_name(core->dev->of_node, cb->of_node_name);
		if (!dev->of_node) {
			ret = -EINVAL;
			goto err_put;
		}
	}
	dev->coherent_dma_mask = core->iris_platform_data->dma_mask;
	dev->dma_mask = &dev->coherent_dma_mask;
	dma_set_max_seg_size(dev, DMA_BIT_MASK(32));
	dma_set_seg_boundary(dev, DMA_BIT_MASK(32));

	ret = dev_set_name(dev, "%s-%s", dev_name(core->dev), cb->name);
	if (ret)
		goto err_put;
	dev_set_drvdata(dev, (void *)cb);

	ret = device_add(dev);
	if (ret)
		goto err_put;

	/* The IOMMU bus callback may run before the SMMU supplier is ready. */
	if (!device_iommu_mapped(dev)) {
		ret = -EPROBE_DEFER;
		goto err_unregister;
	}
	if (cb->num_stream_ids &&
	    (!dev_iommu_fwspec_get(dev) ||
	     dev_iommu_fwspec_get(dev)->num_ids != cb->num_stream_ids)) {
		ret = -EINVAL;
		goto err_unregister;
	}

	while (region_mask) {
		index = __ffs(region_mask);
		core->cb_devs[index] = dev;
		region_mask &= ~BIT(index);
	}

	return 0;

err_unregister:
	device_unregister(dev);
	return dev_err_probe(core->dev, ret, "Failed to map context %s\n", cb->name);
err_put:
	put_device(dev);
	return ret;
}

enum iris_buffer_region iris_get_region(struct iris_inst *inst, enum iris_buffer_type buffer_type)
{
	switch (buffer_type) {
	case BUF_INPUT:
		if (inst->domain == ENCODER)
			return IRIS_PIXEL_REGION;
		else
			return IRIS_BITSTREAM_REGION;
	case BUF_OUTPUT:
		if (inst->domain == ENCODER)
			return IRIS_BITSTREAM_REGION;
		else
			return IRIS_PIXEL_REGION;
	case BUF_BIN:
		return IRIS_BITSTREAM_REGION;
	case BUF_DPB:
	case BUF_PARTIAL:
	case BUF_SCRATCH_2:
	case BUF_VPSS:
		return IRIS_PIXEL_REGION;
	case BUF_ARP:
	case BUF_COMV:
	case BUF_LINE:
	case BUF_NON_COMV:
	case BUF_PERSIST:
		return IRIS_NON_PIXEL_REGION;
	default:
		dev_err(inst->core->dev, "unknown buffer type: %d\n", buffer_type);
		return IRIS_UNKNOWN_REGION;
	}
}

struct device *iris_get_cb_dev(struct iris_core *core, enum iris_buffer_region region)
{
	if (core->cb_devs[region])
		return core->cb_devs[region];

	return core->dev;
}
