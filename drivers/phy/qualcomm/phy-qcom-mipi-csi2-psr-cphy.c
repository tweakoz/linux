// SPDX-License-Identifier: GPL-2.0-only
/* PSR-only experimental C-PHY; exact captured main mode0 / tele RAW10.
 * Analog values generated from OnePlus OSS 5ab2a689ff87; see generator.
 * No DT enables these compatibles by default. No PHY interrupt handler:
 * receiver errors are observed through CSID, PHY IRQ masks remain zero.
 */
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include "phy-qcom-mipi-csi2.h"
struct psr_reg { u16 offset; u8 value; u16 delay_us; };
#include "phy-qcom-mipi-csi2-psr-tables.h"

static void psr_write(struct mipi_csi2phy_device *p,
		      const struct psr_reg *r, size_t n, bool clear)
{
	size_t i;

	for (i = 0; i < n; i++) {
		writel(clear ? 0 : r[i].value, p->base + r[i].offset);
		/* Match OEM cam_io_w_mb ordering before delay/next register. */
		wmb();
		if (r[i].delay_us)
			usleep_range(r[i].delay_us, r[i].delay_us + 5);
	}
}

static void psr_reset(struct mipi_csi2phy_device *p)
{
	writel(1, p->base + 0x1000);
	wmb();
	usleep_range(20, 25);
}

static void psr_version(struct mipi_csi2phy_device *p)
{
	/* Version probing is deliberately omitted: no guessed byte layout. */
}

static void psr_disable(struct mipi_csi2phy_device *p,
			struct mipi_csi2phy_stream_cfg *cfg)
{
	/* OEM stop clears common settings then lane selection before reset. */
	psr_write(p, psr_common, ARRAY_SIZE(psr_common), true);
	writel(0, p->base + 0x1014);
	wmb();
	psr_reset(p);
}

int psr_cphy_validate(struct mipi_csi2phy_device *p,
		       const struct phy_configure_opts_mipi_cphy *c)
{
	bool main = p->soc_cfg == &mipi_csi2_cphy_psr_main;

	if (c->lanes != 3 || p->stream_cfg.num_data_lanes != 3)
		return -EINVAL;
	/* Whitelist exact captured tuple; no inherited modes/RAW12 tele. */
	if (c->symbol_rate != (main ? 3268800000ULL : 1996800000ULL) ||
	    c->data_rate != (main ? 7471533333ULL : 4564114283ULL) ||
	    c->settle_count != (main ? 38 : 45))
		return -EINVAL;
	return 0;
}

static int psr_enable(struct mipi_csi2phy_device *p,
		      struct mipi_csi2phy_stream_cfg *cfg)
{
	bool main = p->soc_cfg == &mipi_csi2_cphy_psr_main;
	u32 v;
	int i, ret;

	ret = psr_cphy_validate(p, &cfg->cphy);
	if (ret)
		return ret;
	/* OEM reset -> REFGEN ready -> common -> lanes -> release reset. */
	ret = readl_poll_timeout(p->base + 0x1184, v, v & BIT(7), 500, 10000);
	if (ret)
		goto off;
	psr_write(p, psr_common, ARRAY_SIZE(psr_common), false);
	writel(0x2a, p->base + 0x1014);
	wmb();
	if (main)
		psr_write(p, psr_main_lanes, ARRAY_SIZE(psr_main_lanes), false);
	else
		psr_write(p, psr_tele_lanes, ARRAY_SIZE(psr_tele_lanes), false);
	for (i = 0; i < 11; i++)
		writel(0, p->base + 0x102c + 4 * i);
	wmb();
	writel(0x0e, p->base + 0x1000);
	wmb();
	usleep_range(10, 15);
	return 0;
 off:
	psr_disable(p, cfg);
	return ret;
}

static const struct mipi_csi2phy_hw_ops psr_ops = {
	.hw_version_read = psr_version,
	.reset = psr_reset,
	.lanes_enable = psr_enable,
	.lanes_disable = psr_disable,
};
static const char * const psr_clks[] = { "core", "timer", "ahb" };
static const char * const psr_supplies[] = { "vdda-0p9", "vdda-1p2" };
static struct mipi_csi2_genpd psr_pds[] = {
	{ .name = "top", .scaled = false },
	{ .name = "mmcx", .scaled = true },
	{ .name = "mx", .scaled = true },
};
#define PSR_CFG { .ops = &psr_ops, \
	.supply_names = (const char **)psr_supplies, \
	.num_supplies = ARRAY_SIZE(psr_supplies), \
	.clk_names = (const char **)psr_clks, .num_clk = ARRAY_SIZE(psr_clks), \
	.genpds = psr_pds, .num_genpds = ARRAY_SIZE(psr_pds) }
const struct mipi_csi2phy_soc_cfg mipi_csi2_cphy_psr_main = PSR_CFG;
const struct mipi_csi2phy_soc_cfg mipi_csi2_cphy_psr_tele = PSR_CFG;
