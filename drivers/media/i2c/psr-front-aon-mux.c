// SPDX-License-Identifier: GPL-2.0-only
/* PSR front-camera CPAS AON selector. Source-derived, diagnostic integration. */
#include <linux/clk.h>
#include <linux/interconnect.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mux/driver.h>
#include <linux/mux/consumer.h>
#include "psr-camera-lease.h"
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#if IS_MODULE(CONFIG_VIDEO_PSR_IMX709)
#error "The PSR diagnostic AON provider requires the built-in front driver"
#endif

#define PSR_AON_SEL 0x1e0
#define PSR_AON_MASK (BIT(0) | BIT(8))
struct psr_aon {
	struct device *dev;
	struct regmap *map;
	struct clk_bulk_data *clks;
	int nclks;
	struct icc_bulk_data paths[2];
	bool held, fault, resources_only;
	u32 saved;
};

static int psr_aon_unvote(struct psr_aon *s)
{
	int ret, err;

	ret = icc_set_bw(s->paths[1].path, 0, 0);
	err = icc_set_bw(s->paths[0].path, 0, 0);
	if (!ret) ret = err;
	err = pm_runtime_put_sync(s->dev);
	if (err < 0 && !ret) ret = err;
	if (ret) {
		WRITE_ONCE(s->fault, true);
		dev_err(s->dev, "resource release failed %d; further acquisition blocked\n", ret);
	}
	return ret;
}

static int psr_aon_power_put(struct psr_aon *s)
{
	clk_bulk_disable_unprepare(s->nclks, s->clks);
	return psr_aon_unvote(s);
}

static int psr_aon_power_get(struct psr_aon *s)
{
	int ret = pm_runtime_resume_and_get(s->dev);

	if (ret < 0)
		return ret;
	ret = icc_bulk_set_bw(ARRAY_SIZE(s->paths), s->paths);
	if (!ret)
		ret = clk_bulk_prepare_enable(s->nclks, s->clks);
	if (!ret)
		return 0;
	/* An interconnect bulk error can follow successful earlier votes. */
	psr_aon_unvote(s);
	return ret;
}

/* Called with the mux framework's exclusive lock held. State1 is restore/idle. */
static int psr_aon_set(struct mux_control *mux, int state)
{
	struct psr_aon *s = mux_chip_priv(mux->chip);
	u32 val;
	int ret, cleanup;

	if (state != 0 && state != 1 && state != 2)
		return -EINVAL;
	if (state == 1 && !s->held)
		return 0;
	/* State2 is a rear-camera resource lease: no selector MMIO, including
	 * failure and release paths. The same exclusive mux serializes cameras. */
	if (state == 2) {
		if (READ_ONCE(s->fault) || s->held)
			return -EIO;
		ret = psr_aon_power_get(s);
		if (ret)
			return ret;
		s->resources_only = true;
		s->held = true;
		dev_info(s->dev, "CPAS resource-only lease acquired; no AON selector access\n");
		return 0;
	}
	if (state == 1 && s->resources_only) {
		s->held = false;
		return psr_aon_power_put(s);
	}
	if (!state) {
		if (READ_ONCE(s->fault) || s->held)
			return -EIO;
		ret = psr_aon_power_get(s);
		if (ret)
			return ret;
		s->resources_only = false;
		ret = regmap_read(s->map, PSR_AON_SEL, &val);
		if (ret) {
			psr_aon_power_put(s);
			return ret;
		}
		s->saved = val & PSR_AON_MASK;
		s->held = true;
		dev_info(s->dev, "AON0 before=%08x; request AP camera+MCLK\n", val);
		ret = regmap_update_bits(s->map, PSR_AON_SEL, PSR_AON_MASK, 0);
		if (!ret) ret = regmap_read(s->map, PSR_AON_SEL, &val);
		if (!ret && (val & PSR_AON_MASK)) ret = -EIO;
		if (!ret) return 0;
		/* An ambiguous write still needs restoration before dropping power. */
		cleanup = regmap_update_bits(s->map, PSR_AON_SEL, PSR_AON_MASK, s->saved);
		if (!cleanup) cleanup = regmap_read(s->map, PSR_AON_SEL, &val);
		if (!cleanup && (val & PSR_AON_MASK) != s->saved) cleanup = -EIO;
		if (cleanup) {
			WRITE_ONCE(s->fault, true);
			dev_err(s->dev, "AON restore failed %d, retaining resource votes\n", cleanup);
			return ret;
		}
	} else {
		ret = regmap_update_bits(s->map, PSR_AON_SEL, PSR_AON_MASK, s->saved);
		if (!ret) ret = regmap_read(s->map, PSR_AON_SEL, &val);
		if (!ret && (val & PSR_AON_MASK) != s->saved) ret = -EIO;
		if (ret) {
			WRITE_ONCE(s->fault, true);
			dev_err(s->dev, "AON restore failed %d, retaining resource votes\n", ret);
			return ret;
		}
	}
	s->held = false;
	cleanup = psr_aon_power_put(s);
	return ret ?: cleanup;
}

static const struct mux_control_ops psr_aon_ops = { .set = psr_aon_set };

/* No semaphore wait while a sensor's V4L2 state/control lock is held. */
int psr_camera_try_select(struct mux_control *mux, unsigned int state)
{
	struct psr_aon *s;
	int ret;

	if (mux->chip->ops != &psr_aon_ops)
		return -EINVAL;
	s = mux_chip_priv(mux->chip);
	if (READ_ONCE(s->fault))
		return -EIO;
	ret = mux_control_try_select(mux, state);
	/* A failed owner may retain the semaphore. Do not endlessly defer probes. */
	if (ret == -EBUSY && READ_ONCE(s->fault))
		return -EIO;
	return ret;
}
EXPORT_SYMBOL_GPL(psr_camera_try_select);

/* Called by the owner before retaining an uncertain power-down lease. */
void psr_camera_lease_fault(struct mux_control *mux)
{
	struct psr_aon *s;

	if (WARN_ON(mux->chip->ops != &psr_aon_ops))
		return;
	s = mux_chip_priv(mux->chip);
	WRITE_ONCE(s->fault, true);
}
EXPORT_SYMBOL_GPL(psr_camera_lease_fault);

static const struct regmap_config psr_aon_regmap = {
	.reg_bits = 32, .val_bits = 32, .reg_stride = 4, .max_register = 0xffc,
};

static int psr_aon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	struct mux_chip *chip;
	struct psr_aon *s;
	void __iomem *base;
	int ret;

	if (!res || res->start != 0x0900b000 || resource_size(res) != 0x1000)
		return -EINVAL;
	chip = devm_mux_chip_alloc(dev, 1, sizeof(*s));
	if (IS_ERR(chip)) return PTR_ERR(chip);
	s = mux_chip_priv(chip); s->dev = dev;
	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base)) return PTR_ERR(base);
	s->map = devm_regmap_init_mmio(dev, base, &psr_aon_regmap);
	if (IS_ERR(s->map)) return PTR_ERR(s->map);
	s->nclks = devm_clk_bulk_get_all(dev, &s->clks);
	if (s->nclks <= 0) return s->nclks ?: -EINVAL;
	/* Reuse the reviewed native CAMSS register-access/interconnect votes. */
	s->paths[0] = (struct icc_bulk_data){ .name = "ahb", .avg_bw = 150000, .peak_bw = 300000 };
	s->paths[1] = (struct icc_bulk_data){ .name = "hf_mnoc", .avg_bw = 471860, .peak_bw = 925857 };
	ret = devm_of_icc_bulk_get(dev, ARRAY_SIZE(s->paths), s->paths);
	if (ret) return ret;
	ret = devm_pm_runtime_enable(dev);
	if (ret) return ret;
	chip->ops = &psr_aon_ops;
	chip->mux[0].states = 3;
	chip->mux[0].idle_state = 1;
	platform_set_drvdata(pdev, chip);
	return devm_mux_chip_register(dev, chip);
}

/* A successful consumer shutdown already deselected the lease. A remaining
 * held lease means power-down was uncertain: do not override ownership or the
 * mux framework lock here. The hardware reset terminates retained votes. */
static void psr_aon_shutdown(struct platform_device *pdev)
{
	struct mux_chip *chip = platform_get_drvdata(pdev);
	struct psr_aon *s = mux_chip_priv(chip);

	if (s->held)
		dev_err(&pdev->dev,
			"shutdown retaining AON lease/resources after incomplete consumer cleanup\n");
}

static const struct of_device_id psr_aon_match[] = {
	{ .compatible = "pod,psr-front-aon-mux" }, {}
};
MODULE_DEVICE_TABLE(of, psr_aon_match);
static struct platform_driver psr_aon_driver = {
	.probe = psr_aon_probe,
	.shutdown = psr_aon_shutdown,
	.driver = { .name = "psr-front-aon-mux", .of_match_table = psr_aon_match,
		.suppress_bind_attrs = true },
};
module_platform_driver(psr_aon_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PSR front camera AON ownership mux");
