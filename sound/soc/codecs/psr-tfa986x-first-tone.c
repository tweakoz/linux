// SPDX-License-Identifier: GPL-2.0-only
/* PSR first-tone candidate. Stock-derived volatile setup only, no DSP/efuse APIs.
 * Requires generated psr-tfa986x-init-data.h. PCM START enables the amplifier;
 * STOP, hw_free and shutdown disable it. No full DSP speaker protection yet.
 */
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "psr-tfa986x-init-data.h"

struct psr_amp {
	struct regmap *map;
	struct device *dev;
	struct mutex lock;
	struct delayed_work stop;
	struct work_struct start;
	atomic_t requested;
	spinlock_t trigger_lock;
	bool removing;
	u16 address;
};

static bool psr_writeable(struct device *dev, unsigned int reg)
{
	unsigned int i;
	/* No efuse-control A3, efuse shadows F0..FF, raw/debug interfaces. */
	if (reg == 0xa3 || reg >= 0xf0) return false;
	if (reg == 0 || reg == 1 || reg == 2 || reg == 4 ||
	    reg == 0x0f || reg == 0xa0 || reg == 0xa1) return true;
#define CHECK_TABLE(t) for (i = 0; i < ARRAY_SIZE(t); i++) if (reg == (t)[i].reg) return true
	CHECK_TABLE(psr_tfa9865_mn1a3_optimal_muted);
	CHECK_TABLE(psr_tfa_34_common);
	CHECK_TABLE(psr_tfa_35_common);
	CHECK_TABLE(psr_tfa_34_lowgain_muted);
	CHECK_TABLE(psr_tfa_35_lowgain_muted);
#undef CHECK_TABLE
	return false;
}
static const struct regmap_config psr_regmap = {
	.reg_bits = 8, .val_bits = 16, .val_format_endian = REGMAP_ENDIAN_BIG,
	.max_register = 0xff, .cache_type = REGCACHE_NONE,
	.writeable_reg = psr_writeable,
};
static int psr_wait(struct psr_amp *a, unsigned int reg, unsigned int mask,
		    unsigned int wanted)
{
	unsigned int v, i;
	int ret;
	for (i = 0; i < 50; i++) {
		ret = regmap_read(a->map, reg, &v);
		if (ret) return ret;
		if ((v & mask) == wanted) return 0;
		usleep_range(1000, 1500);
	}
	return -ETIMEDOUT;
}
static int psr_off(struct psr_amp *a)
{
	int r1, r2, r3;
	/* Every register00 update explicitly masks exact986x AMPEbit3 off. */
	r1 = regmap_update_bits(a->map, 0x00, 0x0009, 0x0001);
	r2 = psr_wait(a, 0x12, 0xf, 0);
	r3 = regmap_update_bits(a->map, 0x04, 0x0040, 0x0040);
	if (r1 || r2 || r3)
		dev_err(a->dev, "powerdown verification failed: %d/%d/%d\n", r1, r2, r3);
	return r1 ? r1 : r2 ? r2 : r3;
}
static int psr_table(struct psr_amp *a, const struct psr_tfa_init_reg *t, size_t n)
{
	unsigned int i;
	int ret;
	for (i = 0; i < n; i++) {
		if (!psr_writeable(a->dev, t[i].reg) ||
		    (t[i].reg == 0 && (!(t[i].mask & 8) || (t[i].value & 8))))
			return -EPERM;
		ret = regmap_update_bits(a->map, t[i].reg, t[i].mask, t[i].value);
		if (ret) return ret;
	}
	return 0;
}
#define APPLY(a,t) psr_table(a,t,ARRAY_SIZE(t))

static int psr_initialize(struct psr_amp *a)
{
	unsigned int r3, r6, f0, f3, f4, key, rev, value;
	bool touched = false, keys_touched = false;
	int ret;
#define TRY(x) do { ret = (x); if (ret) goto fail; } while (0)
	TRY(regmap_read(a->map, 3, &r3));
	TRY(regmap_read(a->map, 6, &r6));
	if (r3 != 0x9866 || r6 != 0x3007) return -ENODEV;
	TRY(regmap_read(a->map, 0, &value));
	if (value & 0xa) return -EBUSY;
	/* Stock tfa98xx_init_v6 volatile I2CR reset, no GPIO toggle or MTP copy. */
	touched = true; /* also covers an ambiguous failed reset write */
	TRY(regmap_write(a->map, 0, 0x0002));
	TRY(psr_wait(a, 0, 0x0002, 0));
	TRY(regmap_update_bits(a->map, 0, 0x0009, 0));
	TRY(regmap_update_bits(a->map, 4, 0x0040, 0));
	TRY(psr_wait(a, 0x12, 0xf, 1));
	TRY(regmap_read(a->map, 0xf3, &f3));
	TRY(regmap_read(a->map, 0xf0, &f0));
	TRY(regmap_read(a->map, 0xf4, &f4));
	rev = 0x3a66;
	/* Exact tfa986x_specific derivative formula; no filename inference. */
	if ((f3 >> 11) & 3) {
		rev--;
		if (f0 & 0x200) { if (!(f4 & 0x8000)) rev |= 0x100000; }
		else rev |= 0x10000;
		if (f4 & 0x8000) rev--;
	}
	rev += ((r6 >> 8) & 3) << 20;
	dev_info(a->dev, "derived revision %x F3=%04x F0=%04x F4=%04x\n", rev,f3,f0,f4);
	if (rev != 0x13a65) { ret = -ENODEV; goto fail; }
	/* Exact stock volatile key sequence. Programming control A3 is forbidden. */
	keys_touched = true;
	TRY(regmap_write(a->map, 0x0f, 0x5a6b));
	TRY(regmap_read(a->map, 0xfb, &key));
	TRY(regmap_write(a->map, 0xa0, key ^ 0x005a));
	TRY(regmap_write(a->map, 0x0f, 0x5a6b));
	TRY(regmap_write(a->map, 0xa1, 0x005a));
	TRY(APPLY(a, psr_tfa9865_mn1a3_optimal_muted));
	TRY(psr_off(a));
	/* Stock rate table gives AUDFS8 for48kHz; exact CNT supplies FRACTDEL. */
	TRY(regmap_update_bits(a->map, 2, 0x000f, 8));
	if (a->address == 0x34) {
		TRY(APPLY(a, psr_tfa_34_common));
		TRY(APPLY(a, psr_tfa_34_lowgain_muted));
	} else {
		TRY(APPLY(a, psr_tfa_35_common));
		TRY(APPLY(a, psr_tfa_35_lowgain_muted));
	}
	/* Lock programming key after setup; never write the efuse trigger. */
	TRY(regmap_write(a->map, 0xa1, 0));
	TRY(regmap_write(a->map, 0x0f, 0));
	TRY(psr_off(a));
	{
		static const unsigned int regs[] = {0x20,0x21,0x22,0x10,0x11,0x12};
		unsigned int i;
		for (i = 0; i < ARRAY_SIZE(regs); i++) {
			TRY(regmap_read(a->map, regs[i], &value));
			dev_info(a->dev, "muted initialized reg%02x=%04x\n", regs[i], value);
		}
	}
	return 0;
fail:
	/* Relock independently even if a prior bus operation failed. */
	if (keys_touched && regmap_write(a->map, 0xa1, 0)) dev_err(a->dev, "key2 relock failed\n");
	if (keys_touched && regmap_write(a->map, 0x0f, 0)) dev_err(a->dev, "key conceal failed\n");
	if (touched) psr_off(a);
	return ret;
#undef TRY
}
static void psr_stop_work(struct work_struct *work)
{
	struct psr_amp *a = container_of(to_delayed_work(work), struct psr_amp, stop);
	int ret;
	atomic_set(&a->requested, 0);
	mutex_lock(&a->lock);
	ret = psr_off(a);
	mutex_unlock(&a->lock);
	if (!ret) dev_info(a->dev, "first-tone stop: amplifier off verified\n");
}
static int psr_hw_params(struct snd_pcm_substream *s, struct snd_pcm_hw_params *p,
			 struct snd_soc_dai *dai)
{
	return params_rate(p) == 48000 && params_channels(p) == 2 &&
		params_format(p) == SNDRV_PCM_FORMAT_S32_LE ? 0 : -EINVAL;
}
static void psr_start_work(struct work_struct *work)
{
 struct psr_amp *a = container_of(work, struct psr_amp, start);
 int ret = 0;
 mutex_lock(&a->lock);
 if (!atomic_read(&a->requested)) goto done;
 ret = regmap_update_bits(a->map, 1, 0x0004, 0x0004); /* MANSCONF */
 if (ret) goto fail;
 ret = regmap_update_bits(a->map, 0, 0x0009, 0); /* PWDN0/AMPE0 */
 if (ret) goto fail;
 ret = regmap_update_bits(a->map, 4, 0x0040, 0); /* exact stock muted wake */
 if (ret) goto fail;
 ret = psr_wait(a, 0x11, 0x0187, 0x0081); /* CLKS+PLLS, no TDMERR/TDMLUTER */
 if (ret) goto fail;
 if (!atomic_read(&a->requested)) goto fail;
 /* Sole AMPE1 write, work scheduled only by explicit PCM START trigger. */
 ret = regmap_update_bits(a->map, 0, 0x0009, 0x0008);
 if (ret) goto fail;
 if (!atomic_read(&a->requested)) { ret = -ECANCELED; goto fail; }
 goto done;
fail:
 atomic_set(&a->requested, 0);
 psr_off(a);
 if (ret) dev_err(a->dev, "first-tone start failed: %d\n", ret);
done:
 mutex_unlock(&a->lock);
}
static int psr_trigger(struct snd_pcm_substream *s, int cmd, struct snd_soc_dai *dai)
{
 struct psr_amp *a = snd_soc_component_get_drvdata(dai->component);
 unsigned long flags;
 int ret = 0;
 spin_lock_irqsave(&a->trigger_lock, flags);
 if (a->removing) { ret = -ENODEV; goto done; }
 switch (cmd) {
 case SNDRV_PCM_TRIGGER_START:
  atomic_set(&a->requested, 1);
  schedule_work(&a->start);
  break;
 case SNDRV_PCM_TRIGGER_STOP:
 case SNDRV_PCM_TRIGGER_SUSPEND:
 case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
  atomic_set(&a->requested, 0);
  mod_delayed_work(system_wq, &a->stop, 0);
  break;
 default: ret = -EINVAL;
 }
done:
 spin_unlock_irqrestore(&a->trigger_lock, flags);
 return ret;
}
static int psr_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
 const unsigned int mask = SND_SOC_DAIFMT_FORMAT_MASK |
  SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK | SND_SOC_DAIFMT_INV_MASK;
 const unsigned int wanted = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBC_CFC |
  SND_SOC_DAIFMT_NB_NF;
 return (fmt & mask) == wanted ? 0 : -EINVAL;
}
static int psr_hw_free(struct snd_pcm_substream *s, struct snd_soc_dai *dai)
{
 struct psr_amp *a = snd_soc_component_get_drvdata(dai->component);
 int ret;
 atomic_set(&a->requested, 0);
 cancel_work_sync(&a->start);
 cancel_delayed_work_sync(&a->stop);
 mutex_lock(&a->lock); ret = psr_off(a); mutex_unlock(&a->lock);
 return ret;
}
static const struct snd_soc_dai_ops psr_dai_ops = {
 .hw_params = psr_hw_params, .hw_free = psr_hw_free, .trigger = psr_trigger,
 .set_fmt = psr_set_fmt,
};
static struct snd_soc_dai_driver psr_dai = {
	.name = "psr-tfa986x-aif",
	.playback = { .stream_name = "Playback", .channels_min = 2, .channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000, .formats = SNDRV_PCM_FMTBIT_S32_LE },
	.ops = &psr_dai_ops,
};
static const struct snd_soc_dapm_widget psr_widgets[] = {
 SND_SOC_DAPM_AIF_IN("AIF RX", "Playback", 0, SND_SOC_NOPM, 0, 0),
 SND_SOC_DAPM_OUTPUT("SPK"),
};
static const struct snd_soc_dapm_route psr_routes[] = { { "SPK", NULL, "AIF RX" } };
static const struct snd_soc_component_driver psr_component = {
 .name = "psr-tfa986x-first-tone", .dapm_widgets = psr_widgets,
 .num_dapm_widgets = ARRAY_SIZE(psr_widgets), .dapm_routes = psr_routes,
 .num_dapm_routes = ARRAY_SIZE(psr_routes),
};
static int psr_probe(struct i2c_client *client)
{
	struct psr_amp *a;
	int ret;
	if (client->addr != 0x34 && client->addr != 0x35) return -ENODEV;
	a = devm_kzalloc(&client->dev, sizeof(*a), GFP_KERNEL);
	if (!a) return -ENOMEM;
	a->dev = &client->dev; a->address = client->addr;
	mutex_init(&a->lock); INIT_DELAYED_WORK(&a->stop, psr_stop_work);
	INIT_WORK(&a->start, psr_start_work); atomic_set(&a->requested, 0);
	spin_lock_init(&a->trigger_lock);
	i2c_set_clientdata(client,a);
	a->map = devm_regmap_init_i2c(client,&psr_regmap);
	if (IS_ERR(a->map)) return PTR_ERR(a->map);
	ret = devm_regulator_get_enable(&client->dev,"tfa9874_vdd");
	if (ret) return ret;
	ret = psr_initialize(a);
	if (ret) return ret;
	ret = devm_snd_soc_register_component(&client->dev,&psr_component,&psr_dai,1);
	if (ret) psr_off(a);
	return ret;
}
static void psr_shutdown(struct i2c_client *client)
{
	struct psr_amp *a = i2c_get_clientdata(client);
	unsigned long flags;
	spin_lock_irqsave(&a->trigger_lock, flags);
	a->removing = true; atomic_set(&a->requested, 0);
	spin_unlock_irqrestore(&a->trigger_lock, flags);
	cancel_work_sync(&a->start);
	cancel_delayed_work_sync(&a->stop);
	mutex_lock(&a->lock); psr_off(a); mutex_unlock(&a->lock);
}
static const struct of_device_id psr_of_match[] = { { .compatible = "pod,psr-tfa986x-first-tone" }, {} };
MODULE_DEVICE_TABLE(of,psr_of_match);
static struct i2c_driver psr_driver = { .driver = { .name = "psr-tfa986x-first-tone", .of_match_table = psr_of_match }, .probe = psr_probe, .remove = psr_shutdown, .shutdown = psr_shutdown };
module_i2c_driver(psr_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PSR stock-derived TFA986x PCM stream playback");
