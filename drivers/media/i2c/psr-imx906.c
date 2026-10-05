// SPDX-License-Identifier: GPL-2.0-only
/* PSR fixed-mode bring-up driver. No live qualification yet.
 * Mode/power data come from the same-phone capture; see FIXED-MODE.json.
 * No guessed VT pixel rate, blanking, frame interval or exposure controls.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mux/consumer.h>
#include "psr-camera-lease.h"
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include "psr-imx906-tables.h"

/* Captured CamX enum4 maps to RGGB; see ../CFA-BINARY-EVIDENCE.json.
 * Verify actual phase with a captured color target;
 * do not infer an additional Bayer permutation from orientation registers. */
#define PSR_CODE MEDIA_BUS_FMT_SRGGB10_1X10
static const s64 psr_link_freq[] = { 3268800000LL };
enum psr_rail_state { PSR_OFF, PSR_ON, PSR_UNKNOWN };
struct psr_imx906 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrls;
	struct mutex lock;
	struct v4l2_ctrl *exposure_gain[2];
	bool configured, stream_fault;
	struct regmap *map;
	struct clk *clk;
	struct mux_control *cpas;
	bool lease_busy;
	bool cpas_owned;
	struct gpio_desc *reset;
	struct regulator *vio, *vana, *vdig, *vaf, *custom1;
	enum psr_rail_state vio_state, vana_state, vdig_state, vaf_state, custom1_state;
	bool clock_enabled;
	int cleanup_error;
	struct pinctrl *pins;
	struct pinctrl_state *active, *sleep;
};

static struct psr_imx906 *to_psr(struct v4l2_subdev *sd)
{
	return container_of(sd, struct psr_imx906, sd);
}

static void psr_sleep_pins(void *data)
{
	struct psr_imx906 *s = data;
	pinctrl_select_state(s->pins, s->sleep);
}

static int psr_write(struct psr_imx906 *s, const struct psr_reg *r, size_t n)
{
	int ret;
	while (n--) {
		ret = regmap_write(s->map, r->address, r->value);
		if (ret)
			return ret;
		if (r->delay_ms)
			msleep(r->delay_ms);
		r++;
	}
	return 0;
}

static int psr_power_off(struct device *dev);

static int psr_power_on(struct device *dev)
{
	struct psr_imx906 *s = to_psr(dev_get_drvdata(dev));
	int ret;
	/* A failed regulator disable may already have consumed its reference.
	 * Do not retry or start another enable cycle with uncertain ownership. */
	if (s->cleanup_error) return s->cleanup_error;
	if (s->cpas_owned || s->clock_enabled || s->vana_state != PSR_OFF ||
	    s->vdig_state != PSR_OFF || s->vio_state != PSR_OFF ||
	    s->vaf_state != PSR_OFF || s->custom1_state != PSR_OFF) return -EBUSY;
	ret = psr_camera_try_select(s->cpas, 2);
	s->lease_busy = ret == -EBUSY;
	if (ret)
		return dev_err_probe(dev, ret, "CPAS resource lease failed\n");
	s->cpas_owned = true;
	/* Reset descriptor must be ACTIVE_LOW: logical1 means physical low. */
	ret = pinctrl_select_state(s->pins, s->active);
	if (ret) goto unwind;
	gpiod_set_value_cansleep(s->reset, 1);
	usleep_range(1000, 1500);
	ret = regulator_enable(s->vio);
	if (ret) goto unwind;
	s->vio_state = PSR_ON;
	usleep_range(6000, 6500);
	ret = regulator_enable(s->vana);
	if (ret) goto unwind;
	s->vana_state = PSR_ON;
	usleep_range(1000, 1500);
	ret = regulator_enable(s->custom1);
	if (ret) goto unwind;
	s->custom1_state = PSR_ON;
	usleep_range(1000, 1500);
	ret = regulator_enable(s->vdig);
	if (ret) goto unwind;
	s->vdig_state = PSR_ON;
	usleep_range(1000, 1500);
	ret = regulator_enable(s->vaf);
	if (ret) goto unwind;
	s->vaf_state = PSR_ON;
	usleep_range(1000, 1500);
	gpiod_set_value_cansleep(s->reset, 0);
	usleep_range(1000, 1500);
	ret = clk_prepare_enable(s->clk);
	if (ret) goto unwind;
	s->clock_enabled = true;
	usleep_range(2000, 2500);
	return 0;
unwind:
	if (psr_power_off(dev))
		dev_err(dev, "power-up unwind left uncertain resources\n");
	return ret;
}

static int psr_disable_rail(struct device *dev, struct regulator *rail,
			    enum psr_rail_state *state, const char *name)
{
	int ret;
	if (*state != PSR_ON) return 0;
	/* A late regulator-core error can occur after enable_count decreased.
	 * UNKNOWN must not be retried, nor reported as confirmed power-off. */
	ret = regulator_disable(rail);
	*state = ret ? PSR_UNKNOWN : PSR_OFF;
	if (ret) dev_err(dev, "%s disable failed (%d); ownership uncertain, no automatic retry\n", name, ret);
	return ret;
}

static int psr_power_off(struct device *dev)
{
	struct psr_imx906 *s = to_psr(dev_get_drvdata(dev));
	int ret = s->cleanup_error, err;
	if (s->clock_enabled) {
		clk_disable_unprepare(s->clk);
		s->clock_enabled = false;
		usleep_range(1000, 1500);
	}
	gpiod_set_value_cansleep(s->reset, 1);
	usleep_range(1000, 1500);
	err = psr_disable_rail(dev, s->vaf, &s->vaf_state, "vaf"); if (err && !ret) ret = err;
	usleep_range(1000, 1500);
	err = psr_disable_rail(dev, s->vdig, &s->vdig_state, "vdig"); if (err && !ret) ret = err;
	err = psr_disable_rail(dev, s->vana, &s->vana_state, "vana"); if (err && !ret) ret = err;
	err = psr_disable_rail(dev, s->custom1, &s->custom1_state, "custom1"); if (err && !ret) ret = err;
	err = psr_disable_rail(dev, s->vio, &s->vio_state, "vio"); if (err && !ret) ret = err;
	err = pinctrl_select_state(s->pins, s->sleep); if (err && !ret) ret = err;
	if (s->cpas_owned && !ret) {
		s->cpas_owned = false;
		ret = mux_control_deselect(s->cpas);
	}
	s->cleanup_error = ret;
	if (ret)
		psr_camera_lease_fault(s->cpas);
	return ret;
}

/* Fixed frame length; captured group hold, exposure and gain encoding. */
static int psr_apply_exposure(struct psr_imx906 *s, u32 exposure, u32 gain)
{
    const struct psr_reg hold = { 0x0104, 1, 0 };
    const struct psr_reg release = { 0x0104, 0, 0 };
    const struct psr_reg values[] = { { 0x0202, exposure >> 8, 0 }, { 0x0203, exposure & 255, 0 },
        { 0x0204, gain >> 8, 0 }, { 0x0205, gain & 255, 0 },
        { 0x020e, 1, 0 }, { 0x020f, 0, 0 }, };
    int ret, release_ret;
    if (exposure < 15 || exposure > 6352 || gain < 0 || gain > 15360)
        return -ERANGE;
    ret = psr_write(s, &hold, 1);
    if (!ret) ret = psr_write(s, values, ARRAY_SIZE(values));
    /* Failed I2C may still have reached hardware: always attempt release. */
    release_ret = psr_write(s, &release, 1);
    return ret ?: release_ret;
}

static int psr_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct psr_imx906 *s = container_of(ctrl->handler, struct psr_imx906, ctrls);
	struct device *dev = s->sd.dev;
	int ret;

	if (ctrl->id != V4L2_CID_EXPOSURE)
		return -EINVAL;
	if (s->stream_fault)
		return -EIO;
	if (!s->configured)
		return 0;
	ret = pm_runtime_get_if_in_use(dev);
	if (ret <= 0)
		return ret < 0 ? ret : 0;
	ret = psr_apply_exposure(s, s->exposure_gain[0]->val,
				 s->exposure_gain[1]->val);
	if (ret) {
		int off_ret;
		/* Do not consume the stream PM reference: disable_streams owns it. */
		s->stream_fault = true;
		s->configured = false;
		off_ret = psr_write(s, psr_off, ARRAY_SIZE(psr_off));
		if (off_ret)
			dev_err(dev, "control failure stream-stop also failed: %d\n", off_ret);
	}
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);
	return ret;
}

static const struct v4l2_ctrl_ops psr_ctrl_ops = { .s_ctrl = psr_set_ctrl };

static int psr_enable(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
		      u32 pad, u64 streams)
{
	struct psr_imx906 *s = to_psr(sd);
	struct device *dev = sd->dev;
	int ret;
	if (pad || streams != 1)
		return -EINVAL;
	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;
	s->stream_fault = false;
	ret = psr_write(s, psr_init, ARRAY_SIZE(psr_init));
	if (!ret) ret = psr_write(s, psr_mode0, ARRAY_SIZE(psr_mode0));
	if (!ret) {
		s->configured = true;
		ret = __v4l2_ctrl_handler_setup(&s->ctrls);
	}
	if (!ret) ret = psr_write(s, psr_on, ARRAY_SIZE(psr_on));
	if (ret) {
		int off_ret;
		s->configured = false;
		psr_write(s, psr_off, ARRAY_SIZE(psr_off));
		off_ret = pm_runtime_put_sync_suspend(dev);
		if (off_ret < 0)
			dev_err(dev, "power-down after stream error also failed: %d\n", off_ret);
	}
	return ret;
}

static int psr_disable(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
		       u32 pad, u64 streams)
{
	int ret;
	if (pad || streams != 1)
		return -EINVAL;
	to_psr(sd)->configured = false;
	to_psr(sd)->stream_fault = false;
	ret = psr_write(to_psr(sd), psr_off, ARRAY_SIZE(psr_off));
	if (ret) {
		int off_ret;
		dev_warn(sd->dev, "stream-off failed (%d); attempting runtime power-down\n", ret);
		off_ret = pm_runtime_put_sync_suspend(sd->dev);
		if (off_ret < 0)
			dev_err(sd->dev, "runtime power-down failed: %d\n", off_ret);
		/* The PM reference was consumed; let the framework clear streaming
		 * state rather than retrying and decrementing the reference twice. */
		return 0;
	}
	pm_runtime_mark_last_busy(sd->dev);
	pm_runtime_put_autosuspend(sd->dev);
	return ret;
}

static void psr_format(struct v4l2_mbus_framefmt *f)
{
	*f = (struct v4l2_mbus_framefmt) {
		.width = 8192, .height = 6144, .code = PSR_CODE,
		.field = V4L2_FIELD_NONE, .colorspace = V4L2_COLORSPACE_RAW,
		.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT, .quantization = V4L2_QUANTIZATION_FULL_RANGE,
		.xfer_func = V4L2_XFER_FUNC_NONE,
	};
}
static int psr_init_state(struct v4l2_subdev *sd, struct v4l2_subdev_state *state)
{
	psr_format(v4l2_subdev_state_get_format(state, 0));
	return 0;
}
static int psr_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
		       struct v4l2_subdev_format *f)
{
	if (f->pad) return -EINVAL;
	if (f->which == V4L2_SUBDEV_FORMAT_ACTIVE && v4l2_subdev_is_streaming(sd))
		return -EBUSY;
	psr_format(&f->format);
	*v4l2_subdev_state_get_format(state, 0) = f->format;
	return 0;
}
static int psr_enum_code(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
			struct v4l2_subdev_mbus_code_enum *c)
{
	if (c->pad || c->index) return -EINVAL;
	c->code = PSR_CODE;
	return 0;
}
static int psr_enum_size(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
			struct v4l2_subdev_frame_size_enum *f)
{
	if (f->pad || f->index || f->code != PSR_CODE) return -EINVAL;
	f->min_width = f->max_width = 8192;
	f->min_height = f->max_height = 6144;
	return 0;
}
static int psr_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
			  struct v4l2_mbus_frame_desc *fd)
{
	if (pad) return -EINVAL;
	*fd = (struct v4l2_mbus_frame_desc) {
		.type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2, .num_entries = 1,
		.entry[0] = {
			.stream = 0, .pixelcode = PSR_CODE,
			.bus.csi2 = { .vc = 0, .dt = 0x2b },
		},
	};
	/* VC0 is an unqualified first-frame hypothesis, supported by the matching
	 * official IMX906 non-HDR full-mode descriptor. Captured metadata lists
	 * [0,1]; verify complete8192x6144 frames/count before claiming operation. */
	return 0;
}
static const struct v4l2_subdev_pad_ops psr_pad_ops = {
	.get_frame_desc = psr_frame_desc,
	.enum_mbus_code = psr_enum_code, .enum_frame_size = psr_enum_size,
	.get_fmt = v4l2_subdev_get_fmt, .set_fmt = psr_set_fmt,
	.enable_streams = psr_enable, .disable_streams = psr_disable,
};
static const struct v4l2_subdev_video_ops psr_video_ops = { .s_stream = v4l2_subdev_s_stream_helper };
static const struct v4l2_subdev_ops psr_ops = { .pad = &psr_pad_ops, .video = &psr_video_ops };
static const struct v4l2_subdev_internal_ops psr_internal_ops = { .init_state = psr_init_state };
static const struct regmap_config psr_regmap = { .reg_bits = 16, .val_bits = 8 };

static int psr_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_CPHY };
	struct fwnode_handle *endpoint;
	struct psr_imx906 *s;
	struct v4l2_ctrl *link;
	unsigned int hi, lo;
	int ret;
	if (client->addr != 0x1a) return -EINVAL;
	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) return -EINVAL;
	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep);
	fwnode_handle_put(endpoint);
	if (!ret && (ep.bus.mipi_csi2.num_data_lanes != 3 ||
	    ep.nr_of_link_frequencies != 1 || ep.link_frequencies[0] != psr_link_freq[0]))
		ret = -EINVAL;
	if (!ret && (ep.bus.mipi_csi2.data_lanes[0] != 0 ||
	    ep.bus.mipi_csi2.data_lanes[1] != 1 || ep.bus.mipi_csi2.data_lanes[2] != 2))
		ret = -EINVAL;
	v4l2_fwnode_endpoint_free(&ep);
	if (ret) return ret;
	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s) return -ENOMEM;
	v4l2_i2c_subdev_init(&s->sd, client, &psr_ops);
	s->sd.internal_ops = &psr_internal_ops;
	s->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	s->map = devm_regmap_init_i2c(client, &psr_regmap);
	if (IS_ERR(s->map)) return PTR_ERR(s->map);
	s->cpas = devm_mux_control_get(dev, "cpas");
	if (IS_ERR(s->cpas)) return dev_err_probe(dev, PTR_ERR(s->cpas), "CPAS mux unavailable\n");
	s->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(s->clk)) return PTR_ERR(s->clk);
	ret = clk_set_rate(s->clk, 19200000);
	if (ret || clk_get_rate(s->clk) != 19200000) return ret ?: -EINVAL;
	s->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(s->reset)) return PTR_ERR(s->reset);
	s->pins = devm_pinctrl_get(dev);
	if (IS_ERR(s->pins)) return PTR_ERR(s->pins);
	/* Driver core's default must be inert; only runtime resume selects active. */
	s->active = pinctrl_lookup_state(s->pins, "active");
	s->sleep = pinctrl_lookup_state(s->pins, "sleep");
	if (IS_ERR(s->active)) return PTR_ERR(s->active);
	if (IS_ERR(s->sleep)) return PTR_ERR(s->sleep);
	ret = devm_add_action_or_reset(dev, psr_sleep_pins, s);
	if (ret) return ret;
	s->vio = devm_regulator_get(dev, "vio");
	s->vana = devm_regulator_get(dev, "vana");
	s->vdig = devm_regulator_get(dev, "vdig");
	s->vaf = devm_regulator_get(dev, "vaf");
	s->custom1 = devm_regulator_get(dev, "custom1");
	if (IS_ERR(s->vio)) return PTR_ERR(s->vio);
	if (IS_ERR(s->vana)) return PTR_ERR(s->vana);
	if (IS_ERR(s->vdig)) return PTR_ERR(s->vdig);
	if (IS_ERR(s->vaf)) return PTR_ERR(s->vaf);
	if (IS_ERR(s->custom1)) return PTR_ERR(s->custom1);
	ret = regulator_set_voltage(s->vio, 1800000, 1800000);
	if (!ret) ret = regulator_set_voltage(s->custom1, 1800000, 1800000);
	if (!ret) ret = regulator_set_load(s->custom1, 3000);
	if (!ret) ret = regulator_set_voltage(s->vdig, 1096000, 1104000);
	if (!ret) ret = regulator_set_load(s->vdig, 12000);
	if (!ret) ret = regulator_set_voltage(s->vana, 2800000, 2800000);
	if (!ret) ret = regulator_set_voltage(s->vaf, 2800000, 2800000);
	if (!ret) ret = regulator_set_load(s->vio, 6000);
	if (!ret) ret = regulator_set_load(s->vana, 50000);
	if (!ret) ret = regulator_set_load(s->vaf, 50000);
	if (ret) return ret;
	ret = psr_power_on(dev);
	if (ret == -EBUSY && s->lease_busy)
		return dev_err_probe(dev, -EPROBE_DEFER, "camera lease temporarily busy\n");
	if (ret) return ret;
	ret = regmap_read(s->map, 0x0016, &hi);
	if (!ret) ret = regmap_read(s->map, 0x0017, &lo);
	if (!ret && ((hi << 8) | lo) != 0x0906) ret = -ENODEV;
	{
		int off_ret = psr_power_off(dev);
		if (!ret) ret = off_ret;
	}
	if (ret) return dev_err_probe(dev, ret, "IMX906 identity check failed\n");
	mutex_init(&s->lock);
    s->sd.state_lock = &s->lock;
    v4l2_ctrl_handler_init(&s->ctrls, 4);
    s->ctrls.lock = &s->lock;
	link = v4l2_ctrl_new_int_menu(&s->ctrls, NULL, V4L2_CID_LINK_FREQ, 0, 0, psr_link_freq);
	if (link) link->flags |= V4L2_CTRL_FLAG_READ_ONLY;
    s->exposure_gain[0] = v4l2_ctrl_new_std(&s->ctrls, &psr_ctrl_ops,
        V4L2_CID_EXPOSURE, 15, 6352, 1, 6352);
    s->exposure_gain[1] = v4l2_ctrl_new_std(&s->ctrls, &psr_ctrl_ops,
        V4L2_CID_ANALOGUE_GAIN, 0, 15360, 1, 0);
    link = v4l2_ctrl_new_std(&s->ctrls, NULL, V4L2_CID_DIGITAL_GAIN,
        256, 256, 1, 256);
    if (link) link->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	ret = s->ctrls.error;
	if (ret) goto controls;
	v4l2_ctrl_cluster(2, s->exposure_gain);
	s->sd.ctrl_handler = &s->ctrls;
	s->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&s->sd.entity, 1, &s->pad);
	if (ret) goto controls;
	ret = v4l2_subdev_init_finalize(&s->sd);
	if (ret) goto entity;
	pm_runtime_set_suspended(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);
	ret = v4l2_async_register_subdev_sensor(&s->sd);
	if (!ret) return 0;
	pm_runtime_disable(dev);
	v4l2_subdev_cleanup(&s->sd);
entity:
	media_entity_cleanup(&s->sd.entity);
controls:
	v4l2_ctrl_handler_free(&s->ctrls);
	return ret;
}
static void psr_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct psr_imx906 *s = to_psr(sd);
	v4l2_async_unregister_subdev(sd);
	pm_runtime_disable(&client->dev);
	if (psr_power_off(&client->dev))
		dev_err(&client->dev, "removal could not confirm all resources powered off\n");
	pm_runtime_set_suspended(&client->dev);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&s->ctrls);
}
static void psr_shutdown(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct psr_imx906 *s = to_psr(sd);
	int ret;

	mutex_lock(&s->lock);
	if (s->configured) {
		ret = psr_write(s, psr_off, ARRAY_SIZE(psr_off));
		if (ret) dev_err(&client->dev, "shutdown stream-stop failed %d\n", ret);
		s->configured = false;
	}
	if (s->cpas_owned) {
		ret = psr_power_off(&client->dev);
		if (ret) dev_err(&client->dev, "shutdown power-off failed %d\n", ret);
	}
	mutex_unlock(&s->lock);
}

static DEFINE_RUNTIME_DEV_PM_OPS(psr_pm_ops, psr_power_off, psr_power_on, NULL);
static const struct of_device_id psr_of_match[] = { { .compatible = "pod,psr-imx906-fixed" }, {} };
MODULE_DEVICE_TABLE(of, psr_of_match);
static struct i2c_driver psr_driver = {
	.driver = { .name = "psr-imx906-fixed", .of_match_table = psr_of_match, .pm = pm_ptr(&psr_pm_ops),
		.suppress_bind_attrs = true },
	.probe = psr_probe, .remove = psr_remove, .shutdown = psr_shutdown,
};
module_i2c_driver(psr_driver);
MODULE_DESCRIPTION("PSR captured IMX906 fixed-mode bring-up");
MODULE_LICENSE("GPL");
