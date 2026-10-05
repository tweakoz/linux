// SPDX-License-Identifier: GPL-2.0-only
/* PSR fixed-mode bring-up driver. No live qualification yet.
 * Mode/power data come from the same-phone capture; see front-mode-evidence.json.
 * Bounded exposure/gain controls; no invented VT pixel rate or frame interval.
 */
#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mux/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include "psr-imx709-tables.h"

static atomic_t psr_probe_gate = ATOMIC_INIT(0);

static int psr_arm_probe(const char *val, const struct kernel_param *kp)
{
	if (system_state != SYSTEM_RUNNING)
		return -EPERM;
	if (!sysfs_streq(val, "geni-884000-0010"))
		return -EINVAL;
	return atomic_cmpxchg(&psr_probe_gate, 0, 1) == 0 ? 0 : -EBUSY;
}

static int psr_get_probe_gate(char *buf, const struct kernel_param *kp)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&psr_probe_gate));
}

static const struct kernel_param_ops psr_probe_gate_ops = {
	.set = psr_arm_probe,
	.get = psr_get_probe_gate,
};
module_param_cb(probe_once, &psr_probe_gate_ops, NULL, 0600);
MODULE_PARM_DESC(probe_once, "Write geni-884000-0010 once after userspace starts; read 0=closed 1=armed 2=consumed");

static int psr_enter_probe(struct device *dev, unsigned int address)
{
	struct device_node *expected;
	bool matches;

	if (address != 0x10 || !dev->of_node)
		return -ENODEV;
	expected = of_find_node_by_path("/soc@0/geniqup@8c0000/i2c@884000/camera@10");
	matches = expected && expected == dev->of_node;
	of_node_put(expected);
	if (!matches)
		return -ENODEV;
	if (atomic_cmpxchg(&psr_probe_gate, 1, 2) != 1) {
		dev_info(dev, "PSR front probe gate closed; no sensor resources acquired\n");
		return -ENODEV;
	}
	dev_info(dev, "PSR front probe gate consumed for geni-884000-0010\n");
	return 0;
}

/* Fixed diagnostic control, built in with this sensor. Storage intentionally
 * lasts for the boot: notifier failure may leave a populated device behind. */
#define PSR_LATE_NODE "/soc@0/geniqup@8c0000/i2c@884000/camera@10"
enum psr_activation_state { PSR_CLOSED, PSR_QUEUED, PSR_APPLYING,
	PSR_BOUND, PSR_FAILED };
static atomic_t psr_activation = ATOMIC_INIT(PSR_CLOSED);
static int psr_activation_error;
static DECLARE_COMPLETION(psr_probe_finished);
static int psr_probe_result = -EINPROGRESS;
static struct of_changeset psr_activation_changeset;
static struct device_node *psr_activation_node;

struct psr_property_pin { const char *node, *name; const u8 *data; size_t size; };
#include "psr-front-activation-pins.h"

static int psr_validate_activation(void)
{
	struct device_node *node;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct property *prop;
	int length, i, ret = 0;

	if (!of_machine_is_compatible("oneplus,infiniti")) return -ENODEV;

	for (i = 0; i < ARRAY_SIZE(psr_available_nodes); i++) {
		node = of_find_node_by_path(psr_available_nodes[i]);
		if (!node) return -ENODEV;
		if (!of_device_is_available(node)) ret = -ENODEV;
		of_node_put(node);
		if (ret) return ret;
	}
	for (i = 0; i < ARRAY_SIZE(psr_property_pins); i++) {
		const struct psr_property_pin *pin = &psr_property_pins[i];
		node = of_find_node_by_path(pin->node);
		if (!node) return -ENODEV;
		prop = of_find_property(node, pin->name, &length);
		if (!prop || length != pin->size || (length && memcmp(prop->value, pin->data, length)))
			ret = -EINVAL;
		of_node_put(node);
		if (ret) return ret;
	}
	node = of_find_node_by_path(PSR_LATE_NODE);
	if (!node) return -ENODEV;
	if (of_node_check_flag(node, OF_POPULATED)) ret = -EBUSY;
	client = of_find_i2c_device_by_node(node);
	if (client) { put_device(&client->dev); ret = -EBUSY; }
	adapter = of_find_i2c_adapter_by_node(node->parent);
	if (!adapter) ret = -ENODEV;
	else put_device(&adapter->dev);
	of_node_put(node);
	return ret;
}

static void psr_activate_work(struct work_struct *work)
{
	struct i2c_client *client;
	unsigned long deadline = jiffies + msecs_to_jiffies(30000), now;
	bool bound = false;
	int ret = psr_validate_activation();

	if (ret) goto failed;
	if (atomic_cmpxchg(&psr_probe_gate, 0, 1) != 0) { ret = -EBUSY; goto failed; }
	psr_activation_node = of_find_node_by_path(PSR_LATE_NODE);
	if (!psr_activation_node) { ret = -ENODEV; goto failed; }
	of_changeset_init(&psr_activation_changeset);
	ret = of_changeset_update_prop_string(&psr_activation_changeset,
		psr_activation_node, "status", "okay");
	if (ret) goto failed;
	atomic_set(&psr_activation, PSR_APPLYING);
	/* This call can block in driver core/notifiers. The external watchdog,
	 * not the observation timeout below, must cover that entire interval. */
	ret = of_changeset_apply(&psr_activation_changeset);
	if (ret) goto failed;
	now = jiffies;
	if (time_after_eq(now, deadline) ||
	    !wait_for_completion_timeout(&psr_probe_finished, deadline - now)) {
		ret = -ETIMEDOUT; goto failed;
	}
	ret = READ_ONCE(psr_probe_result);
	if (ret) goto failed;
	/* Probe completion precedes driver_bound(). Device lock synchronizes
	 * with the core's completion of binding; it too may block. */
	client = of_find_i2c_device_by_node(psr_activation_node);
	if (!client) { ret = -ENODEV; goto failed; }
	device_lock(&client->dev);
	bound = device_is_bound(&client->dev) && client->dev.driver &&
		!strcmp(client->dev.driver->name, "psr-imx709-fixed");
	device_unlock(&client->dev);
	put_device(&client->dev);
	if (time_after_eq(jiffies, deadline)) { ret = -ETIMEDOUT; goto failed; }
	if (!bound) { ret = -ENODEV; goto failed; }
	WRITE_ONCE(psr_activation_error, 0);
	atomic_set(&psr_activation, PSR_BOUND);
	pr_info("PSR front activation bound; chip-ID probe complete, capture graph prepared\n");
	return;
failed:
	/* Revoke an unconsumed arm so a delayed probe cannot start sensor I/O.
	 * A probe already in progress stays consumed; core work is not canceled. */
	atomic_cmpxchg(&psr_probe_gate, 1, 2);
	WRITE_ONCE(psr_activation_error, ret);
	atomic_set(&psr_activation, PSR_FAILED);
	pr_err("PSR front activation failed %d; consumed until reboot, no rollback\n", ret);
}
static DECLARE_WORK(psr_activation_work, psr_activate_work);

static int psr_activate_once(const char *val, const struct kernel_param *kp)
{
	int ret;
	if (system_state != SYSTEM_RUNNING) return -EPERM;
	if (!sysfs_streq(val, "create-geni-884000-0010")) return -EINVAL;
	if (atomic_read(&psr_probe_gate) != 0) return -EBUSY;
	ret = psr_validate_activation();
	if (ret) return ret;
	if (atomic_cmpxchg(&psr_activation, PSR_CLOSED, PSR_QUEUED) != PSR_CLOSED)
		return -EBUSY;
	if (!schedule_work(&psr_activation_work)) {
		WRITE_ONCE(psr_activation_error, -EBUSY);
		atomic_set(&psr_activation, PSR_FAILED);
		return -EBUSY;
	}
	return 0;
}
static int psr_activation_status(char *buf, const struct kernel_param *kp)
{
	return scnprintf(buf, PAGE_SIZE, "state=%d error=%d probe=%d\n",
		atomic_read(&psr_activation), READ_ONCE(psr_activation_error),
		READ_ONCE(psr_probe_result));
}
static const struct kernel_param_ops psr_activation_ops = {
	.set = psr_activate_once, .get = psr_activation_status,
};
module_param_cb(activate_once, &psr_activation_ops, NULL, 0600);
MODULE_PARM_DESC(activate_once, "One exact userspace device creation; 0 closed,1 queued,2 applying,3 bound,4 failed; no retry");



/* Captured CamX mode CFA enum2 maps to GBRG (CFA-BINARY-EVIDENCE.json).
 * Verify actual phase with a captured color target;
 * do not additionally XOR the saved 0101 orientation bits into this metadata. */
#define PSR_CODE MEDIA_BUS_FMT_SGBRG10_1X10
static const s64 psr_link_freq[] = { 375900000 };
struct psr_imx709 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrls;
	struct mutex lock;
	struct v4l2_ctrl *exposure_gain[2];
	bool configured, stream_fault;
	struct regmap *map;
	struct clk *clk;
	struct mux_control *aon;
	bool aon_owned, aon_fault;
	struct gpio_desc *reset;
	struct regulator *vio, *vana, *vaf;
	bool vio_on, vana_on, vaf_on, clock_on;
	unsigned int uncertain_rails;
	struct pinctrl *pins;
	struct pinctrl_state *active, *sleep;
};

static struct psr_imx709 *to_psr(struct v4l2_subdev *sd)
{
	return container_of(sd, struct psr_imx709, sd);
}

static void psr_sleep_pins(void *data)
{
	struct psr_imx709 *s = data;
	pinctrl_select_state(s->pins, s->sleep);
}

static int psr_write(struct psr_imx709 *s, const struct psr_reg *r, size_t n)
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

/* regulator_disable may fail after consuming this driver's reference. Never
 * retry an uncertain disable and risk decrementing another consumer's vote. */
static int psr_disable_rail(struct psr_imx709 *s, struct regulator *reg,
                           bool *owned, unsigned int bit)
{
	int ret;
	if (!*owned)
		return 0;
	*owned = false;
	ret = regulator_disable(reg);
	if (ret) {
		s->uncertain_rails |= bit;
		dev_err(s->sd.dev, "regulator disable failed (%d), uncertain mask %#x; restart blocked\n",
			ret, s->uncertain_rails);
	}
	return ret;
}

static int psr_power_off(struct device *dev)
{
	struct psr_imx709 *s = to_psr(dev_get_drvdata(dev));
	int ret = s->uncertain_rails ? -EIO : 0, err;
	if (s->clock_on) {
		s->clock_on = false;
		clk_disable_unprepare(s->clk);
	}
	gpiod_set_value_cansleep(s->reset, 1);
	err = psr_disable_rail(s, s->vaf, &s->vaf_on, BIT(2)); if (err && !ret) ret = err;
	usleep_range(1000, 1500);
	err = psr_disable_rail(s, s->vana, &s->vana_on, BIT(1)); if (err && !ret) ret = err;
	err = psr_disable_rail(s, s->vio, &s->vio_on, BIT(0)); if (err && !ret) ret = err;
	usleep_range(1000, 1500);
	err = pinctrl_select_state(s->pins, s->sleep); if (err && !ret) ret = err;
	/* Never return camera ownership while power-down is uncertain. */
	if (s->aon_owned && !ret) {
		s->aon_owned = false; /* deselect releases the framework lock on error too */
		err = mux_control_deselect(s->aon);
		if (err) { s->aon_fault = true; ret = err; }
	}
	return ret;
}

static int psr_power_on(struct device *dev)
{
	struct psr_imx709 *s = to_psr(dev_get_drvdata(dev));
	int ret, off_ret;
	if (s->aon_owned || s->aon_fault || s->uncertain_rails || s->vio_on || s->vana_on || s->vaf_on || s->clock_on)
		return -EIO;
	ret = mux_control_select(s->aon, 0);
	if (ret)
		return dev_err_probe(dev, ret, "AON camera/MCLK ownership failed\n");
	s->aon_owned = true;
	/* Active pinctrl drives PONV/GPIO6 high, as stock does before its loop. */
	ret = pinctrl_select_state(s->pins, s->active);
	if (ret)
		goto unwind;
	gpiod_set_value_cansleep(s->reset, 1);
	ret = regulator_enable(s->vio);
	if (ret) goto unwind;
	s->vio_on = true;
	usleep_range(4000, 4500);
	ret = regulator_enable(s->vana);
	if (ret) goto unwind;
	s->vana_on = true;
	/* Stock's missing VDIG is INVALID_VREG and is skipped. */
	ret = regulator_enable(s->vaf);
	if (ret) goto unwind;
	s->vaf_on = true;
	usleep_range(1000, 1500);
	gpiod_set_value_cansleep(s->reset, 0);
	usleep_range(4000, 4500);
	ret = clk_prepare_enable(s->clk);
	if (ret) goto unwind;
	s->clock_on = true;
	usleep_range(1000, 1500);
	return 0;
unwind:
	off_ret = psr_power_off(dev);
	if (off_ret)
		dev_err(dev, "power-up rollback also failed: %d\n", off_ret);
	return ret;
}

/* Captured mode1 FLL2520; module margin48 and minimum9 lines.
 * Gain register encoding is 16384 - 16384/gain; module maximum32x.
 * The mode remains fixed; digital gain stays at its captured unity value.
 * See front/controls-v2/EVIDENCE.json for the limits and pending qualification.
 */
static int psr_apply_exposure(struct psr_imx709 *s, u32 exposure, u32 gain)
{
	const struct psr_reg hold = { 0x0104, 1, 0 };
	const struct psr_reg release = { 0x0104, 0, 0 };
	const struct psr_reg values[] = {
		{ 0x3150, 0, 0 },
		{ 0x0202, exposure >> 8, 0 },
		{ 0x0203, exposure & 0xff, 0 },
		{ 0x0204, gain >> 8, 0 },
		{ 0x0205, gain & 0xff, 0 },
		{ 0x020e, 1, 0 },
		{ 0x020f, 0, 0 },
	};
	int ret, release_ret;

	if (exposure < 9 || exposure > 2472 || gain > 15872)
		return -ERANGE;
	ret = psr_write(s, &hold, 1);
	if (!ret)
		ret = psr_write(s, values, ARRAY_SIZE(values));
	/* Even a failed I2C acknowledgement may follow an accepted hold. */
	release_ret = psr_write(s, &release, 1);
	return ret ?: release_ret;
}

static int psr_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct psr_imx709 *s = container_of(ctrl->handler, struct psr_imx709, ctrls);
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
		int stop_ret;

		s->configured = false;
		s->stream_fault = true;
		stop_ret = psr_write(s, psr_off, ARRAY_SIZE(psr_off));
		if (stop_ret)
			dev_err(dev, "control failure stream-stop also failed: %d\n", stop_ret);
	}
	/* Only drop this callback's temporary reference, not the stream's. */
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);
	return ret;
}

static const struct v4l2_ctrl_ops psr_ctrl_ops = { .s_ctrl = psr_set_ctrl };

static int psr_enable(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
		      u32 pad, u64 streams)
{
	struct psr_imx709 *s = to_psr(sd);
	struct device *dev = sd->dev;
	int ret;
	if (pad || streams != 1)
		return -EINVAL;
	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;
	s->stream_fault = false;
	s->configured = false;
	ret = psr_write(s, psr_init, ARRAY_SIZE(psr_init));
	if (!ret) ret = psr_write(s, psr_mode1, ARRAY_SIZE(psr_mode1));
	if (!ret) {
		s->configured = true;
		ret = __v4l2_ctrl_handler_setup(&s->ctrls);
	}
	if (!ret) ret = psr_write(s, psr_on, ARRAY_SIZE(psr_on));
	if (ret) {
		int off_ret;
		s->configured = false;
		s->stream_fault = true;
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
		.width = 3280, .height = 2464, .code = PSR_CODE,
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
	f->min_width = f->max_width = 3280;
	f->min_height = f->max_height = 2464;
	return 0;
}
static const struct v4l2_subdev_pad_ops psr_pad_ops = {
	.enum_mbus_code = psr_enum_code, .enum_frame_size = psr_enum_size,
	.get_fmt = v4l2_subdev_get_fmt, .set_fmt = psr_set_fmt,
	.enable_streams = psr_enable, .disable_streams = psr_disable,
};
static const struct v4l2_subdev_video_ops psr_video_ops = { .s_stream = v4l2_subdev_s_stream_helper };
static const struct v4l2_subdev_ops psr_ops = { .pad = &psr_pad_ops, .video = &psr_video_ops };
static const struct v4l2_subdev_internal_ops psr_internal_ops = { .init_state = psr_init_state };
static const struct regmap_config psr_regmap = { .reg_bits = 16, .val_bits = 8 };

static int psr_probe_inner(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct fwnode_handle *endpoint;
	struct psr_imx709 *s;
	struct v4l2_ctrl *link;
	unsigned int hi, lo;
	int ret;
	ret = psr_enter_probe(dev, client->addr);
	if (ret) return ret;
	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) return -EINVAL;
	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep);
	fwnode_handle_put(endpoint);
	if (!ret && (ep.bus.mipi_csi2.num_data_lanes != 4 ||
	    ep.nr_of_link_frequencies != 1 || ep.link_frequencies[0] != psr_link_freq[0]))
		ret = -EINVAL;
	v4l2_fwnode_endpoint_free(&ep);
	if (ret) return ret;
	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s) return -ENOMEM;
	mutex_init(&s->lock);
	v4l2_i2c_subdev_init(&s->sd, client, &psr_ops);
	s->sd.state_lock = &s->lock;
	s->sd.internal_ops = &psr_internal_ops;
	s->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	s->map = devm_regmap_init_i2c(client, &psr_regmap);
	if (IS_ERR(s->map)) return PTR_ERR(s->map);
	s->aon = devm_mux_control_get(dev, "aon");
	if (IS_ERR(s->aon)) return dev_err_probe(dev, PTR_ERR(s->aon), "AON mux unavailable\n");
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
	s->vaf = devm_regulator_get(dev, "vaf");
	if (IS_ERR(s->vio)) return PTR_ERR(s->vio);
	if (IS_ERR(s->vana)) return PTR_ERR(s->vana);
	if (IS_ERR(s->vaf)) return PTR_ERR(s->vaf);
	ret = regulator_set_voltage(s->vio, 1800000, 1800000);
	if (ret) return dev_err_probe(dev, ret, "VIO voltage request failed\n");
	/* PM8010 LDO3: 2.608 V + n * 16 mV; DT caps this rail at 2.9 V.
	 * The highest supported value within that cap is 2.896 V, not 2.9 V.
	 */
	ret = regulator_set_voltage(s->vana, 2896000, 2900000);
	if (ret) return dev_err_probe(dev, ret, "VANA voltage request failed\n");
	ret = regulator_set_voltage(s->vaf, 2800000, 2800000);
	if (ret) return dev_err_probe(dev, ret, "VAF voltage request failed\n");
	if (!ret) ret = regulator_set_load(s->vio, 3000);
	if (!ret) ret = regulator_set_load(s->vana, 50000);
	if (!ret) ret = regulator_set_load(s->vaf, 50000);
	if (ret) return ret;
	dev_info(dev, "PSR front identity power-on begin\n");
	ret = psr_power_on(dev);
	dev_info(dev, "PSR front identity power-on end ret=%d\n", ret);
	if (ret) return ret;
	ret = regmap_read(s->map, 0x16, &hi);
	if (!ret) ret = regmap_read(s->map, 0x17, &lo);
	if (!ret && ((hi << 8) | lo) != 0x709) ret = -ENODEV;
	dev_info(dev, "PSR front identity compare ret=%d\n", ret);
	{
		int off_ret = psr_power_off(dev);
		dev_info(dev, "PSR front identity power-off ret=%d\n", off_ret);
		if (!ret) ret = off_ret;
	}
	if (ret) return dev_err_probe(dev, ret, "IMX709 identity check failed\n");
	v4l2_ctrl_handler_init(&s->ctrls, 4);
	s->ctrls.lock = &s->lock;
	link = v4l2_ctrl_new_int_menu(&s->ctrls, NULL, V4L2_CID_LINK_FREQ, 0, 0, psr_link_freq);
	if (link) link->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	s->exposure_gain[0] = v4l2_ctrl_new_std(&s->ctrls, &psr_ctrl_ops,
		V4L2_CID_EXPOSURE, 9, 2472, 1, 2472);
	s->exposure_gain[1] = v4l2_ctrl_new_std(&s->ctrls, &psr_ctrl_ops,
		V4L2_CID_ANALOGUE_GAIN, 0, 15872, 1, 0);
	link = v4l2_ctrl_new_std(&s->ctrls, NULL, V4L2_CID_DIGITAL_GAIN,
		256, 256, 1, 256);
	if (link) link->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	if (!s->ctrls.error) v4l2_ctrl_cluster(2, s->exposure_gain);
	ret = s->ctrls.error;
	if (ret) goto controls;
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
static int psr_probe(struct i2c_client *client)
{
 int ret = psr_probe_inner(client);
 if (atomic_read(&psr_activation) == PSR_APPLYING && client->dev.of_node == psr_activation_node) {
  WRITE_ONCE(psr_probe_result, ret);
  complete_all(&psr_probe_finished);
 }
 return ret;
}
static void psr_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct psr_imx709 *s = to_psr(sd);
	v4l2_async_unregister_subdev(sd);
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev)) psr_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&s->ctrls);
}
static void psr_shutdown(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct psr_imx709 *s = to_psr(sd);
	int ret;

	mutex_lock(&s->lock);
	if (s->configured) {
		ret = psr_write(s, psr_off, ARRAY_SIZE(psr_off));
		if (ret) dev_err(&client->dev, "shutdown stream-stop failed %d\n", ret);
		s->configured = false;
	}
	if (s->aon_owned) {
		ret = psr_power_off(&client->dev);
		if (ret) dev_err(&client->dev, "shutdown power-off failed %d\n", ret);
	}
	mutex_unlock(&s->lock);
}

static DEFINE_RUNTIME_DEV_PM_OPS(psr_pm_ops, psr_power_off, psr_power_on, NULL);
static const struct of_device_id psr_of_match[] = { { .compatible = "pod,psr-imx709-fixed" }, {} };
MODULE_DEVICE_TABLE(of, psr_of_match);
static struct i2c_driver psr_driver = {
	.driver = { .name = "psr-imx709-fixed", .of_match_table = psr_of_match, .pm = pm_ptr(&psr_pm_ops),
		.suppress_bind_attrs = true },
	.probe = psr_probe, .remove = psr_remove, .shutdown = psr_shutdown,
};
module_i2c_driver(psr_driver);
MODULE_DESCRIPTION("PSR captured IMX709 fixed-mode bring-up");
MODULE_LICENSE("GPL");
