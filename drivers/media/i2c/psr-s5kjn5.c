// SPDX-License-Identifier: GPL-2.0-only
/* PSR fixed-mode bring-up driver. No live qualification yet.
 * Mode/power data come from the same-phone capture; see tele/FIXED-MODE.json.
 * Captured bounded exposure/gain controls; no guessed VT pixel rate, blanking or frame interval.
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
#include "psr-s5kjn5-tables.h"

/* Diagnostic gate: never arm via an early kernel command line. One write and
 * one probe attempt per boot; a failed attempt cannot silently retry hardware.
 * Driver-core default pinctrl precedes probe and must remain the inert state.
 */
#define PSR_TELE_GET(dev, label, operation) ({ \
	typeof(operation) __resource; \
	dev_info(dev, "PSR tele resource %s begin\n", label); \
	__resource = (operation); \
	dev_info(dev, "PSR tele resource %s end ret=%ld\n", label, \
		 IS_ERR(__resource) ? PTR_ERR(__resource) : 0L); \
	__resource; \
})

#define PSR_TELE_STEP(dev, label, operation) ({ \
	int __step_ret; \
	dev_info(dev, "PSR tele stage %s begin\n", label); \
	__step_ret = (operation); \
	dev_info(dev, "PSR tele stage %s end ret=%d\n", label, __step_ret); \
	__step_ret; \
})

static atomic_t psr_probe_gate = ATOMIC_INIT(0);

static int psr_arm_probe(const char *val, const struct kernel_param *kp)
{
	if (system_state != SYSTEM_RUNNING)
		return -EPERM;
	if (!sysfs_streq(val, "cci1-master0-002d"))
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
MODULE_PARM_DESC(probe_once, "Write cci1-master0-002d once after userspace starts; read 0=closed 1=armed 2=consumed");

static int psr_enter_probe(struct device *dev, unsigned int address)
{
	struct device_node *expected;
	bool matches;

	if (address != 0x2d || !dev->of_node)
		return -ENODEV;
	expected = of_find_node_by_path("/soc@0/cci@941c000/i2c-bus@0/camera@2d");
	matches = expected && expected == dev->of_node;
	of_node_put(expected);
	if (!matches)
		return -ENODEV;
	if (atomic_cmpxchg(&psr_probe_gate, 1, 2) != 1) {
		dev_info(dev, "PSR tele probe gate closed; no sensor resources acquired\n");
		return -ENODEV;
	}
	dev_info(dev, "PSR tele probe gate consumed for CCI1/master0/0x2d\n");
	return 0;
}

/* Fixed diagnostic control, built in with this sensor. Storage intentionally
 * lasts for the boot: notifier failure may leave a populated device behind. */
#define PSR_TELE_NODE "/soc@0/cci@941c000/i2c-bus@0/camera@2d"
enum psr_activation_state { PSR_CLOSED, PSR_QUEUED, PSR_APPLYING,
	PSR_BOUND, PSR_FAILED };
static atomic_t psr_activation = ATOMIC_INIT(PSR_CLOSED);
static int psr_activation_error;
static DECLARE_COMPLETION(psr_probe_finished);
static int psr_probe_result = -EINPROGRESS;
static struct of_changeset psr_activation_changeset;
static struct device_node *psr_activation_node;

struct psr_property_pin { const char *node, *name; const u8 *data; size_t size; };
#include "psr-tele-activation-pins.h"

static int psr_validate_activation(void)
{
	struct device_node *node;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct property *prop;
	int length, i, ret = 0;

	for (i = 0; i < ARRAY_SIZE(psr_available_nodes); i++) {
		node = of_find_node_by_path(psr_available_nodes[i]);
		if (!node) return -ENODEV;
		if (!of_device_is_available(node)) ret = -ENODEV;
		if (!strcmp(psr_available_nodes[i], "/soc@0/rsc@18900000/regulators-0/ldo16") &&
		    (of_property_read_bool(node, "regulator-boot-on") ||
		     of_property_read_bool(node, "regulator-always-on"))) ret = -EINVAL;
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
	node = of_find_node_by_path(PSR_TELE_NODE);
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
	psr_activation_node = of_find_node_by_path(PSR_TELE_NODE);
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
		!strcmp(client->dev.driver->name, "psr-s5kjn5-fixed");
	device_unlock(&client->dev);
	put_device(&client->dev);
	if (time_after_eq(jiffies, deadline)) { ret = -ETIMEDOUT; goto failed; }
	if (!bound) { ret = -ENODEV; goto failed; }
	WRITE_ONCE(psr_activation_error, 0);
	atomic_set(&psr_activation, PSR_BOUND);
	pr_info("PSR tele activation bound; chip-ID probe complete, capture graph prepared\n");
	return;
failed:
	/* Revoke an unconsumed arm so a delayed probe cannot start sensor I/O.
	 * A probe already in progress stays consumed; core work is not canceled. */
	atomic_cmpxchg(&psr_probe_gate, 1, 2);
	WRITE_ONCE(psr_activation_error, ret);
	atomic_set(&psr_activation, PSR_FAILED);
	pr_err("PSR tele activation failed %d; consumed until reboot, no rollback\n", ret);
}
static DECLARE_WORK(psr_activation_work, psr_activate_work);

static int psr_activate_once(const char *val, const struct kernel_param *kp)
{
	int ret;
	if (system_state != SYSTEM_RUNNING) return -EPERM;
	if (!sysfs_streq(val, "create-cci1-master0-002d")) return -EINVAL;
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

/* Captured CFA enum1 maps to Android BGGR; CFA-BINARY-EVIDENCE.json. Verify actual phase with a captured color target;
 * do not additionally XOR the saved 0101 orientation bits into this metadata. */
#define PSR_CODE MEDIA_BUS_FMT_SBGGR10_1X10
static const s64 psr_link_freq[] = { 1996800000 };
struct psr_s5kjn5 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrls;
	struct mutex lock;
	struct v4l2_ctrl *exposure_gain[2];
	bool configured, stream_fault;
	struct regmap *map;
	struct i2c_client *client;
	struct clk *clk;
	struct mux_control *cpas;
	bool cpas_owned, cpas_fault;
	struct gpio_desc *reset;
	struct regulator *vio, *vana, *vaf, *vdig;
	bool vio_on, vana_on, vaf_on, vdig_on, clock_on;
	unsigned int uncertain_rails;
	struct pinctrl *pins;
	struct pinctrl_state *active, *sleep;
};

static struct psr_s5kjn5 *to_psr(struct v4l2_subdev *sd)
{
	return container_of(sd, struct psr_s5kjn5, sd);
}

static void psr_sleep_pins(void *data)
{
	struct psr_s5kjn5 *s = data;
	pinctrl_select_state(s->pins, s->sleep);
}

/* Captured WRITE_SEQUENTIAL arrays are expanded to explicit byte addresses.
 * Each transfer is at most4 bytes, below native CCI's11-byte message limit.
 * Width and byte order are preserved; no sensor memory payload decompression.
 */
static int psr_write(struct psr_s5kjn5 *s, const struct psr_reg *r, size_t n)
{
	int ret, bytes;
	u8 buf[4];
	while (n--) {
		if (r->data_bytes != 1 && r->data_bytes != 2)
			return -EINVAL;
		buf[0] = r->address >> 8; buf[1] = r->address;
		if (r->data_bytes == 2) {
			buf[2] = r->value >> 8; buf[3] = r->value;
		} else {
			buf[2] = r->value;
		}
		bytes = 2 + r->data_bytes;
		ret = i2c_master_send(s->client, buf, bytes);
		if (ret != bytes)
			return ret < 0 ? ret : -EIO;
		if (r->delay_us)
			usleep_range(r->delay_us, r->delay_us + 50);
		r++;
	}
	return 0;
}

/* regulator_disable may fail after consuming this driver's reference. Never
 * retry an uncertain disable and risk decrementing another consumer's vote. */
static int psr_disable_rail(struct psr_s5kjn5 *s, struct regulator *reg,
                           bool *owned, unsigned int bit)
{
	int ret;
	if (!*owned)
		return 0;
	*owned = false;
	dev_info(s->sd.dev, "PSR tele rail-disable bit=%u begin\n", bit);
	ret = regulator_disable(reg);
	dev_info(s->sd.dev, "PSR tele rail-disable bit=%u end ret=%d\n", bit, ret);
	if (ret) {
		s->uncertain_rails |= bit;
		dev_err(s->sd.dev, "regulator disable failed (%d), uncertain mask %#x; restart blocked\n",
			ret, s->uncertain_rails);
	}
	return ret;
}

static int psr_power_off(struct device *dev)
{
	struct psr_s5kjn5 *s = to_psr(dev_get_drvdata(dev));
	int ret = s->uncertain_rails ? -EIO : 0, err;
	if (s->clock_on) { s->clock_on = false;
		dev_info(dev, "PSR tele clock-disable begin\n");
		clk_disable_unprepare(s->clk);
		dev_info(dev, "PSR tele clock-disable complete (void API)\n");
	}
	usleep_range(1000, 1500);
	dev_info(dev, "PSR tele reset assert begin\n");
	gpiod_set_value_cansleep(s->reset, 1);
	dev_info(dev, "PSR tele reset assert complete (void API)\n");
	usleep_range(1000, 1500);
	err = psr_disable_rail(s, s->vdig, &s->vdig_on, BIT(3)); if (err && !ret) ret = err;
	err = psr_disable_rail(s, s->vana, &s->vana_on, BIT(1)); if (err && !ret) ret = err;
	err = psr_disable_rail(s, s->vaf, &s->vaf_on, BIT(2)); if (err && !ret) ret = err;
	err = psr_disable_rail(s, s->vio, &s->vio_on, BIT(0)); if (err && !ret) ret = err;
	err = PSR_TELE_STEP(dev, "pins-sleep", pinctrl_select_state(s->pins, s->sleep)); if (err && !ret) ret = err;
	if (s->cpas_owned && !ret) {
		s->cpas_owned = false;
		ret = PSR_TELE_STEP(dev, "mux-deselect", mux_control_deselect(s->cpas));
		if (ret) s->cpas_fault = true;
	}
	return ret;
}

static int psr_power_on(struct device *dev)
{
	struct psr_s5kjn5 *s = to_psr(dev_get_drvdata(dev));
	int ret, off_ret;
	if (s->cpas_owned || s->cpas_fault || s->uncertain_rails || s->vio_on || s->vana_on || s->vaf_on || s->vdig_on || s->clock_on)
		return -EIO;
	ret = PSR_TELE_STEP(dev, "mux-select", mux_control_select(s->cpas, 2));
	if (ret)
		return dev_err_probe(dev, ret, "CPAS resource lease failed\n");
	s->cpas_owned = true;
	ret = PSR_TELE_STEP(dev, "pins-active", pinctrl_select_state(s->pins, s->active));
	if (ret) goto unwind;
	dev_info(dev, "PSR tele reset assert begin\n");
	gpiod_set_value_cansleep(s->reset, 1);
	dev_info(dev, "PSR tele reset assert complete (void API)\n");
	usleep_range(3000, 3500);
	ret = PSR_TELE_STEP(dev, "enable-vio", regulator_enable(s->vio)); if (ret) goto unwind;
	s->vio_on = true; usleep_range(1000, 1500);
	ret = PSR_TELE_STEP(dev, "enable-vaf", regulator_enable(s->vaf)); if (ret) goto unwind;
	s->vaf_on = true; usleep_range(1000, 1500);
	ret = PSR_TELE_STEP(dev, "enable-vana", regulator_enable(s->vana)); if (ret) goto unwind;
	s->vana_on = true; usleep_range(1000, 1500);
	ret = PSR_TELE_STEP(dev, "enable-vdig", regulator_enable(s->vdig)); if (ret) goto unwind;
	s->vdig_on = true; usleep_range(6000, 6500);
	dev_info(dev, "PSR tele reset release begin\n");
	gpiod_set_value_cansleep(s->reset, 0);
	dev_info(dev, "PSR tele reset release complete (void API)\n");
	usleep_range(1000, 1500);
	ret = PSR_TELE_STEP(dev, "clock-enable", clk_prepare_enable(s->clk)); if (ret) goto unwind;
	s->clock_on = true; usleep_range(15000, 15500);
	return 0;
unwind:
	off_ret = psr_power_off(dev);
	if (off_ret) dev_err(dev, "power-up rollback also failed: %d\n", off_ret);
	return ret;
}

/* PSR metadata: FLL3204, margin10, minimum5; analogue gain is1/32x.
 * Fixed FLL avoids the OEM long-exposure shift path. Digital gain is1x.
 * Caller holds the shared subdev/control lock. Always release hold, including
 * ambiguous I2C failures. Any failure stops this stream and invalidates init.
 */
static int psr_apply_exposure(struct psr_s5kjn5 *s, u32 exposure, u32 gain)
{
	const struct psr_reg page = { 0xfcfc, 0x4000, 2, 0 };
	const struct psr_reg hold = { 0x0104, 1, 1, 0 };
	const struct psr_reg release = { 0x0104, 0, 1, 0 };
	const struct psr_reg values[] = {
		{ 0x0202, exposure, 2, 0 },
		{ 0x0204, gain, 2, 0 },
		{ 0x020e, 0x0100, 2, 0 },
	};
	int ret, release_ret;

	if (exposure < 5 || exposure > 3194 || gain < 32 || gain > 5120)
		return -ERANGE;
	ret = psr_write(s, &page, 1);
	if (ret)
		return ret;
	ret = psr_write(s, &hold, 1);
	if (!ret)
		ret = psr_write(s, values, ARRAY_SIZE(values));
	/* Even a failed hold write may have reached the sensor. */
	release_ret = psr_write(s, &release, 1);
	return ret ?: release_ret;
}

/* Captured PSR tele TestPatternInfo contains WORD writes 0600=0..3.
 * Same-ID upstream S5KJN5 names these disabled/solid/bars/fade. No PN9
 * is exposed because it is absent from this phone's captured module.
 */
static const char * const psr_test_pattern_menu[] = {
	"Disabled", "Solid Color", "Color Bars", "Fade To Grey Color Bars",
};

static int psr_apply_test_pattern(struct psr_s5kjn5 *s, u32 pattern)
{
	const struct psr_reg values[] = {
		{ 0xfcfc, 0x4000, 2, 0 },
		{ 0x0600, pattern, 2, 0 },
	};

	if (pattern >= ARRAY_SIZE(psr_test_pattern_menu))
		return -ERANGE;
	return psr_write(s, values, ARRAY_SIZE(values));
}

/* Diagnostic only: the existing init/mode/control transaction leaves page
 * 4000 selected. regmap has no cache, so each successful read is physical.
 * Read failures are logged without changing the stream callback's result.
 * No retries, page writes, PM references or actuator/OIS operations here.
 */
static void psr_readback(struct psr_s5kjn5 *s, bool started)
{
	static const unsigned int configured[] = {
		0x0112, 0x0114, 0x0340, 0x0342, 0x034c, 0x034e,
		0x0202, 0x0204, 0x020e, 0x0600,
	};
	unsigned int i, value, reg;
	int ret;

	for (i = 0; i < (started ? 1 : ARRAY_SIZE(configured)); i++) {
		reg = started ? 0x0100 : configured[i];
		value = 0;
		ret = regmap_read(s->map, reg, &value);
		dev_info(s->sd.dev,
			 "PSR tele diagnostic readback started=%u page=4000 reg=%04x ret=%d value=%04x\n",
			 started, reg, ret, value);
	}
}

static int psr_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct psr_s5kjn5 *s = container_of(ctrl->handler, struct psr_s5kjn5, ctrls);
	struct device *dev = s->sd.dev;
	int ret;

	if (ctrl->id != V4L2_CID_EXPOSURE &&
	    ctrl->id != V4L2_CID_TEST_PATTERN)
		return -EINVAL;
	if (s->stream_fault)
		return -EIO;
	if (!s->configured)
		return 0;
	ret = pm_runtime_get_if_in_use(dev);
	if (ret <= 0)
		return ret < 0 ? ret : 0;
	if (ctrl->id == V4L2_CID_TEST_PATTERN)
		ret = psr_apply_test_pattern(s, ctrl->val);
	else
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
	struct psr_s5kjn5 *s = to_psr(sd);
	struct device *dev = sd->dev;
	int ret;
	if (pad || streams != 1)
		return -EINVAL;
	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;
	s->stream_fault = false;
	ret = psr_write(s, psr_init, ARRAY_SIZE(psr_init));
	if (!ret) ret = psr_write(s, psr_mode11, ARRAY_SIZE(psr_mode11));
	if (!ret) {
		s->configured = true;
		ret = __v4l2_ctrl_handler_setup(&s->ctrls);
	}
	if (!ret) psr_readback(s, false);
	if (!ret) ret = psr_write(s, psr_on, ARRAY_SIZE(psr_on));
	if (!ret) psr_readback(s, true);
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
		.width = 4096, .height = 2304, .code = PSR_CODE,
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
	f->min_width = f->max_width = 4096;
	f->min_height = f->max_height = 2304;
	return 0;
}
static int psr_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
                          struct v4l2_mbus_frame_desc *fd)
{
	if (pad) return -EINVAL;
	*fd = (struct v4l2_mbus_frame_desc) {
		.type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2, .num_entries = 1,
		.entry[0] = { .stream = 0, .pixelcode = PSR_CODE,
			.bus.csi2 = { .vc = 0, .dt = 0x2b } },
	};
	/* Mode11 also emits PDAF VC0/DT30, with no userspace pad here. */
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
static const struct regmap_config psr_regmap = { .reg_bits = 16, .val_bits = 16 };

static int psr_probe_inner(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_CPHY };
	struct fwnode_handle *endpoint;
	struct psr_s5kjn5 *s;
	struct v4l2_ctrl *link;
	unsigned int id;
	int ret;
	ret = psr_enter_probe(dev, client->addr);
	if (ret) return ret;
	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) return -EINVAL;
	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep);
	fwnode_handle_put(endpoint);
	if (!ret && (ep.bus.mipi_csi2.num_data_lanes != 3 ||
	    ep.nr_of_link_frequencies != 1 || ep.link_frequencies[0] != psr_link_freq[0]))
		ret = -EINVAL;
	if (!ret)
		for (int i = 0; i < 3; i++)
			if (ep.bus.mipi_csi2.data_lanes[i] != i) ret = -EINVAL;
	v4l2_fwnode_endpoint_free(&ep);
	if (ret) return ret;
	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s) return -ENOMEM;
	mutex_init(&s->lock);
	s->client = client;
	v4l2_i2c_subdev_init(&s->sd, client, &psr_ops);
	s->sd.internal_ops = &psr_internal_ops;
	s->sd.state_lock = &s->lock;
	s->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	s->map = PSR_TELE_GET(dev, "regmap", devm_regmap_init_i2c(client, &psr_regmap));
	if (IS_ERR(s->map)) return PTR_ERR(s->map);
	s->cpas = PSR_TELE_GET(dev, "mux", devm_mux_control_get(dev, "cpas"));
	if (IS_ERR(s->cpas)) return dev_err_probe(dev, PTR_ERR(s->cpas), "CPAS mux unavailable\n");
	s->clk = PSR_TELE_GET(dev, "clock", devm_clk_get(dev, NULL));
	if (IS_ERR(s->clk)) return PTR_ERR(s->clk);
	ret = PSR_TELE_STEP(dev, "clock-set-rate", clk_set_rate(s->clk, 19200000));
	if (ret || clk_get_rate(s->clk) != 19200000) return ret ?: -EINVAL;
	s->reset = PSR_TELE_GET(dev, "reset", devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH));
	if (IS_ERR(s->reset)) return PTR_ERR(s->reset);
	s->pins = PSR_TELE_GET(dev, "pinctrl", devm_pinctrl_get(dev));
	if (IS_ERR(s->pins)) return PTR_ERR(s->pins);
	/* Driver core's default must be inert; only runtime resume selects active. */
	s->active = pinctrl_lookup_state(s->pins, "active");
	s->sleep = pinctrl_lookup_state(s->pins, "sleep");
	if (IS_ERR(s->active)) return PTR_ERR(s->active);
	if (IS_ERR(s->sleep)) return PTR_ERR(s->sleep);
	ret = devm_add_action_or_reset(dev, psr_sleep_pins, s);
	if (ret) return ret;
	s->vio = PSR_TELE_GET(dev, "vio", devm_regulator_get(dev, "vio"));
	s->vana = PSR_TELE_GET(dev, "vana", devm_regulator_get(dev, "vana"));
	s->vaf = PSR_TELE_GET(dev, "vaf", devm_regulator_get(dev, "vaf"));
	s->vdig = PSR_TELE_GET(dev, "vdig", devm_regulator_get(dev, "vdig"));
	if (IS_ERR(s->vio)) return PTR_ERR(s->vio);
	if (IS_ERR(s->vana)) return PTR_ERR(s->vana);
	if (IS_ERR(s->vaf)) return PTR_ERR(s->vaf);
	if (IS_ERR(s->vdig)) return PTR_ERR(s->vdig);
	ret = PSR_TELE_STEP(dev, "voltage-vio", regulator_set_voltage(s->vio, 1800000, 1800000));
	if (!ret) ret = PSR_TELE_STEP(dev, "voltage-vana", regulator_set_voltage(s->vana, 2200000, 2200000));
	if (!ret) ret = PSR_TELE_STEP(dev, "voltage-vaf", regulator_set_voltage(s->vaf, 3300000, 3300000));
	if (!ret) ret = PSR_TELE_STEP(dev, "voltage-vdig", regulator_set_voltage(s->vdig, 1000000, 1000000));
	if (!ret) ret = PSR_TELE_STEP(dev, "load-vdig", regulator_set_load(s->vdig, 1200000));
	if (!ret) ret = PSR_TELE_STEP(dev, "load-vio", regulator_set_load(s->vio, 600000));
	if (!ret) ret = PSR_TELE_STEP(dev, "load-vana", regulator_set_load(s->vana, 300000));
	if (!ret) ret = PSR_TELE_STEP(dev, "load-vaf", regulator_set_load(s->vaf, 15000));
	if (ret) return ret;
	ret = psr_power_on(dev);
	if (ret) return ret;
	/* Captured FTM identity path selects page4000, then reads16-bit ID. */
	ret = PSR_TELE_STEP(dev, "id-select-page", regmap_write(s->map, 0xfcfc, 0x4000));
	if (!ret) ret = PSR_TELE_STEP(dev, "id-read", regmap_read(s->map, 0x0000, &id));
	if (!ret && id != 0x38e5) ret = -ENODEV;
	{
		int off_ret = psr_power_off(dev);
		if (!ret) ret = off_ret;
	}
	if (ret) return dev_err_probe(dev, ret, "S5KJN5 identity check failed\n");
	v4l2_ctrl_handler_init(&s->ctrls, 5);
	s->ctrls.lock = &s->lock;
	link = v4l2_ctrl_new_int_menu(&s->ctrls, NULL, V4L2_CID_LINK_FREQ, 0, 0, psr_link_freq);
	if (link) link->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	s->exposure_gain[0] = v4l2_ctrl_new_std(&s->ctrls, &psr_ctrl_ops,
		V4L2_CID_EXPOSURE, 5, 3194, 1, 20);
	s->exposure_gain[1] = v4l2_ctrl_new_std(&s->ctrls, &psr_ctrl_ops,
		V4L2_CID_ANALOGUE_GAIN, 32, 5120, 1, 32);
	link = v4l2_ctrl_new_std(&s->ctrls, NULL, V4L2_CID_DIGITAL_GAIN,
		256, 256, 1, 256);
	if (link) link->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	v4l2_ctrl_new_std_menu_items(&s->ctrls, &psr_ctrl_ops,
		V4L2_CID_TEST_PATTERN, ARRAY_SIZE(psr_test_pattern_menu) - 1,
		0, 0, psr_test_pattern_menu);
	if (!s->ctrls.error)
		v4l2_ctrl_cluster(ARRAY_SIZE(s->exposure_gain), s->exposure_gain);
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
	struct psr_s5kjn5 *s = to_psr(sd);
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
	struct psr_s5kjn5 *s = to_psr(sd);
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
static const struct of_device_id psr_of_match[] = { { .compatible = "pod,psr-s5kjn5-fixed" }, {} };
MODULE_DEVICE_TABLE(of, psr_of_match);
static struct i2c_driver psr_driver = {
	.driver = { .name = "psr-s5kjn5-fixed", .of_match_table = psr_of_match, .pm = pm_ptr(&psr_pm_ops),
		.suppress_bind_attrs = true },
	.probe = psr_probe, .remove = psr_remove, .shutdown = psr_shutdown,
};
module_i2c_driver(psr_driver);
MODULE_DESCRIPTION("PSR captured S5KJN5 fixed-mode bring-up");
MODULE_LICENSE("GPL");
