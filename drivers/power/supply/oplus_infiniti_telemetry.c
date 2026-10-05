// SPDX-License-Identifier: GPL-2.0-only
/* Experimental CPH2749 telemetry and conservative opt-in charging.
 * Property IDs follow OnePlus oplus_hal_adsp.h. Values deliberately remain
 * raw until compared with this phone's Android values and firmware ABI.
 */
#include <linux/auxiliary_bus.h>
#include <linux/completion.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/soc/qcom/pdr.h>
#include <linux/soc/qcom/pmic_glink.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>
#include <linux/reboot.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/ktime.h>

struct inf_request {
	struct pmic_glink_hdr hdr;
	__le32 battery, property, value;
};
struct inf_response {
	struct pmic_glink_hdr hdr;
	__le32 property, value, result;
};
struct inf_notify_request {
	struct pmic_glink_hdr hdr;
	__le32 battery, power_state, low_capacity, high_capacity;
};
struct inf_notification {
	struct pmic_glink_hdr hdr;
	__le32 notification;
};
enum inf_charge_profile {
	INF_PROFILE_CONSERVATIVE = 0,
	INF_PROFILE_PD_MILD = 1,
	INF_PROFILE_PD_TO90 = 2,
	INF_PROFILE_PD_TO100 = 3,
	INF_PROFILE_PD_MID = 4,
	INF_PROFILE_PD_HIGH = 5,
	INF_PROFILE_PD_FULL = 6,
	INF_PROFILE_STOCK = 7,
};

enum inf_stock_chg_type {
	INF_STOCK_PD = 0,
	INF_STOCK_PPS = 1,
	INF_STOCK_QC = 2,
	INF_STOCK_DCP = 3,
	INF_STOCK_SDP = 4,
	INF_STOCK_CDP = 5,
};

struct inf_thermal_row {
	u32 temp_decidegC;
	u32 current_mA;
};

/* OEM SCREENOFF tables from odm charging_thermal_config_default.txt
 * (temp_decidegC, current_mA). VOOC/SVOOC/UFCS unsupported → PD fallback.
 */
static const struct inf_thermal_row inf_stock_pd[] = {
	{ 380, 3000 }, { 390, 1500 }, { 400, 1000 },
};
static const struct inf_thermal_row inf_stock_pps[] = {
	{ 390, 3000 }, { 410, 2000 },
};
static const struct inf_thermal_row inf_stock_qc[] = {
	{ 380, 3000 }, { 390, 1500 }, { 400, 1000 },
};
static const struct inf_thermal_row inf_stock_dcp[] = {
	{ 390, 1500 }, { 410, 1000 },
};
static const struct inf_thermal_row inf_stock_sdp[] = {
	{ 400, 500 },
};
static const struct inf_thermal_row inf_stock_cdp[] = {
	{ 410, 1000 },
};

/* Conservative: 4.10 V/cell float (FV write 8200), 250 mA FCC, 1 A ICL, stop 80%.
 * pd-mild: 4.20 V/cell, 500 mA FCC, 1 A ICL, stop 80%.
 * pd-to90 / pd-to100: same electrical limits as pd-mild, gauge stop 90% / 100%.
 * pd-mid: 4.30 V/cell (FV 8600), 500 mA FCC, 1 A ICL, stop 100%.
 * pd-high: 4.40 V/cell (FV 8800), 500 mA FCC, 1 A ICL, stop 95%.
 * pd-full (default): 4.48 V/cell (FV 8960), 500 mA FCC, 1 A ICL, stop 100% — stock-comparable
 *   float/gauge; auto-start when USB comes online (Android-like). After full, keep same float and
 *   auto-resume when cell V ≤ ~4.43 V (stock wired-vbatdet / ~50 mV hysteresis). Currents still
 *   capped below OEM PD tables. Manual charging_enabled=0 opts out until cable unplug.
 */
/* Stock CPH2749 (24831): sw-fv ~4480 mV/cell; wired-vbatdet normal 4430 mV; D_RECHGING_CNT=5. */
#define INF_PD_FULL_FLOAT_UV	4480000
#define INF_PD_FULL_RECHG_UV	4430000
#define INF_PD_FULL_RECHG_CNT	5

struct inf_telemetry {
	struct device *dev;
	struct power_supply *battery_psy, *usb_psy;
	struct delayed_work report_work;
	struct mutex control_lock;
	struct delayed_work charge_work;
	struct notifier_block reboot_nb;
	bool charging;
	bool full_hold;		/* pd-full: stopped at 100%, waiting for V sag */
	bool charge_opt_out;	/* user disabled; clear on USB unplug */
	u8 charge_fail_streak;	/* consecutive in-charge check fails */
	bool usb_was_online;
	unsigned int rechg_count;
	enum inf_charge_profile profile;
	enum inf_stock_chg_type stock_type;
	u32 stock_target_uA;
	u32 pd_mv;
	bool pd_auto_tried;
	unsigned long pd_request_jiffies;
	const char *charge_state;
	struct pmic_glink_client *client;
	struct mutex request_lock;
	spinlock_t state_lock;
	struct completion ack;
	bool up, pending, desynced;
	u32 opcode, property, value;
	u32 notify_count, notify_last, notify_enable_count, notify_disable_count;
	u32 notify_ack_count, notify_ack_type, notify_ack_len;
	bool otg_requested;
	u32 otg_cancel_generation;
	bool otg_test_active, otg_stopping;
	int error;
};

static void inf_callback(const void *data, size_t len, void *priv)
{
	struct inf_telemetry *t = priv;
	struct inf_response r;
	struct pmic_glink_hdr hdr;
	unsigned long flags;

	if (len < sizeof(hdr))
		return;
	memcpy(&hdr, data, sizeof(hdr));
	if (le32_to_cpu(hdr.owner) != PMIC_GLINK_OWNER_BATTMGR)
		return;
	/* Independent asynchronous messages must never complete a property RPC.
	 * Observe only: no VBUS, charging, role, or mux changes in this path.
	 */
	if (le32_to_cpu(hdr.opcode) == 0x07) {
		struct inf_notification n;
		u32 event;

		if (len != sizeof(n))
			return;
		memcpy(&n, data, sizeof(n));
		event = le32_to_cpu(n.notification);
		spin_lock_irqsave(&t->state_lock, flags);
		t->notify_count++;
		t->notify_last = event;
		if (event == 0x50) {
			t->notify_enable_count++;
			t->otg_requested = true;
		}
		if (event == 0x51) {
			t->notify_disable_count++;
			t->otg_cancel_generation++;
			t->otg_requested = false;
		}
		spin_unlock_irqrestore(&t->state_lock, flags);
		dev_info_ratelimited(t->dev, "OEM notification=0x%x type=%u (observe only)\n",
			event, le32_to_cpu(hdr.type));
		return;
	}
	if (le32_to_cpu(hdr.opcode) == 0x04) {
		/* Android accepts this opcode without a payload check. Record the
		 * actual envelope; do not invent a result field or success state.
		 */
		spin_lock_irqsave(&t->state_lock, flags);
		t->notify_ack_count++;
		t->notify_ack_type = le32_to_cpu(hdr.type);
		t->notify_ack_len = len;
		spin_unlock_irqrestore(&t->state_lock, flags);
		dev_info_ratelimited(t->dev, "OEM subscribe reply type=%u len=%zu\n",
			le32_to_cpu(hdr.type), len);
		return;
	}
	if (len != sizeof(r))
		return;
	memcpy(&r, data, sizeof(r));
	if (le32_to_cpu(r.hdr.owner) != PMIC_GLINK_OWNER_BATTMGR ||
	    le32_to_cpu(r.hdr.type) != PMIC_GLINK_REQ_RESP)
		return;
	spin_lock_irqsave(&t->state_lock, flags);
	if (t->pending && t->up && !t->desynced &&
	    le32_to_cpu(r.hdr.opcode) == t->opcode &&
	    le32_to_cpu(r.property) == t->property) {
		t->value = le32_to_cpu(r.value);
		t->error = le32_to_cpu(r.result) ? -EREMOTEIO : 0;
		t->pending = false;
		complete(&t->ack);
	}
	spin_unlock_irqrestore(&t->state_lock, flags);
}

static void inf_state(void *priv, int state)
{
	struct inf_telemetry *t = priv;
	unsigned long flags;

	spin_lock_irqsave(&t->state_lock, flags);
	t->up = state == SERVREG_SERVICE_STATE_UP;
	if (!t->up) {
		t->otg_requested = false;
		t->otg_cancel_generation++;
	}
	if (!t->up && t->pending) {
		t->error = -ENOTCONN;
		t->pending = false;
		complete(&t->ack);
	}
	/* A timeout remains latched until driver reload/reboot. The protocol
	 * has no sequence ID, so a late reply must not satisfy a later read.
	 */
	spin_unlock_irqrestore(&t->state_lock, flags);
}

static int inf_rpc(struct inf_telemetry *t, u32 opcode, u32 property,
	u32 input, u32 *value)
{
	struct inf_request req = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
	};
	unsigned long flags;
	int ret;

	/* Fixed read surface; writes only for known profile constants. */
	if (!((opcode == 0x30 && (property <= 2 || property == 7 || property == 9)) ||
	      (opcode == 0x32 && property <= 4) ||
	      (opcode == 0x301 && (property == 5 || property == 12 || property == 13 ||
		property == 15 || property == 16 ||
		property == 17 || property == 66 ||
		property == 104 || (property >= 67 && property <= 70))) ||
	      (opcode == 0x300 && (((property == 70 || property == 12 || property == 13) && input <= 1) ||
		(property == 19 && input <= 1) ||
		(property == 59 && input == 500) ||
		(property == 17 && input == 0) ||
		(property == 71 && (input == 5000 || input == 9000)) ||
		(property == 5 && input >= 100000 && input <= 3000000 &&
		 (input % 100000) == 0) ||
		(property == 66 && (input == 250000 ||
		 (input >= 100000 && input <= 3000000 &&
		  (input % 100000) == 0))) ||
		(property == 104 && (input == 8200 || input == 8400 || input == 8600 || input == 8800 || input == 8960))))))
		return -EPERM;
	req.value = cpu_to_le32(input);
	req.hdr.opcode = cpu_to_le32(opcode);
	req.property = cpu_to_le32(property);
	mutex_lock(&t->request_lock);
	reinit_completion(&t->ack);
	spin_lock_irqsave(&t->state_lock, flags);
	if (!t->up || t->desynced) {
		ret = t->desynced ? -EIO : -ENOTCONN;
		spin_unlock_irqrestore(&t->state_lock, flags);
		goto out;
	}
	t->opcode = opcode;
	t->property = property;
	t->pending = true;
	t->error = -ETIMEDOUT;
	spin_unlock_irqrestore(&t->state_lock, flags);
	ret = pmic_glink_send(t->client, &req, sizeof(req));
	if (!ret)
		wait_for_completion_timeout(&t->ack, msecs_to_jiffies(1000));
	spin_lock_irqsave(&t->state_lock, flags);
	if (!ret)
		ret = t->error;
	/* A matched firmware rejection consumed its reply and is not a
	 * transport desynchronization. Let other supported properties work.
	 * Timeouts and ambiguous send failures still fail closed.
	 */
	if (ret && ret != -EREMOTEIO)
		t->desynced = true;
	t->pending = false;
	if (!ret && value)
		*value = t->value;
	spin_unlock_irqrestore(&t->state_lock, flags);
out:
	mutex_unlock(&t->request_lock);
	return ret;
}

static int inf_read(struct inf_telemetry *t, u32 opcode, u32 property, u32 *value)
{
	return inf_rpc(t, opcode, property, 0, value);
}

static int inf_set(struct inf_telemetry *t, u32 property, u32 value)
{
	return inf_rpc(t, 0x300, property, value, NULL);
}

/* Control mutex serializes policy checks, user requests, and teardown. */
static void inf_charge_stop(struct inf_telemetry *t, const char *reason)
{
	int ret = inf_set(t, 70, 0);

	t->charging = false;
	t->full_hold = false;
	t->rechg_count = 0;
	t->charge_state = reason;
	cancel_delayed_work(&t->charge_work);
	if (ret) {
		/* A desynchronized request channel cannot await another response.
		 * Still transmit the fixed disable command, never an enable.
		 */
		struct inf_request off = {
			.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
			.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
			.hdr.opcode = cpu_to_le32(0x300),
			.property = cpu_to_le32(70),
		};
		pmic_glink_send(t->client, &off, sizeof(off));
		t->charge_state = "disable-unconfirmed";
		dev_err(t->dev, "Charging disable unconfirmed (%d); return to Android\n", ret);
	}
}

/* pd-full only: disable charge but keep watching for stock-like voltage recharge. */
static void inf_charge_enter_full_hold(struct inf_telemetry *t)
{
	int ret = inf_set(t, 70, 0);

	t->charging = false;
	t->full_hold = true;
	t->rechg_count = 0;
	t->charge_state = "full-hold-pd-full";
	if (ret) {
		struct inf_request off = {
			.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
			.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
			.hdr.opcode = cpu_to_le32(0x300),
			.property = cpu_to_le32(70),
		};
		pmic_glink_send(t->client, &off, sizeof(off));
		t->charge_state = "disable-unconfirmed";
		t->full_hold = false;
		dev_err(t->dev, "Full-hold disable unconfirmed (%d)\n", ret);
		return;
	}
	schedule_delayed_work(&t->charge_work, msecs_to_jiffies(2000));
}


static const struct inf_thermal_row *inf_stock_table(enum inf_stock_chg_type type,
	size_t *n)
{
	switch (type) {
	case INF_STOCK_PPS:
		*n = ARRAY_SIZE(inf_stock_pps);
		return inf_stock_pps;
	case INF_STOCK_QC:
		*n = ARRAY_SIZE(inf_stock_qc);
		return inf_stock_qc;
	case INF_STOCK_DCP:
		*n = ARRAY_SIZE(inf_stock_dcp);
		return inf_stock_dcp;
	case INF_STOCK_SDP:
		*n = ARRAY_SIZE(inf_stock_sdp);
		return inf_stock_sdp;
	case INF_STOCK_CDP:
		*n = ARRAY_SIZE(inf_stock_cdp);
		return inf_stock_cdp;
	case INF_STOCK_PD:
	default:
		*n = ARRAY_SIZE(inf_stock_pd);
		return inf_stock_pd;
	}
}

static u32 inf_stock_current_mA(enum inf_stock_chg_type type, s32 temp_raw)
{
	size_t n, i;
	const struct inf_thermal_row *rows = inf_stock_table(type, &n);
	/* battery_temperature_raw is decidegC * 10 (3700 == 37.0 C). */
	u32 temp_dC = temp_raw > 0 ? (u32)temp_raw / 10 : 0;
	u32 cur = rows[0].current_mA;

	for (i = 0; i < n; i++) {
		if (temp_dC >= rows[i].temp_decidegC)
			cur = rows[i].current_mA;
	}
	return cur;
}

static enum inf_stock_chg_type inf_detect_stock_type(u32 available_uA)
{
	struct power_supply *psy;
	union power_supply_propval val;

	psy = power_supply_get_by_name("ucsi-source-psy-pmic_glink.ucsi.01");
	if (psy) {
		if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_USB_TYPE, &val)) {
			switch (val.intval) {
			case POWER_SUPPLY_USB_TYPE_PD_PPS:
				power_supply_put(psy);
				return INF_STOCK_PPS;
			case POWER_SUPPLY_USB_TYPE_PD:
			case POWER_SUPPLY_USB_TYPE_PD_DRP:
			case POWER_SUPPLY_USB_TYPE_C:
				power_supply_put(psy);
				return INF_STOCK_PD;
			case POWER_SUPPLY_USB_TYPE_DCP:
				power_supply_put(psy);
				return INF_STOCK_DCP;
			case POWER_SUPPLY_USB_TYPE_CDP:
				power_supply_put(psy);
				return INF_STOCK_CDP;
			case POWER_SUPPLY_USB_TYPE_SDP:
				power_supply_put(psy);
				return INF_STOCK_SDP;
			default:
				break;
			}
		}
		power_supply_put(psy);
	}
	/* Heuristic when UCSI type is unavailable. QC not distinguishable → PD. */
	if (available_uA >= 1500000)
		return INF_STOCK_PD;
	if (available_uA >= 1000000)
		return INF_STOCK_DCP;
	return INF_STOCK_SDP;
}

static const char *inf_stock_type_name(enum inf_stock_chg_type type)
{
	switch (type) {
	case INF_STOCK_PPS: return "pps";
	case INF_STOCK_QC: return "qc";
	case INF_STOCK_DCP: return "dcp";
	case INF_STOCK_SDP: return "sdp";
	case INF_STOCK_CDP: return "cdp";
	case INF_STOCK_PD:
	default: return "pd";
	}
}

static bool inf_profile_is_float_top(enum inf_charge_profile profile)
{
	return profile == INF_PROFILE_PD_FULL || profile == INF_PROFILE_STOCK;
}


/* Recompute OEM SCREENOFF target and push FCC/ICL without a full stop/start.
 * Called from the charge worker so limits track pack temp while charging.
 */
static int inf_stock_rewalk_limits(struct inf_telemetry *t)
{
	u32 available = 0, temp = 0, icl, fcc, value;
	enum inf_stock_chg_type type;
	u32 target;
	int ret;

	ret = inf_read(t, 0x32, 4, &available);
	if (ret)
		return ret;
	ret = inf_read(t, 0x301, 67, &temp);
	if (ret)
		return ret;
	type = inf_detect_stock_type(available);
	target = inf_stock_current_mA(type, (s32)temp) * 1000U;
	if (type == t->stock_type && target == t->stock_target_uA)
		return 0;

	icl = target;
	if (icl > available)
		icl = available;
	if (t->pd_mv == 9000 && icl > 1000000)
		icl = 1000000;
	icl = (icl / 100000U) * 100000U;
	if (icl < 100000)
		icl = 100000;
	if (icl > 3000000)
		icl = 3000000;
	fcc = icl;

	ret = inf_set(t, 66, fcc);
	if (!ret)
		ret = inf_set(t, 5, icl);
	if (!ret)
		ret = inf_read(t, 0x301, 5, &value);
	if (!ret) {
		u32 lo = icl > 500000 ? 500000 : (icl / 2 > 100000 ? icl / 2 : 100000);
		u32 hi = icl;

		if (hi < lo)
			hi = lo;
		if (value < lo || value > hi)
			ret = -ERANGE;
	}
	if (!ret) {
		t->stock_type = type;
		t->stock_target_uA = target;
	}
	return ret;
}

static int inf_charge_apply_enable(struct inf_telemetry *t)
{
	u32 value, fv, fcc, icl, available = 0, temp = 0;
	int ret;

	if (t->profile == INF_PROFILE_CONSERVATIVE) {
		fv = 8200;
		fcc = 250000;
		icl = 1000000;
	} else if (t->profile == INF_PROFILE_STOCK) {
		fv = 8960;
		ret = inf_read(t, 0x32, 4, &available);
		if (ret)
			return ret;
		ret = inf_read(t, 0x301, 67, &temp);
		if (ret)
			return ret;
		t->stock_type = inf_detect_stock_type(available);
		t->stock_target_uA = inf_stock_current_mA(t->stock_type, (s32)temp) * 1000U;
		icl = t->stock_target_uA;
		if (icl > available)
			icl = available;
		if (t->pd_mv == 9000 && icl > 1000000)
			icl = 1000000;
		/* Round down to 100 mA for RPC allowlist; keep usable floor. */
		icl = (icl / 100000U) * 100000U;
		if (icl < 100000)
			icl = 100000;
		if (icl > 3000000)
			icl = 3000000;
		fcc = icl;
	} else if (t->profile == INF_PROFILE_PD_FULL) {
		fv = 8960;
		fcc = 500000;
		icl = 1000000;
	} else if (t->profile == INF_PROFILE_PD_HIGH) {
		fv = 8800;
		fcc = 500000;
		icl = 1000000;
	} else if (t->profile == INF_PROFILE_PD_MID) {
		fv = 8600;
		fcc = 500000;
		icl = 1000000;
	} else {
		fv = 8400;
		fcc = 500000;
		icl = 1000000;
	}
	ret = inf_set(t, 70, 0);
	if (!ret)
		ret = inf_set(t, 104, fv);
	if (!ret)
		ret = inf_set(t, 66, fcc);
	if (!ret)
		ret = inf_set(t, 5, icl);
	if (!ret)
		ret = inf_read(t, 0x301, 5, &value);
	if (!ret) {
		u32 lo = (t->profile == INF_PROFILE_STOCK) ?
			(icl > 500000 ? 500000 : (icl / 2 > 100000 ? icl / 2 : 100000)) :
			500000;
		u32 hi = (t->profile == INF_PROFILE_STOCK) ? icl : 1000000;

		if (hi < lo)
			hi = lo;
		if (value < lo || value > hi)
			ret = -ERANGE;
	}
	if (!ret)
		ret = inf_set(t, 70, 1);
	if (ret)
		return ret;
	t->charging = true;
	t->full_hold = false;
	t->rechg_count = 0;
	t->charge_fail_streak = 0;
	switch (t->profile) {
	case INF_PROFILE_PD_MILD:
		t->charge_state = "enabled-pd-mild";
		break;
	case INF_PROFILE_PD_TO90:
		t->charge_state = "enabled-pd-to90";
		break;
	case INF_PROFILE_PD_TO100:
		t->charge_state = "enabled-pd-to100";
		break;
	case INF_PROFILE_PD_MID:
		t->charge_state = "enabled-pd-mid";
		break;
	case INF_PROFILE_PD_HIGH:
		t->charge_state = "enabled-pd-high";
		break;
	case INF_PROFILE_PD_FULL:
		t->charge_state = "enabled-pd-full";
		break;
	case INF_PROFILE_STOCK:
		t->charge_state = "enabled-stock";
		break;
	default:
		t->charge_state = "enabled-conservative";
		break;
	}
	schedule_delayed_work(&t->charge_work, 0);
	return 0;
}

static int inf_charge_check(struct inf_telemetry *t, bool starting)
{
	u32 online, vbus, available, temp, voltage, capacity, health, present;
	u32 v_abort;
	int ret;

#define GET(op, prop, val) do { ret = inf_read(t, op, prop, &val); if (ret) return ret; } while (0)
	GET(0x32, 0, online);
	GET(0x32, 1, vbus);
	GET(0x32, 4, available);
	GET(0x301, 67, temp);
	GET(0x301, 68, capacity);
	GET(0x30, 7, voltage);
	GET(0x30, 1, health);
	GET(0x30, 2, present);
#undef GET
	/* Conservative: 4.10/4.15; mild/to90/to100: 4.20/4.25; mid: 4.30/4.35;
	 * high: 4.40/4.45; full: 4.48/4.53 (stock ~4.48 float).
	 */
	if (t->profile == INF_PROFILE_CONSERVATIVE)
		v_abort = starting ? 4100000 : 4150000;
	else if (inf_profile_is_float_top(t->profile))
		v_abort = starting ? 4480000 : 4530000;
	else if (t->profile == INF_PROFILE_PD_HIGH)
		v_abort = starting ? 4400000 : 4450000;
	else if (t->profile == INF_PROFILE_PD_MID)
		v_abort = starting ? 4300000 : 4350000;
	else
		v_abort = starting ? 4200000 : 4250000;
	{
		u32 cap_abort = 8000;

		if (t->profile == INF_PROFILE_PD_TO90)
			cap_abort = 9000;
		else if (t->profile == INF_PROFILE_PD_HIGH)
			cap_abort = 9500;
		else if (t->profile == INF_PROFILE_PD_TO100 ||
			 t->profile == INF_PROFILE_PD_MID ||
			 inf_profile_is_float_top(t->profile))
			cap_abort = 10000;
		/* Pack voltage floor: stock OCV tables go to ~2.90 V. The old 3.30 V
		 * start floor blocked Android-like recovery from deep discharge
		 * (seen live at ~3.02 V / 1%). Keep a soft dead-pack floor only.
		 */
		u32 v_min = starting ? 2800000 : 3000000;

		/* Adapter current floor: require enough for our FCC (500 mA on raised
		 * profiles / 250 mA conservative). Live PD often reports
		 * usb_current_max ≈ 900 mA; the old 1 A floor blocked enable forever
		 * (stopped-check-failed / ERANGE) even though FCC is only 500 mA.
		 */
		{
			s32 t_lo = starting ? 1500 : 1000;
			s32 t_hi = starting ? 3700 : 4000;

			/* Stock OEM tables continue to ~41 C; refuse at 42.0 C. */
			if (t->profile == INF_PROFILE_STOCK)
				t_hi = 4200;
			/* Under load, USB VBUS often sags to ~4.20 V (and briefly under)
			 * on 2 A cables/adapters. Live #32 aborted at ~4.199 V. Keep a
			 * 4.00 V floor / 5.50 V ceiling for stock bring-up.
			 */
			if (t->pd_mv == 9000 && vbus < 7000000 &&
			    time_after(jiffies, t->pd_request_jiffies + msecs_to_jiffies(5000))) {
				t->pd_mv = 5000;
				dev_info(t->dev, "PD 9000 mV contract gone (VBUS %u uV); request back to 5000 mV\n", vbus);
			}
			u32 vbus_lo = t->pd_mv == 9000 ? 8000000 : 4000000;
			u32 vbus_hi = t->pd_mv == 9000 ? 9600000 : 5500000;

			if (online != 1 || present != 1 || health != 1 ||
			    vbus < vbus_lo || vbus > vbus_hi || available < 500000 ||
			    (s32)temp < t_lo ||
			    (s32)temp >= t_hi ||
			    voltage < v_min || voltage >= v_abort)
				return -ERANGE;
			if (t->profile == INF_PROFILE_STOCK &&
			    inf_stock_current_mA(inf_detect_stock_type(available),
						 (s32)temp) < 500)
				return -ERANGE;
		}
		/* Conservative keep-alive still refuses below 5% SOC. pd-full and
		 * other raised profiles match stock: no SOC start floor — plug in
		 * and charge (PMIC handles precharge at very low V).
		 */
		if (t->profile == INF_PROFILE_CONSERVATIVE && capacity < 500)
			return -ERANGE;
		/* pd-full: stop at 100% only once cell V has reached float (~4.48 V).
		 * After voltage-recharge resume, gauge often stays at 100% while V is
		 * still ~4.36–4.43 V; allowing that window is required to top back up.
		 * Start/resume below RECHG remains allowed via the same float check.
		 */
		if (capacity >= cap_abort) {
			if (inf_profile_is_float_top(t->profile) &&
			    voltage < INF_PD_FULL_FLOAT_UV)
				; /* below float: keep / resume topping */
			else
				return -ERANGE;
		}
	}
	return 0;
}

/* True when a failed in-charge check is soft full (gauge), not a hard abort. */
static bool inf_charge_is_soft_full(struct inf_telemetry *t)
{
	u32 capacity, voltage;
	int ret;

	if (!inf_profile_is_float_top(t->profile))
		return false;
	ret = inf_read(t, 0x301, 68, &capacity);
	if (ret)
		return false;
	ret = inf_read(t, 0x30, 7, &voltage);
	if (ret)
		return false;
	/* Soft-full only after float is reached; below float, keep charging. */
	return capacity >= 10000 && voltage >= INF_PD_FULL_FLOAT_UV &&
	       voltage < 4530000;
}

/* Android-like: if USB is present and user has not opted out, start or enter full-hold. */
static bool charge_autostart;
module_param(charge_autostart, bool, 0600);

static void inf_charge_maybe_autostart(struct inf_telemetry *t)
{
	u32 online;
	int ret;

	if (!charge_autostart || t->charging || t->full_hold || t->charge_opt_out)
		return;
	ret = inf_read(t, 0x32, 0, &online);
	if (ret || online != 1)
		return;
	if (t->profile == INF_PROFILE_STOCK && t->pd_mv == 5000 && !t->pd_auto_tried) {
		u32 available = 0;

		if (!inf_read(t, 0x32, 4, &available) && available < 1000000 &&
		    inf_detect_stock_type(available) == INF_STOCK_PD) {
			t->pd_auto_tried = true;
			if (!inf_set(t, 71, 9000)) {
				t->pd_mv = 9000;
				t->pd_request_jiffies = jiffies;
				dev_info(t->dev, "PD auto request 9000 mV (5 V offer %u uA)\n", available);
				return;
			}
		}
	}
	ret = inf_charge_check(t, true);
	if (!ret) {
		ret = inf_charge_apply_enable(t);
		if (ret)
			inf_charge_stop(t, "setup-failed");
	} else if (inf_profile_is_float_top(t->profile) && inf_charge_is_soft_full(t)) {
		inf_charge_enter_full_hold(t);
	}
}

static void inf_charge_worker(struct work_struct *work)
{
	struct inf_telemetry *t = container_of(to_delayed_work(work),
		struct inf_telemetry, charge_work);
	u32 enabled, battery_current, voltage, online;
	s32 i_hi;
	int ret;

	mutex_lock(&t->control_lock);
	if (t->charging) {
		ret = inf_charge_check(t, false);
		if (!ret)
			ret = inf_read(t, 0x301, 70, &enabled);
		if (!ret)
			ret = inf_read(t, 0x30, 9, &battery_current);
		if (t->profile == INF_PROFILE_CONSERVATIVE)
			i_hi = 750000;
		else if (t->profile == INF_PROFILE_STOCK)
			i_hi = t->stock_target_uA ? (s32)t->stock_target_uA + 250000 : 3250000;
		else
			i_hi = 1000000;
		/* Infiniti battery_current_raw: negative = charge into pack.
		 * Abort on discharge while armed (> +0.5 A) or charge above i_hi.
		 */
		if (!ret) {
			s32 i = (s32)battery_current;

			if (i > 500000 || i < -i_hi)
				ret = -ERANGE;
		}
		if (ret || enabled != 1) {
			if (ret && inf_charge_is_soft_full(t)) {
				t->charge_fail_streak = 0;
				inf_charge_enter_full_hold(t);
			} else if (!ret && enabled != 1) {
				/* Firmware cleared enable — stop and let autostart retry. */
				t->charge_fail_streak = 0;
				inf_charge_stop(t, "stopped-by-firmware");
			} else {
				/* Transient gate fail (e.g. VBUS sag): debounce before un-arming
				 * active charge. Autostart keeps retrying while USB is online.
				 */
				t->charge_fail_streak++;
				if (t->charge_fail_streak >= 3) {
					inf_charge_stop(t, "stopped-check-failed");
				} else {
					schedule_delayed_work(&t->charge_work,
						msecs_to_jiffies(1000));
				}
			}
		} else {
			t->charge_fail_streak = 0;
			/* Stock: re-walk OEM thermal table as pack temp changes. */
			if (t->profile == INF_PROFILE_STOCK) {
				ret = inf_stock_rewalk_limits(t);
				if (ret) {
					/* Limits update failed: keep trying, do not un-arm intent. */
					t->charge_fail_streak++;
					if (t->charge_fail_streak >= 3) {
						inf_charge_stop(t, "stopped-rewalk-failed");
						goto worker_out;
					}
					schedule_delayed_work(&t->charge_work,
						msecs_to_jiffies(1000));
					goto worker_out;
				}
			}
			schedule_delayed_work(&t->charge_work, msecs_to_jiffies(2000));
		}
	} else if (t->full_hold && inf_profile_is_float_top(t->profile)) {
		ret = inf_read(t, 0x32, 0, &online);
		if (ret || online != 1) {
			inf_charge_stop(t, "stopped-usb-gone");
		} else {
			ret = inf_read(t, 0x30, 7, &voltage);
			if (ret) {
				inf_charge_stop(t, "stopped-check-failed");
			} else if (voltage <= INF_PD_FULL_RECHG_UV) {
				t->rechg_count++;
				if (t->rechg_count >= INF_PD_FULL_RECHG_CNT) {
					ret = inf_charge_check(t, true);
					if (!ret)
						ret = inf_charge_apply_enable(t);
					if (ret) {
						t->rechg_count = 0;
						t->charge_state = "full-hold-resume-failed";
						t->full_hold = true;
						schedule_delayed_work(&t->charge_work,
							msecs_to_jiffies(2000));
					}
				} else {
					schedule_delayed_work(&t->charge_work,
						msecs_to_jiffies(2000));
				}
			} else {
				t->rechg_count = 0;
				schedule_delayed_work(&t->charge_work,
					msecs_to_jiffies(2000));
			}
		}
	}
worker_out:
	mutex_unlock(&t->control_lock);
}

static ssize_t charging_enabled_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inf_telemetry *t = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%u\n", READ_ONCE(t->charging));
}

static ssize_t charging_enabled_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inf_telemetry *t = dev_get_drvdata(dev);
	bool enable;
	int ret = kstrtobool(buf, &enable);

	if (ret)
		return ret;
	mutex_lock(&t->control_lock);
	if (!enable) {
		t->charge_opt_out = true;
		inf_charge_stop(t, "disabled");
		ret = !strcmp(t->charge_state, "disable-unconfirmed") ? -EIO : 0;
		goto out;
	}
	t->charge_opt_out = false;
	if (t->charging)
		goto out;
	if (t->full_hold && inf_profile_is_float_top(t->profile)) {
		schedule_delayed_work(&t->charge_work, 0);
		goto out;
	}
	ret = inf_charge_check(t, true);
	if (ret) {
		if (inf_profile_is_float_top(t->profile) && inf_charge_is_soft_full(t)) {
			inf_charge_enter_full_hold(t);
			ret = 0;
		}
		goto out;
	}
	/* Disable before changing limits. Stock batt_num=2, no bidirectional
	 * CP flag: set_fv multiplies millivolts per cell by two. No fast-PD,
	 * VOOC, thermal override, gauge initialization or persistence writes.
	 * Stock uses SET acknowledgments for FCC/FV; OEM GETs return zero.
	 */
	ret = inf_charge_apply_enable(t);
	if (ret)
		inf_charge_stop(t, "setup-failed");
out:
	mutex_unlock(&t->control_lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(charging_enabled);

static ssize_t charge_opt_out_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inf_telemetry *t = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%u\n", READ_ONCE(t->charge_opt_out));
}
static DEVICE_ATTR_RO(charge_opt_out);

static ssize_t charging_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inf_telemetry *t = dev_get_drvdata(dev);
	ssize_t ret;
	mutex_lock(&t->control_lock);
	ret = sysfs_emit(buf, "%s\n", t->charge_state);
	mutex_unlock(&t->control_lock);
	return ret;
}
static DEVICE_ATTR_RO(charging_state);

static ssize_t charging_profile_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inf_telemetry *t = dev_get_drvdata(dev);
	const char *name;

	mutex_lock(&t->control_lock);
	switch (t->profile) {
	case INF_PROFILE_PD_MILD:
		name = "pd-mild";
		break;
	case INF_PROFILE_PD_TO90:
		name = "pd-to90";
		break;
	case INF_PROFILE_PD_TO100:
		name = "pd-to100";
		break;
	case INF_PROFILE_PD_MID:
		name = "pd-mid";
		break;
	case INF_PROFILE_PD_HIGH:
		name = "pd-high";
		break;
	case INF_PROFILE_PD_FULL:
		name = "pd-full";
		break;
	case INF_PROFILE_STOCK:
		name = "stock";
		break;
	default:
		name = "conservative";
		break;
	}
	mutex_unlock(&t->control_lock);
	return sysfs_emit(buf, "%s\n", name);
}

static ssize_t charging_profile_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inf_telemetry *t = dev_get_drvdata(dev);
	enum inf_charge_profile profile;
	int ret = 0;

	if (sysfs_streq(buf, "conservative"))
		profile = INF_PROFILE_CONSERVATIVE;
	else if (sysfs_streq(buf, "pd-mild"))
		profile = INF_PROFILE_PD_MILD;
	else if (sysfs_streq(buf, "pd-to90"))
		profile = INF_PROFILE_PD_TO90;
	else if (sysfs_streq(buf, "pd-to100"))
		profile = INF_PROFILE_PD_TO100;
	else if (sysfs_streq(buf, "pd-mid"))
		profile = INF_PROFILE_PD_MID;
	else if (sysfs_streq(buf, "pd-high"))
		profile = INF_PROFILE_PD_HIGH;
	else if (sysfs_streq(buf, "pd-full"))
		profile = INF_PROFILE_PD_FULL;
	else if (sysfs_streq(buf, "stock"))
		profile = INF_PROFILE_STOCK;
	else
		return -EINVAL;

	mutex_lock(&t->control_lock);
	if (t->charging) {
		ret = -EBUSY;
		goto out;
	}
	if (t->full_hold)
		inf_charge_stop(t, "profile-changed");
	t->profile = profile;
out:
	mutex_unlock(&t->control_lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(charging_profile);

static ssize_t pd_request_mv_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inf_telemetry *t = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", READ_ONCE(t->pd_mv));
}

static ssize_t pd_request_mv_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inf_telemetry *t = dev_get_drvdata(dev);
	u32 mv;
	int ret = kstrtou32(buf, 10, &mv);

	if (ret)
		return ret;
	if (mv != 5000 && mv != 9000)
		return -EINVAL;
	mutex_lock(&t->control_lock);
	if (t->charging || t->full_hold) {
		ret = -EBUSY;
		goto out;
	}
	ret = inf_set(t, 71, mv);
	if (!ret) {
		WRITE_ONCE(t->pd_mv, mv);
		t->pd_request_jiffies = jiffies;
	}
	dev_info(t->dev, "PD request %u mV result=%d\n", mv, ret);
out:
	mutex_unlock(&t->control_lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(pd_request_mv);

static int inf_reboot(struct notifier_block *nb, unsigned long event, void *data)
{
	struct inf_telemetry *t = container_of(nb, struct inf_telemetry, reboot_nb);
	WRITE_ONCE(t->otg_stopping, true);
	mutex_lock(&t->control_lock);
	if (t->charging || t->full_hold) inf_charge_stop(t, "reboot");
	if (t->pd_mv == 9000 && !inf_set(t, 71, 5000))
		t->pd_mv = 5000;
	mutex_unlock(&t->control_lock);
	return NOTIFY_DONE;
}

static enum power_supply_property inf_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS, POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT, POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP, POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_SCOPE, POWER_SUPPLY_PROP_MODEL_NAME,
};
static enum power_supply_property inf_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE, POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX, POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
};

static int inf_battery_get(struct power_supply *psy,
	enum power_supply_property prop, union power_supply_propval *val)
{
	struct inf_telemetry *t = power_supply_get_drvdata(psy);
	u32 op = 0x30, id, raw;
	int ret;
	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS: id = 0; break;
	case POWER_SUPPLY_PROP_HEALTH: id = 1; break;
	case POWER_SUPPLY_PROP_PRESENT: id = 2; break;
	case POWER_SUPPLY_PROP_CAPACITY: op = 0x301; id = 68; break;
	case POWER_SUPPLY_PROP_TEMP: op = 0x301; id = 67; break;
	case POWER_SUPPLY_PROP_TECHNOLOGY: val->intval = POWER_SUPPLY_TECHNOLOGY_LION; return 0;
	case POWER_SUPPLY_PROP_SCOPE: val->intval = POWER_SUPPLY_SCOPE_SYSTEM; return 0;
	case POWER_SUPPLY_PROP_MODEL_NAME: val->strval = "OnePlus 15"; return 0;
	default: return -EINVAL;
	}
	ret = inf_read(t, op, id, &raw);
	if (ret) return -ENODATA;
	if (prop == POWER_SUPPLY_PROP_CAPACITY) {
		if (raw > 10000) return -ENODATA;
		val->intval = DIV_ROUND_CLOSEST(raw, 100);
	} else if (prop == POWER_SUPPLY_PROP_TEMP) {
		val->intval = DIV_ROUND_CLOSEST((s32)raw, 10);
	} else {
		val->intval = raw;
	}
	return 0;
}

static int inf_usb_get(struct power_supply *psy,
	enum power_supply_property prop, union power_supply_propval *val)
{
	struct inf_telemetry *t = power_supply_get_drvdata(psy);
	u32 op = 0x32, id, raw;
	int ret;
	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE: id = 0; break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW: id = 1; break;
	case POWER_SUPPLY_PROP_CURRENT_MAX: id = 4; break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT: op = 0x301; id = 5; break;
	default: return -EINVAL;
	}
	ret = inf_read(t, op, id, &raw);
	if (ret) return -ENODATA;
	val->intval = raw;
	return 0;
}
static const struct power_supply_desc inf_battery_desc = {
	.name = "infiniti-battery", .type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = inf_battery_props, .num_properties = ARRAY_SIZE(inf_battery_props),
	.get_property = inf_battery_get,
};
static const struct power_supply_desc inf_usb_desc = {
	.name = "infiniti-usb", .type = POWER_SUPPLY_TYPE_USB,
	.properties = inf_usb_props, .num_properties = ARRAY_SIZE(inf_usb_props),
	.get_property = inf_usb_get,
};
static void inf_report_worker(struct work_struct *work)
{
	struct inf_telemetry *t = container_of(to_delayed_work(work),
		struct inf_telemetry, report_work);
	u32 online = 0;
	unsigned int delay_ms = 15000;

	inf_read(t, 0x32, 0, &online);
	mutex_lock(&t->control_lock);
	if (t->usb_was_online && online != 1) {
		/* Cable gone: allow next plug to auto-start like Android. */
		t->charge_opt_out = false;
		t->pd_mv = 5000;
		t->pd_auto_tried = false;
		if (t->charging || t->full_hold)
			inf_charge_stop(t, "stopped-usb-gone");
	}
	t->usb_was_online = (online == 1);
	inf_charge_maybe_autostart(t);
	if (!t->charging && !t->full_hold && !t->charge_opt_out)
		delay_ms = 2000; /* watch for plug / ready-to-start */
	mutex_unlock(&t->control_lock);
	power_supply_changed(t->battery_psy);
	power_supply_changed(t->usb_psy);
	schedule_delayed_work(&t->report_work, msecs_to_jiffies(delay_ms));
}
static void inf_report_remove(void *data)
{
	struct inf_telemetry *t = data;
	cancel_delayed_work_sync(&t->report_work);
}

static void inf_control_remove(void *data)
{
	struct inf_telemetry *t = data;
	WRITE_ONCE(t->otg_stopping, true);
	unregister_reboot_notifier(&t->reboot_nb);
	cancel_delayed_work_sync(&t->charge_work);
	mutex_lock(&t->control_lock);
	if (t->charging || t->full_hold) inf_charge_stop(t, "removed");
	mutex_unlock(&t->control_lock);
}

#define INF_RAW_ATTR(name, opcode, property) \
static ssize_t name##_show(struct device *dev, \
	struct device_attribute *attr, char *buf) \
{ \
	u32 value; \
	int ret = inf_read(dev_get_drvdata(dev), opcode, property, &value); \
	return ret ? ret : sysfs_emit(buf, "%u\n", value); \
} \
static DEVICE_ATTR_RO(name)

INF_RAW_ATTR(battery_status_raw, 0x30, 0);
INF_RAW_ATTR(battery_health_raw, 0x30, 1);
INF_RAW_ATTR(battery_present_raw, 0x30, 2);
/* SOCCP uses the OEM property space for these three gauge readings:
 * oplus_pmic_glink_id.h OPLUS_BATTERY_TEMP/CAPACITY/CHARGER_COUNTER.
 * Stock fg_sm8350_get_* uses this branch when soccp_support is set.
 */
INF_RAW_ATTR(battery_capacity_raw, 0x301, 68);
INF_RAW_ATTR(battery_voltage_raw, 0x30, 7);
INF_RAW_ATTR(battery_current_raw, 0x30, 9);
INF_RAW_ATTR(battery_temperature_raw, 0x301, 67);
INF_RAW_ATTR(battery_charge_counter_raw, 0x301, 69);
INF_RAW_ATTR(usb_online_raw, 0x32, 0);
INF_RAW_ATTR(usb_voltage_raw, 0x32, 1);
INF_RAW_ATTR(usb_voltage_max_raw, 0x32, 2);
INF_RAW_ATTR(usb_current_raw, 0x32, 3);
INF_RAW_ATTR(usb_current_max_raw, 0x32, 4);
INF_RAW_ATTR(usb_input_current_limit_raw, 0x301, 5);
INF_RAW_ATTR(charger_enabled_raw, 0x301, 70);
INF_RAW_ATTR(charge_current_limit_raw, 0x301, 66);
INF_RAW_ATTR(float_voltage_raw, 0x301, 104);
/* Stock SOCCP getters, matched against shipped oplus_chg_v2.ko.
 * Read-only diagnostics; no OTG, VBUS, role or charging-policy writes.
 */
INF_RAW_ATTR(otg_switch_raw, 0x301, 13);
INF_RAW_ATTR(otg_ap_enable_raw, 0x301, 12);
INF_RAW_ATTR(typec_cc_orientation_raw, 0x301, 15);
INF_RAW_ATTR(typec_cid_raw, 0x301, 16);
INF_RAW_ATTR(typec_mode_raw, 0x301, 17);

static ssize_t otg_switch_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return otg_switch_raw_show(dev, attr, buf);
}

/* Explicit diagnostic only: OEM SOCCP OTG switch, not VBUS/boost/current.
 * Never changed automatically at probe, disconnect, or shutdown.
 */
static ssize_t otg_switch_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inf_telemetry *t = dev_get_drvdata(dev);
	u32 before, after = ~0U, restored = ~0U;
	bool enable;
	int ret, rollback;

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;
	mutex_lock(&t->control_lock);
	ret = inf_read(t, 0x301, 13, &before);
	if (ret)
		goto out;
	if (before > 1) {
		ret = -EPROTO;
		goto out;
	}
	if (before == enable)
		goto out;
	ret = inf_set(t, 13, enable);
	if (!ret)
		ret = inf_read(t, 0x301, 13, &after);
	if (!ret && after != enable)
		ret = -EIO;
	if (ret) {
		rollback = inf_set(t, 13, before);
		if (!rollback)
			rollback = inf_read(t, 0x301, 13, &restored);
		if (!rollback && restored != before)
			rollback = -EIO;
		dev_err(dev, "OTG switch request=%u old=%u failed=%d rollback=%d readback=%u\n",
			enable, before, ret, rollback, restored);
	} else {
		dev_info(dev, "OTG switch verified %u->%u; connector attachment remains separate\n",
			 before, after);
	}
out:
	mutex_unlock(&t->control_lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(otg_switch);

/* Shipped oplus_otg_init_status_func writes SOCCP property 12=1 at
 * startup. This is distinct from OTG_SWITCH (13). Keep this explicit
 * until verified on hardware; no automatic writes or Type-C role writes.
 */
static ssize_t otg_ap_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return otg_ap_enable_raw_show(dev, attr, buf);
}

static ssize_t otg_ap_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inf_telemetry *t = dev_get_drvdata(dev);
	u32 before, after = ~0U, restored = ~0U;
	bool enable;
	int ret, rollback;

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;
	mutex_lock(&t->control_lock);
	ret = inf_read(t, 0x301, 12, &before);
	if (ret)
		goto out;
	if (before > 1) {
		ret = -EPROTO;
		goto out;
	}
	if (before == enable)
		goto out;
	ret = inf_set(t, 12, enable);
	if (!ret)
		ret = inf_read(t, 0x301, 12, &after);
	if (!ret && after != enable)
		ret = -EIO;
	if (ret) {
		rollback = inf_set(t, 12, before);
		if (!rollback)
			rollback = inf_read(t, 0x301, 12, &restored);
		if (!rollback && restored != before)
			rollback = -EIO;
		dev_err(dev, "OTG AP request=%u old=%u failed=%d rollback=%d readback=%u\n",
			enable, before, ret, rollback, restored);
	} else {
		dev_info(dev, "OTG AP verified %u->%u; attachment remains separate\n",
			before, after);
	}
out:
	mutex_unlock(&t->control_lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(otg_ap_enable);

/* Android sends this initialization command without a GET of property12.
 * A successful ACK is only transport acceptance, not proof of OTG/PD state.
 * Explicit action; never run automatically or infer rollback from a GET.
 */
static ssize_t otg_ap_initialize_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inf_telemetry *t = dev_get_drvdata(dev);
	bool request;
	int ret;

	ret = kstrtobool(buf, &request);
	if (ret || !request)
		return ret ? ret : -EINVAL;
	mutex_lock(&t->control_lock);
	ret = inf_set(t, 12, 1);
	mutex_unlock(&t->control_lock);
	dev_info(dev, "OTG AP initialization command result=%d; verify CC/USB/DP separately\n", ret);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(otg_ap_initialize);

/* Shipped Android AP-init path conditionally restores DRP when CID is set.
 * SET17=0 means DRP; GET17 reports live status, not that setter enum. Never
 * compare it to zero as verification or use it as a rollback role value.
 * Expose only the observed DRP command, not arbitrary source/sink/disable.
 */
static ssize_t typec_restore_drp_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inf_telemetry *t = dev_get_drvdata(dev);
	u32 cid, before, after = ~0U;
	bool request;
	int ret;

	ret = kstrtobool(buf, &request);
	if (ret || !request)
		return ret ? ret : -EINVAL;
	mutex_lock(&t->control_lock);
	ret = inf_read(t, 0x301, 16, &cid);
	if (ret)
		goto out;
	if (cid != 1) {
		ret = -EAGAIN;
		goto out;
	}
	ret = inf_read(t, 0x301, 17, &before);
	if (ret)
		goto out;
	ret = inf_set(t, 17, 0);
	if (ret)
		goto out;
	ret = inf_read(t, 0x301, 17, &after);
	dev_info(dev, "TypeC DRP command ACK; CID=%u status_before=%u status_after=%u read_result=%d; verify attachment separately\n",
		cid, before, after, ret);
out:
	mutex_unlock(&t->control_lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(typec_restore_drp);


static ssize_t stock_chg_type_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inf_telemetry *t = dev_get_drvdata(dev);
	const char *name;

	mutex_lock(&t->control_lock);
	name = inf_stock_type_name(t->stock_type);
	mutex_unlock(&t->control_lock);
	return sysfs_emit(buf, "%s\n", name);
}
static DEVICE_ATTR_RO(stock_chg_type);

static ssize_t stock_target_uA_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inf_telemetry *t = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", READ_ONCE(t->stock_target_uA));
}
static DEVICE_ATTR_RO(stock_target_uA);

static ssize_t transport_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inf_telemetry *t = dev_get_drvdata(dev);
	unsigned long flags;
	const char *state;

	spin_lock_irqsave(&t->state_lock, flags);
	state = t->desynced ? "desynchronized" : t->up ? "up" : "down";
	spin_unlock_irqrestore(&t->state_lock, flags);
	return sysfs_emit(buf, "%s\n", state);
}
static DEVICE_ATTR_RO(transport_state);

/* Exact Android subscription envelope; explicit diagnostic opt-in. A send
 * success does not prove subscription. Read notification_status for replies.
 * No automatic renewal, replay, or power actions, including after ADSP reset.
 */
static ssize_t notification_subscribe_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inf_telemetry *t = dev_get_drvdata(dev);
	struct inf_notify_request req = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(2),
		.hdr.opcode = cpu_to_le32(0x04),
	};
	unsigned long flags;
	bool request;
	int ret = kstrtobool(buf, &request);

	if (ret || !request)
		return ret ? ret : -EINVAL;
	mutex_lock(&t->request_lock);
	spin_lock_irqsave(&t->state_lock, flags);
	ret = t->desynced ? -EIO : t->up ? 0 : -ENOTCONN;
	spin_unlock_irqrestore(&t->state_lock, flags);
	if (!ret)
		ret = pmic_glink_send(t->client, &req, sizeof(req));
	mutex_unlock(&t->request_lock);
	dev_info(dev, "OEM notification subscription send=%d; observe only\n", ret);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(notification_subscribe);

static ssize_t notification_status_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inf_telemetry *t = dev_get_drvdata(dev);
	unsigned long flags;
	ssize_t ret;

	spin_lock_irqsave(&t->state_lock, flags);
	ret = sysfs_emit(buf,
		"replies=%u reply_type=%u reply_len=%u events=%u last=0x%x otg_enable=%u otg_disable=%u\n",
		t->notify_ack_count, t->notify_ack_type, t->notify_ack_len,
		t->notify_count, t->notify_last, t->notify_enable_count, t->notify_disable_count);
	spin_unlock_irqrestore(&t->state_lock, flags);
	return ret;
}
static DEVICE_ATTR_RO(notification_status);

/* Bounded diagnostic, NOT a general OTG policy. Android's PMIC/SOCCP path
 * writes boost current (59, mA), then regulator enable (19). Its voltage
 * setter is a no-op. Test only a controller-requested source attachment,
 * outside charging, with conservative battery checks. No persistent enable,
 * automatic retry, voltage increase, or external-GPIO implementation.
 */
static bool inf_otg_requested(struct inf_telemetry *t)
{
	unsigned long flags;
	bool requested;

	spin_lock_irqsave(&t->state_lock, flags);
	requested = t->up && !t->desynced && t->otg_requested &&
		    !READ_ONCE(t->otg_stopping);
	spin_unlock_irqrestore(&t->state_lock, flags);
	return requested;
}

static bool inf_otg_same_attachment(struct inf_telemetry *t, u32 generation)
{
	unsigned long flags;
	bool same;

	spin_lock_irqsave(&t->state_lock, flags);
	same = t->up && !t->desynced && t->otg_requested &&
	       !READ_ONCE(t->otg_stopping) &&
	       generation == t->otg_cancel_generation;
	spin_unlock_irqrestore(&t->state_lock, flags);
	return same;
}

static int inf_otg_test_health(struct inf_telemetry *t, bool sourcing,
	u32 generation, s64 deadline)
{
	u32 cid, online, enabled, temp, capacity, health, present, vbus;
	int ret;

	if (!inf_otg_requested(t) || t->charging || t->full_hold)
		return -EBUSY;
#define OTG_GET(op, id, v) do { \
	if (!inf_otg_same_attachment(t, generation)) return -ECANCELED; \
	if (deadline && ktime_to_ms(ktime_get()) >= deadline) return -ETIME; \
	ret = inf_read(t, op, id, &v); if (ret) return ret; \
} while (0)
	OTG_GET(0x301, 16, cid);
	OTG_GET(0x32, 0, online);
	OTG_GET(0x301, 70, enabled);
	OTG_GET(0x301, 67, temp);
	OTG_GET(0x301, 68, capacity);
	OTG_GET(0x30, 1, health);
	OTG_GET(0x30, 2, present);
	OTG_GET(0x32, 1, vbus);
#undef OTG_GET
	if (cid != 1 || online != 0 || enabled != 0 || present != 1 ||
	    health != POWER_SUPPLY_HEALTH_GOOD || temp < 1000 || temp > 3500 ||
	    capacity < 2000 || capacity > 10000 ||
	    (sourcing ? (vbus < 4500000 || vbus > 5500000) : vbus > 800000)) {
		dev_warn(t->dev, "OTG guard: sourcing=%u cid=%u online=%u charger=%u temp=%u capacity=%u health=%u present=%u vbus_uV=%u\n",
			 sourcing, cid, online, enabled, temp, capacity, health, present, vbus);
		return -ERANGE;
	}
	return inf_otg_same_attachment(t, generation) ? 0 : -ECANCELED;
}

static ssize_t inf_otg_test_run(struct device *dev, const char *buf,
	size_t count, unsigned int duration_ms)
{
	struct inf_telemetry *t = dev_get_drvdata(dev);
	bool request;
	int ret, off_ret;
	u32 generation;
	unsigned long flags;
	s64 deadline, next_health;

	ret = kstrtobool(buf, &request);
	if (ret || !request)
		return ret ? ret : -EINVAL;
	/* Reject a competing test; never queue a second power enable. */
	if (!mutex_trylock(&t->control_lock))
		return -EBUSY;
	WRITE_ONCE(t->otg_test_active, true);
	spin_lock_irqsave(&t->state_lock, flags);
	generation = t->otg_cancel_generation;
	spin_unlock_irqrestore(&t->state_lock, flags);
	ret = inf_otg_test_health(t, false, generation, 0);
	if (ret)
		goto out;
	ret = inf_set(t, 59, 500);
	if (ret)
		goto out;
	if (!inf_otg_same_attachment(t, generation)) {
		ret = -ECANCELED;
		goto out;
	}
	deadline = ktime_to_ms(ktime_get()) + duration_ms;
	next_health = ktime_to_ms(ktime_get()) + 1000;
	ret = inf_set(t, 19, 1);
	dev_info(dev, "OTG bounded test enable_ack=%d limit=500mA duration_max=%ums plus in-flight RPC latency generation=%u\n",
		 ret, duration_ms, generation);
	if (!ret) {
		while (ktime_to_ms(ktime_get()) < deadline) {
			if (!inf_otg_same_attachment(t, generation)) {
				ret = -ECANCELED;
				break;
			}
			/* Preserve the original pulse; monitor the longer test every
			 * second. RPC failure/health change stops it immediately.
			 */
			if (duration_ms > 3000 && ktime_to_ms(ktime_get()) >= next_health) {
				ret = inf_otg_test_health(t, true, generation, deadline);
				if (ret == -ETIME) {
					ret = 0;
					break;
				}
				if (ret)
					break;
				next_health = ktime_to_ms(ktime_get()) + 1000;
				continue;
			}
			if (msleep_interruptible(100)) {
				ret = -EINTR;
				break;
			}
		}
	}
	/* Even a failed enable can be ambiguous: always issue disable. */
	off_ret = inf_set(t, 19, 0);
	if (off_ret) {
		struct inf_request off = {
			.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
			.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
			.hdr.opcode = cpu_to_le32(0x300),
			.property = cpu_to_le32(19),
		};
		int send_ret;
		unsigned long flags;

		/* Same emergency-disable strategy as the existing charger path.
		 * Desynced replies cannot verify completion; report that explicitly.
		 */
		mutex_lock(&t->request_lock);
		spin_lock_irqsave(&t->state_lock, flags);
		t->desynced = true;
		spin_unlock_irqrestore(&t->state_lock, flags);
		send_ret = pmic_glink_send(t->client, &off, sizeof(off));
		mutex_unlock(&t->request_lock);
		dev_err(dev, "OTG test disable UNVERIFIED ack=%d fallback_send=%d\n", off_ret, send_ret);
		ret = off_ret;
	} else {
		dev_info(dev, "OTG bounded test disable acknowledged; verify connector voltage separately\n");
	}
out:
	WRITE_ONCE(t->otg_test_active, false);
	mutex_unlock(&t->control_lock);
	dev_info(dev, "OTG bounded test result=%d\n", ret);
	return ret ? ret : count;
}

static ssize_t otg_vbus_test_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	return inf_otg_test_run(dev, buf, count, 3000);
}
static DEVICE_ATTR_WO(otg_vbus_test);

static unsigned int otg_hold_max_ms = 120000;
static int otg_hold_max_ms_set(const char *val, const struct kernel_param *kp)
{
	unsigned int ms;
	int ret = kstrtouint(val, 0, &ms);

	if (ret)
		return ret;

	if (ms < 1000 || ms > 3600000)
		return -EINVAL;

	WRITE_ONCE(*(unsigned int *)kp->arg, ms);
	return 0;
}

static const struct kernel_param_ops otg_hold_max_ms_ops = {
	.set = otg_hold_max_ms_set,
	.get = param_get_uint,
};
module_param_cb(otg_hold_max_ms, &otg_hold_max_ms_ops, &otg_hold_max_ms, 0644);
MODULE_PARM_DESC(otg_hold_max_ms,
	"Upper bound of one guarded VBUS hold; firmware withdrawal, partner loss and guards still end it");

/* Explicit diagnostic lease, not persistent OTG policy. Writing zero cancels
 * the current test without waiting for its control lock. No auto-renewal.
 */
static ssize_t otg_vbus_hold_test_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inf_telemetry *t = dev_get_drvdata(dev);
	unsigned long flags;
	bool request;
	int ret = kstrtobool(buf, &request);

	if (ret)
		return ret;
	if (request)
		return inf_otg_test_run(dev, buf, count, READ_ONCE(otg_hold_max_ms));
	spin_lock_irqsave(&t->state_lock, flags);
	t->otg_cancel_generation++;
	spin_unlock_irqrestore(&t->state_lock, flags);
	return count;
}
static DEVICE_ATTR_WO(otg_vbus_hold_test);

static ssize_t otg_vbus_test_active_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inf_telemetry *t = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", READ_ONCE(t->otg_test_active));
}
static DEVICE_ATTR_RO(otg_vbus_test_active);

static struct attribute *inf_attrs[] = {
	&dev_attr_charging_enabled.attr, &dev_attr_charge_opt_out.attr, &dev_attr_charging_state.attr,
	&dev_attr_charging_profile.attr,
	&dev_attr_pd_request_mv.attr,
	&dev_attr_stock_chg_type.attr, &dev_attr_stock_target_uA.attr,
	&dev_attr_charge_current_limit_raw.attr, &dev_attr_float_voltage_raw.attr,
	&dev_attr_transport_state.attr,
	&dev_attr_notification_subscribe.attr, &dev_attr_notification_status.attr,
	&dev_attr_otg_vbus_test.attr,
	&dev_attr_otg_vbus_hold_test.attr, &dev_attr_otg_vbus_test_active.attr,
	&dev_attr_battery_status_raw.attr, &dev_attr_battery_health_raw.attr,
	&dev_attr_battery_present_raw.attr, &dev_attr_battery_capacity_raw.attr,
	&dev_attr_battery_voltage_raw.attr, &dev_attr_battery_current_raw.attr,
	&dev_attr_battery_temperature_raw.attr, &dev_attr_usb_online_raw.attr,
	&dev_attr_usb_voltage_raw.attr, &dev_attr_usb_voltage_max_raw.attr,
	&dev_attr_battery_charge_counter_raw.attr, &dev_attr_usb_current_raw.attr,
	&dev_attr_usb_current_max_raw.attr, &dev_attr_usb_input_current_limit_raw.attr,
	&dev_attr_charger_enabled_raw.attr,
	&dev_attr_otg_switch_raw.attr, &dev_attr_typec_mode_raw.attr,
	&dev_attr_otg_switch.attr,
	&dev_attr_otg_ap_enable.attr, &dev_attr_otg_ap_enable_raw.attr,
	&dev_attr_otg_ap_initialize.attr,
	&dev_attr_typec_restore_drp.attr,
	&dev_attr_typec_cc_orientation_raw.attr, &dev_attr_typec_cid_raw.attr,
	NULL,
};
static const struct attribute_group inf_group = {
	.name = "infiniti_telemetry", .attrs = inf_attrs,
};

static int inf_probe(struct auxiliary_device *adev,
		     const struct auxiliary_device_id *id)
{
	struct device *dev = &adev->dev;
	struct inf_telemetry *t;
	int ret;
	struct power_supply_config psy_cfg = {};

	if (!of_machine_is_compatible("oneplus,infiniti"))
		return -ENODEV;
	t = devm_kzalloc(dev, sizeof(*t), GFP_KERNEL);
	if (!t)
		return -ENOMEM;
	t->dev = dev;
	t->profile = INF_PROFILE_CONSERVATIVE;
	t->charge_state = "disabled";
	t->full_hold = false;
	t->pd_mv = 5000;
	t->charge_opt_out = false;
	t->charge_fail_streak = 0;
	t->usb_was_online = false;
	t->rechg_count = 0;
	mutex_init(&t->control_lock);
	INIT_DELAYED_WORK(&t->charge_work, inf_charge_worker);
	mutex_init(&t->request_lock);
	spin_lock_init(&t->state_lock);
	init_completion(&t->ack);
	dev_set_drvdata(dev, t);
	t->client = devm_pmic_glink_client_alloc(dev, PMIC_GLINK_OWNER_BATTMGR,
			inf_callback, inf_state, t);
	if (IS_ERR(t->client))
		return PTR_ERR(t->client);
	pmic_glink_client_register(t->client);
	t->reboot_nb.notifier_call = inf_reboot;
	ret = register_reboot_notifier(&t->reboot_nb);
	if (ret) return ret;
	ret = devm_add_action_or_reset(dev, inf_control_remove, t);
	if (ret) return ret;
	ret = devm_device_add_group(dev, &inf_group);
	if (ret) return ret;
	psy_cfg.drv_data = t;
	t->battery_psy = devm_power_supply_register(dev, &inf_battery_desc, &psy_cfg);
	if (IS_ERR(t->battery_psy)) return PTR_ERR(t->battery_psy);
	t->usb_psy = devm_power_supply_register(dev, &inf_usb_desc, &psy_cfg);
	if (IS_ERR(t->usb_psy)) return PTR_ERR(t->usb_psy);
	INIT_DELAYED_WORK(&t->report_work, inf_report_worker);
	ret = devm_add_action_or_reset(dev, inf_report_remove, t);
	if (ret) return ret;
	schedule_delayed_work(&t->report_work, msecs_to_jiffies(2000));
	return 0;
}

static int inf_suspend(struct device *dev)
{
	/* Monitoring must keep running while this experimental mode is on. */
	struct inf_telemetry *t = dev_get_drvdata(dev);
	return (READ_ONCE(t->charging) || READ_ONCE(t->otg_test_active)) ? -EBUSY : 0;
}

/* The auxiliary bus never calls a driver's legacy .suspend; only dev_pm_ops run. */
static DEFINE_SIMPLE_DEV_PM_OPS(inf_pm_ops, inf_suspend, NULL);

static const struct auxiliary_device_id inf_ids[] = {
	{ .name = "pmic_glink.power-supply" },
	{ .name = "pmic_glink.oplus-power-supply" },
	{}
};
MODULE_DEVICE_TABLE(auxiliary, inf_ids);
static struct auxiliary_driver inf_driver = {
	.name = "oplus_infiniti_telemetry", .probe = inf_probe, .id_table = inf_ids,
	.driver.pm = pm_sleep_ptr(&inf_pm_ops),
};
module_auxiliary_driver(inf_driver);
MODULE_DESCRIPTION("Infiniti PMIC GLINK telemetry and stock thermal charging profiles");
MODULE_LICENSE("GPL");
