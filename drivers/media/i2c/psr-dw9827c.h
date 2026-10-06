/* SPDX-License-Identifier: GPL-2.0-only */
/* PSR sensor-owned manual focus. Calibration is for captured handset
 * 3C167A00PSR00000 only; a live EEPROM comparison precedes all lens writes.
 * See actuator-lifecycle-v1 and manual-focus-v1 for exact stock provenance.
 * No independent PM/CPAS ownership, OTP/NVM programming or autofocus algorithm.
 */
#ifndef PSR_DW9827C_H
#define PSR_DW9827C_H

struct psr_focus_profile {
	const char *name;
	u8 address, shift;
	u16 minimum, maximum, initial;
	u8 calibration[6];
	bool park;
};

struct psr_focus {
	struct i2c_client *client, *eeprom;
	const struct psr_focus_profile *profile;
	bool touched, positioned;
	u16 position, raw_position;
};

static const struct psr_focus_profile psr_focus_main = {
	.name = "main", .address = 0x0c, .shift = 4,
	.minimum = 386, .maximum = 3852, .initial = 386,
	.calibration = { 0x1d, 0x0e, 0x71, 0x02, 0x93, 0x04 },
	.park = true,
};

static const struct psr_focus_profile psr_focus_tele = {
	.name = "tele", .address = 0x0f, .shift = 6,
	.minimum = 181, .maximum = 923, .initial = 230,
	.calibration = { 0x9b, 0x0d, 0x53, 0x05, 0x6f, 0x09 },
};

static bool psr_focus_enable;
module_param_named(focus_enable, psr_focus_enable, bool, 0600);
MODULE_PARM_DESC(focus_enable, "Enable calibrated manual focus at next stream start; default off, protected PSR diagnostics only");

static int psr_focus_read(struct i2c_client *client, u8 *address,
			  u16 address_size, u8 *data, u16 size)
{
	struct i2c_msg messages[] = {
		{ .addr = client->addr, .len = address_size, .buf = address },
		{ .addr = client->addr, .flags = I2C_M_RD,
		  .len = size, .buf = data },
	};
	int ret = i2c_transfer(client->adapter, messages, ARRAY_SIZE(messages));

	return ret == ARRAY_SIZE(messages) ? 0 : ret < 0 ? ret : -EIO;
}

static int psr_focus_byte(struct psr_focus *focus, u8 reg, u8 value,
			  unsigned int delay)
{
	u8 data[] = { reg, value };
	int ret = i2c_master_send(focus->client, data, sizeof(data));

	if (ret != sizeof(data))
		return ret < 0 ? ret : -EIO;
	if (delay)
		usleep_range(delay, delay + 50);
	return 0;
}

static int psr_focus_position(struct psr_focus *focus, u16 *position)
{
	u8 address = 0, data[2];
	u16 raw;
	int ret = psr_focus_read(focus->client, &address, 1, data, sizeof(data));

	if (ret)
		return ret;
	raw = ((u16)data[0] << 8) | data[1];
	/* Captured FormatPositionData discards the low bits; it does not
	 * establish those bits as reserved zero on a physical read. */
	focus->raw_position = raw;
	*position = raw >> focus->profile->shift;
	return 0;
}

static int psr_focus_move(struct psr_focus *focus, u16 position)
{
	u16 raw = position << focus->profile->shift, observed;
	u8 data[] = { 0, raw >> 8, raw & 0xff };
	int ret = i2c_master_send(focus->client, data, sizeof(data));

	/* A failed transfer could have partially changed the position. */
	focus->positioned = false;
	if (ret != sizeof(data))
		return ret < 0 ? ret : -EIO;
	ret = psr_focus_position(focus, &observed);
	if (ret)
		return ret;
	focus->position = observed;
	focus->positioned = true;
	dev_info(&focus->client->dev, "PSR focus %s target=%u read=%u raw=%04x\n",
		 focus->profile->name, position, observed, focus->raw_position);
	return observed == position ? 0 : -EIO;
}

static int psr_focus_set(struct psr_focus *focus, unsigned int position)
{
	if (position < focus->profile->minimum || position > focus->profile->maximum)
		return -ERANGE;
	/* Idle controls cache normally; the enable value is latched by begin(). */
	if (!focus->touched)
		return 0;
	return psr_focus_move(focus, position);
}

static int psr_focus_begin(struct psr_focus *focus)
{
	u8 address[] = { 0, 0x92 }, calibration[6], marker_reg = 0x7d, marker;
	u16 before;
	int ret;

	if (focus->touched)
		return -EBUSY;
	if (!READ_ONCE(psr_focus_enable))
		return 0;
	ret = psr_focus_read(focus->eeprom, address, sizeof(address),
			     calibration, sizeof(calibration));
	if (ret)
		return ret;
	if (memcmp(calibration, focus->profile->calibration, sizeof(calibration)))
		return -ENODEV;
	ret = psr_focus_read(focus->client, &marker_reg, 1, &marker, 1);
	if (!ret)
		ret = psr_focus_position(focus, &before);
	if (ret)
		return ret;
	dev_info(&focus->client->dev,
		 "PSR focus %s calibration matched marker=0x%02x before=%u raw=%04x\n",
		 focus->profile->name, marker, before, focus->raw_position);
	/* Marker observation never triggers the OEM persistent/NVM recipe. */
	focus->touched = true;
	if (focus->profile->park) {
		ret = psr_focus_byte(focus, 0x02, 0x00, 100);
	} else {
		ret = psr_focus_byte(focus, 0x02, 0x40, 3000);
		if (!ret) ret = psr_focus_byte(focus, 0x04, 0x01, 3000);
		if (!ret) ret = psr_focus_byte(focus, 0x00, 0x39, 50);
	}
	if (!ret) ret = psr_focus_byte(focus, 0x01, 0x80, 50);
	if (!ret) ret = psr_focus_byte(focus, 0x02, 0x00, 3000);
	/* Handler setup applies the selected manual position after initialization. */
	return ret;
}

static int psr_focus_end(struct psr_focus *focus)
{
	unsigned int step;
	u16 next;
	int ret = 0, err;

	if (!focus->touched)
		return 0;
	if (focus->profile->park && focus->positioned) {
		/* Captured default ParkLens: at most four writes, 10ms each.
		 * Its parking path deliberately travels below the normal focus range.
		 * Do not invent a final zero write or apply tele's unused park table.
		 */
		for (step = 0; step < 4 && focus->position; step++) {
			next = focus->profile->initial;
			if (focus->position <= next)
				next = focus->position >= 4 ?
					focus->position - focus->position / 4 : 0;
			ret = psr_focus_move(focus, next);
			if (ret)
				break;
			usleep_range(10000, 10050);
		}
	}
	/* Attempt volatile standby even after a failed move; retain first error. */
	err = psr_focus_byte(focus, 0x02, 0x40, 200);
	if (!ret) ret = err;
	dev_info(&focus->client->dev, "PSR focus %s standby ret=%d\n",
		 focus->profile->name, ret);
	focus->touched = false;
	focus->positioned = false;
	return ret;
}

static int psr_focus_init(struct psr_focus *focus, struct i2c_client *sensor,
			  const struct psr_focus_profile *profile)
{
	/* Reservation only, after exact sensor identity probe. No I2C/power I/O. */
	focus->profile = profile;
	focus->client = devm_i2c_new_dummy_device(&sensor->dev, sensor->adapter,
					       profile->address);
	if (IS_ERR(focus->client))
		return PTR_ERR(focus->client);
	focus->eeprom = devm_i2c_new_dummy_device(&sensor->dev, sensor->adapter, 0x50);
	return PTR_ERR_OR_ZERO(focus->eeprom);
}
#endif
