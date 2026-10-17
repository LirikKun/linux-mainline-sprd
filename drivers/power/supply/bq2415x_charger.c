// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * bq2415x charger driver
 *
 * Copyright (C) 2011-2013  Pali Rohár <pali@kernel.org>
 *
 * Datasheets:
 * https://www.ti.com/product/bq24150
 * https://www.ti.com/product/bq24150a
 * https://www.ti.com/product/bq24152
 * https://www.ti.com/product/bq24153
 * https://www.ti.com/product/bq24153a
 * https://www.ti.com/product/bq24155
 * https://www.ti.com/product/bq24157s
 * https://www.ti.com/product/bq24158
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/param.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

#include <linux/power/bq2415x_charger.h>

/* timeout for resetting chip timer */
#define BQ2415X_TIMER_TIMEOUT		10

#define BQ2415X_REG_STATUS		0x00
#define BQ2415X_REG_CONTROL		0x01
#define BQ2415X_REG_VOLTAGE		0x02
#define BQ2415X_REG_VENDER		0x03
#define BQ2415X_REG_CURRENT		0x04

/* reset state for all registers */
#define BQ2415X_RESET_STATUS		BIT(6)
#define BQ2415X_RESET_CONTROL		(BIT(4)|BIT(5))
#define BQ2415X_RESET_VOLTAGE		(BIT(1)|BIT(3))
#define BQ2415X_RESET_CURRENT		(BIT(0)|BIT(3)|BIT(7))

/* status register */
#define BQ2415X_BIT_TMR_RST		7
#define BQ2415X_BIT_OTG			7
#define BQ2415X_BIT_EN_STAT		6
#define BQ2415X_MASK_STAT		(BIT(4)|BIT(5))
#define BQ2415X_SHIFT_STAT		4
#define BQ2415X_BIT_BOOST		3
#define BQ2415X_MASK_FAULT		(BIT(0)|BIT(1)|BIT(2))
#define BQ2415X_SHIFT_FAULT		0

/* control register */
#define BQ2415X_MASK_LIMIT		(BIT(6)|BIT(7))
#define BQ2415X_SHIFT_LIMIT		6
#define BQ2415X_MASK_VLOWV		(BIT(4)|BIT(5))
#define BQ2415X_SHIFT_VLOWV		4
#define BQ2415X_BIT_TE			3
#define BQ2415X_BIT_CE			2
#define BQ2415X_BIT_HZ_MODE		1
#define BQ2415X_BIT_OPA_MODE		0

/* voltage register */
#define BQ2415X_MASK_VO		(BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7))
#define BQ2415X_SHIFT_VO		2
#define BQ2415X_BIT_OTG_PL		1
#define BQ2415X_BIT_OTG_EN		0

/* vender register */
#define BQ2415X_MASK_VENDER		(BIT(5)|BIT(6)|BIT(7))
#define BQ2415X_SHIFT_VENDER		5
#define BQ2415X_MASK_PN			(BIT(3)|BIT(4))
#define BQ2415X_SHIFT_PN		3
#define BQ2415X_MASK_REVISION		(BIT(0)|BIT(1)|BIT(2))
#define BQ2415X_SHIFT_REVISION		0

/* current register */
#define BQ2415X_MASK_RESET		BIT(7)
#define BQ2415X_MASK_VI_CHRG		(BIT(4)|BIT(5)|BIT(6))
#define BQ2415X_SHIFT_VI_CHRG		4
/* N/A					BIT(3) */
#define BQ2415X_MASK_VI_TERM		(BIT(0)|BIT(1)|BIT(2))
#define BQ2415X_SHIFT_VI_TERM		0

/* AW32257 current register differences. */
#define AW32257_MASK_VI_CHRG		(BIT(3)|BIT(4)|BIT(5)|BIT(6))
#define AW32257_SHIFT_VI_CHRG		3

static const int aw32257_ichg_uv[16] = {
	16368, 20460, 28644, 32736, 36828, 40920, 45012, 49104,
	53196, 57288, 61380, 65472, 69564, 73656, 77748, 81840,
};

/* Termination current, REG04[2:0]: 2046 uV base, then 2046/4092/8184 by bit. */
static int aw32257_iterm_uv(int code)
{
	return 2046 + 2046 * (code & 1) + 4092 * ((code >> 1) & 1) +
	       8184 * ((code >> 2) & 1);
}


enum bq2415x_command {
	BQ2415X_TIMER_RESET,
	BQ2415X_OTG_STATUS,
	BQ2415X_STAT_PIN_STATUS,
	BQ2415X_STAT_PIN_ENABLE,
	BQ2415X_STAT_PIN_DISABLE,
	BQ2415X_CHARGE_STATUS,
	BQ2415X_BOOST_STATUS,
	BQ2415X_FAULT_STATUS,

	BQ2415X_CHARGE_TERMINATION_STATUS,
	BQ2415X_CHARGE_TERMINATION_ENABLE,
	BQ2415X_CHARGE_TERMINATION_DISABLE,
	BQ2415X_CHARGER_STATUS,
	BQ2415X_CHARGER_ENABLE,
	BQ2415X_CHARGER_DISABLE,
	BQ2415X_HIGH_IMPEDANCE_STATUS,
	BQ2415X_HIGH_IMPEDANCE_ENABLE,
	BQ2415X_HIGH_IMPEDANCE_DISABLE,
	BQ2415X_BOOST_MODE_STATUS,
	BQ2415X_BOOST_MODE_ENABLE,
	BQ2415X_BOOST_MODE_DISABLE,

	BQ2415X_OTG_LEVEL,
	BQ2415X_OTG_ACTIVATE_HIGH,
	BQ2415X_OTG_ACTIVATE_LOW,
	BQ2415X_OTG_PIN_STATUS,
	BQ2415X_OTG_PIN_ENABLE,
	BQ2415X_OTG_PIN_DISABLE,

	BQ2415X_VENDER_CODE,
	BQ2415X_PART_NUMBER,
	BQ2415X_REVISION,
};

enum bq2415x_chip {
	BQUNKNOWN,
	BQ24150,
	BQ24150A,
	BQ24151,
	BQ24151A,
	BQ24152,
	BQ24153,
	BQ24153A,
	BQ24155,
	BQ24156,
	BQ24156A,
	BQ24157S,
	BQ24158,
	AW32257,
};

static char *bq2415x_chip_name[] = {
	"unknown",
	"bq24150",
	"bq24150a",
	"bq24151",
	"bq24151a",
	"bq24152",
	"bq24153",
	"bq24153a",
	"bq24155",
	"bq24156",
	"bq24156a",
	"bq24157s",
	"bq24158",
	"aw32257",
};

enum bq2415x_lifecycle {
	BQ2415X_STARTING,
	BQ2415X_READY,
	BQ2415X_STOPPING,
};

struct bq2415x_device {
	struct device *dev;
	struct bq2415x_platform_data init_data;
	struct power_supply *charger;
	struct power_supply_desc charger_desc;
	struct delayed_work work;
	struct device_node *notify_node;
	struct notifier_block nb;
	enum bq2415x_mode reported_mode;/* mode reported by hook function */
	enum bq2415x_mode mode;		/* currently configured mode */
	enum bq2415x_chip chip;
	const char *timer_error;
	char *model;
	char *name;
	/* Charge temperature guard; see bq2415x_temp_work(). */
	struct delayed_work temp_work;

	int temp_min;		/* deci-celsius, INT_MIN if unconstrained */
	int temp_max;		/* deci-celsius, INT_MAX if unconstrained */
	bool temp_blocked;	/* charging currently inhibited on temperature */
	/* Framework -> timer_control (external only) -> policy -> transfer.
	 * Workers never take timer_control; supplier reads never hold policy.
	 */
	struct mutex policy_lock;
	struct mutex timer_control;
	enum bq2415x_lifecycle lifecycle;
	int charge_profile; /* -1: INIT defaults; mode is presentation only. */
	bool manual_boost;
	bool regulator_boost;
	bool sensor_valid;
	bool thermal_trip;
	bool config_valid;
	bool input_override;
	bool config_deferred;
	/* -1 preserves inherited HZ/STAT, or the legacy termination default. */
	int termination_enable;
	int high_impedance_enable;
	int stat_pin_enable;
	int otg_pin_enable;
	bool fault_inhibit;
	bool applied_known;
	bool applied_charge;
	bool applied_boost;
	bool notifier_registered;
	unsigned long detector_sequence;
	bool detector_valid;
	int autotimer;	/* 1 - if driver automatically reset timer, 0 - not */
	int automode;	/* 1 - enabled, 0 - disabled; -1 - not supported */
	int charge_status;
	int id;
	struct regulator_dev *usb_vbus_reg;
};

/* each registered chip must have unique id */
static DEFINE_IDR(bq2415x_id);

static DEFINE_MUTEX(bq2415x_id_mutex);
static DEFINE_MUTEX(bq2415x_i2c_mutex);

static void bq2415x_timer_work(struct work_struct *work);
static void bq2415x_temp_work(struct work_struct *work);
static void bq2415x_detector_retry(struct bq2415x_device *bq);

static void bq2415x_policy_init(struct bq2415x_device *bq)
{
	mutex_init(&bq->policy_lock);
	mutex_init(&bq->timer_control);
	INIT_DELAYED_WORK(&bq->work, bq2415x_timer_work);
	INIT_DELAYED_WORK(&bq->temp_work, bq2415x_temp_work);
	bq->lifecycle = BQ2415X_STARTING;
	bq->charge_profile = -1;
	bq->temp_min = INT_MIN;
	bq->temp_max = INT_MAX;
	bq->sensor_valid = false;
	bq->thermal_trip = false;
	bq->temp_blocked = true;
	bq->config_valid = false;
	bq->applied_known = false;
	bq->manual_boost = false;
	bq->regulator_boost = false;
	bq->fault_inhibit = false;
	bq->input_override = false;
	bq->config_deferred = false;
	bq->termination_enable = -1;
	bq->high_impedance_enable = -1;
	bq->stat_pin_enable = -1;
	bq->otg_pin_enable = 0;
	bq->applied_charge = false;
	bq->applied_boost = false;
	bq->autotimer = 0;
	bq->automode = 0;
	bq->mode = BQ2415X_MODE_OFF;
	bq->reported_mode = BQ2415X_MODE_OFF;
	bq->timer_error = NULL;
	bq->notifier_registered = false;
	bq->detector_sequence = 0;
	bq->detector_valid = false;
}

static bool bq2415x_guarded(struct bq2415x_device *bq)
{
	return bq->temp_min != INT_MIN || bq->temp_max != INT_MAX;
}

static bool bq2415x_charge_allowed(struct bq2415x_device *bq)
{
	return bq->lifecycle == BQ2415X_READY &&
		bq->charge_profile != BQ2415X_MODE_OFF &&
		!bq->manual_boost && !bq->regulator_boost &&
		bq->config_valid && !bq->fault_inhibit &&
		(bq->automode <= 0 ||
		 (bq->detector_valid &&
		  bq->charge_profile == (int)bq->reported_mode)) &&
		(!bq2415x_guarded(bq) ||
		 (bq->sensor_valid && !bq->thermal_trip));
}

/**** i2c read functions ****/

/* read value from register */
static int bq2415x_i2c_read(struct bq2415x_device *bq, u8 reg)
{
	struct i2c_client *client = to_i2c_client(bq->dev);
	struct i2c_msg msg[2];
	u8 val;
	int ret;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = &val;
	msg[1].len = sizeof(val);

	mutex_lock(&bq2415x_i2c_mutex);
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	mutex_unlock(&bq2415x_i2c_mutex);

	if (ret < 0)
		return ret;
	if (ret != ARRAY_SIZE(msg))
		return -EIO;

	return val;
}

/* read value from register, apply mask and right shift it */
static int bq2415x_i2c_read_mask(struct bq2415x_device *bq, u8 reg,
				 u8 mask, u8 shift)
{
	int ret;

	if (shift > 8)
		return -EINVAL;

	ret = bq2415x_i2c_read(bq, reg);
	if (ret < 0)
		return ret;
	return (ret & mask) >> shift;
}

/* read value from register and return one specified bit */
static int bq2415x_i2c_read_bit(struct bq2415x_device *bq, u8 reg, u8 bit)
{
	if (bit > 8)
		return -EINVAL;
	return bq2415x_i2c_read_mask(bq, reg, BIT(bit), bit);
}

/**** i2c write functions ****/

/* write value to register */
static int bq2415x_i2c_write(struct bq2415x_device *bq, u8 reg, u8 val)
{
	struct i2c_client *client = to_i2c_client(bq->dev);
	struct i2c_msg msg[1];
	u8 data[2];
	int ret;

	data[0] = reg;
	data[1] = val;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = data;
	msg[0].len = ARRAY_SIZE(data);

	mutex_lock(&bq2415x_i2c_mutex);
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	mutex_unlock(&bq2415x_i2c_mutex);

	/* i2c_transfer returns number of messages transferred */
	if (ret < 0)
		return ret;
	else if (ret != 1)
		return -EIO;

	return 0;
}

/* read value from register, change it with mask left shifted and write back */
static int bq2415x_i2c_write_mask(struct bq2415x_device *bq, u8 reg, u8 val,
				  u8 mask, u8 shift)
{
	int ret;

	if (shift > 8)
		return -EINVAL;

	ret = bq2415x_i2c_read(bq, reg);
	if (ret < 0)
		return ret;

	ret &= ~mask;
	ret |= val << shift;

	return bq2415x_i2c_write(bq, reg, ret);
}

/* change only one bit in register */
static int bq2415x_i2c_write_bit(struct bq2415x_device *bq, u8 reg,
				 bool val, u8 bit)
{
	if (bit > 8)
		return -EINVAL;
	return bq2415x_i2c_write_mask(bq, reg, val, BIT(bit), bit);
}

/**** global functions ****/

/* exec command function */
static int bq2415x_exec_command(struct bq2415x_device *bq,
				enum bq2415x_command command)
{
	int ret;

	if (command == BQ2415X_OTG_PIN_ENABLE && bq2415x_guarded(bq))
		return -EPERM;
	if (command == BQ2415X_CHARGER_ENABLE && !bq2415x_charge_allowed(bq))
		command = BQ2415X_CHARGER_DISABLE;

	switch (command) {
	case BQ2415X_TIMER_RESET:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_STATUS,
				1, BQ2415X_BIT_TMR_RST);
	case BQ2415X_OTG_STATUS:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_STATUS,
				BQ2415X_BIT_OTG);
	case BQ2415X_STAT_PIN_STATUS:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_STATUS,
				BQ2415X_BIT_EN_STAT);
	case BQ2415X_STAT_PIN_ENABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_STATUS, 1,
				BQ2415X_BIT_EN_STAT);
	case BQ2415X_STAT_PIN_DISABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_STATUS, 0,
				BQ2415X_BIT_EN_STAT);
	case BQ2415X_CHARGE_STATUS:
		return bq2415x_i2c_read_mask(bq, BQ2415X_REG_STATUS,
				BQ2415X_MASK_STAT, BQ2415X_SHIFT_STAT);
	case BQ2415X_BOOST_STATUS:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_STATUS,
				BQ2415X_BIT_BOOST);
	case BQ2415X_FAULT_STATUS:
		return bq2415x_i2c_read_mask(bq, BQ2415X_REG_STATUS,
			BQ2415X_MASK_FAULT, BQ2415X_SHIFT_FAULT);

	case BQ2415X_CHARGE_TERMINATION_STATUS:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_CONTROL,
				BQ2415X_BIT_TE);
	case BQ2415X_CHARGE_TERMINATION_ENABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				1, BQ2415X_BIT_TE);
	case BQ2415X_CHARGE_TERMINATION_DISABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				0, BQ2415X_BIT_TE);
	case BQ2415X_CHARGER_STATUS:
		ret = bq2415x_i2c_read_bit(bq, BQ2415X_REG_CONTROL,
			BQ2415X_BIT_CE);
		if (ret < 0)
			return ret;
		return ret > 0 ? 0 : 1;
	case BQ2415X_CHARGER_ENABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				0, BQ2415X_BIT_CE);
	case BQ2415X_CHARGER_DISABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				1, BQ2415X_BIT_CE);
	case BQ2415X_HIGH_IMPEDANCE_STATUS:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_CONTROL,
				BQ2415X_BIT_HZ_MODE);
	case BQ2415X_HIGH_IMPEDANCE_ENABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				1, BQ2415X_BIT_HZ_MODE);
	case BQ2415X_HIGH_IMPEDANCE_DISABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				0, BQ2415X_BIT_HZ_MODE);
	case BQ2415X_BOOST_MODE_STATUS:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_CONTROL,
				BQ2415X_BIT_OPA_MODE);
	case BQ2415X_BOOST_MODE_ENABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				1, BQ2415X_BIT_OPA_MODE);
	case BQ2415X_BOOST_MODE_DISABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				0, BQ2415X_BIT_OPA_MODE);

	case BQ2415X_OTG_LEVEL:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_VOLTAGE,
				BQ2415X_BIT_OTG_PL);
	case BQ2415X_OTG_ACTIVATE_HIGH:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_VOLTAGE,
				1, BQ2415X_BIT_OTG_PL);
	case BQ2415X_OTG_ACTIVATE_LOW:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_VOLTAGE,
				0, BQ2415X_BIT_OTG_PL);
	case BQ2415X_OTG_PIN_STATUS:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_VOLTAGE,
				BQ2415X_BIT_OTG_EN);
	case BQ2415X_OTG_PIN_ENABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_VOLTAGE,
				1, BQ2415X_BIT_OTG_EN);
	case BQ2415X_OTG_PIN_DISABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_VOLTAGE,
				0, BQ2415X_BIT_OTG_EN);

	case BQ2415X_VENDER_CODE:
		return bq2415x_i2c_read_mask(bq, BQ2415X_REG_VENDER,
			BQ2415X_MASK_VENDER, BQ2415X_SHIFT_VENDER);
	case BQ2415X_PART_NUMBER:
		return bq2415x_i2c_read_mask(bq, BQ2415X_REG_VENDER,
				BQ2415X_MASK_PN, BQ2415X_SHIFT_PN);
	case BQ2415X_REVISION:
		return bq2415x_i2c_read_mask(bq, BQ2415X_REG_VENDER,
			BQ2415X_MASK_REVISION, BQ2415X_SHIFT_REVISION);
	}
	return -EINVAL;
}

/* detect chip type */
static enum bq2415x_chip bq2415x_detect_chip(struct bq2415x_device *bq)
{
	struct i2c_client *client = to_i2c_client(bq->dev);
	int ret = bq2415x_exec_command(bq, BQ2415X_PART_NUMBER);

	if (ret < 0)
		return ret;

	switch (client->addr) {
	case 0x6b:
		switch (ret) {
		case 0:
			if (bq->chip == BQ24151A)
				return bq->chip;
			return BQ24151;
		case 1:
			if (bq->chip == BQ24150A ||
				bq->chip == BQ24152 ||
				bq->chip == BQ24155)
				return bq->chip;
			return BQ24150;
		case 2:
			if (bq->chip == BQ24153A)
				return bq->chip;
			return BQ24153;
		default:
			return BQUNKNOWN;
		}
		break;

	case 0x6a:
		switch (ret) {
		case 0:
			if (bq->chip == BQ24156A)
				return bq->chip;
			return BQ24156;
		case 2:
			if (bq->chip == BQ24157S || bq->chip == AW32257)
				return bq->chip;
			return BQ24158;
		default:
			return BQUNKNOWN;
		}
		break;
	}

	return BQUNKNOWN;
}

/* detect chip revision */
static int bq2415x_detect_revision(struct bq2415x_device *bq)
{
	int ret = bq2415x_exec_command(bq, BQ2415X_REVISION);
	int chip = bq2415x_detect_chip(bq);

	if (ret < 0 || chip < 0)
		return -1;

	switch (chip) {
	case BQ24150:
	case BQ24150A:
	case BQ24151:
	case BQ24151A:
	case BQ24152:
		if (ret >= 0 && ret <= 3)
			return ret;
		return -1;
	case BQ24153:
	case BQ24153A:
	case BQ24156:
	case BQ24156A:
	case BQ24157S:
	case BQ24158:
		if (ret == 3)
			return 0;
		else if (ret == 1)
			return 1;
		return -1;
	case BQ24155:
		if (ret == 3)
			return 3;
		return -1;
	case BQUNKNOWN:
		return -1;
	}

	return -1;
}

/* return chip vender code */
static int bq2415x_get_vender_code(struct bq2415x_device *bq)
{
	int ret;

	ret = bq2415x_exec_command(bq, BQ2415X_VENDER_CODE);
	if (ret < 0)
		return 0;

	/* convert to binary */
	return (ret & 0x1) +
	       ((ret >> 1) & 0x1) * 10 +
	       ((ret >> 2) & 0x1) * 100;
}

/* reset all chip registers to default state */
static int bq2415x_reset_chip(struct bq2415x_device *bq)
{
	int ret;

	if (bq2415x_guarded(bq))
		return -EPERM;
	if (bq->regulator_boost)
		return -EBUSY;
	bq->config_valid = false;
	bq->applied_known = false;
	ret = bq2415x_i2c_write(bq, BQ2415X_REG_CURRENT, BQ2415X_RESET_CURRENT);
	if (ret)
		return ret;
	/* Legacy reset only; guarded devices never enter this sequence. */
	msleep(32);
	ret = bq2415x_i2c_write(bq, BQ2415X_REG_VOLTAGE, BQ2415X_RESET_VOLTAGE);
	if (ret)
		return ret;
	ret = bq2415x_i2c_write(bq, BQ2415X_REG_CONTROL, BQ2415X_RESET_CONTROL);
	if (ret)
		return ret;
	ret = bq2415x_i2c_write(bq, BQ2415X_REG_STATUS, BQ2415X_RESET_STATUS);
	if (ret)
		return ret;
	bq->timer_error = NULL;
	return 0;
}

/**** properties functions ****/

/* set current limit in mA */
static int bq2415x_set_current_limit(struct bq2415x_device *bq, int mA)
{
	int val;

	/* REG01[7:6] does not exist on the AW32257 */
	if (bq->chip == AW32257)
		return 0;

	if (mA <= 100)
		val = 0;
	else if (mA <= 500)
		val = 1;
	else if (mA <= 800)
		val = 2;
	else
		val = 3;

	return bq2415x_i2c_write_mask(bq, BQ2415X_REG_CONTROL, val,
			BQ2415X_MASK_LIMIT, BQ2415X_SHIFT_LIMIT);
}

/* get current limit in mA */
static int bq2415x_get_current_limit(struct bq2415x_device *bq)
{
	int ret;

	ret = bq2415x_i2c_read_mask(bq, BQ2415X_REG_CONTROL,
			BQ2415X_MASK_LIMIT, BQ2415X_SHIFT_LIMIT);
	if (ret < 0)
		return ret;
	else if (ret == 0)
		return 100;
	else if (ret == 1)
		return 500;
	else if (ret == 2)
		return 800;
	else if (ret == 3)
		return 1800;
	return -EINVAL;
}

/* set weak battery voltage in mV */
static int bq2415x_set_weak_battery_voltage(struct bq2415x_device *bq, int mV)
{
	int val;

	/* REG01[5:4] does not exist on the AW32257; VSHORT is fixed at 2.1V. */
	if (bq->chip == AW32257)
		return 0;

	/* round to 100mV */
	if (mV <= 3400 + 50)
		val = 0;
	else if (mV <= 3500 + 50)
		val = 1;
	else if (mV <= 3600 + 50)
		val = 2;
	else
		val = 3;

	return bq2415x_i2c_write_mask(bq, BQ2415X_REG_CONTROL, val,
			BQ2415X_MASK_VLOWV, BQ2415X_SHIFT_VLOWV);
}

/* get weak battery voltage in mV */
static int bq2415x_get_weak_battery_voltage(struct bq2415x_device *bq)
{
	int ret;

	ret = bq2415x_i2c_read_mask(bq, BQ2415X_REG_CONTROL,
			BQ2415X_MASK_VLOWV, BQ2415X_SHIFT_VLOWV);
	if (ret < 0)
		return ret;
	return 100 * (34 + ret);
}

/* set battery regulation voltage in mV */
static int bq2415x_set_battery_regulation_voltage(struct bq2415x_device *bq,
						  int mV)
{
	int val = (mV/10 - 350) / 2;

	/*
	 * According to datasheet, maximum battery regulation voltage is
	 * 4440mV which is b101111 = 47.
	 */
	if (val < 0)
		val = 0;
	else if (val > 47)
		return -EINVAL;

	return bq2415x_i2c_write_mask(bq, BQ2415X_REG_VOLTAGE, val,
			BQ2415X_MASK_VO, BQ2415X_SHIFT_VO);
}

/* get battery regulation voltage in mV */
static int bq2415x_get_battery_regulation_voltage(struct bq2415x_device *bq)
{
	int ret = bq2415x_i2c_read_mask(bq, BQ2415X_REG_VOLTAGE,
			BQ2415X_MASK_VO, BQ2415X_SHIFT_VO);

	if (ret < 0)
		return ret;
	return 10 * (350 + 2*ret);
}

/* set charge current in mA (platform data must provide resistor sense) */
static int bq2415x_set_charge_current(struct bq2415x_device *bq, int mA)
{
	int val;

	if (bq->init_data.resistor_sense <= 0)
		return -EINVAL;

	if (bq->chip == AW32257) {
		int uv = mA * bq->init_data.resistor_sense;

		/* Highest code that does not exceed the requested current. */
		for (val = ARRAY_SIZE(aw32257_ichg_uv) - 1; val > 0; val--)
			if (aw32257_ichg_uv[val] <= uv)
				break;

		return bq2415x_i2c_write_mask(bq, BQ2415X_REG_CURRENT, val,
				AW32257_MASK_VI_CHRG | BQ2415X_MASK_RESET,
				AW32257_SHIFT_VI_CHRG);
	}

	val = (mA * bq->init_data.resistor_sense - 37400) / 6800;
	if (val < 0)
		val = 0;
	else if (val > 7)
		val = 7;

	return bq2415x_i2c_write_mask(bq, BQ2415X_REG_CURRENT, val,
			BQ2415X_MASK_VI_CHRG | BQ2415X_MASK_RESET,
			BQ2415X_SHIFT_VI_CHRG);
}

/* get charge current in mA (platform data must provide resistor sense) */
static int bq2415x_get_charge_current(struct bq2415x_device *bq)
{
	int ret;

	if (bq->init_data.resistor_sense <= 0)
		return -EINVAL;

	if (bq->chip == AW32257) {
		ret = bq2415x_i2c_read_mask(bq, BQ2415X_REG_CURRENT,
				AW32257_MASK_VI_CHRG, AW32257_SHIFT_VI_CHRG);
		if (ret < 0)
			return ret;
		return aw32257_ichg_uv[ret] / bq->init_data.resistor_sense;
	}

	ret = bq2415x_i2c_read_mask(bq, BQ2415X_REG_CURRENT,
			BQ2415X_MASK_VI_CHRG, BQ2415X_SHIFT_VI_CHRG);
	if (ret < 0)
		return ret;
	return (37400 + 6800*ret) / bq->init_data.resistor_sense;
}

/* set termination current in mA (platform data must provide resistor sense) */
static int bq2415x_set_termination_current(struct bq2415x_device *bq, int mA)
{
	int val;

	if (bq->init_data.resistor_sense <= 0)
		return -EINVAL;

	if (bq->chip == AW32257) {
		int uv = mA * bq->init_data.resistor_sense;

		for (val = 7; val > 0; val--)
			if (aw32257_iterm_uv(val) <= uv)
				break;

		return bq2415x_i2c_write_mask(bq, BQ2415X_REG_CURRENT, val,
				BQ2415X_MASK_VI_TERM | BQ2415X_MASK_RESET,
				BQ2415X_SHIFT_VI_TERM);
	}

	val = (mA * bq->init_data.resistor_sense - 3400) / 3400;
	if (val < 0)
		val = 0;
	else if (val > 7)
		val = 7;

	return bq2415x_i2c_write_mask(bq, BQ2415X_REG_CURRENT, val,
			BQ2415X_MASK_VI_TERM | BQ2415X_MASK_RESET,
			BQ2415X_SHIFT_VI_TERM);
}

/* get termination current in mA (platform data must provide resistor sense) */
static int bq2415x_get_termination_current(struct bq2415x_device *bq)
{
	int ret;

	if (bq->init_data.resistor_sense <= 0)
		return -EINVAL;

	ret = bq2415x_i2c_read_mask(bq, BQ2415X_REG_CURRENT,
			BQ2415X_MASK_VI_TERM, BQ2415X_SHIFT_VI_TERM);
	if (ret < 0)
		return ret;
	if (bq->chip == AW32257)
		return aw32257_iterm_uv(ret) / bq->init_data.resistor_sense;
	return (3400 + 3400*ret) / bq->init_data.resistor_sense;
}

/* set default value of property */
#define bq2415x_set_default_value(bq, prop) \
	do { \
		int ret = 0; \
		if (bq->init_data.prop != -1) \
			ret = bq2415x_set_##prop(bq, bq->init_data.prop); \
		if (ret < 0) \
			return ret; \
	} while (0)

/* Called with policy_lock. Disable is always retried, not edge-triggered.
 * Unknown applied state is not evidence that the physical chip is disabled.
 */
static int bq2415x_reconcile(struct bq2415x_device *bq)
{
	bool boost = bq->lifecycle == BQ2415X_READY &&
		!bq->fault_inhibit && bq->config_valid &&
		(bq->manual_boost || bq->regulator_boost);
	bool charge = bq2415x_charge_allowed(bq);
	int ret;

	bq->applied_known = false;
	ret = bq2415x_exec_command(bq, BQ2415X_CHARGER_DISABLE);
	if (ret)
		return ret;
	ret = bq2415x_exec_command(bq, boost ? BQ2415X_BOOST_MODE_ENABLE :
						BQ2415X_BOOST_MODE_DISABLE);
	if (ret)
		return ret;
	if (charge) {
		ret = bq2415x_exec_command(bq, BQ2415X_CHARGER_ENABLE);
		if (ret)
			return ret;
	}
	bq->applied_charge = charge;
	bq->applied_boost = boost;
	bq->applied_known = true;
	return 0;
}

/* Establish existing DT/profile parameters without a destructive reset or a
 * fabricated register image. A partial configuration remains invalid until an
 * explicit configuration request retries the entire sequence.
 */
static int bq2415x_configure(struct bq2415x_device *bq)
{
	int ret, limit = bq->init_data.current_limit;

	/* ABI: defaults/limit/aux/reset/raw reconfiguration is busy while the
	 * regulator owns VBUS. Charge-profile intent can be deferred, but an
	 * unrelated store must never request a transient OPA clear.
	 */
	if (bq->regulator_boost)
		return -EBUSY;
	bq->config_valid = false;
	bq->applied_known = false;
	ret = bq2415x_exec_command(bq, BQ2415X_CHARGER_DISABLE);
	if (ret)
		return ret;
	ret = bq2415x_exec_command(bq, BQ2415X_BOOST_MODE_DISABLE);
	if (ret)
		return ret;
	ret = bq2415x_exec_command(bq, bq->otg_pin_enable ?
		BQ2415X_OTG_PIN_ENABLE : BQ2415X_OTG_PIN_DISABLE);
	if (ret)
		return ret;
	ret = bq2415x_exec_command(bq, BQ2415X_CHARGE_TERMINATION_DISABLE);
	if (ret)
		return ret;
	if (bq->charge_profile >= 0 && !bq->input_override) {
		limit = 100;
		if (bq->charge_profile == BQ2415X_MODE_HOST_CHARGER)
			limit = 500;
		else if (bq->charge_profile == BQ2415X_MODE_DEDICATED_CHARGER)
			limit = 1800;
	}
	if (limit != -1) {
		ret = bq2415x_set_current_limit(bq, limit);
		if (ret)
			return ret;
	}
	bq2415x_set_default_value(bq, weak_battery_voltage);
	bq2415x_set_default_value(bq, battery_regulation_voltage);
	if (bq->init_data.resistor_sense > 0) {
		bq2415x_set_default_value(bq, charge_current);
		bq2415x_set_default_value(bq, termination_current);
	}
	if (bq->init_data.resistor_sense > 0 || bq->termination_enable >= 0) {
		ret = bq2415x_exec_command(bq, bq->termination_enable == 0 ?
			BQ2415X_CHARGE_TERMINATION_DISABLE :
			BQ2415X_CHARGE_TERMINATION_ENABLE);
		if (ret)
			return ret;
	}
	if (bq->high_impedance_enable >= 0) {
		ret = bq2415x_exec_command(bq, bq->high_impedance_enable ?
			BQ2415X_HIGH_IMPEDANCE_ENABLE : BQ2415X_HIGH_IMPEDANCE_DISABLE);
		if (ret)
			return ret;
	}
	if (bq->stat_pin_enable >= 0) {
		ret = bq2415x_exec_command(bq, bq->stat_pin_enable ?
			BQ2415X_STAT_PIN_ENABLE : BQ2415X_STAT_PIN_DISABLE);
		if (ret)
			return ret;
	}
	bq->config_valid = true;
	bq->config_deferred = false;
	return bq2415x_reconcile(bq);
}

static int bq2415x_set_defaults(struct bq2415x_device *bq)
{
	int ret;

	mutex_lock(&bq->policy_lock);
	ret = bq->lifecycle == BQ2415X_STOPPING ? -ENODEV :
		bq2415x_configure(bq);
	mutex_unlock(&bq->policy_lock);
	return ret;
}

static int bq2415x_policy_ready(struct bq2415x_device *bq)
{
	int ret;

	mutex_lock(&bq->policy_lock);
	if (bq->lifecycle != BQ2415X_STARTING) {
		ret = -ENODEV;
	} else {
		/* Replay STARTING before READY can permit charging, including
		 * OFF which matches the legacy initial mode but not INIT intent.
		 */
		if (bq->automode > 0) {
			bq->charge_profile = bq->detector_valid ?
				bq->reported_mode : BQ2415X_MODE_OFF;
			ret = bq2415x_configure(bq);
			if (ret)
				goto out;
			bq->mode = bq->charge_profile;
		}
		bq->lifecycle = BQ2415X_READY;
		ret = bq->config_valid ? bq2415x_reconcile(bq) : 0;
		if (bq->automode > 0 && !bq->detector_valid)
			mod_delayed_work(system_percpu_wq, &bq->work, HZ);
	}
out:
	mutex_unlock(&bq->policy_lock);
	return ret;
}

/**** charger mode functions ****/

/* set charger mode */
static int bq2415x_set_mode_locked(struct bq2415x_device *bq,
				 enum bq2415x_mode mode)
{
	int ret;

	if (bq->lifecycle != BQ2415X_READY)
		return -ENODEV;
	if (mode < BQ2415X_MODE_OFF || mode > BQ2415X_MODE_BOOST)
		return -EINVAL;
	bq->manual_boost = mode == BQ2415X_MODE_BOOST;
	if (!bq->manual_boost)
		bq->charge_profile = mode;
	bq->input_override = false;
	/* Requested OFF survives failures; presentation changes only on success. */
	if (bq->regulator_boost) {
		bq->config_deferred = true;
		ret = bq2415x_reconcile(bq);
	} else {
		ret = bq2415x_configure(bq);
	}
	if (ret)
		return ret;
	bq->mode = mode;
	sysfs_notify(&bq->charger->dev.kobj, NULL, "mode");
	return 0;
}

/* Lock-owning entry for non-callback callers and source regressions. */
static int bq2415x_set_mode(struct bq2415x_device *bq, enum bq2415x_mode mode)
	__attribute__((unused));
static int bq2415x_set_mode(struct bq2415x_device *bq, enum bq2415x_mode mode)
{
	int ret;

	mutex_lock(&bq->policy_lock);
	ret = bq2415x_set_mode_locked(bq, mode);
	mutex_unlock(&bq->policy_lock);
	return ret;
}

/**** USB OTG VBUS regulator (charger boost mode) ****/

#ifdef CONFIG_REGULATOR
static int bq2415x_vbus_enable(struct regulator_dev *rdev)
{
	struct bq2415x_device *bq = rdev_get_drvdata(rdev);
	bool previous;
	int ret;

	mutex_lock(&bq->policy_lock);
	if (bq->lifecycle != BQ2415X_READY) {
		ret = -ENODEV;
		goto out;
	}
	if (!bq->config_valid || bq->fault_inhibit) {
		ret = -EIO;
		goto out;
	}
	previous = bq->regulator_boost;
	bq->regulator_boost = true;
	ret = bq2415x_reconcile(bq);
	if (ret) {
		/* The framework rolls back its acquisition on callback failure.
		 * A failed transfer may have landed: keep applied state unknown
		 * and let subsequent reconciliation retry the restored intent.
		 */
		bq->regulator_boost = previous;
		bq->applied_known = false;
	} else {
		bq->mode = BQ2415X_MODE_BOOST;
		sysfs_notify(&bq->charger->dev.kobj, NULL, "mode");
	}
out:
	mutex_unlock(&bq->policy_lock);
	return ret;
}

static int bq2415x_vbus_disable(struct regulator_dev *rdev)
{
	struct bq2415x_device *bq = rdev_get_drvdata(rdev);
	bool previous;
	int ret;

	mutex_lock(&bq->policy_lock);
	if (bq->lifecycle != BQ2415X_READY) {
		ret = -ENODEV;
		goto out;
	}
	previous = bq->regulator_boost;
	bq->regulator_boost = false;
	ret = bq->config_deferred && !bq->manual_boost ?
		bq2415x_configure(bq) : bq2415x_reconcile(bq);
	if (ret) {
		/* A failed release leaves the framework consumer acquired. */
		bq->regulator_boost = previous;
		bq->applied_known = false;
	} else {
		bq->mode = bq->manual_boost ? BQ2415X_MODE_BOOST :
			bq->charge_profile < 0 ? BQ2415X_MODE_OFF : bq->charge_profile;
		sysfs_notify(&bq->charger->dev.kobj, NULL, "mode");
	}
out:
	mutex_unlock(&bq->policy_lock);
	return ret;
}

static int bq2415x_vbus_is_enabled(struct regulator_dev *rdev)
{
	struct bq2415x_device *bq = rdev_get_drvdata(rdev);
	int ret;

	mutex_lock(&bq->policy_lock);
	ret = bq->lifecycle == BQ2415X_READY ?
		bq2415x_exec_command(bq, BQ2415X_BOOST_MODE_STATUS) : -ENODEV;
	/* The core skips .enable for an already-on rail. Do not let it adopt
	 * manual or unowned physical boost without recording an acquisition.
	 * ABI: acquiring VBUS during manual-only boost returns -EBUSY; release
	 * manual boost first. Existing framework owners may coexist with it.
	 */
	if (ret >= 0 && bq->manual_boost && !bq->regulator_boost)
		ret = -EBUSY;
	else if (ret > 0 && !bq->regulator_boost)
		ret = -EIO;
	mutex_unlock(&bq->policy_lock);
	if (ret < 0)
		return ret;

	return !!ret;
}

static const struct regulator_ops bq2415x_vbus_ops = {
	.enable		= bq2415x_vbus_enable,
	.disable	= bq2415x_vbus_disable,
	.is_enabled	= bq2415x_vbus_is_enabled,
};

static const struct regulator_desc bq2415x_vbus_desc = {
	.name		= "usb_otg_vbus",
	.of_match	= of_match_ptr("usb-otg-vbus"),
	.type		= REGULATOR_VOLTAGE,
	.owner		= THIS_MODULE,
	.ops		= &bq2415x_vbus_ops,
	.fixed_uV	= 5000000,
	.n_voltages	= 1,
};

static int bq2415x_register_vbus_regulator(struct bq2415x_device *bq)
{
	struct regulator_config cfg = { };

	cfg.dev = bq->dev;
	cfg.driver_data = bq;
	cfg.of_node = bq->dev->of_node;

	bq->usb_vbus_reg = regulator_register(bq->dev, &bq2415x_vbus_desc,
						   &cfg);
	return PTR_ERR_OR_ZERO(bq->usb_vbus_reg);
}
#else
static inline int bq2415x_register_vbus_regulator(struct bq2415x_device *bq)
{
	return 0;
}
#endif

/* POWER_SUPPLY_PROP_CURRENT_MAX is in microamps, unlike chip setters. */
static bool bq2415x_update_reported_mode(struct bq2415x_device *bq, int uA)
{
	enum bq2415x_mode mode;

	if (uA <= 0)
		mode = BQ2415X_MODE_OFF;
	else if (uA < 500000)
		mode = BQ2415X_MODE_NONE;
	else if (uA < 1800000)
		mode = BQ2415X_MODE_HOST_CHARGER;
	else
		mode = BQ2415X_MODE_DEDICATED_CHARGER;

	if (bq->reported_mode == mode)
		return false;

	bq->reported_mode = mode;
	return true;
}

/* Reserve ordering before the supplier read, not after it. A later event/read
 * invalidates an in-flight snapshot even if the older read completes last.
 * Supplier callbacks run without policy_lock; STOPPING never publishes/rearms.
 */
static int bq2415x_detector_read(struct bq2415x_device *bq,
				struct power_supply *psy)
{
	union power_supply_propval prop;
	unsigned long sequence;
	bool changed;
	int ret;

	mutex_lock(&bq->policy_lock);
	if (bq->lifecycle == BQ2415X_STOPPING) {
		mutex_unlock(&bq->policy_lock);
		return -ENODEV;
	}
	sequence = ++bq->detector_sequence;
	/* An in-flight observation cannot keep an older charge permission. */
	bq->detector_valid = false;
	/* A successful same-profile retry must configure/reconcile again,
	 * even without thermal polling or a changed presentation mode.
	 */
	if (bq->automode > 0)
		bq->config_deferred = true;
	if (bq->lifecycle == BQ2415X_READY && bq->automode > 0)
		bq2415x_reconcile(bq);
	mutex_unlock(&bq->policy_lock);
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	mutex_lock(&bq->policy_lock);
	if (bq->lifecycle == BQ2415X_STOPPING ||
	    sequence != bq->detector_sequence)
		goto out;
	if (ret) {
		/* Unknown is presented as OFF, never as stale source permission. */
		bq->reported_mode = BQ2415X_MODE_OFF;
		if (bq->lifecycle == BQ2415X_READY && bq->automode > 0)
			mod_delayed_work(system_percpu_wq, &bq->work, HZ);
		goto out;
	}
	changed = bq2415x_update_reported_mode(bq, prop.intval);
	changed |= !bq->detector_valid;
	bq->detector_valid = true;
	if (!changed || bq->lifecycle == BQ2415X_STARTING)
		goto out;
	power_supply_changed(bq->charger);
	if (bq->automode > 0)
		mod_delayed_work(system_percpu_wq, &bq->work, 0);
out:
	mutex_unlock(&bq->policy_lock);
	return ret;
}

/* Consumes the reference acquired by probe, also on a failed/stale read. */
static int bq2415x_detector_snapshot(struct bq2415x_device *bq,
				    struct power_supply *psy)
{
	int ret = bq2415x_detector_read(bq, psy);

	power_supply_put(psy);
	return ret;
}

/* Revalidate unknown source state independently of the watchdog setting.
 * One supplier read per delayed invocation; errors retry after one second,
 * not through an immediate notifier/work feedback loop. Never cache a reference.
 */
static void bq2415x_detector_retry(struct bq2415x_device *bq)
{
	struct power_supply *psy;

	mutex_lock(&bq->policy_lock);
	if (bq->lifecycle != BQ2415X_READY || bq->automode <= 0 ||
	    bq->detector_valid || bq->fault_inhibit) {
		mutex_unlock(&bq->policy_lock);
		return;
	}
	mutex_unlock(&bq->policy_lock);
	if (bq->notify_node)
		psy = power_supply_get_by_reference(dev_fwnode(bq->dev),
						  "ti,usb-charger-detection");
	else
		psy = power_supply_get_by_name(bq->init_data.notify_device);
	if (!IS_ERR_OR_NULL(psy)) {
		bq2415x_detector_snapshot(bq, psy);
		return;
	}
	mutex_lock(&bq->policy_lock);
	if (bq->lifecycle == BQ2415X_READY && bq->automode > 0 &&
	    !bq->detector_valid && !bq->fault_inhibit)
		mod_delayed_work(system_percpu_wq, &bq->work, HZ);
	mutex_unlock(&bq->policy_lock);
}

static int bq2415x_notifier_call(struct notifier_block *nb,
		unsigned long val, void *v)
{
	struct bq2415x_device *bq =
		container_of(nb, struct bq2415x_device, nb);
	struct power_supply *psy = v;

	if (val != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	/* Ignore event if it was not send by notify_node/notify_device */
	if (bq->notify_node) {
		if (!psy->dev.parent ||
		    psy->dev.parent->of_node != bq->notify_node)
			return NOTIFY_OK;
	} else if (bq->init_data.notify_device) {
		if (strcmp(psy->desc->name, bq->init_data.notify_device) != 0)
			return NOTIFY_OK;
	}

	dev_dbg(bq->dev, "notifier call was called\n");

	bq2415x_detector_read(bq, psy);
	return NOTIFY_OK;
}

/**** timer functions ****/

/* enable/disable auto resetting chip timer */
static int bq2415x_set_autotimer(struct bq2415x_device *bq, int state)
{
	int ret = 0;

	/* External control serializes cancellation; workers never take it. */
	mutex_lock(&bq->timer_control);
	mutex_lock(&bq->policy_lock);
	if (bq->lifecycle != BQ2415X_READY) {
		ret = -ENODEV;
		goto out;
	}
	bq->autotimer = state;
	if (state) {
		ret = bq2415x_exec_command(bq, BQ2415X_TIMER_RESET);
		if (!ret)
			schedule_delayed_work(&bq->work, BQ2415X_TIMER_TIMEOUT * HZ);
		else
			bq->autotimer = 0;
	}
out:
	mutex_unlock(&bq->policy_lock);
	if (!state)
		cancel_delayed_work_sync(&bq->work);
	/* Cancellation also consumes pending detector/configuration replay.
	 * Keep it independent of watchdog kicks: unknown sources retry slowly,
	 * while a valid observation can replay its configuration immediately.
	 */
	mutex_lock(&bq->policy_lock);
	if (!state && bq->lifecycle == BQ2415X_READY && bq->automode > 0 &&
	    !bq->fault_inhibit &&
	    (!bq->detector_valid || bq->config_deferred ||
	     bq->reported_mode != bq->mode ||
	     (int)bq->reported_mode != bq->charge_profile))
		mod_delayed_work(system_percpu_wq, &bq->work,
				 bq->detector_valid ? 0 : HZ);
	mutex_unlock(&bq->policy_lock);
	mutex_unlock(&bq->timer_control);
	return ret;
}

/* called by bq2415x_timer_work on timer error */
static void bq2415x_timer_error(struct bq2415x_device *bq, const char *msg)
{
	bq->timer_error = msg;
	sysfs_notify(&bq->charger->dev.kobj, NULL, "timer");
	dev_err(bq->dev, "%s\n", msg);
	if (bq->automode > 0)
		bq->automode = 0;
	bq->fault_inhibit = true;
	bq2415x_set_mode_locked(bq, BQ2415X_MODE_OFF);

	/*
	 * Disable the auto-timer WITHOUT bq2415x_set_autotimer(bq, 0): that
	 * helper calls cancel_delayed_work_sync(&bq->work), and this function
	 * only ever runs from inside bq2415x_timer_work() itself, on the
	 * shared "events" workqueue. cancel_*_sync() blocks until the running
	 * instance finishes -- i.e. waits on itself -- so it self-deadlocks.
	 * The wedged worker pins the events queue forever, and any later
	 * flush_scheduled_work()/device teardown during poweroff then hangs
	 * with it (observed: "Powering off" stalls, sdiohal/hci kworkers and
	 * this bq2415x_timer_work all left in D-state, system never powers
	 * off). Every caller returns immediately after this and never
	 * re-arms the timer, so clearing the flag under the mutex is enough
	 * to stop it. The worker holds policy_lock, never timer_control;
	 * external cancellation drops policy_lock before waiting.
	 */
	bq->autotimer = 0;
}

/* delayed work function for auto resetting chip timer */
static void bq2415x_timer_work_locked(struct work_struct *work)
{
	struct bq2415x_device *bq = container_of(work, struct bq2415x_device,
						 work.work);
	int ret;
	int error;
	int boost;
	int charge;

	if (bq->automode > 0 && (bq->reported_mode != bq->mode ||
			       (int)bq->reported_mode != bq->charge_profile ||
			       bq->config_deferred)) {
		sysfs_notify(&bq->charger->dev.kobj, NULL, "reported_mode");
		if (bq2415x_set_mode_locked(bq, bq->reported_mode)) {
			bq2415x_timer_error(bq, "Automatic mode configuration failed");
			return;
		}
	}

	charge = bq2415x_exec_command(bq, BQ2415X_CHARGE_STATUS);
	if (charge >= 0 && bq->charge_status != charge) {
		power_supply_changed(bq->charger);
		bq->charge_status = charge;
	}

	if (!bq->autotimer)
		return;

	ret = bq2415x_exec_command(bq, BQ2415X_TIMER_RESET);
	if (ret < 0) {
		bq2415x_timer_error(bq, "Resetting timer failed");
		return;
	}

	boost = bq2415x_exec_command(bq, BQ2415X_BOOST_MODE_STATUS);
	if (boost < 0) {
		bq2415x_timer_error(bq, "Unknown error");
		return;
	}

	error = bq2415x_exec_command(bq, BQ2415X_FAULT_STATUS);
	if (error < 0) {
		bq2415x_timer_error(bq, "Unknown error");
		return;
	}

	if (boost) {
		switch (error) {
		/* Non fatal errors, chip is OK */
		case 0: /* No error */
			break;
		case 6: /* Timer expired */
			dev_err(bq->dev, "Timer expired\n");
			break;
		case 3: /* Battery voltage too low */
			dev_err(bq->dev, "Battery voltage to low\n");
			break;

		/* Fatal errors, disable and reset chip */
		case 1: /* Overvoltage protection (chip fried) */
			bq2415x_timer_error(bq,
				"Overvoltage protection (chip fried)");
			return;
		case 2: /* Overload */
			bq2415x_timer_error(bq, "Overload");
			return;
		case 4: /* Battery overvoltage protection */
			bq2415x_timer_error(bq,
				"Battery overvoltage protection");
			return;
		case 5: /* Thermal shutdown (too hot) */
			bq2415x_timer_error(bq,
					"Thermal shutdown (too hot)");
			return;
		case 7: /* N/A */
			bq2415x_timer_error(bq, "Unknown error");
			return;
		}
	} else {
		switch (error) {
		/* Non fatal errors, chip is OK */
		case 0: /* No error */
			break;
		case 2: /* Sleep mode */
			dev_err(bq->dev, "Sleep mode\n");
			break;
		case 3: /* Poor input source */
			dev_err(bq->dev, "Poor input source\n");
			break;
		case 6: /* Timer expired */
			dev_err(bq->dev, "Timer expired\n");
			break;
		case 7: /* No battery */
			dev_err(bq->dev, "No battery\n");
			break;

		/* Fatal errors, disable and reset chip */
		case 1: /* Overvoltage protection (chip fried) */
			bq2415x_timer_error(bq,
				"Overvoltage protection (chip fried)");
			return;
		case 4: /* Battery overvoltage protection */
			bq2415x_timer_error(bq,
				"Battery overvoltage protection");
			return;
		case 5: /* Thermal shutdown (too hot) */
			bq2415x_timer_error(bq,
				"Thermal shutdown (too hot)");
			return;
		}
	}

	schedule_delayed_work(&bq->work, BQ2415X_TIMER_TIMEOUT * HZ);
}

static void bq2415x_timer_work(struct work_struct *work)
{
	struct bq2415x_device *bq = container_of(work, struct bq2415x_device, work.work);

	bq2415x_detector_retry(bq);
	mutex_lock(&bq->policy_lock);
	if (bq->lifecycle == BQ2415X_READY)
		bq2415x_timer_work_locked(work);
	mutex_unlock(&bq->policy_lock);
}

/**** power supply interface code ****/

static enum power_supply_property bq2415x_power_supply_props[] = {
	/* TODO: maybe add more power supply properties */
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_ONLINE,
};

static int bq2415x_power_supply_get_property_locked(struct power_supply *psy,
					     enum power_supply_property psp,
					     union power_supply_propval *val)
{
	struct bq2415x_device *bq = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = bq2415x_exec_command(bq, BQ2415X_CHARGE_STATUS);
		if (ret < 0)
			return ret;
		else if (ret == 0) /* Ready */
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else if (ret == 1) /* Charge in progress */
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (ret == 2) /* Charge done */
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = bq->model;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		/* VBUS is present for all charging and fault states,
		 * except the 'Ready' state.
		 */
		ret = bq2415x_exec_command(bq, BQ2415X_CHARGE_STATUS);
		if (ret < 0)
			return ret;
		val->intval = ret > 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int bq2415x_power_supply_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct bq2415x_device *bq = power_supply_get_drvdata(psy);
	int ret;

	mutex_lock(&bq->policy_lock);
	ret = bq->lifecycle == BQ2415X_READY ?
		bq2415x_power_supply_get_property_locked(psy, psp, val) : -ENODEV;
	mutex_unlock(&bq->policy_lock);
	return ret;
}

static void bq2415x_policy_stop(struct bq2415x_device *bq)
{
	int ret;

	mutex_lock(&bq->timer_control);
	mutex_lock(&bq->policy_lock);
	bq->lifecycle = BQ2415X_STOPPING;
	bq->autotimer = 0;
	bq->automode = 0;
	ret = bq2415x_reconcile(bq);
	if (ret)
		dev_err(bq->dev, "stopping charge policy failed: %d\n", ret);
	mutex_unlock(&bq->policy_lock);
	mutex_unlock(&bq->timer_control);
	/* Disconnect producers and drain without any worker-required lock. */
	if (bq->notifier_registered) {
		power_supply_unreg_notifier(&bq->nb);
		bq->notifier_registered = false;
	}
	cancel_delayed_work_sync(&bq->work);
	cancel_delayed_work_sync(&bq->temp_work);
#ifdef CONFIG_REGULATOR
	if (!IS_ERR_OR_NULL(bq->usb_vbus_reg)) {
		regulator_unregister(bq->usb_vbus_reg);
		bq->usb_vbus_reg = NULL;
	}
#endif
}

static void bq2415x_power_supply_exit(struct bq2415x_device *bq)
{
	bq2415x_policy_stop(bq);
	power_supply_unregister(bq->charger);
	bq->charger = NULL;
	kfree(bq->model);
}

/**** additional sysfs entries for power supply interface ****/

/* show *_status entries */
static ssize_t bq2415x_sysfs_show_status(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct power_supply *psy = dev_to_psy(dev);
	struct bq2415x_device *bq = power_supply_get_drvdata(psy);
	enum bq2415x_command command;
	int ret;

	if (strcmp(attr->attr.name, "otg_status") == 0)
		command = BQ2415X_OTG_STATUS;
	else if (strcmp(attr->attr.name, "charge_status") == 0)
		command = BQ2415X_CHARGE_STATUS;
	else if (strcmp(attr->attr.name, "boost_status") == 0)
		command = BQ2415X_BOOST_STATUS;
	else if (strcmp(attr->attr.name, "fault_status") == 0)
		command = BQ2415X_FAULT_STATUS;
	else
		return -EINVAL;

	ret = bq2415x_exec_command(bq, command);
	if (ret < 0)
		return ret;
	return sysfs_emit(buf, "%d\n", ret);
}

/*
 * set timer entry:
 *    auto - enable auto mode
 *    off - disable auto mode
 *    (other values) - reset chip timer
 */
static ssize_t bq2415x_sysfs_set_timer(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf,
				       size_t count)
{
	struct power_supply *psy = dev_to_psy(dev);
	struct bq2415x_device *bq = power_supply_get_drvdata(psy);
	int ret = 0;

	if (strncmp(buf, "auto", 4) == 0)
		ret = bq2415x_set_autotimer(bq, 1);
	else if (strncmp(buf, "off", 3) == 0)
		ret = bq2415x_set_autotimer(bq, 0);
	else {
		mutex_lock(&bq->policy_lock);
		ret = bq->lifecycle == BQ2415X_READY ?
			bq2415x_exec_command(bq, BQ2415X_TIMER_RESET) : -ENODEV;
		mutex_unlock(&bq->policy_lock);
	}

	if (ret < 0)
		return ret;
	return count;
}

/* show timer entry (auto or off) */
static ssize_t bq2415x_sysfs_show_timer(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct power_supply *psy = dev_to_psy(dev);
	struct bq2415x_device *bq = power_supply_get_drvdata(psy);

	if (bq->timer_error)
		return sysfs_emit(buf, "%s\n", bq->timer_error);

	if (bq->autotimer)
		return sysfs_emit(buf, "auto\n");
	return sysfs_emit(buf, "off\n");
}

/*
 * set mode entry:
 *    auto - if automode is supported, enable it and set mode to reported
 *    off - explicit charge-off request
 *    none - unknown charger, charge requested with a 100mA input limit
 *    host - charging mode for host/hub chargers (current limit 500mA)
 *    dedicated - charging mode for dedicated chargers (unlimited current limit)
 *    boost - disable charger and enable boost mode
 */
static ssize_t bq2415x_sysfs_set_mode_locked(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t count)
{
	struct power_supply *psy = dev_to_psy(dev);
	struct bq2415x_device *bq = power_supply_get_drvdata(psy);
	enum bq2415x_mode mode;
	int ret = 0;

	if (strncmp(buf, "auto", 4) == 0) {
		if (bq->automode < 0)
			return -EINVAL;
		bq->automode = 1;
		mode = bq->reported_mode;
	} else if (strncmp(buf, "off", 3) == 0) {
		if (bq->automode > 0)
			bq->automode = 0;
		mode = BQ2415X_MODE_OFF;
	} else if (strncmp(buf, "none", 4) == 0) {
		if (bq->automode > 0)
			bq->automode = 0;
		mode = BQ2415X_MODE_NONE;
	} else if (strncmp(buf, "host", 4) == 0) {
		if (bq->automode > 0)
			bq->automode = 0;
		mode = BQ2415X_MODE_HOST_CHARGER;
	} else if (strncmp(buf, "dedicated", 9) == 0) {
		if (bq->automode > 0)
			bq->automode = 0;
		mode = BQ2415X_MODE_DEDICATED_CHARGER;
	} else if (strncmp(buf, "boost", 5) == 0) {
		if (bq->automode > 0)
			bq->automode = 0;
		mode = BQ2415X_MODE_BOOST;
	} else if (strncmp(buf, "reset", 5) == 0) {
		ret = bq2415x_reset_chip(bq);
		if (ret)
			return ret;
		ret = bq2415x_configure(bq);
		if (ret)
			return ret;
		if (bq->automode <= 0)
			return count;
		bq->automode = 1;
		mode = bq->reported_mode;
	} else {
		return -EINVAL;
	}

	ret = bq2415x_set_mode_locked(bq, mode);
	if (ret < 0)
		return ret;
	return count;
}

static ssize_t bq2415x_sysfs_set_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct bq2415x_device *bq = power_supply_get_drvdata(dev_to_psy(dev));
	ssize_t ret;

	mutex_lock(&bq->policy_lock);
	ret = bq->lifecycle == BQ2415X_READY ?
		bq2415x_sysfs_set_mode_locked(dev, attr, buf, count) : -ENODEV;
	mutex_unlock(&bq->policy_lock);
	return ret;
}

/* show mode entry (auto, none, host, dedicated or boost) */
static ssize_t bq2415x_sysfs_show_mode(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct power_supply *psy = dev_to_psy(dev);
	struct bq2415x_device *bq = power_supply_get_drvdata(psy);
	ssize_t ret = 0;

	if (bq->automode > 0)
		ret += sysfs_emit_at(buf, ret, "auto (");

	switch (bq->mode) {
	case BQ2415X_MODE_OFF:
		ret += sysfs_emit_at(buf, ret, "off");
		break;
	case BQ2415X_MODE_NONE:
		ret += sysfs_emit_at(buf, ret, "none");
		break;
	case BQ2415X_MODE_HOST_CHARGER:
		ret += sysfs_emit_at(buf, ret, "host");
		break;
	case BQ2415X_MODE_DEDICATED_CHARGER:
		ret += sysfs_emit_at(buf, ret, "dedicated");
		break;
	case BQ2415X_MODE_BOOST:
		ret += sysfs_emit_at(buf, ret, "boost");
		break;
	}

	if (bq->automode > 0)
		ret += sysfs_emit_at(buf, ret, ")");

	ret += sysfs_emit_at(buf, ret, "\n");
	return ret;
}

/* show reported_mode entry (none, host, dedicated or boost) */
static ssize_t bq2415x_sysfs_show_reported_mode(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct power_supply *psy = dev_to_psy(dev);
	struct bq2415x_device *bq = power_supply_get_drvdata(psy);

	if (bq->automode < 0)
		return -EINVAL;

	switch (bq->reported_mode) {
	case BQ2415X_MODE_OFF:
		return sysfs_emit(buf, "off\n");
	case BQ2415X_MODE_NONE:
		return sysfs_emit(buf, "none\n");
	case BQ2415X_MODE_HOST_CHARGER:
		return sysfs_emit(buf, "host\n");
	case BQ2415X_MODE_DEDICATED_CHARGER:
		return sysfs_emit(buf, "dedicated\n");
	case BQ2415X_MODE_BOOST:
		return sysfs_emit(buf, "boost\n");
	}

	return -EINVAL;
}

/* directly set raw value to chip register, format: 'register value' */
static ssize_t bq2415x_sysfs_set_registers_locked(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t count)
{
	struct power_supply *psy = dev_to_psy(dev);
	struct bq2415x_device *bq = power_supply_get_drvdata(psy);
	ssize_t ret = 0;
	unsigned int reg;
	unsigned int val;

	if (bq2415x_guarded(bq))
		return -EPERM;

	if (sscanf(buf, "%x %x", &reg, &val) != 2)
		return -EINVAL;

	if (reg > 4 || val > 255)
		return -EINVAL;

	if (bq->regulator_boost)
		return -EBUSY;
	ret = bq2415x_i2c_write(bq, reg, val);
	if (ret < 0)
		return ret;
	return count;
}

static ssize_t bq2415x_sysfs_set_registers(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct bq2415x_device *bq = power_supply_get_drvdata(dev_to_psy(dev));
	ssize_t ret;

	mutex_lock(&bq->policy_lock);
	ret = bq->lifecycle == BQ2415X_READY ?
		bq2415x_sysfs_set_registers_locked(dev, attr, buf, count) : -ENODEV;
	mutex_unlock(&bq->policy_lock);
	return ret;
}

/* print value of chip register, format: 'register=value' */
static ssize_t bq2415x_sysfs_print_reg(struct bq2415x_device *bq,
				       u8 reg,
				       char *buf)
{
	int ret = bq2415x_i2c_read(bq, reg);

	if (ret < 0)
		return sysfs_emit(buf, "%#.2x=error %d\n", reg, ret);
	return sysfs_emit(buf, "%#.2x=%#.2x\n", reg, ret);
}

/* show all raw values of chip register, format per line: 'register=value' */
static ssize_t bq2415x_sysfs_show_registers(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct power_supply *psy = dev_to_psy(dev);
	struct bq2415x_device *bq = power_supply_get_drvdata(psy);
	ssize_t ret = 0;

	ret += bq2415x_sysfs_print_reg(bq, BQ2415X_REG_STATUS, buf+ret);
	ret += bq2415x_sysfs_print_reg(bq, BQ2415X_REG_CONTROL, buf+ret);
	ret += bq2415x_sysfs_print_reg(bq, BQ2415X_REG_VOLTAGE, buf+ret);
	ret += bq2415x_sysfs_print_reg(bq, BQ2415X_REG_VENDER, buf+ret);
	ret += bq2415x_sysfs_print_reg(bq, BQ2415X_REG_CURRENT, buf+ret);
	return ret;
}

/* set current and voltage limit entries (in mA or mV) */
static ssize_t bq2415x_sysfs_set_limit_locked(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf,
				       size_t count)
{
	struct power_supply *psy = dev_to_psy(dev);
	struct bq2415x_device *bq = power_supply_get_drvdata(psy);
	long val;
	int ret;

	if (bq->regulator_boost)
		return -EBUSY;
	if (kstrtol(buf, 10, &val) < 0 || val < 0 || val > INT_MAX)
		return -EINVAL;

	/* Current encoders multiply by resistor_sense in signed int arithmetic. */
	if ((strcmp(attr->attr.name, "charge_current") == 0 ||
	     strcmp(attr->attr.name, "termination_current") == 0) &&
	    (bq->init_data.resistor_sense <= 0 ||
	     val > INT_MAX / bq->init_data.resistor_sense))
		return -EINVAL;
	if (strcmp(attr->attr.name, "current_limit") == 0) {
		bq->init_data.current_limit = val;
		bq->input_override = true;
	} else if (strcmp(attr->attr.name, "weak_battery_voltage") == 0)
		bq->init_data.weak_battery_voltage = val;
	else if (strcmp(attr->attr.name, "battery_regulation_voltage") == 0)
		bq->init_data.battery_regulation_voltage = val;
	else if (strcmp(attr->attr.name, "charge_current") == 0)
		bq->init_data.charge_current = val;
	else if (strcmp(attr->attr.name, "termination_current") == 0)
		bq->init_data.termination_current = val;
	else
		return -EINVAL;

	ret = bq2415x_configure(bq);
	if (ret < 0)
		return ret;
	return count;
}

static ssize_t bq2415x_sysfs_set_limit(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct bq2415x_device *bq = power_supply_get_drvdata(dev_to_psy(dev));
	ssize_t ret;

	mutex_lock(&bq->policy_lock);
	ret = bq->lifecycle == BQ2415X_READY ?
		bq2415x_sysfs_set_limit_locked(dev, attr, buf, count) : -ENODEV;
	mutex_unlock(&bq->policy_lock);
	return ret;
}

/* show current and voltage limit entries (in mA or mV) */
static ssize_t bq2415x_sysfs_show_limit(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct power_supply *psy = dev_to_psy(dev);
	struct bq2415x_device *bq = power_supply_get_drvdata(psy);
	int ret;

	if (strcmp(attr->attr.name, "current_limit") == 0)
		ret = bq2415x_get_current_limit(bq);
	else if (strcmp(attr->attr.name, "weak_battery_voltage") == 0)
		ret = bq2415x_get_weak_battery_voltage(bq);
	else if (strcmp(attr->attr.name, "battery_regulation_voltage") == 0)
		ret = bq2415x_get_battery_regulation_voltage(bq);
	else if (strcmp(attr->attr.name, "charge_current") == 0)
		ret = bq2415x_get_charge_current(bq);
	else if (strcmp(attr->attr.name, "termination_current") == 0)
		ret = bq2415x_get_termination_current(bq);
	else
		return -EINVAL;

	if (ret < 0)
		return ret;
	return sysfs_emit(buf, "%d\n", ret);
}

/* set *_enable entries */
static ssize_t bq2415x_sysfs_set_enable_locked(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct power_supply *psy = dev_to_psy(dev);
	struct bq2415x_device *bq = power_supply_get_drvdata(psy);
	enum bq2415x_command command;
	long val;
	bool configured;
	int *desired;
	int ret;

	if (kstrtol(buf, 10, &val) < 0)
		return -EINVAL;

	if (strcmp(attr->attr.name, "charge_termination_enable") == 0) {
		desired = &bq->termination_enable;
		command = val ? BQ2415X_CHARGE_TERMINATION_ENABLE :
			BQ2415X_CHARGE_TERMINATION_DISABLE;
	} else if (strcmp(attr->attr.name, "high_impedance_enable") == 0) {
		desired = &bq->high_impedance_enable;
		command = val ? BQ2415X_HIGH_IMPEDANCE_ENABLE :
			BQ2415X_HIGH_IMPEDANCE_DISABLE;
	} else if (strcmp(attr->attr.name, "otg_pin_enable") == 0) {
		desired = &bq->otg_pin_enable;
		command = val ? BQ2415X_OTG_PIN_ENABLE :
			BQ2415X_OTG_PIN_DISABLE;
	} else if (strcmp(attr->attr.name, "stat_pin_enable") == 0) {
		desired = &bq->stat_pin_enable;
		command = val ? BQ2415X_STAT_PIN_ENABLE :
			BQ2415X_STAT_PIN_DISABLE;
	} else {
		return -EINVAL;
	}

	if (command == BQ2415X_OTG_PIN_ENABLE && bq2415x_guarded(bq))
		return -EPERM;
	if (bq->regulator_boost)
		return -EBUSY;
	/* Inhibit before the auxiliary transaction, never enable in preparation
	 * for a write which may invalidate configuration. A failed disable does
	 * not prove hardware safety, and a failed update retains the inhibit.
	 */
	configured = bq->config_valid;
	bq->config_valid = false;
	bq->applied_known = false;
	ret = bq2415x_exec_command(bq, BQ2415X_CHARGER_DISABLE);
	if (ret)
		return ret;
	ret = bq2415x_exec_command(bq, command);
	if (ret < 0) {
		/* Best effort only; preserve the original transaction error. */
		bq2415x_exec_command(bq, BQ2415X_CHARGER_DISABLE);
		bq->applied_known = false;
		return ret;
	}
	*desired = !!val;
	bq->config_valid = configured;
	ret = bq2415x_reconcile(bq);
	if (ret)
		return ret;
	return count;
}

static ssize_t bq2415x_sysfs_set_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct bq2415x_device *bq = power_supply_get_drvdata(dev_to_psy(dev));
	ssize_t ret;

	mutex_lock(&bq->policy_lock);
	ret = bq->lifecycle == BQ2415X_READY ?
		bq2415x_sysfs_set_enable_locked(dev, attr, buf, count) : -ENODEV;
	mutex_unlock(&bq->policy_lock);
	return ret;
}

/* show *_enable entries */
static ssize_t bq2415x_sysfs_show_enable(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct power_supply *psy = dev_to_psy(dev);
	struct bq2415x_device *bq = power_supply_get_drvdata(psy);
	enum bq2415x_command command;
	int ret;

	if (strcmp(attr->attr.name, "charge_termination_enable") == 0)
		command = BQ2415X_CHARGE_TERMINATION_STATUS;
	else if (strcmp(attr->attr.name, "high_impedance_enable") == 0)
		command = BQ2415X_HIGH_IMPEDANCE_STATUS;
	else if (strcmp(attr->attr.name, "otg_pin_enable") == 0)
		command = BQ2415X_OTG_PIN_STATUS;
	else if (strcmp(attr->attr.name, "stat_pin_enable") == 0)
		command = BQ2415X_STAT_PIN_STATUS;
	else
		return -EINVAL;

	ret = bq2415x_exec_command(bq, command);
	if (ret < 0)
		return ret;
	return sysfs_emit(buf, "%d\n", ret);
}

/* One publication boundary for every custom diagnostic read. */
static ssize_t bq2415x_sysfs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq2415x_device *bq = power_supply_get_drvdata(dev_to_psy(dev));
	const char *name = attr->attr.name;
	ssize_t ret;

	mutex_lock(&bq->policy_lock);
	if (bq->lifecycle != BQ2415X_READY)
		ret = -ENODEV;
	else if (!strcmp(name, "mode"))
		ret = bq2415x_sysfs_show_mode(dev, attr, buf);
	else if (!strcmp(name, "reported_mode"))
		ret = bq2415x_sysfs_show_reported_mode(dev, attr, buf);
	else if (!strcmp(name, "timer"))
		ret = bq2415x_sysfs_show_timer(dev, attr, buf);
	else if (!strcmp(name, "registers"))
		ret = bq2415x_sysfs_show_registers(dev, attr, buf);
	else if (strstr(name, "_status"))
		ret = bq2415x_sysfs_show_status(dev, attr, buf);
	else if (strstr(name, "_enable"))
		ret = bq2415x_sysfs_show_enable(dev, attr, buf);
	else
		ret = bq2415x_sysfs_show_limit(dev, attr, buf);
	mutex_unlock(&bq->policy_lock);
	return ret;
}

static DEVICE_ATTR(current_limit, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show, bq2415x_sysfs_set_limit);
static DEVICE_ATTR(weak_battery_voltage, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show, bq2415x_sysfs_set_limit);
static DEVICE_ATTR(battery_regulation_voltage, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show, bq2415x_sysfs_set_limit);
static DEVICE_ATTR(charge_current, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show, bq2415x_sysfs_set_limit);
static DEVICE_ATTR(termination_current, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show, bq2415x_sysfs_set_limit);

static DEVICE_ATTR(charge_termination_enable, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show, bq2415x_sysfs_set_enable);
static DEVICE_ATTR(high_impedance_enable, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show, bq2415x_sysfs_set_enable);
static DEVICE_ATTR(otg_pin_enable, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show, bq2415x_sysfs_set_enable);
static DEVICE_ATTR(stat_pin_enable, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show, bq2415x_sysfs_set_enable);

static DEVICE_ATTR(reported_mode, S_IRUGO,
		bq2415x_sysfs_show, NULL);
static DEVICE_ATTR(mode, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show, bq2415x_sysfs_set_mode);
static DEVICE_ATTR(timer, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show, bq2415x_sysfs_set_timer);

static DEVICE_ATTR(registers, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show, bq2415x_sysfs_set_registers);

static DEVICE_ATTR(otg_status, S_IRUGO, bq2415x_sysfs_show, NULL);
static DEVICE_ATTR(charge_status, S_IRUGO, bq2415x_sysfs_show, NULL);
static DEVICE_ATTR(boost_status, S_IRUGO, bq2415x_sysfs_show, NULL);
static DEVICE_ATTR(fault_status, S_IRUGO, bq2415x_sysfs_show, NULL);

static struct attribute *bq2415x_sysfs_attrs[] = {
	/*
	 * TODO: some (appropriate) of these attrs should be switched to
	 * use power supply class props.
	 */
	&dev_attr_current_limit.attr,
	&dev_attr_weak_battery_voltage.attr,
	&dev_attr_battery_regulation_voltage.attr,
	&dev_attr_charge_current.attr,
	&dev_attr_termination_current.attr,

	&dev_attr_charge_termination_enable.attr,
	&dev_attr_high_impedance_enable.attr,
	&dev_attr_otg_pin_enable.attr,
	&dev_attr_stat_pin_enable.attr,

	&dev_attr_reported_mode.attr,
	&dev_attr_mode.attr,
	&dev_attr_timer.attr,

	&dev_attr_registers.attr,

	&dev_attr_otg_status.attr,
	&dev_attr_charge_status.attr,
	&dev_attr_boost_status.attr,
	&dev_attr_fault_status.attr,
	NULL,
};

ATTRIBUTE_GROUPS(bq2415x_sysfs);

static int bq2415x_power_supply_init(struct bq2415x_device *bq)
{
	int ret;
	int chip;
	char revstr[8];
	struct power_supply_config psy_cfg = {
		.drv_data = bq,
		.fwnode = dev_fwnode(bq->dev),
		.attr_grp = bq2415x_sysfs_groups,
	};

	bq->charger_desc.name = bq->name;
	bq->charger_desc.type = POWER_SUPPLY_TYPE_USB;
	bq->charger_desc.properties = bq2415x_power_supply_props;
	bq->charger_desc.num_properties =
			ARRAY_SIZE(bq2415x_power_supply_props);
	bq->charger_desc.get_property = bq2415x_power_supply_get_property;

	ret = bq2415x_detect_chip(bq);
	if (ret < 0)
		chip = BQUNKNOWN;
	else
		chip = ret;

	ret = bq2415x_detect_revision(bq);
	if (ret < 0)
		strscpy(revstr, "unknown", sizeof(revstr));
	else
		sprintf(revstr, "1.%d", ret);

	bq->model = kasprintf(GFP_KERNEL,
				"chip %s, revision %s, vender code %.3d",
				bq2415x_chip_name[chip], revstr,
				bq2415x_get_vender_code(bq));
	if (!bq->model) {
		dev_err(bq->dev, "failed to allocate model name\n");
		return -ENOMEM;
	}

	bq->charger = power_supply_register(bq->dev, &bq->charger_desc,
					    &psy_cfg);
	if (IS_ERR(bq->charger)) {
		kfree(bq->model);
		return PTR_ERR(bq->charger);
	}

	return 0;
}

/**** charge temperature guard ****/

#define BQ2415X_TEMP_POLL_MS		15000
#define BQ2415X_TEMP_HYSTERESIS		30	/* deci-celsius */

static void bq2415x_temp_work(struct work_struct *work)
{
	struct bq2415x_device *bq =
		container_of(work, struct bq2415x_device, temp_work.work);
	union power_supply_propval val;
	struct power_supply *gauge;
	int min, max, ret = -ENODEV;

	mutex_lock(&bq->policy_lock);
	if (bq->lifecycle != BQ2415X_READY) {
		mutex_unlock(&bq->policy_lock);
		return;
	}
	mutex_unlock(&bq->policy_lock);
	/* Supplier locks must not nest inside the charger lock. */
	gauge = power_supply_get_by_reference(dev_fwnode(bq->dev), "fuel-gauge");
	if (!IS_ERR_OR_NULL(gauge)) {
		ret = power_supply_get_property(gauge, POWER_SUPPLY_PROP_TEMP, &val);
		power_supply_put(gauge);
	}
	mutex_lock(&bq->policy_lock);
	if (bq->lifecycle != BQ2415X_READY)
		goto out;
	bq->sensor_valid = ret == 0;
	min = bq->temp_min;
	max = bq->temp_max;
	if (bq->thermal_trip) {
		if (min != INT_MIN)
			min += BQ2415X_TEMP_HYSTERESIS;
		if (max != INT_MAX)
			max -= BQ2415X_TEMP_HYSTERESIS;
	}
	if (!ret)
		bq->thermal_trip = val.intval < min || val.intval > max;
	/* Record inhibition before I/O, including failed disable requests. */
	bq->temp_blocked = !bq->sensor_valid || bq->thermal_trip;
	ret = bq2415x_reconcile(bq);
	if (ret)
		dev_err(bq->dev, "charge policy reconciliation failed: %d\n", ret);
	schedule_delayed_work(&bq->temp_work,
			      msecs_to_jiffies(BQ2415X_TEMP_POLL_MS));
out:
	mutex_unlock(&bq->policy_lock);
}

static int bq2415x_temp_guard_init(struct bq2415x_device *bq)
{
	struct fwnode_handle *node = dev_fwnode(bq->dev), *battery, *gauge;
	u32 range[2];
	s32 min, max;
	int ret = 0;

	/* Parse BEFORE publication: power_supply's battery-info parser ignores
	 * malformed operating-range arrays. Absence is legacy, errors are not.
	 */
	if (!fwnode_property_present(node, "monitored-battery"))
		return 0;
	battery = fwnode_find_reference(node, "monitored-battery", 0);
	if (IS_ERR_OR_NULL(battery))
		return battery ? PTR_ERR(battery) : -ENODEV;
	if (!fwnode_device_is_available(battery)) {
		ret = -ENODEV;
		goto out;
	}
	if (!fwnode_property_present(battery, "operating-range-celsius"))
		goto out;
	ret = fwnode_property_count_u32(battery, "operating-range-celsius");
	if (ret != ARRAY_SIZE(range)) {
		ret = ret < 0 ? ret : -EINVAL;
		goto out;
	}
	ret = fwnode_property_read_u32_array(battery, "operating-range-celsius",
					     range, ARRAY_SIZE(range));
	if (ret)
		goto out;
	min = (s32)range[0];
	max = (s32)range[1];
	if (min > max || min <= INT_MIN / 10 || max >= INT_MAX / 10) {
		ret = -EINVAL;
		goto out;
	}
	/* Leave room for deci-degree conversion and recovery hysteresis. */
	if (min * 10 < INT_MIN + BQ2415X_TEMP_HYSTERESIS ||
	    max * 10 > INT_MAX - BQ2415X_TEMP_HYSTERESIS) {
		ret = -EINVAL;
		goto out;
	}
	if (fwnode_property_present(node, "fuel-gauge")) {
		gauge = fwnode_find_reference(node, "fuel-gauge", 0);
		if (IS_ERR_OR_NULL(gauge)) {
			ret = gauge ? PTR_ERR(gauge) : -ENODEV;
			goto out;
		}
		ret = fwnode_device_is_available(gauge) ? 0 : -ENODEV;
		fwnode_handle_put(gauge);
		if (ret)
			goto out;
	}
	bq->temp_min = min * 10;
	bq->temp_max = max * 10;
out:
	fwnode_handle_put(battery);
	return ret;
}

/* main bq2415x probe function */
static int bq2415x_probe(struct i2c_client *client)
{
	const struct i2c_device_id *id = i2c_client_get_device_id(client);
	int ret;
	int num;
	char *name = NULL;
	struct bq2415x_device *bq = NULL;
	struct device_node *np = client->dev.of_node;
	struct bq2415x_platform_data *pdata = client->dev.platform_data;
	const struct acpi_device_id *acpi_id = NULL;
	struct power_supply *notify_psy = NULL;

	if (!np && !pdata && !ACPI_HANDLE(&client->dev)) {
		dev_err(&client->dev, "Neither devicetree, nor platform data, nor ACPI support\n");
		return -ENODEV;
	}

	/* Get new ID for the new device */
	mutex_lock(&bq2415x_id_mutex);
	num = idr_alloc(&bq2415x_id, client, 0, 0, GFP_KERNEL);
	mutex_unlock(&bq2415x_id_mutex);
	if (num < 0)
		return num;

	if (id) {
		name = kasprintf(GFP_KERNEL, "%s-%d", id->name, num);
	} else if (ACPI_HANDLE(&client->dev)) {
		acpi_id =
			acpi_match_device(client->dev.driver->acpi_match_table,
					  &client->dev);
		if (!acpi_id) {
			dev_err(&client->dev, "failed to match device name\n");
			ret = -ENODEV;
			goto error_1;
		}
		name = kasprintf(GFP_KERNEL, "%s-%d", acpi_id->id, num);
	}
	if (!name) {
		dev_err(&client->dev, "failed to allocate device name\n");
		ret = -ENOMEM;
		goto error_1;
	}

	bq = devm_kzalloc(&client->dev, sizeof(*bq), GFP_KERNEL);
	if (!bq) {
		ret = -ENOMEM;
		goto error_2;
	}

	bq2415x_policy_init(bq);
	i2c_set_clientdata(client, bq);

	bq->id = num;
	bq->dev = &client->dev;
	if (id)
		bq->chip = id->driver_data;
	else if (ACPI_HANDLE(bq->dev))
		bq->chip = acpi_id->driver_data;
	bq->name = name;
	bq->mode = BQ2415X_MODE_OFF;
	bq->reported_mode = BQ2415X_MODE_OFF;
	bq->autotimer = 0;
	bq->automode = 0;

	if (np || ACPI_HANDLE(bq->dev)) {
		ret = device_property_read_u32(bq->dev,
					       "ti,current-limit",
					       &bq->init_data.current_limit);
		if (ret)
			goto error_2;
		ret = device_property_read_u32(bq->dev,
					"ti,weak-battery-voltage",
					&bq->init_data.weak_battery_voltage);
		if (ret)
			goto error_2;
		ret = device_property_read_u32(bq->dev,
				"ti,battery-regulation-voltage",
				&bq->init_data.battery_regulation_voltage);
		if (ret)
			goto error_2;
		ret = device_property_read_u32(bq->dev,
					       "ti,charge-current",
					       &bq->init_data.charge_current);
		if (ret)
			goto error_2;
		ret = device_property_read_u32(bq->dev,
				"ti,termination-current",
				&bq->init_data.termination_current);
		if (ret)
			goto error_2;
		ret = device_property_read_u32(bq->dev,
					       "ti,resistor-sense",
					       &bq->init_data.resistor_sense);
		if (ret)
			goto error_2;
		if (np)
			bq->notify_node = of_parse_phandle(np,
						"ti,usb-charger-detection", 0);
	} else {
		memcpy(&bq->init_data, pdata, sizeof(bq->init_data));
	}

	ret = bq2415x_temp_guard_init(bq);
	if (ret)
		goto error_2;
	if (!bq2415x_guarded(bq)) {
		ret = bq2415x_reset_chip(bq);
		if (ret)
			goto error_2;
	}
	ret = bq2415x_set_defaults(bq);
	if (ret) {
		dev_err(bq->dev, "failed to set default values: %d\n", ret);
		goto error_2;
	}
	ret = bq2415x_power_supply_init(bq);
	if (ret)
		goto error_2;

	if (bq->notify_node || bq->init_data.notify_device) {
		bq->automode = 1;
		bq->nb.notifier_call = bq2415x_notifier_call;
		ret = power_supply_reg_notifier(&bq->nb);
		if (ret) {
			dev_err(bq->dev, "failed to reg notifier: %d\n", ret);
			goto error_3;
		}

		bq->notifier_registered = true;
		dev_info(bq->dev, "automode supported, waiting for events\n");
	} else {
		bq->automode = -1;
		dev_info(bq->dev, "automode not supported\n");
	}

	/* Query for initial reported_mode and set it */
	if (bq->nb.notifier_call) {
		if (np) {
			notify_psy = power_supply_get_by_reference(of_fwnode_handle(np),
						"ti,usb-charger-detection");
			if (IS_ERR(notify_psy))
				notify_psy = NULL;
		} else if (bq->init_data.notify_device) {
			notify_psy = power_supply_get_by_name(
						bq->init_data.notify_device);
		}
	}
	if (notify_psy) {
		/* STARTING capture and READY replay share notifier ordering. */
		bq2415x_detector_snapshot(bq, notify_psy);
	}

	ret = bq2415x_policy_ready(bq);
	if (ret)
		goto error_3;
	/* Regulator registration can invoke ops; it must see a ready policy. */
	ret = bq2415x_register_vbus_regulator(bq);
	if (ret)
		goto error_3;
	ret = bq2415x_set_autotimer(bq, 1);
	if (ret)
		goto error_3;

	if (bq2415x_guarded(bq)) {
		mutex_lock(&bq->policy_lock);
		schedule_delayed_work(&bq->temp_work, 0);
		mutex_unlock(&bq->policy_lock);
		dev_info(bq->dev, "charge temperature guard: %d.%dC to %d.%dC\n",
			 bq->temp_min / 10, abs(bq->temp_min % 10),
			 bq->temp_max / 10, abs(bq->temp_max % 10));
	} else {
		dev_warn(bq->dev,
			 "no operating-range-celsius on the battery, charging is not temperature limited\n");
	}

	dev_info(bq->dev, "driver registered\n");
	return 0;

error_3:
	bq2415x_power_supply_exit(bq);
error_2:
	if (bq) {
		if (bq->lifecycle != BQ2415X_STOPPING)
			bq2415x_policy_stop(bq);
		of_node_put(bq->notify_node);
	}
	kfree(name);
error_1:
	mutex_lock(&bq2415x_id_mutex);
	idr_remove(&bq2415x_id, num);
	mutex_unlock(&bq2415x_id_mutex);

	return ret;
}

/* main bq2415x remove function */

static void bq2415x_remove(struct i2c_client *client)
{
	struct bq2415x_device *bq = i2c_get_clientdata(client);

	bq2415x_power_supply_exit(bq);
	of_node_put(bq->notify_node);

	mutex_lock(&bq2415x_id_mutex);
	idr_remove(&bq2415x_id, bq->id);
	mutex_unlock(&bq2415x_id_mutex);

	dev_info(bq->dev, "driver unregistered\n");

	kfree(bq->name);
}

static const struct i2c_device_id bq2415x_i2c_id_table[] = {
	{ "bq2415x", BQUNKNOWN },
	{ "bq24150", BQ24150 },
	{ "bq24150a", BQ24150A },
	{ "bq24151", BQ24151 },
	{ "bq24151a", BQ24151A },
	{ "bq24152", BQ24152 },
	{ "bq24153", BQ24153 },
	{ "bq24153a", BQ24153A },
	{ "bq24155", BQ24155 },
	{ "bq24156", BQ24156 },
	{ "bq24156a", BQ24156A },
	{ "bq24157s", BQ24157S },
	{ "bq24158", BQ24158 },
	{ "aw32257", AW32257 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq2415x_i2c_id_table);

#ifdef CONFIG_ACPI
static const struct acpi_device_id bq2415x_i2c_acpi_match[] = {
	{ "BQ2415X", BQUNKNOWN },
	{ "BQ241500", BQ24150 },
	{ "BQA24150", BQ24150A },
	{ "BQ241510", BQ24151 },
	{ "BQA24151", BQ24151A },
	{ "BQ241520", BQ24152 },
	{ "BQ241530", BQ24153 },
	{ "BQA24153", BQ24153A },
	{ "BQ241550", BQ24155 },
	{ "BQ241560", BQ24156 },
	{ "BQA24156", BQ24156A },
	{ "BQS24157", BQ24157S },
	{ "BQ241580", BQ24158 },
	{},
};
MODULE_DEVICE_TABLE(acpi, bq2415x_i2c_acpi_match);
#endif

#ifdef CONFIG_OF
static const struct of_device_id bq2415x_of_match_table[] = {
	{ .compatible = "ti,bq24150" },
	{ .compatible = "ti,bq24150a" },
	{ .compatible = "ti,bq24151" },
	{ .compatible = "ti,bq24151a" },
	{ .compatible = "ti,bq24152" },
	{ .compatible = "ti,bq24153" },
	{ .compatible = "ti,bq24153a" },
	{ .compatible = "ti,bq24155" },
	{ .compatible = "ti,bq24156" },
	{ .compatible = "ti,bq24156a" },
	{ .compatible = "ti,bq24157s" },
	{ .compatible = "ti,bq24158" },
	{ .compatible = "awinic,aw32257" },
	{},
};
MODULE_DEVICE_TABLE(of, bq2415x_of_match_table);
#endif

static struct i2c_driver bq2415x_driver = {
	.driver = {
		.name = "bq2415x-charger",
		.of_match_table = of_match_ptr(bq2415x_of_match_table),
		.acpi_match_table = ACPI_PTR(bq2415x_i2c_acpi_match),
	},
	.probe = bq2415x_probe,
	.remove = bq2415x_remove,
	.id_table = bq2415x_i2c_id_table,
};
module_i2c_driver(bq2415x_driver);

MODULE_AUTHOR("Pali Rohár <pali@kernel.org>");
MODULE_DESCRIPTION("bq2415x charger driver");
MODULE_LICENSE("GPL");
