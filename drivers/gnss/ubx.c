// SPDX-License-Identifier: GPL-2.0
/*
 * u-blox GNSS receiver driver
 *
 * Copyright (C) 2018 Johan Hovold <johan@kernel.org>
 */

#include <linux/errno.h>
#include <linux/gnss.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kstrtox.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/serdev.h>

#include "serial.h"

/* Total configuration message length = PREAMBLE_LEN + MESSAGE_CLASS_LEN +
 *   MESSAGE_LENGTH_LEN + payload length + CHECKSUM_LEN
 */
const int32_t PREAMBLE_LEN = 2;
const int32_t MESSAGE_CLASS_LEN = 2;
const int32_t MESSAGE_LENGTH_LEN = 2;
const int32_t CHECKSUM_LEN = 2;
const size_t FIRST_CONFIG_REGISTER_BYTE = 10U;
const size_t FIRST_VALUE_BYTE = 14U;
const size_t FIRST_CHECKSUM_BYTE = 18U;
const size_t CFG_MSG_TOTAL_LEN = 20U;

uint8_t ZED_F9_CFG_VALSET_MSG[] = {
	0xB5, 0x62, /* 0-1 preamble */
	0x06, 0x8A, /* 2-3 CFG_VALSET command */
	0x0C, 0x00, /* 4-5 payload length = 12 for one key-value pair */
	0x00, /* 6 U-Blox API version */
	0x01, /* 7 Write to RAM */
	0x00, 0x00, /* 8-9 Reserved */
	0x00, 0x00, 0x00, 0x00, /* 10-13 Placeholder for configuration register */
	0x00, 0x00, 0x00, 0x00, /* 14-17 Placeholder for baud value */
	0x00, 0x00 /* 18-19 Placeholder for checksum */
};

struct ubx_features {
	int (*open)(struct gnss_device *gdev);
	size_t baud_config_reg;
	u32 min_baud;
	u32 default_baud;
	u32 max_baud;
};

struct ubx_data {
	struct regulator *v_bckp;
	struct regulator *vcc;
	const struct ubx_features *features;
	unsigned long is_configured;
};

union message_length {
	uint16_t ml;
	uint8_t bytes[2];
};

union int_to_bytes {
	uint32_t int_val;
	uint8_t bytes[4];
};

/* Payload  length is contained in bytes 0-2 after message class and ID.
 *  While the checksum includes the Message class and ID plus message length, the
 *  payload does not.
 */
static uint16_t get_payload_length(const uint8_t msg[])
{
	union message_length hs_msg_len;

	hs_msg_len.bytes[0] = msg[PREAMBLE_LEN + MESSAGE_CLASS_LEN];
	hs_msg_len.bytes[1] = msg[PREAMBLE_LEN + MESSAGE_CLASS_LEN + 1U];
	return hs_msg_len.ml;
}

static int32_t get_msg_total_len(const uint8_t msg[])
{
	const size_t payload_len = get_payload_length(msg);

	return PREAMBLE_LEN + MESSAGE_CLASS_LEN + MESSAGE_LENGTH_LEN + payload_len
		+ CHECKSUM_LEN;
}

/* The checksum is calculated on message class, message ID, message length and
 * payload.
 */
static void calc_ubx_checksum(const uint8_t msg[], uint8_t checksum[],
			   const uint16_t total_len)
{
	uint8_t CK_A = 0;
	uint8_t CK_B = 0;
	int i;

	for (i = PREAMBLE_LEN; i < (total_len - CHECKSUM_LEN); i++) {
		CK_A += msg[i];
		CK_B += CK_A;
	}
	checksum[0] = CK_A;
	checksum[1] = CK_B;
}

static uint32_t  check_baud(speed_t speed, const struct device *dev,
					const struct ubx_features *features)
{
	if ((speed < features->min_baud) || (speed > features->max_baud)) {
		dev_warn(dev, "Baud rate specification %d out of range\n", speed);
		speed = features->default_baud;
	}
	return speed;
}

static int prepare_zedf9_config_msg(const speed_t speed,
					const struct device *dev,
					    const struct ubx_features *features)
{
	union int_to_bytes cfg_val, cfg_register;
	int i = 0;
	uint8_t checksum[2];
	const size_t total_len = get_msg_total_len(ZED_F9_CFG_VALSET_MSG);

	if (total_len != CFG_MSG_TOTAL_LEN)
		goto bad_msg;

	cfg_val.int_val = check_baud(speed, dev, features);
	cfg_register.int_val = features->baud_config_reg;
	for (i = 0; i < 4; i++) {
		ZED_F9_CFG_VALSET_MSG[FIRST_VALUE_BYTE + i] = cfg_val.bytes[i];
		ZED_F9_CFG_VALSET_MSG[FIRST_CONFIG_REGISTER_BYTE + i] = cfg_register.bytes[i];
	}
	calc_ubx_checksum(ZED_F9_CFG_VALSET_MSG, checksum, total_len);
	ZED_F9_CFG_VALSET_MSG[FIRST_CHECKSUM_BYTE] = checksum[0];
	ZED_F9_CFG_VALSET_MSG[FIRST_CHECKSUM_BYTE + 1U] = checksum[1];
	return 0;

 bad_msg:
	dev_err(dev, "Malformed UBX-CFG-VALSET message\n");
	return -EINVAL;
}

/* Configure the Zed F9 baud rate via the UBX-CFG-VALSET message. */
static int set_zedf9_baud(struct gnss_device *gdev,
					struct serdev_device *serdev, struct gnss_serial *gserial)
{
	const struct ubx_data *data = gnss_serial_get_drvdata(gserial);
	const struct ubx_features *features = data->features;
	size_t count = 0U;
	int ret;

	if (!data->features)
		return -EINVAL;
	if (gserial->speed == features->default_baud)
		return 0;

	ret = prepare_zedf9_config_msg(gserial->speed, &gdev->dev, features);
	if (ret)
		return ret;
	/* Initially set the UART to the default speed to match the GNSS' power-on value. */
	serdev_device_set_baudrate(serdev, features->default_baud);
	/* Now set the new baud rate. */
	count = gdev->ops->write_raw(gdev, ZED_F9_CFG_VALSET_MSG, CFG_MSG_TOTAL_LEN);
	if (count != CFG_MSG_TOTAL_LEN)
		return count;

	return 0;
}

static int zed_f9_serial_open(struct gnss_device *gdev)
{
	struct gnss_serial *gserial = gnss_get_drvdata(gdev);
	struct serdev_device *serdev = gserial->serdev;
	struct ubx_data *data = gnss_serial_get_drvdata(gserial);
	int ret;

	ret = serdev_device_open(serdev);
	if (ret)
		return ret;
	if (!data->features)
		return -EINVAL;

	serdev_device_set_flow_control(serdev, false);

	if (!data->is_configured) {
		/* 4800 is the default value set by gnss_serial_parse_dt() */
		if (gserial->speed == 4800) {
			/* Fall back instead to Zed F9 default */
			gserial->speed = data->features->default_baud;
		} else {
			ret = set_zedf9_baud(gdev, serdev, gserial);
			if (ret)
				return ret;
		}
		data->is_configured = 1;
	}
	serdev_device_set_baudrate(serdev, gserial->speed);

	ret = pm_runtime_get_sync(&serdev->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(&serdev->dev);
		goto err_close;
	}
	return 0;

err_close:
	serdev_device_close(serdev);

	return ret;
}

static int ubx_set_active(struct gnss_serial *gserial)
{
	struct ubx_data *data = gnss_serial_get_drvdata(gserial);
	int ret;

	ret = regulator_enable(data->vcc);
	if (ret)
		return ret;

	return 0;
}

static int ubx_set_standby(struct gnss_serial *gserial)
{
	struct ubx_data *data = gnss_serial_get_drvdata(gserial);
	int ret;

	ret = regulator_disable(data->vcc);
	if (ret)
		return ret;

	return 0;
}

static int ubx_set_power(struct gnss_serial *gserial,
				enum gnss_serial_pm_state state)
{
	switch (state) {
	case GNSS_SERIAL_ACTIVE:
		return ubx_set_active(gserial);
	case GNSS_SERIAL_OFF:
	case GNSS_SERIAL_STANDBY:
		return ubx_set_standby(gserial);
	}

	return -EINVAL;
}

static const struct gnss_serial_ops ubx_gserial_ops = {
	.set_power = ubx_set_power,
};


static const struct ubx_features __maybe_unused zedf9_feats = {
	.open			=	zed_f9_serial_open,
	.baud_config_reg	=	0x40520001,
	.min_baud		=	9600,
	.default_baud		=	38400,
	.max_baud		=	921600,
};

#ifdef CONFIG_OF
static const struct of_device_id ubx_of_match[] = {
	{ .compatible = "u-blox,neo-6m" },
	{ .compatible = "u-blox,neo-8" },
	{ .compatible = "u-blox,neo-m8" },
	{ .compatible = "u-blox,zed-f9", .data = &zedf9_feats },
	{},
};
MODULE_DEVICE_TABLE(of, ubx_of_match);
#endif

static int ubx_probe(struct serdev_device *serdev)
{
	struct gnss_serial *gserial;
	struct gpio_desc *reset;
	struct ubx_data *data;
	struct gnss_operations *ubx_gnss_ops;
	int ret;

	gserial = gnss_serial_allocate(serdev, sizeof(*data));
	if (IS_ERR(gserial)) {
		ret = PTR_ERR(gserial);
		return ret;
	}
	ubx_gnss_ops = kzalloc(sizeof(struct gnss_operations), GFP_KERNEL);
	if (IS_ERR(ubx_gnss_ops)) {
		ret = PTR_ERR(ubx_gnss_ops);
		return ret;
	}

	gserial->ops = &ubx_gserial_ops;

	gserial->gdev->type = GNSS_TYPE_UBX;

	data = gnss_serial_get_drvdata(gserial);
#if IS_ENABLED(CONFIG_OF)
	{
		data->is_configured = 0;
		data->features = of_match_device(ubx_of_match, &serdev->dev)->data;
		if (data->features && data->features->open) {
			ubx_gnss_ops->open  = data->features->open;
			ubx_gnss_ops->close = gserial->gdev->ops->close;
			ubx_gnss_ops->write_raw = gserial->gdev->ops->write_raw;
			gserial->gdev->ops = ubx_gnss_ops;
		}
	}
#endif
	data->vcc = devm_regulator_get(&serdev->dev, "vcc");
	if (IS_ERR(data->vcc)) {
		ret = PTR_ERR(data->vcc);
		goto err_free_gserial;
	}

	ret = devm_regulator_get_enable_optional(&serdev->dev, "v-bckp");
	if (ret < 0 && ret != -ENODEV)
		goto err_free_gserial;

	/* Deassert reset */
	reset = devm_gpiod_get_optional(&serdev->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(reset)) {
		ret = PTR_ERR(reset);
		goto err_free_gserial;
	}

	ret = gnss_serial_register(gserial);
	if (ret)
		goto err_free_gserial;

	return 0;

err_free_gserial:
	gnss_serial_free(gserial);

	return ret;
}

static void ubx_remove(struct serdev_device *serdev)
{
	struct gnss_serial *gserial = serdev_device_get_drvdata(serdev);

	gnss_serial_deregister(gserial);
	gnss_serial_free(gserial);
}

static struct serdev_device_driver ubx_driver = {
	.driver	= {
		.name		= "gnss-ubx",
		.of_match_table	= of_match_ptr(ubx_of_match),
		.pm		= &gnss_serial_pm_ops,
	},
	.probe	= ubx_probe,
	.remove	= ubx_remove,
};
module_serdev_device_driver(ubx_driver);

MODULE_AUTHOR("Johan Hovold <johan@kernel.org>");
MODULE_DESCRIPTION("u-blox GNSS receiver driver");
MODULE_LICENSE("GPL v2");
