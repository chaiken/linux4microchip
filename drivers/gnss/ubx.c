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

#include "core.h"
#include "serial.h"

/* Total configuration message length = PREAMBLE_LEN + MESSAGE_CLASS_LEN +
 *   MESSAGE_LENGTH_LEN + payload length + CHECKSUM_LEN
 */
const size_t PREAMBLE_LEN = 2;
const size_t MESSAGE_CLASS_LEN = 2;
const size_t MESSAGE_LENGTH_LEN = 2;
const size_t CHECKSUM_LEN = 2;
const size_t FIRST_CONFIG_REGISTER_BYTE = 10U;
const size_t FIRST_VALUE_BYTE = 14U;
const size_t BAUD_FIRST_CHECKSUM_BYTE = 18U;
const size_t ANT_FIRST_CHECKSUM_BYTE = 30U;
const size_t PROTOCOL_FIRST_CHECKSUM_BYTE = 45U;
const size_t BAUD_MSG_TOTAL_LEN = 20U;
const size_t NUM_ANT_COMMANDS = 4U;
const size_t NUM_PROTOCOL_COMMANDS = 7U;
const size_t ANT_MSG_TOTAL_LEN = 32U;
const size_t PROTOCOL_MSG_TOTAL_LEN = 47U;

enum gnss_output_protocol {
	PROTOCOL_NONE,
	UBX,
	NMEA,
};

uint8_t ZED_F9_BAUD_MSG[] = {
	0xB5, 0x62, /* 0-1 preamble */
	0x06, 0x8A, /* 2-3 CFG_VALSET command */
	0x0C, 0x00, /* 4-5 payload length = 12 for one key + int-value */
	0x00, /* 6 U-Blox API version */
	0x01, /* 7 Write to RAM */
	0x00, 0x00, /* 8-9 Reserved */
	0x00, 0x00, 0x00, 0x00, /* 10-13 Placeholder for configuration register = key */
	0x00, 0x00, 0x00, 0x00, /* 14-17 Placeholder for baud value */
	0x00, 0x00 /* 18-19 Placeholder for checksum */
};

uint8_t ZED_F9_ANTENNA_MSG[] = {
	0xB5, 0x62, /* 0-1 preamble */
	0x06, 0x8A, /* 2-3 CFG_VALSET command */
	0x18, 0x00, /* 4-5 payload length = 4 + (4 settings * (4B key + 1B value)) */
	0x00, /* 6 U-Blox API version */
	0x01, /* 7 Write to RAM */
	0x00, 0x00, /* 8-9 Reserved */
	0x00, 0x00, 0x00, 0x00, /* 10-13 Placeholder for configuration register = key */
	0x00, /* 14 Placeholder for boolean value */
	0x00, 0x00, 0x00, 0x00, /* 15-18 Placeholder for configuration register = key */
	0x00, /* 19 Placeholder for boolean value */
	0x00, 0x00, 0x00, 0x00, /* 20-23 Placeholder for configuration register = key */
	0x00, /* 24 Placeholder for boolean value */
	0x00, 0x00, 0x00, 0x00, /* 25-28 Placeholder for configuration register = key */
	0x00, /* 29 Placeholder for boolean value */
	0x00, 0x00 /* 30-31 Placeholder for checksum */
};

/*
 * Disable the NMEA output protocol (default).
 * Enable UBX output protocol.
 * Request these messages be sent each navigation epoch:
 *    UBX-NAV_PVT for receiver-generated fixes and associated metadata.
 *    UBX-NAV-TIMEGPS to get the correspondence between local and GPS time.
 *    UBX-NAV-EOE to completion of navigation epoch messages.
 *    UBX-RXM_RAWX to get raw measurements from each satellite.
 *    UBX_RXM_SFRBX to get raw satellite broadcast orbit data.
 */
uint8_t ZED_F9_PROTOCOL_MSG[] = {
	0xB5, 0x62, /* 0-1 preamble */
	0x06, 0x8A, /* 2-3 CFG_VALSET command */
	0x27, 0x00, /* 4-5 payload length = 4 + 7 * (4B key + 1B value) */
	0x00, /* 6 U-Blox API version */
	0x01, /* 7 Write to RAM */
	0x00, 0x00, /* 8-9 Reserved */

	0x00, 0x00, 0x00, 0x00, /* 10-13 Placeholder for configuration register = key */
	0x00, /* 14 Placeholder for boolean value */
	0x00, 0x00, 0x00, 0x00, /* 15-18 Placeholder for configuration register = key */
	0x00, /* 19 Placeholder for boolean value */
	0x00, 0x00, 0x00, 0x00, /* 20-23 Placeholder for configuration register = key */
	0x00, /* 24 Placeholder for boolean value */
	0x00, 0x00, 0x00, 0x00, /* 25-28 Placeholder for configuration register = key */
	0x00, /* 29 Placeholder for boolean value */
	0x00, 0x00, 0x00, 0x00, /* 30-33 Placeholder for configuration register = key */
	0x00, /* 34 Placeholder for boolean value */
	0x00, 0x00, 0x00, 0x00, /* 35-38 Placeholder for configuration register = key */
	0x00, /* 39 Placeholder for boolean value */
	0x00, 0x00, 0x00, 0x00, /* 40-43 Placeholder for configuration register = key */
	0x00, /* 44 Placeholder for boolean value */

	0x00, 0x00 /* 45-46 Placeholder for checksum */
};

struct ubx_features {
	int (*open)(struct gnss_device *gdev);
	size_t antenna_regs[4U];  /* Size must be kept in sync with NUM_ANT_COMMANDS */
	size_t baud_config_reg;
	size_t protocol_regs[7U];   /* Size must be kept in sync with NUM_PROTOCOL_COMMANDS */
	u32 min_baud;
	u32 default_baud;
	u32 max_baud;
	enum gnss_output_protocol default_protocol;
};

struct ubx_data {
	struct regulator *v_bckp;
	struct regulator *vcc;
	const struct ubx_features *features;
	unsigned long is_configured;
	enum gnss_output_protocol selected_protocol;
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

static int prepare_zedf9_gnss_protocol_msg(const enum gnss_output_protocol protocol,
					const struct device *dev,
					    const struct ubx_features *features)
{
	union int_to_bytes ubx_register, nmea_register, cfg_register;
	int i = 0, j = 0, offset = 0;
	uint8_t checksum[2];
	const bool protocol_is_ubx = (UBX == protocol);
	const size_t total_len = get_msg_total_len(ZED_F9_PROTOCOL_MSG);
	const size_t setting_len = sizeof(int) + sizeof(bool);

	if (IS_ERR_OR_NULL(features)) {
		dev_err(dev, "Invalid GNSS driver data.\n");
		return -EINVAL;
	}
	if (total_len != PROTOCOL_MSG_TOTAL_LEN) {
		dev_err(dev, "Malformed UBX protocol configuration message\n");
		return -EINVAL;
	}

	/* Support either UBX or NMEA output, so disable one when enabling the other. */
	if ((UBX != protocol) && (NMEA != protocol)) {
		dev_err(dev, "Illegal output protocol.");
		return -EINVAL;
	}
	if (protocol_is_ubx){
		dev_info(dev, "Selecting UBX output protocol.\n");
	} else {
		dev_info(dev, "Selecting NMEA output protocol.\n");
	}

	/* Enable one output protocol and disable the other. */
	ZED_F9_PROTOCOL_MSG[FIRST_VALUE_BYTE] = protocol_is_ubx;
	ZED_F9_PROTOCOL_MSG[FIRST_VALUE_BYTE + setting_len] = !protocol_is_ubx;
	ubx_register.int_val = features->protocol_regs[0];
	nmea_register.int_val = features->protocol_regs[1];
	for (i = 0; i < sizeof(int); i++) {
		ZED_F9_PROTOCOL_MSG[FIRST_CONFIG_REGISTER_BYTE + i]
			= ubx_register.bytes[i];
		ZED_F9_PROTOCOL_MSG[FIRST_CONFIG_REGISTER_BYTE + i + setting_len]
			= nmea_register.bytes[i];
	}

	/* Enable messages to be sent each epoch. */
	for (i = 2; i < (int) NUM_PROTOCOL_COMMANDS; i++) {
		/* 5U = 4 bytes for the key (register) plus 1 byte for the boolean value */
		offset = i * 5U;
		ZED_F9_PROTOCOL_MSG[FIRST_VALUE_BYTE + offset] = 1;
		cfg_register.int_val = features->protocol_regs[i];
		for (j = 0; j < sizeof(int); j++) {
			ZED_F9_PROTOCOL_MSG[FIRST_CONFIG_REGISTER_BYTE + offset + j]
				= cfg_register.bytes[j];
		}
	}

	calc_ubx_checksum(ZED_F9_PROTOCOL_MSG, checksum, total_len);
	ZED_F9_PROTOCOL_MSG[PROTOCOL_FIRST_CHECKSUM_BYTE] = checksum[0];
	ZED_F9_PROTOCOL_MSG[PROTOCOL_FIRST_CHECKSUM_BYTE + 1U] = checksum[1];
	return 0;
}

static int prepare_zedf9_antenna_msg(const bool state,
					const struct device *dev,
					    const struct ubx_features *features)
{
	union int_to_bytes cfg_register;
	int i = 0, j = 0, offset = 0;
	uint8_t checksum[2];
	const size_t total_len = get_msg_total_len(ZED_F9_ANTENNA_MSG);

	if (total_len != ANT_MSG_TOTAL_LEN)
		goto bad_msg;

	if (1 == state){
		dev_info(dev, "Enabling antenna controls.\n");
	}
	if (0 == state){
		dev_info(dev, "Disabling antenna controls.\n");
	}
	for (i = 0; i < (int) NUM_ANT_COMMANDS; i++) {
		/* 5U = 4 bytes for the key (register) plus 1 byte for the boolean value */
		offset = i * 5U;
		ZED_F9_ANTENNA_MSG[FIRST_VALUE_BYTE + offset] = state;
		cfg_register.int_val = features->antenna_regs[i];
		for (j = 0; j < sizeof(int); j++) {
			ZED_F9_ANTENNA_MSG[FIRST_CONFIG_REGISTER_BYTE + offset + j]
				= cfg_register.bytes[j];
		}
	}
	calc_ubx_checksum(ZED_F9_ANTENNA_MSG, checksum, total_len);
	ZED_F9_ANTENNA_MSG[ANT_FIRST_CHECKSUM_BYTE] = checksum[0];
	ZED_F9_ANTENNA_MSG[ANT_FIRST_CHECKSUM_BYTE + 1U] = checksum[1];
	return 0;

 bad_msg:
	dev_err(dev, "Malformed UBX antenna-control message\n");
	return -EINVAL;
}

static int prepare_zedf9_baud_msg(const speed_t speed,
					const struct device *dev,
					    const struct ubx_features *features)
{
	union int_to_bytes cfg_val, cfg_register;
	int i = 0;
	uint8_t checksum[2];
	const size_t total_len = get_msg_total_len(ZED_F9_BAUD_MSG);

	if (total_len != BAUD_MSG_TOTAL_LEN)
		goto bad_msg;

	cfg_val.int_val = check_baud(speed, dev, features);
	cfg_register.int_val = features->baud_config_reg;
	for (i = 0; i < 4; i++) {
		ZED_F9_BAUD_MSG[FIRST_VALUE_BYTE + i] = cfg_val.bytes[i];
		ZED_F9_BAUD_MSG[FIRST_CONFIG_REGISTER_BYTE + i] = cfg_register.bytes[i];
	}
	calc_ubx_checksum(ZED_F9_BAUD_MSG, checksum, total_len);
	ZED_F9_BAUD_MSG[BAUD_FIRST_CHECKSUM_BYTE] = checksum[0];
	ZED_F9_BAUD_MSG[BAUD_FIRST_CHECKSUM_BYTE + 1U] = checksum[1];
	return 0;

 bad_msg:
	dev_err(dev, "Malformed UBX baud-setting message\n");
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
	ret = prepare_zedf9_baud_msg(gserial->speed, &gdev->dev, features);
	if (ret)
		return ret;
	/* Now set the new baud rate. */
	count = gdev->ops->write_raw(gdev, ZED_F9_BAUD_MSG, BAUD_MSG_TOTAL_LEN);
	if (count != BAUD_MSG_TOTAL_LEN) {
		dev_err(&gdev->dev, "Baud-rate setting failed.");
		return count;
	}
	return 0;
}

/* Enable the Zed F9 antenna voltage control  rate via the UBX-CFG-VALSET message. */
static int enable_zedf9_antenna_control(struct gnss_device *gdev, struct gnss_serial *gserial)
{
	const struct ubx_data *data = gnss_serial_get_drvdata(gserial);
	const struct ubx_features *features = data->features;
	size_t count = 0U;
	int ret;

	if (!data->features)
		return -EINVAL;

	ret = prepare_zedf9_antenna_msg(true, &gdev->dev, features);
	if (ret)
		return ret;
	count = gdev->ops->write_raw(gdev, ZED_F9_ANTENNA_MSG, ANT_MSG_TOTAL_LEN);
	if (count != ANT_MSG_TOTAL_LEN) {
		dev_err(&gdev->dev, "Antenna-control enablement failed.");
		return count;
	}

	dev_info(&gdev->dev, "Enabled GNSS antenna controls.\n");
	return 0;
}

/* Set the default protocol defined in the driver data. */
static int set_zedf9_gnss_protocol(struct gnss_device *gdev, struct gnss_serial *gserial, const enum gnss_output_protocol protocol) {
	const struct ubx_data *data = gnss_serial_get_drvdata(gserial);
	const struct ubx_features *features = data->features;
	size_t count = 0U;

	int ret = prepare_zedf9_gnss_protocol_msg(protocol, &gdev->dev, features);
	if (ret)
		return ret;
	count = gdev->ops->write_raw(gdev, ZED_F9_PROTOCOL_MSG, PROTOCOL_MSG_TOTAL_LEN);
	if (count != PROTOCOL_MSG_TOTAL_LEN) {
		dev_err(&gdev->dev, "Failed to set GNSS protocol to %s.\n", (UBX == protocol) ? "UBX" : "NMEA");
		return EINVAL;
	}
	dev_info(&gdev->dev, "Set GNSS protocol to %s.\n", (UBX == protocol) ? "UBX" : "NMEA");

	return 0;
}

/*
 *  Opens serial device in order to ready it to receive commands.
 *  If the device is not yet configured, also set the serial device to the GNSS default baud.
 *  Closes device on error.
 */
static int zed_f9_serdev_maybe_set_default_baud(struct serdev_device *serdev,
		const speed_t default_baud, const bool is_configured) {
	speed_t new_baud = 0U;
	int ret = serdev_device_open(serdev);
	if (ret)
		return ret;

	serdev_device_set_flow_control(serdev, false);
	/* Don't reset to default baud if a higher baud is already set */
	if (!is_configured) {
		/* Initially set the UART to the default speed to match the GNSS' power-on value. */
		new_baud = serdev_device_set_baudrate(serdev, default_baud);
		if (default_baud != new_baud) {
		    dev_err(&serdev->dev, "Failed to set serial device to GNSS default baud %u\n",
			    default_baud);
		    serdev_device_close(serdev);
		    return -EINVAL;
		}
		dev_info(&serdev->dev, "Configured serial device default baud.\n");
	}
	return 0;
}

static ssize_t protocol_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count) {
	struct gnss_device *gdev = to_gnss_device(dev);
	struct gnss_serial *gserial;
	struct ubx_data *data;
	ssize_t ret = -EINVAL;

	if (!gdev) {
		dev_err(dev, "Invalid GNSS device.\n");
		return ret;
	}
	gserial = gnss_get_drvdata(gdev);
	if (!gserial) {
		dev_err(dev, "Invalid GNSS serial device.\n");
		return ret;
	}
	data = gnss_serial_get_drvdata(gserial);
	if (!data || !data->features) {
		dev_err(dev, "Lookup of driver data failed.\n");
		return ret;
	}

	/* Leaves serial device open on success, but closes it on failure. */
	ret = zed_f9_serdev_maybe_set_default_baud(gserial->serdev,
		data->features->default_baud, data->is_configured);
	if (ret)
		return ret;

	if (sysfs_streq(buf, "UBX")) {
		data->selected_protocol = UBX;
		ret = set_zedf9_gnss_protocol(gdev, gserial, UBX);
	} else if (sysfs_streq(buf, "NMEA")) {
		data->selected_protocol = NMEA;
		ret = set_zedf9_gnss_protocol(gdev, gserial, NMEA);
	} else {
		data->selected_protocol = PROTOCOL_NONE;
		dev_err(dev, "Valid GNSS protocol specification are 'NMEA' or 'UBX'.\n");
		ret = -EINVAL;
	}

	serdev_device_close(gserial->serdev);

	if (ret)
		return ret;
	return count;
}
static DEVICE_ATTR_WO(protocol);

static int zed_f9_serial_open(struct gnss_device *gdev)
{
	struct gnss_serial *gserial = gnss_get_drvdata(gdev);
	struct serdev_device *serdev = gserial->serdev;
	struct ubx_data *data = gnss_serial_get_drvdata(gserial);
	const struct ubx_features *features = data->features;
	speed_t new_baud = 0;
	int ret = -EINVAL;

	if (!features)
		goto err_close;

	/* If successful, opens the serial device */
	ret = zed_f9_serdev_maybe_set_default_baud(serdev, features->default_baud,
		data->is_configured);
	if (ret)
		return ret;
	if (!data->is_configured) {
		/* 4800 is the default value set by gnss_serial_parse_dt() */
		if (gserial->speed == 4800) {
			/* Fall back instead to Zed F9 default */
			gserial->speed = features->default_baud;
		} else {
			ret = set_zedf9_baud(gdev, serdev, gserial);
			if (ret) {
				dev_err(&gdev->dev, "GNSS speed setting to %u failed\n", gserial->speed);
				goto err_close;
			}
			new_baud = serdev_device_set_baudrate(serdev, gserial->speed);
			if (gserial->speed != new_baud) {
				dev_err(&gdev->dev, "Serial device speed setting to %u failed\n", gserial->speed);
				goto err_close;
			}
			dev_info(&gdev->dev, "Set GNSS speed to %u\n", gserial->speed);
		}

		ret = enable_zedf9_antenna_control(gdev, gserial);
		if (ret)
			goto err_close;
		/* Don't overwrite the protocol set by userspace, if any. */
		enum gnss_output_protocol protocol_to_set = (PROTOCOL_NONE == data->selected_protocol) ?
			features->default_protocol : data->selected_protocol;
		ret = set_zedf9_gnss_protocol(gdev, gserial, protocol_to_set);
		if (ret)
			goto err_close;

		data->is_configured = 1;
	}
	/* Increase reference count of the serial device since it is open. */
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
	.open					=	zed_f9_serial_open,
	/* ANT_CFG_VOLTCTRL, ANT_CFG_SHORTDET, ANT_CFG_OPENDET, ANT_CFG_PWRDOWN */
	.antenna_regs				=	{0x10a3002e, 0x10a3002f, 0x10a30031, 0x10a30033},
	.baud_config_reg			=	0x40520001,
						/* CFG_UART1OUTPROT_UBX, CFG_UART1OUTPROT_NMEA, */
	.protocol_regs				=	{0x10740001, 0x10740002,
						/* CFG_MSGOUT_UBX_NAV_PVT_UART1, CFG_MSGOUT_UBX_NAV_TIMEGPS_UART1, */
						          0x20910007, 0x20910048,
						/* CFG_MSGOUT_UBX_NAV_EOE_UART1, CFG_MSGOUT_UBX_RXM_RAWX_UART1, */
							  0x20910160, 0x209102a5,
						/* CFG_MSGOUT_UBX_RXM_SFRBX_UART1*/
							  0x20910232},
	.min_baud				=	9600,
	.default_baud				=	38400,
	.max_baud				=	921600,
	.default_protocol     			=	UBX,
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
	struct gnss_device *gdev;
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

	gdev = gserial->gdev;
	gdev->type = GNSS_TYPE_UBX;

	data = gnss_serial_get_drvdata(gserial);
#if IS_ENABLED(CONFIG_OF)
	{
		data->is_configured = 0;
		data->selected_protocol = PROTOCOL_NONE;
		data->features = of_match_device(ubx_of_match, &serdev->dev)->data;
		if (data->features && data->features->open) {
			ubx_gnss_ops->open  = data->features->open;
			ubx_gnss_ops->close = gserial->gdev->ops->close;
			ubx_gnss_ops->write_raw = gserial->gdev->ops->write_raw;
			gdev->ops = ubx_gnss_ops;
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

#if IS_ENABLED(CONFIG_OF)
	{
		/* Create sysfs attribute for GNSS protocol setting. */
		ret = device_create_file(&gdev->dev, &dev_attr_protocol);
		if (ret) {
			dev_err(&gdev->dev, "Error creating GNSS protocol sysfs entry.\n");
			return ret;
		}
	}
#endif

	return 0;

err_free_gserial:
	gnss_serial_free(gserial);

	return ret;
}

/* TODO: free the sysfs GNSS protocol attribute if it exists? */
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
