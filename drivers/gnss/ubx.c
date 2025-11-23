// SPDX-License-Identifier: GPL-2.0
/*
 * u-blox GNSS receiver driver
 *
 * Copyright (C) 2018 Johan Hovold <johan@kernel.org>
 */

#include <linux/cleanup.h>
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

DEFINE_FREE(free_serial, struct gnss_serial *, if (_T) gnss_serial_free(_T))

/* Total configuration message length = INVARIANTS_LEN + payload length */
const size_t PREAMBLE_LEN = 2;
const size_t MESSAGE_CLASS_LEN = 2;
const size_t MESSAGE_LENGTH_LEN = 2;
const size_t CHECKSUM_LEN = 2;
/* API version, RAM-based configuration and reserved bytes */
const size_t RESERVED_LEN = 4;
const size_t INVARIANTS_LEN = PREAMBLE_LEN + MESSAGE_CLASS_LEN + MESSAGE_LENGTH_LEN + CHECKSUM_LEN + RESERVED_LEN;
const size_t FIRST_CONFIG_REGISTER_BYTE = INVARIANTS_LEN - CHECKSUM_LEN;
/* 4B is the register length. */
const size_t FIRST_VALUE_BYTE = FIRST_CONFIG_REGISTER_BYTE + 4;
/* 4B for register and 1B for the value */
const size_t SINGLE_BYTE_SETTING_LEN = 5;
/* 4B for register and 2B for the value */
const size_t SHORT_VAL_SETTING_LEN = 6;
/* 4B for register and 4B for the value */
const size_t INTEGER_VAL_SETTING_LEN = 8;

/*
 * 4 settings to turn choose between NMEA and UBX protocol for each of UART1 and
 * I2C, plus 7 message-output-enablement commands.
 */
const size_t NUM_PROTOCOL_ENABLE_COMMANDS = 10;
/* Disable 3 BeiDou-constellation configurations plus automotive dead-reckoning. */
const size_t NUM_PROTOCOL_DISABLE_COMMANDS = 5;
const size_t PROTOCOL_MSG_TOTAL_LEN = INVARIANTS_LEN + ((NUM_PROTOCOL_ENABLE_COMMANDS +
					NUM_PROTOCOL_DISABLE_COMMANDS) * SINGLE_BYTE_SETTING_LEN);
const size_t NUM_SEND_PERIOD_MSGS = 5;
const size_t SEND_PERIOD_MSG_TOTAL_LEN = INVARIANTS_LEN + (NUM_SEND_PERIOD_MSGS *
    SINGLE_BYTE_SETTING_LEN);
const size_t NUM_ANT_COMMANDS = 4;
const size_t ANT_MSG_TOTAL_LEN = INVARIANTS_LEN + (NUM_ANT_COMMANDS * SINGLE_BYTE_SETTING_LEN);
const size_t PPS_MSG_TOTAL_LEN = INVARIANTS_LEN + SINGLE_BYTE_SETTING_LEN;
const size_t MODEL_MSG_TOTAL_LEN = INVARIANTS_LEN + SINGLE_BYTE_SETTING_LEN;
const size_t BAUD_MSG_TOTAL_LEN = INVARIANTS_LEN + INTEGER_VAL_SETTING_LEN;
const size_t GENERATION_PERIOD_MSG_TOTAL_LEN = INVARIANTS_LEN + SHORT_VAL_SETTING_LEN;


enum gnss_output_protocol {
	PROTOCOL_NONE,
	UBX,
	NMEA,
};

enum gnss_timepulse_reference {
	UTC,
	GPS,
	GLO,
	BDS,
	GAL,
	NAVIC,
};

char *timepulse_reference_names[] = {
	"UTC",
	"GPS",
	"GLO",
	"BDS",
	"GAL",
	"NAVIC",
};

const size_t max_reference_name = ARRAY_SIZE(timepulse_reference_names) - 1U;

/*
 * enum value 1 and AIR3 are skipped.   See u-blox F9 LAP Interface description,
 *  p. 173.
 */
enum dynamic_platform_model {
	PORTABLE = 0,
	STATIONARY = 2,
	PEDESTRIAN,
	AUTOMOTIVE,
	SEA,
	AIR1,   // Airborne with <1g acceleration
	AIR2,   // Airborne with <2g acceleration
	AIR4,   // Airborne with <4g acceleration
	WRIST,  // Wrist worn watch.
};

const char * const dynamic_platform_model_names[] = {
	"PORTABLE",
	"STATIONARY",
	"PEDESTRIAN",
	"AUTOMOTIVE",
	"SEA",
	"AIR1",
	"AIR2",
	"AIR4",
	"WRIST",
};

/* The last index == ARRAY_SIZE, but value 1 is illegal. */
const size_t max_platform_model_name = ARRAY_SIZE(dynamic_platform_model_names);

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
 * First 3 settings:
 *    CFG-UART1OUTPROT-NMEA to disable the NMEA output protocol (default) on UART1.
 *    CFG-UART1OUTPROT-UBX to enable UBX output protocol on UART1.
 *    CFG-I2COUTPROT-UBX to enable the NMEA output protocol (default) on I2C.
 * Request these messages be sent once every second:
 *    UBX-RXM_RAWX to output raw measurements from each satellite to UART1.
 *    UBX-RXM-SFRBX to output raw satellite broadcast orbit data to UART1.
 *    UBX-MON-COMMS to output communication port statistics to UART1.
 *    UBX-TIM-TP to output time pulse data to UART1.
 *    UBX-TIM-TP to output time pulse data to I2C.
 *    UBX-MON-RF to output RF port status to UART1.
 *    UBX-MON-RF to output RF port status to I2C.
 * Disable these messages altogether:
 *    CFG-I2COUTPROT-NMEA to disable the NMEA output protocol (default) on I2C.
 *    CFG-SIGNAL-BDS_ENA to turn off BeiDou constellation.
 *    CFG-SIGNAL-BDS_B1_ENA to turn off another BeiDou constellation.
 *    CFG-SIGNAL-BDS_B2_ENA to turn off yet another BeiDou constellation.
 *    CFG-SFCORE-USE_SF to turn off automotive dead reckoning.
 */
uint8_t ZED_F9_PROTOCOL_MSG[] = {
	0xB5, 0x62, /* 0-1 preamble */
	0x06, 0x8A, /* 2-3 CFG_VALSET command */
	0x4F, 0x00, /* 4-5 payload length = 4 + 15 * (4B key + 1B value) */
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
	0x00, 0x00, 0x00, 0x00, /* 45-48 Placeholder for configuration register = key */
	0x00, /* 49 Placeholder for boolean value */
	0x00, 0x00, 0x00, 0x00, /* 50-53 Placeholder for configuration register = key */
	0x00, /* 54 Placeholder for boolean value */
	0x00, 0x00, 0x00, 0x00, /* 55-58 Placeholder for configuration register = key */
	0x00, /* 59 Placeholder for boolean value */
	0x00, 0x00, 0x00, 0x00, /* 60-63 Placeholder for configuration register = key */
	0x00, /* 64 Placeholder for boolean value */
	0x00, 0x00, 0x00, 0x00, /* 65-68 Placeholder for configuration register = key */
	0x00, /* 69 Placeholder for boolean value */
	0x00, 0x00, 0x00, 0x00, /* 70-73 Placeholder for configuration register = key */
	0x00, /* 74 Placeholder for boolean value */
	0x00, 0x00, 0x00, 0x00, /* 75-78 Placeholder for configuration register = key */
	0x00, /* 79 Placeholder for boolean value */
	0x00, 0x00, 0x00, 0x00, /* 80-83 Placeholder for configuration register = key */
	0x00, /* 84 Placeholder for boolean value */
	0x00, 0x00 /* 90-91 Placeholder for checksum */
};

/*
 * Configure the generation period of navigation messages.
 */
uint8_t ZED_F9_GENERATION_PERIOD_MSG[] = {
	0xB5, 0x62, /* 0-1 preamble */
	0x06, 0x8A, /* 2-3 CFG_VALSET command */
	0x0A, 0x00, /* 4-5 payload length = 10 = 4 + 6 (4B key + 2B value) */
	0x00, /* 6 U-Blox API version */
	0x01, /* 7 Write to RAM */
	0x00, 0x00, /* 8-9 Reserved */
	/* 10-13 Placeholder for configuration register = key */
	0x00, 0x00, 0x00, 0x00,
	/* 14-15 Placeholder for uint16_t measurement generation period value */
	0x00, 0x00,
	0x00, 0x00 /* 16-17 Placeholder for checksum */
};

/*
 * Configure the period governing the transmission of navigation messages to the
 * UART.
 *     UBX-NAV_PVT for receiver-generated fixes and associated metadata.
 *     UBX-NAV-TIMEGPS to get the correspondence between local and GPS time.
 *     UBX-NAV-EOE for completion of navigation epoch messages.
 *     UBX-NAV-CLOCK to get the clock solution.
 */
uint8_t ZED_F9_SEND_PERIOD_MSG[] = {
	0xB5, 0x62, /* 0-1 preamble */
	0x06, 0x8A, /* 2-3 CFG_VALSET command */
	0x18, 0x00, /* 4-5 payload length = 4 + 4 * (4B key + 1B value) */
	0x00, /* 6 U-Blox API version */
	0x01, /* 7 Write to RAM */
	0x00, 0x00, /* 8-9 Reserved */
	0x00, 0x00, 0x00, 0x00, /* 10-13 Placeholder for configuration register = key */
	0x00, /* 14 Placeholder for byte value */
	0x00, 0x00, 0x00, 0x00, /* 15-18 Placeholder for configuration register = key */
	0x00, /* 19 Placeholder for byte value */
	0x00, 0x00, 0x00, 0x00, /* 20-23 Placeholder for configuration register = key */
	0x00, /* 24 Placeholder for byte value */
	0x00, 0x00, 0x00, 0x00, /* 25-28 Placeholder for configuration register = key */
	0x00, /* 29 Placeholder for byte value */
	0x00, 0x00 /* 30-31 Placeholder for checksum */
};


/*
 * Select GPS reference for time pulse via CFG-TP-TIMEGRID_TP1 KeyID 0x2005000c.
 */
uint8_t ZED_F9_PPS_MSG[] = {
	0xB5, 0x62, /* 0-1 preamble */
	0x06, 0x8A, /* 2-3 CFG_VALSET command */
	0x09, 0x00, /* 4-5 payload length = 9 bytes = 4 + one key + enum value */
	0x00, /* 6 U-Blox API version */
	0x01, /* 7 Write to RAM */
	0x00, 0x00, /* 8-9 Reserved */
	0x00, 0x00, 0x00, 0x00, /* 10-13 Placeholder for CFG-TP-TIMEGRID_TP1 register */
	0x00, /* 14 Placeholder for time reference choice */
	0x00, 0x00 /* 15-16 Placeholder for checksum */
};

/*
 * Select dynamic model AIR4 via CFG-NAVSPG-DYNMODEL KeyID 0x20110021.
 */
uint8_t ZED_F9_MODEL_MSG[] = {
	0xB5, 0x62, /* 0-1 preamble */
	0x06, 0x8A, /* 2-3 CFG_VALSET command */
	0x09, 0x00, /* 4-5 payload length = 9 bytes = 4 + one key + enum value */
	0x00, /* 6 U-Blox API version */
	0x01, /* 7 Write to RAM */
	0x00, 0x00, /* 8-9 Reserved */
	0x00, 0x00, 0x00, 0x00, /* 10-13 Placeholder for register */
	0x00, /* 14 Placeholder for enum value */
	0x00, 0x00 /* 15-16 Placeholder for checksum */
};

struct ubx_features {
	int (*open)(struct gnss_device *gdev);
	size_t antenna_regs[4U];  /* Size must be kept in sync with NUM_ANT_COMMANDS */
	size_t baud_config_reg;
	/* Size must be kept in sync with NUM_PROTOCOL_ENABLE_COMMANDS +
	   NUM_PROTOCOL_DISABLE_COMMANDS */
	size_t protocol_regs[15U];
	size_t timepulse_reg;
	size_t generation_period_reg;
	size_t send_period_regs[4U];
	size_t dynamic_model_reg;
	u32 min_baud;
	u32 default_baud;
	u32 max_baud;
	u16 default_meas_period;
	u16 min_meas_period;
	enum gnss_output_protocol default_protocol;
	enum gnss_timepulse_reference default_time_reference;
	enum dynamic_platform_model default_dynamic_model;
};

struct ubx_data {
	struct regulator *v_bckp;
	struct regulator *vcc;
	const struct ubx_features *features;
	unsigned long is_configured;
	enum gnss_output_protocol selected_protocol;
};

union short_to_bytes {
	uint16_t short_val;
	uint8_t bytes[2];
};

union int_to_bytes {
	uint32_t int_val;
	uint8_t bytes[4];
};

static const char * gnss_output_protocol_name(const enum gnss_output_protocol protocol) {
	switch (protocol) {
	case PROTOCOL_NONE:
		return "None";
	case UBX:
		return "UBX";
	case NMEA:
		return "NMEA";
	default:
		return "Invalid";
	}
}

/* Payload  length is contained in bytes 0-2 after message class and ID.
 * While the checksum includes the Message class and ID plus message length, the
 * payload does not.
 */
static uint16_t get_payload_length(const uint8_t msg[])
{
	union short_to_bytes hs_msg_len;

	hs_msg_len.bytes[0] = msg[PREAMBLE_LEN + MESSAGE_CLASS_LEN];
	hs_msg_len.bytes[1] = msg[PREAMBLE_LEN + MESSAGE_CLASS_LEN + 1U];
	return hs_msg_len.short_val;
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

	/* Enable one output protocol and disable the other for UART1. */
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

	/*
	 * Enable messages which should be sent each epoch. Includes enabling UBX
	 * protocol on I2C.
	 */
	for (i = 2; i < (int) NUM_PROTOCOL_ENABLE_COMMANDS; i++) {
		offset = i * setting_len;
		/* 1 for enable */
		ZED_F9_PROTOCOL_MSG[FIRST_VALUE_BYTE + offset] = 1;
		cfg_register.int_val = features->protocol_regs[i];
		for (j = 0; j < sizeof(int); j++) {
			ZED_F9_PROTOCOL_MSG[FIRST_CONFIG_REGISTER_BYTE + offset + j]
				= cfg_register.bytes[j];
		}
	}
	/*
	 * Disable BeiDou satellite message processing and Automotive Dead Reckoning.
         * Also disable NMEA protocol on I2C.
	 * The registers corresponding to disabled settings must be at the end of the
	 * array.
	 */
	for (i = NUM_PROTOCOL_ENABLE_COMMANDS ;
	     i < (int) (NUM_PROTOCOL_ENABLE_COMMANDS + NUM_PROTOCOL_DISABLE_COMMANDS); i++) {
		offset = i * setting_len;
		/* 0 for disable */
		ZED_F9_PROTOCOL_MSG[FIRST_VALUE_BYTE + offset] = 0;
		cfg_register.int_val = features->protocol_regs[i];
		for (j = 0; j < sizeof(int); j++) {
			ZED_F9_PROTOCOL_MSG[FIRST_CONFIG_REGISTER_BYTE + offset + j]
				= cfg_register.bytes[j];
		}
	}
	calc_ubx_checksum(ZED_F9_PROTOCOL_MSG, checksum, total_len);
	ZED_F9_PROTOCOL_MSG[PROTOCOL_MSG_TOTAL_LEN - 2] = checksum[0];
	ZED_F9_PROTOCOL_MSG[PROTOCOL_MSG_TOTAL_LEN - 1] = checksum[1];
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
	ZED_F9_ANTENNA_MSG[ANT_MSG_TOTAL_LEN - 2] = checksum[0];
	ZED_F9_ANTENNA_MSG[ANT_MSG_TOTAL_LEN - 1] = checksum[1];
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
	for (i = 0; i < ARRAY_SIZE(cfg_val.bytes); i++) {
		ZED_F9_BAUD_MSG[FIRST_VALUE_BYTE + i] = cfg_val.bytes[i];
		ZED_F9_BAUD_MSG[FIRST_CONFIG_REGISTER_BYTE + i] = cfg_register.bytes[i];
	}
	calc_ubx_checksum(ZED_F9_BAUD_MSG, checksum, total_len);
	ZED_F9_BAUD_MSG[BAUD_MSG_TOTAL_LEN - 2] = checksum[0];
	ZED_F9_BAUD_MSG[BAUD_MSG_TOTAL_LEN - 1] = checksum[1];
	return 0;

 bad_msg:
	dev_err(dev, "Malformed UBX baud-setting message\n");
	return -EINVAL;
}

/*
 * Set the message-send period to a message every epoch.  The message-generation
 * period configured below affects the length of the epoch.
 */
static int prepare_zedf9_send_period_msg(const struct device *dev, const struct
				       ubx_features *features)
{
	union int_to_bytes cfg_register;
	int i = 0, j = 0, offset = 0;
	uint8_t checksum[2];
	const size_t total_len = get_msg_total_len(ZED_F9_SEND_PERIOD_MSG);
	const size_t setting_len = sizeof(int) + sizeof(bool);

	if (IS_ERR_OR_NULL(features)) {
		dev_err(dev, "Invalid GNSS driver data.\n");
		return -EINVAL;
	}
	if (total_len != SEND_PERIOD_MSG_TOTAL_LEN) {
		dev_err(dev, "Malformed UBX message-send-rate configuration message\n");
		return -EINVAL;
	}
	/*
	 * Although the documentation calls the values "rates", making them larger than
	 * 1 slows transmission.
	 */
	for (i = 0; i < (int) NUM_SEND_PERIOD_MSGS; i++) {
		offset = i * setting_len;
		/* 1 means once per epoch.  2 means send alternate epochs. */
		ZED_F9_SEND_PERIOD_MSG[FIRST_VALUE_BYTE + offset] = 1;
		cfg_register.int_val = features->send_period_regs[i];
		for (j = 0; j < sizeof(int); j++) {
			ZED_F9_SEND_PERIOD_MSG[FIRST_CONFIG_REGISTER_BYTE + offset + j]
				= cfg_register.bytes[j];
		}
	}
	calc_ubx_checksum(ZED_F9_SEND_PERIOD_MSG, checksum, total_len);
	ZED_F9_SEND_PERIOD_MSG[SEND_PERIOD_MSG_TOTAL_LEN - 2] = checksum[0];
	ZED_F9_SEND_PERIOD_MSG[SEND_PERIOD_MSG_TOTAL_LEN - 1] = checksum[1];
	return 0;
}

/* Set the message-generation period in ms. */
static int prepare_zedf9_generation_period_msg(const uint32_t period,
					const struct device *dev,
					    const struct ubx_features *features)
{
	union short_to_bytes cfg_val;
	union int_to_bytes cfg_register;
	int i = 0;
	uint8_t checksum[2];
	const size_t total_len = get_msg_total_len(ZED_F9_GENERATION_PERIOD_MSG);

	if (IS_ERR_OR_NULL(features)) {
		dev_err(dev, "Invalid GNSS driver data.\n");
		return -EINVAL;
	}
	if (total_len != GENERATION_PERIOD_MSG_TOTAL_LEN) {
	    dev_err(dev, "Malformed solution generation message\n");
	return -EINVAL;
	}

	cfg_val.short_val = (period >= features->min_meas_period) ? period :
	    features->min_meas_period;
	ZED_F9_GENERATION_PERIOD_MSG[FIRST_VALUE_BYTE] = cfg_val.bytes[0];
	ZED_F9_GENERATION_PERIOD_MSG[FIRST_VALUE_BYTE + 1] = cfg_val.bytes[1];
	cfg_register.int_val = features->generation_period_reg;
	for (i = 0; i < ARRAY_SIZE(cfg_register.bytes); i++) {
		ZED_F9_GENERATION_PERIOD_MSG[FIRST_CONFIG_REGISTER_BYTE + i] = cfg_register.bytes[i];
	}
	calc_ubx_checksum(ZED_F9_GENERATION_PERIOD_MSG, checksum, total_len);
	ZED_F9_GENERATION_PERIOD_MSG[GENERATION_PERIOD_MSG_TOTAL_LEN - 2] = checksum[0];
	ZED_F9_GENERATION_PERIOD_MSG[GENERATION_PERIOD_MSG_TOTAL_LEN - 1] = checksum[1];
	return 0;
}

static int prepare_zedf9_time_pulse_msg(const struct device *dev,
		const struct ubx_features *features)
{
	union int_to_bytes cfg_register;
	int i = 0;
	uint8_t checksum[2];
	const size_t total_len = get_msg_total_len(ZED_F9_PPS_MSG);

	if (total_len != PPS_MSG_TOTAL_LEN)
		goto bad_msg;

	cfg_register.int_val = features->timepulse_reg;
	for (i = 0; i < ARRAY_SIZE(cfg_register.bytes); i++) {
		ZED_F9_PPS_MSG[FIRST_CONFIG_REGISTER_BYTE + i] = cfg_register.bytes[i];
	}
	ZED_F9_PPS_MSG[FIRST_VALUE_BYTE] = features->default_time_reference;
	calc_ubx_checksum(ZED_F9_PPS_MSG, checksum, total_len);
	ZED_F9_PPS_MSG[PPS_MSG_TOTAL_LEN - 2] = checksum[0];
	ZED_F9_PPS_MSG[PPS_MSG_TOTAL_LEN - 1] = checksum[1];
	return 0;

 bad_msg:
	dev_err(dev, "Malformed UBX timepulse-reference message\n");
	return -EINVAL;
}

static int prepare_zedf9_dynamic_model_msg(const struct device *dev,
		const struct ubx_features *features)
{
	union int_to_bytes cfg_register;
	int i = 0;
	uint8_t checksum[2];
	const size_t total_len = get_msg_total_len(ZED_F9_MODEL_MSG);

	if (total_len != MODEL_MSG_TOTAL_LEN) {
		dev_err(dev, "Malformed UBX dynamic-model message\n");
		return -EINVAL;
	}

	cfg_register.int_val = features->dynamic_model_reg;
	for (i = 0; i < ARRAY_SIZE(cfg_register.bytes); i++) {
		ZED_F9_MODEL_MSG[FIRST_CONFIG_REGISTER_BYTE + i] = cfg_register.bytes[i];
	}
	ZED_F9_MODEL_MSG[FIRST_VALUE_BYTE] = features->default_dynamic_model;
	calc_ubx_checksum(ZED_F9_MODEL_MSG, checksum, total_len);
	ZED_F9_MODEL_MSG[MODEL_MSG_TOTAL_LEN - 2] = checksum[0];
	ZED_F9_MODEL_MSG[MODEL_MSG_TOTAL_LEN - 1] = checksum[1];
	return 0;
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

/* Configure the rate at which navigation messages are sent. */
static int set_zedf9_send_period(struct gnss_device *gdev, struct gnss_serial *gserial)
{
	const struct ubx_data *data = gnss_serial_get_drvdata(gserial);
	const struct ubx_features *features = data->features;
	size_t count = 0U;
	int ret;

	if (!data->features)
		return -EINVAL;
	ret = prepare_zedf9_send_period_msg(&gdev->dev, features);
	if (ret)
		return ret;
	/* Now set the new measurement rate. */
	count = gdev->ops->write_raw(gdev, ZED_F9_SEND_PERIOD_MSG,
				     SEND_PERIOD_MSG_TOTAL_LEN);
	if (count != SEND_PERIOD_MSG_TOTAL_LEN) {
		dev_err(&gdev->dev, "Navigation-message-send-rate setting failed.");
		return count;
	}
	dev_info(&gdev->dev,
		 "Set GNSS navigation-message-send rate to 1 message every epoch.\n");
	return 0;
}

/* Configure the rate at which navigation messages are generated. */
static int set_zedf9_generation_period(struct gnss_device *gdev, struct gnss_serial *gserial)
{
	const struct ubx_data *data = gnss_serial_get_drvdata(gserial);
	const struct ubx_features *features = data->features;
	size_t count = 0U;
	int ret;

	if (!data->features)
		return -EINVAL;
	ret = prepare_zedf9_generation_period_msg(features->default_meas_period, &gdev->dev, features);
	if (ret)
		return ret;
	/* Now set the new measurement rate. */
	count = gdev->ops->write_raw(gdev, ZED_F9_GENERATION_PERIOD_MSG,
				     GENERATION_PERIOD_MSG_TOTAL_LEN);
	if (count != GENERATION_PERIOD_MSG_TOTAL_LEN) {
		dev_err(&gdev->dev, "Navigation-message-generation-period setting failed.");
		return count;
	}
	dev_info(&gdev->dev, "Set GNSS navigation-message-generation period to %u ms.\n",
		 features->default_meas_period);
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
static int set_zedf9_gnss_protocol(struct gnss_device *gdev,
				   struct gnss_serial *gserial, const enum gnss_output_protocol protocol) {
	const struct ubx_data *data = gnss_serial_get_drvdata(gserial);
	const struct ubx_features *features = data->features;
	size_t count = 0U;

	int ret = prepare_zedf9_gnss_protocol_msg(protocol, &gdev->dev, features);
	if (ret)
		return ret;
	count = gdev->ops->write_raw(gdev, ZED_F9_PROTOCOL_MSG, PROTOCOL_MSG_TOTAL_LEN);
	if (count != PROTOCOL_MSG_TOTAL_LEN) {
	    dev_err(&gdev->dev, "Failed to set GNSS protocol to %s.\n", gnss_output_protocol_name(protocol));
		return EINVAL;
	}
	dev_info(&gdev->dev, "Set GNSS protocol to %s.\n", gnss_output_protocol_name(protocol));

	return 0;
}

/* Configure PPS by selecting the time pulse reference. */
static int set_zedf9_time_pulse_reference(struct gnss_device *gdev, struct gnss_serial* gserial) {
	const struct ubx_data *data = gnss_serial_get_drvdata(gserial);
	const struct ubx_features *features = data->features;
	size_t count = 0U;
	int ret;

	if (!data->features)
		return -EINVAL;

	ret = prepare_zedf9_time_pulse_msg(&gdev->dev, features);
	if (ret)
		return ret;
	count = gdev->ops->write_raw(gdev, ZED_F9_PPS_MSG, PPS_MSG_TOTAL_LEN);
	if (PPS_MSG_TOTAL_LEN != count) {
		dev_err(&gdev->dev, "Time pulse reference setting failed.");
		return count;
	}
	BUG_ON(max_reference_name < features->default_time_reference);
	dev_info(&gdev->dev, "Set time pulse reference to %s.\n", timepulse_reference_names[features->default_time_reference]);
	return 0;
}

static int set_zedf9_dynamic_model(struct gnss_device *gdev, struct gnss_serial* gserial) {
	const struct ubx_data *data = gnss_serial_get_drvdata(gserial);
	const struct ubx_features *features = data->features;
	size_t count = 0U;
	int ret;

	if (!features)
		return -EINVAL;
	const size_t default_model = features->default_dynamic_model;
	BUG_ON(max_platform_model_name < default_model);
	/* The enum values dictated by U-Blox skip the value 1. */
	BUG_ON(1 == default_model);

	ret = prepare_zedf9_dynamic_model_msg(&gdev->dev, features);
	if (ret)
		return ret;
	count = gdev->ops->write_raw(gdev, ZED_F9_MODEL_MSG, MODEL_MSG_TOTAL_LEN);
	if (MODEL_MSG_TOTAL_LEN != count) {
		dev_err(&gdev->dev, "Dynamic platform model setting failed.");
		return count;
	}
	/* Code around the missing enum value. */
	const char* dynamic_model_name = (default_model == 0) ?
	    dynamic_platform_model_names[0] :
		    dynamic_platform_model_names[(default_model - 1)];
	dev_info(&gdev->dev, "Set dynamic platform model to %s.\n", dynamic_model_name);
	return 0;
}


/*
 * Almost the same as gnss_serial_open() but does not set baud.
 * Leaves device closed on error.
 */
static int _do_serial_open(struct serdev_device* serdev) {
	int ret = serdev_device_open(serdev);
	if (ret) {
		dev_err(&serdev->dev, "Unable to open GNSS serial device.\n");
		return ret;
	}
	serdev_device_set_flow_control(serdev, false);
	ret = pm_runtime_get_sync(&serdev->dev);
	if (ret < 0) {
		serdev_device_close(serdev);
		pm_runtime_put_noidle(&serdev->dev);
		return ret;
	}
	return 0;
}

/* Same as gnss_serial_close(). */
static void _do_serial_close(struct serdev_device* serdev) {
	serdev_device_close(serdev);
	pm_runtime_put(&serdev->dev);
}

/* Call with serial device already open */
static int zed_f9_configure(struct gnss_device *gdev) {
	struct gnss_serial *gserial = gnss_get_drvdata(gdev);
	struct serdev_device *serdev = gserial->serdev;
	struct ubx_data *data = gnss_serial_get_drvdata(gserial);
	const struct ubx_features *features;
	speed_t new_baud = 0U;
	int ret;

	if (!data) {
		dev_err(&gdev->dev, "Lookup of driver data failed.\n");
		return -ENODATA;
	}
	if (data->is_configured)
		return 0;
	features = data->features;
	if (!features) {
		dev_warn(&gdev->dev, "No configured driver features.\n");
		return 0;
	}

	/* Initially set the UART to the default speed to match the GNSS' power-on value. */
	new_baud = serdev_device_set_baudrate(serdev, features->default_baud);
	if (features->default_baud != new_baud) {
		dev_err(&gdev->dev, "Failed to set serial device to GNSS default baud %u\n",
			features->default_baud);
	}
	/* 4800 is the default value set by gnss_serial_parse_dt() */
	if (gserial->speed == 4800) {
		/* Fall back instead to Zed F9 default */
		gserial->speed = features->default_baud;
	} else {
		ret = set_zedf9_baud(gdev, serdev, gserial);
		if (ret) {
			dev_err(&gdev->dev, "GNSS speed setting to %u failed\n", gserial->speed);
			return ret;
		}
		new_baud = serdev_device_set_baudrate(serdev, gserial->speed);
		if (gserial->speed != new_baud) {
			dev_err(&gdev->dev, "Serial device speed setting to %u failed\n", gserial->speed);
			return ret;
		}
		dev_info(&gdev->dev, "Set GNSS speed to %u\n", gserial->speed);
	}

	ret = enable_zedf9_antenna_control(gdev, gserial);
	if (ret)
		return ret;

	/* Don't overwrite the protocol set by userspace, if any. */
	enum gnss_output_protocol protocol_to_set = (PROTOCOL_NONE == data->selected_protocol) ?
			features->default_protocol : data->selected_protocol;
	ret = set_zedf9_gnss_protocol(gdev, gserial, protocol_to_set);
	if (ret)
		return ret;
	ret = set_zedf9_time_pulse_reference(gdev, gserial);
	if (ret)
		return ret;
	ret = set_zedf9_generation_period(gdev, gserial);
	if (ret)
		return ret;
	ret = set_zedf9_send_period(gdev, gserial);
	if (ret)
		return ret;
	ret = set_zedf9_dynamic_model(gdev, gserial);
	if (ret)
		return ret;

	data->is_configured = 1;

	return 0;
}

/* Opens and then closes the serial device.   Configures the GNSS if need be. */
static ssize_t protocol_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count) {
	struct gnss_device *gdev = to_gnss_device(dev);
	struct gnss_serial *gserial;
	struct serdev_device *serdev;
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
	serdev = gserial->serdev;
	data = gnss_serial_get_drvdata(gserial);
	if (!data) {
		dev_err(dev, "Lookup of driver data failed.\n");
		return ret;
	}

	/* Atomically take device reference. */
	get_device(&gdev->dev);
	/* Protect the counter from changes by concurrent gnss users. */
	down_write(&gdev->rwsem);
	if (gdev->disconnected) {
		ret = -ENODEV;
		goto close;
	}
	/* Open the device iff it is not open. Will configure the device if needed. */
	if (gdev->count++ == 0) {
		ret = gdev->ops->open(gdev);
		if (ret) {
			dev_err(&gdev->dev, "Unable to open GNSS serial device.\n");
			goto close;
		}
	}

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
	if (ret)
		goto close;
	ret = count;

close:
	if (--gdev->count == 0) {
		_do_serial_close(serdev);
	}
	up_write(&gdev->rwsem);
	put_device(&gdev->dev);
	return ret;
}
static __maybe_unused DEVICE_ATTR_WO(protocol);

/* Opens the serial device, not the GNSS. */
static int zed_f9_serial_open(struct gnss_device *gdev)
{
	struct gnss_serial *gserial = gnss_get_drvdata(gdev);
	struct serdev_device *serdev = gserial->serdev;


	int ret = _do_serial_open(serdev);
	if (ret) {
		dev_err(&gdev->dev, "Unable to open GNSS serial device.\n");
		return ret;
	}
	ret = zed_f9_configure(gdev);
	if (ret) {
		dev_err(&gdev->dev, "GNSS serial device configuration failed.\n");
		goto err_close;
	}
	return 0;

err_close:
	_do_serial_close(serdev);
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
	/*
         * The first set of writes turn the feature on by writing '1' to the
	 * register.  The second set of writes at the end disable the features by
	 * writing '0'.
         */
	.protocol_regs				=	{
							0x10740001, /* CFG_UART1OUTPROT_UBX */
							0x10740002, /* CFG_UART1OUTPROT_NMEA */
							0x10720001, /* CFG_I2COUTPROT_UBX -- enable */
							0x209102a5, /* CFG-MSGOUT-UBX_RXM_RAWX_UART1 */
							0x20910232, /* CFG-MSGOUT-UBX_RXM_SFRBX_UART1 */
							0x20910350, /* CFG-MSGOUT-UBX_MON_COMMS_UART1 */
							0x2091017e, /* CFG-MSGOUT-UBX_TIM_TP_UART1, */
							0x2091017d, /* CFG-MSGOUT-UBX_TIM_TP_I2C, */
							0x2091035a, /* CFG-MSGOUT-UBX_MON_RF_UART1 */
							0x20910359, /* CFG-MSGOUT-UBX_MON_RF_I2C */
							0x10720002, /* CFG_I2COUTPROT_NMEA -- disable */
							0x10310022, /* CFG-SIGNAL-BDS_ENA */
							0x1031000d, /* CFG-SIGNAL-BDS_B1_ENA */
							0x1031000e, /* CFG-SIGNAL-BDS_B2_ENA */
							0x10080001, /* CFG-SFCORE-USE_SF */
							},
	/*
	 * Write a value larger than 1 to these registers for a higher message output
	 * rate.
	 */
	.send_period_regs			=	{
							0x20910066, /* CFG-MSGOUT-UBX_NAV_CLOCK_UART1 */
							0x20910160, /* CFG-MSGOUT-UBX_NAV_EOE_UART1 */
							0x20910007, /* CFG-MSGOUT-UBX_NAV_PVT_UART1 */
							0x20910048, /* CFG-MSGOUT-UBX_NAV_TIMEGPS_UART1 */
							},
	.generation_period_reg			=	0x30210001, /* CFG-RATE-MEAS */
	.timepulse_reg				=	0x2005000c,
	.dynamic_model_reg			=	0x20110021,
	.min_baud				=	9600,
	.default_baud				=	38400,
	.max_baud				=	921600,
	/* The default period for navigation solution generation and reporting = 200 ms. */
	.default_meas_period			=	200,
	.min_meas_period			=	25,
	.default_protocol     			=	UBX,
	.default_time_reference			=	GPS,
	.default_dynamic_model			=	AIR4,
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
	struct gpio_desc *reset;
	struct ubx_data *data;
	struct gnss_operations *ubx_gnss_ops;
	struct gnss_device *gdev;
	int ret;

	struct gnss_serial *gserial __free(free_serial) = gnss_serial_allocate(serdev, sizeof(*data));
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
		return PTR_ERR(data->vcc);
	}

	ret = devm_regulator_get_enable_optional(&serdev->dev, "v-bckp");
	if (ret < 0 && ret != -ENODEV)
		return ret;

	/* Deassert reset */
	reset = devm_gpiod_get_optional(&serdev->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(reset)) {
		return PTR_ERR(reset);
	}

	/* Ownership of gserial is transferred to the GNSS subsystem by passing
	 * no_free_ptr(gserial) to gnss_serial_register(). After this point,
	 * gserial will not be automatically freed in this function.
	 */
	ret = gnss_serial_register(no_free_ptr(gserial));
	if (ret)
		return ret;

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
	/* Open and close the device just to run the configure function, which requires
	 *  the device to be open. */
	ret = zed_f9_serial_open(gdev);
	if (ret) {
		dev_err(&gdev->dev, "Unable to open serial device from driver probe \
			method.\n");
		return ret;
	}
	_do_serial_close(serdev);

	return 0;
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
