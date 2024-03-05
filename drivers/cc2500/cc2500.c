/*
 * Copyright (c) 2024 Eve Redero
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_cc2500

#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

#include <app/drivers/propy_radio.h>
#include "cc2500_defines.h"

#define SPI_MAX_MSG_LEN 64
#define SPI_MAX_REG_LEN 5
#define SPI_MSG_QUEUE_LEN 10

LOG_MODULE_REGISTER(cc2500, CONFIG_CC2500_LOG_LEVEL);

struct cc2500_config {
	const struct spi_dt_spec spi;
	struct gpio_dt_spec ce;
	struct gpio_dt_spec irq;
};

struct cc2500_data {
	int addr_width;
	int channel_frequency;
	bool data_rate_2mbps;
	int rf_power_attenuation;
	bool lna_gain;
	bool crc_encoding_twobytes;
	uint8_t tx_address[5];
	bool dynamic_payload;
	uint8_t payload_fixed_size;
	bool payload_ack;
	bool payload_crc;
	int rx_datapipes_number;
	uint8_t rx_datapipe0_address[5];
	uint8_t rx_datapipe1_address[5];
	uint8_t rx_child_datapipes_addresses[4];
	uint8_t rx_datapipes_dynamic_payload[6];
	bool is_listening;
	uint8_t write_ret_code;
#ifdef CONFIG_CC2500_TRIGGER
	/** RX queue buffer. */
	uint8_t rx_queue_buf[SPI_MSG_QUEUE_LEN * SPI_MAX_MSG_LEN];
	/** RX queue. */
	struct k_msgq rx_queue;
	/** Trigger work queue. */
	struct k_work trig_work;
	/** Touch GPIO callback. */
	struct gpio_callback irq_cb;
	/** Semaphore for TX. */
	struct k_sem sem;
	/** Self reference (used in work queue context). */
	const struct device *dev;
#endif /* CONFIG_CC2500_TRIGGER */
};

/* Init */
static int cc2500_init(const struct device *dev)
{
	const struct cc2500_config *config = dev->config;
	struct cc2500_data *data = dev->data;
	int ret;

	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("SPI not ready");
	}
	if (!spi_cs_is_gpio_dt(&config->spi)) {
		LOG_ERR("No CS GPIO found");
	}

	if (!gpio_is_ready_dt(&config->ce)) {
		return -EBUSY;
	}
	ret = gpio_pin_configure_dt(&config->ce, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Could not configure CE GPIO (%d)", ret);
		return ret;
	}

	ret = gpio_pin_set_dt(&config->ce, HIGH);
	if (ret < 0) {
		LOG_ERR("Could not toggle CE (%d)", ret);

		return ret;
	}
	ret = gpio_pin_configure_dt(&config->spi.config.cs.gpio, GPIO_OUTPUT);
	if (ret < 0) {
		LOG_ERR("Could not configure CS GPIO (%d)", ret);
		return ret;
	}

	if (!cc2500_test_spi(dev))
	{
		LOG_ERR("Issue with SPI read/write");
		return -EIO;
	}

	return 0;
}

#define CC2500_SPI_MODE SPI_WORD_SET(8)

#define CC2500_DEFINE(i)                                               \
	static const struct cc2500_config cc2500_config_##i = {          \
		.spi = SPI_DT_SPEC_INST_GET(i, CC2500_SPI_MODE, 2),            \
		.ce = GPIO_DT_SPEC_INST_GET(i, ce_gpios),                        \
		IF_ENABLED(CONFIG_CC2500_TRIGGER,                              \
			(.irq = GPIO_DT_SPEC_INST_GET(i, irq_gpios),))               \
	};                                                                            \
                                                                                  \
	static struct cc2500_data cc2500_##i = {                                  \
		.addr_width = DT_INST_PROP_OR(i, addr_width, 5),                                \
		.channel_frequency = DT_INST_PROP(i, channel_frequency),                  \
		.data_rate_2mbps = DT_INST_PROP_OR(i, data_rate_2mbps, false),            \
		.rf_power_attenuation = DT_INST_PROP(i, rf_power_attenuation),            \
		.lna_gain = DT_INST_PROP_OR(i, lna_gain, false),                          \
		.crc_encoding_twobytes = DT_INST_PROP_OR(i, crc_encoding_twobytes, false),\
		.tx_address = DT_INST_PROP(i, tx_address),                                \
		.payload_fixed_size = DT_INST_PROP_OR(i, payload_fixed_size, 32),            \
		.dynamic_payload = DT_INST_PROP_OR(i, dynamic_payload, false),            \
		.payload_ack = DT_INST_PROP_OR(i, payload_ack, false),                    \
		.payload_crc = DT_INST_PROP_OR(i, payload_crc, false),                    \
		.rx_datapipes_number = DT_INST_PROP(i, rx_datapipes_number),                      \
		.rx_datapipe0_address = DT_INST_PROP(i, rx_datapipe0_address),            \
		.rx_datapipe1_address = DT_INST_PROP(i, rx_datapipe1_address),            \
		.is_listening = false,                                                    \
		.write_ret_code = 0,                                                    \
		.rx_child_datapipes_addresses = {                 \
			DT_INST_PROP(i, rx_datapipe2_address),           \
			DT_INST_PROP(i, rx_datapipe3_address),            \
			DT_INST_PROP(i, rx_datapipe4_address),            \
			DT_INST_PROP(i, rx_datapipe5_address)},            \
		.rx_datapipes_dynamic_payload = DT_INST_PROP_OR(i,                        \
				rx_datapipes_dynamic_payload, {}),                                \
	};                                                                            \
	DEVICE_DT_INST_DEFINE(i, cc2500_init, NULL, &cc2500_##i,  \
			      &cc2500_config_##i, POST_KERNEL,              \
			      CONFIG_CC2500_INIT_PRIORITY, &cc2500_api);

DT_INST_FOREACH_STATUS_OKAY(CC2500_DEFINE)
