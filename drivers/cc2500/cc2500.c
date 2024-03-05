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
	const uint8_t array[40];
	const int array_len;
};

struct cc2500_data {
};

/* Private core communication functions */
uint8_t cc2500_write_register(const struct device *dev, uint8_t reg, uint8_t data)
{
	/* Register config can only be done in power down or standby */
	const struct cc2500_config *config = dev->config;
	uint8_t tx_data[2];
	uint8_t rx_data = 0;
	int ret;
	const struct spi_buf tx_buf[1] = {
		{
			.buf = tx_data,
			.len = 2
		}
	};
	const struct spi_buf rx_buf[1] = {
		{
			.buf = &rx_data,
			.len = 2
		}
	};
	struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = 1
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 1
	};

	// 5 lower bits for address, the 6th is 0 for read and 1 for write
	tx_data[0] = ( WRITE_SINGLE | ( RW_MASK & reg ) );
	tx_data[1] = data;

	ret = spi_transceive_dt(&config->spi, &tx, &rx);
	if (ret) {
		LOG_ERR("Error transceive %d", ret);
		return 0;
	}
	return rx_data;
}

uint8_t cc2500_read_register(const struct device *dev, uint8_t reg)
{
	const struct cc2500_config *config = dev->config;
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	int ret;
	const struct spi_buf tx_buf[1] = {
		{
			.buf = tx_data,
			.len = 2
		}
	};
	const struct spi_buf rx_buf[1] = {
		{
			.buf = (void *)rx_data,
			.len = 2
		}
	};
	struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = 1
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 1
	};

	// 5 lower bits for address, the 6th is 0 for read and 1 for write
	tx_data[0] = ( READ_SINGLE | ( RW_MASK & reg ) );
	tx_data[1] = NOP;

	ret = spi_transceive_dt(&config->spi, &tx, &rx);

	if (ret) {
		LOG_ERR("Error transceive %d", ret);
		return 0;
	}

	// status is 1st byte of receive buffer
	return rx_data[1];
}

static int cc2500_set_config_registers(const struct device *dev)
{
	int ret = 0;
	int i, reg;
	const struct cc2500_config *config = dev->config;

	if (config->array_len < 40) {
		LOG_DBG("No valid default config");
		/* No valid startup config */
		return 0;
	}
	for (i=0; i<40; i++) {
		cc2500_write_register(dev, i+0x07, config->array[i]);
	}
	return ret;
}

/* API functions */

static int cc2500_read(const struct device *dev, uint8_t *buffer, uint8_t data_len)
{
	int ret = 0;
	struct cc2500_data *data = dev->data;
	return ret;
}

static int cc2500_write(const struct device *dev, uint8_t *buffer, uint8_t data_len)
{
	int ret = 0;
	struct cc2500_data *data = dev->data;
	return ret;
}

static const struct propy_radio_api cc2500_api = {
	.read = cc2500_read,
	.write = cc2500_write,
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

	ret = gpio_pin_configure_dt(&config->spi.config.cs.gpio, GPIO_OUTPUT);
	if (ret < 0) {
		LOG_ERR("Could not configure CS GPIO (%d)", ret);
		return ret;
	}

	cc2500_set_config_registers(dev);

	return 0;
}

#define CC2500_SPI_MODE SPI_WORD_SET(8)

#define CC2500_DEFINE(i)                                               \
	static const struct cc2500_config cc2500_config_##i = {          \
		.spi = SPI_DT_SPEC_INST_GET(i, CC2500_SPI_MODE, 2),            \
		.array = DT_INST_PROP_OR(i, conf_array, {}), \
		.array_len = DT_INST_PROP_LEN_OR(i, conf_array, 0), \
	};                                                                            \
                                                                                  \
	static struct cc2500_data cc2500_##i = {                                  \
	};                                                                            \
	DEVICE_DT_INST_DEFINE(i, cc2500_init, NULL, &cc2500_##i,  \
			      &cc2500_config_##i, POST_KERNEL,              \
			      CONFIG_CC2500_INIT_PRIORITY, &cc2500_api);

DT_INST_FOREACH_STATUS_OKAY(CC2500_DEFINE)
