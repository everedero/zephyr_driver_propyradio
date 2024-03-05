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
};

struct cc2500_data {
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
	};                                                                            \
                                                                                  \
	static struct cc2500_data cc2500_##i = {                                  \
	};                                                                            \
	DEVICE_DT_INST_DEFINE(i, cc2500_init, NULL, &cc2500_##i,  \
			      &cc2500_config_##i, POST_KERNEL,              \
			      CONFIG_CC2500_INIT_PRIORITY, &cc2500_api);

DT_INST_FOREACH_STATUS_OKAY(CC2500_DEFINE)
