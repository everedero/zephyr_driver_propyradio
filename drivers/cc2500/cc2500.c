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

/* Max data fifo len */
#define SPI_MAX_MSG_LEN 64

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

uint8_t cc2500_read_status(const struct device *dev)
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
	tx_data[0] = ( READ_SINGLE | ( RW_MASK & 0x00 ) );
	tx_data[1] = 0x00;

	ret = spi_transceive_dt(&config->spi, &tx, &rx);

	if (ret) {
		LOG_ERR("Error transceive %d", ret);
		return 0;
	}

	// status is 1st byte of receive buffer
	return rx_data[0];
}

uint8_t cc2500_write_register_len(const struct device *dev, uint8_t reg, const uint8_t* data, uint8_t len)
{
	const struct cc2500_config *config = dev->config;
	uint8_t tx_data[SPI_MAX_MSG_LEN + 1] = {0};
	uint8_t rx_data = 0;
	int ret;
	const struct spi_buf tx_buf[1] = {
		{
			.buf = tx_data,
			.len = (len + 1)
		}
	};
	const struct spi_buf rx_buf[1] = {
		{
			.buf = &rx_data,
			.len = 1
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

	tx_data[0] = ( WRITE_BURST | ( RW_MASK & reg ) );
	memcpy(&tx_data[1], data, len);

	ret = spi_transceive_dt(&config->spi, &tx, &rx);
	if (ret) {
		LOG_ERR("Error transceive %d", ret);
		return 0;
	}
	return rx_data;
}

uint8_t cc2500_cmd_register(const struct device *dev, uint8_t cmd)
{
	const struct cc2500_config *config = dev->config;
	int ret;
	uint8_t tx_data[1];
	uint8_t rx_data[1];
	const struct spi_buf tx_buf[1] = {
		{
			.buf = tx_data,
			.len = 1
		}
	};
	const struct spi_buf rx_buf[1] = {
		{
			.buf = rx_data,
			.len = 1
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

	tx_data[0] = cmd;

	ret = spi_transceive_dt(&config->spi, &tx, &rx);
	if (ret) {
		LOG_ERR("Error transceive %d\n", ret);
		return 0;
	}

	return rx_data[0];
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

/* Private driver function */
static uint8_t cc2500_reset(const struct device *dev)
{
	return(cc2500_cmd_register(dev, SRES));
}

static uint8_t cc2500_flush_rx(const struct device *dev)
{
	return(cc2500_cmd_register(dev, SFRX));
}

static uint8_t cc2500_flush_tx(const struct device *dev)
{
	return(cc2500_cmd_register(dev, SFTX));
}

static uint8_t cc2500_idle(const struct device *dev)
{
	return(cc2500_cmd_register(dev, SIDLE));
}

static uint8_t cc2500_set_rx(const struct device *dev)
{
	return(cc2500_cmd_register(dev, SRX));
}

static uint8_t cc2500_set_tx(const struct device *dev)
{
	return(cc2500_cmd_register(dev, STX));
}

static uint8_t cc2500_set_pkt_len(const struct device *dev, int len)
{
	return(cc2500_write_register(dev, PKTLEN, len));
}

static void cc2500_write_register_burst(const struct device *dev, uint8_t reg, const uint8_t* data, uint8_t len)
{
	/* Packet length +1 because we count the reg byte */
	cc2500_set_pkt_len(dev, len+1);
	cc2500_write_register_len(dev, reg, data, len);
}

/* API functions */
static int cc2500_read(const struct device *dev, uint8_t *buffer, uint8_t data_len)
{
	int ret = 0;

	return ret;
}

static int cc2500_write(const struct device *dev, uint8_t *buffer, uint8_t data_len)
{
	int ret = 0;
	int status = 0;

	cc2500_idle(dev);
	cc2500_write_register_burst(dev, TXFIFO, buffer, data_len);
	cc2500_set_tx(dev);
	status = cc2500_read_status(dev);
	LOG_DBG("Status: 0x%x", status);
	k_msleep(1);
	status = cc2500_read_status(dev);
	LOG_DBG("Status: 0x%x", status);

	cc2500_flush_tx(dev);
	status = cc2500_idle(dev);
	LOG_DBG("Status idle: 0x%x", status);
	cc2500_set_rx(dev);
	status = cc2500_idle(dev);
	LOG_DBG("Status idle: 0x%x", status);
	return ret;
}

static const struct propy_radio_api cc2500_api = {
	.read = cc2500_read,
	.write = cc2500_write,
};


/* Init */
static int cc2500_init(const struct device *dev)
{
	uint8_t pa_data[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	const struct cc2500_config *config = dev->config;
	struct cc2500_data *data = dev->data;
	uint8_t reg_value;
	int i;
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
	cc2500_reset(dev);
	cc2500_set_config_registers(dev);

	cc2500_write_register_burst(dev, PATABLE, pa_data, 8);

	reg_value = cc2500_read_register(dev, FREQ0);
	LOG_DBG("Freq 0: %d", reg_value);

	cc2500_write_register(dev, IOCFG2, 0x5C);
	cc2500_write_register(dev, IOCFG0, 0x5B);

	cc2500_flush_rx(dev);
	cc2500_flush_tx(dev);
	cc2500_idle(dev);

	cc2500_set_rx(dev);

	/* Single read */
	reg_value = cc2500_read_register(dev, FSCAL1);
	LOG_DBG("FS Cal1: %d", reg_value);

	for (i=0; i<30; i++) {
		reg_value = cc2500_read_register(dev, RSSI);
		LOG_DBG("RSSI: %d", reg_value);
		k_msleep(1);
	}

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
