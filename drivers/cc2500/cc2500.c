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
#define SPI_MSG_QUEUE_LEN 10

LOG_MODULE_REGISTER(cc2500, CONFIG_CC2500_LOG_LEVEL);

struct cc2500_config {
	const struct spi_dt_spec spi;
	const uint8_t array[40];
	const int array_len;
	struct gpio_dt_spec irq;
	bool gpio2_as_irq;
};

struct cc2500_data {
	int channel_frequency;
	int start_frequency;
	int modulation_format;
	bool payload_crc;
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

uint8_t cc2500_read_register_len(const struct device *dev, uint8_t reg, uint8_t* data, uint8_t len)
{
	const struct cc2500_config *config = dev->config;
	uint8_t tx_data[SPI_MAX_MSG_LEN + 1] = {0};
	uint8_t rx_data[SPI_MAX_MSG_LEN + 1] = {0};
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
			.len = (len + 1)
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

	tx_data[0] = ( READ_BURST | ( RW_MASK & reg ) );

	ret = spi_transceive_dt(&config->spi, &tx, &rx);
	if (ret) {
		LOG_ERR("Error transceive %d", ret);
		return 0;
	}
	memcpy(data, &rx_data[1], len);
	return rx_data[0];
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
	int i;
	uint8_t idx;
	const struct cc2500_config *config = dev->config;
	/*uint8_t array[] = {0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10,
		0x11, 0x12, 0x13, 0x14, 0xA, 0x15, 0x21, 0x22,
		0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x23, 0x24,
		0x25, 0x26, 0x29, 0x2C, 0x2D, 0x2E, 0x07, 0x08,
		0x09};*/

	if (config->array_len < 40) {
		LOG_DBG("No valid default config");
		/* No valid startup config */
		return 0;
	}
	/*for (i=0; i<31; i++) {
		idx = array[i] - 7;
		LOG_DBG("Write reg 0x%x: 0x%x", array[i], config->array[idx]);
		cc2500_write_register(dev, array[i], config->array[idx]);
	}*/

	for (i=0; i<40; i++) {
		if ((i+0x07) == 0x2A || (i+0x07) == 0x2B
			|| (i+0x07) == 0x29) {
			//(i+0x07) == 0x27 || (i+0x07) == 0x28) {
			continue;
		}
		LOG_DBG("Write reg 0x%x: 0x%x", i+0x07, config->array[i]);
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

static uint8_t cc2500_set_channel_num(const struct device *dev, uint8_t chan_num)
{
	return(cc2500_write_register(dev, CHANNR, chan_num));
}

static uint8_t cc2500_set_pkt_len(const struct device *dev, uint8_t len)
{
	return(cc2500_write_register(dev, PKTLEN, len));
}

static void cc2500_write_register_burst(const struct device *dev, uint8_t reg, const uint8_t* data, uint8_t len)
{
	cc2500_write_register_len(dev, reg, data, len);
}

static int cc2500_get_rssi(const struct device *dev)
{
	/* RSSI has burst-only access */
	uint8_t reg_value;
	int rssi;
	cc2500_read_register_len(dev, RSSI, &reg_value, 1);
	if (reg_value >= 128) {
		rssi = reg_value - 256;
	} else {
		rssi = reg_value;
	}
	return(rssi);
}

static bool cc2500_test_spi(const struct device *dev)
{
	uint8_t ret = 0;
	cc2500_write_register(dev, SYNC0, 0x0E);
	ret = cc2500_read_register(dev, SYNC0);
	cc2500_write_register(dev, SYNC0, 0x91);
	if (ret != 0x0E) {
		return(false);
	}
	return(true);
}

static uint8_t cc2500_get_pkt_status(const struct device *dev)
{
	uint8_t status = 0;
	/* PKTSTATUS has burst-only access */
	cc2500_read_register_len(dev, PKTSTATUS, &status, 1);
	LOG_INF("GDO0 val: %d", (status & 0x01) >> 0);
	LOG_INF("GDO2 val: %d", (status & 0x04) >> 2);
	LOG_INF("Sync word found: %d", (status & 0x08) >> 3);
	LOG_INF("Channel clear: %d", (status & 0x10) >> 4);
	LOG_INF("Carrier sense: %d", (status & 0x40) >> 6);
	LOG_INF("CRC OK: %d", (status & 0x80) >> 7);
	/* Return CRC OK bit */
	return((status & 0x80) >> 7);
}

#ifndef CONFIG_CC2500_TRIGGER
/* Polling behaviour */
static uint8_t cc2500_is_crc_ok(const struct device *dev)
{
	uint8_t status = 0;
	/* PKTSTATUS has burst-only access */
	cc2500_read_register_len(dev, PKTSTATUS, &status, 1);
	/* Return CRC OK bit */
	return((status & CRC_OK) >> 7);
}
#endif /* not CONFIG_CC2500_TRIGGER */

static uint8_t cc2500_has_data(const struct device *dev)
{
	/* Reads status byte to know if data is available */
	uint8_t status = 0;
	status = cc2500_read_status(dev);
	if ((status & 0xf) > 0) {
		return(true);
	}
	return(false);
}

/* API functions */
static int cc2500_read(const struct device *dev, uint8_t *buffer, uint8_t data_len)
{
	int ret = 0;
	uint8_t status = 0;
	const struct cc2500_config *config = dev->config;
	struct cc2500_data *data = dev->data;
	cc2500_idle(dev);
	k_msleep(1);
	cc2500_flush_rx(dev);
	cc2500_set_pkt_len(dev, data_len);
	cc2500_set_rx(dev);
#ifdef CONFIG_CC2500_TRIGGER
	uint8_t buffer_full[SPI_MAX_MSG_LEN] = {0};

	if (k_msgq_get(&data->rx_queue, buffer_full, K_MSEC(CONFIG_CC2500_READ_TIMEOUT)) < 0) {
		LOG_INF("Nothing in RX queue");
		return(-EIO);
	}
	memcpy(buffer, buffer_full, data_len);
#else
	while (!cc2500_has_data(dev)){
		k_msleep(1);
	}
	if (data->payload_crc && !cc2500_is_crc_ok(dev)) {
		LOG_WRN("Wrong CRC");
		return -EIO;
	}
	status = cc2500_read_register_len(dev, RXFIFO, buffer, data_len);
#endif /* CONFIG_CC2500_TRIGGER */
	cc2500_idle(dev);
	cc2500_flush_rx(dev);
	return ret;
}

static int cc2500_write(const struct device *dev, uint8_t *buffer, uint8_t data_len)
{
	int ret = 0;
	int status = 0;

	cc2500_idle(dev);
	status = cc2500_read_status(dev);
	cc2500_write_register_burst(dev, TXFIFO, buffer, data_len);
	/* Packet length +1 because we count the reg byte */
	cc2500_set_pkt_len(dev, data_len);

	cc2500_set_tx(dev);
	status = cc2500_read_status(dev);
	k_usleep(800);
	status = cc2500_read_status(dev);

	cc2500_flush_tx(dev);
	status = cc2500_idle(dev);
	cc2500_set_rx(dev);
	status = cc2500_idle(dev);
	return ret;
}

static const struct propy_radio_api cc2500_api = {
	.read = cc2500_read,
	.write = cc2500_write,
};

/* Init subfunction */
static int cc2500_set_channel_process(const struct device *dev, uint8_t chann)
{
	uint8_t status;
	int ret = 0;
	cc2500_idle(dev);
	cc2500_idle(dev);
	status = cc2500_read_status(dev);
	cc2500_set_channel_num(dev, chann);
	cc2500_flush_rx(dev);
	cc2500_flush_tx(dev);
	cc2500_idle(dev);
	cc2500_set_rx(dev);

	return(ret);
}

static int cc2500_rssi_process(const struct device *dev)
{
	uint8_t reg_value;
	int i;
	int ret = 0;
	long total = 0;
	const int num_mes = 50;
	uint8_t current_chan = 0x01;
	const uint8_t last_chan = 9;
	uint8_t max_chan = 0;
	long max = -0x7FFFFFFF;

	for (current_chan=1; current_chan<=last_chan; current_chan++) {
		total = 0;
		cc2500_set_channel_process(dev, current_chan);
		k_msleep(5);
		/* Single read */
		reg_value = cc2500_read_register(dev, FSCAL1);
		LOG_DBG("FS cal1: %d",  reg_value);
		for (i=0; i<num_mes; i++) {
			total += cc2500_get_rssi(dev);
			k_usleep(3600);
		}
		//total = total / 45;
		if (total > max) {
			max = total;
			max_chan = current_chan;
		}
		LOG_DBG("RSSI 0x%x: %ld", current_chan, total);
	}
	LOG_INF("RSSI max 0x%x: %ld", max_chan, max);
	// I guess we choose the max here
	cc2500_set_channel_process(dev, max_chan);

	return(ret);
}

static void cc2500_read_default(const struct device *dev)
{
	int i;
	uint8_t reg_val;

	for (i = 0x07; i<=0x2E; i++) {
		reg_val = cc2500_read_register(dev, i);
		LOG_DBG("Reg 0x%x: val 0x%x", i, reg_val);
	}
}

#ifdef CONFIG_CC2500_TRIGGER
/* Interrupts */

void irq_callback_handler(const struct device *port,
	struct gpio_callback *cb, uint32_t pins)
{
	struct cc2500_data *data = CONTAINER_OF(cb, struct cc2500_data, irq_cb);
	const struct cc2500_config *config = data->dev->config;
	int ret;
	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	/* disable any new touch interrupts until work is processed */
	ret = gpio_pin_interrupt_configure_dt(&config->irq, GPIO_INT_DISABLE);
	if (ret < 0)
	{
		LOG_ERR("Could not deactivate interrupt");
	}
	k_work_submit(&data->trig_work);
}

void work_queue_callback_handler(struct k_work *item)
{
	struct cc2500_data *data =
		CONTAINER_OF(item, struct cc2500_data, trig_work);
	return;
}
#endif /* CONFIG_CC2500_TRIGGER */

/* Init */
static int cc2500_init(const struct device *dev)
{
	//uint8_t pa_data[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	uint8_t pa_data[] = {0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6};
	const struct cc2500_config *config = dev->config;
	struct cc2500_data *data = dev->data;
	uint8_t reg_value;
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
	if (!cc2500_test_spi(dev)) {
		LOG_ERR("SPI read write test failed");
		return(-EIO);
	}
#ifdef CONFIG_CC2500_TRIGGER
	uint8_t irq_mode;
	LOG_INF("Trigger config");
	data->dev = dev;

	// Semaphore for tx
	k_sem_init(&data->sem, 0, 1);

	// Message queue
	k_msgq_init(&data->rx_queue, data->rx_queue_buf, SPI_MAX_MSG_LEN,
		SPI_MSG_QUEUE_LEN);

	if (!gpio_is_ready_dt(&config->irq)) {
		return -EBUSY;
	}
	ret = gpio_pin_configure_dt(&config->irq, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Could not configure IRQ GPIO (%d)", ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&config->irq, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Could not configure interrupt for IRQ GPIO (%d)", ret);
		return ret;
	}

	/* Setup GPIO 0 or GPIO 2 as RX FIFO */
	irq_mode = 0x00;
	if (data->payload_crc) {
		/* IRQ on CRC OK */
		irq_mode = 0x07;
	}
	if (config->gpio2_as_irq) {
		cc2500_write_register(dev, IOCFG2, 0x00);
	} else {
		cc2500_write_register(dev, IOCFG0, 0x00);
	}
	/* 4 bytes in RX FIFO before threshold, deasserts when empty */
	cc2500_write_register(dev, FIFOTHR, 0x01);

	// setup the interrupt function
    gpio_init_callback(&data->irq_cb, irq_callback_handler, BIT(config->irq.pin));

	ret = gpio_add_callback(config->irq.port, &data->irq_cb);
	if (ret < 0) {
		LOG_ERR("Could not configure irq callback (%d)", ret);
		return ret;
	}
	k_work_init(&data->trig_work, work_queue_callback_handler);

#endif /* CONFIG_CC2500_TRIGGER */
	cc2500_reset(dev);
	cc2500_set_config_registers(dev);

	cc2500_set_pkt_len(dev, 9);
	cc2500_write_register_burst(dev, PATABLE, pa_data, 8);

	reg_value = cc2500_read_register(dev, FREQ0);
	LOG_DBG("Freq 0: %d", reg_value);

	cc2500_flush_rx(dev);
	cc2500_flush_tx(dev);
	cc2500_idle(dev);

	cc2500_cmd_register(dev, SFSTXON);

	return 0;
}

#define CC2500_SPI_MODE SPI_WORD_SET(8)

#define CC2500_DEFINE(i)                                               \
	static const struct cc2500_config cc2500_config_##i = {          \
		.spi = SPI_DT_SPEC_INST_GET(i, CC2500_SPI_MODE, 2),            \
		.array = DT_INST_PROP_OR(i, conf_array, {}), \
		.array_len = DT_INST_PROP_LEN_OR(i, conf_array, 0), \
		.gpio2_as_irq = DT_INST_PROP_OR(i, use_gpio_2_as_irq, false),      \
		IF_ENABLED(CONFIG_CC2500_TRIGGER,                              \
			(.irq = GPIO_DT_SPEC_INST_GET(i, irq_gpios),))           \
	};                                                                            \
                                                                                  \
	static struct cc2500_data cc2500_##i = {                                  \
		.channel_frequency = DT_INST_PROP(i, channel_frequency),                    \
		.start_frequency = DT_INST_PROP(i, start_frequency),                    \
		.modulation_format = DT_INST_PROP(i, modulation_format),            \
		.payload_crc = DT_INST_PROP_OR(i, payload_crc, true)                    \
	};                                                                            \
	DEVICE_DT_INST_DEFINE(i, cc2500_init, NULL, &cc2500_##i,  \
			      &cc2500_config_##i, POST_KERNEL,              \
			      CONFIG_CC2500_INIT_PRIORITY, &cc2500_api);

DT_INST_FOREACH_STATUS_OKAY(CC2500_DEFINE)
