/*
 * Copyright (c) 2024 Eve Redero
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nordic_nrf24l01

#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

#include <app/drivers/nrf24.h>
#include "nrf24l01_defines.h"

#define SPI_MAX_MSG_LEN 64
#define SPI_MAX_REG_LEN 5
#define SPI_MSG_QUEUE_LEN 10

LOG_MODULE_REGISTER(nrf24l01, CONFIG_NRF24L01_LOG_LEVEL);

struct nrf24l01_config {
	const struct spi_dt_spec spi;
	struct gpio_dt_spec ce;
	struct gpio_dt_spec irq;
};

struct nrf24l01_data {
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
#ifdef CONFIG_NRF24L01_TRIGGER
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
#endif /* CONFIG_NRF24L01_TRIGGER */
};

/* Private core communication functions */
uint8_t nrf24l01_write_register(const struct device *dev, uint8_t reg, uint8_t data)
{
	/* Register config can only be done in power down or standby */
	const struct nrf24l01_config *config = dev->config;
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
	tx_data[0] = ( W_REGISTER | ( REGISTER_MASK & reg ) );
	tx_data[1] = data;

	ret = spi_transceive_dt(&config->spi, &tx, &rx);
	if (ret) {
		LOG_ERR("Error transceive %d", ret);
		return 0;
	}
	return rx_data;
}

uint8_t nrf24l01_read_register(const struct device *dev, uint8_t reg)
{
	const struct nrf24l01_config *config = dev->config;
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
	tx_data[0] = ( R_REGISTER | ( REGISTER_MASK & reg ) );
	tx_data[1] = RF24_NOP;

	ret = spi_transceive_dt(&config->spi, &tx, &rx);

	if (ret) {
		LOG_ERR("Error transceive %d", ret);
		return 0;
	}

	// status is 1st byte of receive buffer
	return rx_data[1];
}

uint8_t nrf24l01_cmd_register(const struct device *dev, uint8_t cmd)
{
	const struct nrf24l01_config *config = dev->config;
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

uint8_t nrf24l01_cmd_register_with_arg(const struct device *dev, uint8_t cmd, uint8_t arg)
{
	const struct nrf24l01_config *config = dev->config;
	int ret;
	uint8_t tx_data[2];
	uint8_t rx_data[1];
	const struct spi_buf tx_buf[1] = {
		{
			.buf = tx_data,
			.len = 2
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
	tx_data[1] = arg;

	ret = spi_transceive_dt(&config->spi, &tx, &rx);
	if (ret) {
		LOG_ERR("Error transceive %d\n", ret);
		return 0;
	}

	return rx_data[0];
}

uint8_t nrf24l01_activate(const struct device *dev)
{
	return(nrf24l01_cmd_register_with_arg(dev, ACTIVATE, 0x73));
}

/*
 * Clear all 3 IRQ in one SPI command
 */
uint8_t nrf24l01_clear_irq(const struct device *dev)
{
	uint8_t reg;
	reg = nrf24l01_read_register(dev, NRF_STATUS);
#if CLEARED
	reg = reg | (BIT(RX_DR) | BIT(TX_DS) | BIT(MAX_RT));
#else
	reg = reg & ~(BIT(RX_DR) | BIT(TX_DS) | BIT(MAX_RT));
#endif
	return(nrf24l01_write_register(dev, NRF_STATUS, reg));
}

uint8_t nrf24l01_set_register_bit(const struct device *dev, uint8_t reg, uint8_t bit, bool val)
{
	uint8_t reg_data, reg_read;
	reg_read = nrf24l01_read_register(dev, reg);
	if (val) // Setting bit to 1
	{
		reg_data = reg_read | BIT(bit);
	}
	else // Setting bit to 0
	{
		reg_data = reg_read & (~BIT(bit));
	}
	// Only write if value is different
	if (reg_read != reg_data)
	{
		reg_data = nrf24l01_write_register(dev, reg, reg_data);
	}
	return(reg_data);
}

bool nrf24l01_get_register_bit(const struct device *dev, uint8_t reg, uint8_t bit)
{
	uint8_t reg_data;
	bool res;
	reg_data = nrf24l01_read_register(dev, reg);
	res = (bool)((reg_data & BIT(bit)) >> bit);
	return(res);
}

uint8_t nrf24l01_write_register_len(const struct device *dev, uint8_t reg, const uint8_t* data, uint8_t len)
{
	const struct nrf24l01_config *config = dev->config;
	uint8_t tx_data[SPI_MAX_REG_LEN + 1] = {0};
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

	tx_data[0] = ( W_REGISTER | ( REGISTER_MASK & reg ) );
	memcpy(&tx_data[1], data, len);

	ret = spi_transceive_dt(&config->spi, &tx, &rx);
	if (ret) {
		LOG_ERR("Error transceive %d", ret);
		return 0;
	}
	return rx_data;
}

int nrf24l01_toggle_ce(const struct device *dev, bool level)
{
	const struct nrf24l01_config *config = dev->config;
	int ret;
	ret = gpio_pin_set_dt(&config->ce, level);
	if (ret < 0) {
		LOG_ERR("Could not toggle CE (%d)", ret);
		return ret;
	}
	return 0;
}

#ifdef CONFIG_NRF24L01_TRIGGER
bool nrf24l01_read_irq(const struct device *dev)
{
	const struct nrf24l01_config *config = dev->config;
	int ret;
	ret = gpio_pin_get_dt(&config->irq);
	return((bool)ret);
}
#endif

/* Private configuration functions */
uint8_t nrf24l01_set_channel(const struct device *dev)
{
	struct nrf24l01_data *data = dev->data;
	const uint8_t max_channel = 125;
	return nrf24l01_write_register(dev, RF_CH, MIN(data->channel_frequency, max_channel));
}

uint8_t nrf24l01_set_config(const struct device *dev)
{
	uint8_t val;
	struct nrf24l01_data *data = dev->data;
	nrf24l01_write_register(dev, SETUP_AW, (data->addr_width - 2) & 0x3);
	nrf24l01_set_register_bit(dev, RF_SETUP, RF_DR, data->data_rate_2mbps);
	switch (data->rf_power_attenuation)
	{
		case 18:
			val = 0b00;
		break;
		case 12:
			val = 0b01;
		break;
		case 6:
			val = 0b10;
		break;
		case 0:
			val = 0b11;
		break;
		default:
			val = 0b00;
			LOG_ERR("Invalid RF power value (%d)", data->rf_power_attenuation);

	}
	nrf24l01_set_register_bit(dev, RF_SETUP, RF_PWR_LOW, val & 0b01);
	nrf24l01_set_register_bit(dev, RF_SETUP, RF_PWR_HIGH, val & 0b10);

	nrf24l01_set_register_bit(dev, RF_SETUP, LNA_HCURR, data->lna_gain);

	// CRC configuration
	nrf24l01_set_register_bit(dev, NRF_CONFIG, EN_CRC, data->payload_crc);
	nrf24l01_set_register_bit(dev, NRF_CONFIG, CRCO, data->crc_encoding_twobytes);

	// Unmask interrupts
	nrf24l01_set_register_bit(dev, NRF_CONFIG, MASK_RX_DR, false);
	nrf24l01_set_register_bit(dev, NRF_CONFIG, MASK_TX_DS, false);
	nrf24l01_set_register_bit(dev, NRF_CONFIG, MASK_MAX_RT, false);

	return 0;
}

uint8_t nrf24l01_get_channel(const struct device *dev)
{
	return nrf24l01_read_register(dev, RF_CH);
}

uint8_t nrf24l01_get_config(const struct device *dev)
{
	return nrf24l01_read_register(dev, NRF_CONFIG);
}

uint8_t nrf24l01_write_payload_core(const struct device *dev, const void* buf, uint8_t data_len, const uint8_t write_type)
{
	int ret;
	/* Can be TX or RX ACK*/
	struct nrf24l01_data *data = dev->data;
	const struct nrf24l01_config *config = dev->config;
	uint8_t blank_len = data->dynamic_payload ? 0 : data->payload_fixed_size - data_len;
	uint8_t size;
	size = data_len + blank_len + 1 ; // Add register value to transmit buffer
	uint8_t tx_data[SPI_MAX_MSG_LEN + 1] = {0};
	uint8_t rx_data[1];
	const struct spi_buf tx_buf[1] = {
		{
			.buf = tx_data,
			.len = size
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

	tx_data[0] = write_type;
	memcpy(&tx_data[1], buf, data_len);

	ret = spi_transceive_dt(&config->spi, &tx, &rx);
	if (ret) {
		LOG_ERR("Error transceive %d\n", ret);
		return 0;
	}
	return rx_data[0];
}

uint8_t nrf24l01_write_ack_payload(const struct device *dev, const void* buf, uint8_t data_len, uint8_t pipe)
{
	uint8_t ret;
	/* In RX mode, write ACK packet */
	ret = nrf24l01_write_payload_core(dev, buf, data_len, W_ACK_PAYLOAD | ( pipe & 0x07 ));
	return ret;
}

uint8_t nrf24l01_write_tx_payload(const struct device *dev, const void* buf, uint8_t data_len)
{
	uint8_t ret, cmd;
	struct nrf24l01_data *data = dev->data;
	cmd = data->payload_ack ? W_TX_PAYLOAD : W_TX_PAYLOAD_NO_ACK;
	/* Write a TX payload to send*/
	ret = nrf24l01_write_payload_core(dev, buf, data_len, cmd);

	return ret;
}

uint8_t nrf24l01_read_payload(const struct device *dev, void* buf, uint8_t data_len)
{
	/* In RX mode only*/
	struct nrf24l01_data *data = dev->data;
	const struct nrf24l01_config *config = dev->config;
	int ret;
	uint8_t size;
	uint8_t blank_len = data->dynamic_payload ? 0 : data->payload_fixed_size - data_len;
	size = data_len + blank_len +  1 ; // Add register value to transmit buffer
	uint8_t tx_data[SPI_MAX_MSG_LEN + 1] = {0};
	uint8_t rx_data[SPI_MAX_MSG_LEN + 1] = {0};
	const struct spi_buf tx_buf[1] = {
		{
			.buf = tx_data,
			.len = size
		}
	};
	const struct spi_buf rx_buf[1] = {
		{
			.buf = rx_data,
			.len = size
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

	tx_data[0] = R_RX_PAYLOAD;
	memset(&tx_data[1], RF24_NOP, data_len);

	ret = spi_transceive_dt(&config->spi, &tx, &rx);
	if (ret) {
		LOG_ERR("Error transceive %d\n", ret);
		return 0;
	}
	memcpy(buf, &rx_data[1], data_len);

	// Clear interrupt flags
	nrf24l01_clear_irq(dev);

	return rx_data[0];
}

void nrf24l01_toggle_reading_pipe(const struct device *dev, uint8_t rx_pipe, bool activate)
{
	nrf24l01_set_register_bit(dev, EN_RXADDR, child_pipe_enable_bit[rx_pipe], activate);
}

void nrf24l01_configure_pipes(const struct device *dev)
{
	struct nrf24l01_data *data = dev->data;
	int idx;
	// Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
	// expects it LSB first too, so we're good.
	if (data->addr_width != 4) {
		LOG_ERR("Width should be 4");
		return;
	}
	// Writing pipe addresses
	nrf24l01_write_register_len(dev, RX_ADDR_P0, data->rx_datapipe0_address, data->addr_width);
	nrf24l01_write_register_len(dev, RX_ADDR_P1, data->rx_datapipe1_address, data->addr_width);
	nrf24l01_write_register_len(dev, TX_ADDR, data->tx_address, data->addr_width);

	for (idx=0; idx<4; idx++)
	{
		nrf24l01_write_register(dev, RX_ADDR_P2+idx, data->rx_child_datapipes_addresses[idx]);
	}

	// Writing pipes lengths
	for (idx=0; idx<6; idx++)
	{
		nrf24l01_write_register(dev, RX_PW_P0+idx, data->payload_fixed_size);
	}
	// Enabling the N first RX pipes
	for (idx=0; idx<data->rx_datapipes_number; idx++)
	{
		nrf24l01_toggle_reading_pipe(dev, idx, true);
	}
}

void nrf24l01_configure_ack(const struct device *dev)
{
	struct nrf24l01_data *data = dev->data;
	int idx;

	for (idx=0; idx<data->rx_datapipes_number; idx++)
	{
		nrf24l01_set_register_bit(dev, DYNPD, DPL_P0+idx, data->rx_datapipes_dynamic_payload[idx]);
	}
	nrf24l01_set_register_bit(dev, FEATURE, EN_DPL, data->dynamic_payload);
	if (data->payload_ack) {
		// Auto retransmit with 3 attempts
		nrf24l01_write_register(dev, SETUP_RETR, 0x23);
	} else {
		// Disable auto retransmit
		nrf24l01_write_register(dev, SETUP_RETR, 0x00);
	}
	// TODO Verify, but this is automatically set
	if (data->payload_ack)
	{
		// Auto ack on enabled pipes
		nrf24l01_write_register(dev, EN_AA,
				GENMASK(data->rx_datapipes_number-1, 0));
	} else {
		nrf24l01_write_register(dev, EN_AA, 0x00);
	}
	if (data->payload_ack)
	{
		if (memcmp(data->rx_datapipe0_address, data->tx_address, data->addr_width)) {
			LOG_WRN("RX datapipe 0 and TX address are different, use this config only for RX mode");
		}
	}
}

bool nrf24l01_is_rx_data_available(const struct device *dev, uint8_t* pipe_num)
{
	/* Set pipe_num to 0 to ignore the pipe number retrieval */
	uint8_t status;
	if (! nrf24l01_get_register_bit(dev, FIFO_STATUS, RX_EMPTY))
	{
		// If the caller wants the pipe number, include it
		if ( pipe_num ){
			status = nrf24l01_read_register(dev, NRF_STATUS);
			*pipe_num = ( status >> RX_P_NO ) & 0x07;
		}
		return true;
	}
	return false;
}

void nrf24l01_radio_power_up(const struct device *dev)
{
	 // if not powered up then power up and wait for the radio to initialize
	if (!nrf24l01_get_register_bit(dev, NRF_CONFIG, PWR_UP))
	{
		nrf24l01_set_register_bit(dev, NRF_CONFIG, PWR_UP, true);

		// For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
		// There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
		// the CE is set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
		k_sleep(K_MSEC(5));
	}
}

void nrf24l01_radio_power_down(const struct device *dev)
{
	nrf24l01_toggle_ce(dev, LOW); // Guarantee CE is low on powerDown
	nrf24l01_set_register_bit(dev, NRF_CONFIG, PWR_UP, false);
}

void nrf24l01_start_listening(const struct device *dev)
{
	struct nrf24l01_data *data = dev->data;
	if (data->is_listening) {
		return;
	}
	data->is_listening = true;
	nrf24l01_set_register_bit(dev, NRF_CONFIG, PRIM_RX, true);
	// Set CE high for RX
	nrf24l01_toggle_ce(dev, HIGH);
	// In listening mode, rx datapipe has payload size
	nrf24l01_write_register(dev, RX_PW_P0, data->payload_fixed_size);
	// Set CE high for RX
	nrf24l01_toggle_ce(dev, HIGH);
	nrf24l01_radio_power_up(dev);
}

void nrf24l01_stop_listening(const struct device *dev)
{
	struct nrf24l01_data *data = dev->data;
	const uint8_t tx_delay=250;
	if (!data->is_listening) {
		return;
	}
	// In non-listening mode, rx datapipe has payload size 2 for ACK
	nrf24l01_write_register(dev, RX_PW_P0, 2);
	// Set CE low
	nrf24l01_toggle_ce(dev, LOW);

	k_sleep(K_MSEC(tx_delay));

	nrf24l01_set_register_bit(dev, NRF_CONFIG, PRIM_RX, false);

	nrf24l01_toggle_reading_pipe(dev, 0, true);
	data->is_listening = false;
	nrf24l01_radio_power_up(dev);
	nrf24l01_clear_irq(dev);
}

bool nrf24l01_test_spi(const struct device *dev)
{
	uint8_t ret = 0;
	nrf24l01_write_register(dev, RX_PW_P5, 0x0E);
	ret = nrf24l01_read_register(dev, RX_PW_P5);
	nrf24l01_write_register(dev, RX_PW_P5, 0x00);
	if (ret != 0x0E) {
		return(false);
	}
	return(true);
}
#ifndef CONFIG_NRF24L01_TRIGGER
// not CONFIG_NRF24L01_TRIGGER
static int nrf24l01_read_polling(const struct device *dev, uint8_t *buffer, uint8_t data_len)
{
	uint8_t pipe_num = 0;

	while (!nrf24l01_is_rx_data_available(dev, &pipe_num))
	{
		k_msleep(1);
	}
	nrf24l01_read_payload(dev, buffer, data_len);
	return 0;
}
#endif // not CONFIG_NRF24L01_TRIGGER

/* API functions */

static int nrf24l01_read(const struct device *dev, uint8_t *buffer, uint8_t data_len)
{
	LOG_DBG("Read RX");
	nrf24l01_start_listening(dev);
#ifdef CONFIG_NRF24L01_TRIGGER
	struct nrf24l01_data *data = dev->data;
	uint8_t buffer_full[SPI_MAX_MSG_LEN] = {0};

	if (k_msgq_get(&data->rx_queue, buffer_full, K_MSEC(CONFIG_NRF24L01_READ_TIMEOUT)) < 0) {
		LOG_INF("Nothing in RX queue");
		return(-EIO);
	}
	memcpy(buffer, buffer_full, data_len);
#else // not CONFIG_NRF24L01_TRIGGER
	nrf24l01_read_polling(dev, buffer, data_len);
#endif // CONFIG_NRF24L01_TRIGGER

	return 0;
}

static int nrf24l01_write(const struct device *dev, uint8_t *buffer, uint8_t data_len)
{
	int ret = 0;
#ifdef CONFIG_NRF24L01_TRIGGER
	struct nrf24l01_data *data = dev->data;
#else // not CONFIG_NRF24L01_TRIGGER
	uint8_t status;
#endif // CONFIG_NRF24L01_TRIGGER
	LOG_DBG("Send TX");
	nrf24l01_stop_listening(dev);

	nrf24l01_write_tx_payload(dev, buffer, data_len);
	nrf24l01_toggle_ce(dev, HIGH);
	// 10 us pulse
	k_usleep(10);
	nrf24l01_toggle_ce(dev, LOW);
#ifdef CONFIG_NRF24L01_TRIGGER
	if (k_sem_take(&data->sem, K_MSEC(CONFIG_NRF24L01_WRITE_TIMEOUT)) != 0) {
		LOG_ERR("TX sending timed out");
		nrf24l01_toggle_ce(dev, LOW);
		return -ETIME;
	}
	ret = (int)data->write_ret_code;
#else // not CONFIG_NRF24L01_TRIGGER
	// Wait for status bits TX_DS or MAX_RT to be asserted
	while( !(nrf24l01_get_register_bit(dev, NRF_STATUS, TX_DS) |
		nrf24l01_get_register_bit(dev, NRF_STATUS, MAX_RT)))
	{
		k_usleep(10);
	}
	// If max retries exceeded, flush TX
	status = nrf24l01_read_register(dev, NRF_STATUS);
	if ( status & BIT(MAX_RT) ) {
		nrf24l01_cmd_register(dev, FLUSH_TX);
		nrf24l01_clear_irq(dev);
		return -EIO;
	}
	nrf24l01_clear_irq(dev);
#endif // CONFIG_NRF24L01_TRIGGER

	return ret;
}

static const struct nrf24_api nrf24l01_api = {
	.read = nrf24l01_read,
	.write = nrf24l01_write,
};

#if !defined(MINIMAL)
static void nrf24l01_info(const struct device *dev)
{
	const struct nrf24l01_config *config = dev->config;
	uint8_t chan;

	LOG_INF("Pin CS number 0x%x", *(&config->spi.config.cs.gpio.pin));
	LOG_INF("Port CS name %s", (**(&config->spi.config.cs.gpio.port)).name);
	LOG_INF("CS delay %d", *(&config->spi.config.cs.delay));
	LOG_INF("Frequency %d Hz", *(&config->spi.config.frequency));
	LOG_INF("Operation 0x%x Hz", *(&config->spi.config.operation));
	LOG_INF("Config register: 0x%x", nrf24l01_get_config(dev));
	chan = nrf24l01_get_channel(dev);
	LOG_INF("Channel selected: %d", chan);
}

void nrf24l01_print_status(uint8_t status)
{
	LOG_INF("STATUS = 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x",
		status,
		(status & BIT(RX_DR))?1:0,
		(status & BIT(TX_DS))?1:0,
		(status & BIT(MAX_RT))?1:0,
		((status >> RX_P_NO) & 0x07),
		(status & BIT(TX_FULL))?1:0
		);
}
#endif

#ifdef CONFIG_NRF24L01_TRIGGER
/* Interrupts */

void irq_callback_handler(const struct device *port,
	struct gpio_callback *cb, uint32_t pins)
{
	struct nrf24l01_data *data = CONTAINER_OF(cb, struct nrf24l01_data, irq_cb);
	const struct nrf24l01_config *config = data->dev->config;
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
	struct nrf24l01_data *data =
		CONTAINER_OF(item, struct nrf24l01_data, trig_work);
	const struct device *dev = data->dev;
	const struct nrf24l01_config *config = dev->config;
	uint8_t ret;
	uint8_t buffer[SPI_MAX_MSG_LEN] = {0};
	uint8_t pipe_num = 0;
	uint8_t size = data->dynamic_payload ? SPI_MAX_MSG_LEN : data->payload_fixed_size;
	ret = nrf24l01_read_register(dev, NRF_STATUS);

    if (ret & BIT(RX_DR))
	{
		LOG_DBG("RX received");
		if (!nrf24l01_is_rx_data_available(dev, &pipe_num))
		{
			LOG_ERR("No data available in RX interrupt");
			nrf24l01_clear_irq(dev);
			ret = gpio_pin_interrupt_configure_dt(&config->irq, GPIO_INT_EDGE_TO_ACTIVE);
			if (ret < 0)
			{
				LOG_ERR("Could not reactivate interrupt");
			}
			return;
		}
		if (data->is_listening)
		{ // Not an ACK interrupt
			nrf24l01_read_payload(dev, buffer, size);
			if (k_msgq_put(&data->rx_queue, buffer, K_NO_WAIT) < 0) {
				LOG_WRN("RX queue full, dropping packet");
			}
		} else {
			LOG_INF("Receive ACK");
		}
	}
	else if (ret & BIT(TX_DS))
	{
		LOG_DBG("TX OK!");
		nrf24l01_toggle_ce(dev, LOW);
		// free semaphore
		data->write_ret_code = 0;
		k_sem_give(&data->sem);

	}
	else if (ret & BIT(MAX_RT))
	{
		// If nobody receives the message, we end up here
		LOG_DBG("TX not acked");
		nrf24l01_toggle_ce(dev, LOW);
		// Max retries exceeded, flush TX
		nrf24l01_cmd_register(dev, FLUSH_TX);
		// Write error code
		data->write_ret_code = MAX_RT;
		// free semaphore
		k_sem_give(&data->sem);
	}
	nrf24l01_clear_irq(dev);
	ret = gpio_pin_interrupt_configure_dt(&config->irq, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0)
	{
		LOG_ERR("Could not reactivate interrupt");
	}
	return;
}
#endif /* CONFIG_NRF24L01_TRIGGER */

/* Init */
static int nrf24l01_init(const struct device *dev)
{
	const struct nrf24l01_config *config = dev->config;
	struct nrf24l01_data *data = dev->data;
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

	if (!nrf24l01_test_spi(dev))
	{
		LOG_ERR("Issue with SPI read/write");
		return -EIO;
	}

	// Configuration
	ret = nrf24l01_set_channel(dev);
    ret = nrf24l01_set_config(dev);

    if (data->payload_fixed_size > 32)
	{
		LOG_ERR("Selected data payload size too big, max 32");
		return -ENOTSUP;
	}

	nrf24l01_configure_pipes(dev);
	nrf24l01_configure_ack(dev);
	// TODO
	if (data->dynamic_payload) {
		LOG_ERR("Dynamic payload not implemented");
		return -ENOSYS;
	}
	nrf24l01_toggle_ce(dev, HIGH);
	k_sleep(K_MSEC(100));

#if !defined(MINIMAL)
	nrf24l01_info(dev);
	// Command
	ret = nrf24l01_cmd_register(dev, RF24_READSTAT);
	nrf24l01_print_status(ret);
#endif

	// Starting chip
	nrf24l01_cmd_register(dev, FLUSH_TX);
	nrf24l01_cmd_register(dev, FLUSH_RX);


	nrf24l01_activate(dev);

#ifdef CONFIG_NRF24L01_TRIGGER
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
	// setup the interrupt function
    gpio_init_callback(&data->irq_cb, irq_callback_handler, BIT(config->irq.pin));

	ret = gpio_add_callback(config->irq.port, &data->irq_cb);
	if (ret < 0) {
		LOG_ERR("Could not configure irq callback (%d)", ret);
		return ret;
	}
	k_work_init(&data->trig_work, work_queue_callback_handler);

#endif /* CONFIG_NRF24L01_TRIGGER */

	// Initialize in non listining mode
	data->is_listening = false;
	nrf24l01_write_register(dev, RX_PW_P0, 2);
	nrf24l01_toggle_ce(dev, LOW);
	const uint8_t tx_delay=250;
	k_sleep(K_MSEC(tx_delay));
	nrf24l01_set_register_bit(dev, NRF_CONFIG, PRIM_RX, false);

	nrf24l01_toggle_reading_pipe(dev, 0, true);
	// Turn radio on
	// TODO Power up in read() and not write(), fix state machine logic
	nrf24l01_radio_power_up(dev);
	// Clear interrupts
	nrf24l01_clear_irq(dev);

	return 0;
}

#define NRF24L01_SPI_MODE SPI_WORD_SET(8)

#define NRF24L01_DEFINE(i)                                               \
	static const struct nrf24l01_config nrf24l01_config_##i = {          \
		.spi = SPI_DT_SPEC_INST_GET(i, NRF24L01_SPI_MODE, 2),            \
		.ce = GPIO_DT_SPEC_INST_GET(i, ce_gpios),                        \
		IF_ENABLED(CONFIG_NRF24L01_TRIGGER,                              \
			(.irq = GPIO_DT_SPEC_INST_GET(i, irq_gpios),))               \
	};                                                                            \
                                                                                  \
	static struct nrf24l01_data nrf24l01_##i = {                                  \
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
	DEVICE_DT_INST_DEFINE(i, nrf24l01_init, NULL, &nrf24l01_##i,  \
			      &nrf24l01_config_##i, POST_KERNEL,              \
			      CONFIG_NRF24L01_INIT_PRIORITY, &nrf24l01_api);

DT_INST_FOREACH_STATUS_OKAY(NRF24L01_DEFINE)
