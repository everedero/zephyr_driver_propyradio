/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nordic_nrf24l01

#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <app/drivers/nrf24.h>
#include "nrf24l01_defines.h"
#include <zephyr/kernel.h>

#define SPI_MAX_MSG_LEN 64
#define SPI_MAX_REG_LEN 5

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
	uint8_t tx_payload_fixed_size;
	bool payload_ack;
	int rx_datapipes_number;
	uint8_t rx_datapipe0_address[5];
	uint8_t rx_datapipe1_address[5];
	uint8_t rx_child_datapipes_addresses[4];
	uint8_t rx_datapipes_dynamic_payload[6];
	uint8_t rx_datapipes_fixed_size_payload[6];
};

/* Core communication functions */
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
			.len = 1//2
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

/* Configuration functions */
uint8_t nrf24l01_set_channel(const struct device *dev)
{
	const struct nrf24l01_data *data = dev->data;
	const uint8_t max_channel = 125;
	return nrf24l01_write_register(dev, RF_CH, MIN(data->channel_frequency, max_channel));
}

uint8_t nrf24l01_set_config(const struct device *dev)
{
	uint8_t val;
	const struct nrf24l01_data *data = dev->data;
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
	nrf24l01_set_register_bit(dev, NRF_CONFIG, EN_CRC, data->payload_ack);
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
	const struct nrf24l01_data *data = dev->data;
	const struct nrf24l01_config *config = dev->config;
	uint8_t blank_len = data->dynamic_payload ? 0 : data->tx_payload_fixed_size - data_len;
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
	const struct nrf24l01_data *data = dev->data;
	cmd = data->payload_ack ? W_TX_PAYLOAD : W_TX_PAYLOAD_NO_ACK;
	/* Write a TX payload to send*/
	ret = nrf24l01_write_payload_core(dev, buf, data_len, cmd);//W_TX_PAYLOAD);
	nrf24l01_toggle_ce(dev, true);
	return ret;
}

uint8_t nrf24l01_read_payload(const struct device *dev, void* buf, uint8_t data_len)
{
	/* In RX mode only*/
	const struct nrf24l01_data *data = dev->data;
	const struct nrf24l01_config *config = dev->config;
	int ret;
	uint8_t size;
	uint8_t blank_len = data->dynamic_payload ? 0 : data->rx_datapipes_fixed_size_payload[0] - data_len;
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

	//if(data_len > payload_size) data_len = payload_size;
	//uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

	//LOG_INF("Reading %u bytes %u blanks", data_len, blank_len);

	tx_data[0] = R_RX_PAYLOAD;
	//tx_data[1] = RF24_NOP;
	memset(&tx_data[1], RF24_NOP, data_len);

	ret = spi_transceive_dt(&config->spi, &tx, &rx);
	if (ret) {
		LOG_ERR("Error transceive %d\n", ret);
		return 0;
	}
	memcpy(buf, &rx_data[1], data_len);

	// Clear interrupt flags
    nrf24l01_set_register_bit(dev, NRF_STATUS, RX_DR, true);
    nrf24l01_set_register_bit(dev, NRF_STATUS, MAX_RT, true);
    nrf24l01_set_register_bit(dev, NRF_STATUS, TX_DS, true);

	return rx_data[0];
}

void nrf24l01_toggle_reading_pipe(const struct device *dev, uint8_t rx_pipe, bool activate)
{
	nrf24l01_set_register_bit(dev, EN_RXADDR, child_pipe_enable_bit[rx_pipe], activate);
}

void nrf24l01_configure_pipes(const struct device *dev)
{
	const struct nrf24l01_data *data = dev->data;
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
		nrf24l01_write_register(dev, RX_PW_P0+idx, data->rx_datapipes_fixed_size_payload[idx]);
	}
	// Enabling the N first RX pipes
	for (idx=0; idx<data->rx_datapipes_number; idx++)
	{
		nrf24l01_toggle_reading_pipe(dev, idx, true);
	}
}

void nrf24l01_configure_ack(const struct device *dev)
{
	const struct nrf24l01_data *data = dev->data;
	int idx;

	for (idx=0; idx<data->rx_datapipes_number; idx++)
	{
		nrf24l01_set_register_bit(dev, DYNPD, DPL_P0+idx, data->rx_datapipes_dynamic_payload[idx]);
	}
	nrf24l01_set_register_bit(dev, FEATURE, EN_ACK_PAY, data->payload_ack);
	nrf24l01_set_register_bit(dev, FEATURE, EN_DPL, data->dynamic_payload);
}
bool nrf24l01_is_rx_data_available(const struct device *dev, uint8_t* pipe_num)
{
	/* Set pipe_num to 0 to ignore the pipe number retrieval */
	uint8_t status;
	// TODO trying to poll interrupt
	/*if (! nrf24l01_get_register_bit(dev, NRF_STATUS, RX_DR)) {
		nrf24l01_set_register_bit(dev, NRF_STATUS, RX_DR, true);
	} else {
		return(false);
	}*/
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
	nrf24l01_radio_power_up(dev);
	nrf24l01_set_register_bit(dev, NRF_CONFIG, PRIM_RX, true);
	nrf24l01_set_register_bit(dev, NRF_CONFIG, RX_DR, true);
	nrf24l01_set_register_bit(dev, NRF_CONFIG, TX_DS, true);
	nrf24l01_set_register_bit(dev, NRF_CONFIG, MAX_RT, true);
	// Set CE high for RX
	nrf24l01_toggle_ce(dev, HIGH);
	// Restore the pipe0 adddress, if exists
	if (nrf24l01_get_register_bit(dev, FEATURE, EN_ACK_PAY))
	{
		nrf24l01_cmd_register(dev, FLUSH_TX);
	}
}
void nrf24l01_stop_listening(const struct device *dev)
{
	const uint8_t tx_delay=250;
	// Set CE low
	nrf24l01_toggle_ce(dev, LOW);

	k_sleep(K_MSEC(tx_delay));

	if (nrf24l01_get_register_bit(dev, FEATURE, EN_ACK_PAY)) {
		k_sleep(K_MSEC(tx_delay));
		nrf24l01_cmd_register(dev, FLUSH_TX);
	}
	nrf24l01_set_register_bit(dev, NRF_CONFIG, PRIM_RX, false);

	// for 3 pins solution TX mode is only left with additonal powerDown/powerUp cycle
//	if (three_pins) {
//		nrf24_powerDown(spi, spi_cfg);
//	}
//	nrf24_powerUp(spi, spi_cfg);

	nrf24l01_toggle_reading_pipe(dev, 0, true);

}

/* API functions */

static int nrf24l01_read(const struct device *dev, uint8_t *buffer, uint8_t data_len)
{
	uint8_t pipe_num = 0;
	static uint8_t got_byte = 0;
	const struct nrf24l01_data *data = dev->data;

	nrf24l01_start_listening(dev);
	while (!nrf24l01_is_rx_data_available(dev, &pipe_num))
	{
		k_msleep(1);
	}
	//LOG_INF("Read found on pipe %d", pipe_num);
	nrf24l01_read_payload(dev, buffer, data_len);
	if (data->payload_ack)
	{
		// Acking
		got_byte += 1;
		nrf24l01_write_ack_payload(dev, &got_byte, 1, pipe_num);
	}
	// Debug
	//nrf24l01_cmd_register(dev, FLUSH_RX);

	return 0;
}

static int nrf24l01_write(const struct device *dev, uint8_t *buffer, uint8_t data_len)
{
	uint8_t status;
	nrf24l01_stop_listening(dev);

	// Like startFastWrite
	nrf24l01_write_tx_payload(dev, buffer, data_len);
	nrf24l01_toggle_ce(dev, HIGH);
	// Wait for status bits TX_DS or MAX_RT to be asserted
	while( !(nrf24l01_get_register_bit(dev, NRF_STATUS, TX_DS) |
		nrf24l01_get_register_bit(dev, NRF_STATUS, MAX_RT)))
	{
		k_msleep(1);
	}
	nrf24l01_toggle_ce(dev, LOW);

	status = nrf24l01_set_register_bit(dev, NRF_STATUS, RX_DR, true);
	nrf24l01_set_register_bit(dev, NRF_STATUS, TX_DS, true);
	nrf24l01_set_register_bit(dev, NRF_STATUS, MAX_RT, true);

	// Max retries exceeded, flush TX
	if ( status & BIT(MAX_RT) ) {
		nrf24l01_cmd_register(dev, FLUSH_TX);
		return -EIO;
	}

	return 0;
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

static int nrf24l01_init(const struct device *dev)
{
	const struct nrf24l01_config *config = dev->config;
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

	if (!gpio_is_ready_dt(&config->irq)) {
		return -EBUSY;
	}
	ret = gpio_pin_configure_dt(&config->irq, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Could not configure IRQ GPIO (%d)", ret);
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

	ret = nrf24l01_read_register(dev, NRF_CONFIG);
	LOG_INF("Config at first: 0x%x", ret);

	// Configuration
	ret = nrf24l01_set_channel(dev);
    ret = nrf24l01_set_config(dev);

	const struct nrf24l01_data *data = dev->data;
    if (data->tx_payload_fixed_size > 32)
	{
		LOG_ERR("Selected data payload size too big, max 32");
		return -ENOTSUP;
	}

	nrf24l01_configure_pipes(dev);
	nrf24l01_configure_ack(dev);
	// TODO
	if (data->dynamic_payload) {
		LOG_ERR("Dynamic payload not implemented");
	}
	nrf24l01_toggle_ce(dev, HIGH);
	k_sleep(K_MSEC(100));

#if !defined(MINIMAL)
	nrf24l01_info(dev);
	// Command
	ret = nrf24l01_cmd_register(dev, RF24_READSTAT);
	nrf24l01_print_status(ret);
#endif
	// Test register write/read
	//ret =  nrf24l01_write_register(dev, EN_AA, 0x01);
	//ret =  nrf24l01_read_register(dev, EN_AA);
	//if (ret != 0x01) {
	//	LOG_ERR("Register not set");
	//}

	// Payload write and read
	/*uint8_t buf[24] = {0};
	uint8_t data_len = 24;
	int i;
	for (i=0; i<data_len; i++) {
		buf[i] = i%0xff;
	}

	ret = nrf24l01_write_tx_payload(dev, buf, data_len);
	LOG_DBG("Write payload: 0x%x", ret);
	ret = nrf24l01_read_payload(dev, buf, data_len);
	LOG_DBG("Read payload: 0x%x", ret);
	LOG_HEXDUMP_DBG(buf, data_len, "Buffer");
	nrf24l01_print_status(ret);*/

	// Starting chip
	nrf24l01_cmd_register(dev, FLUSH_TX);
	nrf24l01_cmd_register(dev, FLUSH_RX);

	// Clear interrupts
	nrf24l01_set_register_bit(dev, NRF_STATUS, RX_DR, true);
	nrf24l01_set_register_bit(dev, NRF_STATUS, MAX_RT, true);
	nrf24l01_set_register_bit(dev, NRF_STATUS, TX_DS, true);

	nrf24l01_activate(dev);

	// Turn radio on
	// TODO Power up in read() and not write(), fix state machine logic
	nrf24l01_radio_power_up(dev);

	return 0;
}

#define NRF24L01_SPI_MODE SPI_WORD_SET(8)

#define NRF24L01_DEFINE(i)                                             \
	static const struct nrf24l01_config nrf24l01_config_##i = {        \
		.spi = SPI_DT_SPEC_INST_GET(i, NRF24L01_SPI_MODE, 2),           \
		.ce = GPIO_DT_SPEC_INST_GET_OR(i, ce_gpios, {}),               \
		.irq = GPIO_DT_SPEC_INST_GET_OR(i, irq_gpios, {}),             \
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
		.tx_payload_fixed_size = DT_INST_PROP_OR(i, tx_payload_fixed_size, 32),            \
		.dynamic_payload = DT_INST_PROP_OR(i, dynamic_payload, false),            \
		.payload_ack = DT_INST_PROP_OR(i, payload_ack, false),                    \
		.rx_datapipes_number = DT_INST_PROP(i, payload_ack),                      \
		.rx_datapipe0_address = DT_INST_PROP(i, rx_datapipe0_address),            \
		.rx_datapipe1_address = DT_INST_PROP(i, rx_datapipe1_address),            \
		.rx_child_datapipes_addresses = {                 \
			DT_INST_PROP(i, rx_datapipe2_address),           \
			DT_INST_PROP(i, rx_datapipe3_address),            \
			DT_INST_PROP(i, rx_datapipe4_address),            \
			DT_INST_PROP(i, rx_datapipe5_address)},            \
		.rx_datapipes_dynamic_payload = DT_INST_PROP_OR(i,                        \
				rx_datapipes_dynamic_payload, {}),                                \
		.rx_datapipes_fixed_size_payload = DT_INST_PROP_OR(i,                     \
				rx_datapipes_fixed_size_payload, {}),                             \
	};                                                                            \
	DEVICE_DT_INST_DEFINE(i, nrf24l01_init, NULL, &nrf24l01_##i,  \
			      &nrf24l01_config_##i, POST_KERNEL,              \
			      CONFIG_NRF24L01_INIT_PRIORITY, &nrf24l01_api);

DT_INST_FOREACH_STATUS_OKAY(NRF24L01_DEFINE)
