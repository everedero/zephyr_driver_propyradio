/*
 * Copyright (C) 2024 Eve Redero
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>

#include <app/drivers/nrf24.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);
#if !DT_NODE_EXISTS(DT_NODELABEL(radio0))
#error "whoops, node label radio0 not found"
#endif

#ifdef CONFIG_NRF24L01_TRIGGER
#define TRIGGER
#endif

#if CONFIG_ROLE == 0
	#define ALICE
#elif CONFIG_ROLE == 1
	#define BOB
#else
	#define EVE
#endif


int main(void)
{
	static const struct device *nrf24 = DEVICE_DT_GET(DT_NODELABEL(radio0));
	uint8_t data_len = 16;
	uint8_t buffer[16] = {0};
#ifndef TRIGGER
	int i;
#endif // TRIGGER

	if (!device_is_ready(nrf24)) {
		LOG_ERR("Sensor not ready");
		return 0;
	}
	LOG_INF("Device ready");
#ifdef TRIGGER
	LOG_DBG("Trigger mode activated");
#endif //TRIGGER

#ifdef ALICE
	LOG_WRN("I am Alice!");
	while (true) {
		strncpy(buffer, "I am Alice, hi!", 16);
#ifdef TRIGGER
		while (nrf24_write(nrf24, buffer, data_len))
		{
			k_sleep(K_MSEC(10));
		}
#else
		for (i=0; i<10; i++)
		{
			nrf24_write(nrf24, buffer, data_len);
			k_sleep(K_MSEC(10));
		}
#endif // TRIGGER
		LOG_HEXDUMP_INF(buffer, data_len, "Sent: ");
		LOG_DBG("Switch to read");
#ifdef TRIGGER
		while (nrf24_read(nrf24, buffer, data_len));
#else
		nrf24_read(nrf24, buffer, data_len);
#endif // TRIGGER
		LOG_HEXDUMP_INF(buffer, data_len, "Received: ");
		k_sleep(K_MSEC(1000));
		LOG_DBG("Switch to write");
	}
#endif // ALICE

#ifdef BOB
	LOG_WRN("I am Bob!");
	while (true) {
#ifdef TRIGGER
		while (nrf24_read(nrf24, buffer, data_len));
#else
		nrf24_read(nrf24, buffer, data_len);
#endif // TRIGGER
		LOG_HEXDUMP_INF(buffer, data_len, "Received: ");
		LOG_DBG("Switch to write");
		strncpy(buffer, "Hi Alice Im Bob", 16);
		k_sleep(K_MSEC(1000));
#ifdef TRIGGER
		while (nrf24_write(nrf24, buffer, data_len))
		{
			k_sleep(K_MSEC(10));
		}
#else
		for (i=0; i<10; i++)
		{
			nrf24_write(nrf24, buffer, data_len);
			k_sleep(K_MSEC(10));
		}
#endif // TRIGGER
		LOG_HEXDUMP_INF(buffer, data_len, "Sent: ");
		LOG_DBG("Switch to read");
	}
#endif // BOB

#ifdef EVE
	LOG_WRN("I am Eve!");
	while (true) {
#ifdef TRIGGER
		while (nrf24_read(nrf24, buffer, data_len));
#else
		strncpy(buffer, "               ", 16);
		nrf24_read(nrf24, buffer, data_len);
#endif // TRIGGER
		LOG_HEXDUMP_INF(buffer, data_len, "I spied: ");
	}
#endif // EVE

	return 0;
}

