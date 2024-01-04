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
#endif // CONFIG_NRF24L01_TRIGGER

	if (!device_is_ready(nrf24)) {
		LOG_ERR("Sensor not ready");
		return 0;
	}
	LOG_INF("Device ready");

#ifdef ALICE
	printk("I am Alice!\n");
	while (true) {
		strncpy(buffer, "I am Alice, hi!", 16);
#ifdef TRIGGER
		printk("Trigger mode!\n");
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
#endif // CONFIG_NRF24L01_TRIGGER
		printk("Switch to read");
#ifdef TRIGGER
		while (nrf24_read(nrf24, buffer, data_len));
#else
		nrf24_read(nrf24, buffer, data_len);
#endif // CONFIG_NRF24L01_TRIGGER
		LOG_HEXDUMP_INF(buffer, data_len, "Received: ");
		k_sleep(K_MSEC(1000));
		printk("Switch to write\n");
	}
#endif // ALICE

#ifdef BOB
	printk("I am Bob!\n");
	while (true) {
#ifdef TRIGGER
		while (nrf24_read(nrf24, buffer, data_len));
#else
		nrf24_read(nrf24, buffer, data_len);
#endif // CONFIG_NRF24L01_TRIGGER
		LOG_HEXDUMP_INF(buffer, data_len, "Received: ");
		printk("Switch to write\n");
		strncpy(buffer, "Hi Alice Im Bob", 16);
		k_sleep(K_MSEC(1000));
#ifdef TRIGGER
		printk("Trigger mode!\n");
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
#endif // CONFIG_NRF24L01_TRIGGER
	}
#endif // BOB
	return 0;
}

