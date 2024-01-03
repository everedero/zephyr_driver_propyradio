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

//#define IS_TX

int main(void)
{
	static const struct device *nrf24 = DEVICE_DT_GET(DT_NODELABEL(radio0));
	uint8_t data_len = 8;
	uint8_t buffer[] = {0xab, 0xcd, 0xef, 0xaa, 0xbb, 0xcc, 0xdd, 0xee};

	if (!device_is_ready(nrf24)) {
		LOG_ERR("Sensor not ready");
		return 0;
	}
	LOG_INF("Device ready");
#ifdef IS_TX
	LOG_INF("I am TX!");
#else
	LOG_INF("I am RX");
#endif

#ifdef IS_TX
	while (true) {
		nrf24_write(nrf24, buffer, data_len);
		LOG_INF("Wrote stuff");
		k_sleep(K_MSEC(1000));
	}
#else
	while (true) {
		nrf24_read(nrf24, buffer, data_len);
		k_sleep(K_MSEC(10));
		LOG_HEXDUMP_INF(buffer, data_len, "Received: ");
	}
#endif
	return 0;
}

