/*
 * Copyright (C) 2024 Eve Redero
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>

#include <app/drivers/propy_radio.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);
#if !DT_NODE_EXISTS(DT_NODELABEL(radio0))
#error "whoops, node label radio0 not found"
#endif

int select_mode(const struct device *cc2500, uint8_t mode, uint8_t volume)
{
	uint8_t data_len = 9;
	uint8_t buffer[] = {0x01, 0x00, 0xa5, 0x28, 0x28, 0x00, 0x00, mode, volume};
	int i = 0;
	static uint8_t accel = 0x00;
	buffer[4] = accel;
	buffer[3] = accel;
	accel++;

	for (i=0; i<20; i++) {
		propy_radio_write(cc2500, buffer, data_len-2);
		k_sleep(K_MSEC(50));
		propy_radio_write(cc2500, buffer, data_len);
		k_sleep(K_MSEC(50));
	}
	return(0);
}
int main(void)
{
	static const struct device *cc2500 = DEVICE_DT_GET(DT_NODELABEL(radio0));
	int i;

	if (!device_is_ready(cc2500)) {
		LOG_ERR("Sensor not ready");
		return 0;
	}
	LOG_INF("Device ready");
	for (i=0; i<20; i++) {
		LOG_INF("Mode 5, 1");
		select_mode(cc2500, 5, 1);
		LOG_INF("Mode 5, 0");
		select_mode(cc2500, 5, 0);
	}
	for (i=0; i<20; i++) {
		LOG_INF("Mode 5 level 1");
		select_mode(cc2500, 4, 1);
		LOG_INF("Mode 5 level 0");
		select_mode(cc2500, 4, 0);
	}
	for (i=0; i<20; i++) {
		LOG_INF("Mode 1");
		select_mode(cc2500, 1, 6);
	}
	LOG_INF("Vol 0");
	select_mode(cc2500, 0, 0);

	return 0;
}

