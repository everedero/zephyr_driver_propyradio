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


int main(void)
{
	static const struct device *cc2500 = DEVICE_DT_GET(DT_NODELABEL(radio0));
	uint8_t data_len = 7;
	uint8_t buffer[] = {0x01, 0x00, 0xa5, 0x28, 0x28, 0x00, 0x00};
	int i = 0;

	if (!device_is_ready(cc2500)) {
		LOG_ERR("Sensor not ready");
		return 0;
	}
	LOG_INF("Device ready");
	for (i=0; i<20; i++) {
		propy_radio_write(cc2500, buffer, data_len);
		k_sleep(K_MSEC(20));
	}
	/*
	while (!propy_radio_write(cc2500, buffer, data_len))
	{
		k_sleep(K_MSEC(10));
	}*/

	return 0;
}

