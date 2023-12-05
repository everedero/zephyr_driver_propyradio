/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
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

int main(void)
{
	static const struct device *nrf24 = DEVICE_DT_GET(DT_NODELABEL(radio0));

	if (!device_is_ready(nrf24)) {
		LOG_ERR("Sensor not ready");
		return 0;
	}
	return 0;
}

