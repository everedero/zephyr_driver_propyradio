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

/* Remote sends a packet every ~58 ms, alternating short (7 B) and long (9 B) */
#define PKT_GAP_MS 54
/* Tilt value of a remote lying flat */
#define TILT_ZERO  0x28

/* Mimic the original remote for duration_ms (see docs/sniff_2026-11-17_*.log) */
static void hold(const struct device *cc2500, uint8_t mode, uint8_t strength, int duration_ms)
{
	uint8_t buf[] = {0x01, 0x00, 0xA5, TILT_ZERO, TILT_ZERO, 0x00, 0x00, mode, strength};
	int64_t end = k_uptime_get() + duration_ms;

	LOG_INF("mode %u strength %u for %d ms", mode, strength, duration_ms);
	while (k_uptime_get() < end) {
		propy_radio_write(cc2500, buf, sizeof(buf) - 2);
		k_msleep(PKT_GAP_MS);
		propy_radio_write(cc2500, buf, sizeof(buf));
		k_msleep(PKT_GAP_MS);
	}
}

int main(void)
{
	static const struct device *cc2500 = DEVICE_DT_GET(DT_NODELABEL(radio0));

	if (!device_is_ready(cc2500)) {
		LOG_ERR("Radio not ready");
		return 0;
	}
	LOG_INF("Device ready");

	/* Remote start-up: idle packets before any mode change */
	hold(cc2500, 0, 5, 10000);
	hold(cc2500, 1, 5, 10000);
	hold(cc2500, 3, 9, 10000);
	hold(cc2500, 3, 5, 10000);
	hold(cc2500, 0, 5, 5000);
	LOG_INF("Done");

	return 0;
}
