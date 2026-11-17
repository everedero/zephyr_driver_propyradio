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
/* Idle motor level the remote sends right after pairing */
#define LEVEL_IDLE 0x28

/*
 * Bytes 3-4 carry the motor level the remote computed (tilt, pattern...),
 * bytes 7-8 the mode and strength it is in (see docs/sniff_2026-11-17_*.log).
 */
static void send(const struct device *cc2500, uint8_t mode, uint8_t strength, uint8_t level)
{
	uint8_t buf[] = {0x01, 0x00, 0xA5, level, level, 0x00, 0x00, mode, strength};

	propy_radio_write(cc2500, buf, sizeof(buf) - 2);
	k_msleep(PKT_GAP_MS);
	propy_radio_write(cc2500, buf, sizeof(buf));
	k_msleep(PKT_GAP_MS);
}

static void hold(const struct device *cc2500, uint8_t mode, uint8_t strength, uint8_t level,
		 int duration_ms)
{
	int64_t end = k_uptime_get() + duration_ms;

	LOG_INF("mode %u strength %u level 0x%02x for %d ms", mode, strength, level, duration_ms);
	while (k_uptime_get() < end) {
		send(cc2500, mode, strength, level);
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

	/* Remote start-up: idle level before any mode change */
	hold(cc2500, 0, 5, LEVEL_IDLE, 10000);
	/* Level steps, 5 s each */
	hold(cc2500, 3, 9, 0x00, 5000);
	hold(cc2500, 3, 9, 0x48, 5000);
	hold(cc2500, 3, 9, 0x80, 5000);
	hold(cc2500, 3, 9, 0xC0, 5000);
	/* Ramp 0x28 -> 0x80 and back, like the remote's mode 6, three times */
	LOG_INF("ramp");
	for (int n = 0; n < 3; n++) {
		for (int l = 0x28; l <= 0x80; l += 8) {
			send(cc2500, 6, 0x10, l);
		}
		for (int l = 0x80; l >= 0x28; l -= 8) {
			send(cc2500, 6, 0x10, l);
		}
	}
	hold(cc2500, 0, 5, LEVEL_IDLE, 3000);
	LOG_INF("Done");

	return 0;
}
