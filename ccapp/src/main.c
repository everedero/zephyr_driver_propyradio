/*
 * Copyright (C) 2024 Eve Redero
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>

#include <app/drivers/propy_radio.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>
LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);
#if !DT_NODE_EXISTS(DT_NODELABEL(radio0))
#error "whoops, node label radio0 not found"
#endif

/* Remote sends a packet every ~58 ms, alternating short (7 B) and long (9 B) */
#define PKT_GAP_MS 54
/* Remote strength range (- / + buttons), level = 8 * S */
#define S_MIN      5
#define S_MAX      16
#define S_BOOT     5
#define FLOOR      (8 * S_MIN)
/* Tilt/motion test pattern (no accelerometer here): ramp 0 -> 8*S, then rest at 0 */
#define TEST_RAMP_PKTS 52 /* ~3 s */
#define TEST_REST_PKTS 17 /* ~1 s */

/*
 * Patterns measured on the remote, see spi_cc2500_protocol.md and
 * docs/sniff_2026-11-20_*.log. One step = one packet (58 ms).
 */
enum { IDLE, TILT, MOTION, STEADY, SLOW, FAST, RAMP, WAVE, RANDOM };

static const struct {
	const char *name;
	uint8_t byte; /* mode byte on air (device ignores it) */
	const char *help;
} modes[] = {
	[IDLE]   = {"idle", 0, "SenseMotion 1 at rest (boot/pairing): 0x28"},
	[TILT]   = {"tilt", 0, "SenseMotion 1, test pattern: ramp 0 -> 8*S, rest"},
	[MOTION] = {"motion", 1, "SenseMotion 2, test pattern: ramp 0 -> 8*S, rest"},
	[STEADY] = {"steady", 2, "Pattern 1: constant 8*S"},
	[SLOW]   = {"slow", 3, "Pattern 2: slow pulse, 0.4 s off / 0.4 s on"},
	[FAST]   = {"fast", 4, "Pattern 3: fast pulse, 0.17 s on / 0.23 s off"},
	[RAMP]   = {"ramp", 5, "Pattern 4: sawtooth 0x28 -> 8*S, hold 0.5 s"},
	[WAVE]   = {"wave", 6, "Pattern 5: triangle 0x28 <-> 8*S"},
	[RANDOM] = {"random", 7, "Pattern 6: random 0.4 s steps, ignores power"},
};

/* Pattern 6 levels, one per 7 packets, repeats after 127 steps (7-bit LFSR period) */
static const uint8_t random_levels[127] = {
	0x60, 0x50, 0x30, 0x70, 0x60, 0x50, 0x30, 0x60, 0x50, 0x30, 0x70, 0x70, 0x60, 0x50, 0x48, 0x40,
	0x28, 0x38, 0x48, 0x50, 0x30, 0x60, 0x40, 0x38, 0x48, 0x50, 0x30, 0x70, 0x70, 0x70, 0x60, 0x40,
	0x28, 0x38, 0x48, 0x40, 0x28, 0x28, 0x28, 0x28, 0x38, 0x30, 0x60, 0x40, 0x38, 0x30, 0x60, 0x50,
	0x30, 0x60, 0x40, 0x28, 0x38, 0x30, 0x70, 0x60, 0x40, 0x38, 0x30, 0x70, 0x60, 0x50, 0x48, 0x50,
	0x30, 0x70, 0x60, 0x40, 0x28, 0x28, 0x38, 0x48, 0x40, 0x38, 0x30, 0x60, 0x40, 0x28, 0x28, 0x28,
	0x38, 0x48, 0x50, 0x48, 0x50, 0x48, 0x50, 0x30, 0x60, 0x50, 0x48, 0x40, 0x38, 0x48, 0x40, 0x38,
	0x48, 0x50, 0x48, 0x40, 0x38, 0x30, 0x70, 0x70, 0x60, 0x40, 0x38, 0x48, 0x40, 0x28, 0x38, 0x30,
	0x60, 0x50, 0x48, 0x50, 0x48, 0x40, 0x28, 0x28, 0x38, 0x30, 0x70, 0x70, 0x70, 0x70, 0x70,
};

/* Motor level of mode m at strength s, t packets after the mode was entered */
static uint8_t level_at(int m, uint32_t s, uint32_t t)
{
	uint32_t top = 8 * s;
	uint32_t n = (top - FLOOR) / 8; /* 8-unit steps from FLOOR to top */
	uint32_t i;

	switch (m) {
	case IDLE:
		return FLOOR;
	case TILT:
	case MOTION:
		t %= TEST_RAMP_PKTS + TEST_REST_PKTS;
		return t < TEST_RAMP_PKTS ? top * t / (TEST_RAMP_PKTS - 1) : 0;
	case STEADY:
		return top;
	case SLOW:
		return t % 14 < 7 ? 0 : top;
	case FAST:
		return t % 7 < 3 ? top : 0;
	case RAMP:
		t %= 2 * n + 9;
		return t < 2 * n ? FLOOR + 8 * (t / 2) : top;
	case WAVE:
		/* down then up, one step every 8/3 packets, ends held twice as long */
		i = (t * 3 / 8) % (2 * n + 2);
		return top - 8 * MIN(i, 2 * n + 1 - i);
	case RANDOM:
		return random_levels[(t / 7) % ARRAY_SIZE(random_levels)];
	default:
		return 0;
	}
}

static atomic_t mode = ATOMIC_INIT(IDLE);
static atomic_t strength = ATOMIC_INIT(S_BOOT);
static atomic_t stopped;

static int cmd_power(const struct shell *sh, size_t argc, char **argv)
{
	int err = 0;
	unsigned long s = shell_strtoul(argv[1], 0, &err);

	if (err || (s != 0 && (s < S_MIN || s > S_MAX))) {
		shell_error(sh, "expected %d..%d like the remote, or 0 to stop", S_MIN, S_MAX);
		return -EINVAL;
	}
	atomic_set(&stopped, s == 0);
	if (s) {
		atomic_set(&strength, s);
	}
	return 0;
}

static int cmd_mode(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1) {
		for (size_t i = 0; i < ARRAY_SIZE(modes); i++) {
			shell_print(sh, "%-7s %s", modes[i].name, modes[i].help);
		}
		return 0;
	}
	for (size_t i = 0; i < ARRAY_SIZE(modes); i++) {
		if (strcmp(argv[1], modes[i].name) == 0) {
			atomic_set(&mode, i);
			return 0;
		}
	}
	shell_error(sh, "unknown mode %s", argv[1]);
	return -EINVAL;
}

static void mode_get(size_t idx, struct shell_static_entry *entry)
{
	entry->syntax = idx < ARRAY_SIZE(modes) ? modes[idx].name : NULL;
	entry->handler = NULL;
	entry->help = idx < ARRAY_SIZE(modes) ? modes[idx].help : NULL;
	entry->subcmd = NULL;
}
SHELL_DYNAMIC_CMD_CREATE(mode_names, mode_get);

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	shell_print(sh, "mode %s power %ld%s", modes[atomic_get(&mode)].name,
		    atomic_get(&strength), atomic_get(&stopped) ? " (stopped)" : "");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(tia_cmds,
	SHELL_CMD_ARG(power, NULL, "Set power 5-16 like the remote (0 stops)", cmd_power, 2, 0),
	SHELL_CMD_ARG(mode, &mode_names, "Set mode by name, no name lists them", cmd_mode, 1, 1),
	SHELL_CMD(status, NULL, "Show current state", cmd_status),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(tia, &tia_cmds, "Control the remote-controlled device", NULL);

int main(void)
{
	static const struct device *cc2500 = DEVICE_DT_GET(DT_NODELABEL(radio0));
	int cur = IDLE;
	uint32_t t = 0;

	if (!device_is_ready(cc2500)) {
		LOG_ERR("Radio not ready");
		return 0;
	}
	LOG_INF("Device ready");

	/* One packet per step like the remote, alternating short/long; boot = pairing idle */
	for (bool lng = false;; lng = !lng, t++) {
		int m = atomic_get(&mode);
		uint8_t s = atomic_get(&strength);

		if (m != cur) {
			cur = m;
			t = 0;
		}
		uint8_t l = atomic_get(&stopped) ? 0 : level_at(m, s, t);
		uint8_t buf[] = {0x01, 0x00, 0xA5, l, l, 0x00, 0x00, modes[m].byte, s};

		propy_radio_write(cc2500, buf, lng ? sizeof(buf) : sizeof(buf) - 2);
		k_msleep(PKT_GAP_MS);
	}
	return 0;
}
