/*
 * Copyright (c) 2026 Eve Redero
 * SPDX-License-Identifier: Apache-2.0
 *
 * Raw SPI wiring check for a CC2500 (no driver).
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cc2500_pinout, LOG_LEVEL_INF);

#define READ   0x80
#define BURST  0x40
#define IOCFG0 0x02
#define SYNC0  0x05
#define SRES   0x30
#define SNOP   0x3D
#define PARTNUM 0x30
#define VERSION 0x31

static const struct spi_dt_spec spi = SPI_DT_SPEC_GET(DT_NODELABEL(cc2500), SPI_WORD_SET(8));
static const struct gpio_dt_spec gdo0 = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), gdo0_gpios);

/* Send header (+ optional data byte), return the byte clocked back after the header. */
static int xfer(uint8_t hdr, uint8_t val, size_t len, uint8_t *out)
{
	uint8_t tx[2] = {hdr, val};
	uint8_t rx[2] = {0};
	const struct spi_buf txb = {.buf = tx, .len = len};
	const struct spi_buf rxb = {.buf = rx, .len = len};
	const struct spi_buf_set txs = {.buffers = &txb, .count = 1};
	const struct spi_buf_set rxs = {.buffers = &rxb, .count = 1};
	int ret = spi_transceive_dt(&spi, &txs, &rxs);

	*out = rx[len - 1];
	return ret;
}

static uint8_t strobe(uint8_t cmd) { uint8_t s = 0; xfer(cmd, 0, 1, &s); return s; }
static uint8_t rd(uint8_t a) { uint8_t v = 0; xfer(a | READ, 0, 2, &v); return v; }
static uint8_t rd_status(uint8_t a) { uint8_t v = 0; xfer(a | READ | BURST, 0, 2, &v); return v; }
static void wr(uint8_t a, uint8_t v) { uint8_t s; xfer(a, v, 2, &s); }

static int check(void)
{
	int fails = 0;
	uint8_t s, part, ver, v;
	int g_lo, g_hi;

	strobe(SRES);
	k_msleep(1);
	s = strobe(SNOP);
	if (s & 0x80) {
		LOG_ERR("FAIL ready: status=0x%02x (CHIP_RDYn high) -> check VDD/GND/CSn/SO/SCLK", s);
		fails++;
	} else {
		LOG_INF("PASS ready: status=0x%02x", s);
	}

	part = rd_status(PARTNUM);
	ver = rd_status(VERSION);
	if (part != 0x80) {
		LOG_ERR("FAIL id: PARTNUM=0x%02x VERSION=0x%02x (want 0x80) -> check SI/SO/SCLK", part, ver);
		fails++;
	} else {
		LOG_INF("PASS id: PARTNUM=0x%02x VERSION=0x%02x (expect 0x03)", part, ver);
	}

	wr(SYNC0, 0xA5);
	v = rd(SYNC0);
	if (v != 0xA5) {
		LOG_ERR("FAIL write: SYNC0 readback 0x%02x != 0xA5 -> check SI", v);
		fails++;
	} else {
		LOG_INF("PASS write: SYNC0 readback 0xA5");
	}

	wr(IOCFG0, 0x2F); /* GDO0 driven low */
	k_usleep(100);
	g_lo = gpio_pin_get_dt(&gdo0);
	wr(IOCFG0, 0x6F); /* inverted: GDO0 driven high */
	k_usleep(100);
	g_hi = gpio_pin_get_dt(&gdo0);
	if (g_lo != 0 || g_hi != 1) {
		LOG_ERR("FAIL gdo0: low=%d high=%d -> check GDO0 wire / P0.03", g_lo, g_hi);
		fails++;
	} else {
		LOG_INF("PASS gdo0: low=0 high=1");
	}

	return fails;
}

int main(void)
{
	if (!spi_is_ready_dt(&spi) || !gpio_is_ready_dt(&gdo0)) {
		LOG_ERR("SPI or GPIO not ready");
		return -ENODEV;
	}
	if (gpio_pin_configure_dt(&gdo0, GPIO_INPUT) < 0) {
		LOG_ERR("GDO0 configure failed");
		return -EIO;
	}

	while (1) {
		int fails = check();

		LOG_INF("=== %s (%d failed) ===", fails ? "FAIL" : "ALL PASS", fails);
		k_msleep(2000);
	}
	return 0;
}
