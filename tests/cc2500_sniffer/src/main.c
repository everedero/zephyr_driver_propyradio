/*
 * Copyright (c) 2026 Eve Redero
 * SPDX-License-Identifier: Apache-2.0
 *
 * CC2500 RX sniffer (raw SPI, no driver): configure the chip like the original
 * remote (see spi_cc2500_protocol.md), sweep channels and log every packet with
 * RSSI, LQI and CRC status, plus per-channel RSSI average/peak (carrier sense).
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cc2500_sniffer, LOG_LEVEL_INF);

#define READ     0x80
#define BURST    0x40
#define PKTLEN   0x06
#define PKTCTRL1 0x07
#define CHANNR   0x0A
#define MCSM1    0x17
#define RSSI     0x34
#define SRES     0x30
#define SRX      0x34
#define SIDLE    0x36
#define SFRX     0x3A
#define MARCSTATE 0x35
#define RXBYTES  0x3B
#define RXFIFO   0x3F

/* Longest packet seen on the remote's SPI bus (short ones are 7 bytes) */
#define PKT_LEN  9
/* Channels to sweep and dwell time per channel: calibration knobs */
#define CH_FIRST 0
#define CH_LAST  9
#define DWELL_MS 1000
#define RSSI_OFFSET 72

static const struct spi_dt_spec spi = SPI_DT_SPEC_GET(DT_NODELABEL(cc2500), SPI_WORD_SET(8));

/* Register init sequence copied from the original remote's SPI capture */
static const uint8_t remote_init[][2] = {
	{0x0B, 0x0A}, {0x0C, 0x00}, {0x0D, 0x5D}, {0x0E, 0x13}, {0x0F, 0xB1},
	{0x10, 0x2D}, {0x11, 0x3B}, {0x12, 0x73}, {0x13, 0x22}, {0x14, 0xF8},
	{0x0A, 0x00}, {0x15, 0x00}, {0x21, 0xB6}, {0x22, 0x10}, {0x18, 0x18},
	{0x19, 0x1D}, {0x1A, 0x1C}, {0x1B, 0xC7}, {0x1C, 0x00}, {0x1D, 0xB0},
	{0x23, 0xEA}, {0x24, 0x0A}, {0x25, 0x00}, {0x26, 0x11}, {0x29, 0x59},
	{0x2C, 0x88}, {0x2D, 0x31}, {0x2E, 0x0B}, {0x00, 0x5C}, {0x02, 0x5B},
	{0x07, 0x0A}, {0x08, 0x04}, {0x09, 0x01},
};

static int xfer(uint8_t *buf, size_t len)
{
	const struct spi_buf b = {.buf = buf, .len = len};
	const struct spi_buf_set s = {.buffers = &b, .count = 1};

	return spi_transceive_dt(&spi, &s, &s);
}

static void strobe(uint8_t cmd) { uint8_t b[1] = {cmd}; xfer(b, 1); }
static void wr(uint8_t a, uint8_t v) { uint8_t b[2] = {a, v}; xfer(b, 2); }
static uint8_t rd_status(uint8_t a) { uint8_t b[2] = {a | READ | BURST, 0}; xfer(b, 2); return b[1]; }

static void rx_restart(uint8_t ch)
{
	strobe(SIDLE);
	strobe(SFRX);
	wr(CHANNR, ch);
	strobe(SRX); /* FS_AUTOCAL calibrates on IDLE -> RX */
}

static void read_packet(uint8_t ch)
{
	/* header + payload + 2 appended status bytes (RSSI, LQI|CRC_OK) */
	uint8_t b[1 + PKT_LEN + 2] = {RXFIFO | READ | BURST};
	int8_t rssi;
	uint8_t lqi;

	xfer(b, sizeof(b));
	rssi = (int8_t)b[1 + PKT_LEN];
	lqi = b[2 + PKT_LEN];
	LOG_INF("ch=%u rssi=%d dBm lqi=%u crc=%s", ch, rssi / 2 - RSSI_OFFSET,
		lqi & 0x7F, (lqi & 0x80) ? "OK" : "BAD");
	LOG_HEXDUMP_INF(&b[1], PKT_LEN, "pkt");
}

int main(void)
{
	if (!spi_is_ready_dt(&spi)) {
		LOG_ERR("SPI not ready");
		return -ENODEV;
	}

	strobe(SRES);
	k_msleep(1);
	for (size_t i = 0; i < ARRAY_SIZE(remote_init); i++) {
		wr(remote_init[i][0], remote_init[i][1]);
	}
	/* Sniffer overrides: append RSSI/LQI, keep CRC-failing packets, no address filter */
	wr(PKTCTRL1, 0x04);
	wr(PKTLEN, PKT_LEN);
	wr(MCSM1, 0x3C); /* RXOFF_MODE = stay in RX: no blind gap after a packet */

	LOG_INF("Sniffing ch %d..%d, %d ms each, %d-byte packets", CH_FIRST, CH_LAST, DWELL_MS,
		PKT_LEN);

	while (1) {
		for (uint8_t ch = CH_FIRST; ch <= CH_LAST; ch++) {
			int64_t end = k_uptime_get() + DWELL_MS;
			int pkts = 0, polls = 0, rssi_max = -128;
			long rssi_sum = 0;

			rx_restart(ch);
			while (k_uptime_get() < end) {
				uint8_t n = rd_status(RXBYTES);
				uint8_t st = rd_status(MARCSTATE) & 0x1F;
				int8_t rssi = (int8_t)rd_status(RSSI);

				rssi_sum += rssi;
				rssi_max = MAX(rssi_max, rssi);
				polls++;

				if (n & 0x80 || st == 0x11) { /* RX FIFO overflow */
					rx_restart(ch);
				} else if ((n & 0x7F) >= PKT_LEN + 2) {
					read_packet(ch);
					pkts++;
				} else if (st == 0x01) { /* IDLE without a full packet */
					rx_restart(ch);
				}
				k_msleep(1);
			}
			LOG_INF("ch=%u: %d packets, rssi avg %ld max %d dBm", ch, pkts,
				rssi_sum / (2 * polls) - RSSI_OFFSET, rssi_max / 2 - RSSI_OFFSET);
		}
	}
	return 0;
}
