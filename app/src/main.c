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

int main(void)
{
	static const struct device *nrf24 = DEVICE_DT_GET(DT_NODELABEL(radio0));
	uint8_t data_len = 16;
	uint8_t buffer[16] = {0};
	int ret;

	if (!device_is_ready(nrf24)) {
		LOG_ERR("Sensor not ready");
		return 0;
	}
	LOG_INF("Device ready");

#if CONFIG_ROLE == 0
	#define ALICE
	printk("I am Alice!\n");
#elif CONFIG_ROLE == 1
	#define BOB
	printk("I am Bob!\n");
#else
	#define EVE
	printk("I am Eve!\n");
#endif

#ifdef ALICE
	while (true) {
		strncpy(buffer, "I am Alice, hi!", 16);
		while (true) {
			ret = nrf24_write(nrf24, buffer, data_len);
			if (ret != 0) {
				printk("Not received\n");
			} else {
				printk("Ret: %d\n", ret);
				break;
			}
			k_sleep(K_MSEC(1000));
		}
		printk("Switch to read");
		while (!nrf24_read(nrf24, buffer, data_len));
		LOG_HEXDUMP_INF(buffer, data_len, "Received: ");
		k_sleep(K_MSEC(1000));
		printk("Switch to write\n");
	}
#endif
#ifdef BOB
	while (true) {
		//while(true) {
		ret = nrf24_read(nrf24, buffer, data_len);
		LOG_HEXDUMP_INF(buffer, data_len, "Received: ");
		//}
		printk("Switch to write\n");
		strncpy(buffer, "Hi Alice Im Bob", 16);
		while (true) {
			k_sleep(K_MSEC(1000));
			ret = nrf24_write(nrf24, buffer, data_len);
			printk("Ret: %d\n", ret);
			if (ret != 0) {
				printk("Not received or ACK not set\n");
			} else {
				printk("Switch to read\n");
				break;
			}
		}
	}
#endif
	return 0;
}

