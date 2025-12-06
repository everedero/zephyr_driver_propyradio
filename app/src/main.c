/*
 * Copyright (C) 2024 Eve Redero
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <lvgl.h>
#include <stdio.h>
#include <string.h>
#include <lvgl_input_device.h>
#include <app/drivers/nrf24.h>
#include <vars.h>
#include <ui.h>

static int32_t counter;
static bool increment_counter = false;

int32_t get_var_counter() {
	static char str_buf[11];
    snprintf(str_buf, sizeof(str_buf), "%d", counter);
    return (int32_t) str_buf;
}

void set_var_counter(int32_t value) {
    counter = value;
}


#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#if !DT_NODE_EXISTS(DT_NODELABEL(radio0))
#error "whoops, node label radio0 not found"
#endif

#ifdef CONFIG_RESET_COUNTER_SW0
static struct gpio_dt_spec button_gpio = GPIO_DT_SPEC_GET_OR(
		DT_ALIAS(sw0), gpios, {0});
static struct gpio_callback button_callback;

static void button_isr_callback(const struct device *port,
				struct gpio_callback *cb,
				uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	// count = 0;
}
#endif /* CONFIG_RESET_COUNTER_SW0 */

#ifdef CONFIG_LV_Z_ENCODER_INPUT
static const struct device *lvgl_encoder =
	DEVICE_DT_GET(DT_COMPAT_GET_ANY_STATUS_OKAY(zephyr_lvgl_encoder_input));
#endif /* CONFIG_LV_Z_ENCODER_INPUT */

#ifdef CONFIG_LV_Z_KEYPAD_INPUT
static const struct device *lvgl_keypad =
	DEVICE_DT_GET(DT_COMPAT_GET_ANY_STATUS_OKAY(zephyr_lvgl_keypad_input));
#endif /* CONFIG_LV_Z_KEYPAD_INPUT */

void action_start_button_pressed(lv_event_t *e)
{
	ARG_UNUSED(e);
	if (increment_counter) {
		increment_counter = false;
		lv_label_set_text(objects.button_label, "Start");
	} else {
		increment_counter = true;
		lv_label_set_text(objects.button_label, "Stop");
	}

	// count = 0;
}

#ifdef CONFIG_NRF24L01_TRIGGER
#define TRIGGER
#endif

#if CONFIG_ROLE == 0
	#define ALICE
#elif CONFIG_ROLE == 1
	#define BOB
#else
	#define EVE
#endif


int main(void)
{
	static const struct device *nrf24 = DEVICE_DT_GET(DT_NODELABEL(radio0));
	uint8_t data_len = 16;
	uint8_t buffer[16] = {0};
	const struct device *display_dev;
	
	display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

	/* Check if the Display device is ready */
	if (!device_is_ready(display_dev)) {
		LOG_ERR("Device not ready, aborting test");
		return 0;
	}
	/* Initialize UI */
	ui_init();
	lv_timer_handler();
	display_blanking_off(display_dev);

#ifndef TRIGGER
	int i;
#endif // TRIGGER
	/* Check if Radio device is ready */
	if (!device_is_ready(nrf24)) {
		LOG_ERR("Sensor not ready");
		return 0;
	}
	LOG_INF("Device ready");
#ifdef TRIGGER
	LOG_DBG("Trigger mode activated");
#endif //TRIGGER

#ifdef ALICE
	LOG_WRN("I am Alice!");
	/* Initialize counter */
	set_var_counter(0);

	while (true) {
		/* Update counter */
		if (increment_counter) {
			set_var_counter(counter + 1);
		}

		/* Update UI */
		ui_tick();
		lv_timer_handler();

		strncpy(buffer, "I am Alice, hi!", 16);
#ifdef TRIGGER
		while (nrf24_write(nrf24, buffer, data_len))
		{
			k_sleep(K_MSEC(10));
		}
#else
		for (i=0; i<10; i++)
		{
			nrf24_write(nrf24, buffer, data_len);
			k_sleep(K_MSEC(10));
		}
#endif // TRIGGER
		LOG_HEXDUMP_INF(buffer, data_len, "Sent: ");

#if 0
		LOG_DBG("Switch to read");
#ifdef TRIGGER
		while (nrf24_read(nrf24, buffer, data_len));
#else
		nrf24_read(nrf24, buffer, data_len);
#endif // TRIGGER
		LOG_HEXDUMP_INF(buffer, data_len, "Received: ");
		k_sleep(K_MSEC(100));
		LOG_DBG("Switch to write");
#endif // 0
	}
#endif // ALICE

#ifdef BOB
	LOG_WRN("I am Bob!");
	while (true) {
		lv_timer_handler();
#ifdef TRIGGER
		while (nrf24_read(nrf24, buffer, data_len));
#else
		nrf24_read(nrf24, buffer, data_len);
#endif // TRIGGER
		LOG_HEXDUMP_INF(buffer, data_len, "Received: ");
		LOG_DBG("Switch to write");
		strncpy(buffer, "Hi Alice Im Bob", 16);
		k_sleep(K_MSEC(1000));
#ifdef TRIGGER
		while (nrf24_write(nrf24, buffer, data_len))
		{
			k_sleep(K_MSEC(10));
		}
#else
		for (i=0; i<10; i++)
		{
			nrf24_write(nrf24, buffer, data_len);
			k_sleep(K_MSEC(10));
		}
#endif // TRIGGER
		LOG_HEXDUMP_INF(buffer, data_len, "Sent: ");
		LOG_DBG("Switch to read");
	}
#endif // BOB

#ifdef EVE
	LOG_WRN("I am Eve!");
	while (true) {
		lv_timer_handler();
#ifdef TRIGGER
		while (nrf24_read(nrf24, buffer, data_len));
#else
		strncpy(buffer, "               ", 16);
		nrf24_read(nrf24, buffer, data_len);
#endif // TRIGGER
		LOG_HEXDUMP_INF(buffer, data_len, "I spied: ");
	}
#endif // EVE
	return 0;
}
