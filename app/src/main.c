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
#include <zephyr/sys/__assert.h>

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

/* Maximum number of channels */
#define MAX_CHANNELS 6

/* Maximum number of auxiliary channels */
#define MAX_AUX_CHANNELS 4

/* Radio information structure */
struct radio_info_t {
	uint8_t rx_channel[MAX_CHANNELS];
	uint32_t aux_channels_bitmap;
	uint32_t timestamp;	
};

/* Radio fifo structure */
struct radio_data_t {
	void *fifo_reserved; /* 1st word reserved for use by fifo */
	struct radio_info_t rx_info;
};

/* Channel mapping function type */
typedef uint8_t (*map_t)(
	const uint8_t min,
	const uint8_t max,
	const uint8_t center,
	const uint8_t step,
	const uint32_t data);

/* Channel mapping structure */
struct channel_map {
	uint8_t min;
	uint8_t max;
	uint8_t center;
	uint8_t step;
	uint32_t input;
	map_t map;
};

struct aux_settings {
	bool activated;
	uint8_t aux_function;
};

struct rf_settings {
	struct channel_map ch_settings[MAX_CHANNELS];
	struct aux_settings aux_settings[MAX_AUX_CHANNELS];
};

K_FIFO_DEFINE(radio_fifo);

int fill_radio_info(struct radio_info_t *info, const struct rf_settings *settings) {
	if (info == NULL || settings == NULL) {
		return -1; // Error: Null pointer
	}

	// Fill the rx_channel array
	for (int i = 0; i < MAX_CHANNELS; i++) {
		if (settings->ch_settings[i].map != NULL) {
			info->rx_channel[i] = settings->ch_settings[i].map(
				settings->ch_settings[i].min,
				settings->ch_settings[i].max,
				settings->ch_settings[i].center,
				settings->ch_settings[i].step,
				settings->ch_settings[i].input
			);
		} else {
			info->rx_channel[i] = settings->ch_settings[i].center; // Default to center if no mapping function
		}
	}

	// Fill the aux_channels_bitmap
	for (int j = 0; j < MAX_AUX_CHANNELS; j++) {
		if (settings->aux_settings[j].activated) {
			info->aux_channels_bitmap |= (1 << j);
		}
		else {
			info->aux_channels_bitmap &= ~(1 << j);
		}
	}

	// Fill the timestamp (for example, using a simple counter or system time)
	info->timestamp = k_uptime_get_32(); // Using system uptime in milliseconds

	return 0; // Success
}


static int32_t counter;
static bool increment_counter = false;

struct rf_settings rf_parameters;

int initialize_rf_parameters(struct rf_settings *settings) {
	if (settings == NULL) {
		return -1; // Error: Null pointer
	}

	// Initialize channel settings with default values
	for (int i = 0; i < MAX_CHANNELS; i++) {
		settings->ch_settings[i].min = 0;
		settings->ch_settings[i].max = 255;
		settings->ch_settings[i].center = 127;
		settings->ch_settings[i].step = 1;
		settings->ch_settings[i].input = 127; // Default input value
		settings->ch_settings[i].map = NULL; // No mapping function by default
	}

	// Initialize auxiliary channel settings with default values
	for (int j = 0; j < MAX_AUX_CHANNELS; j++) {
		settings->aux_settings[j].activated = false;
		settings->aux_settings[j].aux_function = 0; // Default function
	}

	return 0; // Success
}

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

void action_menu_back_action(lv_event_t *e) {
    // TODO: Implement action menu_back_action here
	loadScreen(SCREEN_ID_MAIN);
}

void action_menu_settings_action(lv_event_t *e) {
    // TODO: Implement action menu_settings_action here
	loadScreen(SCREEN_ID_SETTINGS);
}


#ifdef CONFIG_NRF24L01_TRIGGER
#define TRIGGER
#endif

void radio_thread(void)
{
	const struct device *nrf24 = DEVICE_DT_GET(DT_NODELABEL(radio0));
	int err;
	struct radio_data_t *rx_data;

	/* Check if Radio device is ready */
	if (!device_is_ready(nrf24)) {
		LOG_ERR("Sensor not ready");
		return;
	}
	LOG_INF("Radio device ready");

	if (initialize_rf_parameters(&rf_parameters) != 0) {
		LOG_ERR("Failed to initialize RF parameters");
		return;
	}

	size_t size = sizeof(struct radio_info_t);
	char *mem_ptr = k_malloc(size);
	__ASSERT_NO_MSG(mem_ptr != 0);
	fill_radio_info((struct radio_info_t *)mem_ptr, &rf_parameters);


	while (true) {
		rx_data = k_fifo_get(&radio_fifo, K_FOREVER);
		if (rx_data) {
			/* Process received data */
			//LOG_INF("Received data on channel: %d", rx_data->rx_info.rx_channel[0]);

			err = nrf24_write(nrf24, (uint8_t *)&(rx_data->rx_info), sizeof(rx_data->rx_info));
			if (err != 0) {
				// LOG_HEXDUMP_INF(err, sizeof(err), "Error nbr: ");
				LOG_ERR("Failed to write data to NRF24L01+ device");
			}
			//LOG_DBG(rx_data->rx_info, sizeof(rx_data->rx_info), "Sent: ");
		}
		/* Free the allocated memory */
		k_free(rx_data);
	}
}

K_THREAD_DEFINE(radio_thread_id, STACKSIZE, radio_thread, NULL, NULL, NULL,
		PRIORITY, 0, 0);


int main(void)
{
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

	set_var_counter(0);

	while (true) {
		/* Update counter */
		if (increment_counter) {
			set_var_counter(counter + 1);
		}

		/* Update UI */
		ui_tick();
		lv_timer_handler();

		k_sleep(K_MSEC(1000));
	}
	return 0;
}
