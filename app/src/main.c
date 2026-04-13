/*
 * Copyright (C) 2024 Eve Redero
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pwm.h>
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

/* ----------- PCF8575 ----------- */
#define PCF_NODE DT_NODELABEL(pcf8575)

static const struct device *pcf_dev = DEVICE_DT_GET(PCF_NODE);

static struct gpio_callback int_cb_data;

/* ----------- Debounce ----------- */

#define DEBOUNCE_TIME_MS 20

#define INPUT_PIN 0

static struct k_work_delayable debounce_work;

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
	const uint16_t resolution,
	const uint16_t data);

uint8_t def_map(
	const uint8_t min,
	const uint8_t max,
	const uint8_t center,
	const uint16_t resolution,
	const uint16_t data) {
	// Simple linear mapping for demonstration
	return (uint8_t)((data - min) * (max - min) / (resolution - min) + min);
}

/* Channel mapping structure */
struct channel_map {
	uint8_t min;
	uint8_t max;
	uint8_t center;
	uint16_t resolution;
	uint16_t input;
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

const struct pwm_dt_spec sBuzzer = PWM_DT_SPEC_GET(DT_PATH(zephyr_user));

/* Thread plays song on buzzer */
K_SEM_DEFINE(buzzer_initialized_sem, 0, 1); /* Wait until buzzer is ready */

static const enum adc_action adc_callback(const struct device *dev,
		const struct adc_sequence *sequence,
		uint16_t sampling_index);

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
				settings->ch_settings[i].resolution,
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
		settings->ch_settings[i].resolution = 255;
		settings->ch_settings[i].input = 127; // Default input value
		settings->ch_settings[i].map = def_map; // linear mapping function
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


#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};

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

void adc_read_thread(void)
{
	int err;
	__aligned(32) uint16_t buf[32 * 6];
	const struct adc_sequence_options adc_options = {
		.interval_us = 10000,
		.callback = &adc_callback,
		/* How many to read -1 */
		.extra_samplings = 31,
	};
	struct adc_sequence sequence = {
		.buffer = buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
		.options = &adc_options,
		.channels = 0xf210, /* 0b1111001000010000, adc channels bitmask */
	};

	/* Configure channels individually prior to sampling. */
	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		if (!adc_is_ready_dt(&adc_channels[i])) {
			printk("ADC controller device %s not ready\n", adc_channels[i].dev->name);
			return;
		}

		err = adc_channel_setup_dt(&adc_channels[i]);
		if (err < 0) {
			printk("Could not setup channel #%d (%d)\n", i, err);
			return;
		}
	}
	/* Initializes sequence from channel 0 parameters */
	/* All elements should have same resolution and oversampling parameters */
	err = adc_sequence_init_dt(adc_channels, &sequence);

	if(err<0) {
		printk("Could not initialize sequence from device tree (%d)\n", err);
		return;
	}

	/* Re-set multiple channel config, rewritten by sequence_init */
	sequence.channels = 0xf210; /* 0b1111001000010000, adc channels bitmask */

	while (true) {
		err = adc_read_dt(adc_channels, &sequence);
		if (err < 0) {
			printk("Could not read (%d)\n", err);
			continue;
		}
		else {
			/* Process ADC samples stored in buf */
			for (uint8_t ch = 0; ch < ARRAY_SIZE(adc_channels); ch++) {
				// printk("Channel %d Sample: %d", ch, ((int16_t *)sequence.buffer)[ch * (sequence.options->extra_samplings + 1)]);
				rf_parameters.ch_settings[ch].resolution = (uint16_t)((1 << adc_channels[ch].resolution) - 1); // Calculate resolution from ADC resolution bits
				rf_parameters.ch_settings[ch].input = ((int16_t *)sequence.buffer)[ch * (sequence.options->extra_samplings + 1)];
			}
		}
		k_msleep(100); // Sleep for a while before the next read
	}
}

void buzzer_thread(void *d0, void *d1, void *d2)
{
	/* Block until buzzer is available */
	k_sem_take(&buzzer_initialized_sem, K_FOREVER);
	while (true) {
		
		pwm_set_dt(&sBuzzer, PWM_HZ(1000), PWM_HZ(1000) / 2);
		k_msleep(100);
		
		/* turn buzzer off (pulse duty to 0) */
		pwm_set_pulse_dt(&sBuzzer, 0);

		/* Sleep thread until awoken externally */
		k_sleep(K_FOREVER);
	}
}
K_THREAD_DEFINE(buzzer_tid, STACKSIZE, buzzer_thread, NULL, NULL, NULL,
		PRIORITY, 0, 0);


K_THREAD_DEFINE(radio_thread_id, STACKSIZE, radio_thread, NULL, NULL, NULL,
		(PRIORITY-1), 0, 0);


K_THREAD_DEFINE(adc_read_thread_id, STACKSIZE, adc_read_thread, NULL, NULL, NULL,
		(PRIORITY-2), 0, 0);

static const enum adc_action adc_callback(const struct device *dev,
		const struct adc_sequence *sequence,
		uint16_t sampling_index)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sequence);
	ARG_UNUSED(sampling_index);
	return(ADC_ACTION_CONTINUE);
}

/* ----------- Work handler ----------- */

static void debounce_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
	int val = gpio_pin_get(pcf_dev, INPUT_PIN);
	LOG_INF("Debounced value: %d", val);
	/* re-enable interrupt */
	gpio_pin_interrupt_configure(pcf_dev, INPUT_PIN, GPIO_INT_EDGE_BOTH);
}

/* ----------- ISR ----------- */

static void pcf_int_callback(const struct device *dev,
                             struct gpio_callback *cb,
                             uint32_t pins)
{
    /* disable interrupt */
    gpio_pin_interrupt_configure(dev, pins, GPIO_INT_DISABLE);

	/* start debounce timer*/
    k_work_reschedule(&debounce_work, K_MSEC(DEBOUNCE_TIME_MS));
}


int main(void)
{
	const struct device *display_dev;
	int ret;

	if (!device_is_ready(pcf_dev)) {
        LOG_ERR("Device PCF8575 not ready");
        return -ENODEV;;
    }

	/* By default all pins are inputs so do nothing */
    // gpio_pin_configure(pcf_dev, INPUT_PIN, GPIO_INPUT); // Configure pin 0 as input (INT pin)
    
    gpio_init_callback(&int_cb_data,
                       pcf_int_callback,
                       BIT(INPUT_PIN));

    gpio_add_callback(pcf_dev, &int_cb_data);

    k_work_init_delayable(&debounce_work,
                          debounce_work_handler);

	ret = gpio_pin_interrupt_configure(pcf_dev, INPUT_PIN, GPIO_INT_EDGE_TO_ACTIVE); // Configure interrupt on rising edge

	if (ret < 0) {
        LOG_ERR("Interrupt config failed");
        return ret;
    }

    LOG_INF("PCF8575 debounce example ready");
	
	if (!device_is_ready(sBuzzer.dev)) {
		return -ENODEV;
	}
	k_sem_give(&buzzer_initialized_sem);
	LOG_INF("Buzzer device ready");
	

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

	// 1 Beep at startup
	k_wakeup(buzzer_tid);

	while (true) {
		/* Update counter */
		if (increment_counter) {
			set_var_counter(counter + 1);
		}

		/* Update UI */
		ui_tick();
		lv_timer_handler();

		k_sleep(K_MSEC(1));
	}
	return 0;
}
