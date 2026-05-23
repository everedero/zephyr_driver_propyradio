/*
 * Copyright (C) 2026 Philippe Peurichard <p.peurichard@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/__assert.h>

#include <lvgl.h>
#include <lvgl_input_device.h>

#include <stdio.h>
#include <string.h>

#include <app/drivers/nrf24.h>

#include <vars.h>
#include <ui.h>
#include "platform.h"

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

/* ----------- PCF8575 ----------- */
#define PCF_NODE DT_NODELABEL(pcf8575)

static const struct device *pcf_dev = DEVICE_DT_GET(PCF_NODE);

static struct gpio_callback int_cb_data;

/* ----------- MyButton (overlay label: MyButton) ----------- */
static const struct gpio_dt_spec mybutton = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(user_button), gpios, {0});
static struct gpio_callback mybutton_cb;
static struct k_work_delayable mybutton_work;

/* ----------- Debounce ----------- */

static struct k_work_delayable debounce_work;

/* Radio information structure */
struct radio_info_t {
	uint8_t tx_channel[MAX_CHANNELS];
	uint32_t aux_channels_bitmap;
	uint32_t timestamp;	
};

/* Radio fifo structure */
struct radio_data_t {
	void *fifo_reserved; /* 1st word reserved for use by fifo */
	struct radio_info_t tx_info;
};

/* Channel mapping function type */
typedef uint8_t (*map_t)(
	const uint16_t min,
	const uint16_t max,
	const uint16_t center,
	      uint16_t data,
	const bool is_reversed);

/* Fonction de remappage linéaire */
static inline uint8_t fmap(uint16_t val, uint16_t low1, uint16_t max1, uint16_t low2, uint16_t max2) {
	return (uint8_t)((low2 + (val - low1) * (max2 - low2) / (max1 - low1)));
}

/* Linear mapping function with reversal support , boundaries check 
 * and tuning based on center point to give more precision around it
 * useful for joysticks and trims
 */
uint8_t def_map(
	const uint16_t min,
	const uint16_t max,
	const uint16_t center,
	      uint16_t data,
	const bool is_reversed) {

	uint8_t mapped_value = 0;
	/* Check boundaries */
	if (data < min) {
		data = min;
	}
	else if (data > max) {
		data = max;
	}
	if (data < center) {
		/* Map from [min, center] to [0, 127] */
		mapped_value = fmap(data, min, center, 0, 127);
	}
	else {
		/* Map from [center, max] to [128, 255] */
		mapped_value = fmap(data, center, max, 128, 255);
	}

	if (is_reversed) {
		mapped_value = 255 - mapped_value;
	}
	return mapped_value;
}

/* Channel mapping structure */
struct channel_map {
	uint16_t min;
	uint16_t max;
	uint16_t center;
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

struct radio_data_t radio_data;

const struct pwm_dt_spec sBuzzer = PWM_DT_SPEC_GET(DT_PATH(zephyr_user));

/* Thread plays song on buzzer */
K_SEM_DEFINE(buzzer_initialized_sem, 0, 1); /* Wait until buzzer is ready */

static bool increment_counter = false;

struct rf_settings rf_parameters;

extern void set_var_ch1_int(int32_t value);
extern void set_var_ch2_int(int32_t value);
extern void set_var_ch3_int(int32_t value);
extern void set_var_ch4_int(int32_t value);
extern void set_var_ch5_int(int32_t value);
extern void set_var_ch6_int(int32_t value);
extern void set_var_counter_int(int32_t value);
extern int32_t get_var_counter_int(void);

static const enum adc_action adc_callback(const struct device *dev,
		const struct adc_sequence *sequence,
		uint16_t sampling_index);

int fill_radio_info(struct radio_info_t *info, const struct rf_settings *settings) {
	if (info == NULL || settings == NULL) {
		return -1; // Error: Null pointer
	}

	// Fill the tx_channel array
	for (int i = 0; i < MAX_CHANNELS; i++) {
		if (settings->ch_settings[i].map != NULL) {
			info->tx_channel[i] = settings->ch_settings[i].map(
				settings->ch_settings[i].min,
				settings->ch_settings[i].max,
				settings->ch_settings[i].center,
				settings->ch_settings[i].input,
				false // Assuming no reversal for now, you can add a field in channel_map if you want to support reversed channels
			);
		} else {
			info->tx_channel[i] = 127; // Default to center if no mapping function
		}
	}
	set_var_ch1_int(info->tx_channel[0]);
	set_var_ch2_int(info->tx_channel[1]);
	set_var_ch3_int(info->tx_channel[2]);
	set_var_ch4_int(info->tx_channel[3]);
	set_var_ch5_int(info->tx_channel[4]);
	set_var_ch6_int(info->tx_channel[5]);

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

int initialize_rf_parameters(struct rf_settings *settings) {
	if (settings == NULL) {
		return -1; // Error: Null pointer
	}

	// Initialize channel settings with default values
	for (int i = 0; i < MAX_CHANNELS; i++) {
		settings->ch_settings[i].min = 0;
		settings->ch_settings[i].max = ADC_MAX_VALUE;
		settings->ch_settings[i].center = ADC_MAX_VALUE / 2;
		settings->ch_settings[i].input = ADC_MAX_VALUE / 2; // Default input value
		settings->ch_settings[i].map = def_map; // linear mapping function
	}

	// Initialize auxiliary channel settings with default values
	for (int j = 0; j < MAX_AUX_CHANNELS; j++) {
		settings->aux_settings[j].activated = false;
		settings->aux_settings[j].aux_function = 0; // Default function
	}

	return 0; // Success
}

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
	loadScreen(SCREEN_ID_MAIN);
}

void action_menu_settings_action(lv_event_t *e) {
	loadScreen(SCREEN_ID_SETTINGS);
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
		PRIORITY_BUZZER, 0, 0);


#define RADIO_ERR_WINDOW 100
static uint8_t radio_err_ring[RADIO_ERR_WINDOW] = {0};
static uint8_t radio_err_idx = 0;
static uint8_t radio_err_count = 0;

void update_radio_error_stats(int err) {
	// Remove oldest value from count
	if (radio_err_ring[radio_err_idx]) {
		radio_err_count--;
	}
	// Store new value
	radio_err_ring[radio_err_idx] = (err != 0) ? 1 : 0;
	if (err != 0) {
		radio_err_count++;
	}
	radio_err_idx = (radio_err_idx + 1) % RADIO_ERR_WINDOW;
}

float get_radio_error_percent(void) {
	return (100.0f * radio_err_count) / RADIO_ERR_WINDOW;
}

void radio_thread(void)
{
	   const struct device *nrf24 = DEVICE_DT_GET(DT_NODELABEL(radio0));
	   int err;
	   bool connected = false;
	   struct radio_data_t *tx_data;

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

	while(!connected) {
		// Buffer to get binding key from the other device, can be used to trigger buzzer or other actions on the remote controller when a specific key is pressed on the transmitter
		uint8_t ack_data[PAYLOAD_SIZE] = {0};

		err = nrf24_read(nrf24, ack_data, PAYLOAD_SIZE);

		if (err != 0) {
			LOG_ERR("Failed to read data from NRF24L01+ device");

			continue;
		}
		LOG_INF("Received data from NRF24L01+ device");

		// Process acknowledgment data if necessary
		if (ack_data[0] == 0xAA) {
			k_wakeup(buzzer_tid);
			LOG_INF("Received Ack, connected to the device");
			connected = true;
			break;
		}
	}

	while (connected) {
		tx_data = k_fifo_get(&radio_fifo, K_FOREVER);

		if (tx_data) {
			if (sizeof(tx_data->tx_info) > PAYLOAD_SIZE) {
				LOG_ERR("Data size exceeds NRF24L01+ payload limit");
				continue; // Skip sending if data is too large, or you can choose to truncate it
		   	}
	       	err = nrf24_write(nrf24, (uint8_t *)&(tx_data->tx_info), sizeof(tx_data->tx_info));
		   	update_radio_error_stats(err);

		   	// Optionally log error percentage every 100 transmissions
			static int tx_count = 0;
			tx_count++;
			if (tx_count % RADIO_ERR_WINDOW == 0) {
				LOG_INF("Radio TX error rate (last 100): %.1f%%", get_radio_error_percent());
			}
			if (err != 0) {
				LOG_ERR("Failed to write data to NRF24L01+ device");
				k_msleep(50); // Wait before retrying
				continue; // Skip to the next iteration if writing fails, or you can choose to break the loop if you want to stop trying
			}
			// LOG_INF("Sent data to NRF24L01+ device: %d bytes", err);
		}
		else {
			LOG_ERR("Failed to get data from radio FIFO");
		}
	}
}

void adc_read_thread(void)
{
	int err;
	uint16_t buf[MAX_CHANNELS]; // Buffer to hold ADC samples for all channels and samplings
	const struct adc_sequence_options adc_options = {
		.interval_us = 10000,
		.callback = &adc_callback,
		/* How many to read -1 */
		.extra_samplings = 0,
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
			for (uint8_t c = 0; c < ARRAY_SIZE(adc_channels); c++) {
				// printk("Channel %d Sample: %d", c, ((int16_t *)sequence.buffer)[c * (sequence.options->extra_samplings + 1)]);
				rf_parameters.ch_settings[c].input = ((uint16_t *)sequence.buffer)[c * (sequence.options->extra_samplings + 1)];
			}
		}
		/* Fill radio info and put it in the FIFO for transmission */
		fill_radio_info(&radio_data.tx_info, &rf_parameters);

		/* Put the radio data in the FIFO for transmission */
		k_fifo_put(&radio_fifo, &radio_data);

		/* Sleep for a while before the next read */
		k_msleep(30);
	}
}

K_THREAD_DEFINE(radio_thread_id, STACKSIZE, radio_thread, NULL, NULL, NULL,
		(PRIORITY_RADIO), 0, 0);


K_THREAD_DEFINE(adc_read_thread_id, STACKSIZE, adc_read_thread, NULL, NULL, NULL,
		(PRIORITY_ADC), 0, 0);

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
	uint32_t changed_pins;

	/* read changed pins value */
	gpio_port_get_raw(pcf_dev, &changed_pins);

	LOG_INF("Debounced value: 0x%x", changed_pins);
}

/* ----------- ISR ----------- */

static void pcf_int_callback(const struct device *dev,
                             struct gpio_callback *cb,
                             uint32_t pins)
{
    /* disable interrupt */
    // changed_pins = pins & 0xFFFF; // Mask to get only the relevant pins (assuming 16 pins)
	/* start debounce timer*/
    k_work_reschedule(&debounce_work, K_MSEC(DEBOUNCE_TIME_MS));
}

/* MyButton debounce handler */
static void mybutton_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	int val = gpio_pin_get_dt(&mybutton);
	LOG_INF("MyButton debounced state: %d", val);

	/* Re-enable interrupt on active edge */
	gpio_pin_interrupt_configure_dt(&mybutton, GPIO_INT_EDGE_TO_ACTIVE);
}

/* MyButton ISR: schedule debounce and disable further interrupts */
static void mybutton_isr(const struct device *dev,
						 struct gpio_callback *cb,
						 uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	/* Disable further interrupts while debouncing */
	gpio_pin_interrupt_configure_dt(&mybutton, GPIO_INT_DISABLE);
	k_work_reschedule(&mybutton_work, K_MSEC(10));
}


int main(void)
{
	const struct device *display_dev;

	if (!device_is_ready(pcf_dev)) {
        LOG_ERR("Device PCF8575 not ready");
        return -ENODEV;;
    }

	/* By default all pins are inputs so do nothing */
	gpio_init_callback(&int_cb_data,
                       pcf_int_callback,
                       0xFFFF); // Listen to all pins, we will check in the callback which one triggered the interrupt

    gpio_add_callback(pcf_dev, &int_cb_data);

    k_work_init_delayable(&debounce_work,
                          debounce_work_handler);

	/* Initialize MyButton if available in devicetree */
	if (mybutton.port == NULL || !device_is_ready(mybutton.port)) {
		LOG_WRN("MyButton GPIO not ready or not present in devicetree");
	} else {
		int rc;

		k_work_init_delayable(&mybutton_work, mybutton_work_handler);

		rc = gpio_pin_configure_dt(&mybutton, GPIO_INPUT);
		if (rc < 0) {
			LOG_ERR("Failed to configure MyButton pin (%d)", rc);
		} else {
			gpio_init_callback(&mybutton_cb, mybutton_isr, BIT(mybutton.pin));
			gpio_add_callback(mybutton.port, &mybutton_cb);
			gpio_pin_interrupt_configure_dt(&mybutton, GPIO_INT_EDGE_TO_ACTIVE);
			LOG_INF("MyButton initialized on %p:%d", mybutton.port, mybutton.pin);
		}
	}

    LOG_INF("PCF8575 IO Expander ready");
	
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

	set_var_counter_int(0);

	// 1 Beep at startup
	k_wakeup(buzzer_tid);

	while (true) {
		/* Update counter */
		if (increment_counter) {
			set_var_counter_int(get_var_counter_int() + 1);
		}

		/* Update UI */
		ui_tick();
		lv_timer_handler();

		k_sleep(K_MSEC(20));
	}
	return 0;
}
