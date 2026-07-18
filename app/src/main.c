/*
 * Project : rc_remote_controller
 * File    : main.c
 * Author  : Philippe Peurichard <p.peurichard@gmail.com>
 * Date    : 2026-06-07
 * Brief   : Main application entry and initialization
 * License : Apache-2.0
 *
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
#include "model.h"

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

/**
 * @brief Button interrupt service routine.
 *
 * This ISR handles the physical button press event and performs any
 * immediate processing required by the application.
 *
 * @param[in] port GPIO device that generated the interrupt.
 * @param[in] cb   GPIO callback data.
 * @param[in] pins Bitmask of pins that triggered the interrupt.
 */
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

/* Channel mapping types are defined in model.h */

/**
 * @brief Linear interpolation mapping helper.
 *
 * Maps a value from one range to another using integer arithmetic.
 *
 * @param[in] val  Input value in the source range.
 * @param[in] low1 Lower bound of the source range.
 * @param[in] max1 Upper bound of the source range.
 * @param[in] low2 Lower bound of the destination range.
 * @param[in] max2 Upper bound of the destination range.
 * @return Mapped value in the destination range.
 */
static inline uint8_t fmap(uint16_t val, uint16_t low1, uint16_t max1, uint16_t low2, uint16_t max2) {
	return (uint8_t)((low2 + (val - low1) * (max2 - low2) / (max1 - low1)));
}

/**
 * @brief Convert ADC input into a radio channel value.
 *
 * This function maps an ADC input range to a radio channel output range
 * and optionally reverses the result for inverted controls.
 *
 * @param[in]  min         Minimum ADC input value.
 * @param[in]  max         Maximum ADC input value.
 * @param[in]  center      Center point for the input range.
 * @param[in]  data        Current ADC reading.
 * @param[in]  is_reversed True to reverse the mapped value.
 * @return Mapped channel value between 0 and 255.
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

/* `struct channel_map` defined in model.h */

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

/**
 * @brief Semaphore used to delay buzzer startup until initialization completes.
 */
K_SEM_DEFINE(buzzer_initialized_sem, 0, 1); /* Wait until buzzer is ready */

static bool increment_counter = false;

struct rf_settings rf_parameters;

typedef enum {
	ROULIS = 0,
	TANGAGE = 1,
	GAZ = 2,
	LACET = 3,
	AUX1 = 4,
	AUX2 = 5
} index_name_t;

index_name_t index_lockup_table[MAX_CHANNELS] = {
	LACET,    // default mapping for channel 1
	GAZ,	  // default mapping for channel 2
	TANGAGE,  // default mapping for channel 3
	AUX1,	  // default mapping for channel 4
	AUX2,     // default mapping for aux1 channel
	ROULIS    // default mapping for aux2 channel
};

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

/**
 * @brief Populate radio transfer data from RF settings.
 *
 * Converts configured channel and auxiliary settings into a radio payload
 * structure and updates selected UI variables for the mapped channels.
 *
 * @param[out] info     Destination radio info structure.
 * @param[in]  settings Source RF settings including channel maps.
 * @return 0 on success, -1 on invalid input.
 */
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
				settings->ch_settings[i].is_reversed
			);
		} else {
			info->tx_channel[i] = 127; // Default to center if no mapping function
		}
	}
	set_var_ch1_int(info->tx_channel[index_lockup_table[get_var_selection1()]]);
	set_var_ch2_int(info->tx_channel[index_lockup_table[get_var_selection2()]]);
	set_var_ch3_int(info->tx_channel[index_lockup_table[get_var_selection3()]]);
	set_var_ch4_int(info->tx_channel[index_lockup_table[get_var_selection4()]]);
	set_var_ch5_int(info->tx_channel[index_lockup_table[AUX1]]);
	set_var_ch6_int(info->tx_channel[index_lockup_table[AUX2]]);

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

/**
 * @brief Initialize RF parameter defaults from hardware state.
 *
 * Reads GPIO switch positions and initializes channel and auxiliary
 * settings for the RF transmitter.
 *
 * @param[out] settings RF settings structure to populate.
 * @return 0 on success, -1 if the parameter pointer is null.
 */
int initialize_rf_parameters(struct rf_settings *settings) {
	uint32_t changed_pins;
	uint8_t channel_selection[MODEL_SELECTION_COUNT] = {ROULIS, TANGAGE, GAZ, LACET};

	if (settings == NULL) {
		return -1; // Error: Null pointer
	}

	if (load_model(&active_model_index, settings->ch_settings, channel_selection) == -1) {
		LOG_ERR("Failed to load model");
		return -1;
	}

	set_var_selection1(channel_selection[0]);
	set_var_selection2(channel_selection[1]);
	set_var_selection3(channel_selection[2]);
	set_var_selection4(channel_selection[3]);
	/* read changed pins value */
	gpio_port_get_raw(pcf_dev, &changed_pins);

	settings->aux_settings[0].activated = IS_SWITCH_SW_1_ACTIVATED(changed_pins); // Default state based on switch position
	settings->aux_settings[1].activated = IS_SWITCH_SW_2_ACTIVATED(changed_pins); // Default state based on switch position
	settings->aux_settings[2].activated = IS_SWITCH_SW_3_ACTIVATED(changed_pins); // Default state based on switch position
	settings->aux_settings[3].activated = IS_SWITCH_SW_4_ACTIVATED(changed_pins); // Default state based on switch position

	// Initialize auxiliary channel settings with default values
	for (int j = 0; j < MAX_AUX_CHANNELS; j++) {
		settings->aux_settings[j].aux_function = 0; // Default function
	}

	return 0; // Success
}

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};

/**
 * @brief Handle the start/stop button press event.
 *
 * Toggles the counter state and updates the button label accordingly.
 *
 * @param[in] e LVGL event object.
 */
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

/**
 * @brief Switch back to the main screen.
 *
 * @param[in] e LVGL event object.
 */
void action_menu_back_action(lv_event_t *e) {
	loadScreen(SCREEN_ID_MAIN);
}

/**
 * @brief Switch to the settings screen.
 *
 * @param[in] e LVGL event object.
 */
void action_menu_settings_action(lv_event_t *e) {
	loadScreen(SCREEN_ID_SETTINGS);
}

void action_save_button_pressed(lv_event_t *e) {
	/* Todo */
}
/**
 * @brief Thread entry point for buzzer control.
 *
 * Waits for initialization to complete and then pulses the buzzer when awakened.
 *
 * @param[in] d0 Unused thread argument.
 * @param[in] d1 Unused thread argument.
 * @param[in] d2 Unused thread argument.
 */
void buzzer_thread(void *d0, void *d1, void *d2)
{
	if (!device_is_ready(sBuzzer.dev)) {
		LOG_ERR("Buzzer device not ready");
		return;
	}
	set_var_load_bar_progress(get_var_load_bar_progress() + 20);
	LOG_INF("Buzzer device ready");

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
		PRIORITY_BUZZER, 0, BUZZER_STARTUP_DELAY);


#define RADIO_ERR_WINDOW 100
static uint8_t radio_err_ring[RADIO_ERR_WINDOW] = {0};
static uint8_t radio_err_idx = 0;
static uint8_t radio_err_count = 0;

/**
 * @brief Update the sliding window of radio transmission errors.
 *
 * Tracks the most recent radio transmission errors in a circular buffer
 * and maintains a count of failing transmissions.
 *
 * @param[in] err Error code from the latest radio operation.
 */
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

/**
 * @brief Get the current radio error rate.
 *
 * @return Fraction of recent transmissions that failed, expressed as a percentage.
 */
float get_radio_error_percent(void) {
	return (100.0f * radio_err_count) / RADIO_ERR_WINDOW;
}

/**
 * @brief Main radio communication thread.
 *
 * Handles NRF24 initialization, connection establishment, and periodic
 * transmission of radio data from the FIFO.
 */
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

	set_var_load_bar_progress(get_var_load_bar_progress() + 20);

	if (initialize_rf_parameters(&rf_parameters) != 0) {
		LOG_ERR("Failed to initialize RF parameters");
		return;
	}
try_reconnect:
	while(!connected) {
		// Buffer to get binding key from the other device, can be used to trigger buzzer or other actions on the remote controller when a specific key is pressed on the transmitter
		uint8_t ack_data[PAYLOAD_SIZE] = {0};

		err = nrf24_read(nrf24, ack_data, PAYLOAD_SIZE);

		if (err != 0) {
			LOG_ERR("Failed to read data from NRF24L01+ device");

			continue;
		}
		LOG_INF("Received data from NRF24L01+ device");
		#if defined ACK
			// TO DO: manage ACK payload properly, for now we just read it without checking if it is actually an ACK or not, 
			// and we do not use it for anything specific
			uint8_t ack[2] = {0};
			err = nrf24l01_write_ack_payload(nrf24, ack, sizeof(ack), 0); 
			// pipe 0 for ACK payload
		#endif

		// Process acknowledgment data if necessary
		if (ack_data[0] == 0xAA) {
			k_wakeup(buzzer_tid);
			LOG_INF("Received Ack, connected to the device");
			set_var_binding_led_color(0x00FF00); // Green color for binding status LED
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
				// LOG_INF("Radio TX error rate (last 100): %.1f%%", ( double )get_radio_error_percent());
				if (get_radio_error_percent() > 80.0f) { // If error rate is above 10%, trigger some action, for example, change LED color to indicate poor connection
					set_var_binding_led_color(0xFF0000); // Red color for poor connection
					connected = false; // Optionally, you can also set connected to false to stop trying to send data until a new connection is established, or you can choose to keep trying to send data and just indicate the poor connection status with the LED color.
					k_wakeup(buzzer_tid); // Wake up buzzer thread to alert the user about poor connection
					goto try_reconnect; // Jump to reconnection logic
				}
				else {
					set_var_binding_led_color(0x00FF00); // Green color for good connection
				}
			}
			if (err != 0) {
				LOG_ERR("Failed to write data to NRF24L01+ device! err: %d",err);
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

/**
 * @brief ADC sampling thread.
 *
 * Periodically reads ADC channels, updates RF input values, and enqueues
 * radio payload data for transmission.
 */
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

	set_var_load_bar_progress(get_var_load_bar_progress() + 20);

	LOG_INF("Starting ADC read thread");
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
		(PRIORITY_RADIO), 0, RADIO_STARTUP_DELAY);


K_THREAD_DEFINE(adc_read_thread_id, STACKSIZE, adc_read_thread, NULL, NULL, NULL,
		(PRIORITY_ADC), 0, ADC_STARTUP_DELAY);

/**
 * @brief ADC callback used during asynchronous conversions.
 *
 * @param[in] dev           ADC device pointer.
 * @param[in] sequence      ADC sequence descriptor.
 * @param[in] sampling_index Sampling index supplied by the ADC subsystem.
 * @return ADC_ACTION_CONTINUE to keep sampling.
 */
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

/**
 * @brief Debounce handler for GPIO input changes.
 *
 * Reads input pin state and updates auxiliary channel activation and trim
 * centers while the corresponding buttons are pressed.
 *
 * @param[in] work Work item pointer.
 */
static void debounce_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
	uint32_t changed_pins;

	/* read changed pins value */
	gpio_port_get_raw(pcf_dev, &changed_pins);
	rf_parameters.aux_settings[0].activated = IS_SWITCH_SW_1_ACTIVATED(changed_pins);
	rf_parameters.aux_settings[1].activated = IS_SWITCH_SW_2_ACTIVATED(changed_pins);
	rf_parameters.aux_settings[2].activated = IS_SWITCH_SW_3_ACTIVATED(changed_pins);
	rf_parameters.aux_settings[3].activated = IS_SWITCH_SW_4_ACTIVATED(changed_pins);

	while(IS_TRIM_JOYSTICK_ROULIS_UP_ACTIVATED(changed_pins)) {
		rf_parameters.ch_settings[ROULIS].center += TRIM_INCREMENT; // Gaz channel to max
		k_msleep(DEBOUNCE_TIME_MS); // Add a small delay to avoid too fast changes, adjust as needed
		gpio_port_get_raw(pcf_dev, &changed_pins);
	}
	while(IS_TRIM_JOYSTICK_ROULIS_DOWN_ACTIVATED(changed_pins)) {
		rf_parameters.ch_settings[ROULIS].center -= TRIM_INCREMENT; // Gaz channel to min
		k_msleep(DEBOUNCE_TIME_MS); // Add a small delay to avoid too fast changes, adjust as needed
		gpio_port_get_raw(pcf_dev, &changed_pins);
	}
	while(IS_TRIM_JOYSTICK_TANGAGE_UP_ACTIVATED(changed_pins)) {
		rf_parameters.ch_settings[TANGAGE].center += TRIM_INCREMENT; // Gaz channel to max
		k_msleep(DEBOUNCE_TIME_MS); // Add a small delay to avoid too fast changes, adjust as needed
		gpio_port_get_raw(pcf_dev, &changed_pins);
	}
	while(IS_TRIM_JOYSTICK_TANGAGE_DOWN_ACTIVATED(changed_pins)) {
		rf_parameters.ch_settings[TANGAGE].center -= TRIM_INCREMENT; // Gaz channel to min
		k_msleep(DEBOUNCE_TIME_MS); // Add a small delay to avoid too fast changes, adjust as needed
		gpio_port_get_raw(pcf_dev, &changed_pins);
	}
	while(IS_TRIM_JOYSTICK_LACET_UP_ACTIVATED(changed_pins)) {
		rf_parameters.ch_settings[LACET].center += TRIM_INCREMENT; // Gaz channel to max
		k_msleep(DEBOUNCE_TIME_MS); // Add a small delay to avoid too fast changes, adjust as needed
		gpio_port_get_raw(pcf_dev, &changed_pins);
	}
	while(IS_TRIM_JOYSTICK_LACET_DOWN_ACTIVATED(changed_pins)) {
		rf_parameters.ch_settings[LACET].center -= TRIM_INCREMENT; // Gaz channel to min
		k_msleep(DEBOUNCE_TIME_MS); // Add a small delay to avoid too fast changes, adjust as needed
		gpio_port_get_raw(pcf_dev, &changed_pins);
	}
	while(IS_TRIM_JOYSTICK_GAZ_UP_ACTIVATED(changed_pins)) {
		rf_parameters.ch_settings[GAZ].center += TRIM_INCREMENT; // Gaz channel to max
		k_msleep(DEBOUNCE_TIME_MS); // Add a small delay to avoid too fast changes, adjust as needed
		gpio_port_get_raw(pcf_dev, &changed_pins);
	}
	while(IS_TRIM_JOYSTICK_GAZ_DOWN_ACTIVATED(changed_pins)) {
		rf_parameters.ch_settings[GAZ].center -= TRIM_INCREMENT; // Gaz channel to min
		k_msleep(DEBOUNCE_TIME_MS); // Add a small delay to avoid too fast changes, adjust as needed
		gpio_port_get_raw(pcf_dev, &changed_pins);
	}



	if (IS_SWITCH_SW_A_UP_ACTIVATED(changed_pins)) {
		//LOG_INF("Switch SWA_UP activated");
	} else {
		//LOG_INF("Switch SWA_UP deactivated");
	}
	if (IS_SWITCH_SW_A_DOWN_ACTIVATED(changed_pins)) {
		//LOG_INF("Switch SWA_DOWN activated");
	} else {
		//LOG_INF("Switch SWA_DOWN deactivated");
	}
	if (IS_SWITCH_SW_B_UP_ACTIVATED(changed_pins)) {
		//LOG_INF("Switch SWB_UP activated");
	} else {
		//LOG_INF("Switch SWB_UP deactivated");
	}
	if (IS_SWITCH_SW_B_DOWN_ACTIVATED(changed_pins)) {
		//LOG_INF("Switch SWB_DOWN activated");
	} else {
		//LOG_INF("Switch SWB_DOWN deactivated");
	}

	LOG_INF("Debounced value: 0x%x", changed_pins);
}

/* ----------- ISR ----------- */

/**
 * @brief PCF8575 interrupt callback.
 *
 * Schedules debounce processing when an input pin on the expander changes.
 *
 * @param[in] dev GPIO device that generated the interrupt.
 * @param[in] cb  GPIO callback structure.
 * @param[in] pins Bitmask of pins that changed.
 */
static void pcf_int_callback(const struct device *dev,
                             struct gpio_callback *cb,
                             uint32_t pins)
{
    /* disable interrupt */
    // changed_pins = pins & 0xFFFF; // Mask to get only the relevant pins (assuming 16 pins)
	/* start debounce timer*/
    k_work_reschedule(&debounce_work, K_MSEC(DEBOUNCE_TIME_MS));
}

/**
 * @brief Handler invoked after MyButton debounce interval.
 *
 * Reads the debounced state and re-enables the MyButton interrupt.
 *
 * @param[in] work Work item pointer.
 */
static void mybutton_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	int val = gpio_pin_get_dt(&mybutton);
	LOG_INF("MyButton debounced state: %d", val);

	/* Re-enable interrupt on active edge */
	gpio_pin_interrupt_configure_dt(&mybutton, GPIO_INT_EDGE_TO_ACTIVE);
}

/**
 * @brief Button ISR that schedules debounce handling.
 *
 * Disables further interrupts and reschedules the MyButton work item.
 *
 * @param[in] dev GPIO device that generated the interrupt.
 * @param[in] cb  GPIO callback structure.
 * @param[in] pins Bitmask of pins that triggered the interrupt.
 */
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

/**
 * @brief Update the initialization progress bar event.
 *
 * This action is called by the UI to reflect initialization progress.
 *
 * @param[in] e LVGL event object.
 */
void action_update_init_bar(lv_event_t *e) {
    // TODO: Implement action update_init_bar here
	LOG_INF("Updating init bar progress");
}

/**
 * @brief Main application entry point.
 *
 * Initializes peripherals, UI, and worker threads before entering the main
 * event loop.
 *
 * @return 0 on success.
 */
int main(void)
{
	const struct device *display_dev;

	/* Initialize UI variables */
	set_var_binding_led_color(0xFF0000); // Red color for binding status LED
	set_var_load_bar_progress(10); // 10% progress at the start of initialization

	display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

	/* Check if the Display device is ready */
	if (!device_is_ready(display_dev)) {
		LOG_ERR("Device not ready, aborting test");
		return 0;
	}

	// Short delay to ensure everything is settled before starting the main loop
	k_msleep(100);

	/* Initialize UI */
	ui_init();
	lv_timer_handler();
	display_blanking_off(display_dev);

	// Short delay to ensure everything is settled before starting the main loop
	k_msleep(100);

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
	set_var_load_bar_progress(get_var_load_bar_progress() + 20);
	/* Update UI */
	ui_tick();
	lv_timer_handler();

	// Short delay to ensure everything is settled before starting the main loop
	k_msleep(500);
	
	while(get_var_load_bar_progress() <= 80) {
		/* Update UI */
		ui_tick();
		lv_timer_handler();
		k_msleep(500);
	}
	k_msleep(500);
	set_var_load_bar_progress(100);
	/* Update UI */
	ui_tick();
	lv_timer_handler();

	// Release buzzer thread to play startup sound
	k_sem_give(&buzzer_initialized_sem);
	
	k_msleep(500);
	/* Initialize main screen */
	loadScreen(SCREEN_ID_MAIN);

	/* Update UI */
	ui_tick();
	lv_timer_handler();

	set_var_counter_int(0);

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
