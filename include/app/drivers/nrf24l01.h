/*
 */
#ifndef ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_WIRELESS_NRF24L01_H_
#define ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_WIRELESS_NRF24L01_H_

#include <zephyr/drivers/sensor.h>
/** @brief Any identifier (used to do 1:N identification). */
#define NRF24L01_ID_ANY 0xFFFFU

/** @brief NRF24L01 custom channels. */
enum nrf24l01_channel {
	/** Fingerprint verification. */
	JM101_CHAN_FINGERPRINT = SENSOR_CHAN_PRIV_START,
};

/** @brief JM101 custom attributes. */
enum nrf24l01_attribute {
	/** Fingerprint ID used when verifying. */
	JM101_ATTR_ID_NUM = SENSOR_ATTR_PRIV_START,
	/** Run the enrolling sequence. */
	JM101_ATTR_ENROLL,
	/** Emtpies the fingerprint database. */
	JM101_ATTR_EMPTYDB,
};

#endif /* ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_WIRELESS_NRF24L01_H_ */
