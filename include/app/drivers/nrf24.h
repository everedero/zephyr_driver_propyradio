/*
 */
#ifndef ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_WIRELESS_NRF24_H_
#define ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_WIRELESS_NRF24_H_

#include <zephyr/device.h>
#include <zephyr/toolchain.h>

/** @cond INTERNAL_HIDDEN */

typedef int (*nrf24_read_t)(const struct device *dev, uint8_t *buffer, uint8_t data_len);
typedef int (*nrf24_write_t)(const struct device *dev, uint8_t *buffer, uint8_t data_len);

__subsystem struct nrf24_api {
	nrf24_read_t read;
	nrf24_write_t write;
};

/** @endcond */
/**
 * @brief Read data.
 *
 * @param dev nrf24 instance.
 * @param buf Read buffer.
 *
 * @retval 0 On success.
 * @retval -errno Other negative errno in case of failure.
 */
__syscall int nrf24_read(const struct device *dev, uint8_t *buffer, uint8_t data_len);

static inline int z_impl_nrf24_read(const struct device *dev, uint8_t *buffer, uint8_t data_len)
{
	const struct nrf24_api *api =
		(const struct nrf24_api *)dev->api;

	return api->read(dev, buffer, data_len);
}

/**
 * @brief Write data.
 *
 * @param dev nrf24 instance.
 * @param buf Write buffer.
 *
 * @retval 0 On success.
 * @retval -errno Other negative errno in case of failure.
 */
__syscall int nrf24_write(const struct device *dev, uint8_t *buffer, uint8_t data_len);

static inline int z_impl_nrf24_write(const struct device *dev, uint8_t *buffer, uint8_t data_len)
{
	const struct nrf24_api *api =
		(const struct nrf24_api *)dev->api;

	return api->write(dev, buffer, data_len);
}

#include <syscalls/nrf24.h>
#endif /* ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_WIRELESS_NRF24_H_ */
