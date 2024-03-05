/*
 */
#ifndef ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_WIRELESS_PROPY_RADIO_H_
#define ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_WIRELESS_PROPY_RADIO_H_

#include <zephyr/device.h>
#include <zephyr/toolchain.h>

/** @cond INTERNAL_HIDDEN */

typedef int (*propy_radio_read_t)(const struct device *dev, uint8_t *buffer, uint8_t data_len);
typedef int (*propy_radio_write_t)(const struct device *dev, uint8_t *buffer, uint8_t data_len);

__subsystem struct propy_radio_api {
	propy_radio_read_t read;
	propy_radio_write_t write;
};

/** @endcond */
/**
 * @brief Read data.
 *
 * @param dev propy_radio instance.
 * @param buf Read buffer.
 * @param data_len Number of bytes to read
 *
 * @retval 0 On success.
 * @retval -EIO Nothing in RX queue (trigger mode)
 * @retval -errno Other negative errno in case of failure.
 */
__syscall int propy_radio_read(const struct device *dev, uint8_t *buffer, uint8_t data_len);

static inline int z_impl_propy_radio_read(const struct device *dev, uint8_t *buffer, uint8_t data_len)
{
	const struct propy_radio_api *api =
		(const struct propy_radio_api *)dev->api;

	return api->read(dev, buffer, data_len);
}

/**
 * @brief Write data.
 *
 * @param dev propy_radio instance.
 * @param buf Write buffer.
 * @param data_len Number of bytes to write
 *
 * @retval 0 On success.
 * @retval -ETIME TX sending timed out (trigger mode)
 * @retval -EIO Max retries exceeded (polling mode)
 * @retval MAX_RT Message not acknowledged (trigger mode)
 * @retval -errno Other negative errno in case of failure.
 */
__syscall int propy_radio_write(const struct device *dev, uint8_t *buffer, uint8_t data_len);

static inline int z_impl_propy_radio_write(const struct device *dev, uint8_t *buffer, uint8_t data_len)
{
	const struct propy_radio_api *api =
		(const struct propy_radio_api *)dev->api;

	return api->write(dev, buffer, data_len);
}

#include <syscalls/propy_radio.h>
#endif /* ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_WIRELESS_PROPY_RADIO_H_ */
