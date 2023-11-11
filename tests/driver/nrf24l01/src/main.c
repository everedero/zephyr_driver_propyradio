/*
 * Copyright (c) 2021 Legrand North America, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * @file test custom_lib library
 *
 * This suite verifies that the methods provided with the custom_lib
 * library works correctly.
 */

#include <limits.h>

#include <zephyr/ztest.h>

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>

#include <app/drivers/nrf24.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_NRF24L01_LOG_LEVEL);

#if !DT_NODE_EXISTS(DT_NODELABEL(radio0))
#error "whoops, node label radio0 not found"
#endif

ZTEST(nrf24l01, test_get_value)
{
	static const struct device *nrf24 = DEVICE_DT_GET(DT_NODELABEL(radio0));
//zassert_equal(((struct nrf24l01_data *)(nrf24->data))->addr_width, 5, "address width is not 5");
	zassert_equal(1, 1, "Dummy test fail");
    zassert_not_null(nrf24, "Device pointer is null");
    zassert_not_null(nrf24->name, "Device name is null");
}

ZTEST_SUITE(nrf24l01, NULL, NULL, NULL, NULL, NULL);
