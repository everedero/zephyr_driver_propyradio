/*
* Copyright (c) 2024 Eve Redero
* SPDX-License-Identifier: Apache-2.0
*/

/*
* @file test nrf24l01 driver
*
* This suite verifies that the driver builds correctly
* on selected target
*/

#include <zephyr/ztest.h>

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>

#include <app/drivers/propy_radio.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_NRF24L01_LOG_LEVEL);

#if !DT_NODE_EXISTS(DT_NODELABEL(radio0))
#error "whoops, node label radio0 not found"
#endif

ZTEST(nrf24l01, test_get_value)
{
	static const struct device *nrf24 = DEVICE_DT_GET(DT_NODELABEL(radio0));
	zassert_equal(1, 1, "Dummy test fail");
    zassert_not_null(nrf24, "Device pointer is null");
    zassert_not_null(nrf24->name, "Device name is null");
}

ZTEST_SUITE(nrf24l01, NULL, NULL, NULL, NULL, NULL);
