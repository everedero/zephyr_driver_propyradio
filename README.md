# Zephyr out-of-tree driver for nRF24L01 SPI 2.4GHz remote control module

This repository contains a Zephyr out-of-tree driver.

This repository is versioned together with the [Zephyr main tree][zephyr]. This
means that every time that Zephyr is tagged, this repository is tagged as well
with the same version number, and the [manifest](west.yml) entry for `zephyr`
will point to the corresponding Zephyr tag. For example, the `example-application`
v2.6.0 will point to Zephyr v2.6.0. Note that the `main` branch always
points to the development branch of Zephyr, also `main`.

[bindings]: https://docs.zephyrproject.org/latest/guides/dts/bindings.html
[drivers]: https://docs.zephyrproject.org/latest/reference/drivers/index.html
[zephyr]: https://github.com/zephyrproject-rtos/zephyr
[west_ext]: https://docs.zephyrproject.org/latest/develop/west/extensions.html

## Getting Started

Before getting started, make sure you have a proper Zephyr development
environment. Follow the official
[Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/getting_started/index.html).

### Initialization

The first step is to initialize the workspace folder (``my-workspace``) where
the application and all Zephyr modules will be cloned. Run the following
command:

```shell
# initialize my-workspace for the example-application (main branch)
west init -m https://github.com/everedero/driver_nrf24l01 --mr main my-workspace
# update Zephyr modules
cd my-workspace
west update
```

This has been tested with Zephyr 3.5.99

### Building and running

To build the application, run the following command:

```shell
west build -b $BOARD -p always app -- -DOVERLAY_CONFIG=prj.conf
```

where `$BOARD` is the target board.
```shell
BOARD="nucleo_f756zg"
BOARD="nrf52dk_nrf52832"
BOARD="esp32_devkitc_wroom"
```

In order to activate debug logs:
```shell
west build -b $BOARD -p always app -- -DOVERLAY_CONFIG=debug.conf
```

Once you have built the application, run the following command to flash it:

```shell
west flash
```

### Testing

To execute Twister integration tests, run the following command:

```shell
west twister -T tests --integration
```
