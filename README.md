# RC Airplane remote controller applicaation based on Zephyr

This repository contains a Zephyr based application.
The application deals with a RC plane remote controller.
It is built on 'stm32f469i-disco' board with a specific custom shield.

This shield interconnects: 
1. a nRF24L01 module for the radio part
2. 2 joysticks and 4 associated trim buttons
3. additionnal on/off, push buttons and 1 variator

The application uses Zephyr as ecosystem with ['LVGL'](https://lvgl.io) as graphical framework.
For the GUI (Graphical User Interface) ['EEZ Studio'](https://www.envox.eu/studio/studio-introduction/) has been used to 
generate code (available in /ui/ directory)

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

The application starts from a fork of 'https://github.com/everedero/driver_nrf24l01' out of tree nRF24 Zephyr driver test application
'

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
west init -m https://github.com/phildefer/rc-remote-controller --mr main my-workspace
# update Zephyr modules
cd my-workspace
west update
```

This has been tested with Zephyr 4.4

### Building and running

To build the application, run the following command:

```shell
west build -p always -b stm32f469i_disco ./app/
```

Once you have built the application, run the following command to flash it:

```shell
west flash
```

For more detailed information, see the [example app Readme](app/README.md)

### Testing
To do

# Modules & SW components
This repository is organized into the following top-level directories:

- `app/`: main application source tree, board configuration, app Kconfig, and generated GUI code in `app/ui/`.
- `build/`: generated build output and intermediate CMake artifacts produced by `west build`.
- `drivers/`: custom and out-of-tree driver code, including the nRF24L01 integration used by this controller.
- `dts/`: device tree source files and binding information used for hardware configuration.
- `include/`: public header files for application code and shared interfaces.
- `lib/`: reusable libraries and helper modules used by the application.
- `scripts/`: helper utilities, example west commands, and build-related scripts.
- `tests/`: driver and application test cases for verifying functionality.
- `zephyr/`: the Zephyr RTOS module tree checked out by `west` that contains the underlying OS, board support, and subsystems.

Within `app/src/`, `main.c` is the main entry point and runtime orchestrator for the controller. It:

- initializes the LVGL UI and the display, including startup progress updates.
- configures the PCF8575 IO expander and handles GPIO input interrupts with debounce processing.
- reads ADC channels periodically to sample joystick/trim inputs.
- maps ADC input values into radio channel values and builds NRF24L01+ payloads.
- manages radio connection state, handles reconnection logic, and updates UI indicators for binding and error status.
- controls the buzzer thread for startup and alert tones.

This means the `app/` directory contains both the application logic and the hardware interaction glue that connects UI, ADC, GPIO, and radio transmission in a single Zephyr-based firmware package.

# Troubleshooting
