#zephyr_library()
zephyr_library_sources_ifdef(CONFIG_NRF24L01 nrf24l01.c)
