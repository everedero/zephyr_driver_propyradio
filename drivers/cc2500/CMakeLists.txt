#zephyr_library()
zephyr_library_sources_ifdef(CONFIG_CC2500 cc2500.c)
