#-------------------------------------------------------------------------------
# Zephyr Example Application
#
# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
# Use the project local board root folder
set(BOARD_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../boards/nucleo_spi.overlay")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C)

target_sources(app PRIVATE src/main.c)
