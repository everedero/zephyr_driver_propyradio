#-------------------------------------------------------------------------------
# Zephyr Example Application
#
# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
set(SYSCALL_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../include")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
include_directories(ui)
project(app LANGUAGES C)
file(GLOB SOURCES src/main.c ui/*.h ui/*.c)
target_sources(app PRIVATE ${SOURCES})
