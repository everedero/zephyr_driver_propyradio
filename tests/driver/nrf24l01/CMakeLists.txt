# Copyright (c) 2021, Legrand North America, LLC.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
set(SYSCALL_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../../../include")
# Use the project local board root folder
set(BOARD_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../app")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C)

target_sources(app PRIVATE src/main.c)
