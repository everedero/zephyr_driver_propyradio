#-------------------------------------------------------------------------------
# Zephyr Example Application
#
# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
set(SYSCALL_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../include")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C)

target_sources(app PRIVATE src/main.c)

# Stored program: duration;mode;power CSV (header line, empty duration = forever) -> C table
set(PROGRAM_CSV ${CMAKE_CURRENT_SOURCE_DIR}/programs/${CONFIG_CCAPP_PROGRAM})
if("${CONFIG_CCAPP_PROGRAM}" STREQUAL "" OR NOT EXISTS ${PROGRAM_CSV} OR IS_DIRECTORY ${PROGRAM_CSV})
  file(GLOB available RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/programs ${CMAKE_CURRENT_SOURCE_DIR}/programs/*.csv)
  message(FATAL_ERROR "CONFIG_CCAPP_PROGRAM='${CONFIG_CCAPP_PROGRAM}' not found in programs/, available: ${available}")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${PROGRAM_CSV})
file(READ ${PROGRAM_CSV} csv)
string(REPLACE ";" "," csv "${csv}")
string(REPLACE "\r" "" csv "${csv}")
string(REPLACE "\n" ";" rows "${csv}")
list(POP_FRONT rows)
set(inc "")
foreach(row IN LISTS rows)
  string(STRIP "${row}" row)
  if(row STREQUAL "")
    continue()
  endif()
  string(REGEX MATCH "^([0-9]*),([a-z-]+),([0-9]+)$" ok "${row}")
  if(NOT ok)
    message(FATAL_ERROR "${PROGRAM_CSV}: bad row '${row}'")
  endif()
  set(dur "${CMAKE_MATCH_1}")
  string(TOUPPER "${CMAKE_MATCH_2}" m)
  string(REPLACE "-" "_" m "${m}")
  if("${dur}" STREQUAL "")
    set(dur 0)
  endif()
  string(APPEND inc "{${dur}, ${m}, ${CMAKE_MATCH_3}},\n")
endforeach()
file(CONFIGURE OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/program.inc CONTENT "${inc}")
target_include_directories(app PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
