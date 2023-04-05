cmake_minimum_required(VERSION 3.5.1)
# Assembler files (.s) in the source list are ignored by CMake unless we
# “enable” the assembler by telling CMake in the project definition that we’re
# using assembly files.
project(H745_TEST C ASM)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Configs" FORCE)
if("${CMAKE_BUILD_TYPE}" STREQUAL "")
  message(STATUS "CMAKE_BUILD_TYPE was not set. Giving default value: Release")
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE Release)
endif()

option(WITH_USB_MIDDLEWARE "Build with STM USB Middleware." OFF)
if(WITH_USB_MIDDLEWARE)
  add_definitions(-DWITH_USB_MIDDLEWARE)
  message(STATUS "Building with STM USB MIddlwware.")
endif()

add_subdirectory(CM4)
add_subdirectory(CM7)
