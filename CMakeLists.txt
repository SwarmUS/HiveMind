cmake_minimum_required(VERSION 3.13)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)


include(freertos/common)
include(googletest/common)

option(CROSS_COMPILE "CrossCompile" 0)

set(COMPILE_STM32 0)
set(COMPILE_NATIVE 0)


if (CROSS_COMPILE)
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/stm32_gcc.cmake)
    set(COMPILE_STM32 1)
else()
    set(COMPILE_NATIVE 1)
endif()


project(hive-mind CXX C ASM)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

enable_testing()

add_subdirectory(src)

