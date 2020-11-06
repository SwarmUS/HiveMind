cmake_minimum_required(VERSION 3.13)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)


include(freertos/common)
include(googletest/common)

option(CrossCompile "CrossCompile" OFF)

if (CrossCompile)
    include(stm32/common)
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/stm32_gcc.cmake)
endif()


enable_testing()

project(hive-mind CXX C ASM)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

add_subdirectory(src)

