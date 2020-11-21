cmake_minimum_required(VERSION 3.13)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

option(CROSS_COMPILE "Cross compile to stm32" 0)
option(ENABLE_TESTS "Build the tests" ON)

set(COMPILE_STM32_F429ZI 0)

set(WARNING_FLAG -Wall -Wextra -Wconversion -Wsign-conversion -Wno-unused-variable -pedantic)
set(ERROR_WARNING_FLAG -Werror)

include(freertos/common)

project(hive-mind CXX C ASM)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

if(ENABLE_TESTS)
    enable_testing()
    include(googletest/common)
    googletest_get_populate()
endif()


add_subdirectory(src)

