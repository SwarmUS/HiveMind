cmake_minimum_required(VERSION 3.13)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Options
option(ENABLE_WARNINGS "Enable all warnings on build" ON)
option(ENABLE_WARNINGS_AS_ERROR "Enable all warnings as errors" ON)
option(ENABLE_TESTS "Build the tests" ON)
option(ENABLE_CLANG_TIDY_CHECK "Enable clang tidy check on compilation" ON)

option(DISABLE_EXTERNAL_WARNINGS "Disables warnings from exernal libs (gtest, freertos, etc)" ON)

# Init compile variable
set(COMPILE_STM32_F429ZI 0)

# Define project
project(hive-mind CXX C ASM)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

# Warning flags constants
set(WARNING_FLAG -Wall -Wextra -Wconversion -Wsign-conversion -Wno-unused-variable -Wpedantic)
set(WARNING_AS_ERROR_FLAG -Werror)

# Includes
include(clang-tools/clang-tidy)
include(clang-tools/clang-format)
include(doc/doxygen)

# Testing
if(ENABLE_TESTS)
    include(googletest/common)
    enable_testing()
    googletest_get_populate()
endif()

# Include src to compile
add_subdirectory(src)


