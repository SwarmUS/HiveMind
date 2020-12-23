cmake_minimum_required(VERSION 3.13)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Options
option(ENABLE_WARNINGS "Enable all warnings on build" ON)
option(ENABLE_WARNINGS_AS_ERROR "Enable all warnings as errors" ON)
option(ENABLE_TESTS "Build the tests" ON)
option(ENABLE_CLANG_TIDY_CHECK "Enable clang tidy check on compilation" ON)
option(ENABLE_ERROR_ON_MISSING_TOOL "If a tools (clang-tidy, clang-format, doxygen) is not disabled and not installed, throws error" ON)
option(DISABLE_EXTERNAL_WARNINGS "Disables warnings from exernal libs (gtest, freertos, etc)" ON)

# Init compile variable
set(COMPILE_STM32_F429ZI 0)

# Define project
project(hive_mind CXX C ASM)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

# Warning flags constants
set(WARNING_FLAG -Wall -Wextra -Wsign-conversion -Wuninitialized  -Wunused-variable -Wpedantic )
set(WARNING_AS_ERROR_FLAG -Werror)

# Generator expression doesn't work with clang-tidy, we need to make an IF statement manually
if(ENABLE_WARNINGS)
add_compile_options(${WARNING_FLAG})
endif()

if(ENABLE_WARNINGS_AS_ERROR)
  add_compile_options(${WARNING_AS_ERROR_FLAG})
endif()

# Includes
include(ros/generators.cmake)
include(catkin/common)
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


