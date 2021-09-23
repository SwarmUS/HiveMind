if (${COMPILE_STM32_H735ZG})
    set(HARDWARE_TESTS_EXECUTABLE "hardware_tests.elf")
    set(HARDWARE_TESTS_SOURCES HardwareTests.cpp)

    add_executable(${HARDWARE_TESTS_EXECUTABLE} ${HARDWARE_TESTS_SOURCES})
    target_link_libraries(${HARDWARE_TESTS_EXECUTABLE}
            PUBLIC
            SwarmUS::HiveMind::BSP::Linker
            SwarmUS::HiveMind::BSP
            SwarmUS::HiveMind::OS
            SwarmUS::HiveMind::Logger
            SwarmUS::HiveMind::BSP::Platform::Tests
            )

    set_target_properties(
            ${HARDWARE_TESTS_EXECUTABLE}
            PROPERTIES CXX_CLANG_TIDY ""
    )
    add_custom_target(
            flash_tests
            COMMAND openocd  -f ${OPENOCD_CFG_PATH} -c 'program $<TARGET_FILE:${HARDWARE_TESTS_EXECUTABLE}> verify reset exit'
    )
endif()

if (DECAWAVE_TESTS)
    if (${DECAWAVE_TESTS} STREQUAL "TX")
        add_definitions(-DDECA_TEST_TX)
    elseif(${DECAWAVE_TESTS} STREQUAL "RX")
        add_definitions(-DDECA_TEST_RX)
    elseif(${DECAWAVE_TESTS} STREQUAL "SPI")
        add_definitions(-DDECA_TEST_SPI)
    elseif(${DECAWAVE_TESTS} STREQUAL "IDLE")
        add_definitions(-DDECA_TEST_IDLE)
    elseif(${DECAWAVE_TESTS} STREQUAL "LED")
        add_definitions(-DDECA_TEST_LED)
    endif()

    set(DECAWAVE_TESTS_EXECUTABLE "decawave_tests.elf")
    set(DECAWAVE_TESTS_SOURCES DecawaveTests.cpp)

    add_executable(${DECAWAVE_TESTS_EXECUTABLE} ${DECAWAVE_TESTS_SOURCES})
    target_link_libraries(${DECAWAVE_TESTS_EXECUTABLE}
            PUBLIC
            SwarmUS::HiveMind::BSP::Linker
            SwarmUS::HiveMind::BSP
            SwarmUS::HiveMind::BSP::Platform::Interloc
            SwarmUS::HiveMind::OS
            SwarmUS::HiveMind::Logger
            )

    set_target_properties(
            ${DECAWAVE_TESTS_EXECUTABLE}
            PROPERTIES CXX_CLANG_TIDY ""
    )
    add_custom_target(
            flash_deca_tests
            COMMAND openocd  -f ${OPENOCD_CFG_PATH} -c 'program $<TARGET_FILE:${DECAWAVE_TESTS_EXECUTABLE}> verify reset exit'
    )
endif()