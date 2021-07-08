if (${COMPILE_STM32_H735ZG})
    set(HARDWARE_TESTS_EXECUTABLE "hardware_tests.elf")
    set(HARDWARE_TESTS_SOURCES DecawaveTxTest.cpp)

    add_executable(${HARDWARE_TESTS_EXECUTABLE} ${HARDWARE_TESTS_SOURCES})
    target_link_libraries(${HARDWARE_TESTS_EXECUTABLE}
            PUBLIC
            SwarmUS::HiveMind::BSP::Linker
            SwarmUS::HiveMind::BSP
            SwarmUS::HiveMind::OS
            SwarmUS::HiveMind::Logger
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