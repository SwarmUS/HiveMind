add_subdirectory(bsp)
add_subdirectory(freertos)

set(PROJECT_SOURCES
    main.c
)


add_executable(stm32-freertos.elf ${PROJECT_SOURCES})

target_link_libraries(stm32-freertos.elf
    PUBLIC
    SwarmUS::HiveMind::BSP 
)


# Tests

if (NOT CMAKE_CROSSCOMPILING)
    option(PACKAGE_TESTS "Build the tests" ON)
    if(PACKAGE_TESTS)
        googletest_fetch()
    endif()
endif()
