add_subdirectory(freertos)
add_subdirectory(bsp)

set(PROJECT_SOURCES
    main.c
)


if (CMAKE_CROSSCOMPILING)
    add_executable(stm32-freertos.elf ${PROJECT_SOURCES})

    target_link_libraries(stm32-freertos.elf
        PRIVATE
        SwarmUS::HiveMind::BSP 
    )

    target_compile_options(stm32-freertos.elf
        PUBLIC
            $<$<C_COMPILER_ID:GNU>:--specs=nosys.specs>
    )
    target_link_options(stm32-freertos.elf
        PUBLIC
            $<$<C_COMPILER_ID:GNU>:--specs=nosys.specs>
    )
endif()

# Tests
#
#
if (NOT CMAKE_CROSSCOMPILING)
    option(PACKAGE_TESTS "Build the tests" ON)
    if(PACKAGE_TESTS)
        googletest_fetch()
    endif()
endif()
