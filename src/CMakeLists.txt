add_subdirectory(freertos)
add_subdirectory(bsp)

set(PROJECT_SOURCES
    main.c
)

add_executable(stm32-freertos.elf ${PROJECT_SOURCES})

target_link_libraries(stm32-freertos.elf
    PRIVATE
       SwarmUS::HiveMind::BSP 
)

