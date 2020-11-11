
set(PROJECT_SOURCES
    main.c
)


add_executable(${CMAKE_PROJECT_NAME}.elf ${PROJECT_SOURCES})

target_link_libraries(${CMAKE_PROJECT_NAME}.elf
    PUBLIC
    SwarmUS::HiveMind::BSP 
)

add_subdirectory(bsp)
add_subdirectory(freertos)
