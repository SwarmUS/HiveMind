set(EXECUTABLE_NAME ${CMAKE_PROJECT_NAME})
if(${CMAKE_CROSSCOMPILING})
    set(EXECUTABLE_NAME ${EXECUTABLE_NAME}.elf)
endif()

add_subdirectory(bsp)
add_subdirectory(freertos)

set(PROJECT_SOURCES
    main.c
)

add_executable(${EXECUTABLE_NAME} ${PROJECT_SOURCES})
target_link_libraries(${EXECUTABLE_NAME}
    PUBLIC
    SwarmUS::HiveMind::BSP 
)

