set(EXECUTABLE_NAME ${CMAKE_PROJECT_NAME})
if(${CMAKE_CROSSCOMPILING})
    set(EXECUTABLE_NAME ${EXECUTABLE_NAME}.elf)
endif()

set(PROJECT_SOURCES
    main.cpp
)

add_executable(${EXECUTABLE_NAME} ${PROJECT_SOURCES})
target_link_libraries(${EXECUTABLE_NAME}
    PUBLIC
      SwarmUS::HiveMind::BSP::Linker
      SwarmUS::HiveMind::BSP 
      SwarmUS::HiveMind::FreeRTOS
      SwarmUS::HiveMind::Logger
)

target_compile_options(${EXECUTABLE_NAME}
    PUBLIC
        $<$<BOOL:${ENABLE_WARNINGS}>:${WARNING_FLAG}>
        $<$<BOOL:${ENABLE_WARNINGS_AS_ERROR}>:${WARNING_AS_ERROR_FLAG}>
)

include(freertos/common)
freertos_fetch_kernel()


add_subdirectory(bsp)
add_subdirectory(logger)
add_subdirectory(freertos)

# Removing warnings from freertos compilation on executable target
if (DISABLE_EXTERNAL_WARNINGS) 
    # Since set_source_files_properties is scoped only to targets in the curremt CMakeLists.txt
    # we need to add it everywhere...
    disable_freertos_warnings()
endif()

