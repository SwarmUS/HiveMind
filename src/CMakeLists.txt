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
      stm32f4-linker
      #SwarmUS::HiveMind::BSP::Linker
      SwarmUS::HiveMind::BSP 
      SwarmUS::HiveMind::FreeRTOS
)
target_compile_options(${EXECUTABLE_NAME}
    PRIVATE
        $<$<BOOL:${ENABLE_WARNINGS}>:${WARNING_FLAG}>
        $<$<BOOL:${ENABLE_WARNINGS_AS_ERROR}>:${WARNING_AS_ERROR_FLAG}>
)

add_subdirectory(bsp)
add_subdirectory(freertos)
add_subdirectory(logger)


# Removing warnings from freertos compilation on executable target
if (DISABLE_EXTERNAL_WARNINGS) 
    disable_freertos_warnings()
endif()
