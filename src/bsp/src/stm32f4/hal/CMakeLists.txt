set(LIB_HAL_LINKER_ALIAS "SwarmUS::HiveMind::HAL::Linker")

# Sources that needs to be global on the project
# Ex: interrupts, syscalls, callbacks
set(GLOBAL_SOURCES
    src/uart_print.c
    src/syscalls.c
)


# Creating the interface library
add_library(stm32f4-linker INTERFACE)
add_library(${LIB_BSP_LINKER_ALIAS} ALIAS stm32f4-linker) # Provide alias for linker options

target_sources(stm32f4-linker
    INTERFACE
        ${GLOBAL_SOURCES})

target_link_libraries(stm32f4-linker
    INTERFACE
        SwarmUS::HiveMind::HAL::Linker)

if (${COMPILE_STM32_F429ZI})
    add_subdirectory(stm32f429zi)
else()
    message(FATAL_ERROR "The current board is not supported")
endif()
