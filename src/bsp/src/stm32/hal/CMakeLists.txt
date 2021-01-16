set(LIB_NAME swarmus-hivemind-hal)
set(LIB_ALIAS "SwarmUS::HiveMind::HAL")
set(LIB_HAL_LINKER_ALIAS "SwarmUS::HiveMind::HAL::Linker::STM32F4")


set(LIB_SOURCES
    src/hal.c
)

set(LIB_HEADERS
    include/hal/hal.h
    include/hal/uart_print.h
        include/hal/uart_phone.h)

# Sources that needs to be global on the project
# Ex: interrupts, syscalls, callbacks
set(GLOBAL_SOURCES
    src/uart_print.c
    src/uart.c
    src/syscalls.c
    src/uart_phone.c
)

# Creating the interface library
add_library(stm32f4-linker INTERFACE)
add_library(${LIB_BSP_LINKER_ALIAS} ALIAS stm32f4-linker) # Provide alias for linker options

target_sources(stm32f4-linker
    INTERFACE
        ${GLOBAL_SOURCES})

target_link_libraries(stm32f4-linker
    INTERFACE
        SwarmUS::HiveMind::HAL::Linker::STM32F4
        SwarmUS::HiveMind::BSP::Common)

target_include_directories(stm32f4-linker
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Actual Hal lib used in the BSP
add_library(${LIB_NAME} STATIC 
    ${LIB_SOURCES}
    ${LIB_HEADERS}
)
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_link_libraries(${LIB_NAME}
    PUBLIC
        SwarmUS::HiveMind::HAL::STM32F4
    PRIVATE
        stm32f4-linker
)

target_include_directories(${LIB_NAME} 
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        src
)


if (${COMPILE_STM32_F429ZI})
    add_subdirectory(stm32f429zi)
else()
    message(FATAL_ERROR "The current board is not supported")
endif()


# Disable compiler warnings and clang-tidy for couple of files 
if(DISABLE_EXTERNAL_WARNINGS)
    set_source_files_properties(
        ${GLOBAL_SOURCES}
        PROPERTIES COMPILE_FLAGS
            -w #Disables all warnings
    )

    set_target_properties(
        ${LIB_NAME}
        PROPERTIES C_CLANG_TIDY ""
    )
endif()
