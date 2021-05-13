set(LIB_NAME swarmus-hivemind-hal)
set(LIB_ALIAS "SwarmUS::HiveMind::HAL")
set(LIB_HAL_LINKER_ALIAS "SwarmUS::HiveMind::HAL::Linker::STM32")

if (${COMPILE_STM32_F429ZI})
    add_subdirectory(stm32f429zi)
elseif (${COMPILE_STM32_H735ZG})
    add_subdirectory(stm32h735zg)
else()
    message(FATAL_ERROR "STM PORT: The current board is not supported" )
endif()

add_subdirectory(decawave)

set(LIB_SOURCES
    src/hal.c
    src/usb.c
    src/hal_flash.c
)

set(LIB_HEADERS
    include/hal/hal.h
    include/hal/uart_print.h
    include/hal/uart.h
    include/hal/hal_spi.h
    include/hal/esp_spi.h
    include/hal/hal_gpio.h
    include/hal/usb.h
    include/hal/hal_flash.h)

# Sources that needs to be global on the project
# Ex: interrupts, syscalls, callbacks
set(GLOBAL_SOURCES
    src/uart_print.c
    src/uart.c
    src/syscalls.c
    src/esp_spi.c
    src/hal_gpio.c
    src/hal_spi.c
)

# Creating the interface library
add_library(stm32-linker INTERFACE)
add_library(${LIB_BSP_LINKER_ALIAS} ALIAS stm32-linker) # Provide alias for linker options
get_target_property(LINKER_SOURCES SwarmUS::HiveMind::HAL::Linker::STM32 INTERFACE_SOURCES)

target_sources(stm32-linker
    INTERFACE
        ${GLOBAL_SOURCES}
        ${LINKER_SOURCES})

target_link_libraries(stm32-linker
    INTERFACE
        SwarmUS::HiveMind::HAL::Decawave
        SwarmUS::Propolis::C::Common)

target_include_directories(stm32-linker
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
        SwarmUS::HiveMind::HAL::STM32
        SwarmUS::HiveMind::HAL::Decawave
    PRIVATE
        stm32-linker
)

target_include_directories(${LIB_NAME} 
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        src
)

# Disable compiler warnings for couple of files 
if(DISABLE_EXTERNAL_WARNINGS)
    set_source_files_properties(
        ${GLOBAL_SOURCES}
        PROPERTIES COMPILE_FLAGS
            -w #Disables all warnings
    )
    target_compile_options(${LIB_NAME} PRIVATE -w)
endif()
