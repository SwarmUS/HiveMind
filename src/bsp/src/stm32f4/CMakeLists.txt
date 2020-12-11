set(LIB_NAME "swarmus-hivemind-bsp-stm32")
set(LIB_ALIAS "SwarmUS::HiveMind::BSP::Platform")

set(LIB_SOURCES
    src/bsp.cpp
    src/ui_impl.cpp
)

set(LIB_PRIVATE_HEADERS
)


add_library(${LIB_NAME}
  ${LIB_SOURCES}
  ${LIB_PRIVATE_HEADERS}
)

add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME} 
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        ${BSP_CMAKE_DIR}/include
)

target_link_libraries(${LIB_NAME}
    PUBLIC
        # Defined libraries
        SwarmUS::HiveMind::FreeRTOS

    PRIVATE
        SwarmUS::HiveMind::BSP::STM32F4

)

add_custom_target(
    flash
    COMMAND openocd  -f ${OPENOCD_CFG_PATH} -c 'program $<TARGET_FILE:${EXECUTABLE_NAME}> verify exit'
)

add_subdirectory(hal)

