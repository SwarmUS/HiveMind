set(LIB_NAME "swarmus-hivemind-lwip")
set(LIB_ALIAS "SwarmUS::HiveMind::LWIP")

find_package(CMSIS_RTOS COMPONENTS
        STM32H735ZG_M7
        REQUIRED)

find_package(LWIP COMPONENTS
        STM32H735ZG_M7
        REQUIRED)

set(LIB_SOURCES
    App/lwip.c
    Target/ethernetif.c
)

set(LIB_HEADERS
    App/lwip.h
)

add_library(${LIB_NAME} 
    STATIC
        ${LIB_SOURCES}
        ${LIB_HEADERS}
)
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
    SYSTEM PUBLIC
        Target

    SYSTEM PRIVATE
        ../Core/Inc

    SYSTEM PUBLIC
        $<INSTALL_INTERFACE:include>
        ${lwip_INC}/
)

get_target_property(LINKER_SOURCES BSP::STM32::H7::M7::LAN8742 INTERFACE_SOURCES)
message(WOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOW ${LINKER_SOURCES})

target_link_libraries(${LIB_NAME}
    PRIVATE
        CMSIS::STM32::H735ZG::M7
        HAL::STM32::H7::M7

        CMSIS::STM32::H7::M7::RTOS_V2
        CMSIS::STM32::H7::M7::LWIP
        CMSIS::STM32::H7::M7::LWIP::IPV4
        BSP::STM32::H7::M7::LAN8742
        SwarmUS::HiveMind::HAL::Linker::STM32
        SwarmUS::Propolis::C::Common
    )

if (ENABLE_TARGET_IPERF_SERVER)
    target_link_libraries(${LIB_NAME}
        PRIVATE
            CMSIS::STM32::H7::M7::LWIP::IPERF
    )
endif()

# Removing warnings and compile options
if (DISABLE_EXTERNAL_WARNINGS)
    target_compile_options(${LIB_NAME} PRIVATE -w)
endif()
