set(LIB_NAME "swarmus-hivemind-hal-stm32h735zg-usb")
set(LIB_ALIAS "SwarmUS::HiveMind::HAL::STM32H7::USB")

find_package(USB COMPONENTS
        STM32H735ZG_M7
        REQUIRED)

set(LIB_SOURCES
    App/usb_device.c
    App/usbd_cdc_if.c
    App/usbd_desc.c
    Target/usbd_conf.c
    App/hivemind_usb_hal.h
)

add_library(${LIB_NAME} STATIC ${LIB_SOURCES})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
    SYSTEM PUBLIC
        Target
        App

    SYSTEM PRIVATE
        ../Core/Inc

    SYSTEM PUBLIC
        ${USB_INCLUDE_DIRS}
)

target_link_libraries(${LIB_NAME}
    PRIVATE
        Swarmus::CMSIS::Startup
        HAL::STM32::H7::M7
        HAL::STM32::H7::M7::PCDEx
        HAL::STM32::H7::M7::LL_USB
        CMSIS::STM32::H7::M7::USB
    )

# Removing warnings and compile options
if (DISABLE_EXTERNAL_WARNINGS)
    target_compile_options(${LIB_NAME} PRIVATE -w)
    set_target_properties(${LIB_NAME} PROPERTIES CXX_CLANG_TIDY "" )
    set_target_properties(${LIB_NAME} PROPERTIES C_CLANG_TIDY "" )
endif()
