set(LIB_NAME "swarmus-hivemind-hal-stm32f429zi-usb")
set(LIB_ALIAS "SwarmUS::HiveMind::HAL::STM32F4::USB")

find_package(USB COMPONENTS
        STM32F429ZI
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
        HAL::STM32::F4
        HAL::STM32::F4::PCDEx
        HAL::STM32::F4::LL_USB
        CMSIS::STM32::F4::USB
    )

# Removing warnings and compile options
if (DISABLE_EXTERNAL_WARNINGS)
    target_compile_options(${LIB_NAME} PRIVATE -w)
    set_target_properties(${LIB_NAME} PROPERTIES CXX_CLANG_TIDY "" )
    set_target_properties(${LIB_NAME} PROPERTIES C_CLANG_TIDY "" )
endif()
