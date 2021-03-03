if(NOT USB_FIND_COMPONENTS)
    set(USB_FIND_COMPONENTS
            STM32F0 STM32F1 STM32F2 STM32F3 STM32F4 STM32F7
            STM32G0 STM32G4
            STM32H7_M7 STM32H7_M4
            STM32L0 STM32L1 STM32L4
            )
endif()

if(STM32H7 IN_LIST USB_FIND_COMPONENTS)
    list(REMOVE_ITEM USB_FIND_COMPONENTS STM32H7)
    list(APPEND USB_FIND_COMPONENTS STM32H7_M7 STM32H7_M4)
endif()
list(REMOVE_DUPLICATES USB_FIND_COMPONENTS)

include(stm32/devices)

foreach(COMP ${USB_FIND_COMPONENTS})
    string(TOLOWER ${COMP} COMP_L)
    string(TOUPPER ${COMP} COMP)

    string(REGEX MATCH "^STM32([A-Z][0-9])([0-9A-Z][0-9][A-Z][0-9A-Z])?_?(M[47])?.*$" COMP ${COMP})

    if((NOT CMAKE_MATCH_1) AND (NOT CMAKE_MATCH_2))
        message(FATAL_ERROR "Unknown CMSIS component: ${COMP}")
    endif()

    if(CMAKE_MATCH_2)
        set(FAMILY ${CMAKE_MATCH_1})
        set(DEVICES "${CMAKE_MATCH_1}${CMAKE_MATCH_2}")
    else()
        set(FAMILY ${CMAKE_MATCH_1})
        stm32_get_devices_by_family(DEVICES FAMILY ${FAMILY} CORE ${CORE})
    endif()

    if(CMAKE_MATCH_3)
        set(CORE ${CMAKE_MATCH_3})
        set(CORE_C "::${CORE}")
        set(CORE_U "_${CORE}")
    else()
        unset(CORE)
        unset(CORE_C)
        unset(CORE_U)
    endif()

    string(TOLOWER ${FAMILY} FAMILY_L)

    if((NOT STM32_CMSIS_${FAMILY}_PATH) AND (NOT STM32_CUBE_${FAMILY}_PATH))
        set(STM32_CUBE_${FAMILY}_PATH /opt/STM32Cube${FAMILY} CACHE PATH "Path to STM32Cube${FAMILY}")
        message(STATUS "Neither STM32_CUBE_${FAMILY}_PATH nor STM32_CMSIS_${FAMILY}_PATH specified using default  STM32_CUBE_${FAMILY}_PATH: ${STM32_CUBE_${FAMILY}_PATH}")
    endif()

    find_path(CMSIS_${FAMILY}${CORE_U}_USB_CORE_SRC_PATH
        NAMES usbd_core.c
        PATHS "${STM32_CUBE_${FAMILY}_PATH}/Middlewares/ST/STM32_USB_Device_Library/Core/Src"
        NO_DEFAULT_PATH
    )
    find_path(CMSIS_${FAMILY}${CORE_U}_USB_CORE_INC_PATH
        NAMES usbd_core.h
        PATHS "${STM32_CUBE_${FAMILY}_PATH}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc"
        NO_DEFAULT_PATH
    )

    find_path(CMSIS_${FAMILY}${CORE_U}_USB_CDC_SRC_PATH
        NAMES usbd_cdc.c
        PATHS "${STM32_CUBE_${FAMILY}_PATH}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src"
        NO_DEFAULT_PATH
    )
    find_path(CMSIS_${FAMILY}${CORE_U}_USB_CDC_INC_PATH
        NAMES usbd_cdc.h
        PATHS "${STM32_CUBE_${FAMILY}_PATH}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc"
        NO_DEFAULT_PATH
    )

    set(LIB_SOURCES
        ${CMSIS_${FAMILY}${CORE_U}_USB_CORE_SRC_PATH}/usbd_core.c
        ${CMSIS_${FAMILY}${CORE_U}_USB_CORE_SRC_PATH}/usbd_ctlreq.c
        ${CMSIS_${FAMILY}${CORE_U}_USB_CORE_SRC_PATH}/usbd_ioreq.c
        ${CMSIS_${FAMILY}${CORE_U}_USB_CDC_SRC_PATH}/usbd_cdc.c
    )

    set(LIB_HEADERS
        ${CMSIS_${FAMILY}${CORE_U}_USB_CORE_INC_PATH}/usbd_core.h
        ${CMSIS_${FAMILY}${CORE_U}_USB_CORE_INC_PATH}/usbd_ctlreq.h
        ${CMSIS_${FAMILY}${CORE_U}_USB_CORE_INC_PATH}/usbd_def.h
        ${CMSIS_${FAMILY}${CORE_U}_USB_CORE_INC_PATH}/usbd_ioreq.h
        ${CMSIS_${FAMILY}${CORE_U}_USB_CDC_INC_PATH}/usbd_cdc.h
    )

    list(APPEND USB_INCLUDE_DIRS ${CMSIS_${FAMILY}${CORE_U}_USB_CORE_INC_PATH})
    list(APPEND USB_INCLUDE_DIRS ${CMSIS_${FAMILY}${CORE_U}_USB_CDC_INC_PATH})

    set(LIB_NAME_USB "cmsis_stm32_${FAMILY}${CORE_U}_usb")
    set(LIB_ALIAS_USB "CMSIS::STM32::${FAMILY}${CORE_C}::USB")

    if(NOT (TARGET CMSIS::STM32::${FAMILY}${CORE_C}::USB))
        add_library(${LIB_ALIAS_USB} INTERFACE IMPORTED)

        target_sources(${LIB_ALIAS_USB} INTERFACE ${LIB_SOURCES})
        target_include_directories(${LIB_ALIAS_USB}
                SYSTEM INTERFACE
                ${CMSIS_${FAMILY}${CORE_U}_USB_CORE_INC_PATH}
                ${CMSIS_${FAMILY}${CORE_U}_USB_CDC_INC_PATH}
            )
    endif()

    set(USB_${COMP}_FOUND true)
endforeach()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(USB
        REQUIRED_VARS USB_INCLUDE_DIRS
        FOUND_VAR USB_FOUND
        HANDLE_COMPONENTS
        )
