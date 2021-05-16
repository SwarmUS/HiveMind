if(NOT CMSIS_RTOS_FIND_COMPONENTS)
    set(CMSIS_RTOS_FIND_COMPONENTS ${STM32_SUPPORTED_FAMILIES_LONG_NAME})
endif()

if(STM32H7 IN_LIST CMSIS_RTOS_FIND_COMPONENTS)
    list(REMOVE_ITEM CMSIS_RTOS_FIND_COMPONENTS STM32H7)
    list(APPEND CMSIS_RTOS_FIND_COMPONENTS STM32H7_M7 STM32H7_M4)
endif()
list(REMOVE_DUPLICATES CMSIS_RTOS_FIND_COMPONENTS)

include(stm32/devices)

foreach(COMP ${CMSIS_RTOS_FIND_COMPONENTS})
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

    find_path(CMSIS_${FAMILY}${CORE_U}_CMSIS_RTOS2_PATH
        NAMES cmsis_os2.h
        PATHS "${STM32_CUBE_${FAMILY}_PATH}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2"
        NO_DEFAULT_PATH
    )
    list(APPEND CMSIS_RTOS_INCLUDE_DIRS ${CMSIS_${FAMILY}${CORE_U}_CMSIS_RTOS2_PATH})

    # For some reason, cmsis_os2.c uses signed char* for vApplicationStackOverflowHook() while FreeRTOS uses char*
    # This causes compilation to fail, when using a c++ compiler. This changes this signature before compiling to ensure
    # everything works.
    file(READ "${CMSIS_${FAMILY}${CORE_U}_CMSIS_RTOS2_PATH}/cmsis_os2.c" INPUT_TEXT)
    string(REPLACE "signed char" "char" INPUT_TEXT "${INPUT_TEXT}")
    file(WRITE "${CMSIS_${FAMILY}${CORE_U}_CMSIS_RTOS2_PATH}/cmsis_os2.c" "${INPUT_TEXT}")

    set(LIB_NAME_RTOS "cmsis_stm32_${FAMILY}${CORE_U}_rtos_v2")
    set(LIB_ALIAS_RTOS "CMSIS::STM32::${FAMILY}${CORE_C}::RTOS_V2")

    if(NOT (TARGET CMSIS::STM32::${FAMILY}${CORE_C}::RTOS_V2))
        add_library(${LIB_NAME_RTOS} STATIC "${CMSIS_${FAMILY}${CORE_U}_CMSIS_RTOS2_PATH}/cmsis_os2.c")
        add_library(${LIB_ALIAS_RTOS} ALIAS ${LIB_NAME_RTOS})

        target_include_directories(${LIB_NAME_RTOS} SYSTEM PUBLIC "${CMSIS_${FAMILY}${CORE_U}_CMSIS_RTOS2_PATH}")
        target_link_libraries(${LIB_NAME_RTOS}
            PUBLIC
                SwarmUS::HiveMind::OS
            PRIVATE
                CMSIS::STM32::${DEVICES}${CORE_C}
            )
    endif()

    if (DISABLE_EXTERNAL_WARNINGS)
        target_compile_options(${LIB_NAME_RTOS} PRIVATE -w)
        set_target_properties(${LIB_NAME_RTOS} PROPERTIES CXX_CLANG_TIDY "" )
        set_target_properties(${LIB_NAME_RTOS} PROPERTIES C_CLANG_TIDY "" )
    endif()

    set(CMSIS_RTOS_${COMP}_FOUND true)
endforeach()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CMSIS_RTOS
        REQUIRED_VARS CMSIS_RTOS_INCLUDE_DIRS
        FOUND_VAR CMSIS_RTOS_FOUND
        HANDLE_COMPONENTS
        )
