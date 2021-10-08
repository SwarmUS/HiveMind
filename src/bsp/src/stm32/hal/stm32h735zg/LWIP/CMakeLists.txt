set(LIB_NAME "swarmus-hivemind-lwip")
set(LIB_ALIAS "SwarmUS::HiveMind::LWIP")

# https://stackoverflow.com/questions/59494184/how-to-remove-a-line-of-text-in-a-string-in-cmake-working-around-cmakes-lack-o
function(deleteinplace IN_FILE pattern)
    # create list of lines form the contens of a file
    file (STRINGS ${IN_FILE} LINES)

    # overwrite the file....
    file(WRITE ${IN_FILE} "")

    # loop through the lines,
    # remove unwanted parts
    # and write the (changed) line ...
    foreach(LINE IN LISTS LINES)
        string(REGEX REPLACE ${pattern} "" STRIPPED "${LINE}")
        file(APPEND ${IN_FILE} "${STRIPPED}\n")
    endforeach()
endfunction()

# D-Cache was not activated as it causes problems when saving data to cache
# This line deletes any references to SCB_InvalidateDCache_by_Addr() as it can cause hardfaults at runtime if D-Cache is not activated.
deleteinplace(Target/ethernetif.c "SCB_InvalidateDCache_by_Addr.*")

find_package(CMSIS_RTOS COMPONENTS
        STM32H735ZG_M7
        REQUIRED)

find_package(LWIP COMPONENTS
        STM32H735ZG_M7
        REQUIRED)

find_package(BSP COMPONENTS 
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

target_link_libraries(${LIB_NAME}
    PRIVATE
        CMSIS::STM32::H735xx::M7
        HAL::STM32::H7::M7
        HAL::STM32::H7::M7::ETH

        CMSIS::STM32::H7::M7::RTOS_V2
        CMSIS::STM32::H7::M7::LWIP
        CMSIS::STM32::H7::M7::LWIP::IPV4
        BSP::STM32::H7::M7::LAN8742
        SwarmUS::HiveMind::HAL::Linker::STM32
        SwarmUS::Propolis::C::Common
    )

if (ENABLE_TARGET_IPERF_SERVER OR ENABLE_HARDWARE_TESTS)
    target_link_libraries(${LIB_NAME}
        PRIVATE
            CMSIS::STM32::H7::M7::LWIP::IPERF
    )
endif()

# Removing warnings and compile options
if (DISABLE_EXTERNAL_WARNINGS)
    target_compile_options(${LIB_NAME} PRIVATE -w)
    set_target_properties(${LIB_NAME} PROPERTIES CXX_CLANG_TIDY "" )
    set_target_properties(${LIB_NAME} PROPERTIES C_CLANG_TIDY "" )
endif()
