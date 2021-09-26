set(LIB_NAME "swarmus-hivemind-os-freertos")
set(LIB_ALIAS "SwarmUS::HiveMind::OS::FreeRTOS")

set(LIB_NAME_OS "swarmus-hivemind-os")
set(LIB_ALIAS_OS "SwarmUS::HiveMind::OS")


include(freertos/common)
freertos_fetch_kernel()

add_subdirectory(platforms)

set(LIB_SOURCES
    freertos_hooks.c
 )

set(LIB_HEADERS
    include/freertos/FreeRTOSConfig.h
)

add_library(${LIB_NAME} STATIC ${LIB_SOURCES} ${LIB_HEADERS})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

find_package(FreeRTOS COMPONENTS
            NONE
            REQUIRED)

target_include_directories(${LIB_NAME}
    PUBLIC
       $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>

       # Allows freertos to build properly
       $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/freertos>
       $<INSTALL_INTERFACE:include>
       ${FREERTOS_KERNEL_PATH}/include
)

target_link_libraries(${LIB_NAME}
    PUBLIC
        SwarmUS::HiveMind::FreeRTOS::Platform::Config
        SwarmUS::Propolis::OS::Common
        SwarmUS::HiveMind::BSP # Makes sur the external symbols are present

    PRIVATE    
        SwarmUS::HiveMind::FreeRTOS::Platform

        FreeRTOS::Coroutine
        FreeRTOS::EventGroups
        FreeRTOS::StreamBuffer
        FreeRTOS::Timers
        FreeRTOS::Heap::3

)

# Removing warnings from freertos compilation on executable target
if (DISABLE_EXTERNAL_WARNINGS) 
    target_compile_options(${LIB_NAME} PRIVATE -w)
    set_target_properties(${LIB_NAME} PROPERTIES CXX_CLANG_TIDY "" )
    set_target_properties(${LIB_NAME} PROPERTIES C_CLANG_TIDY "" )
endif()

# Wraps the interface library in a static library
get_target_property(PROPOLIS_OS_INCLUDES SwarmUS::Propolis::OS::FreeRTOS INTERFACE_INCLUDE_DIRECTORIES)
add_library(${LIB_NAME_OS} STATIC ${LIB_HEADERS})
add_library(${LIB_ALIAS_OS} ALIAS ${LIB_NAME_OS})
target_include_directories(${LIB_NAME_OS}
    PUBLIC
        ${PROPOLIS_OS_INCLUDES}
    )

target_link_libraries(${LIB_NAME_OS}
    PUBLIC
        SwarmUS::HiveMind::OS::FreeRTOS
        SwarmUS::Propolis::OS::FreeRTOS
    )
