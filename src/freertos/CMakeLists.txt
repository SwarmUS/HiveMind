set(LIB_NAME "swarmus-hivemind-freertos")
set(LIB_ALIAS "SwarmUS::HiveMind::FreeRTOS")

include(freertos/common)

freertos_fetch_kernel()
add_subdirectory(platforms)

set(LIB_SOURCE
    freertos_hook.cpp
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

)

target_link_libraries(${LIB_NAME}
    PUBLIC
        FreeRTOS::Timers
        FreeRTOS::Heap::1
        SwarmUS::HiveMind::FreeRTOS::Platform
)

