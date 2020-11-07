set(LIB_NAME "swarmus-hivemind-bsp-posix")
set(LIB_ALIAS "SwarmUS::HiveMind::BSP::Platform")

set(LIB_SOURCES
    bsp.cpp
)

set(LIB_PRIVATE_HEADERS

)

add_library(${LIB_NAME} ${LIB_SOURCES} ${LIB_PRIVATE_HEADERS})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})


target_include_directories(${LIB_NAME} 
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        ../../include
)


target_link_libraries(${LIB_NAME}
PUBLIC
        # Defined libraries
        SwarmUS::HiveMind::FreeRTOS
)

