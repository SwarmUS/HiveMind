set(LIB_NAME "swarmus-hivemind-logger")
set(LIB_ALIAS "SwarmUS::HiveMind::logger")

set(LIB_SOURCES
    src/logger.cpp
)

set(LIB_HEADERS
    include/logging/logger.h
)

add_library(${LIB_NAME} STATIC ${LIB_HEADERS})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
    PUBLIC
       $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
       $<INSTALL_INTERFACE:include>
    PRIVATE
       src

)

target_link_libraries(${LIB_NAME}
    PRIVATE
        SwarmUS::HiveMind::BSP
        SwarmUS::HiveMind::FreeRTOS
)

