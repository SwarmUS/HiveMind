set(LIB_NAME "SwarmUS::HiveMind::FreeRTOS")

set(LIB_SOURCES
)

set(LIB_HEADERS
    FreeRTOSConfig.h
)

add_library(${LIB_NAME} ${LIB_SOURCES} ${LIB_HEADERS})
target_include_directories(${LIB_NAME} PUBLIC inc/)
target_include_directories(${LIB_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE 
    src)
